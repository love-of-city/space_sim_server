"""Record a live scripted manipulation task through the production UE/LeRobot pipeline.

Runs an isolated copy of the selected scene, keeping the interactive instance
untouched. No replay, synthetic images, or alternate dataset writer is used.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]

from space_arm_platform.capture_lifecycle import CaptureLifecycle
from space_arm_platform.camera_defaults import (
    CAPTURE_CAMERA_WIDTH,
    CAPTURE_CAMERA_HEIGHT,
)
from space_arm_platform.capture_receiver import CaptureReceiver
from space_arm_platform.models import EpisodeStart, EpisodeStop
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub
from space_arm_platform.sampling import (
    DYNAMICS_HZ,
    DEFAULT_CAPTURE_HZ,
    DEFAULT_IK_HZ,
    WRENCH_HZ,
    SCRIPTED_TASK_HZ,
)

from simulation.scripted_tasks.motion_options import (
    add_motion_options,
    motion_arguments,
)
from simulation.scripted_tasks.plug_profiles import (
    PLUG_PROFILES,
    is_plug_task,
    plug_profile,
)

PROCESS_STARTED = time.monotonic()
PROCESS_STARTED_AT_NS = time.time_ns()

CAMERAS = ["teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam"]


def scene_copy(source, task="drawer_lift"):
    scene = json.loads(source.read_text(encoding="utf-8-sig"))
    if scene.get("template_id") != "sarm-task-box-20261012":
        raise ValueError("Scripted task requires the SARM 2026-10-12 scene")
    runtime = scene.get("runtime", {})
    if scene.get("arm_preparation_required"):
        raise ValueError("Prepare the scene's operating pose before running this task")
    scene["source_instance_id"] = scene["instance_id"]
    scene["instance_id"] = (
        "scripted-" + task.replace("_", "-") + "-" + uuid.uuid4().hex[:12]
    )
    scene["runtime"] = {
        **runtime,
        "dataset_capture": True,
        "dynamics_rate_hz": DYNAMICS_HZ,
        "dynamics_step_s": 1 / DYNAMICS_HZ,
        "wrist_wrench_rate_hz": WRENCH_HZ,
        "scripted_task_rate_hz": SCRIPTED_TASK_HZ,
        "capture_rate_hz": DEFAULT_CAPTURE_HZ,
        "ik_rate_hz": DEFAULT_IK_HZ,
    }
    return scene


def validate_episode(result):
    if result["dataset_status"] != "complete":
        raise RuntimeError(
            f"Dataset finalization failed: {result.get('dataset_error')}"
        )
    frames = result["dataset_frame_count"]
    if (
        frames <= 0
        or result["capture_count"] != len(CAMERAS) * frames
        or result["step_count"] != frames
    ):
        raise RuntimeError("Camera/state frame counts do not match")
    if result["incomplete_sample_count"] or result["rejected_capture_count"]:
        raise RuntimeError("Dataset has incomplete or rejected capture samples")


async def run(args):
    scene = scene_copy(args.scene, args.task)
    project = args.adapter / "Unreal/BskUnrealRenderer"
    editor = args.unreal / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"
    catalog = project / "Saved/AssetImport/sarm_20261012.catalog.json"
    for path in (editor, project / "BskUnrealRenderer.uproject", catalog):
        if not path.is_file():
            raise FileNotFoundError(path)
    ports = [
        args.control_port,
        args.capture_port,
        args.render_port,
        args.signalling_port,
    ]
    if len(set(ports)) != len(ports):
        raise ValueError("Isolated ports must be distinct")
    for port in ports:
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", port))
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    from simulation.scripted_tasks.randomization import prepare_episode, write_model_variant
    prepare_episode(scene)
    write_model_variant(ROOT / "model/SARM/platform/sarm_20261012.xml", out / "model.xml", scene)
    scene_path = out / "scene.json"
    scene_path.write_text(
        json.dumps(scene, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    stop_file = out / "stop.request"
    recorder = EpisodeRecorder(args.episodes_root.resolve())
    hub = SimulationHub()
    lifecycle = CaptureLifecycle(hub, recorder)

    async def observation(obs):
        await asyncio.to_thread(recorder.record_observation, obs, None)
        if hub.capture_pair_ack_supported:
            await asyncio.to_thread(recorder.wait_for_capture_pair, obs)

    hub.on_observation = observation
    hub.on_transport_error = recorder.fail
    receiver = CaptureReceiver(
        "127.0.0.1",
        args.capture_port,
        recorder.record_authoritative_capture,
        recorder.fail,
    )
    processes, logs = [], []
    simulation = None
    report = {
        "source_scene": str(args.scene.resolve()),
        "scene_instance": scene,
        "result": "failed",
        "started_at_ns": str(PROCESS_STARTED_AT_NS),
        "task_name": args.task,
    }

    def emit(stage, **values):
        print(json.dumps({"stage": stage, **values}, ensure_ascii=False), flush=True)

    def launch(command, name, env):
        log = (out / (name + ".log")).open("w", encoding="utf-8")
        logs.append(log)
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        processes.append(process)
        emit("process_started", name=name, pid=process.pid)
        return process

    def check_processes():
        if getattr(args, "cancel_file", None) and args.cancel_file.exists():
            raise RuntimeError("Collection cancelled by operator")
        for process in processes:
            if process.poll() is not None:
                raise RuntimeError(
                    f"Runtime process {process.pid} exited ({process.returncode}); see {out}"
                )
        error = recorder.sync_status().get("dataset_error")
        if error:
            raise RuntimeError(error)

    try:
        await asyncio.to_thread(recorder.prepare)
        await hub.start("127.0.0.1", args.control_port)
        receiver.start()
        emit("writer_ready", output=str(out))
        env = os.environ.copy()
        env["UE-LocalDataCachePath"] = str(project / "Saved/DerivedDataCache")
        env["PYTHONIOENCODING"] = "utf-8"
        command = [
            str(editor),
            str(project / "BskUnrealRenderer.uproject"),
            "-game",
            "-RenderOffscreen",
            "-ForceRes",
            f"-ResX={CAPTURE_CAMERA_WIDTH}",
            f"-ResY={CAPTURE_CAMERA_HEIGHT}",
            "-BskListen=127.0.0.1",
            f"-BskPort={args.render_port}",
            "-BskCaptureHost=127.0.0.1",
            f"-BskCapturePort={args.capture_port}",
            f"-BskCaptureRate={DEFAULT_CAPTURE_HZ}",
            "-BskCaptureProducts=rgb",
            "-ExecCmds=t.MaxFPS 90,r.VSync 0",
            "-DDC=InstalledDerivedDataBackendGraph",
            f"-abslog={out / 'renderer.log'}",
            f"-PixelStreamingConnectionURL=ws://127.0.0.1:{args.signalling_port}",
            "-PixelStreamingID=BskScriptedTask",
            f"-BskPixelStreamingURL=ws://127.0.0.1:{args.signalling_port}",
            "-BskPixelStreamingBaseId=BskScriptedTask",
            "-BskPixelStreamingCameras=" + "+".join(CAMERAS),
            f"-BskPixelStreamingCameraWidth={CAPTURE_CAMERA_WIDTH}",
            f"-BskPixelStreamingCameraHeight={CAPTURE_CAMERA_HEIGHT}",
            "-BskPixelStreamingCameraFps=90",
            "-PixelStreamingWebRTCFps=90",
            "-PixelStreamingUseMediaCapture=false",
            "-PixelStreamingWebRTCDisableTransmitAudio=true",
        ]
        launch(command, "renderer-process", env)
        deadline = time.monotonic() + 240
        while True:
            check_processes()
            try:
                with socket.create_connection(
                    ("127.0.0.1", args.render_port), timeout=0.1
                ):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise TimeoutError("UE renderer did not start within 240 seconds")
                await asyncio.sleep(0.25)
        env["PYTHONPATH"] = os.pathsep.join(
            [str(ROOT), str(ROOT / "backend"), str(args.adapter / "Adapters")]
        )
        command = [
            sys.executable,
            str(ROOT / "tools/run_scripted_task.py"),
            "--task",
            args.task,
            "--target-site",
            args.target_site,
            "--output",
            str(out / "dynamics"),
            "--scene",
            str(scene_path),
            "--capture-control-port",
            str(args.control_port),
            "--capture-render-port",
            str(args.render_port),
            "--catalog",
            str(catalog),
            "--capture-stop-file",
            str(stop_file),
            "--max-sim-seconds",
            str(args.max_sim_seconds),
            "--warmup-sim-seconds",
            str(args.warmup_sim_seconds),
        ]
        command += motion_arguments(args.task, args.linear_speed)
        if args.substeps is not None:
            command += ["--substeps", str(args.substeps)]
        simulation = launch(command, "simulation", env)
        deadline = time.monotonic() + 180
        while hub.latest_observation is None:
            check_processes()
            if time.monotonic() > deadline:
                raise TimeoutError(
                    "No authoritative simulation state within 180 seconds"
                )
            await asyncio.sleep(0.1)
        if not hub.capture_pair_ack_supported:
            raise RuntimeError(
                "Simulation does not support completion-based capture acknowledgements"
            )
        while int(hub.latest_observation.sim_time_ns) < round(
            args.warmup_sim_seconds * 1e9
        ):
            check_processes()
            if time.monotonic() > deadline:
                raise TimeoutError("Simulation pre-roll did not complete")
            await asyncio.sleep(0.1)
        # Asset registration and exposure settle with the policy held at its initial pose.
        emit("warming_renderer")
        warm_until = time.monotonic() + args.warmup_seconds
        while time.monotonic() < warm_until:
            check_processes()
            await asyncio.sleep(0.1)
        if receiver.authoritative_count:
            raise RuntimeError("Idle renderer unexpectedly produced dataset images")
        owner = scene.get("created_by", {})
        info = await lifecycle.start(
            EpisodeStart(
                fps=DEFAULT_CAPTURE_HZ,
                camera_ids=CAMERAS,
                capture_products=["rgb"],
                max_frames=int(args.max_sim_seconds * DEFAULT_CAPTURE_HZ) + 100,
                task="scripted " + args.task,
                instruction=(
                    plug_profile(args.task).instruction
                    if is_plug_task(args.task)
                    else (
                        "夹住操作台最右侧抽屉，沿导轨抽出约8厘米并保持，然后放回归位、松开夹爪并撤离，确认抽屉保持关闭。"
                        if args.task == "drawer_cycle"
                        else "移动至操作台最右侧抽屉上方，转动夹爪、张开、下降夹住抽屉并提起约8厘米。"
                    )
                ),
                operator=owner.get("username", "scripted-task"),
                operator_user_id=owner.get("user_id"),
                operator_username=owner.get("username"),
                operator_role=owner.get("role"),
                seed=scene.get("seed"),
                scene_instance=scene,
                tags=["scripted", args.task, "live-ue-capture"],
            )
        )
        report["episode_id"] = info["episode_id"]
        report["episode_path"] = str(args.episodes_root.resolve() / info["episode_id"])
        (out / "episode.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        emit(
            "recording",
            episode_id=info["episode_id"],
            episode_path=report["episode_path"],
        )
        start_wall = last_report = time.monotonic()
        last_stage = None
        while True:
            check_processes()
            current = hub.latest_observation
            task = (
                (current.model_extra or {}).get("scripted_task", {}) if current else {}
            )
            if task.get("stage") != last_stage or time.monotonic() - last_report >= 10:
                emit("task_progress", task=task, sync=recorder.sync_status())
                last_stage, last_report = task.get("stage"), time.monotonic()
            if task.get("result"):
                report["task"] = task
                break
            if time.monotonic() - start_wall > args.timeout_seconds:
                raise TimeoutError(
                    "Live scripted recording exceeded wall-clock timeout"
                )
            await asyncio.sleep(0.1)
        report["recording_wall_seconds"] = time.monotonic() - start_wall
        emit("finalizing", task=task, sync=recorder.sync_status())
        result = await lifecycle.stop(
            EpisodeStop(
                outcome="success" if task["result"] == "success" else "failure",
                note="Live scripted task using authoritative UE capture. "
                + str(task.get("reason", "")),
            )
        )
        report["finalization_wall_seconds"] = (
            time.monotonic() - start_wall - report["recording_wall_seconds"]
        )
        report["episode"] = result
        validate_episode(result)
        report["result"] = task["result"]
        stop_file.write_text("capture finalized", encoding="utf-8")
        await asyncio.to_thread(simulation.wait, 30)
        if simulation.returncode != (0 if task["result"] == "success" else 1):
            raise RuntimeError("Simulation did not shut down cleanly")
        report["capture_transport"] = receiver.status()
        emit(
            "completed",
            result=report["result"],
            episode_path=report["episode_path"],
            frames=result["dataset_frame_count"],
            captures=result["capture_count"],
        )
        return 0 if report["result"] == "success" else 1
    except BaseException as error:
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if recorder.episode_id:
            try:
                report["episode"] = await lifecycle.stop(
                    EpisodeStop(
                        outcome="aborted", note="Scripted recording interrupted"
                    )
                )
            except Exception as error:
                report["cleanup_error"] = str(error)
        stop_file.write_text("recording stopped", encoding="utf-8")
        for process in reversed(processes):
            if process.poll() is None:
                if process is simulation:
                    try:
                        await asyncio.to_thread(process.wait, 5)
                    except subprocess.TimeoutExpired:
                        pass
                if process.poll() is None:
                    process.terminate()
                    try:
                        await asyncio.to_thread(process.wait, 10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        await asyncio.to_thread(process.wait, 10)
        await asyncio.to_thread(receiver.close)
        await hub.close()
        await asyncio.to_thread(recorder.close)
        for log in logs:
            log.close()
        report["finished_at_ns"] = str(time.time_ns())
        report["end_to_end_wall_seconds"] = time.monotonic() - PROCESS_STARTED
        (out / "summary.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )


def main(default_task="drawer_cycle"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task",
        choices=["drawer_lift", "drawer_cycle", "plug_insert", *PLUG_PROFILES],
        default=default_task,
    )
    parser.add_argument("--target-site", default=None)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument(
        "--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter"
    )
    parser.add_argument(
        "--unreal", type=Path, default=Path("C:/Program Files/Epic Games/UE_5.6")
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument("--episodes-root", type=Path, default=ROOT / "data/episodes")
    parser.add_argument("--control-port", type=int, default=18766)
    parser.add_argument("--capture-port", type=int, default=18767)
    parser.add_argument("--render-port", type=int, default=15558)
    parser.add_argument("--signalling-port", type=int, default=18888)
    parser.add_argument("--warmup-seconds", type=float, default=5.0)
    parser.add_argument("--warmup-sim-seconds", type=float, default=None)
    parser.add_argument("--timeout-seconds", type=float, default=1200.0)
    parser.add_argument("--max-sim-seconds", type=float, default=360.0)
    parser.add_argument("--substeps", type=int)
    parser.add_argument("--cancel-file", type=Path)
    add_motion_options(parser)
    args = parser.parse_args()
    try:
        motion_arguments(args.task, args.linear_speed)
    except ValueError as error:
        parser.error(str(error))
    if args.target_site is None:
        args.target_site = (
            plug_profile(args.task).target_site
            if is_plug_task(args.task)
            else "cylindrical_insert_40p5"
        )
    if args.warmup_sim_seconds is None:
        args.warmup_sim_seconds = 6.466666667 if args.task == "plug_insert" else 0.0
    if args.output is None:
        folder = args.task.replace("_", "-") + "-recording"
        args.output = ROOT / "run" / folder / time.strftime("%Y%m%d-%H%M%S")
    if (
        not 0 <= args.warmup_seconds <= 60
        or not 0 <= args.warmup_sim_seconds <= 60
        or not 0 < args.timeout_seconds <= 7200
        or not 0 < args.max_sim_seconds <= 540
    ):
        parser.error("Invalid warmup, recording timeout or simulation duration")
    return asyncio.run(run(args))


if __name__ == "__main__":
    sys.exit(main())
