"""Record a live scripted manipulation task through the production UE/LeRobot pipeline.

Runs an isolated copy of the selected scene, keeping the interactive instance
untouched. No replay, synthetic images, or alternate dataset writer is used.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
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
from space_arm_platform.collection_runtime import renderer_command, renderer_environment, check_port_available
from space_arm_platform.camera_defaults import (
    CAPTURE_CAMERA_WIDTH,
    CAPTURE_CAMERA_HEIGHT,
)
from space_arm_platform.capture_receiver import CaptureReceiver
from space_arm_platform.models import EpisodeStart, EpisodeStop
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.recording_config import load_recording_config
from space_arm_platform.unreal_runtime import default_unreal_root, unreal_editor
from space_arm_platform.simulation_hub import SimulationHub
from space_arm_platform.sampling import (
    DYNAMICS_HZ,
    DEFAULT_CAPTURE_HZ,
    DEFAULT_IK_HZ,
    WRENCH_HZ,
    SCRIPTED_TASK_HZ,
    capture_preroll_ns,
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
from simulation.scripted_tasks.threaded_tasks import (
    THREADED_INSTRUCTIONS,
    THREADED_TASKS,
    task_default_duration_s,
)

PROCESS_STARTED = time.monotonic()
PROCESS_STARTED_AT_NS = time.time_ns()

CAMERAS = ["teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam"]


class ScriptedObservationRecorder:
    """Confirm task completion only after the terminal sample reaches the writer."""

    def __init__(self, hub, recorder):
        self.hub = hub
        self.recorder = recorder
        self.terminal_task = None

    async def __call__(self, observation):
        await asyncio.to_thread(self.recorder.record_observation, observation, None)
        if self.hub.capture_pair_ack_supported:
            await asyncio.to_thread(self.recorder.wait_for_capture_pair, observation)
        task = (observation.model_extra or {}).get("scripted_task", {})
        if not task.get("result") or not observation.capture_episode_id:
            return
        status = self.recorder.sync_status()
        if (self.recorder.episode_id == observation.capture_episode_id
                and not status["dataset_error"] and not status["capture_boundary_frozen"]
                and int(status["last_written_frame_id"]) >= int(observation.render_frame_id)):
            self.terminal_task = dict(task)
            # Close the accepted interval at this paired terminal frame. Waiting
            # for the main loop's next poll can append another in-flight frame
            # after success, with a different physical state but the latched
            # success flag (e.g. a released plug oscillating above tolerance).
            # RGB pairing and writer append have already completed; OFF/drain
            # still use the normal lifecycle and no interior frames are skipped.
            await asyncio.to_thread(self.recorder.freeze_observations)


async def wait_for_simulation_startup(hub, preroll_ns, timeout, check_processes, emit):
    """Give native initialization and observable pre-roll separate full budgets."""
    deadline = time.monotonic() + timeout
    while hub.latest_observation is None:
        check_processes()
        if time.monotonic() > deadline:
            raise TimeoutError(f"No authoritative simulation state within {timeout:g} seconds")
        await asyncio.sleep(0.1)
    if not hub.capture_pair_ack_supported:
        raise RuntimeError("Simulation does not support completion-based capture acknowledgements")
    emit("first_observation")
    deadline = time.monotonic() + timeout
    while int(hub.latest_observation.sim_time_ns) < preroll_ns:
        check_processes()
        if time.monotonic() > deadline:
            raise TimeoutError("Simulation pre-roll did not complete")
        await asyncio.sleep(0.1)


def submit_dynamics(command, mailbox, stop_file, launch, env):
    """Use identical episode arguments for a child or an isolated worker request."""
    if mailbox is None:
        return launch(command, "simulation", env)
    from space_arm_platform.dynamics_worker import EpisodeProcess
    entrypoint = str(ROOT / "tools/run_scripted_task.py")
    if command[1] != entrypoint:
        raise ValueError("Dynamics profiling requires a dedicated process")
    return EpisodeProcess(mailbox, command[2:], stop_file)


def recording_settings(path=None):
    settings = load_recording_config(path or ROOT / "config/recording.json")
    # This task's transport and acceptance contract require paired 50 Hz RGB.
    if (settings.capture_rate_hz != DEFAULT_CAPTURE_HZ
            or set(settings.camera_ids) != set(CAMERAS)
            or settings.capture_products != ["rgb"]):
        raise ValueError("Scripted collection requires 50 Hz and both RGB cameras in config/recording.json")
    return settings


def task_recording_settings(settings, task_name, max_sim_seconds):
    """Size this isolated threaded recording without editing global settings."""
    if task_name not in THREADED_TASKS:
        return settings, None
    required = math.ceil(max_sim_seconds * DEFAULT_CAPTURE_HZ) + 1
    if settings.max_frames >= required:
        return settings, None
    original = settings.max_frames
    settings = type(settings).model_validate({**settings.model_dump(), "max_frames": required})
    return settings, dict(
        reason="threaded_task_duration", original_max_frames=original,
        effective_max_frames=required, max_sim_seconds=max_sim_seconds,
        capture_rate_hz=DEFAULT_CAPTURE_HZ,
    )


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


def task_target_site(task, requested=None):
    """Resolve the physical target used by both the controller and metadata."""
    if is_plug_task(task):
        profile = plug_profile(task)
        target = requested or profile.target_site
        if target not in (profile.target_site, profile.prefix + "_insert_40p1"):
            raise ValueError("Target must be this plug's corresponding assembly hole")
        return target
    if task == "aviation_bolt_insert":
        target = requested or "bolt_assembly_hole"
        if target != "bolt_assembly_hole":
            raise ValueError("Aviation bolt task requires bolt_assembly_hole")
        return target
    return requested


def task_instruction(scene, task, target_site):
    """Return the instruction realized by this episode's appearance config."""
    colour = (scene.get("collection_config") or {}).get("color_randomization") or {}
    prompt = colour.get("prompt_instruction")
    if prompt is not None:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("color_randomization.prompt_instruction must be nonempty text")
        return prompt
    if is_plug_task(task):
        profile = plug_profile(task)
        object_name = {
            "aviation_plug": "aviation plug",
            "guide_plug": "guide plug",
            "cylindrical_plug": "cylindrical plug",
        }[profile.body]
        return f"Pick up the {object_name}, align it with the corresponding socket, and insert it fully."
    if task in THREADED_INSTRUCTIONS:
        return THREADED_INSTRUCTIONS[task]
    drawer_location = (
        "drawer" if scene.get("task_box_layout", {}).get("schema")
        in ("task-box-layout/2", "task-box-layout/3") else "rightmost drawer"
    )
    if task == "drawer_cycle":
        return (
            f"Open the {drawer_location} approximately 8 cm, then close it fully and release it."
        )
    return (
        f"Move above the {drawer_location} on the workbench, orient and open the gripper, "
        "then lower it to grasp the drawer and lift it approximately 8 cm."
    )


def task_id(task, target_site):
    """Stable metadata key shared by the scene and EpisodeStart request."""
    return task + (":" + target_site if target_site is not None else "")


def validate_color_instruction(scene):
    """Reject colour-labelled data unless that assignment reached the model."""
    cfg = scene.get("collection_config") or {}
    colour = cfg.get("color_randomization") or {}
    if colour.get("prompt_instruction") is None:
        return
    assignment = colour.get("assignment_sha256")
    visual = cfg.get("model_visual_randomization") or {}
    if (not assignment or not colour.get("applied_schema")
            or colour.get("appearance_sha256") != assignment
            or visual.get("assignment_sha256") != assignment):
        raise ValueError("Colour instruction does not match the applied model appearance")


def task_metadata(scene, task, target_site, instruction):
    """Persist the task/target/prompt identity in the scene snapshot."""
    cfg = scene.setdefault("collection_config", {})
    result = {
        "schema": "scripted-task-metadata/1",
        "task_id": task_id(task, target_site),
        "task": task,
        "target_site": target_site,
        "instruction": instruction,
    }
    colour = cfg.get("color_randomization")
    if colour is not None:
        target_colors = colour.get("hole_colors", {})
        source_colors = colour.get("source_colors", {})
        palette = colour.get("palette", {})
        source = (
            plug_profile(task).body if is_plug_task(task)
            else "bolt" if task == "aviation_bolt_insert" else None
        )
        result["color_randomization"] = {
            "schema": colour.get("schema"),
            "seed": colour.get("seed"),
            "hole_stream": colour.get("hole_stream"),
            "source_stream": colour.get("source_stream"),
            "target_site": target_site,
            "target_color_key": target_colors.get(target_site),
            "target_color": palette.get(target_colors.get(target_site)),
            "source_body": source,
            "source_color_key": source_colors.get(source) if source else None,
            "source_color": palette.get(source_colors.get(source)) if source else None,
            "assignment_sha256": colour.get("assignment_sha256"),
            "appearance_sha256": colour.get("appearance_sha256"),
            "prompt_instruction": colour.get("prompt_instruction", instruction),
        }
    cfg["scripted_task"] = result
    cfg["task_id"] = result["task_id"]
    cfg["task_instruction"] = instruction
    # These top-level aliases make old episode readers able to locate the same
    # identity without parsing the nested collection configuration.
    scene["scripted_task_name"] = task
    scene["scripted_task_id"] = result["task_id"]
    scene["scripted_task_target_site"] = target_site
    scene["scripted_task_instruction"] = instruction
    return result


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
    run_started = time.monotonic()
    scene = scene_copy(args.scene, args.task)
    saved_layout = scene.get("collection_config", {}).get("layout_randomization", {})
    if (scene.get("collection_layout_randomization") or scene.get("task_box_layout")
            or saved_layout.get("enabled")) and getattr(args, "model", None) is None:
        raise ValueError("--model is required for a saved layout; pass its matching --catalog")
    settings = recording_settings(getattr(args, "recording_config", None))
    settings, frame_limit_override = task_recording_settings(
        settings, args.task, getattr(args, "max_sim_seconds", task_default_duration_s(args.task)))
    if getattr(args, "gpu_index", None) is not None:
        settings.video = type(settings.video).model_validate(
            {**settings.video.model_dump(), "gpu_index": args.gpu_index})
    args.target_site = task_target_site(args.task, getattr(args, "target_site", None))
    scene["scripted_task_name"] = args.task
    collection_config = scene.setdefault("collection_config", {})
    collection_config["task"] = args.task
    collection_config["target_site"] = args.target_site
    if args.task in THREADED_TASKS:
        collection_config["threaded_speed_profile"] = args.threaded_speed_profile
    if frame_limit_override:
        collection_config["recording_frame_limit_override"] = frame_limit_override
    from space_arm_platform.overview_camera import validate_overview_camera
    validate_overview_camera(scene)
    project = args.adapter / "Unreal/BskUnrealRenderer"
    editor = unreal_editor(args.unreal)
    catalog = (getattr(args, "catalog", None) or project / "Saved/AssetImport/sarm_20261012.catalog.json").resolve()
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
        if getattr(args, "external_renderer", False) and port == args.render_port:
            continue
        check_port_available(port)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    from simulation.scripted_tasks.randomization import prepare_episode, write_model_variant
    prepare_episode(scene)
    source_model = (getattr(args, "model", None) or ROOT / "model/SARM/platform/sarm_20261012.xml").resolve()
    if getattr(args, "model", None) is not None:
        from simulation.task_box_layout import validated_capture_layout
        placement = validated_capture_layout(source_model, catalog)
        scene["task_box_layout"] = placement
        selected = scene.get("collection_config", {}).get("layout_randomization")
        if selected and selected.get("enabled"):
            from space_arm_platform.collection_layouts import digest
            if (digest(source_model) != selected["model_sha256"]
                    or digest(catalog) != selected["catalog_sha256"]):
                raise ValueError("Selected collection layout model/catalog changed before recording")
    write_model_variant(source_model, out / "model.xml", scene)
    validate_color_instruction(scene)
    instruction = task_instruction(scene, args.task, args.target_site)
    scripted_metadata = task_metadata(scene, args.task, args.target_site, instruction)
    scene_path = out / "scene.json"
    scene_path.write_text(
        json.dumps(scene, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    stop_file = out / "stop.request"
    recorder = EpisodeRecorder(
        args.episodes_root.resolve(),
        save_raw_camera_images=settings.save_raw_camera_images,
        video_encoding=settings.video,
        storage_profile=settings.storage_profile,
    )
    hub = SimulationHub()
    lifecycle = CaptureLifecycle(hub, recorder)

    observations = ScriptedObservationRecorder(hub, recorder)
    hub.on_observation = observations
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
        "recording_settings": settings.model_dump(mode="json"),
        "source_scene": str(args.scene.resolve()),
        "scene_instance": scene,
        "result": "failed",
        "started_at_ns": str(PROCESS_STARTED_AT_NS),
        "task_name": args.task,
        "target_site": args.target_site,
        "instruction": instruction,
        "scripted_task_metadata": scripted_metadata,
        "recording_frame_limit_override": frame_limit_override,
    }

    def emit(stage, **values):
        elapsed = time.monotonic() - run_started
        report.setdefault("stage_seconds", {})[stage] = elapsed
        print(json.dumps({"stage": stage, "elapsed_seconds": elapsed, **values}, ensure_ascii=False), flush=True)

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
        env = renderer_environment(args)
        if not getattr(args, "external_renderer", False):
            launch(renderer_command(args, out), "renderer-process", env)
        emit("renderer_launched", reused=getattr(args, "external_renderer", False))
        env["PYTHONPATH"] = os.pathsep.join(
            [str(ROOT), str(ROOT / "backend"), str(args.adapter / "Adapters")]
        )
        command = [
            sys.executable,
            str(ROOT / "tools/run_scripted_task.py"),
            "--task",
            args.task,
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
            "--capture-backpressure-poll-s",
            str(settings.capture_backpressure_poll_s),
        ]
        if args.target_site is not None:
            command += ["--target-site", args.target_site]
        if getattr(args, "model", None) is not None:
            command += ["--model", str(source_model)]
        if getattr(args, "profile_dynamics", False):
            command[1:1] = ["-m", "cProfile", "-o", str(out / "dynamics.pstats")]
        if not settings.save_dynamics_trace:
            command.append("--no-save-trace")
        if settings.scripted_native_geometry:
            command.append("--native-geometry")
        command += motion_arguments(args.task, args.linear_speed, args.threaded_speed_profile)
        if args.substeps is not None:
            command += ["--substeps", str(args.substeps)]
        mailbox = getattr(args, "dynamics_worker", None)
        simulation = submit_dynamics(command, mailbox, stop_file, launch, env)
        if mailbox is not None:
            processes.append(simulation)
            report["dynamics_worker"] = dict(
                directory=str(mailbox), pid=simulation.pid,
                request_id=simulation.request_id, result=None,
            )
            emit("process_started", name="simulation", pid=simulation.pid, persistent=True)
        # Dynamics initialization and UE startup are independent. The render
        # publisher already reconnects and retains the manifest/latest idle
        # frame; require both peers ready before the capture boundary below.
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
        startup_timeout = getattr(args, "simulation_startup_timeout_seconds", 600.0)
        emit("renderer_listening")
        await wait_for_simulation_startup(
            hub, capture_preroll_ns(args.warmup_sim_seconds), startup_timeout, check_processes, emit)
        # Asset registration and exposure settle with the policy held at its initial pose.
        emit("warming_renderer")
        warm_until = time.monotonic() + args.warmup_seconds
        while time.monotonic() < warm_until:
            check_processes()
            await asyncio.sleep(0.1)
        if receiver.authoritative_count:
            raise RuntimeError("Idle renderer unexpectedly produced dataset images")
        owner = scene.get("created_by", {})
        episode_task_id = task_id(args.task, args.target_site)
        info = await lifecycle.start(
            EpisodeStart(
                fps=DEFAULT_CAPTURE_HZ,
                camera_ids=CAMERAS,
                capture_products=["rgb"],
                max_frames=settings.max_frames,
                task="scripted " + args.task,
                task_id=episode_task_id,
                instruction=instruction,
                operator=owner.get("username", "scripted-task"),
                operator_user_id=owner.get("user_id"),
                operator_username=owner.get("username"),
                operator_role=owner.get("role"),
                seed=scene.get("seed"),
                scene_instance=scene,
                tags=["scripted", args.task, "live-ue-capture"] + (
                    ["color-randomized"]
                    if scene.get("collection_config", {}).get("color_randomization")
                    else []
                ),
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
        report["startup_wall_seconds"] = start_wall - run_started
        last_stage = None
        while True:
            check_processes()
            current = hub.latest_observation
            task = (
                (current.model_extra or {}).get("scripted_task", {}) if current else {}
            )
            if task.get("stage") != last_stage or time.monotonic() - last_report >= 10:
                emit("task_progress", task=task, sync=recorder.sync_status(),
                     capture_transport=receiver.status())
                last_stage, last_report = task.get("stage"), time.monotonic()
            if observations.terminal_task is not None:
                task = observations.terminal_task
                report["task"] = task
                break
            if time.monotonic() - start_wall > args.timeout_seconds:
                raise TimeoutError(
                    "Live scripted recording exceeded wall-clock timeout"
                )
            await asyncio.sleep(0.1)
        report["recording_wall_seconds"] = time.monotonic() - start_wall
        emit("finalizing", task=task, sync=recorder.sync_status())
        finalizing_started = time.monotonic()
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
        report["finalization_wall_seconds"] = time.monotonic() - finalizing_started
        report["capture_and_finalize_wall_seconds"] = time.monotonic() - start_wall
        validate_episode(result)
        report["recorded_sim_seconds"] = (result["dataset_frame_count"] - 1) / DEFAULT_CAPTURE_HZ
        report["recording_realtime_factor"] = report["recorded_sim_seconds"] / report["recording_wall_seconds"]
        report["capture_and_finalize_realtime_factor"] = report["recorded_sim_seconds"] / report["capture_and_finalize_wall_seconds"]
        report["result"] = task["result"]
        stop_file.write_text("capture finalized", encoding="utf-8")
        await asyncio.to_thread(simulation.wait, 30)
        # Python's cProfile CLI consumes the target's SystemExit, returning 0
        # even for a deliberately bounded failed task segment.
        expected_exit = 0 if task["result"] == "success" or getattr(args, "profile_dynamics", False) else 1
        if simulation.returncode != expected_exit:
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
        # Preserve the last render/readback/transport counters on failed runs too.
        report["capture_transport"] = receiver.status()
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
        if report.get("dynamics_worker") is not None and simulation is not None:
            report["dynamics_worker"]["result"] = simulation.result
        report["total_wall_seconds"] = time.monotonic() - run_started
        if "recorded_sim_seconds" in report:
            report["total_realtime_factor"] = report["recorded_sim_seconds"] / report["total_wall_seconds"]
        report["finished_at_ns"] = str(time.time_ns())
        report["end_to_end_wall_seconds"] = time.monotonic() - PROCESS_STARTED
        (out / "summary.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )


def main(default_task="drawer_cycle"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task",
        choices=["drawer_lift", "drawer_cycle", "plug_insert", *PLUG_PROFILES, *THREADED_TASKS],
        default=default_task,
    )
    parser.add_argument("--target-site", default=None)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--model", type=Path, help="Explicit generated placement model; recorded with its layout manifest")
    parser.add_argument("--catalog", type=Path, help="Asset catalog for a generated model with new geometry")
    parser.add_argument(
        "--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter"
    )
    parser.add_argument(
        "--unreal", type=Path, default=default_unreal_root()
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument("--episodes-root", type=Path, default=ROOT / "data/episodes")
    parser.add_argument("--recording-config", type=Path, help="Override config/recording.json for an isolated run")
    parser.add_argument("--offline", action="store_true", help="Disable WebRTC and the unused offscreen viewport")
    parser.add_argument("--gpu-index", type=int)
    parser.add_argument("--external-renderer", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dynamics-worker", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--profile-dynamics", action="store_true", help="Diagnostic CPU profile; adds timing overhead")
    parser.add_argument("--control-port", type=int, default=18766)
    parser.add_argument("--capture-port", type=int, default=18767)
    parser.add_argument("--render-port", type=int, default=15558)
    parser.add_argument("--signalling-port", type=int, default=18888)
    parser.add_argument("--warmup-seconds", type=float, default=5.0)
    parser.add_argument("--warmup-sim-seconds", type=float, default=None)
    parser.add_argument("--simulation-startup-timeout-seconds", type=float, default=600.0)
    parser.add_argument("--timeout-seconds", type=float, default=None)
    parser.add_argument("--max-sim-seconds", type=float, default=None)
    parser.add_argument("--substeps", type=int)
    parser.add_argument("--cancel-file", type=Path)
    add_motion_options(parser)
    args = parser.parse_args()
    if (not math.isfinite(args.simulation_startup_timeout_seconds)
            or not 0 < args.simulation_startup_timeout_seconds <= 7200):
        parser.error("simulation-startup-timeout-seconds must be finite and in (0,7200]")
    if args.dynamics_worker is not None:
        args.dynamics_worker = args.dynamics_worker.expanduser().resolve()
        if args.profile_dynamics:
            parser.error("--profile-dynamics cannot use a persistent dynamics worker")
    if args.max_sim_seconds is None:
        args.max_sim_seconds = task_default_duration_s(args.task)
    if args.timeout_seconds is None:
        args.timeout_seconds = 5400.0 if args.task in THREADED_TASKS else 1200.0
    try:
        motion_arguments(args.task, args.linear_speed, args.threaded_speed_profile)
    except ValueError as error:
        parser.error(str(error))
    if args.target_site is None:
        args.target_site = task_target_site(args.task)
    if args.warmup_sim_seconds is None:
        args.warmup_sim_seconds = 6.466666667 if args.task == "plug_insert" else 0.0
    if args.output is None:
        folder = args.task.replace("_", "-") + "-recording"
        args.output = ROOT / "run" / folder / time.strftime("%Y%m%d-%H%M%S")
    if (
        not 0 <= args.warmup_seconds <= 60
        or not 0 <= args.warmup_sim_seconds <= 60
        or not 0 < args.timeout_seconds <= 7200
        or not 0 < args.max_sim_seconds <= (1800 if args.task in THREADED_TASKS else 540)
    ):
        parser.error("Invalid warmup, recording timeout or simulation duration")
    return asyncio.run(run(args))


if __name__ == "__main__":
    sys.exit(main())
