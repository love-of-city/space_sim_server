"""Native Linux supervisor for scenes created by the existing platform API."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from types import SimpleNamespace

from .collection_processes import ManagerLease
from .collection_runtime import check_port_available, renderer_command, renderer_environment
from .linux_processes import Children, identity, owned_process, terminate, write_json


def stop_scene(root: Path):
    path = root / "run/scene_runtime.json"
    if not path.exists():
        return
    state = json.loads(path.read_text())
    if state.get("owner_os") != "linux":
        return
    terminate(state.get("launcher_identity"), 60)
    # Recover verified children after a launcher crash, never by name or port.
    for name in ("prepare", "simulation", "renderer"):
        terminate(state.get("children", {}).get(name), 15)
    state = json.loads(path.read_text())
    if state.get("phase") != "failed":
        state.update(phase="stopped", updated_at_ns=str(time.time_ns()))
        write_json(path, state)


def supervise(config_path: Path, instance_path: Path):
    config = json.loads(config_path.read_text())
    root = Path(config["project_root"])
    state_path = root / "run/scene_runtime.json"
    lease = ManagerLease(root / "run/linux-scene.lock")
    instance = json.loads(instance_path.read_text())
    adapter = Path(config["adapter_root"])
    stamp = instance["instance_id"]
    children = Children()
    parent = identity(os.getppid())
    stopping = False
    def request_stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    state = {"owner_os": "linux", "launcher_pid": os.getpid(),
             "launcher_identity": identity(os.getpid()), "instance": instance,
             "instance_path": str(instance_path), "adapter_root": str(adapter),
             "preview_fps": config["preview_rate"], "encoder_min_quality": config["encoder_min_quality"]}
    def update(phase, error=""):
        state.update(phase=phase, error=error, children=children.records,
                     updated_at_ns=str(time.time_ns()))
        for key, name in (("renderer_pid", "renderer"), ("simulation_pid", "simulation")):
            state[key] = children.records.get(name, {}).get("pid", 0)
        write_json(state_path, state)
    def check():
        if stopping or owned_process(parent) is None:
            raise InterruptedError("scene stopped or backend exited")
        children.check()
    def wait_port(port, timeout):
        deadline = time.monotonic() + timeout
        while True:
            check()
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=.2):
                    return
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError(f"UE render port {port} did not become ready")
            time.sleep(.2)
    failure = None
    try:
        update("launching")
        for port in (config["render_port"],):
            check_port_available(port)
        runtime = instance["runtime"]
        prepare = [sys.executable, str(root / "tools/assets/prepare_sarm_assets.py"),
                   "--adapter", str(adapter), "--model-root", config["model_root"],
                   "--unreal", config["unreal_root"], "--template-id", instance["template_id"],
                   "--use-existing-model"]
        preparation = children.start("prepare", prepare, cwd=root,
                                     log=root / f"logs/{stamp}.prepare.log")
        update("launching")
        while preparation.poll() is None:
            if stopping or owned_process(parent) is None:
                raise InterruptedError("scene preparation stopped")
            time.sleep(.2)
        if preparation.returncode:
            raise RuntimeError(f"asset preparation failed; see logs/{stamp}.prepare.log")
        del children.processes["prepare"]
        args = SimpleNamespace(adapter=adapter, unreal=Path(config["unreal_root"]),
                               render_port=config["render_port"], capture_port=config["capture_port"],
                               signalling_port=config["pixel_streamer_port"], offline=False)
        output = root / f"logs/{stamp}"
        output.mkdir(exist_ok=True)
        command = renderer_command(args, output)
        # Replace CLI recording defaults with the scene's existing platform settings.
        keys = ("-PixelStreamingID=", "-BskPixelStreamingBaseId=", "-BskPixelStreamingCameras=",
                "-BskPixelStreamingCameraWidth=", "-BskPixelStreamingCameraHeight=",
                "-BskPixelStreamingCameraFps=", "-PixelStreamingWebRTCFps=", "-BskCaptureRate=",
                "-BskCaptureProducts=", "-ExecCmds=")
        command = [value for value in command if not value.startswith(keys)]
        command += [f"-PixelStreamingID={config['pixel_streaming_id']}",
                    f"-BskPixelStreamingBaseId={config['pixel_streaming_id']}",
                    "-BskPixelStreamingCameras=" + "+".join(config["pixel_streaming_camera_ids"]),
                    f"-BskPixelStreamingCameraWidth={config['pixel_streaming_camera_width']}",
                    f"-BskPixelStreamingCameraHeight={config['pixel_streaming_camera_height']}",
                    f"-BskPixelStreamingCameraFps={config['preview_rate']}",
                    f"-PixelStreamingWebRTCFps={config['preview_rate']}",
                    f"-BskPreviewRate={config['preview_rate']}",
                    f"-PixelStreamingEncoderMinQuality={config['encoder_min_quality']}",
                    "-PixelStreamingEncoderCodec=H264", "-PixelStreamingDecoupleFramerate=false",
                    f"-ExecCmds=t.MaxFPS {config['preview_rate']},r.VSync 0"]
        if runtime["dataset_capture"]:
            command += ["-BskCaptureProducts=rgb", f"-BskCaptureRate={runtime['capture_rate_hz']}"]
        children.start("renderer", command, cwd=root, env=renderer_environment(args),
                       log=output / "renderer-process.log")
        update("starting_renderer")
        wait_port(config["render_port"], config["renderer_ready_timeout"])
        catalog_names = {"sarm-task-box-contacts": "sarm_task_box", "sarm-task-box-free-plugs": "sarm_task_box_plugs"}
        template = instance["template_id"]
        catalog = "sarm_20261012" if template.startswith("sarm-task-box-20261012") else catalog_names.get(template, "sarm_platform")
        children.start("simulation", [sys.executable, "-u", str(root / "simulation/teleop_grasp_unreal.py"),
            "--adapter-root", str(adapter), "--model-root", config["model_root"],
            "--catalog", str(adapter / f"Unreal/BskUnrealRenderer/Saved/AssetImport/{catalog}.catalog.json"),
            "--control-port", str(config["control_port"]), "--render-port", str(config["render_port"]),
            "--scene-instance", str(instance_path), "--dynamics-backend", "local",
            "--simulation-rate", str(runtime["simulation_rate"]),
            "--capture-rate", str(runtime["capture_rate_hz"]), "--ik-rate", str(runtime["ik_rate_hz"])],
            cwd=root, log=output / "simulation.log")
        update("running")
        while True:
            check()
            time.sleep(.3)
    except InterruptedError:
        update("stopping")
    except BaseException as error:
        failure = error
        update("failed", str(error))
    finally:
        try:
            children.close(("prepare", "simulation", "renderer"))
        except RuntimeError as error:
            failure = error
        update("failed" if failure else "stopped", str(failure) if failure else "")
        lease.close()
    if failure:
        raise RuntimeError(str(failure))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop"))
    parser.add_argument("--config", type=Path)
    parser.add_argument("--instance", type=Path)
    parser.add_argument("--project-root", type=Path)
    args = parser.parse_args()
    if args.action == "stop":
        stop_scene(args.project_root.resolve())
    else:
        supervise(args.config.resolve(), args.instance.resolve())


if __name__ == "__main__":
    main()
