"""Isolated offline renderer ownership for CLI recording workers."""
from pathlib import Path
import os
import subprocess
import socket

from .camera_defaults import CAPTURE_CAMERA_WIDTH, CAPTURE_CAMERA_HEIGHT
from .recording_config import DEFAULT_CAMERA_IDS
from .sampling import DEFAULT_CAPTURE_HZ
from .unreal_runtime import unreal_editor


def check_port_available(port):
    with socket.socket() as sock:
        # Linux listeners support rebinding after a completed episode's TIME_WAIT.
        # Do not enable Windows SO_REUSEADDR, which can steal a live listener.
        if os.name != "nt":
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", port))


def renderer_command(args, output: Path):
    project = args.adapter / "Unreal/BskUnrealRenderer"
    command = [
        str(unreal_editor(args.unreal)), str(project / "BskUnrealRenderer.uproject"),
        "-game", "-RenderOffscreen", "-ForceRes", "-unattended", "-nosplash",
        f"-ResX={CAPTURE_CAMERA_WIDTH}", f"-ResY={CAPTURE_CAMERA_HEIGHT}",
        "-BskListen=127.0.0.1", f"-BskPort={args.render_port}",
        "-BskCaptureHost=127.0.0.1", f"-BskCapturePort={args.capture_port}",
        f"-BskCaptureRate={DEFAULT_CAPTURE_HZ}", "-BskCaptureProducts=rgb",
        "-ExecCmds=t.MaxFPS 90,r.VSync 0", "-DDC=InstalledDerivedDataBackendGraph",
        f"-abslog={output / 'renderer.log'}",
    ]
    gpu = getattr(args, "gpu_index", None)
    if gpu is not None:
        command.append(f"-graphicsadapter={gpu}")
    if getattr(args, "offline", False):
        command += ["-BskOfflineCapture", "-BskPreviewRate=0"]
    else:
        command += [
            f"-PixelStreamingConnectionURL=ws://127.0.0.1:{args.signalling_port}",
            "-PixelStreamingID=BskScriptedTask",
            f"-BskPixelStreamingURL=ws://127.0.0.1:{args.signalling_port}",
            "-BskPixelStreamingBaseId=BskScriptedTask",
            "-BskPixelStreamingCameras=" + "+".join(DEFAULT_CAMERA_IDS),
            f"-BskPixelStreamingCameraWidth={CAPTURE_CAMERA_WIDTH}",
            f"-BskPixelStreamingCameraHeight={CAPTURE_CAMERA_HEIGHT}",
            "-BskPixelStreamingCameraFps=90", "-PixelStreamingWebRTCFps=90",
            "-PixelStreamingUseMediaCapture=false", "-PixelStreamingWebRTCDisableTransmitAudio=true",
        ]
    return command


def renderer_environment(args):
    env = os.environ.copy()
    env["UE-LocalDataCachePath"] = str(args.adapter / "Unreal/BskUnrealRenderer/Saved/DerivedDataCache")
    env["PYTHONIOENCODING"] = "utf-8"
    return env


class PersistentRenderer:
    """One renderer per batch worker; dynamics still starts fresh each episode."""
    def __init__(self, args, output):
        self.process = None
        self.log = (output / "renderer-process.log").open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(renderer_command(args, output),
                env=renderer_environment(args), stdout=self.log, stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except BaseException:
            self.log.close()
            raise

    def check(self):
        if self.process.poll() is not None:
            raise RuntimeError(f"Persistent UE exited with code {self.process.returncode}")

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(15)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(10)
        finally:
            self.log.close()
