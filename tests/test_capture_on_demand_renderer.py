"""Opt-in real UE/RGB smoke test; isolated ports, synthetic scene, no user data."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("SPACE_SIM_UE_CAPTURE_SMOKE") != "1",
    reason="set SPACE_SIM_UE_CAPTURE_SMOKE=1 to launch an isolated offscreen UE renderer")


def test_real_ue_only_emits_rgb_between_tagged_start_and_stop(tmp_path):
    from space_arm_platform.capture_receiver import CaptureReceiver
    from PIL import Image
    import io

    root = Path(__file__).resolve().parents[1]
    adapter = Path(os.environ.get("SPACE_SIM_RESET_ADAPTER", str(root.parents[1]/"space_sim_UE_adapter/space_sim_UE_Adapter")))
    sys.path.insert(0, str(adapter/"Adapters"))
    from Basilisk.architecture import messaging
    from bsk_render_adapter import BasiliskRenderBridge, CameraVisual, GeometryVisual

    unreal_root = os.environ.get("UE56_ROOT")
    if not unreal_root:
        pytest.skip("set UE56_ROOT to the UE 5.6 installation for this opt-in smoke test")
    editor = Path(unreal_root)/"Engine/Binaries/Win64/UnrealEditor-Cmd.exe"
    if not editor.is_file(): pytest.skip("UE 5.6 renderer not installed at UE56_ROOT")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        render_port = reservation.getsockname()[1]
    received, errors = [], []
    captures = CaptureReceiver("127.0.0.1", 0, lambda m, p: received.append((m, p)), errors.append)
    captures.start()
    process = bridge = None
    try:
        deadline = time.monotonic()+5
        while captures._listener is None or not captures._listener.getsockname()[1]:
            assert time.monotonic() < deadline
            time.sleep(.01)
        capture_port = captures._listener.getsockname()[1]
        args = [str(editor), str(adapter/"Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject"),
            "-game", "-unattended", "-nop4", "-nosplash", "-RenderOffscreen", "-ForceRes", "-ResX=320", "-ResY=180",
            "-BskListen=127.0.0.1", f"-BskPort={render_port}", "-BskCaptureProducts=rgb", "-BskCaptureRate=30",
            "-BskCaptureHost=127.0.0.1", f"-BskCapturePort={capture_port}", "-ExecCmds=t.MaxFPS 60,r.VSync 0",
            f"-AbsLog={tmp_path/'ue.log'}"]
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
        process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW)
        deadline = time.monotonic()+60
        while True:
            assert process.poll() is None, (tmp_path/"ue.log").read_text(encoding="utf-8", errors="replace")
            try:
                with socket.create_connection(("127.0.0.1", render_port), timeout=.2): break
            except OSError:
                assert time.monotonic() < deadline, "UE did not open its isolated render port"
                time.sleep(.1)
        mode = {"capture_episode_id": "", "capture_request_id": "idle"}
        bridge = BasiliskRenderBridge(port=render_port, origin_object="smoke/body", capture_state_provider=lambda: dict(mode))
        message = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
        bridge.add_object("smoke/body", message,
            geometries=[GeometryVisual("smoke/box", "box", (.3, .3, .3))])
        for camera in ("overview", "wrist"):
            bridge.add_camera(CameraVisual(camera_id=camera, parent_id="smoke/body", position_body_m=(0, 0, 2),
                resolution=(160, 90), capture_rate_hz=30, capture_products=("rgb",)))
        bridge.Reset(0)
        frame = 0
        def publish(count):
            nonlocal frame
            for _ in range(count):
                bridge.UpdateState((frame * 1_000_000_000 + 15)//30)
                frame += 1
                time.sleep(.04)
        publish(30)
        time.sleep(.5)
        assert not received, "capture-capable idle scene unexpectedly produced dataset RGB"
        expected = {}
        for episode, count in (("episode-A", 6), ("episode-B", 3)):
            mode.update(capture_episode_id=episode, capture_request_id=episode)
            expected[episode] = set(range(frame, frame+count))
            publish(count)
            mode.update(capture_episode_id="", capture_request_id="stop")
            publish(15)
            deadline = time.monotonic()+15
            target = sum(len(frames)*2 for frames in expected.values())
            while len(received) < target:
                assert process.poll() is None
                assert time.monotonic() < deadline, (len(received), target, errors, captures.last_error)
                time.sleep(.02)
            publish(15)
            time.sleep(.2)
            assert len(received) == target, "STOP kept producing new strict images"
        assert not errors
        for episode, frames in expected.items():
            for camera in ("overview", "wrist"):
                rows = [m for m, _ in received if m["capture_episode_id"] == episode and m["camera_id"] == camera]
                assert [int(m["source_frame_id"]) for m in rows] == sorted(frames)
        for metadata, products in received:
            assert metadata["state_kind"] == "authoritative"
            assert Image.open(io.BytesIO(products["rgb"])).size == (160, 90)
        print("UE on-demand smoke: idle=0 images; episode-A=12; stopped=0 new; episode-B=6; stopped=0 new")
    finally:
        if bridge: bridge.close()
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(15)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(10)
        captures.close()
