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
    import numpy as np
    import uuid

    root = Path(__file__).resolve().parents[1]
    adapter = Path(os.environ.get("SPACE_SIM_RESET_ADAPTER", str(root.parent/"space_sim_UE_Adapter")))
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
        if os.environ.get('SPACE_SIM_SYNCHRONOUS_CAPTURE') == '1':
            args.append('-BskSynchronousCapture')
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
        # An empty geometry list intentionally creates a large UE placeholder;
        # use a tiny explicit origin marker so it cannot occlude the test box.
        bridge.add_object("smoke/body", message,
            geometries=[GeometryVisual('smoke/origin', 'box', (.001, .001, .001))])
        marker = messaging.SCStatesMsg()
        bridge.add_object('smoke/marker', marker,
            geometries=[GeometryVisual('smoke/box', 'box', (.3, .3, .3),
                color_rgba=(1., 0., 0., 1.), material_emission=1.)])
        for camera in ("overview", "wrist"):
            bridge.add_camera(CameraVisual(camera_id=camera, parent_id="smoke/body", position_body_m=(-2, 0, 0),
                resolution=(160, 90), capture_rate_hz=30, capture_products=("rgb",)))
        bridge.Reset(0)
        frame = 0
        def publish(count, delay=.04):
            nonlocal frame
            for _ in range(count):
                pose = messaging.SCStatesMsgPayload()
                pose.r_BN_N = [0., .45 if frame % 2 == 0 else -.45, .25]
                marker.write(pose)
                bridge.UpdateState((frame * 1_000_000_000 + 15)//30)
                frame += 1
                time.sleep(delay)
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
        # Reset with GPU/codec jobs still in flight, then prove the new session
        # can capture without inheriting the cancelled episode's images.
        mode.update(capture_episode_id='aborted-before-reset', capture_request_id='abort')
        publish(8, delay=.001)
        time.sleep(.04)
        bridge.session_id = uuid.uuid4().hex
        mode.update(capture_episode_id='', capture_request_id='reset-idle')
        bridge.Reset(0)
        frame = 0
        publish(20)
        mode.update(capture_episode_id='episode-after-reset', capture_request_id='new')
        frames = set(range(frame, frame + 6))
        publish(6)
        mode.update(capture_episode_id='', capture_request_id='stop-new')
        publish(15)
        deadline = time.monotonic() + 15
        while sum(m['capture_episode_id'] == 'episode-after-reset' for m, _ in received) < 12:
            assert process.poll() is None and time.monotonic() < deadline
            time.sleep(.02)
        for camera in ('overview', 'wrist'):
            rows = [m for m, _ in received if m['capture_episode_id'] == 'episode-after-reset' and m['camera_id'] == camera]
            assert [int(m['source_frame_id']) for m in rows] == sorted(frames)
            assert all(m['session_id'] == bridge.session_id for m in rows)
        for metadata, products in received:
            assert metadata["state_kind"] == "authoritative"
            image = Image.open(io.BytesIO(products['rgb']))
            assert image.size == (160, 90)
            image.save(tmp_path / f"{metadata['capture_episode_id']}-{metadata['source_frame_id']}-{metadata['camera_id']}.png")
            rgb = np.asarray(image.convert('RGB'), dtype=np.int16)
            yy, xx = np.nonzero((rgb[:, :, 0] > 80) & (rgb[:, :, 0] > rgb[:, :, 1] + 40)
                               & (rgb[:, :, 0] > rgb[:, :, 2] + 40))
            assert len(xx) > 25, 'missing/red-blue-swapped marker'
            assert yy.mean() < 40, 'image was vertically flipped'
            # Protocol mirrors body Y into UE Y: positive Y is image-left.
            if int(metadata['source_frame_id']) % 2 == 0: assert xx.mean() < 70
            else: assert xx.mean() > 90
        assert not errors
        print('UE capture: idle/STOP boundaries, two episodes, in-flight reset, moving-frame identity and RGB orientation passed')
    finally:
        if bridge: bridge.close()
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(15)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(10)
        captures.close()
