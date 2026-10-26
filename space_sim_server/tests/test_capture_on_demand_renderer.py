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
    from space_arm_platform.unreal_runtime import unreal_editor
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
    editor = unreal_editor(Path(unreal_root))
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
        startup_options = {}
        if os.name == "nt":
            startup = subprocess.STARTUPINFO()
            startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startup.wShowWindow = 0
            startup_options = dict(startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW)
        process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            **startup_options)
        # Cold Linux shader compilation can exceed a minute. Keep the normal
        # bound while allowing deployment acceptance to budget a cold start.
        startup_timeout = float(os.environ.get("SPACE_SIM_UE_STARTUP_TIMEOUT", "60"))
        assert 0 < startup_timeout <= 1800
        deadline = time.monotonic()+startup_timeout
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


def test_real_ue_camera_modes_combine_fisheye_projection_and_monochrome(tmp_path):
    import io
    import json
    import math
    import numpy as np
    from PIL import Image
    from Basilisk.architecture import messaging
    from bsk_render_adapter import BasiliskRenderBridge, CameraVisual, GeometryVisual
    from space_arm_platform.capture_receiver import CaptureReceiver
    from space_arm_platform.unreal_runtime import unreal_editor

    unreal_root = os.environ.get("UE56_ROOT")
    if not unreal_root:
        pytest.skip("set UE56_ROOT for the isolated real-GPU camera test")
    root = Path(__file__).resolve().parents[1]
    adapter = Path(os.environ.get("SPACE_SIM_RESET_ADAPTER", str(root.parent / "space_sim_UE_Adapter")))
    modes = ("rgb", "fisheye", "monochrome", "monochrome_fisheye")
    received, errors = [], []
    captures = CaptureReceiver("127.0.0.1", 0, lambda m, p: received.append((m, p)), errors.append)
    captures.start()
    process = bridge = None
    try:
        deadline = time.monotonic() + 5
        while captures._listener is None:
            assert time.monotonic() < deadline
            time.sleep(.01)
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            render_port = reservation.getsockname()[1]
        command = [str(unreal_editor(Path(unreal_root))),
            str(adapter / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject"),
            "-game", "-unattended", "-nop4", "-nosplash", "-RenderOffscreen",
            "-ForceRes", "-ResX=320", "-ResY=180", "-BskListen=127.0.0.1", f"-BskPort={render_port}",
            "-BskCaptureProducts=rgb", "-BskCaptureRate=30", "-BskCaptureHost=127.0.0.1",
            f"-BskCapturePort={captures._listener.getsockname()[1]}",
            "-ExecCmds=t.MaxFPS 30,r.VSync 0", f"-AbsLog={tmp_path / 'ue.log'}"]
        options = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {}
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, **options)
        deadline = time.monotonic() + float(os.environ.get("SPACE_SIM_UE_STARTUP_TIMEOUT", "120"))
        while True:
            assert process.poll() is None, (tmp_path / "ue.log").read_text(errors="replace")
            try:
                with socket.create_connection(("127.0.0.1", render_port), timeout=.2):
                    break
            except OSError:
                assert time.monotonic() < deadline, "isolated renderer startup timeout"
                time.sleep(.1)
        capture = {"capture_episode_id": "", "capture_request_id": "idle"}
        bridge = BasiliskRenderBridge(port=render_port, origin_object="lens/body",
            capture_state_provider=lambda: dict(capture))
        origin = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
        bridge.add_object("lens/body", origin,
            geometries=[GeometryVisual("lens/origin", "box", (.001, .001, .001))])
        # A static coloured target makes image geometry directly comparable
        # across cameras without motion, timing or dynamics confounders.
        # Native subscribers do not own the source messages.
        marker_messages = []
        for index, (y, z, colour) in enumerate((
                (-.5, .3, (1., 0., 0., 1.)), (.5, -.3, (0., 1., 0., 1.)),
                (-3., .4, (0., 0., 1., 1.)), (3., -.4, (1., 1., 0., 1.)))):
            pose = messaging.SCStatesMsgPayload()
            pose.r_BN_N = [0., y, z]
            message = messaging.SCStatesMsg().write(pose)
            marker_messages.append(message)
            bridge.add_object(f"lens/marker{index}", message,
                geometries=[GeometryVisual(f"lens/box{index}", "box", (.3, .3, .3),
                    color_rgba=colour, material_emission=1.)])
        for mode in modes:
            bridge.add_camera(CameraVisual(camera_id=mode, parent_id="lens/body",
                position_body_m=(-2, 0, 0), camera_mode=mode, resolution=(1024, 1024),
                field_of_view_rad=math.radians(160 if "fisheye" in mode else 60),
                capture_rate_hz=30, capture_products=("rgb",)))
        bridge.Reset(0)
        for frame in range(30):
            bridge.UpdateState((frame * 1_000_000_000 + 15) // 30)
            time.sleep(.04)
        assert not received, "idle scene produced authoritative frames"
        capture.update(capture_episode_id="camera-mode-test", capture_request_id="capture")
        for frame in range(30, 33):
            bridge.UpdateState((frame * 1_000_000_000 + 15) // 30)
            time.sleep(.06)
        capture.update(capture_episode_id="", capture_request_id="stop")
        bridge.UpdateState(1_100_000_000)
        deadline = time.monotonic() + 120
        idle_frame = 34
        while len(received) < 12:
            assert process.poll() is None and time.monotonic() < deadline, (len(received), errors)
            bridge.UpdateState((idle_frame * 1_000_000_000 + 15) // 30)
            idle_frame += 1
            time.sleep(.05)
        assert not errors, errors
        images = {}
        for mode in modes:
            rows = [(m, p) for m, p in received if m.get("camera_id") == mode]
            assert len(rows) == 3
            assert [int(m["source_frame_id"]) for m, _ in rows] == [30, 31, 32]
            metadata, products = rows[-1]
            (tmp_path / f"{mode}.json").write_text(json.dumps(metadata, indent=2))
            assert metadata["camera_mode"] == mode
            assert metadata["color_model"] == ("monochrome_rgb" if "monochrome" in mode else "rgb")
            lens = metadata["intrinsics"]
            assert lens["model"] == ("equidistant_fisheye" if "fisheye" in mode else "pinhole")
            if "fisheye" in mode:
                assert lens["valid_circle_radius_px"] == 512
                assert lens["fx_px"] == pytest.approx(1024 / math.radians(160))
            image = Image.open(io.BytesIO(products["rgb"])).convert("RGB")
            assert image.size == (1024, 1024)
            image.save(tmp_path / f"{mode}.png")
            rgb = np.asarray(image, dtype=np.int16)
            assert np.count_nonzero(rgb.max(axis=2) > 30) > 300, "empty camera image"
            if "monochrome" in mode:
                assert np.max(np.ptp(rgb, axis=2)) == 0, "monochrome RGB channels differ"
            else:
                assert np.count_nonzero(np.ptp(rgb, axis=2) > 40) > 300, "colour mode lost chroma"
            if "fisheye" in mode:
                yy, xx = np.indices(rgb.shape[:2])
                outside = (xx + .5 - 512)**2 + (yy + .5 - 512)**2 > 514**2
                assert np.max(rgb[outside]) <= 1, "fisheye circle exterior is not black"
            images[mode] = rgb
        # Both transforms must combine without moving, cropping or mirroring
        # the target. Allow tone mapping / output quantisation at dark edges.
        for colour, gray in (("rgb", "monochrome"), ("fisheye", "monochrome_fisheye")):
            a = images[colour].max(axis=2) > 30
            b = images[gray].max(axis=2) > 15
            assert np.count_nonzero(a & b) / np.count_nonzero(a | b) > .95
        assert sum(m["capture_episode_id"] == "camera-mode-test" for m, _ in received) == 12
    finally:
        if bridge:
            bridge.close()
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(10)
        captures.close()
