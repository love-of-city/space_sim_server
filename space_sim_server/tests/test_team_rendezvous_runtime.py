"""Check real platform request selection and handoff admission boundaries."""

import hashlib
import json
import contextlib
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from space_arm_platform.app import RendezvousStart
from space_arm_platform.rendezvous_runtime import RendezvousRuntime

from simulation.orbital_rendezvous.handoff import load_handoff


@pytest.mark.skipif(os.name != "nt", reason="Windows child handle inheritance")
def test_renderer_launcher_does_not_wait_for_long_lived_child_stdout(tmp_path, monkeypatch):
    """Exercise the actual PowerShell Start-Process path without launching UE."""
    import psutil

    adapter = tmp_path / "adapter"
    scripts = adapter / "scripts"
    scripts.mkdir(parents=True)
    child_pid_file = tmp_path / "child.pid"
    marker = f"renderer-probe-{tmp_path.name}"
    quote = lambda value: "'" + str(value).replace("'", "''") + "'"
    (scripts / "start_renderer.ps1").write_text(
        "$ErrorActionPreference='Stop'\n"
        f"$child = Start-Process -FilePath {quote(sys.executable)} "
        f"-ArgumentList @('-u','-c','\"import time; time.sleep(30) # {marker}\"') -PassThru -WindowStyle Hidden\n"
        f"$child.Id | Set-Content -LiteralPath {quote(child_pid_file)} -Encoding ascii\n"
        "Write-Output 'renderer launcher finished'\n", encoding="utf-8")
    config = SimpleNamespace(runtime_adapter_root=adapter, runtime_unreal_root=tmp_path,
                             runtime_powershell_exe="pwsh", runtime_pixel_streamer_port=28888,
                             pixel_streaming_streamer_id="test", runtime_preview_rate=90,
                             runtime_renderer_ready_timeout=2)
    runtime = RendezvousRuntime(None, Path(sys.executable), 0, tmp_path / "logs", tmp_path,
                                launch_config=config)
    connections = 0

    def connect(*args, **kwargs):
        nonlocal connections
        connections += 1
        if connections == 1:
            raise OSError("renderer not started")
        return contextlib.nullcontext()

    monkeypatch.setattr("space_arm_platform.rendezvous_runtime.socket.create_connection", connect)
    monkeypatch.setattr(runtime, "_remember_renderer", lambda *args, **kwargs: None)
    started = time.monotonic()
    try:
        runtime._ensure_renderer()
        assert time.monotonic() - started < 10, "Launcher waited for child pipe EOF"
        assert psutil.Process(int(child_pid_file.read_text())).is_running()
        assert "renderer launcher finished" in (tmp_path / "logs/renderer-start.log").read_text()
    finally:
        if child_pid_file.exists():
            with contextlib.suppress(psutil.NoSuchProcess):
                child = psutil.Process(int(child_pid_file.read_text()))
                # A failing test may outlive the probe; never clean a reused PID.
                if any(marker in argument for argument in child.cmdline()):
                    owned = [*child.children(recursive=True), child]
                    for process in owned:
                        with contextlib.suppress(psutil.NoSuchProcess):
                            process.kill()
                    psutil.wait_procs(owned, timeout=5)


@pytest.mark.parametrize("render_port", [5558, 25558])
def test_team_launch_does_not_require_old_demo_repository(tmp_path, monkeypatch, render_port):
    entry = tmp_path / "simulation/orbital_rendezvous/run.py"
    entry.parent.mkdir(parents=True)
    entry.write_text("")
    launched = []

    class Process:
        pid = 42

        def __init__(self, command, **kwargs):
            launched.append((command, kwargs))

        def poll(self):
            return None

        def terminate(self):
            pass

        def wait(self, **kwargs):
            return 0

    monkeypatch.setattr(
        "space_arm_platform.rendezvous_runtime.subprocess.Popen", Process
    )
    runtime = RendezvousRuntime(
        None, Path(sys.executable), 28768, tmp_path / "logs", tmp_path, render_port=render_port
    )
    assert runtime.start(scenario="team-orbit-raise")["running"]
    command, options = launched[0]
    assert command[command.index("-m") + 1] == "simulation.orbital_rendezvous.run"
    assert command[command.index("--scenario") + 1] == "orbit-raise"
    assert command[command.index("--state-port") + 1] == "28768"
    assert command[command.index("--render-port") + 1] == str(render_port)
    assert options["cwd"] == tmp_path
    assert str(tmp_path / "run/team_rendezvous") in str(runtime.output_dir)
    with pytest.raises(RuntimeError):
        runtime.start(scenario="team-in-orbit")
    runtime.close()


def test_api_defaults_to_team_models_and_rejects_unknown_scenario():
    assert RendezvousStart().scenario == "team-in-orbit"
    with pytest.raises(ValueError):
        RendezvousStart(scenario="arbitrary-command")


def test_flight_renderer_uses_the_same_camera_streams_as_the_operator_console(tmp_path, monkeypatch):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    (scripts / "start_renderer.ps1").write_text("")
    config = SimpleNamespace(runtime_adapter_root=tmp_path, runtime_unreal_root=tmp_path,
        runtime_powershell_exe="pwsh", runtime_pixel_streamer_port=28888,
        pixel_streaming_streamer_id="test", runtime_preview_rate=60, runtime_renderer_ready_timeout=2,
        runtime_encoder_min_quality=65, runtime_pixel_streaming_camera_ids=("teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam"),
        runtime_pixel_streaming_camera_width=640, runtime_pixel_streaming_camera_height=360)
    runtime = RendezvousRuntime(None, Path(sys.executable), 0, tmp_path / "logs", tmp_path, launch_config=config)
    attempts = 0
    def connect(*args, **kwargs):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise OSError("cold renderer")
        return contextlib.nullcontext()
    commands = []
    def launch(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr("space_arm_platform.rendezvous_runtime.socket.create_connection", connect)
    monkeypatch.setattr("space_arm_platform.rendezvous_runtime.subprocess.run", launch)
    monkeypatch.setattr(runtime, "_remember_renderer", lambda *args, **kwargs: None)
    runtime._ensure_renderer()
    command = commands[0]
    assert command[command.index("-PixelStreamingCameraIds") + 1] == "team/camera/spacecraft_overview+team/camera/sarm_wrist_cam"
    assert command[command.index("-PixelStreamingCameraFps") + 1] == "60"
    assert command[command.index("-PixelStreamingCameraWidth") + 1] == "640"
    assert command[command.index("-PixelStreamingCameraHeight") + 1] == "360"
    assert command[command.index("-EncoderMinQuality") + 1] == "65"


def test_preview_aliases_are_scoped_to_configured_cameras_in_stream_tokens(tmp_path):
    from fastapi.testclient import TestClient
    from space_arm_platform.app import PlatformConfig, create_app
    from space_arm_platform.stream_access import verify_stream_access_token
    (tmp_path / "frontend").mkdir()
    app = create_app(PlatformConfig(project_root=tmp_path, data_root=tmp_path / "episodes",
        simulation_port=0, capture_port=0, pixel_streaming_streamer_id="own",
        pixel_streaming_camera_streamers=(("own__teleop_camera_sarm_wrist_cam", "WristCamera"), ("custom", "Custom")),
        stream_access_jwt_secret="test-secret"))
    with TestClient(app) as client:
        client.post('/api/auth/login', json={"username": "admin", "password": "ChangeMe123!"})
        data = client.get('/api/client-config').json()
        assert data['pixel_streaming_streamers'][1]['aliases'] == ['own__team_camera_sarm_wrist_cam']
        assert 'aliases' not in data['pixel_streaming_streamers'][2]
        claims = verify_stream_access_token('test-secret', data['pixel_streaming_access_token'])
        assert claims['streamer_ids'] == ['own', 'own__teleop_camera_sarm_wrist_cam', 'own__team_camera_sarm_wrist_cam', 'custom']


def test_handoff_refuses_failed_or_different_model(tmp_path):
    model = tmp_path / "source.xml"
    model.write_text("<mujoco/>")
    handoff = tmp_path / "handoff.json"
    data = {
        "schema": "spacesim-near-field-handoff/1",
        "accepted": False,
        "source_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
    }
    handoff.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="accepted"):
        load_handoff(handoff, model)
    data.update(accepted=True, source_sha256="different")
    handoff.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="differs"):
        load_handoff(handoff, model)


def test_handoff_scene_preserves_epoch_and_stowed_configuration(tmp_path):
    from space_arm_platform.models import SceneInstanceCreate
    from space_arm_platform.scene_runtime import SceneRuntimeManager

    manager = SceneRuntimeManager(None, project_root=tmp_path)
    handoff = {"accepted": True, "ephemeris_epoch_utc": "2026 SEPTEMBER 02 00:00:00.000",
               "state": {"sim_time_s": 11120.5}}
    instance = manager.create_instance(SceneInstanceCreate(randomization_profile="none"), handoff=handoff)
    saved = json.loads(Path(instance["config_path"]).read_text(encoding="utf-8"))
    assert saved["environment"]["ephemeris_epoch_utc"] == "2026 September 02 03:05:20.500000"
    assert saved["rendezvous_handoff"] == handoff
    assert saved["randomization"]["arm_joint_position_rad"] == [0.0] * 8
    assert saved["arm_preparation_required"] is False


def test_handoff_api_admission_before_releasing_renderer(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from space_arm_platform.app import PlatformConfig, create_app
    from simulation.orbital_rendezvous.model import DEFAULT_MODEL

    source = tmp_path / DEFAULT_MODEL
    source.parent.mkdir(parents=True)
    source.write_text("<mujoco/>")
    (tmp_path / "frontend").mkdir()
    app = create_app(PlatformConfig(project_root=tmp_path, data_root=tmp_path / "data",
                                    simulation_port=0, capture_port=0))
    calls = []
    monkeypatch.setattr(app.state.rendezvous_runtime, "release_renderer", lambda: calls.append("release"))
    monkeypatch.setattr(app.state.scenes, "start", lambda request, creator, **kw: {
        "phase": "launching", "created_by": creator, "handoff": kw["handoff"]})
    folder = tmp_path / "flight"
    folder.mkdir()
    app.state.rendezvous_runtime._output_dir = folder
    state = {"body_id": "rendezvous/chaser", "position_m": [7000000, 0, 0],
             "velocity_m_s": [0, 7500, 0], "attitude_wxyz": [1, 0, 0, 0],
             "angular_velocity_rad_s": [0, 0, 0]}
    data = {"schema": "spacesim-near-field-handoff/1", "accepted": False,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "state": {"bodies": [state, {**state, "body_id": "rendezvous/target"}]}}
    path = folder / "handoff.json"
    with TestClient(app) as client:
        assert client.post("/api/rendezvous/handoff").status_code == 401
        client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"})
        assert client.post("/api/rendezvous/handoff").status_code == 409
        path.write_text(json.dumps(data))
        assert client.post("/api/rendezvous/handoff").status_code == 409
        assert not calls
        data["accepted"] = True
        path.write_text(json.dumps(data))
        monkeypatch.setattr(app.state.scenes, "status", lambda: {"active": True})
        assert client.post("/api/rendezvous/handoff").status_code == 409
        assert not calls
        monkeypatch.setattr(app.state.scenes, "status", lambda: {"active": False})
        response = client.post("/api/rendezvous/handoff")
        assert response.status_code == 200
        assert calls == ["release"]
        assert response.json()["handoff"] == data
        assert response.json()["created_by"]["username"] == "admin"
