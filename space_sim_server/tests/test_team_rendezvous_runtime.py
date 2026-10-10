"""Check real platform request selection and handoff admission boundaries."""

import hashlib
import json
import sys
from pathlib import Path

import pytest
from space_arm_platform.app import RendezvousStart
from space_arm_platform.rendezvous_runtime import RendezvousRuntime

from simulation.orbital_rendezvous.handoff import load_handoff


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
