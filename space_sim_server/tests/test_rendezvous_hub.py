"""Exercise the real TCP framing and validation of the read-only rendezvous port."""

import asyncio
import json
from pathlib import Path

from fastapi.testclient import TestClient
from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.protocol import write_async
from space_arm_platform.rendezvous_hub import RendezvousHub, RendezvousState
from space_arm_platform.rendezvous_runtime import RendezvousRuntime


def test_rendezvous_state_transport():
    async def exercise():
        hub = RendezvousHub()
        await hub.start("127.0.0.1", 0)
        try:
            _reader, writer = await asyncio.open_connection("127.0.0.1", hub.bound_port)
            body = {
                "position_m": [7_000_000.0, 0.0, 0.0],
                "velocity_m_s": [0.0, 7500.0, 0.0],
                "attitude_wxyz": [1.0, 0.0, 0.0, 0.0],
                "angular_velocity_rad_s": [0.0, 0.0, 0.0],
            }
            packet = {
                "protocol": "space-sim-state/1",
                "type": "state",
                "sim_time_s": 0.5,
                "bodies": [
                    {**body, "body_id": "rendezvous/target"},
                    {
                        **body,
                        "body_id": "rendezvous/chaser",
                        "position_m": [7_000_000.0, -7000.0, 0.0],
                    },
                ],
                "metadata": {"run_id": "probe", "phase": "Transfer acquisition"},
            }
            await write_async(writer, packet)
            for _ in range(100):
                if hub.status()["state"] is not None:
                    break
                await asyncio.sleep(0.01)
            status = hub.status()
            assert status["connected"]
            assert status["phase"] == "Transfer acquisition"
            assert status["separation_m"] == 7000.0
            await write_async(writer, {**packet, "sim_time_s": 0.5})
            await asyncio.sleep(0.05)
            assert hub.status()["sim_time_s"] == 0.5
            writer.close()
            await writer.wait_closed()
        finally:
            await hub.close()

    asyncio.run(exercise())


def test_authenticated_rendezvous_endpoint(tmp_path):
    app = create_app(
        PlatformConfig(
            project_root=Path(__file__).resolve().parents[1],
            data_root=tmp_path,
            simulation_port=0,
            capture_port=0,
        )
    )
    with TestClient(app) as client:
        assert client.get("/api/rendezvous/state").status_code == 401
        login = client.post(
            "/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}
        )
        assert login.status_code == 200
        state = client.get("/api/rendezvous/state")
        assert state.status_code == 200
        assert state.json()["runtime"]["configured"] is False
        assert (
            client.post("/api/rendezvous/start", json={"live": True}).status_code == 409
        )
        assert client.post("/api/rendezvous/start", json={"live": 1}).status_code == 422


def test_completed_report_survives_backend_restart(tmp_path):
    report_dir = tmp_path / "repo" / "output" / "spacesim_platform" / "run-1"
    report_dir.mkdir(parents=True)
    (report_dir / "mission_report.json").write_text(
        json.dumps(
            {
                "outcome": "Docked (native contact and latch)",
                "basic_nominal_passed": True,
                "capture": {"latch_time_s": 6460.086},
                "final_time_s": 6640.5,
                "final_body_origin_distance_m": 2.0,
            }
        ),
        encoding="utf-8",
    )
    runtime = RendezvousRuntime(tmp_path / "repo", None, 8768, tmp_path / "logs")
    assert runtime.status()["report"]["latch_time_s"] == 6460.086


def test_new_mission_does_not_show_previous_mission_state(tmp_path, monkeypatch):
    app = create_app(PlatformConfig(
        project_root=Path(__file__).resolve().parents[1], data_root=tmp_path,
        simulation_port=0, capture_port=0,
    ))
    body = dict(position_m=[7_000_000.0, 0.0, 0.0], velocity_m_s=[0.0, 7500.0, 0.0],
                attitude_wxyz=[1.0, 0.0, 0.0, 0.0], angular_velocity_rad_s=[0.0, 0.0, 0.0])
    with TestClient(app) as client:
        client.post('/api/auth/login', json={'username': 'admin', 'password': 'ChangeMe123!'})
        app.state.rendezvous._latest = RendezvousState(
            protocol='space-sim-state/1', type='state', sim_time_s=5120.0,
            bodies=[dict(body, body_id='rendezvous/target'), dict(body, body_id='rendezvous/chaser')],
            metadata={'run_id': 'previous-flight', 'phase': '100 m handoff hold', 'scenario': 'in-orbit'},
        )
        def unavailable(*args):
            raise RuntimeError('Renderer unavailable')
        monkeypatch.setattr(app.state.rendezvous_runtime, 'start', unavailable)
        assert client.post('/api/rendezvous/start', json={'scenario': 'team-orbit-raise'}).status_code == 409
        assert client.get('/api/rendezvous/state').json()['run_id'] == 'previous-flight'
        monkeypatch.setattr(app.state.rendezvous_runtime, 'start', lambda *args: {'running': True})
        assert client.post('/api/rendezvous/start', json={'scenario': 'team-orbit-raise'}).status_code == 200
        state = client.get('/api/rendezvous/state').json()
        assert state['state'] is None
        assert 'run_id' not in state and 'sim_time_s' not in state
