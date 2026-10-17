"""Mission admission, persistence and cancellation with actual worker scheduling."""
import asyncio
import json
import threading
from types import SimpleNamespace

import pytest

from space_arm_platform.mission_runtime import MissionRuntime
from space_arm_platform.models import SceneInstanceCreate

OWNER = {"user_id": "operator-a", "username": "a", "role": "operator"}
OTHER = {"user_id": "operator-b", "username": "b", "role": "operator"}
ADMIN = {"user_id": "admin", "username": "admin", "role": "admin"}


async def wait_for_condition(predicate):
    async def wait():
        while not predicate():
            await asyncio.sleep(.01)
    await asyncio.wait_for(wait(), 4)


class Flight:
    def __init__(self, root):
        self.output_dir = None
        self.root = root
        self.running = False
        self.report = None
        self.entered = threading.Event()
        self.proceed = threading.Event()
        self.cancelled = False
        self.released = 0
        self.calls = 0

    def status(self):
        return {"configured": True, "running": self.running, "report": self.report,
                "scenario": "team-in-orbit", "output_dir": str(self.output_dir) if self.output_dir else None}

    def prepare_start(self):
        self.cancelled = False

    def cancel_start(self):
        self.cancelled = True
        self.proceed.set()

    def start(self, *args):
        self.calls += 1
        self.entered.set()
        assert self.proceed.wait(5), "test worker not released"
        if self.cancelled:
            raise RuntimeError("cancelled")
        self.output_dir = self.root / "flight"
        self.running = True
        return self.status()

    def release_renderer(self):
        self.released += 1

    def stop(self):
        self.running = False
        return self.status()


def setup(tmp_path):
    flight = Flight(tmp_path)
    scene_state = {"active": False, "phase": "idle"}
    scenes = SimpleNamespace(status=lambda: dict(scene_state), project_root=tmp_path)
    telemetry = SimpleNamespace(status=lambda: {"connected": False}, clear_cached_state=lambda: None)
    busy = [False]
    manager = MissionRuntime(tmp_path / "missions", scenes, flight, telemetry, lambda: busy[0])
    return manager, flight, scenes, busy, scene_state


def test_continuous_mission_preserves_identity_and_orders_accepted_phases(tmp_path, monkeypatch):
    async def run():
        manager, flight, scenes, _, scene_state = setup(tmp_path)
        starts, scene_starts = [], []
        def start(live, mode, scenario, options):
            starts.append((scenario, options))
            flight.output_dir = tmp_path / f'flight-{len(starts)}'
            flight.running, flight.report = True, None
            return flight.status()
        flight.start = start
        def scene_start(request, user, **kwargs):
            scene_starts.append(kwargs)
            scene_state.update(active=True, phase='running')
            return {'instance_id':'continuous-operation'}
        scenes.start = scene_start
        monkeypatch.setattr('simulation.orbital_rendezvous.handoff.load_handoff',
                            lambda *args: {'accepted':True,'state':{'metadata':{'run_id':'near-source'}}})
        await manager.start_flight(OWNER, scenario='team-in-orbit', options={}, continuous=True)
        identity = manager.status(OWNER)['mission_id']
        assert starts[0][1]['differential_gnss']['relative_position_sigma_m'] == .03
        assert starts[0][1]['gnss_position_sigma_m'] == starts[0][1]['differential_gnss']['absolute_position_sigma_m']
        flight.running, flight.report = False, {'basic_nominal_passed':True, 'outcome':'far accepted'}
        with pytest.raises(RuntimeError, match='连续任务'):
            await manager.start_scene(SceneInstanceCreate(), OWNER)
        await wait_for_condition(lambda: len(starts) == 2)
        await wait_for_condition(lambda: manager._operation is None)
        assert starts[1][0] == 'near-field-resume'
        flight.running, flight.report = False, {'basic_nominal_passed':True, 'outcome':'CaptureHold'}
        await wait_for_condition(lambda: manager._monitor is None)
        status = manager.status(OWNER)
        assert status['mission_id'] == identity and status['stage'] == 'teleop'
        assert [stage['phase'] for stage in status['stages']] == ['far','near']
        assert scene_starts[0]['mission']['mission_id'] == identity
        assert status['source_run_id'] == 'near-source'
        await manager.close()
    asyncio.run(run())


@pytest.mark.parametrize('stop', [False, True])
def test_continuous_failure_or_user_stop_never_launches_the_next_phase(tmp_path, stop):
    async def run():
        manager, flight, _, _, _ = setup(tmp_path)
        flight.proceed.set()
        await manager.start_flight(OWNER, scenario='team-orbit-raise',
                                   options={'gnss_position_sigma_m': .8, 'gnss_velocity_sigma_m_s': .004}, continuous=True)
        effective = manager.status(OWNER)['options']['differential_gnss']
        assert effective['absolute_position_sigma_m'] == .8
        assert effective['absolute_velocity_sigma_m_s'] == .004
        if stop:
            await manager.stop(OWNER, None)
        else:
            flight.running, flight.report = False, {'basic_nominal_passed':False}
            await wait_for_condition(lambda: manager._monitor is None)
        await asyncio.sleep(.3)
        assert flight.calls == 1
        assert manager.status(OWNER)['phase'] == ('stopped' if stop else 'failed')
        assert not manager.status(OWNER)['active']
        await manager.close()
    asyncio.run(run())


def test_background_admission_blocks_both_executors_and_retains_creator(tmp_path):
    async def run():
        manager, flight, scenes, _, _ = setup(tmp_path)
        accepted = await manager.start_flight(OWNER, scenario="team-in-orbit", options={}, background=True)
        assert accepted["phase"] == "starting" and accepted["active"]
        try:
            with pytest.raises(RuntimeError):
                await manager.start_scene(SceneInstanceCreate(), OTHER)
            with pytest.raises(RuntimeError):
                await manager.start_flight(OTHER, scenario="team-orbit-raise", options={})
            assert not manager.status(OTHER)["can_manage"]
            assert manager.status(OWNER)["can_manage"]
            assert manager.status(ADMIN)["can_manage"]
        finally:
            flight.proceed.set()
            await manager._task
        assert flight.calls == 1
        saved = json.loads((tmp_path / "missions/latest.json").read_text(encoding="utf-8"))
        assert saved["mission_id"] == accepted["mission_id"]
        assert saved["created_by"] == OWNER
        assert saved["flight_output_dir"] == str(flight.output_dir)
    asyncio.run(run())


def test_stop_during_cold_start_cancels_then_releases_renderer_without_launch(tmp_path):
    async def run():
        manager, flight, _, _, _ = setup(tmp_path)
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={}, background=True)
        await asyncio.to_thread(flight.entered.wait, 2)
        with pytest.raises(PermissionError):
            await manager.stop(OTHER, None)
        await manager.stop(OWNER, None)
        assert flight.cancelled and not flight.running and flight.released == 1
        assert manager.status(OWNER)["phase"] == "stopped"
        assert not manager.status(OWNER)["active"]
        # The worker is drained before the same task can be replaced.
        flight.proceed.set()
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={})
        assert flight.running and flight.calls == 2
    asyncio.run(run())


def test_rejected_options_and_capture_conflict_do_not_replace_existing_record(tmp_path):
    async def run():
        manager, flight, _, busy, _ = setup(tmp_path)
        flight.proceed.set()
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={"seed": 19})
        flight.running = False
        original = manager.status(OWNER)["mission_id"]
        for options in ({"seed": -1}, {"unknown": True}):
            with pytest.raises(ValueError):
                await manager.start_flight(OWNER, scenario="team-in-orbit", options=options)
        busy[0] = True
        with pytest.raises(RuntimeError):
            await manager.start_scene(SceneInstanceCreate(), OWNER)
        assert manager.status(OWNER)["mission_id"] == original
    asyncio.run(run())


def test_reload_preserves_task_and_episode_provenance_and_ownership(tmp_path):
    async def run():
        manager, flight, scenes, _, _ = setup(tmp_path)
        flight.proceed.set()
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={})
        flight.running = False
        flight.report = {"basic_nominal_passed": True, "final_center_distance_m": 110.0}
        manager.add_episode("episode-1")
        manager.add_episode("episode-1")
        restored = MissionRuntime(manager.root, scenes, flight, manager.telemetry, lambda: False)
        status = restored.status(OWNER)
        assert status["mission_id"] == manager.status(OWNER)["mission_id"]
        assert status["episodes"] == ["episode-1"]
        assert status["phase"] == "ready" and status["can_handoff"]
        assert not restored.status(OTHER)["can_manage"]
    asyncio.run(run())


def test_launch_failure_is_visible_and_releases_admission(tmp_path):
    async def run():
        manager, flight, _, _, _ = setup(tmp_path)
        def fail(*args):
            raise RuntimeError("renderer unavailable")
        flight.start = fail
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={}, background=True)
        await manager._task
        assert manager.status(OWNER)["phase"] == "failed"
        assert manager.status(OWNER)["error"] == "renderer unavailable"
        assert not manager.status(OWNER)["active"]
        manager.require_available()
    asyncio.run(run())


def test_handoff_keeps_mission_identity_in_scene_and_can_be_repeated_after_stop(tmp_path, monkeypatch):
    async def run():
        manager, flight, scenes, _, state = setup(tmp_path)
        flight.proceed.set()
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={})
        flight.running = False
        flight.report = {"basic_nominal_passed": True}
        identity = manager.status(OWNER)["mission_id"]
        handoff = {"accepted": True, "state": {"metadata": {"run_id": "source-flight"}}, "mission_options": {"seed": 19}}
        monkeypatch.setattr("simulation.orbital_rendezvous.handoff.load_handoff", lambda *args: handoff)
        instances = []
        def start(request, user, **kwargs):
            assert request.randomization_profile == "none"
            item = {"instance_id": f"scene-{len(instances)}", "created_by": user, **kwargs}
            instances.append(item)
            state.update(active=True, phase="running", instance=item)
            return item
        scenes.start = start
        with pytest.raises(PermissionError):
            await manager.handoff(OTHER)
        await manager.handoff(OWNER)
        assert instances[0]["mission"]["mission_id"] == identity
        assert instances[0]["mission"]["source_run_id"] == "source-flight"
        assert manager.status(OWNER)["stage"] == "teleop"
        with pytest.raises(RuntimeError):
            await manager.handoff(OWNER)
        async def stop_scene():
            state.update(active=False, phase="stopped")
            return state
        await manager.stop(OWNER, stop_scene)
        assert manager.status(OWNER)["can_handoff"]
        await manager.handoff(OWNER)
        assert manager.status(OWNER)["mission_id"] == identity
        assert len(manager.status(OWNER)["scene_instance_ids"]) == 2
    asyncio.run(run())


def test_existing_and_new_http_routes_share_admission_and_original_page_redirect(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from space_arm_platform.app import PlatformConfig, create_app
    (tmp_path / "frontend").mkdir()
    app = create_app(PlatformConfig(project_root=tmp_path, data_root=tmp_path / "data/episodes",
                                    simulation_port=0, capture_port=0))
    assert app.state.missions.root == tmp_path / "data/episodes/.missions"
    flight = Flight(tmp_path)
    app.state.missions.flight = flight
    with TestClient(app) as client:
        assert client.post('/api/mission/start', json={"mode": "team-in-orbit"}).status_code == 401
        client.post('/api/auth/login', json={"username": "admin", "password": "ChangeMe123!"})
        old = client.get('/static/rendezvous.html?access_key=test', follow_redirects=False)
        assert old.status_code == 307 and old.headers['location'] == '/?access_key=test#mission'
        try:
            started = client.post('/api/mission/start', json={"mode": "team-in-orbit", "options": {"seed": 19}})
            assert started.status_code == 202
            identity = started.json()['mission_id']
            state = client.get('/api/state').json()['mission']
            assert state['mission_id'] == identity and state['phase'] == 'starting'
            assert client.post('/api/scenes/start', json={}).status_code == 409
            assert client.post('/api/rendezvous/start', json={}).status_code == 409
            assert client.post('/api/mission/handoff').status_code == 409
            assert client.post('/api/scenes/reset').status_code == 409
            assert client.post('/api/mission/start', json={"mode": "team-in-orbit", "options": {"seed": -1}}).status_code == 422
            stopped = client.post('/api/mission/stop')
            assert stopped.status_code == 200 and not stopped.json()['running']
            assert client.get('/api/mission/state').json()['phase'] == 'stopped'
        finally:
            flight.cancel_start()


def test_download_stays_bound_to_original_mission_and_rejects_foreign_paths(tmp_path):
    manager, flight, _, _, _ = setup(tmp_path)
    manager._new("team-in-orbit", OWNER)
    original = manager._record["mission_id"]
    folder = tmp_path / "run/team_rendezvous/first"
    folder.mkdir(parents=True)
    manager._record["flight_output_dir"] = str(folder)
    manager._save()
    manager._new("team-orbit-raise", OWNER)
    assert manager.artifact(original, "mission_report.json") == folder / "mission_report.json"
    with pytest.raises(ValueError):
        manager.artifact("../../private", "mission_report.json")
    manager._record["flight_output_dir"] = str(tmp_path.parent)
    manager._save()
    with pytest.raises(ValueError, match="不属于"):
        manager.artifact(manager._record["mission_id"], "mission_report.json")


def test_disconnected_start_request_does_not_abandon_the_launch_worker(tmp_path):
    async def run():
        manager, flight, _, _, _ = setup(tmp_path)
        request = asyncio.create_task(manager.start_flight(OWNER, scenario="team-in-orbit", options={}))
        await asyncio.to_thread(flight.entered.wait, 2)
        request.cancel()
        with pytest.raises(asyncio.CancelledError):
            await request
        assert manager.status(OWNER)["phase"] == "starting"
        assert not manager._task.cancelled()
        flight.proceed.set()
        await manager._task
        assert manager.status(OWNER)["phase"] == "flight"
        assert manager.status(OWNER)["flight_output_dir"] == str(flight.output_dir)
    asyncio.run(run())


def test_disconnected_stop_request_still_finishes_cold_start_cleanup(tmp_path):
    async def run():
        manager, flight, _, _, _ = setup(tmp_path)
        stopping, finish_stop = threading.Event(), threading.Event()
        original_stop = flight.stop
        def slow_stop():
            stopping.set()
            assert finish_stop.wait(5)
            return original_stop()
        flight.stop = slow_stop
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={}, background=True)
        await asyncio.to_thread(flight.entered.wait, 2)
        request = asyncio.create_task(manager.stop(OWNER, None))
        await asyncio.to_thread(stopping.wait, 2)
        request.cancel()
        with pytest.raises(asyncio.CancelledError):
            await request
        finish_stop.set()
        await manager._task
        assert manager.status(OWNER)["phase"] == "stopped"
        assert not flight.running and flight.released == 1
    asyncio.run(run())


@pytest.mark.parametrize("invalid", [[], 12, {"schema": "spacesim-mission/1", "mission_id": "../foreign"}])
def test_invalid_persisted_record_is_not_trusted(tmp_path, invalid):
    manager, flight, scenes, _, _ = setup(tmp_path)
    manager.root.mkdir()
    (manager.root / "latest.json").write_text(json.dumps(invalid), encoding="utf-8")
    restored = MissionRuntime(manager.root, scenes, flight, manager.telemetry, lambda: False)
    assert restored.status(OWNER)["phase"] == "idle"


def test_handoff_after_scene_launch_failure_is_retryable_without_repeating_flight(tmp_path, monkeypatch):
    async def run():
        manager, flight, scenes, _, state = setup(tmp_path)
        flight.proceed.set()
        await manager.start_flight(OWNER, scenario="team-in-orbit", options={})
        flight.running = False
        flight.report = {"basic_nominal_passed": True}
        handoff = {"accepted": True, "state": {"metadata": {"run_id": "source"}}}
        monkeypatch.setattr("simulation.orbital_rendezvous.handoff.load_handoff", lambda *args: handoff)
        def start(*args, **kwargs):
            state.update(phase="failed", active=False)
            return {"instance_id": "failed-scene"}
        scenes.start = start
        await manager.handoff(OWNER)
        identity = manager.status(OWNER)["mission_id"]
        assert manager.status(OWNER)["can_handoff"]
        await manager.handoff(OWNER)
        assert flight.calls == 1
        assert manager.status(OWNER)["mission_id"] == identity
    asyncio.run(run())
