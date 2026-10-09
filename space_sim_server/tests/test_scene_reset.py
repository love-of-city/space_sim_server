"""Reset transport/API tests without a native dynamics dependency."""
import asyncio
from pathlib import Path
from unittest.mock import AsyncMock, Mock

import pytest
from fastapi.testclient import TestClient
from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.models import AppliedAction, SimulationObservation
from space_arm_platform.protocol import read_async, write_async
from space_arm_platform.simulation_hub import SimulationHub

PROTOCOL = "space-arm-control/1"


def observation(generation="", session="initial"):
    return dict(protocol=PROTOCOL, type="observation", simulation_id="test", step_id="1",
                render_frame_id="0", sim_time_ns="0", wall_time_ns="1", applied_action_sequence="0",
                joint_position_rad=[0.] * 6, joint_velocity_rad_s=[0.] * 6,
                target_joint_position_rad=[0.] * 6,
                reset_generation=generation, render_session_id=session)


def action():
    return AppliedAction(server_sequence="1", server_time_ns="1", client_sequence="1", client_time_ns="1",
                         deadman=True, end_effector_linear_velocity_body_m_s=[.1, 0, 0],
                         end_effector_angular_velocity_body_rad_s=[0, 0, 0],
                         gripper_velocity_rad_s=0, input_source="keyboard")


async def connect(hub, generation="", capabilities=None):
    reader, writer = await asyncio.open_connection("127.0.0.1", hub.bound_port)
    await write_async(writer, dict(protocol=PROTOCOL, type="sim_hello", simulation_id="test",
                                  capabilities=capabilities if capabilities is not None else ["scene_reset"],
                                  reset_generation=generation))
    for _ in range(100):
        if hub.connected:
            break
        await asyncio.sleep(.001)
    return reader, writer


def test_reset_acknowledgement_is_a_new_generation_observation():
    async def scenario():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        reader, writer = await connect(hub)
        try:
            assert await hub.publish_action(action())
            assert (await read_async(reader))["reset_generation"] == ""
            request_id = hub.begin_reset()
            with pytest.raises(RuntimeError, match="正在重置"):
                hub.begin_reset()
            assert not await hub.publish_action(action())
            pending = asyncio.create_task(hub.finish_reset(request_id, timeout=1))
            assert await read_async(reader) == dict(protocol=PROTOCOL, type="reset", request_id=request_id)
            await write_async(writer, observation())  # old in-flight frame
            await asyncio.sleep(.01)
            assert hub.latest_observation is None and not pending.done()
            await write_async(writer, observation(request_id, "new-render-session"))
            result = await pending
            assert result["status"] == "completed"
            assert result["request_id"] == request_id
            assert not hub.resetting
            assert hub.latest_observation.reset_generation == request_id
            assert await hub.publish_action(action())
            assert (await read_async(reader))["reset_generation"] == request_id
            await write_async(writer, observation())
            await asyncio.sleep(.01)
            assert hub.latest_observation.reset_generation == request_id
        finally:
            writer.close()
            await hub.close()
    asyncio.run(scenario())


def test_timeout_keeps_controls_locked_until_late_ack_or_scene_stop():
    async def scenario():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        reader, writer = await connect(hub)
        try:
            request_id = hub.begin_reset()
            with pytest.raises(RuntimeError, match="尚未确认"):
                await hub.finish_reset(request_id, timeout=.01)
            assert (await read_async(reader))["request_id"] == request_id
            assert hub.resetting and not await hub.publish_action(action())
            await write_async(writer, observation(request_id, "late-session"))
            for _ in range(100):
                if not hub.resetting:
                    break
                await asyncio.sleep(.001)
            assert not hub.resetting
            assert hub.status()["reset_error"] is None
            request_id = hub.begin_reset()
            pending = asyncio.create_task(hub.finish_reset(request_id))
            await read_async(reader)
            hub.cancel_reset()
            assert (await pending)["status"] == "cancelled"
        finally:
            writer.close()
            await hub.close()
    asyncio.run(scenario())


def test_reset_requires_connected_capable_simulator():
    async def scenario():
        hub = SimulationHub()
        with pytest.raises(RuntimeError, match="未连接"):
            hub.begin_reset()
        await hub.start("127.0.0.1", 0)
        _, writer = await connect(hub, capabilities=[])
        try:
            with pytest.raises(RuntimeError, match="不支持"):
                hub.begin_reset()
        finally:
            writer.close()
            await hub.close()
    asyncio.run(scenario())


def test_api_reset_guards_and_control_revocation(tmp_path, monkeypatch):
    app = create_app(PlatformConfig(project_root=Path(__file__).resolve().parents[1],
                                   data_root=tmp_path / "episodes", simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        assert client.post('/api/scenes/reset').status_code == 401
        assert client.post('/api/auth/login', json={'username':'admin','password':'ChangeMe123!'}).status_code == 200
        monkeypatch.setattr(app.state.scenes, 'status', lambda: {'phase':'idle'})
        assert client.post('/api/scenes/reset').status_code == 409
        monkeypatch.setattr(app.state.scenes, 'status', lambda: {'phase':'running', 'instance':{}})
        assert client.post('/api/episodes/start', json={'camera_ids': []}).status_code == 200
        assert '采集' in client.post('/api/scenes/reset').json()['detail']
        assert client.post('/api/episodes/stop', json={}).status_code == 200
        assert '未连接' in client.post('/api/scenes/reset').json()['detail']
        monkeypatch.setattr(app.state.hub, 'begin_reset', lambda: 'request')
        finish = AsyncMock(return_value={'status':'completed', 'request_id':'request'})
        monkeypatch.setattr(app.state.hub, 'finish_reset', finish)
        app.state.sessions.active_operator = 'old-page'
        assert client.post('/api/scenes/reset').json()['status'] == 'completed'
        assert app.state.sessions.active_operator is None
        assert not app.state.safety.last_action.deadman
        finish.assert_awaited_once_with('request')
        app.state.hub._reset_request_id = 'busy'
        assert client.post('/api/episodes/start', json={}).status_code == 409
        assert client.post('/api/tasks/missing/start').status_code == 409
        app.state.hub.cancel_reset()


def test_non_owner_cannot_reset_scene(tmp_path, monkeypatch):
    app = create_app(PlatformConfig(project_root=Path(__file__).resolve().parents[1],
                                   data_root=tmp_path / "episodes", simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        client.post('/api/auth/login', json={'username':'admin','password':'ChangeMe123!'})
        client.post('/api/users/operators', json={'username':'operator1','password':'Operator123!'})
        client.post('/api/auth/logout')
        client.post('/api/auth/login', json={'username':'operator1','password':'Operator123!'})
        monkeypatch.setattr(app.state.scenes, 'status', lambda: {
            'phase':'running', 'instance': {'created_by': {'user_id':'someone-else'}}})
        assert client.post('/api/scenes/reset').status_code == 403


def test_capture_from_previous_render_session_is_not_recorded(tmp_path, monkeypatch):
    app = create_app(PlatformConfig(project_root=Path(__file__).resolve().parents[1],
                                   data_root=tmp_path / 'episodes', simulation_port=0, capture_port=0))
    with TestClient(app):
        record = Mock()
        monkeypatch.setattr(app.state.recorder, 'record_authoritative_capture', record)
        app.state.hub._latest_observation = SimulationObservation.model_validate(observation('reset-1', 'new'))
        callback = app.state.captures.on_authoritative_capture
        callback({'session_id': 'old'}, {})
        record.assert_not_called()
        callback({'session_id': 'new'}, {})
        record.assert_called_once_with({'session_id': 'new'}, {})
        app.state.hub._reset_request_id = 'busy'
        callback({'session_id': 'new'}, {})
        assert record.call_count == 1
        app.state.hub.cancel_reset()


def test_reset_reconnect_resends_only_the_same_request_not_old_actions():
    async def scenario():
        hub = SimulationHub()
        await hub.start('127.0.0.1', 0)
        reader, writer = await connect(hub)
        try:
            assert await hub.publish_action(action())
            await read_async(reader)
            request_id = hub.begin_reset()
            pending = asyncio.create_task(hub.finish_reset(request_id, timeout=2))
            await read_async(reader)
            writer.close()
            await writer.wait_closed()
            for _ in range(100):
                if not hub.connected:
                    break
                await asyncio.sleep(.001)
            assert not hub.connected
            reader, writer = await connect(hub, generation=request_id)
            resent = await asyncio.wait_for(read_async(reader), timeout=1)
            assert resent == dict(protocol=PROTOCOL, type='reset', request_id=request_id)
            await write_async(writer, observation(request_id, 'reconnected-session'))
            assert (await pending)['status'] == 'completed'
            assert hub._latest_action is None
        finally:
            writer.close()
            await hub.close()
    asyncio.run(scenario())
