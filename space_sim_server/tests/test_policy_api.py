"""Policy HTTP/control contracts with mocked runtime boundaries, never a closed loop."""
import asyncio
import uuid
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient
from pydantic import ValidationError

import space_arm_platform.app as app_module
from space_arm_platform.auth import SESSION_COOKIE
from space_arm_platform.policy_api import PolicyStart, PolicyStep
from space_arm_platform.simulation_hub import SimulationHub


TICK_NS = 20_000_000
INITIAL = dict(render_session_id="render-A", render_frame_id="10", sim_time_ns="200000000")
NEXT = dict(render_session_id="render-A", render_frame_id="11", sim_time_ns="220000000")
ACTION = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.75]
START = {"instruction": "Open the drawer", "task_name": "drawer", "reset": False}


def step_payload(observation=INITIAL, request_id="step-1", actions=None):
    return dict(request_id=uuid.uuid5(uuid.NAMESPACE_URL, request_id).hex, expected_observation=dict(observation),
                actions=deepcopy([ACTION] if actions is None else actions))


@pytest.fixture
def api(tmp_path, monkeypatch):
    """Real app/auth/route wiring; no server, writer worker, UE, or BSK startup."""
    hub = Mock(spec=SimulationHub)
    hub.connected = True
    hub.resetting = False
    hub.joint_policy_supported = True
    hub.capture_on_demand_supported = True
    hub.latest_observation = SimpleNamespace(gripper_opening_normalized=0.5, scripted_task=None)
    hub.publish_action = AsyncMock(return_value=True)
    hub.begin_reset = Mock(return_value="reset-token")
    hub.finish_reset = AsyncMock(return_value={"status": "completed"})
    hub.set_capture_episode = AsyncMock()

    async def wait_observation(_revision):
        await asyncio.Event().wait()

    hub.wait_for_observation = wait_observation
    cache = Mock()
    scenes = Mock()
    scenes.enabled = True
    scenes.status.return_value = {"phase": "idle", "active": False}
    monkeypatch.setattr(app_module, "SimulationHub", lambda: hub)
    monkeypatch.setattr(app_module, "PolicyObservationCache", lambda: cache)
    monkeypatch.setattr(app_module, "SceneRuntimeManager", lambda *args, **kwargs: scenes)
    project = tmp_path / "project"
    (project / "frontend").mkdir(parents=True)
    app = app_module.create_app(app_module.PlatformConfig(
        project_root=project, data_root=tmp_path / "data" / "episodes", simulation_port=0, capture_port=0))
    owner = app.state.auth.create_operator("policy-owner", "Policy-Owner-Password!")
    other = app.state.auth.create_operator("other-owner", "Other-Owner-Password!")
    admin = next(user for user in app.state.auth.list_users() if user["role"] == "admin")
    instance = dict(created_by=owner, runtime={"dataset_capture": True, "capture_rate_hz": 50})
    scenes.status.return_value = {"phase": "running", "instance": instance}
    fixture = SimpleNamespace(app=app, manager=app.state.policy_sessions, hub=hub, cache=cache,
        scenes=scenes, owner=owner, other=other, admin=admin, instance=instance,
        acquire_identity=dict(INITIAL), step_receipt=dict(NEXT, executed_steps=1), task_state={}, events=[])

    async def command(session_id, operation, **kwargs):
        fixture.events.append(("command", operation))
        receipt = fixture.step_receipt if operation == "step" else fixture.acquire_identity
        return dict(receipt, ok=True, session_id=session_id, operation=operation,
                    request_id=kwargs.get("request_id", operation + "-receipt"))

    fixture.reply = command
    hub.policy_command = AsyncMock(side_effect=command)

    def observed(key):
        return dict(render_session_id=key[0], render_frame_id=key[1], sim_time_ns=key[2],
                    state=[0.0] * 6 + [0.5], applied_action=list(ACTION), images={},
                    policy_task_state=deepcopy(fixture.task_state))

    cache.get.side_effect = observed
    cache.begin.side_effect = lambda lease: fixture.events.append(("capture_begin", lease))
    cache.end.side_effect = lambda: fixture.events.append(("capture_end",))
    client = TestClient(app)  # Intentionally omit lifespan: all runtime IO is mocked.
    fixture.client = client

    def authenticate(user):
        client.cookies.clear()
        token, _ = app.state.auth.create_session(user["user_id"])
        client.cookies.set(SESSION_COOKIE, token)

    fixture.authenticate = authenticate
    authenticate(owner)
    yield fixture
    client.close()
    app.state.collections.close()
    app.state.jobs.close()
    app.state.auth.close()


def start(api, **changes):
    response = api.client.post("/api/policy/sessions", json={**START, **changes})
    assert response.status_code == 200, response.text
    return response.json()


def test_all_policy_routes_require_authentication_before_mutation(api):
    api.client.cookies.clear()
    for method, path, payload in [
        ("GET", "/api/policy/status", None),
        ("POST", "/api/policy/sessions", START),
        ("POST", "/api/policy/sessions/unknown/step", step_payload()),
        ("POST", "/api/policy/sessions/unknown/reset", None),
        ("DELETE", "/api/policy/sessions/unknown", None),
    ]:
        response = api.client.request(method, path, json=payload)
        assert response.status_code == 401, (method, path, response.text)
    api.hub.policy_command.assert_not_awaited()
    api.cache.begin.assert_not_called()


def test_start_returns_absolute_seven_dimensional_contract_and_explicit_success_unavailable(api):
    result = start(api)
    assert result["action_format"] == "joint_position"
    assert result["action_semantics"] == "absolute_target"
    assert (result["action_dim"], result["period_ns"], result["action_steps"]) == (7, TICK_NS, 0)
    assert result["observation"]["instruction"] == START["instruction"]
    assert result["success"] is False and result["success_available"] is False and result["done"] is False
    assert result["recording_episode_id"] is None
    assert api.manager.status()["owner_user_id"] == api.owner["user_id"]
    api.hub.set_capture_episode.assert_awaited_once_with("policy-" + result["session_id"])
    assert api.app.state.recorder.episode_id is None
    neutral = api.hub.publish_action.await_args.args[0]
    assert neutral.deadman is False
    assert neutral.end_effector_linear_velocity_body_m_s == [0.0] * 3


@pytest.mark.parametrize("enabled", [False, True])
def test_service_gyro_reaches_start_step_and_reset_with_real_observation_cache(api, enabled):
    from simulation.gyroscope import Gyroscope
    from space_arm_platform.models import GyroObservation
    from space_arm_platform.policy_observation import PolicyObservationCache
    from test_policy_observation import complete, state

    sensor = Gyroscope(seed=7)
    readings = {str(stamp): sensor.sample(stamp, [.01, -.02, .03])
                for stamp in range(0, int(NEXT["sim_time_ns"]) + 1, TICK_NS)}
    cache = PolicyObservationCache(timeout=.1)
    api.manager.cache = cache

    async def command(session_id, operation, **kwargs):
        receipt = await api.reply(session_id, operation, **kwargs)
        if operation in {"acquire", "step"}:
            observation = state(lease="policy-" + session_id, session=receipt["render_session_id"],
                                frame=receipt["render_frame_id"], stamp=receipt["sim_time_ns"])
            if enabled:
                observation.service_gyro = GyroObservation.model_validate(readings[receipt["sim_time_ns"]])
            complete(cache, observation)
        return receipt

    api.hub.policy_command.side_effect = command
    initial = start(api)
    prefix = "/api/policy/sessions/" + initial["session_id"]
    stepped = api.client.post(prefix + "/step", json=step_payload())
    assert stepped.status_code == 200, stepped.text
    reset = api.client.post(prefix + "/reset")
    assert reset.status_code == 200, reset.text
    for response in (initial, stepped.json(), reset.json()):
        observation = response["observation"]
        expected = readings[observation["sim_time_ns"]] if enabled else None
        assert observation["service_gyro"] == expected


@pytest.mark.parametrize("scene_kind,role,expected", [
    ("own", "owner", 200), ("foreign", "owner", 403), ("foreign", "admin", 200),
    ("external", "owner", 403), ("external", "admin", 200),
])
def test_scenario_ownership_and_admin_takeover(api, scene_kind, role, expected):
    if scene_kind == "foreign":
        api.instance["created_by"] = api.other
    elif scene_kind == "external":
        api.scenes.status.return_value["instance"] = None
    api.authenticate(getattr(api, role))
    response = api.client.post("/api/policy/sessions", json=START)
    assert response.status_code == expected, response.text
    if expected != 200:
        api.hub.policy_command.assert_not_awaited()
        api.hub.publish_action.assert_not_awaited()
        assert not api.manager.active


@pytest.mark.parametrize("blocker", ["disconnected", "unsupported", "no_state", "uncalibrated", "scripted", "capture_disabled", "wrong_rate", "recording", "capture_busy"])
def test_start_preconditions_do_not_take_control_or_enable_capture(api, monkeypatch, blocker):
    if blocker == "disconnected":
        api.hub.connected = False
    elif blocker == "unsupported":
        api.hub.joint_policy_supported = False
    elif blocker == "no_state":
        api.hub.latest_observation = None
    elif blocker == "uncalibrated":
        api.hub.latest_observation.gripper_opening_normalized = None
    elif blocker == "scripted":
        api.hub.latest_observation.scripted_task = {"task": "drawer"}
    elif blocker == "capture_disabled":
        api.instance["runtime"]["dataset_capture"] = False
    elif blocker == "wrong_rate":
        api.instance["runtime"]["capture_rate_hz"] = 25
    elif blocker == "recording":
        monkeypatch.setattr(api.manager, "recorder", SimpleNamespace(episode_id="existing-recording"))
    else:
        monkeypatch.setattr(type(api.app.state.capture_lifecycle), "busy", property(lambda self: True))
    response = api.client.post("/api/policy/sessions", json=START)
    assert response.status_code == 409, response.text
    assert not api.manager.active
    api.hub.publish_action.assert_not_awaited()
    api.hub.policy_command.assert_not_awaited()
    api.cache.begin.assert_not_called()


def test_only_owner_or_admin_can_step_reset_and_close_a_session(api):
    result = start(api)
    prefix = "/api/policy/sessions/" + result["session_id"]
    api.hub.policy_command.reset_mock()
    api.authenticate(api.other)
    for method, path, payload in [("POST", prefix + "/step", step_payload()),
                                  ("POST", prefix + "/reset", None), ("DELETE", prefix, None)]:
        assert api.client.request(method, path, json=payload).status_code == 403
    api.hub.policy_command.assert_not_awaited()
    assert api.manager.active
    api.authenticate(api.admin)
    assert api.client.delete(prefix).status_code == 200
    assert not api.manager.active
    assert api.client.delete(prefix).status_code == 200  # Idempotent receipt after confirmed release.


@pytest.mark.parametrize("transition", ["starting", "handoff", "stopping", "stop_requested"])
def test_policy_cannot_acquire_during_a_mission_transition(api, transition):
    missions = api.app.state.missions
    if transition == "stop_requested":
        missions._stop_requested = True
    else:
        missions._operation = transition
    response = api.client.post("/api/policy/sessions", json=START)
    assert response.status_code == 409
    assert "任务正在切换阶段" in response.json()["detail"]
    assert not api.manager.active
    api.hub.begin_reset.assert_not_called()
    api.hub.policy_command.assert_not_awaited()


def test_policy_recordings_remain_linked_to_the_current_mission(api, monkeypatch):
    missions = api.app.state.missions
    missions._new("teleop", api.owner)

    async def begin_recording(request, *, before_enable):
        before_enable("mission-policy-episode")
        return {"episode_id": "mission-policy-episode"}

    monkeypatch.setattr(api.manager.capture_lifecycle, "start", AsyncMock(side_effect=begin_recording))
    result = start(api, record=True)
    assert result["recording_episode_id"] == "mission-policy-episode"
    import json
    saved = json.loads((missions.root / "latest.json").read_text(encoding="utf-8"))
    assert saved["episodes"] == ["mission-policy-episode"]


def test_policy_excludes_recording_scene_reset_start_and_teleop_in_the_real_app(api):
    result = start(api)
    for path, payload in [
        ("/api/policy/sessions", START),
        ("/api/episodes/start", {"instruction": "competing recording", "camera_ids": []}),
        ("/api/episodes/stop", {"outcome": "unknown"}),
        ("/api/scenes/start", {"template_id": "sarm-task-box-20261012"}),
        ("/api/scenes/reset", None), ("/api/tasks/missing-task/start", None),
        ("/api/mission/start", {"mode": "teleop"}),
        ("/api/mission/start", {"mode": "team-in-orbit"}),
        ("/api/mission/handoff", None),
        ("/api/rendezvous/start", {"scenario": "team-in-orbit"}),
        ("/api/rendezvous/handoff", None),
    ]:
        response = api.client.post(path, json=payload)
        assert response.status_code == 409, (path, response.text)
        assert "独占控制" in response.json()["detail"], (path, response.text)
    with api.client.websocket_connect("/ws/operator") as websocket:
        # A connection may receive a revocation before its initial session packet.
        for _ in range(3):
            message = websocket.receive_json()
            if message["type"] == "session":
                break
        assert message["type"] == "session" and message["control_granted"] is False
        websocket.send_json({"type": "activate"})
        assert websocket.receive_json() == {"type": "action_rejected", "reason": "policy_control_active"}
    api.scenes.start.assert_not_called()
    assert api.manager.status()["session_id"] == result["session_id"]


@pytest.mark.parametrize("actions", [[], [[0.0] * 6], [[0.0] * 8], [ACTION] * 101,
    [["NaN"] + ACTION[1:]], [["Infinity"] + ACTION[1:]], [["-Infinity"] + ACTION[1:]],
    [ACTION[:6] + [-0.01]], [ACTION[:6] + [1.01]]])
def test_invalid_action_payload_never_reaches_the_runtime(api, actions):
    result = start(api)
    api.hub.policy_command.reset_mock()
    response = api.client.post("/api/policy/sessions/" + result["session_id"] + "/step",
                               json=step_payload(actions=actions))
    assert response.status_code == 422, response.text
    api.hub.policy_command.assert_not_awaited()
    assert api.manager.status()["action_steps"] == 0


@pytest.mark.parametrize("nonfinite", [float("nan"), float("inf"), -float("inf")])
def test_step_model_rejects_nonfinite_python_values(nonfinite):
    for component in (0, 6):
        action = list(ACTION)
        action[component] = nonfinite
        with pytest.raises(ValidationError):
            PolicyStep.model_validate(step_payload(actions=[action]))


@pytest.mark.parametrize("opening", [0.0, 1.0])
def test_gripper_endpoints_and_joint_targets_are_forwarded_without_remapping(api, opening):
    result = start(api)
    action = ACTION[:6] + [opening]
    response = api.client.post("/api/policy/sessions/" + result["session_id"] + "/step",
                               json=step_payload(actions=[action]))
    assert response.status_code == 200, response.text
    command = api.hub.policy_command.await_args
    assert command.args == (result["session_id"], "step")
    assert command.kwargs["actions"] == [action]
    assert command.kwargs["period_ns"] == TICK_NS
    assert response.json()["action_steps"] == 1


@pytest.mark.parametrize("field,wrong", [("render_session_id", "old-render"), ("render_frame_id", "9"), ("sim_time_ns", "180000000")])
def test_step_rejects_any_stale_observation_identity_component(api, field, wrong):
    result = start(api)
    api.hub.policy_command.reset_mock()
    stale = {**INITIAL, field: wrong}
    response = api.client.post("/api/policy/sessions/" + result["session_id"] + "/step", json=step_payload(stale))
    assert response.status_code == 409
    api.hub.policy_command.assert_not_awaited()


def test_successful_step_cannot_be_replayed_even_with_the_new_observation(api):
    result = start(api)
    path = "/api/policy/sessions/" + result["session_id"] + "/step"
    first = api.client.post(path, json=step_payload())
    assert first.status_code == 200
    api.hub.policy_command.reset_mock()
    replay = api.client.post(path, json=step_payload(NEXT))
    assert replay.status_code == 409
    api.hub.policy_command.assert_not_awaited()
    assert api.manager.status()["action_steps"] == 1


@pytest.mark.parametrize("bad_receipt", [
    {"executed_steps": 0}, {"executed_steps": True}, {"executed_steps": "1"},
    {"sim_time_ns": "220000001"}, {"sim_time_ns": "180000000"}, {"render_session_id": "render-B"},
])
def test_bad_execution_receipt_invalidates_session_until_reset_then_recovers(api, bad_receipt):
    result = start(api)
    prefix = "/api/policy/sessions/" + result["session_id"]
    api.step_receipt.update(bad_receipt)
    assert api.client.post(prefix + "/step", json=step_payload()).status_code == 409
    assert api.manager.status()["failed"]
    assert api.manager.status()["action_steps"] == 0
    api.hub.policy_command.reset_mock()
    assert api.client.post(prefix + "/step", json=step_payload(request_id="another-step")).status_code == 409
    api.hub.policy_command.assert_not_awaited()
    api.acquire_identity = dict(render_session_id="render-B", render_frame_id="0", sim_time_ns="0")
    reset = api.client.post(prefix + "/reset")
    assert reset.status_code == 200, reset.text
    assert reset.json()["action_steps"] == 0
    assert api.manager.status()["failed"] is None
    api.step_receipt = dict(render_session_id="render-B", render_frame_id="1", sim_time_ns="20000000", executed_steps=1)
    api.hub.policy_command.reset_mock()
    # IDs remain consumed after uncertainty; a reset must not replay an old action.
    assert api.client.post(prefix + "/step", json=step_payload(api.acquire_identity)).status_code == 409
    api.hub.policy_command.assert_not_awaited()
    recovered = api.client.post(prefix + "/step", json=step_payload(api.acquire_identity, request_id="after-reset"))
    assert recovered.status_code == 200, recovered.text
    assert recovered.json()["action_steps"] == 1


def test_missing_paired_images_after_execution_invalidates_and_does_not_reexecute(api):
    result = start(api)
    api.cache.get.side_effect = RuntimeError("timed out pairing policy RGB and state")
    path = "/api/policy/sessions/" + result["session_id"] + "/step"
    response = api.client.post(path, json=step_payload())
    assert response.status_code == 409
    assert api.manager.status()["failed"]
    api.hub.policy_command.reset_mock()
    assert api.client.post(path, json=step_payload(request_id="retry")).status_code == 409
    api.hub.policy_command.assert_not_awaited()


@pytest.mark.parametrize("task_state,success,available", [
    ({}, False, False),
    ({"task_name": "other-task", "success_available": True, "success": True}, False, False),
    ({"task_name": "drawer", "success_available": False, "success": True}, False, False),
    ({"task_name": "drawer", "success_available": True, "success": False}, False, True),
    ({"task_name": "drawer", "success_available": True, "success": True}, True, True),
])
def test_only_matching_simulator_task_telemetry_can_assert_success(api, task_state, success, available):
    api.task_state = task_state
    initial = start(api)
    assert initial["success"] is False and initial["done"] is False
    # Acquiring a new task must not inherit success from a prior session.
    response = api.client.post("/api/policy/sessions/" + initial["session_id"] + "/step",
                               json=step_payload())
    assert response.status_code == 200, response.text
    result = response.json()
    assert result["success"] is success
    assert result["success_available"] is available
    assert result["done"] is success
    assert result["observation"]["task_success"] is success


def test_action_budget_stops_execution_without_claiming_task_success(api):
    result = start(api, max_steps=1)
    path = "/api/policy/sessions/" + result["session_id"] + "/step"
    api.hub.policy_command.reset_mock()
    assert api.client.post(path, json=step_payload(actions=[ACTION, ACTION])).status_code == 409
    api.hub.policy_command.assert_not_awaited()
    response = api.client.post(path, json=step_payload())
    assert response.status_code == 200
    assert response.json()["done"] is True and response.json()["success"] is False
    api.hub.policy_command.reset_mock()
    assert api.client.post(path, json=step_payload(NEXT, request_id="over-budget")).status_code == 409
    api.hub.policy_command.assert_not_awaited()


def test_failed_acquire_disables_capture_and_frees_reservation_for_next_start(api):
    async def acquire_fails(session_id, operation, **kwargs):
        if operation == "acquire":
            raise RuntimeError("runtime rejected acquisition")
        return await api.reply(session_id, operation, **kwargs)

    api.hub.policy_command.side_effect = acquire_fails
    failed = api.client.post("/api/policy/sessions", json=START)
    assert failed.status_code == 409
    assert "rejected acquisition" in failed.json()["detail"]
    assert not api.manager.active
    assert [call.args[1] for call in api.hub.policy_command.await_args_list] == ["acquire", "release"]
    assert api.hub.set_capture_episode.await_args.args == ("",)
    api.cache.end.assert_called()
    api.hub.policy_command.side_effect = api.reply
    assert start(api)["action_steps"] == 0


def test_recorded_acquire_failure_aborts_recording_with_matching_cache_lease(api, monkeypatch):
    async def begin_recording(request, *, before_enable):
        assert request.fps == 50
        assert request.camera_ids == ["teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam"]
        assert request.operator_user_id == api.owner["user_id"]
        before_enable("recorded-policy-episode")
        return {"episode_id": "recorded-policy-episode"}

    async def acquire_fails(session_id, operation, **kwargs):
        if operation == "acquire":
            raise RuntimeError("acquisition failed")
        return await api.reply(session_id, operation, **kwargs)

    lifecycle = api.app.state.capture_lifecycle
    monkeypatch.setattr(lifecycle, "start", AsyncMock(side_effect=begin_recording))
    monkeypatch.setattr(lifecycle, "stop", AsyncMock(return_value={"outcome": "aborted"}))
    api.hub.policy_command.side_effect = acquire_fails
    response = api.client.post("/api/policy/sessions", json={**START, "record": True})
    assert response.status_code == 409, response.text
    api.cache.begin.assert_called_once_with("recorded-policy-episode")
    lifecycle.stop.assert_awaited_once()
    assert lifecycle.stop.await_args.args[0].outcome == "aborted"
    assert not api.manager.active


def test_reservation_blocks_competing_start_before_first_acquire_completes(api):
    async def scenario():
        entered, continue_acquire = asyncio.Event(), asyncio.Event()

        async def command(session_id, operation, **kwargs):
            if operation == "acquire":
                entered.set()
                await continue_acquire.wait()
            return await api.reply(session_id, operation, **kwargs)

        api.hub.policy_command.side_effect = command
        pending = asyncio.create_task(api.manager.start(PolicyStart(**START), api.owner))
        try:
            await asyncio.wait_for(entered.wait(), 1)
            assert api.manager.active
            with pytest.raises(HTTPException) as rejected:
                await api.manager.start(PolicyStart(**START), api.owner)
            assert rejected.value.status_code == 409
            with pytest.raises(HTTPException):
                api.manager.require_free()
        finally:
            continue_acquire.set()
            await asyncio.wait_for(pending, 1)
        assert [call.args[1] for call in api.hub.policy_command.await_args_list] == ["acquire"]
        await api.manager.shutdown()

    asyncio.run(scenario())


def test_close_waits_for_inflight_step_and_release_before_reopening_teleop(api):
    async def scenario():
        started = await api.manager.start(PolicyStart(**START), api.owner)
        identifier = started["session_id"]
        step_entered, finish_step = asyncio.Event(), asyncio.Event()
        release_entered, finish_release = asyncio.Event(), asyncio.Event()

        async def command(session_id, operation, **kwargs):
            if operation == "step":
                step_entered.set()
                await finish_step.wait()
            elif operation == "release":
                release_entered.set()
                await finish_release.wait()
            return await api.reply(session_id, operation, **kwargs)

        api.hub.policy_command.side_effect = command
        api.cache.end.reset_mock()
        stepping = asyncio.create_task(api.manager.step(identifier, PolicyStep(**step_payload()), api.owner))
        closing = None
        try:
            await asyncio.wait_for(step_entered.wait(), 1)
            for operation in (
                api.manager.step(identifier, PolicyStep(**step_payload(request_id="competing")), api.owner),
                api.manager.reset(identifier, api.owner),
            ):
                with pytest.raises(HTTPException) as rejected:
                    await operation
                assert rejected.value.status_code == 409
            closing = asyncio.create_task(api.manager.close(identifier, api.owner))
            await asyncio.sleep(0)
            assert not closing.done()
            assert api.manager.active
            api.cache.end.assert_not_called()
            finish_step.set()
            assert (await asyncio.wait_for(stepping, 1))["action_steps"] == 1
            await asyncio.wait_for(release_entered.wait(), 1)
            assert api.manager.active
            with pytest.raises(HTTPException):
                api.manager.require_free()
            api.cache.end.assert_not_called()
            finish_release.set()
            assert (await asyncio.wait_for(closing, 1))["status"] == "closed"
            api.manager.require_free()
            assert not api.manager.active
        finally:
            finish_step.set()
            finish_release.set()
            await asyncio.gather(*(task for task in (stepping, closing) if task is not None), return_exceptions=True)

    asyncio.run(scenario())


def test_cancelling_close_during_release_does_not_prematurely_release_teleop(api):
    async def scenario():
        started = await api.manager.start(PolicyStart(**START), api.owner)
        entered, finish_release = asyncio.Event(), asyncio.Event()

        async def command(session_id, operation, **kwargs):
            if operation == "release":
                entered.set()
                await finish_release.wait()
            return await api.reply(session_id, operation, **kwargs)

        api.hub.policy_command.side_effect = command
        closing = asyncio.create_task(api.manager.close(started["session_id"], api.owner))
        try:
            await asyncio.wait_for(entered.wait(), 1)
            closing.cancel()
            await asyncio.sleep(0)
            assert api.manager.active, "HTTP cancellation must not expose teleop while runtime release is unconfirmed"
            with pytest.raises(HTTPException):
                api.manager.require_free()
        finally:
            finish_release.set()
            await asyncio.gather(closing, return_exceptions=True)
            await api.manager.shutdown()

    asyncio.run(scenario())


def test_unacknowledged_release_keeps_control_reserved_until_recovery(api):
    result = start(api)

    async def uncertain_release(session_id, operation, **kwargs):
        if operation == "release":
            raise RuntimeError("release receipt lost: runtime may still hold joint targets")
        return await api.reply(session_id, operation, **kwargs)

    api.hub.policy_command.side_effect = uncertain_release
    api.hub.set_capture_episode.side_effect = RuntimeError("capture OFF not acknowledged")
    response = api.client.delete("/api/policy/sessions/" + result["session_id"])
    assert response.status_code in (200, 409)
    assert api.manager.active, "A lost release AND OFF acknowledgement cannot make teleop safe"
    with pytest.raises(HTTPException):
        api.manager.require_free()
    api.hub.policy_command.side_effect = api.reply
    api.hub.set_capture_episode.side_effect = None
    assert api.client.delete("/api/policy/sessions/" + result["session_id"]).status_code == 200
    assert not api.manager.active



def test_action_chunk_advances_exact_intervals_and_returns_only_its_final_pair(api):
    result = start(api, max_steps=3)
    final = dict(render_session_id="render-A", render_frame_id="12", sim_time_ns="240000000")
    api.step_receipt = dict(final, executed_steps=2)
    actions = [ACTION, [-0.1, 0.2, -0.3, 0.4, -0.5, 0.6, 0.0]]
    response = api.client.post("/api/policy/sessions/" + result["session_id"] + "/step",
                               json=step_payload(actions=actions))
    assert response.status_code == 200, response.text
    assert response.json()["action_steps"] == 2
    assert response.json()["done"] is False
    assert {field: response.json()["observation"][field] for field in final} == final
    assert api.hub.policy_command.await_args.kwargs["actions"] == actions
    api.cache.get.assert_called_with(("render-A", "12", "240000000"))


def test_cancelled_step_stays_failed_and_cannot_be_replayed(api):
    async def scenario():
        result = await api.manager.start(PolicyStart(**START), api.owner)
        entered = asyncio.Event()

        async def command(session_id, operation, **kwargs):
            if operation == "step":
                entered.set()
                await asyncio.Event().wait()
            return await api.reply(session_id, operation, **kwargs)

        api.hub.policy_command.side_effect = command
        stepping = asyncio.create_task(api.manager.step(result["session_id"], PolicyStep(**step_payload()), api.owner))
        try:
            await asyncio.wait_for(entered.wait(), 1)
            stepping.cancel()
            with pytest.raises(asyncio.CancelledError):
                await stepping
            assert api.manager.active and api.manager.status()["failed"]
            assert api.manager.status()["action_steps"] == 0
            api.hub.policy_command.reset_mock()
            with pytest.raises(HTTPException) as rejected:
                await api.manager.step(result["session_id"], PolicyStep(**step_payload(request_id="retry")), api.owner)
            assert rejected.value.status_code == 409
            api.hub.policy_command.assert_not_awaited()
        finally:
            stepping.cancel()
            await asyncio.gather(stepping, return_exceptions=True)
            await api.manager.shutdown()

    asyncio.run(scenario())


def test_cancelled_acquire_keeps_reservation_until_release_and_capture_off_finish(api):
    async def scenario():
        acquire_entered, release_entered, finish_release = asyncio.Event(), asyncio.Event(), asyncio.Event()

        async def command(session_id, operation, **kwargs):
            if operation == "acquire":
                acquire_entered.set()
                await asyncio.Event().wait()
            elif operation == "release":
                release_entered.set()
                await finish_release.wait()
            return await api.reply(session_id, operation, **kwargs)

        api.hub.policy_command.side_effect = command
        pending = asyncio.create_task(api.manager.start(PolicyStart(**START), api.owner))
        try:
            await asyncio.wait_for(acquire_entered.wait(), 1)
            pending.cancel()
            await asyncio.wait_for(release_entered.wait(), 1)
            assert api.manager.active
            with pytest.raises(HTTPException):
                api.manager.require_free()
            api.cache.end.assert_not_called()
            finish_release.set()
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(pending, 1)
            assert not api.manager.active
            assert api.hub.set_capture_episode.await_args.args == ("",)
        finally:
            finish_release.set()
            await asyncio.gather(pending, return_exceptions=True)

    asyncio.run(scenario())


def test_non_uuid_request_is_rejected_before_runtime(api):
    session = start(api)
    api.hub.policy_command.reset_mock()
    payload = step_payload()
    payload["request_id"] = "not-a-uuid"
    response = api.client.post("/api/policy/sessions/" + session["session_id"] + "/step", json=payload)
    assert response.status_code == 422
    api.hub.policy_command.assert_not_awaited()


def test_definitively_refused_acquire_does_not_release_an_unowned_runtime(api):
    from space_arm_platform.simulation_hub import PolicyCommandRejected

    async def reject(session_id, operation, **kwargs):
        assert operation == "acquire"
        raise PolicyCommandRejected({"ok": False, "error": "unsupported scene"})

    api.hub.policy_command.side_effect = reject
    response = api.client.post("/api/policy/sessions", json=START)
    assert response.status_code == 409
    assert not api.manager.active
    assert [call.args[1] for call in api.hub.policy_command.await_args_list] == ["acquire"]
    assert api.hub.set_capture_episode.await_args.args == ("",)


def test_capture_off_retry_does_not_release_already_released_runtime_twice(api):
    result = start(api)
    api.hub.set_capture_episode.side_effect = RuntimeError("OFF acknowledgement lost")
    response = api.client.delete("/api/policy/sessions/" + result["session_id"])
    assert response.status_code == 409 and api.manager.active
    api.hub.policy_command.reset_mock()
    api.hub.set_capture_episode.side_effect = None
    response = api.client.delete("/api/policy/sessions/" + result["session_id"])
    assert response.status_code == 200 and not api.manager.active
    api.hub.policy_command.assert_not_awaited()


def test_concurrent_close_and_late_retry_do_not_stop_new_session(api):
    async def scenario():
        first = await api.manager.start(PolicyStart(**START), api.owner)
        entered, finish_release = asyncio.Event(), asyncio.Event()

        async def command(session_id, operation, **kwargs):
            if operation == "release":
                entered.set()
                await finish_release.wait()
            return await api.reply(session_id, operation, **kwargs)

        api.hub.policy_command.side_effect = command
        api.hub.set_capture_episode.reset_mock()
        first_close = asyncio.create_task(api.manager.close(first["session_id"], api.owner))
        await asyncio.wait_for(entered.wait(), 1)
        second_close = asyncio.create_task(api.manager.close(first["session_id"], api.owner))
        await asyncio.sleep(0)
        finish_release.set()
        first_result, second_result = await asyncio.wait_for(asyncio.gather(first_close, second_close), 1)
        assert first_result == second_result
        api.hub.set_capture_episode.assert_awaited_once_with("", timeout=5)
        assert sum(c.args[1] == "release" for c in api.hub.policy_command.await_args_list) == 1

        new_session = await api.manager.start(PolicyStart(**START), api.owner)
        api.hub.set_capture_episode.reset_mock()
        api.hub.policy_command.reset_mock()
        api.cache.end.reset_mock()
        assert await api.manager.close(first["session_id"], api.owner) == first_result
        assert api.manager.status()["session_id"] == new_session["session_id"]
        api.hub.set_capture_episode.assert_not_awaited()
        api.hub.policy_command.assert_not_awaited()
        api.cache.end.assert_not_called()
        with pytest.raises(HTTPException) as denied:
            await api.manager.close(first["session_id"], api.other)
        assert denied.value.status_code == 403
        await api.manager.shutdown()

    asyncio.run(scenario())


def test_reset_does_not_hide_failed_recording_finalization(api, monkeypatch):
    async def begin_recording(request, *, before_enable):
        before_enable("recorded-policy-episode")
        return {"episode_id": "recorded-policy-episode"}

    lifecycle = api.manager.capture_lifecycle
    monkeypatch.setattr(lifecycle, "start", AsyncMock(side_effect=begin_recording))
    monkeypatch.setattr(lifecycle, "stop", AsyncMock(return_value={"dataset_status": "failed"}))
    result = start(api, record=True)
    lifecycle.start.reset_mock()
    api.hub.finish_reset.reset_mock()
    prefix = "/api/policy/sessions/" + result["session_id"]
    response = api.client.post(prefix + "/reset")
    assert response.status_code == 409
    assert "failed finalization" in response.json()["detail"]
    assert api.manager.status()["failed"]
    lifecycle.start.assert_not_awaited()
    api.hub.finish_reset.assert_not_awaited()
    assert api.client.delete(prefix).status_code == 200
