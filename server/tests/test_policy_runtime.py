"""Policy admission, physical-reference ownership and socket delivery regressions."""
import socket
import time
from types import SimpleNamespace
import uuid

import numpy as np
import pytest

from simulation.control_client import SimulationControlClient
from simulation.policy_control import (
    DrawerPolicyState, JointPolicyController, PolicyCommandGate, POLICY_PERIOD_NS,
)
from space_arm_platform.protocol import CONTROL_PROTOCOL, FramedSocketReader, encode_packet


def command(operation="acquire", **kwargs):
    packet = dict(protocol=CONTROL_PROTOCOL, type="joint_policy_command",
                  request_id=str(uuid.uuid4()), session_id="policy-session",
                  reset_generation="", operation=operation)
    if operation == "step":
        packet.update(period_ns=POLICY_PERIOD_NS, actions=[[0.] * 6 + [.5]])
    return {**packet, **kwargs}


def observation(frame, *, episode="", request="", generation=""):
    return dict(render_session_id="render-session", render_frame_id=str(frame),
                sim_time_ns=str(frame * POLICY_PERIOD_NS), reset_generation=generation,
                capture_episode_id=episode, capture_request_id=request)


class Clock:
    value = 100.
    def __call__(self):
        return self.value


class Targets:
    def __init__(self):
        self.joint_min = np.array([-1.] * 6 + [0., -1.])
        self.joint_max = np.array([1.] * 6 + [1., 0.])
        self.preparation = SimpleNamespace(ready=True)
        self.held = []
        self.calls = []
        self.policy_active = False

    def set_policy_control(self, enabled, now):
        self.policy_active = enabled
        self.held.append((enabled, now))

    def apply_policy_action(self, action):
        self.calls.append(list(action))
        # Distinguish actual bounded references from requested absolute targets.
        return [v / 2 for v in action[:6]] + [action[6]]


def setup_controller(*, supported=True, max_request_ids=65536):
    clock = Clock()
    gate = PolicyCommandGate(clock=clock, timeout_s=5, max_request_ids=max_request_ids)
    target = Targets()
    capture = dict(capture_episode_id="", capture_request_id="")
    dropped, published = [], []
    controller = JointPolicyController(gate, target, supported=supported,
        discard_teleop=lambda: dropped.append(True), capture_state=lambda: dict(capture),
        publish_reference=published.append, available_tasks=("drawer_cycle",))
    return SimpleNamespace(gate=gate, target=target, clock=clock, control=controller,
                           capture=capture, dropped=dropped, published=published)


def acquire(env):
    env.control.observe(observation(1))
    env.gate.submit(command())
    env.control.service()
    result, = env.gate.take_results()
    assert result["ok"]
    return result


def test_acquire_uses_completed_frame_and_matching_capture_identity():
    env = setup_controller()
    env.capture.update(capture_episode_id="policy-rgb", capture_request_id="capture-request")
    request = command(task_name="drawer_cycle")
    env.gate.submit(request)
    for frame in (None, observation(0), observation(1)):
        if frame:
            env.control.observe(frame)
        env.control.service()
        assert not env.gate.take_results()
        assert not env.control.paused
    env.control.observe(observation(2, episode="policy-rgb", request="capture-request"))
    env.control.service()
    result, = env.gate.take_results()
    assert result["ok"] and result["executed_steps"] == 0
    assert result["sim_time_ns"] == "40000000" and result["render_frame_id"] == "2"
    assert result["observation"]["render_session_id"] == "render-session"
    assert result["request_id"] == request["request_id"]
    assert env.control.paused and not env.control.stepping and not env.target.calls
    assert env.published == [40_000_000] and env.dropped
    assert env.control.task_name == "drawer_cycle"


def test_chunk_advances_exactly_one_render_interval_per_action_and_reports_actual_targets():
    env = setup_controller()
    acquire(env)
    packet = command("step", actions=[[.2] * 6 + [.5], [.4] * 6 + [1.], [-.2] * 6 + [0.]])
    env.gate.submit(packet)
    env.control.service()
    for index, frame in enumerate((2, 3, 4)):
        assert env.control.prepare_step() == frame * POLICY_PERIOD_NS
        metadata = env.control.execution_metadata()
        assert metadata["executed_steps"] == index + 1
        assert metadata["request_id"] == packet["request_id"]
        assert metadata["applied_action"] == [v / 2 for v in packet["actions"][index][:6]] + [packet["actions"][index][6]]
        env.control.complete_step([observation(frame)])
        if frame < 4:
            assert env.gate.take_results() == []
    result, = env.gate.take_results()
    assert result["ok"] and result["executed_steps"] == 3
    assert result["applied_actions"] == [[.1] * 6 + [.5], [.2] * 6 + [1.], [-.1] * 6 + [0.]]
    assert result["sim_time_ns"] == "80000000"
    assert env.control.paused and not env.control.stepping
    assert env.control.prepare_step() is None
    assert len(env.target.calls) == 3


@pytest.mark.parametrize("change", [
    {"actions": []}, {"actions": [[0.] * 7] * 101}, {"actions": [[0.] * 6]},
    {"actions": [[0.] * 6 + [1.01]]}, {"actions": [[0.] * 6 + [-.01]]},
    {"actions": [[float('nan')] + [0.] * 6]}, {"actions": [[float('inf')] + [0.] * 6]},
    {"actions": [[True] + [0.] * 6]}, {"actions": [[10 ** 1000] + [0.] * 6]},
    {"period_ns": 20_000_001}, {"period_ns": 20_000_000.}, {"request_id": "not-a-uuid"},
    {"reset_generation": "old"}, {"session_id": "wrong-owner"}, {"operation": []},
])
def test_rejected_input_never_reaches_target(change):
    env = setup_controller()
    acquire(env)
    env.gate.submit({**command("step"), **change})
    env.control.service()
    result, = env.gate.take_results()
    assert not result["ok"] and result["error"] and result["executed_steps"] == 0
    assert not env.target.calls and not env.control.stepping


def test_validate_entire_chunk_before_any_joint_reference_changes():
    env = setup_controller()
    acquire(env)
    env.gate.submit(command("step", actions=[[.2] * 6 + [.5], [1.1] * 6 + [.5]]))
    env.control.service()
    assert "joint limits" in env.gate.take_results()[0]["error"]
    assert not env.target.calls


def test_concurrent_and_canonical_uuid_replay_cannot_duplicate_step():
    env = setup_controller()
    acquire(env)
    packet = command("step")
    env.gate.submit(packet)
    env.control.service()
    env.gate.submit(command("step"))
    assert "in progress" in env.gate.take_results()[0]["error"]
    env.gate.submit({**packet, "request_id": "{" + packet["request_id"].upper() + "}"})
    assert "already received" in env.gate.take_results()[0]["error"]
    env.control.prepare_step()
    env.control.complete_step([observation(2)])
    assert env.gate.take_results()[0]["ok"]
    env.gate.submit(packet)
    assert "already received" in env.gate.take_results()[0]["error"]
    assert env.control.prepare_step() is None and len(env.target.calls) == 1


def test_dedupe_storage_does_not_evict_and_release_remains_possible_at_capacity():
    env = setup_controller(max_request_ids=2)
    first = command()
    env.control.observe(observation(1))
    env.gate.submit(first)
    env.control.service()
    env.gate.take_results()
    env.gate.submit(command("step", session_id="wrong-owner"))
    env.gate.take_results()
    env.gate.submit(command("step"))
    assert "history full" in env.gate.take_results()[0]["error"]
    env.gate.submit(command("release"))
    env.control.service()
    assert env.gate.take_results()[0]["ok"] and not env.control.paused
    env.gate.submit(first)
    assert "already received" in env.gate.take_results()[0]["error"]
    env.gate.submit(command())
    assert "history full" in env.gate.take_results()[0]["error"]
    assert len(env.gate._seen) == 2


@pytest.mark.parametrize("reason", ["control disconnected", "explicit STOP", "scene reset"])
def test_interruption_during_interval_accounts_for_completed_step_then_freezes(reason):
    env = setup_controller()
    acquire(env)
    env.gate.submit(command("step", actions=[[.1] * 6 + [.5]] * 3))
    assert env.control.prepare_step() == 40_000_000
    env.gate.invalidate(reason)
    env.control.complete_step([observation(2)])
    result, = env.gate.take_results()
    assert not result["ok"] and result["error"] == reason
    assert result["executed_steps"] == 1 and result["sim_time_ns"] == "40000000"
    assert env.control.paused and env.control.prepare_step() is None
    env.gate.submit(command("step"))
    assert "stalled" in env.gate.take_results()[0]["error"]
    env.gate.submit(command("release"))
    env.control.service()
    assert env.gate.take_results()[0]["ok"]
    assert not env.control.paused and not env.target.policy_active


def test_deadlines_while_waiting_for_capacity_and_between_chunks_fail_closed():
    env = setup_controller()
    acquire(env)
    env.gate.submit(command("step", actions=[[.1] * 6 + [.5]] * 2))
    env.control.service()
    env.clock.value += 6
    env.control.service()  # the same path used by the backpressure loop
    result, = env.gate.take_results()
    assert not result["ok"] and result["executed_steps"] == 0
    assert "deadline" in result["error"] and env.control.paused
    assert not env.target.calls
    env.gate.submit(command("release"))
    env.control.service()
    env.gate.take_results()
    acquire(env)
    env.clock.value += 31
    env.control.service()
    assert env.control.paused and "idle" in env.gate.fault
    assert not env.gate.take_results()


def test_pending_acquire_disconnect_is_latched_and_cannot_be_replayed_on_reconnect():
    env = setup_controller()
    packet = command()
    env.gate.submit(packet)
    env.gate.invalidate("disconnect")
    env.control.service()
    assert env.control.paused and not env.gate.take_results()[0]["ok"]
    env.control.observe(observation(1))
    env.gate.submit(packet)
    assert "already received" in env.gate.take_results()[0]["error"]
    assert env.control.prepare_step() is None


def test_reset_discards_queued_step_and_old_generation():
    env = setup_controller()
    acquire(env)
    env.gate.submit(command("step"))
    env.gate.invalidate("scene reset")
    env.control.service()
    result, = env.gate.take_results()
    assert not result["ok"] and result["executed_steps"] == 0
    env.gate.reset("reset-2")
    env.gate.submit(command("step"))
    assert "reset_generation" in env.gate.take_results()[0]["error"]
    assert not env.target.calls


@pytest.mark.parametrize("case", ["backend", "preparation", "task"])
def test_unsupported_runtime_does_not_pause(case):
    env = setup_controller(supported=case != "backend")
    if case == "preparation":
        env.target.preparation.ready = False
    if case == "task":
        env.control.available_tasks.clear()
    env.control.observe(observation(1))
    env.gate.submit(command(task_name="drawer_cycle"))
    env.control.service()
    assert not env.gate.take_results()[0]["ok"]
    assert not env.control.paused and not env.target.calls


def test_unexpected_render_boundary_is_failure_and_cannot_continue_chunk():
    env = setup_controller()
    acquire(env)
    env.gate.submit(command("step"))
    env.control.prepare_step()
    env.control.complete_step([observation(2), observation(3)])
    result, = env.gate.take_results()
    assert not result["ok"] and "exactly one" in result["error"]
    assert env.control.paused and env.control.prepare_step() is None


def test_cartesian_target_uses_governor_and_rate_limits_without_ik_or_deadman_overwrite():
    from simulation.teleop_grasp_unreal import CartesianTeleopTarget, ARM_JOINT_VELOCITY_LIMIT
    from simulation.online_elbow_ik import OnlineElbowPreference
    class Chain:
        def forward(self, q):
            return np.asarray(q[:3]).copy(), np.eye(3)
        def jacobian(self, q):
            return np.eye(6)
    class Client:
        def latest_action(self):
            raise AssertionError("teleop must not be read while policy owns references")
    limits = SimpleNamespace(names=tuple(f"joint{i}" for i in range(6)) + ("ee_link11_joint", "ee_link12_joint"),
                             lower=(-2.,) * 6 + (0., -1.), upper=(2.,) * 6 + (1., 0.))
    target = CartesianTeleopTarget(np.array([0.] * 6 + [.5, -.5]), Client(), Chain(),
        joint_limits=limits, elbow_preference=OnlineElbowPreference(enabled=False))
    target.bind_joint_state_provider(lambda: np.zeros(6))
    target.set_policy_control(True, .02)
    applied = target.apply_policy_action([1.] * 6 + [1.])
    np.testing.assert_allclose(applied[:6], ARM_JOINT_VELOCITY_LIMIT * .02)
    assert applied[6] == pytest.approx(.504)
    assert target.position[7] == pytest.approx(-.504)
    held, velocity = target.cached_reference(.02)
    for timestamp in (.0225, .025, .03, .04):
        q, v = target.update(timestamp)
        np.testing.assert_equal(q, held)
        np.testing.assert_equal(v, velocity)
    assert target.ik_solve_count == 0
    target.position[0] = .079
    applied = target.apply_policy_action([1.] * 6 + [0.])
    assert applied[0] <= .08
    assert target.governor_state == "tracking_limited"
    before = target.position.copy()
    target.set_policy_control(False, .04)
    np.testing.assert_equal(target.position, before)
    np.testing.assert_equal(target.velocity, np.zeros(8))


def test_transport_discards_teleop_and_handles_stop_and_reset_without_advancing():
    client = SimulationControlClient("127.0.0.1", 0, "test", policy_runtime=True)
    try:
        client._accept_message(command())
        client.policy.session_id = "policy-session"
        action = {**client._neutral_action(), "deadman": True, "server_sequence": "9"}
        client._accept_message(action)
        assert client.latest_action()[0]["deadman"] is False
        client._accept_message({**action, "deadman": False, "reason": "input_timeout"})
        assert client.policy.fault is None
        client._accept_message({**action, "deadman": False, "reason": "episode_stopped"})
        assert "STOP" in client.policy.fault
        client._accept_message(dict(protocol=CONTROL_PROTOCOL, type="reset", request_id="generation-2"))
        assert client.take_reset() == "generation-2"
        assert client.policy.pending is None and client.policy.session_id is None
        assert client.latest_action()[1]
        client._accept_message(command("step"))
        assert any("reset" in r.get("error", "") for r in client.policy.take_results())
    finally:
        client.close()


def test_disabled_scripted_transport_rejects_policy_instead_of_queueing_forever():
    client = SimulationControlClient("127.0.0.1", 0, "scripted")
    client._accept_message(command())
    assert "does not support" in client.policy.take_results()[0]["error"]
    assert client.policy.pending is None and not client.policy.controlling


def test_socket_hello_and_separate_result_delivery_while_no_physics_advances():
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen()
    listener.settimeout(3.)
    client = SimulationControlClient("127.0.0.1", listener.getsockname()[1], "test", policy_runtime=True)
    client.start()
    connection = None
    try:
        connection, _ = listener.accept()
        connection.settimeout(3.)
        reader = FramedSocketReader()
        hello = reader.receive(connection)
        assert "joint_policy_v1" in hello["capabilities"]
        connection.sendall(encode_packet(dict(protocol=CONTROL_PROTOCOL, type="observation_ready",
            observation_stream_id=hello["observation_stream_id"], observation_sequence="0")))
        client.wait_until_ready(2.)
        packet = command()
        connection.sendall(encode_packet(packet))
        deadline = time.monotonic() + 2
        while client.policy.pending is None and time.monotonic() < deadline:
            time.sleep(.005)
        assert client.policy.pending is not None
        client.policy.observation = observation(1)
        client.policy.session_id = packet["session_id"]
        client.policy.finish(client.policy.pending)
        result = reader.receive(connection)
        assert result["type"] == "joint_policy_result" and result["ok"]
        assert result["request_id"] == packet["request_id"] and result["sim_time_ns"] == "20000000"
        assert client._next_sequence == 1  # results never consume reliable observation sequences
        connection.close()
        connection = None
        deadline = time.monotonic() + 2
        while client.policy.fault is None and time.monotonic() < deadline:
            time.sleep(.005)
        assert client.policy.fault and client.policy.controlling
    finally:
        if connection:
            connection.close()
        client.close()
        listener.close()


def test_drawer_state_reads_actual_geometry_and_contacts_without_a_script():
    class Message:
        def __init__(self, value):
            self.value = value
        def addSubscriber(self):
            return lambda: SimpleNamespace(state=self.value)
    drawer = SimpleNamespace(stateOutMsg=Message(.04), stateDotOutMsg=Message(.002))
    buckle = SimpleNamespace(stateOutMsg=Message(.1))
    bodies = {"drawer_base": drawer, "drawer_buckle": buckle}
    scene = SimpleNamespace(getBody=lambda name: SimpleNamespace(getScalarJoint=lambda _: bodies[name]))
    names = ["world", "drawer_base", "drawer_buckle", "ee_finger1", "ee_finger2", "unrelated"]
    stepper = SimpleNamespace(nbody=len(names), _name=lambda _, i: names[i],
        contact_snapshot=lambda: [(0, 0, 1, 3, -.001), (0, 0, 4, 2, .00001),
                                  (0, 0, 1, 4, .1), (0, 0, 3, 5, -.001)])
    provider = DrawerPolicyState(scene, stepper)
    first = provider.snapshot([0.] * 6 + [.6, -.6], [.1, .2, .3])
    assert first["finger_drawer_contacts"] == [1, 1]
    assert first["drawer_position_m"] == .04 and first["drawer_velocity_m_s"] == .002
    assert first["buckle_rad"] == .1 and first["gripper_opening_normalized"] == .6
    drawer.stateOutMsg.value = .09
    second = provider.snapshot([0.] * 8, [.3, .2, .1])
    assert second["drawer_displacement_m"] == pytest.approx(.05)
    assert second["initial_end_effector_position_body_m"] == [.1, .2, .3]
    assert "phase" not in second
    assert second["success_available"] is False and second["success"] is False


@pytest.mark.parametrize("task_name", [None, ""])
def test_generic_acquire_accepts_empty_task_without_enabling_a_monitor(task_name):
    env = setup_controller()
    resets = []
    env.control.reset_task = resets.append
    env.control.observe(observation(1))
    env.gate.submit(command(task_name=task_name, idle_timeout_s=180))
    env.control.service()
    assert env.gate.take_results()[0]["ok"]
    assert env.control.task_name is None and resets == [None]


@pytest.mark.parametrize("idle", [10, 180., 3600])
def test_session_idle_timeout_does_not_extend_command_deadline(idle):
    env = setup_controller()
    env.control.observe(observation(1))
    env.gate.submit(command(idle_timeout_s=idle))
    env.control.service()
    assert env.gate.take_results()[0]["ok"]
    env.clock.value += idle - .1
    env.control.service()
    assert env.gate.fault is None and env.control.paused
    assert env.control.execution_metadata()["idle_timeout_s"] == idle
    env.gate.submit(command("step"))
    assert env.gate.pending.deadline == env.clock.value + 5
    env.clock.value += 5.1
    env.control.service()
    result, = env.gate.take_results()
    assert not result["ok"] and "command monotonic deadline" in result["error"]
    assert env.control.paused and not env.target.calls


@pytest.mark.parametrize("idle", [True, False, "180", None, 9.99, 3600.01, float("nan"), float("inf"), 10 ** 1000])
def test_invalid_session_idle_timeout_does_not_acquire(idle):
    env = setup_controller()
    env.control.observe(observation(1))
    env.gate.submit(command(idle_timeout_s=idle))
    env.control.service()
    result, = env.gate.take_results()
    assert not result["ok"] and "idle_timeout_s" in result["error"]
    assert not env.control.paused


def test_idle_deadline_expires_at_configured_limit_and_acquire_resets_task():
    env = setup_controller()
    resets = []
    env.control.reset_task = resets.append
    env.control.observe(observation(1))
    env.gate.submit(command(task_name="drawer_cycle", idle_timeout_s=180))
    env.control.service()
    env.gate.take_results()
    env.clock.value += 179.99
    env.control.service()
    assert env.gate.fault is None
    env.clock.value += .02
    env.control.service()
    assert "idle" in env.gate.fault and env.control.paused
    env.gate.submit(command("release"))
    env.control.service()
    assert env.gate.take_results()[0]["ok"]
    assert resets == ["drawer_cycle", None]


@pytest.mark.parametrize("task_name", ["future_physical_task", "t" * 128])
def test_unknown_task_passes_gate_but_controller_refuses_without_claiming_control(task_name):
    env = setup_controller()
    env.control.observe(observation(1))
    request = command(task_name=task_name)
    env.gate.submit(request)
    assert env.gate.pending is not None and not env.gate.take_results()
    env.control.service()
    result, = env.gate.take_results()
    assert result["request_id"] == request["request_id"]
    assert not result["ok"] and "task state is unavailable" in result["error"]
    assert result["executed_steps"] == 0
    assert env.gate.session_id is None and env.gate.pending is None and env.gate.fault is None
    assert not env.control.paused and not env.target.policy_active and not env.target.calls
    env.gate.submit(command(task_name=""))
    env.control.service()
    assert env.gate.take_results()[0]["ok"], "definitive refusal needs no release before a new acquire"


@pytest.mark.parametrize("task_name", [True, 5, [], {}, "t" * 129])
def test_gate_rejects_invalid_task_name_type_or_length(task_name):
    env = setup_controller()
    env.gate.submit(command(task_name=task_name))
    result, = env.gate.take_results()
    assert not result["ok"] and "task_name" in result["error"]
    assert env.gate.pending is None and not env.control.paused


def test_controller_can_enable_a_new_registered_task_without_transport_changes():
    env = setup_controller()
    env.control.available_tasks.add("future_physical_task")
    selected = []
    env.control.reset_task = selected.append
    env.control.observe(observation(1))
    env.gate.submit(command(task_name="future_physical_task"))
    env.control.service()
    assert env.gate.take_results()[0]["ok"]
    assert env.control.task_name == "future_physical_task"
    assert selected == ["future_physical_task"]
