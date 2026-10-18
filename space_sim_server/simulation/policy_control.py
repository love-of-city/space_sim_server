"""Joint policy admission and render-boundary execution, independent of native physics.

The socket worker only admits/cancels requests. The simulation thread services the
controller between advances. No request, including a retry after reconnect, can
cause a second execution of the same UUID within a reset generation.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
import uuid
from typing import Any, Callable

from space_arm_platform.protocol import CONTROL_PROTOCOL

POLICY_PERIOD_NS = 20_000_000
MAX_POLICY_HORIZON = 100
DEFAULT_POLICY_IDLE_TIMEOUT_S = 30.


def validate_actions(actions: Any, lower=None, upper=None) -> list[list[float]]:
    if not isinstance(actions, list) or not 1 <= len(actions) <= MAX_POLICY_HORIZON:
        raise ValueError("actions must contain 1..100 seven-element targets")
    checked = []
    for row in actions:
        if not isinstance(row, list) or len(row) != 7:
            raise ValueError("each action must contain six absolute radians and one gripper opening")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in row):
            raise ValueError("actions must contain finite numbers")
        try:
            finite = all(math.isfinite(v) for v in row)
        except OverflowError:
            finite = False
        if not finite:
            raise ValueError("actions must contain finite numbers")
        if not 0 <= row[6] <= 1:
            raise ValueError("gripper opening must be in [0, 1] (0 closed, 1 open)")
        if lower is not None and any(not lo <= v <= hi for v, lo, hi in zip(row[:6], lower, upper)):
            raise ValueError("absolute arm target exceeds model joint limits")
        checked.append([float(v) for v in row])
    return checked


@dataclass
class PolicyRequest:
    packet: dict[str, Any]
    deadline: float
    epoch: int


class PolicyCommandGate:
    """A single outstanding command, bounded replies, and non-evicting UUID tombstones.

    Results are not replayed: a duplicate fails with an explicit uncertain-outcome
    error. At the tombstone limit, new motion is refused until reset. Release is
    always admissible when idle, including after the limit, since it cannot step.
    """

    def __init__(self, *, timeout_s: float = 30., max_request_ids: int = 65536,
                 clock: Callable[[], float] = time.monotonic, enabled: bool = True):
        if not math.isfinite(timeout_s) or timeout_s <= 0 or max_request_ids < 1:
            raise ValueError("positive policy timeout and request capacity required")
        self.lock = threading.RLock()
        self.clock, self.timeout_s = clock, timeout_s
        self.max_request_ids = max_request_ids
        self.enabled = enabled
        self.generation = ""
        self.session_id: str | None = None
        self.fault: str | None = None
        self.epoch = 0
        self.pending: PolicyRequest | None = None
        self.claimed = False
        self.observation: dict[str, Any] | None = None
        self._seen: set[uuid.UUID] = set()
        # Loss of a reply is safe: UUID tombstones still forbid executing it again.
        self._results: deque[dict[str, Any]] = deque(maxlen=128)

    @property
    def controlling(self) -> bool:
        with self.lock:
            return bool(self.session_id or self.fault or
                        (self.pending and self.pending.packet["operation"] == "acquire"))

    def _result(self, packet, error=None, *, executed_steps=0, applied_actions=()):
        identity = ({key: self.observation[key] for key in
                     ("render_session_id", "render_frame_id", "sim_time_ns")}
                    if self.observation is not None else {})
        result = dict(protocol=CONTROL_PROTOCOL, type="joint_policy_result",
                      request_id=packet.get("request_id"), session_id=packet.get("session_id"),
                      reset_generation=packet.get("reset_generation", self.generation),
                      ok=error is None, executed_steps=executed_steps,
                      period_ns=POLICY_PERIOD_NS, applied_actions=deepcopy(list(applied_actions)),
                      observation=identity, **identity)
        if error is not None:
            result["error"] = str(error)
        return result

    def submit(self, packet: dict[str, Any], *, resetting: bool = False) -> None:
        with self.lock:
            try:
                if not self.enabled:
                    raise ValueError("this runtime does not support joint_policy_v1")
                raw_id = packet.get("request_id")
                if not isinstance(raw_id, str) or len(raw_id) > 64:
                    raise ValueError("request_id must be a UUID")
                request_id = uuid.UUID(raw_id)
                if request_id in self._seen:
                    raise ValueError("request_id already received; original outcome may be uncertain; release or reset, never replay")
                if len(self._seen) >= self.max_request_ids:
                    if packet.get("operation") != "release":
                        raise ValueError("request history full; release and reset before further policy motion")
                else:
                    self._seen.add(request_id)
                if packet.get("protocol") != CONTROL_PROTOCOL or packet.get("type") != "joint_policy_command":
                    raise ValueError("invalid joint policy protocol")
                session = packet.get("session_id")
                if not isinstance(session, str) or not 1 <= len(session) <= 128:
                    raise ValueError("session_id must be a nonempty string of at most 128 characters")
                if resetting or packet.get("reset_generation") != self.generation:
                    raise ValueError("stale reset_generation or reset in progress")
                operation = packet.get("operation")
                if operation not in {"acquire", "step", "release"}:
                    raise ValueError("unsupported joint policy operation")
                task_name = packet.get("task_name")
                if task_name is not None and (not isinstance(task_name, str) or len(task_name) > 128):
                    raise ValueError("task_name must be null or a string of at most 128 characters")
                if "idle_timeout_s" in packet:
                    idle_timeout = packet["idle_timeout_s"]
                    if (operation != "acquire" or isinstance(idle_timeout, bool)
                            or not isinstance(idle_timeout, (int, float))
                            or not 10 <= idle_timeout <= 3600):
                        raise ValueError("acquire idle_timeout_s must be finite and within [10, 3600] seconds")
                if operation == "step":
                    if type(packet.get("period_ns")) is not int or packet["period_ns"] != POLICY_PERIOD_NS:
                        raise ValueError("period_ns must be 20000000")
                    validate_actions(packet.get("actions"))
                if self.pending is not None:
                    raise ValueError("another joint policy request is in progress")
                if operation == "acquire" and (self.session_id or self.fault):
                    raise ValueError("policy control already acquired or stalled; release or reset required")
                if operation != "acquire" and session != self.session_id:
                    raise ValueError("session does not own joint policy control")
                if operation == "step" and self.fault:
                    raise ValueError(f"policy stalled: {self.fault}; release or reset required")
                self.pending = PolicyRequest(deepcopy(packet), self.clock() + self.timeout_s, self.epoch)
                self.claimed = False
            except (ValueError, TypeError, AttributeError) as error:
                self._results.append(self._result(packet, error))

    def invalidate(self, reason: str) -> None:
        with self.lock:
            if self.controlling or self.pending is not None:
                self.epoch += 1
                self.fault = reason
                # A queued acquire is an uncertain ownership transition on disconnect.
                if self.session_id is None and self.pending is not None:
                    self.session_id = self.pending.packet["session_id"]

    def reset(self, generation: str) -> None:
        with self.lock:
            if self.pending is not None:
                self.finish(self.pending, "scene reset before execution")
            self.generation = generation
            self.session_id = self.fault = None
            self.observation = None
            self.epoch += 1
            self._seen.clear()

    def finish(self, request: PolicyRequest, error=None, *, applied_actions=()) -> None:
        with self.lock:
            self._results.append(self._result(request.packet, error,
                                             executed_steps=len(applied_actions), applied_actions=applied_actions))
            if self.pending is request:
                self.pending = None
                self.claimed = False

    def take_results(self) -> list[dict[str, Any]]:
        with self.lock:
            results = list(self._results)
            self._results.clear()
            return results


class JointPolicyController:
    """Own a completed render boundary; permit at most one 20 ms advance at a time."""

    def __init__(self, gate: PolicyCommandGate, targets, *, supported: bool,
                 discard_teleop: Callable[[], None], capture_state: Callable[[], dict],
                 publish_reference: Callable[[int], None], available_tasks=(),
                 reset_task: Callable[[str | None], None] | None = None):
        self.gate, self.targets = gate, targets
        self.supported = supported
        self.discard_teleop, self.capture_state = discard_teleop, capture_state
        self.publish_reference = publish_reference
        self.available_tasks = set(available_tasks)
        self.reset_task = reset_task or (lambda _task_name: None)
        self.idle_timeout_s = DEFAULT_POLICY_IDLE_TIMEOUT_S
        self.active: PolicyRequest | None = None
        self.actions: list[list[float]] = []
        self.applied: list[list[float]] = []
        self.inflight: list[float] | None = None
        self.expected_stop_ns: int | None = None
        self.last_progress = gate.clock()
        self.task_name: str | None = None

    @property
    def paused(self) -> bool:
        return self.gate.session_id is not None or self.gate.fault is not None

    @property
    def stepping(self) -> bool:
        return self.active is not None and self.gate.fault is None

    def _hold(self, enabled=True) -> None:
        snapshot = self.gate.observation
        now_ns = int(snapshot["sim_time_ns"]) if snapshot else 0
        self.targets.set_policy_control(enabled, now_ns * 1e-9)
        if snapshot is not None:
            self.publish_reference(now_ns)

    def observe(self, snapshot: dict[str, Any]) -> None:
        # Called only after the authoritative snapshot was enqueued for delivery.
        with self.gate.lock:
            self.gate.observation = snapshot

    def _ready_to_acquire(self) -> bool:
        snapshot = self.gate.observation
        capture = self.capture_state()
        return bool(snapshot and int(snapshot["sim_time_ns"]) >= POLICY_PERIOD_NS
                    and int(snapshot["sim_time_ns"]) % POLICY_PERIOD_NS == 0
                    and snapshot.get("reset_generation", "") == self.gate.generation
                    and snapshot.get("capture_episode_id", "") == capture.get("capture_episode_id", "")
                    and snapshot.get("capture_request_id", "") == capture.get("capture_request_id", ""))

    def service(self) -> None:
        """Never advances physics or waits on I/O. Call even under backpressure."""
        # Read transport/capture state before taking the gate lock (lock order).
        ready = self._ready_to_acquire()
        with self.gate.lock:
            now = self.gate.clock()
            request = self.gate.pending
            if request and now >= request.deadline:
                self.gate.invalidate("policy command monotonic deadline expired")
            elif self.gate.session_id and not request and now - self.last_progress >= self.idle_timeout_s:
                if self.gate.fault is None:
                    self.gate.invalidate("policy idle monotonic deadline expired")
            if request and (request.epoch != self.gate.epoch or
                            (self.gate.fault and request.packet["operation"] != "release")):
                self._hold()
                self.gate.finish(request, self.gate.fault or "policy command invalidated",
                                 applied_actions=self.applied if self.active is request else ())
                self.active, self.inflight = None, None
                request = None
            if self.gate.fault:
                self._hold()
            if request is None or self.gate.claimed:
                return
            packet = request.packet
            operation = packet["operation"]
            if operation == "acquire":
                if not self.supported or not self.targets.preparation.ready:
                    self.gate.finish(request, "joint policy requires local dynamics, a supported gripper, and no scripted task or pending arm preparation")
                    return
                if packet.get("task_name") and packet["task_name"] not in self.available_tasks:
                    self.gate.finish(request, "requested physical task state is unavailable in this scene")
                    return
                if not ready:
                    return  # In particular, time-zero alone is never an acquisition boundary.
                self.gate.session_id = packet["session_id"]
                self.task_name = packet.get("task_name") or None
                self.idle_timeout_s = float(packet.get("idle_timeout_s", DEFAULT_POLICY_IDLE_TIMEOUT_S))
                self.reset_task(self.task_name)
                self.applied = []
                self._hold()
                self.gate.finish(request)
                self.last_progress = now
            elif operation == "release":
                self._hold(False)
                self.gate.session_id = self.gate.fault = None
                self.task_name = None
                self.reset_task(None)
                self.applied = []
                self.gate.finish(request)
                self.last_progress = now
            else:
                try:
                    self.actions = validate_actions(packet["actions"], self.targets.joint_min[:6], self.targets.joint_max[:6])
                except ValueError as error:
                    self.gate.finish(request, error)
                    return
                self.active = request
                self.applied = []
                self.gate.claimed = True
        # Drop both pre-acquire and policy-period human input. Never call client
        # methods with the gate lock held; the worker takes client then gate.
        self.discard_teleop()

    def prepare_step(self) -> int | None:
        self.service()
        with self.gate.lock:
            if not self.stepping:
                return None
            if self.inflight is not None:
                raise RuntimeError("policy interval already in flight")
            snapshot = self.gate.observation
            self.expected_stop_ns = int(snapshot["sim_time_ns"]) + POLICY_PERIOD_NS
            try:
                self.inflight = self.targets.apply_policy_action(self.actions[len(self.applied)])
                # Publish before the first physical substep, including when the
                # normal IK publisher's stride does not divide this render tick.
                self.publish_reference(int(snapshot["sim_time_ns"]))
            except (ValueError, RuntimeError, FloatingPointError) as error:
                self.gate.invalidate(f"policy target application failed: {error}")
                self.inflight = None
                self._hold()
                self.gate.finish(self.active, self.gate.fault, applied_actions=self.applied)
                self.active = None
                return None
            return self.expected_stop_ns

    def complete_step(self, snapshots: list[dict[str, Any]]) -> None:
        with self.gate.lock:
            if self.inflight is None:
                return
            previous = self.gate.observation
            valid = (len(snapshots) == 1
                     and int(snapshots[0]["sim_time_ns"]) == self.expected_stop_ns
                     and snapshots[0]["render_session_id"] == previous["render_session_id"]
                     and int(snapshots[0]["render_frame_id"]) == int(previous["render_frame_id"]) + 1)
            for snapshot in snapshots:
                self.observe(snapshot)
            if valid:
                self.applied.append(self.inflight)
                self.last_progress = self.gate.clock()
            else:
                self.gate.invalidate("policy advance did not emit exactly one expected authoritative render frame")
            self.inflight = None
            self._hold()
            if self.gate.clock() >= self.active.deadline:
                self.gate.invalidate("policy command monotonic deadline expired")
            if self.gate.fault or len(self.applied) == len(self.actions):
                self.gate.finish(self.active, self.gate.fault, applied_actions=self.applied)
                self.active = None

    def execution_metadata(self) -> dict[str, Any] | None:
        if not self.gate.session_id:
            return None
        return dict(session_id=self.gate.session_id,
                    request_id=self.active.packet["request_id"] if self.active else None,
                    executed_steps=len(self.applied) + int(self.inflight is not None),
                    period_ns=POLICY_PERIOD_NS,
                    applied_action=list(self.inflight) if self.inflight is not None else None,
                    task_name=self.task_name, idle_timeout_s=self.idle_timeout_s, stalled=self.gate.fault)


class DrawerPolicyState:
    """Read actual drawer/contact state and evaluate read-only task predicates."""

    def __init__(self, scene, stepper, *, model_path=None):
        from simulation.policy_task_success import PolicyTaskSuccessProvider

        self.success = PolicyTaskSuccessProvider(model_path)
        self.latest_sample = None
        self.stepper = stepper
        drawer = scene.getBody("drawer_base").getScalarJoint("drawer_base_slide")
        buckle = scene.getBody("drawer_buckle").getScalarJoint("drawer_buckle_joint")
        self.drawer = drawer.stateOutMsg.addSubscriber()
        self.velocity = drawer.stateDotOutMsg.addSubscriber()
        self.buckle = buckle.stateOutMsg.addSubscriber()
        self.body_names = {i: stepper._name(1, i) for i in range(stepper.nbody)}
        self.initial: dict[str, Any] | None = None

    def reset(self, task_name: str | None) -> None:
        # The cached sample is the last authoritative render snapshot, including
        # its original simulation timestamp; never resample paused native state.
        self.success.reset(task_name, self.latest_sample)
        self.initial = (dict(initial_drawer_position_m=self.latest_sample["drawer_position_m"],
                             initial_buckle_rad=self.latest_sample["buckle_rad"],
                             initial_end_effector_position_body_m=list(self.latest_sample["end_effector_position_body_m"]))
                        if self.latest_sample is not None else None)

    def snapshot(self, joints, ee_position, *, sim_time_ns=None, joint_velocities=None,
                 ee_rotation=None) -> dict[str, Any]:
        drawer, buckle = float(self.drawer().state), float(self.buckle().state)
        contacts = [0, 0]
        for _, _, first, second, distance in self.stepper.contact_snapshot():
            names = {self.body_names.get(int(first)), self.body_names.get(int(second))}
            if distance <= .0001 and names.intersection({"drawer_base", "drawer_buckle"}):
                for index in (0, 1):
                    contacts[index] += int(f"ee_finger{index + 1}" in names)
        if self.initial is None:
            self.initial = dict(initial_drawer_position_m=drawer, initial_buckle_rad=buckle,
                                initial_end_effector_position_body_m=list(ee_position))
        state = dict(drawer_position_m=drawer,
                    drawer_velocity_m_s=float(self.velocity().state), buckle_rad=buckle,
                    finger_drawer_contacts=contacts, gripper_drive_position_rad=float(joints[6]),
                    gripper_opening_normalized=max(0., min(1., float(joints[6]))),
                    end_effector_position_body_m=list(ee_position),
                    drawer_displacement_m=drawer - self.initial["initial_drawer_position_m"], **self.initial)
        self.latest_sample = dict(state, sim_time_ns=sim_time_ns,
                                  arm_joint_velocity_rad_s=joint_velocities[:6] if joint_velocities is not None else None,
                                  end_effector_rotation_body=ee_rotation)
        return {**state, **self.success.update(self.latest_sample)}
