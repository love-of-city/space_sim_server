"""Operator ownership, scaling, limits and dead-man handling."""

from __future__ import annotations

import math
import threading
import time

from .models import AppliedAction, OperatorAction


class ActionRejected(ValueError):
    """The operator action cannot safely be accepted."""


class SafetyController:
    """Convert normalized human input into bounded Cartesian and gripper commands."""

    LINEAR_SPEED_MIN_M_S = 0.01
    LINEAR_SPEED_DEFAULT_M_S = 0.05
    LINEAR_SPEED_MAX_M_S = 0.20
    ANGULAR_MAX_RAD_S = 0.50
    GRIPPER_MAX_RAD_S = 0.80

    def __init__(self, timeout_s: float = 0.25) -> None:
        self.timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._server_sequence = 0
        self._last_client_sequence: dict[str, int] = {}
        self._last_input_monotonic = 0.0
        self._last_action: AppliedAction | None = None

    @property
    def last_action(self) -> AppliedAction | None:
        with self._lock:
            return self._last_action.model_copy(deep=True) if self._last_action else None

    def process(self, operator_id: str, request: OperatorAction, episode_id: str | None) -> AppliedAction:
        values = [
            request.end_effector_linear_speed_m_s,
            *request.end_effector_linear_velocity,
            *request.end_effector_angular_velocity,
            request.gripper_velocity,
        ]
        if not all(math.isfinite(value) for value in values):
            raise ActionRejected("action contains a non-finite number")
        with self._lock:
            previous = self._last_client_sequence.get(operator_id, -1)
            if request.client_sequence <= previous:
                raise ActionRejected("client_sequence is stale or duplicated")
            self._last_client_sequence[operator_id] = request.client_sequence
            self._last_input_monotonic = time.monotonic()
            action = self._build_action(request, episode_id)
            self._last_action = action
            return action.model_copy(deep=True)

    def timeout_action(self, episode_id: str | None) -> AppliedAction | None:
        """Return one neutral action when a live command crosses the timeout."""

        with self._lock:
            if not self._last_action or not self._last_action.deadman:
                return None
            if time.monotonic() - self._last_input_monotonic <= self.timeout_s:
                return None
            request = OperatorAction(
                client_sequence=int(self._last_action.client_sequence) + 1,
                client_time_ns=str(time.time_ns()),
                deadman=False,
                end_effector_linear_velocity=[0.0] * 3,
                end_effector_angular_velocity=[0.0] * 3,
                gripper_velocity=0.0,
                input_source="unknown",
            )
            action = self._build_action(request, episode_id, reason="input_timeout")
            self._last_action = action
            return action.model_copy(deep=True)

    def neutral(self, episode_id: str | None, reason: str) -> AppliedAction:
        with self._lock:
            request = OperatorAction(
                client_sequence=0,
                client_time_ns=str(time.time_ns()),
                deadman=False,
                end_effector_linear_velocity=[0.0] * 3,
                end_effector_angular_velocity=[0.0] * 3,
                gripper_velocity=0.0,
                input_source="unknown",
            )
            action = self._build_action(request, episode_id, reason=reason)
            self._last_action = action
            return action.model_copy(deep=True)

    def _build_action(
        self,
        request: OperatorAction,
        episode_id: str | None,
        reason: str = "",
    ) -> AppliedAction:
        self._server_sequence += 1
        requested_linear_speed = float(request.end_effector_linear_speed_m_s)
        linear_speed = max(
            self.LINEAR_SPEED_MIN_M_S,
            min(self.LINEAR_SPEED_MAX_M_S, requested_linear_speed),
        )
        limited = linear_speed != requested_linear_speed
        normalized_linear: list[float] = []
        for value in request.end_effector_linear_velocity:
            clipped = max(-1.0, min(1.0, float(value)))
            limited |= clipped != value
            normalized_linear.append(clipped)
        normalized_angular: list[float] = []
        for value in request.end_effector_angular_velocity:
            clipped = max(-1.0, min(1.0, float(value)))
            limited |= clipped != value
            normalized_angular.append(clipped)
        grip = max(-1.0, min(1.0, float(request.gripper_velocity)))
        limited |= grip != request.gripper_velocity
        deadman = bool(request.deadman)
        if not deadman:
            normalized_linear = [0.0] * 3
            normalized_angular = [0.0] * 3
            grip = 0.0
        return AppliedAction(
            episode_id=episode_id,
            server_sequence=str(self._server_sequence),
            server_time_ns=str(time.time_ns()),
            client_sequence=str(request.client_sequence),
            client_time_ns=request.client_time_ns,
            deadman=deadman,
            requested_end_effector_linear_velocity_normalized=[
                float(value) for value in request.end_effector_linear_velocity
            ],
            requested_end_effector_angular_velocity_normalized=[
                float(value) for value in request.end_effector_angular_velocity
            ],
            requested_gripper_velocity_normalized=float(request.gripper_velocity),
            requested_end_effector_linear_speed_m_s=requested_linear_speed,
            applied_end_effector_linear_speed_m_s=linear_speed,
            end_effector_linear_velocity_body_m_s=[
                value * linear_speed for value in normalized_linear
            ],
            end_effector_angular_velocity_body_rad_s=[
                value * self.ANGULAR_MAX_RAD_S for value in normalized_angular
            ],
            gripper_velocity_rad_s=grip * self.GRIPPER_MAX_RAD_S,
            input_source=request.input_source,
            limited=limited,
            reason=reason,
        )
