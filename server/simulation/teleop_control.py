"""Cartesian teleoperation targets, reference limits and scheduled IK control."""
from __future__ import annotations
import math
import os
import time
from typing import Callable
import numpy as np
from Basilisk.architecture import sysModel
from simulation.serial_chain_kinematics import SerialChainKinematics, IkResult, axis_angle_to_matrix, rotation_matrix_to_vector
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
from simulation.arm_preparation import ArmPreparation
from space_arm_platform.joint_limits import load_joint_limits
from simulation.motion_diagnostics import MotionSpeedMonitor
from simulation.online_elbow_ik import OnlineElbowPreference, ElbowStepDiagnostics, ElbowDeviationState, apply_online_elbow_preference
from simulation.reference_governor import JointReferenceGovernor
from simulation.gamepad_control import add_joint6_command
from simulation.control_client import SimulationControlClient
from simulation.policy_control import POLICY_PERIOD_NS, validate_actions
from pathlib import Path
PROJECT_ROOT = Path(__file__).resolve().parents[1]

SARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7)) + ("joint_finger1", "joint_finger2")
# The default model limits serve direct diagnostic construction; live sessions
# pass the limits of their selected model explicitly.
_DEFAULT_LIMITS = load_joint_limits(PROJECT_ROOT / "model/SARM/platform/sarm_platform.xml", SARM_JOINT_NAMES)
JOINT_MIN = np.array(_DEFAULT_LIMITS.lower)
JOINT_MAX = np.array(_DEFAULT_LIMITS.upper)
ARM_JOINT_VELOCITY_LIMIT = np.array([0.70, 0.70, 0.70, 0.90, 1.00, 1.00])
IK_MODE_IK_POSE = "ik_pose"
IK_MODE_STRICT = "strict"
IK_MODES = (IK_MODE_IK_POSE, IK_MODE_STRICT)
# Damping grows near singularities while preserving Cartesian direction.
IK_POSE_BASE_DAMPING = 1.0e-3
IK_POSE_MAXIMUM_DAMPING = 5.0e-2
IK_POSE_SINGULAR_VALUE_THRESHOLD = 2.0e-2
MAX_LINEAR_COMMAND_ACCELERATION_M_S2 = 0.20
MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 = 2.0
TELEOP_ARM_KP = np.array([32.0, 32.0, 32.0, 30.0, 30.0, 15.0])
TELEOP_ARM_KD = np.array([2.0, 2.0, 2.0, 0.7, 0.5, 0.25])
TELEOP_ARM_TORQUE_LIMIT = np.array([2.0, 2.0, 2.0, 1.0, 1.0, 0.70])
ARM_JOINT_NAMES = (
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6",
)



class CartesianTeleopTarget:
    """Integrate bounded joint references from operator twist; telemetry is passive."""

    def __init__(
        self,
        initial_position: np.ndarray,
        client: SimulationControlClient,
        kinematics: SerialChainKinematics,
        *,
        ik_mode: str = IK_MODE_IK_POSE,
        joint_limits=None,
        elbow_preference: OnlineElbowPreference | None = None,
        preparation_required: bool = False,
        operating_joint_deg=None,
        preparation_plan=None,
    ) -> None:
        if ik_mode not in IK_MODES:
            raise ValueError(f"unsupported IK mode {ik_mode!r}; expected one of {IK_MODES}")
        self.initial_position = np.asarray(initial_position, dtype=float).copy()
        self.position = self.initial_position.copy()
        self.velocity = np.zeros_like(self.initial_position)
        self.client = client
        self.kinematics = kinematics
        self.ik_mode = ik_mode
        if elbow_preference is None:
            elbow_mode = os.environ.get("SPACE_SIM_ONLINE_ELBOW_MODE", "prefer")
            if elbow_mode not in {"off", "prefer"}:
                raise ValueError("SPACE_SIM_ONLINE_ELBOW_MODE must be off or prefer")
            wrist_mode = os.environ.get("SPACE_SIM_ONLINE_WRIST_MODE", "prefer")
            if wrist_mode not in {"off", "prefer"}:
                raise ValueError("SPACE_SIM_ONLINE_WRIST_MODE must be off or prefer")
            joint3_mode = os.environ.get("SPACE_SIM_ONLINE_JOINT3_MODE", "prefer")
            if joint3_mode not in {"off", "prefer"}:
                raise ValueError("SPACE_SIM_ONLINE_JOINT3_MODE must be off or prefer")
            elbow_preference = OnlineElbowPreference(
                enabled=elbow_mode == "prefer", wrist_enabled=wrist_mode == "prefer",
                joint3_enabled=joint3_mode == "prefer")
        self.elbow_preference = elbow_preference
        self.elbow_deviation = ElbowDeviationState()
        self.elbow_diagnostics = ElbowStepDiagnostics()
        limits = joint_limits or _DEFAULT_LIMITS
        self.linkage_gripper = limits.names[-2:] == ('ee_link11_joint', 'ee_link12_joint')
        if limits.names != SARM_JOINT_NAMES and not self.linkage_gripper:
            raise ValueError("joint limit order does not match the SARM chain")
        self.joint_min, self.joint_max = np.array(limits.lower), np.array(limits.upper)
        if self.linkage_gripper:
            self.joint_min[6:], self.joint_max[6:] = [0., -1.], [1., 0.]
        if np.any(self.initial_position < self.joint_min) or np.any(self.initial_position > self.joint_max):
            raise ValueError("initial joint positions exceed selected model limits")
        self.preparation = ArmPreparation(
            preparation_required,
            np.deg2rad(DEFAULT_OPERATING_JOINT_DEG if operating_joint_deg is None else operating_joint_deg),
            self.joint_min[:6], self.joint_max[:6], ARM_JOINT_VELOCITY_LIMIT,
            plan=preparation_plan)
        self.speed_monitor = MotionSpeedMonitor()
        self.solver_status = "idle"
        self.solver_reasons = []
        self.solve_time_ms = 0.
        self.raw_operator_twist = np.zeros(6)
        self._joint_velocity_provider = None
        self._actuator_state_provider = None
        self.actuator_diagnostics = None
        self._joint_state_provider: Callable[[], np.ndarray] | None = None
        self._effort_provider: Callable[[], np.ndarray] | None = None
        self.governor = JointReferenceGovernor()
        self.governor_state = "holding"
        self.governor_limited_joints: tuple[int, ...] = ()
        self.arm_effort_ratio = np.zeros(6)
        self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(
            self.initial_position[:6]
        )
        self.actual_tool_position = self.target_tool_position.copy()
        self.actual_tool_rotation = self.target_tool_rotation.copy()
        self.position_error = np.zeros(3)
        self.orientation_error = np.zeros(3)
        self.commanded_linear_velocity = np.zeros(3)
        self.commanded_angular_velocity = np.zeros(3)
        self.commanded_joint6_velocity = 0.0
        self.tracking_scale = 1.0
        self.velocity_scale = 1.0
        self.minimum_singular_value = 0.0
        self.condition_number = math.inf
        self.ik_damping = 0.0
        self.nullspace_correction_norm = 0.0
        self.last_sim_seconds: float | None = None
        self.applied_sequence = "0"
        self.command_stale = True
        self.policy_active = False
        self.desired_twist = np.zeros(6)
        self.achieved_twist = np.zeros(6)
        self.residual_twist = np.zeros(6)
        self.jacobian_rank = int(
            np.linalg.matrix_rank(self.kinematics.jacobian(self.position[:6]))
        )
        self.update_count = 0
        self.ik_solve_count = 0

    def bind_joint_state_provider(self, provider: Callable[[], np.ndarray]) -> None:
        """Use measured arm positions for telemetry; do not gate operator motion."""

        self._joint_state_provider = provider

    def bind_joint_velocity_provider(self, provider: Callable[[], np.ndarray]) -> None:
        self._joint_velocity_provider = provider

    def bind_actuator_state_provider(self, provider) -> None:
        self._actuator_state_provider = provider

    def _actual_arm_position(self) -> np.ndarray:
        if self._joint_state_provider is None:
            return self.position[:6].copy()
        measured = np.asarray(self._joint_state_provider(), dtype=float)
        if measured.shape != (6,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured arm joint state must contain six finite values")
        return measured

    def bind_effort_provider(self, provider: Callable[[], np.ndarray]) -> None:
        """Read absolute applied effort divided by each arm actuator's limit."""

        self._effort_provider = provider

    @staticmethod
    def _limit_vector(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= maximum_norm or norm <= 1.0e-14:
            return vector
        return vector * (maximum_norm / norm)

    @classmethod
    def _approach_vector(
        cls, current: np.ndarray, target: np.ndarray, maximum_delta: float
    ) -> np.ndarray:
        return current + cls._limit_vector(target - current, maximum_delta)

    def _advance_target(self, linear_velocity: np.ndarray, angular_velocity: np.ndarray, dt: float) -> None:
        self.target_tool_position = self.target_tool_position + linear_velocity * dt
        angular_speed = float(np.linalg.norm(angular_velocity))
        if angular_speed > 1.0e-14:
            delta = axis_angle_to_matrix(angular_velocity / angular_speed, angular_speed * dt)
            # Commands and Jacobian angular velocity are expressed in the
            # spacecraft body frame, so apply the spatial rotation on the left.
            self.target_tool_rotation = delta @ self.target_tool_rotation

    def reset(self, sim_seconds: float) -> None:
        """Reset the held joint and Cartesian references at simulation start."""

        self.position = self.initial_position.copy()
        self.policy_active = False
        self.preparation.reset()
        self.elbow_deviation.reset()
        self.elbow_diagnostics = ElbowStepDiagnostics()
        self.raw_operator_twist.fill(0.)
        self.actuator_diagnostics = None
        self.speed_monitor.reset()
        self.solver_reasons = []
        self.solver_status = "idle"
        self.solve_time_ms = 0.
        self.governor.reset()
        self.governor_state = "holding"
        self.governor_limited_joints = ()
        self.arm_effort_ratio.fill(0.0)
        self.velocity.fill(0.0)
        self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(
            self.initial_position[:6]
        )
        self.actual_tool_position = self.target_tool_position.copy()
        self.actual_tool_rotation = self.target_tool_rotation.copy()
        self.position_error.fill(0.0)
        self.orientation_error.fill(0.0)
        self.commanded_linear_velocity.fill(0.0)
        self.commanded_angular_velocity.fill(0.0)
        self.commanded_joint6_velocity = 0.0
        self.tracking_scale = 1.0
        self.velocity_scale = 1.0
        self.minimum_singular_value = 0.0
        self.condition_number = math.inf
        self.ik_damping = 0.0
        self.nullspace_correction_norm = 0.0
        self.last_sim_seconds = float(sim_seconds)
        self.applied_sequence = "0"
        self.command_stale = True
        self.desired_twist.fill(0.0)
        self.achieved_twist.fill(0.0)
        self.residual_twist.fill(0.0)
        self.update_count = 0
        self.ik_solve_count = 0

    def set_policy_control(self, enabled: bool, sim_seconds: float) -> None:
        """Hold the existing safe reference; ownership changes never assign qpos."""
        self.policy_active = enabled
        self.last_sim_seconds = float(sim_seconds)
        self.velocity.fill(0.)
        self.commanded_linear_velocity.fill(0.)
        self.commanded_angular_velocity.fill(0.)
        self.commanded_joint6_velocity = 0.
        self.raw_operator_twist.fill(0.)
        self.desired_twist.fill(0.)
        self.achieved_twist.fill(0.)
        self.residual_twist.fill(0.)
        self.command_stale = True
        self.applied_sequence = "0"
        self.solver_status = "policy_hold" if enabled else "holding"
        self.solve_time_ms = 0.
        self.solver_reasons = []
        self.elbow_deviation.reset()
        self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])

    def apply_policy_action(self, action: list[float]) -> list[float]:
        """Bound one absolute target through the existing reference/servo path."""
        if not self.policy_active:
            raise RuntimeError("joint policy does not own the reference")
        checked = validate_actions([action], self.joint_min[:6], self.joint_max[:6])[0]
        if not self.linkage_gripper:
            raise ValueError("joint policy requires the calibrated SARM linkage gripper")
        dt = POLICY_PERIOD_NS * 1e-9
        requested = np.clip((np.asarray(checked[:6]) - self.position[:6]) / dt,
                            -ARM_JOINT_VELOCITY_LIMIT, ARM_JOINT_VELOCITY_LIMIT)
        self.arm_effort_ratio = self._effort_provider() if self._effort_provider else np.zeros(6)
        governed = self.governor.apply(self.position[:6], self._actual_arm_position(),
                                       requested, dt, enabled=True, effort_ratio=self.arm_effort_ratio)
        self.velocity[:6] = governed.velocity
        self.position[:6] = np.clip(self.position[:6] + governed.velocity * dt,
                                    self.joint_min[:6], self.joint_max[:6])
        # The calibrated tendon coordinate is 0 closed / 1 open. Preserve the
        # existing 0.2 rad/s drive limit and the paired linkage reference.
        self.velocity[6] = float(np.clip((checked[6] - self.position[6]) / dt, -.2, .2))
        self.velocity[7] = -self.velocity[6]
        self.position[6] = float(np.clip(self.position[6] + self.velocity[6] * dt, 0., 1.))
        self.position[7] = -self.position[6]
        self.tracking_scale = governed.scale
        self.governor_state = governed.state
        self.governor_limited_joints = governed.limited_joints
        self.solver_status = "policy_target"
        self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])
        return self.position[:6].tolist() + [float(self.position[6])]

    def update(self, sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Update bounded joint references; hold them on release or stale input."""

        if self.policy_active:
            # Policy targets are published once at the render boundary. Neither
            # IK, deadman expiry, preparation nor reference recovery may change them.
            self.last_sim_seconds = float(sim_seconds)
            return self.cached_reference(sim_seconds)
        if self.last_sim_seconds is None:
            self.last_sim_seconds = sim_seconds
            return self.position.copy(), self.velocity.copy()
        dt = max(0.0, min(0.02, sim_seconds - self.last_sim_seconds))
        self.last_sim_seconds = sim_seconds
        if dt <= 0.0:
            return self.position.copy(), self.velocity.copy()

        action, stale = self.client.latest_action()
        if self.preparation.required:
            try:
                measured = self._actual_arm_position() if self._joint_state_provider is not None else np.full(6, np.nan)
                measured_velocity = (np.asarray(self._joint_velocity_provider(), dtype=float)
                                     if self._joint_velocity_provider is not None else np.full(6, np.nan))
            except (ValueError, TypeError, RuntimeError):
                measured = measured_velocity = np.full(6, np.nan)
            preparation_reference = self.preparation.step(
                dt, action, stale, self.position[:6], measured, measured_velocity,
                snapshot={"effort_ratio": self._effort_provider() if self._effort_provider else np.zeros(6)})
            if preparation_reference is not None:
                self.position[:6], self.velocity[:6] = preparation_reference
                self.velocity[6:] = 0.
                self.commanded_linear_velocity.fill(0.)
                self.commanded_angular_velocity.fill(0.)
                self.commanded_joint6_velocity = 0.0
                self.raw_operator_twist.fill(0.)
                self.desired_twist.fill(0.)
                self.achieved_twist.fill(0.)
                self.residual_twist.fill(0.)
                self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])
                self.actual_tool_position, self.actual_tool_rotation = self.kinematics.forward(measured) if np.all(np.isfinite(measured)) else (self.target_tool_position.copy(), self.target_tool_rotation.copy())
                self.position_error = self.target_tool_position - self.actual_tool_position
                self.orientation_error = rotation_matrix_to_vector(self.target_tool_rotation @ self.actual_tool_rotation.T)
                self.elbow_deviation.reset()
                self.elbow_diagnostics = ElbowStepDiagnostics(status="preparation")
                self.solver_status = "preparation_" + self.preparation.status
                self.solver_reasons = []
                self.solve_time_ms = 0.
                self.applied_sequence = str(action.get("server_sequence", "0"))
                self.command_stale = stale
                self.update_count += 1
                return self.position.copy(), self.velocity.copy()
        enabled = bool(action.get("deadman")) and not stale
        requested_linear = np.asarray(
            action["end_effector_linear_velocity_body_m_s"], dtype=float
        )
        requested_angular = np.asarray(
            action["end_effector_angular_velocity_body_rad_s"], dtype=float
        )
        actual_arm = self._actual_arm_position()
        self.actual_tool_position, self.actual_tool_rotation = self.kinematics.forward(actual_arm)
        requested_angular = requested_angular + self.actual_tool_rotation @ np.asarray(
            action.get("end_effector_angular_velocity_tool_rad_s", [0.0] * 3), dtype=float
        )
        requested_joint6 = float(action.get("joint6_velocity_rad_s", 0.0))
        self.raw_operator_twist = np.concatenate((requested_linear, requested_angular))
        requested_gripper = float(
            action.get("gripper_velocity_m_s", action.get("gripper_velocity_rad_s", 0.0))
        )
        if self.linkage_gripper:
            requested_gripper = float(action.get('gripper_drive_velocity_rad_s') or 0.) if action.get('gripper_type') == 'four_bar_tendon' else 0.
        # The packet deadman authorizes both channels, but a gripper action must
        # never enable arm IK. Numerical zero only: not a new joystick dead zone.
        cartesian_enabled = enabled and bool(np.any(np.abs(self.raw_operator_twist) > 1.0e-14))
        joint6_enabled = enabled and abs(requested_joint6) > 1.0e-14
        arm_enabled = cartesian_enabled or joint6_enabled
        gripper_enabled = enabled and abs(requested_gripper) > 1.0e-14
        if cartesian_enabled:
            self.commanded_linear_velocity = self._approach_vector(
                self.commanded_linear_velocity,
                requested_linear,
                MAX_LINEAR_COMMAND_ACCELERATION_M_S2 * dt,
            )
            self.commanded_angular_velocity = self._approach_vector(
                self.commanded_angular_velocity,
                requested_angular,
                MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 * dt,
            )
        else:
            self.commanded_linear_velocity.fill(0.0)
            self.commanded_angular_velocity.fill(0.0)
        if joint6_enabled:
            if requested_joint6 * self.commanded_joint6_velocity < 0.0:
                self.commanded_joint6_velocity = 0.0
            self.commanded_joint6_velocity += float(np.clip(
                requested_joint6 - self.commanded_joint6_velocity,
                -MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 * dt,
                MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 * dt,
            ))
        else:
            self.commanded_joint6_velocity = 0.0
        operator_linear = self.commanded_linear_velocity
        operator_angular = self.commanded_angular_velocity

        self.desired_twist = np.concatenate((operator_linear, operator_angular))
        # First solve the original bounded DLS task, then apply a bounded
        # geometric preference (no initial/home joint reference). Measured
        # plant error remains diagnostic; this is not plant pose feedback.
        self.elbow_diagnostics = ElbowStepDiagnostics(
            enabled=self.elbow_preference.enabled and self.ik_mode == IK_MODE_IK_POSE,
            status="idle" if self.elbow_preference.enabled and self.ik_mode == IK_MODE_IK_POSE else "off",
            wrist_enabled=self.elbow_preference.wrist_enabled and self.elbow_preference.enabled
                and self.ik_mode == IK_MODE_IK_POSE,
            joint3_enabled=self.elbow_preference.joint3_enabled and self.elbow_preference.enabled
                and self.ik_mode == IK_MODE_IK_POSE,
            position_offset_m=self.elbow_diagnostics.position_offset_m,
            orientation_offset_rad=self.elbow_diagnostics.orientation_offset_rad,
        )
        if not cartesian_enabled:
            # Keep measured telemetry running below, but skip reference-chain
            # Jacobian/SVD/IK work. Geometry diagnostics retain the last sample.
            result = IkResult(
                joint_velocity_rad_s=np.zeros(6), achieved_twist=np.zeros(6),
                residual_twist=np.zeros(6), jacobian_rank=self.jacobian_rank,
                minimum_singular_value=self.minimum_singular_value,
                condition_number=self.condition_number,
            )
            self.solve_time_ms = 0.0
        else:
            solve_start = time.perf_counter()
            if self.ik_mode == IK_MODE_STRICT:
                result = self.kinematics.inverse_velocity_bounded(
                    self.position[:6],
                    self.desired_twist,
                    joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
                    joint_position_min=self.joint_min[:6],
                    joint_position_max=self.joint_max[:6],
                    dt=dt,
                )
            else:
                # Task-only DLS is the baseline; never pass a home reference.
                result = self.kinematics.inverse_velocity_ik_pose(
                    self.position[:6],
                    self.desired_twist,
                    joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
                    joint_position_min=self.joint_min[:6],
                    joint_position_max=self.joint_max[:6],
                    dt=dt,
                    base_damping=IK_POSE_BASE_DAMPING,
                    maximum_damping=IK_POSE_MAXIMUM_DAMPING,
                    singular_value_threshold=IK_POSE_SINGULAR_VALUE_THRESHOLD,
                )
                result, self.elbow_diagnostics = apply_online_elbow_preference(
                    self.kinematics, self.position[:6], self.desired_twist, result,
                    joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
                    joint_position_min=self.joint_min[:6], joint_position_max=self.joint_max[:6],
                    dt=dt, preference=self.elbow_preference, deviation_state=self.elbow_deviation,
                )
            self.solve_time_ms = (time.perf_counter() - solve_start) * 1000.0
            self.ik_solve_count += 1
        if joint6_enabled:
            jacobian = self.kinematics.jacobian(self.position[:6])
            self.raw_operator_twist += jacobian[:, 5] * requested_joint6
            self.desired_twist += jacobian[:, 5] * self.commanded_joint6_velocity
            result = add_joint6_command(
                result, jacobian, self.desired_twist, self.commanded_joint6_velocity,
                self.position[:6], ARM_JOINT_VELOCITY_LIMIT,
                self.joint_min[:6], self.joint_max[:6], dt,
            )
        self.solver_reasons = [dict(reason) for reason in getattr(result, "limit_reasons", ())]
        self.solver_status = (
            "holding" if not arm_enabled else
            "limited" if result.velocity_scale < 1.0 - 1e-6 else "tracking"
        )
        if self.elbow_diagnostics.status == "active":
            self.solver_reasons.append({"code": "elbow_preference", "joints": []})
        if arm_enabled and np.linalg.norm(result.residual_twist) > 1e-5:
            if result.damping > IK_POSE_BASE_DAMPING + 1e-8:
                self.solver_reasons.append({"code": "damped_task_error", "joints": []})
            if cartesian_enabled and result.minimum_singular_value < IK_POSE_SINGULAR_VALUE_THRESHOLD:
                self.solver_reasons.append({"code": "singularity_nearby", "joints": []})
            if not self.solver_reasons:
                self.solver_reasons.append({"code": "task_residual", "joints": []})
            if self.solver_status == "tracking":
                self.solver_status = "approximate"
        self.arm_effort_ratio = self._effort_provider() if self._effort_provider else np.zeros(6)
        governed = self.governor.apply(
            self.position[:6], actual_arm, result.joint_velocity_rad_s, dt,
            enabled=arm_enabled,
            effort_ratio=self.arm_effort_ratio,
            allow_recovery=(
                action.get("allow_reference_recovery") is True and not stale
                and not enabled and not action.get("reason")
                and requested_joint6 == 0.0
            ),
        )
        self.velocity[:6] = governed.velocity
        self.tracking_scale = governed.scale
        self.governor_state = governed.state
        self.governor_limited_joints = governed.limited_joints
        self.achieved_twist = (
            self.kinematics.jacobian(self.position[:6]) @ governed.velocity
            if governed.reference_rebased else result.achieved_twist * governed.scale
        )
        self.residual_twist = self.desired_twist - self.achieved_twist
        self.jacobian_rank = result.jacobian_rank
        self.velocity_scale = result.velocity_scale
        self.minimum_singular_value = result.minimum_singular_value
        self.condition_number = result.condition_number
        self.ik_damping = result.damping
        self.nullspace_correction_norm = result.nullspace_correction_norm
        if arm_enabled:
            self._advance_target(self.achieved_twist[:3], self.achieved_twist[3:], dt)
        self.position_error = self.target_tool_position - self.actual_tool_position
        self.orientation_error = rotation_matrix_to_vector(
            self.target_tool_rotation @ self.actual_tool_rotation.T
        )

        gripper_velocity = requested_gripper if gripper_enabled else 0.0
        if self.linkage_gripper:
            # Normalized operator intent has no legacy slider units.
            angular_command = action.get('gripper_drive_velocity_rad_s')
            gripper_velocity = max(-.2, min(.2, float(angular_command or 0.))) if gripper_enabled and action.get('gripper_type') == 'four_bar_tendon' else 0.
        self.velocity[6] = gripper_velocity
        self.velocity[7] = -gripper_velocity if self.linkage_gripper else gripper_velocity

        proposed_arm = self.position[:6] + self.velocity[:6] * dt
        joint_limited_arm = np.clip(proposed_arm, self.joint_min[:6], self.joint_max[:6])
        at_arm_limit = joint_limited_arm != proposed_arm
        self.velocity[:6][at_arm_limit] = 0.0
        self.position[:6] = joint_limited_arm
        if governed.reference_rebased:
            self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])
            self.position_error = self.target_tool_position - self.actual_tool_position
            self.orientation_error = rotation_matrix_to_vector(self.target_tool_rotation @ self.actual_tool_rotation.T)
        if self._joint_velocity_provider is not None:
            measured_velocity = np.asarray(self._joint_velocity_provider(), dtype=float)
            if measured_velocity.shape != (6,):
                raise ValueError("measured arm velocity must have six entries")
            measured_twist = self.kinematics.jacobian(actual_arm) @ measured_velocity
        else:
            # Never mislabel the kinematic prediction as a physical measurement.
            measured_velocity = np.full(6, np.nan)
            measured_twist = np.full(6, np.nan)
        self.actuator_diagnostics = self._actuator_state_provider() if self._actuator_state_provider is not None else None
        self.speed_monitor.update(sim_seconds, self.desired_twist, measured_twist, self.achieved_twist,
            enabled=arm_enabled, stale=stale, solver_reasons=self.solver_reasons,
            measured_joints=actual_arm, target_joints=self.position[:6],
            measured_joint_velocity=measured_velocity, target_joint_velocity=self.velocity[:6],
            actuator_evidence=self.actuator_diagnostics)

        proposed_gripper = self.position[6:] + self.velocity[6:] * dt
        clipped_gripper = np.clip(proposed_gripper, self.joint_min[6:], self.joint_max[6:])
        at_gripper_limit = clipped_gripper != proposed_gripper
        self.velocity[6:][at_gripper_limit] = 0.0
        self.position[6:] = clipped_gripper

        self.applied_sequence = str(action.get("server_sequence", "0"))
        self.command_stale = stale
        self.update_count += 1
        return self.position.copy(), self.velocity.copy()

    def cached_reference(self, _sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the held target without running input handling or IK."""

        return self.position.copy(), self.velocity.copy()

    def reference(self, sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Compatibility wrapper for callers that explicitly request an update."""

        return self.update(sim_seconds)

class CartesianIkControlModel(sysModel.SysModel):
    """Run endpoint IK on a scheduled BSK control task, outside MJScene RK stages."""

    def __init__(self, target: CartesianTeleopTarget, *, clock=None, stride: int = 1) -> None:
        super().__init__()
        self.ModelTag = "so101CartesianIkController"
        self.target = target
        self.clock = clock
        self.stride = stride

    def Reset(self, CurrentSimNanos: int) -> None:
        """Reset the held reference when the Basilisk simulation resets."""

        self.target.reset(CurrentSimNanos * 1.0e-9)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Hold targets between integer-divided physics steps (default: every 2)."""
        if self.clock is not None and self.clock.step_index % self.stride:
            return
        self.target.update(CurrentSimNanos * 1.0e-9)
