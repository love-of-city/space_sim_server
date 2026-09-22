"""Online geometric arch preference with bounded EXTRA task disturbance.

Elbow-up, wrist-down and negative joint3 share one correction to a limited DLS
step. It is not a global IK branch switch, a collision planner, or home tracking.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from simulation.serial_chain_kinematics import (
    IkResult, SerialChainKinematics, matrix_to_quaternion_wxyz,
)


@dataclass(frozen=True)
class OnlineElbowPreference:
    enabled: bool = True
    up_axis: tuple[float, float, float] = (0., 0., 1.)
    shoulder_joint: str = "joint2"
    elbow_joint: str = "joint3"
    preferred_height_m: float = 0.30
    height_gain_s: float = 1.0
    maximum_height_speed_m_s: float = 0.04
    posture_weight: float = 4.0
    # Positive drop = joint6 lies below joint4 in the same body-frame up axis.
    wrist_enabled: bool = True
    wrist_upper_joint: str = "joint4"
    wrist_lower_joint: str = "joint6"
    preferred_wrist_drop_m: float = 0.05
    wrist_height_gain_s: float = 1.0
    maximum_wrist_drop_speed_m_s: float = 0.02
    wrist_weight: float = 4.0
    # A soft angle preference, NOT a change to the model's mechanical limits.
    joint3_enabled: bool = True
    joint3_negative_margin_rad: float = math.radians(5.0)
    joint3_angle_scale_rad: float = 1.0
    joint3_length_scale_m: float = 0.20
    joint3_gain_s: float = 1.0
    maximum_joint3_preference_speed_rad_s: float = 0.05
    joint3_weight: float = 4.0
    regularization: float = 0.05
    maximum_correction_rad_s: float = 0.15
    maximum_linear_disturbance_m_s: float = 0.002
    maximum_angular_disturbance_rad_s: float = 0.01
    relative_disturbance: float = 0.10
    characteristic_length_m: float = 0.54
    nominal_command_speed_m_s: float = 0.05
    maximum_position_offset_m: float = 0.002
    maximum_orientation_offset_rad: float = math.pi / 180.0

    def __post_init__(self):
        if any(not isinstance(value, bool) for value in (self.enabled, self.wrist_enabled, self.joint3_enabled)):
            raise ValueError("enabled, wrist_enabled and joint3_enabled must be boolean")
        for first, second in ((self.shoulder_joint, self.elbow_joint),
                              (self.wrist_upper_joint, self.wrist_lower_joint)):
            if not isinstance(first, str) or not isinstance(second, str) or not first or not second or first == second:
                raise ValueError("each height pair must name two distinct joints")
        axis = np.asarray(self.up_axis, dtype=float)
        if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-12:
            raise ValueError("up_axis must be finite and nonzero")
        if not math.isfinite(self.preferred_height_m):
            raise ValueError("preferred_height_m must be finite")
        for name in ("height_gain_s", "maximum_height_speed_m_s", "posture_weight",
                     "regularization", "maximum_correction_rad_s", "characteristic_length_m",
                     "maximum_position_offset_m", "maximum_orientation_offset_rad", "nominal_command_speed_m_s",
                     "preferred_wrist_drop_m", "wrist_height_gain_s", "maximum_wrist_drop_speed_m_s", "wrist_weight",
                     "joint3_negative_margin_rad", "joint3_angle_scale_rad", "joint3_length_scale_m", "joint3_gain_s",
                     "maximum_joint3_preference_speed_rad_s", "joint3_weight"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name in ("maximum_linear_disturbance_m_s", "maximum_angular_disturbance_rad_s",
                     "relative_disturbance"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative and finite")
        if self.relative_disturbance > 1:
            raise ValueError("relative_disturbance must not exceed 1")


@dataclass(frozen=True)
class ElbowStepDiagnostics:
    enabled: bool = False
    status: str = "off"
    height_m: float = 0.
    predicted_height_m: float = 0.
    wrist_enabled: bool = False
    wrist_drop_m: float = 0.
    predicted_wrist_drop_m: float = 0.
    elbow_objective_active: bool = False
    wrist_objective_active: bool = False
    joint3_enabled: bool = False
    joint3_rad: float = 0.
    predicted_joint3_rad: float = 0.
    joint3_objective_active: bool = False
    base_posture_cost: float = 0.
    predicted_posture_cost: float = 0.
    correction_norm_rad_s: float = 0.
    linear_disturbance_m_s: float = 0.
    angular_disturbance_rad_s: float = 0.
    linear_budget_m_s: float = 0.
    angular_budget_rad_s: float = 0.
    scale: float = 0.
    position_offset_m: float = 0.
    orientation_offset_rad: float = 0.


@dataclass
class ElbowDeviationState:
    """Accumulate ONLY the extra reference-motion contribution of preference.

    Advance an anchor with the task-only FK increment at each CURRENT q. Thus
    without a correction the existing offset norm is unchanged. This is NOT a
    counterfactual full trajectory from a second arm, nor measured plant error.
    Keep this state across ticks and deadman releases; reset only with the scene.
    """
    position: np.ndarray | None = None
    rotation: np.ndarray | None = None

    def reset(self):
        self.position = self.rotation = None

    def advance(self, chain, q, base_q):
        p, r = chain.forward(q)
        bp, br = chain.forward(base_q)
        if self.position is None:
            self.position, self.rotation = bp.copy(), br.copy()
        else:
            self.position += bp - p
            self.rotation = br @ r.T @ self.rotation
        return bp, br


def _angle(rotation):
    quat = matrix_to_quaternion_wxyz(rotation)
    return 2.0 * math.atan2(float(np.linalg.norm(quat[1:])), abs(float(quat[0])))


def _ball_step_scale(offset, increment, radius):
    """Largest nonnegative step inside a ball (linearized finite-step offset)."""
    a = float(increment @ increment)
    if a <= 1e-30:
        return 1.
    b = float(offset @ increment)
    remaining = max(0., radius**2 - float(offset @ offset))
    return max(0., min(1., (-b + math.sqrt(b*b + a*remaining))/a))


def apply_online_elbow_preference(
    chain: SerialChainKinematics, q: np.ndarray, desired_twist: np.ndarray, base: IkResult,
    *, joint_velocity_limits: np.ndarray, joint_position_min: np.ndarray,
    joint_position_max: np.ndarray, dt: float,
    preference: OnlineElbowPreference = OnlineElbowPreference(),
    deviation_state: ElbowDeviationState | None = None,
) -> tuple[IkResult, ElbowStepDiagnostics]:
    """Preserve the base solve if no beneficial, admissible correction exists.

    Disturbance is relative to the already bounded task-only step, not a promise
    that a singular/unreachable operator command can be tracked. Both J*dq and
    finite-step FK are checked. Live callers must keep a deviation_state across
    ticks to bound accumulated extra reference offsets as well. These budgets
    do not bound original DLS tracking error or actual physical plant error.
    """
    if not preference.enabled:
        return base, ElbowStepDiagnostics()
    q, twist = np.asarray(q, float), np.asarray(desired_twist, float)
    speed = np.asarray(joint_velocity_limits, float)
    low, high = np.asarray(joint_position_min, float), np.asarray(joint_position_max, float)
    n = len(chain.joint_names)
    if (any(a.shape != (n,) for a in (q, speed, low, high)) or twist.shape != (6,)
            or not np.all(np.isfinite(q)) or not np.all(np.isfinite(twist))
            or not np.all(np.isfinite(speed)) or np.any(speed <= 0)
            or np.any(np.isnan(low)) or np.any(np.isnan(high)) or np.any(low > high)
            or np.any(q < low) or np.any(q > high)
            or not math.isfinite(dt) or dt <= 0):
        raise ValueError("invalid online elbow step inputs")
    height, gradient = chain.relative_joint_height(q, shoulder_joint=preference.shoulder_joint,
        elbow_joint=preference.elbow_joint, up_axis=preference.up_axis)
    if preference.wrist_enabled:
        # relative_joint_height returns second-minus-first: p4 - p6.
        wrist_drop, wrist_gradient = chain.relative_joint_height(q,
            shoulder_joint=preference.wrist_lower_joint, elbow_joint=preference.wrist_upper_joint,
            up_axis=preference.up_axis)
    else:
        wrist_drop, wrist_gradient = 0., np.zeros(n)
    joint3_index = chain.joint_names.index("joint3") if preference.joint3_enabled else None
    joint3_angle = float(q[joint3_index]) if preference.joint3_enabled else 0.
    # Convert normalized angle error into a length-equivalent scalar, so the
    # angle goal is not added unscaled to the metre-valued height goals.
    joint3_scale = preference.joint3_length_scale_m / preference.joint3_angle_scale_rad
    diag = ElbowStepDiagnostics(enabled=True, status="idle", height_m=height, predicted_height_m=height,
        wrist_enabled=preference.wrist_enabled, wrist_drop_m=wrist_drop, predicted_wrist_drop_m=wrist_drop,
        joint3_enabled=preference.joint3_enabled, joint3_rad=joint3_angle, predicted_joint3_rad=joint3_angle)
    # Defense in depth: even direct calls may not move an idle arm.
    if not np.any(twist != 0):
        return base, diag
    velocity = np.asarray(base.joint_velocity_rad_s, float)
    if velocity.shape != (n,) or not np.all(np.isfinite(velocity)):
        raise ValueError("base velocity must be finite and match the chain")
    lower = np.maximum(-speed, (low-q)/dt)
    upper = np.minimum(speed, (high-q)/dt)
    if np.any(velocity < lower-1e-10) or np.any(velocity > upper+1e-10):
        raise ValueError("base IK step must already satisfy joint bounds")
    base_q = q + dt * velocity
    state = deviation_state if deviation_state is not None else ElbowDeviationState()
    base_p, base_r = state.advance(chain, q, base_q)
    diag = replace(diag, position_offset_m=float(np.linalg.norm(base_p-state.position)),
                   orientation_offset_rad=_angle(base_r @ state.rotation.T))
    up = np.array(preference.up_axis, dtype=float, copy=True)
    up /= np.linalg.norm(up)
    shoulder = chain.joint_names.index(preference.shoulder_joint)
    elbow = chain.joint_names.index(preference.elbow_joint)
    wrist_upper = chain.joint_names.index(preference.wrist_upper_joint) if preference.wrist_enabled else None
    wrist_lower = chain.joint_names.index(preference.wrist_lower_joint) if preference.wrist_enabled else None
    def shape_values(joints):
        origins = chain.joint_origins(joints)
        elbow_h = float(up @ (origins[elbow]-origins[shoulder]))
        drop = float(up @ (origins[wrist_upper]-origins[wrist_lower])) if preference.wrist_enabled else 0.
        return elbow_h, drop
    def shape_cost(elbow_h, drop, joints):
        cost = preference.posture_weight * max(0., preference.preferred_height_m-elbow_h)**2
        if preference.wrist_enabled:
            cost += preference.wrist_weight * max(0., preference.preferred_wrist_drop_m-drop)**2
        if preference.joint3_enabled:
            angle_error = joint3_scale * max(0., float(joints[joint3_index]) + preference.joint3_negative_margin_rad)
            cost += preference.joint3_weight * angle_error**2
        return cost
    base_h, base_drop = shape_values(base_q)
    base_cost = shape_cost(base_h, base_drop, base_q)
    diag = replace(diag, predicted_height_m=base_h, predicted_wrist_drop_m=base_drop,
                   base_posture_cost=base_cost, predicted_posture_cost=base_cost,
                   predicted_joint3_rad=float(base_q[joint3_index]) if preference.joint3_enabled else 0.)
    goals = [(height, gradient, preference.preferred_height_m,
              preference.height_gain_s, preference.maximum_height_speed_m_s, preference.posture_weight)]
    if preference.wrist_enabled:
        goals.append((wrist_drop, wrist_gradient, preference.preferred_wrist_drop_m,
                      preference.wrist_height_gain_s, preference.maximum_wrist_drop_speed_m_s,
                      preference.wrist_weight))
    if preference.joint3_enabled:
        angle_gradient = np.zeros(n)
        angle_gradient[joint3_index] = -joint3_scale
        # At q3=0 this has a nonzero desired rate. Once q3<=-margin, no drive.
        # No wrapping: respect actual model coordinates even for multi-turn axes.
        goals.append((-joint3_scale * joint3_angle, angle_gradient,
                      joint3_scale * preference.joint3_negative_margin_rad,
                      preference.joint3_gain_s,
                      joint3_scale * preference.maximum_joint3_preference_speed_rad_s,
                      preference.joint3_weight))
    if all(value >= target for value, _, target, _, _, _ in goals):
        return base, replace(diag, status="shape_satisfied" if preference.wrist_enabled or preference.joint3_enabled else "height_satisfied")
    terms = []
    active = []
    for value, grad, target, gain, max_rate, weight in goals:
        desired_rate = min(max_rate, gain * max(0., target-value))
        deficit = desired_rate - float(grad @ velocity)
        use = bool(value < target and deficit > 0 and np.linalg.norm(grad) >= 1e-12)
        active.append(use)
        if use:
            terms.append((grad, deficit, weight))
    diag = replace(diag, elbow_objective_active=active[0],
                   wrist_objective_active=active[1] if preference.wrist_enabled else False,
                   joint3_objective_active=active[-1] if preference.joint3_enabled else False)
    if not terms:
        return base, replace(diag, status="task_already_improves_shape" if preference.wrist_enabled or preference.joint3_enabled
                            else "task_already_raises_elbow")
    jacobian = chain.jacobian(q)
    # ONE coupled correction, not sequential elbow/wrist/angle adjustments:
    # ||W J c||^2 + reg^2||c||^2 + sum_i weight_i*(gradient_i @ c-deficit_i)^2.
    # A satisfied goal adds no drive. Actual finite-step hinge-cost acceptance
    # below still counts every enabled goal, so worsening a satisfied goal is
    # not silently ignored. Soft conflicts trade off in one total shape score.
    weighted_j = jacobian.copy()
    weighted_j[3:] *= preference.characteristic_length_m
    metric = weighted_j.T @ weighted_j + preference.regularization**2 * np.eye(n)
    rhs = np.zeros(n)
    for grad, deficit, weight in terms:
        metric += weight * np.outer(grad, grad)
        rhs += weight * deficit * grad
    correction = np.linalg.solve(metric, rhs)
    if np.linalg.norm(correction) < 1e-12:
        return base, replace(diag, status="no_correction")

    # Cross-channel disturbance uses one equivalent command magnitude; a tiny
    # command cannot cause a fixed large posture motion. No idle floor.
    equivalent_speed = float(np.hypot(np.linalg.norm(twist[:3]),
        preference.characteristic_length_m * np.linalg.norm(twist[3:])))
    linear_budget = min(preference.maximum_linear_disturbance_m_s,
                        preference.relative_disturbance * equivalent_speed)
    angular_budget = min(preference.maximum_angular_disturbance_rad_s,
                         preference.relative_disturbance * equivalent_speed / preference.characteristic_length_m)
    diag = replace(diag, linear_budget_m_s=linear_budget, angular_budget_rad_s=angular_budget)
    activity = min(1., equivalent_speed / preference.nominal_command_speed_m_s)
    scale = min(1., preference.maximum_correction_rad_s * activity / float(np.max(np.abs(correction))))
    for value, v, lo, hi in zip(correction, velocity, lower, upper):
        if value > 1e-14:
            scale = min(scale, max(0., float((hi-v)/value)))
        elif value < -1e-14:
            scale = min(scale, max(0., float((lo-v)/value)))
    disturbance = jacobian @ correction
    for norm, budget in ((np.linalg.norm(disturbance[:3]),linear_budget),
                         (np.linalg.norm(disturbance[3:]),angular_budget)):
        if norm > 0:
            scale = min(scale, float(budget/norm))
    if scale <= 1e-12:
        return base, replace(diag, status="limited")

    # Predict the remaining cumulative offset budget before expensive FK
    # backtracking; do not retry many times when the budget is already spent.
    position_offset = base_p-state.position
    quat = matrix_to_quaternion_wxyz(base_r @ state.rotation.T)
    sine = float(np.linalg.norm(quat[1:]))
    orientation_offset = quat[1:] * (2. if sine < 1e-12 else _angle(base_r @ state.rotation.T)/sine)
    for offset, change, radius in (
        (position_offset, dt*disturbance[:3], preference.maximum_position_offset_m),
        (orientation_offset, dt*disturbance[3:], preference.maximum_orientation_offset_rad),
    ):
        if radius-np.linalg.norm(offset) < 1e-8 and float(offset @ change) > 0:
            return base, replace(diag, status="limited")
        allowed = _ball_step_scale(offset, change, radius)
        if allowed < scale:
            scale = .95 * allowed
    if scale <= 1e-10:
        return base, replace(diag, status="limited")
    # Bounded backtracking, not a global search or per-tick branch switch.
    for _ in range(4):
        step = velocity + scale * correction
        next_q = q + dt * step
        next_p, next_r = chain.forward(next_q)
        next_h, next_drop = shape_values(next_q)
        next_cost = shape_cost(next_h, next_drop, next_q)
        linear = max(float(np.linalg.norm(jacobian[:3] @ (scale*correction))),
                     float(np.linalg.norm(next_p-base_p))/dt)
        angular = max(float(np.linalg.norm(jacobian[3:] @ (scale*correction))),
                      _angle(next_r @ base_r.T)/dt)
        if (np.all(next_q >= low-1e-12) and np.all(next_q <= high+1e-12)
                and linear <= linear_budget+1e-12 and angular <= angular_budget+1e-12
                and np.linalg.norm(next_p-state.position) <= preference.maximum_position_offset_m+1e-12
                and _angle(next_r @ state.rotation.T) <= preference.maximum_orientation_offset_rad+1e-12
                and next_cost < base_cost-1e-12):
            achieved = jacobian @ step
            return replace(base, joint_velocity_rad_s=step, achieved_twist=achieved,
                           residual_twist=twist-achieved), replace(diag,
                status="active", predicted_height_m=next_h,
                predicted_wrist_drop_m=next_drop, predicted_posture_cost=next_cost,
                predicted_joint3_rad=float(next_q[joint3_index]) if preference.joint3_enabled else 0.,
                correction_norm_rad_s=float(np.linalg.norm(scale*correction)),
                linear_disturbance_m_s=linear, angular_disturbance_rad_s=angular, scale=scale,
                position_offset_m=float(np.linalg.norm(next_p-state.position)),
                orientation_offset_rad=_angle(next_r @ state.rotation.T))
        scale *= .5
    return base, replace(diag, status="limited")
