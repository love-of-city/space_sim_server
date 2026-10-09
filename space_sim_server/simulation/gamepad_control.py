"""Combine Cartesian IK and direct wrist input before joint-reference protection."""
from dataclasses import replace

import numpy as np

from simulation.serial_chain_kinematics import IkResult, _uniform_limit_reasons, _uniform_limit_scale


def add_joint6_command(result: IkResult, jacobian: np.ndarray, requested_twist: np.ndarray,
                       joint6_velocity: float, position: np.ndarray, velocity_limits: np.ndarray,
                       minimum: np.ndarray, maximum: np.ndarray, dt: float) -> IkResult:
    """Add wrist velocity, then bound the combined step, including simultaneous IK."""
    velocity = result.joint_velocity_rad_s.copy()
    velocity[5] += joint6_velocity
    lower = np.maximum(-velocity_limits, (minimum - position) / dt)
    upper = np.minimum(velocity_limits, (maximum - position) / dt)
    scale = _uniform_limit_scale(velocity, lower, upper)
    reasons = _uniform_limit_reasons(velocity, lower, upper, velocity_limits, scale)
    velocity *= scale
    achieved = jacobian @ velocity
    return replace(result, joint_velocity_rad_s=velocity, achieved_twist=achieved,
                   residual_twist=requested_twist - achieved,
                   velocity_scale=result.velocity_scale * scale,
                   limit_reasons=tuple(result.limit_reasons) + reasons)
