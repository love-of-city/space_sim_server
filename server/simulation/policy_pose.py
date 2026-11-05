"""Cartesian policy targets through the existing bounded SARM IK controller."""
import numpy as np

from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector


def validate_kinematics(chain, declaration):
    if not isinstance(declaration, dict) or declaration.get("schema") != "serial_chain_fk_v1":
        raise ValueError("Cartesian policy requires explicit serial-chain geometry")
    if declaration.get("reference_frame") != "cubesat_bus" or declaration.get("tool_frame") != "sarm_ee":
        raise ValueError("Cartesian policy must target sarm_ee in cubesat_bus coordinates")
    if declaration.get("joint_names") != list(chain.joint_names):
        raise ValueError("Cartesian policy joint order differs from the active scene")
    segments = [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in vars(s).items()}
                for s in chain.segments]
    if declaration.get("segments") != segments:
        raise ValueError("Cartesian policy geometry differs from the active scene")


def pose_to_joint_target(chain, measured, action, *, lower, upper, velocity_limits, dt):
    """One pose target, one bounded differential IK update; no hidden reset."""
    q = np.asarray(measured, float)
    row = np.asarray(action, float)
    if q.shape != (6,) or row.shape != (7,) or not np.isfinite(row).all() or not np.isfinite(q).all():
        raise ValueError("Cartesian target/state must be finite and correctly sized")
    p, r = chain.forward(q)
    angle = np.linalg.norm(row[3:6])
    goal_r = np.eye(3) if angle < 1e-14 else axis_angle_to_matrix(row[3:6] / angle, angle)
    twist = np.concatenate((row[:3] - p, rotation_matrix_to_vector(goal_r @ r.T))) / dt
    solved = chain.inverse_velocity_ik_pose(
        q, twist, joint_velocity_limits=velocity_limits,
        joint_position_min=lower, joint_position_max=upper, dt=dt)
    target = np.clip(q + solved.joint_velocity_rad_s * dt, lower, upper)
    return target.tolist() + [float(row[6])]
