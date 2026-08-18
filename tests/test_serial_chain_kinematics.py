from pathlib import Path

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics,
    rotation_matrix_to_vector,
)


MODEL = Path(__file__).resolve().parents[2] / "test" / "model" / "spacecraft_and_arm" / "assets" / "cubesat_so101_grasp" / "cubesat_so101_grasp.xml"
JOINTS = (
    "so101_shoulder_pan",
    "so101_shoulder_lift",
    "so101_elbow_flex",
    "so101_wrist_flex",
    "so101_wrist_roll",
)


def chain() -> SerialChainKinematics:
    return SerialChainKinematics.from_mjcf(
        MODEL,
        base_body="cubesat_bus",
        joint_names=JOINTS,
        tool_site="so101_gripperframe",
    )


def test_analytic_jacobian_matches_forward_kinematics_difference() -> None:
    kinematics = chain()
    q = np.array([0.15, -0.35, 0.25, -0.1, 0.2])
    position, rotation = kinematics.forward(q)
    analytic = kinematics.jacobian(q)
    epsilon = 1e-7
    numeric = np.zeros_like(analytic)
    for index in range(5):
        displaced = q.copy()
        displaced[index] += epsilon
        next_position, next_rotation = kinematics.forward(displaced)
        numeric[:3, index] = (next_position - position) / epsilon
        numeric[3:, index] = rotation_matrix_to_vector(next_rotation @ rotation.T) / epsilon
    assert np.allclose(analytic, numeric, atol=2e-6)
    assert np.linalg.matrix_rank(analytic, tol=1e-5) == 5


def test_damped_ik_projects_six_dimensional_command_to_five_joints() -> None:
    kinematics = chain()
    q = np.array([0.0, -0.1790243, 0.2159404, -0.0368382, 0.0])
    desired = np.array([0.02, -0.01, 0.015, 0.1, -0.05, 0.08])
    result = kinematics.inverse_velocity(
        q,
        desired,
        joint_velocity_limits=np.array([0.7, 0.7, 0.7, 0.9, 1.0]),
    )
    assert result.joint_velocity_rad_s.shape == (5,)
    assert result.jacobian_rank == 5
    assert np.linalg.norm(result.residual_twist) < np.linalg.norm(desired)
    assert np.linalg.norm(result.residual_twist) > 1e-7
