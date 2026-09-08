from pathlib import Path

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics,
    rotation_matrix_to_vector,
)


MODEL = Path(__file__).resolve().parents[2] / "model" / "SARM" / "platform" / "sarm_platform.xml"
JOINTS = tuple(f"joint{i}" for i in range(1, 7))


def chain() -> SerialChainKinematics:
    return SerialChainKinematics.from_mjcf(
        MODEL,
        base_body="cubesat_bus",
        joint_names=JOINTS,
        tool_site="sarm_ee",
    )


def test_analytic_jacobian_matches_forward_kinematics_difference() -> None:
    kinematics = chain()
    q = np.array([0.15, -0.35, 0.25, -0.1, 0.2, 0.3])
    position, rotation = kinematics.forward(q)
    analytic = kinematics.jacobian(q)
    epsilon = 1e-7
    numeric = np.zeros_like(analytic)
    for index in range(6):
        displaced = q.copy()
        displaced[index] += epsilon
        next_position, next_rotation = kinematics.forward(displaced)
        numeric[:3, index] = (next_position - position) / epsilon
        numeric[3:, index] = rotation_matrix_to_vector(next_rotation @ rotation.T) / epsilon
    assert np.allclose(analytic, numeric, atol=2e-6)
    assert np.linalg.matrix_rank(analytic, tol=1e-5) == 6


def test_damped_ik_handles_six_axis_sarm_at_singular_pregrasp() -> None:
    kinematics = chain()
    q = np.array([0.0, -0.1790243, 0.2159404, -0.0368382, 0.0, 0.0])
    desired = np.array([0.02, -0.01, 0.015, 0.1, -0.05, 0.08])
    result = kinematics.inverse_velocity(
        q,
        desired,
        joint_velocity_limits=np.array([0.7, 0.7, 0.7, 0.9, 1.0, 1.0]),
    )
    assert result.joint_velocity_rad_s.shape == (6,)
    assert result.jacobian_rank == 5
    assert np.linalg.norm(result.residual_twist) < np.linalg.norm(desired)
    assert np.linalg.norm(result.residual_twist) > 1e-7
