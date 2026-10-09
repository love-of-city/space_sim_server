from pathlib import Path

import numpy as np
import pytest

from simulation.serial_chain_kinematics import (
    SerialChainKinematics,
    rotation_matrix_to_vector,
)
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME


MODEL = Path(__file__).resolve().parents[1] / "model" / "SARM" / "platform" / "sarm_platform.xml"
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


def test_bounded_ik_tracks_all_translation_axes_at_balanced_home() -> None:
    kinematics = chain()
    q = np.asarray(BALANCED_TELEOP_HOME[:6], dtype=float)
    limits = np.array([0.7, 0.7, 0.7, 0.9, 1.0, 1.0])
    for axis in range(3):
        for direction in (-1.0, 1.0):
            desired = np.zeros(6)
            desired[axis] = direction * 0.05
            result = kinematics.inverse_velocity_bounded(
                q,
                desired,
                joint_velocity_limits=limits,
                joint_position_min=np.array([-3.1416] * 5 + [-6.2832]),
                joint_position_max=np.array([3.1416] * 5 + [6.2832]),
                dt=0.01,
            )
            assert result.velocity_scale == 1.0
            assert np.allclose(result.achieved_twist, desired, atol=1e-10)
            assert np.all(np.abs(result.joint_velocity_rad_s) <= limits + 1e-12)
            assert result.minimum_singular_value > 0.03


def test_bounded_ik_uses_one_scale_without_changing_cartesian_direction() -> None:
    kinematics = chain()
    q = np.asarray(BALANCED_TELEOP_HOME[:6], dtype=float)
    limits = np.array([0.7, 0.7, 0.7, 0.9, 1.0, 1.0])
    desired = np.array([0.20, 0.0, 0.0, 0.0, 0.0, 0.0])
    result = kinematics.inverse_velocity_bounded(
        q,
        desired,
        joint_velocity_limits=limits,
    )
    assert 0.0 < result.velocity_scale < 1.0
    assert np.allclose(result.achieved_twist, desired * result.velocity_scale, atol=1e-10)
    assert np.allclose(result.achieved_twist[3:], 0.0, atol=1e-10)
    assert np.max(np.abs(result.joint_velocity_rad_s) / limits) == pytest.approx(1.0)


def test_bounded_ik_holds_an_unachievable_strict_pose_task_at_singularity() -> None:
    kinematics = chain()
    q = np.array([0.0, -0.1790243, 0.2159404, -0.0368382, 0.0, 0.0])
    desired = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
    result = kinematics.inverse_velocity_bounded(
        q,
        desired,
        joint_velocity_limits=np.array([0.7, 0.7, 0.7, 0.9, 1.0, 1.0]),
    )
    assert result.jacobian_rank == 5
    assert result.velocity_scale == 0.0
    assert np.array_equal(result.joint_velocity_rad_s, np.zeros(6))
    assert np.array_equal(result.residual_twist, desired)


SINGULAR_PREGRASP = np.array([0.0, -0.1790243, 0.2159404, -0.0368382, 0.0, 0.0])
ARM_VELOCITY_LIMITS = np.array([0.7, 0.7, 0.7, 0.9, 1.0, 1.0])


def test_ik_pose_slows_down_instead_of_freezing_at_singular_pregrasp() -> None:
    kinematics = chain()
    desired = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
    strict = kinematics.inverse_velocity_bounded(
        SINGULAR_PREGRASP, desired, joint_velocity_limits=ARM_VELOCITY_LIMITS
    )
    assert np.array_equal(strict.joint_velocity_rad_s, np.zeros(6))

    result = kinematics.inverse_velocity_ik_pose(
        SINGULAR_PREGRASP, desired, joint_velocity_limits=ARM_VELOCITY_LIMITS
    )
    assert result.jacobian_rank == 5
    assert result.damping == pytest.approx(5.0e-2)
    assert result.velocity_scale == 1.0
    assert np.all(np.isfinite(result.joint_velocity_rad_s))
    assert np.max(np.abs(result.joint_velocity_rad_s)) < 0.1
    assert np.linalg.norm(result.achieved_twist) > 0.0
    assert np.linalg.norm(result.residual_twist) < np.linalg.norm(desired)


def test_ik_pose_keeps_full_task_accuracy_at_balanced_home() -> None:
    kinematics = chain()
    q = np.asarray(BALANCED_TELEOP_HOME[:6], dtype=float)
    desired = np.array([0.02, -0.01, 0.015, 0.1, -0.05, 0.08])
    result = kinematics.inverse_velocity_ik_pose(
        q, desired, joint_velocity_limits=ARM_VELOCITY_LIMITS
    )
    assert result.damping == pytest.approx(1.0e-3)
    assert result.minimum_singular_value > 2.0e-2
    assert result.velocity_scale == 1.0
    assert result.jacobian_rank == 6
    assert np.linalg.norm(result.residual_twist) / np.linalg.norm(desired) < 1.0e-3


def test_ik_pose_scales_every_joint_with_one_uniform_factor() -> None:
    kinematics = chain()
    q = np.asarray(BALANCED_TELEOP_HOME[:6], dtype=float)
    desired = np.array([0.20, 0.0, 0.0, 0.0, 0.0, 0.0])
    bounded = kinematics.inverse_velocity_ik_pose(
        q, desired, joint_velocity_limits=ARM_VELOCITY_LIMITS
    )
    reference = kinematics.inverse_velocity_ik_pose(
        q, desired, joint_velocity_limits=ARM_VELOCITY_LIMITS * 100.0
    )
    assert 0.0 < bounded.velocity_scale < 1.0
    assert reference.velocity_scale == 1.0
    assert np.allclose(
        bounded.joint_velocity_rad_s,
        reference.joint_velocity_rad_s * bounded.velocity_scale,
        atol=1.0e-12,
    )
    assert np.all(np.abs(bounded.joint_velocity_rad_s) <= ARM_VELOCITY_LIMITS + 1.0e-12)
    assert np.max(np.abs(bounded.joint_velocity_rad_s) / ARM_VELOCITY_LIMITS) == pytest.approx(1.0)


def test_ik_pose_posture_term_moves_only_inside_the_jacobian_nullspace() -> None:
    kinematics = chain()
    rest = np.asarray(BALANCED_TELEOP_HOME[:6], dtype=float)
    desired = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
    free = kinematics.inverse_velocity_ik_pose(
        SINGULAR_PREGRASP, desired, joint_velocity_limits=ARM_VELOCITY_LIMITS
    )
    posture = kinematics.inverse_velocity_ik_pose(
        SINGULAR_PREGRASP,
        desired,
        joint_velocity_limits=ARM_VELOCITY_LIMITS,
        nullspace_reference=rest,
        nullspace_gains=np.full(6, 0.15),
    )
    assert posture.nullspace_correction_norm > 1.0e-3
    assert not np.allclose(posture.joint_velocity_rad_s, free.joint_velocity_rad_s)
    assert np.allclose(posture.achieved_twist, free.achieved_twist, atol=1.0e-9)
    assert np.all(np.abs(posture.joint_velocity_rad_s) <= ARM_VELOCITY_LIMITS + 1.0e-12)


def test_ik_pose_without_posture_reference_holds_a_zero_command() -> None:
    kinematics = chain()
    result = kinematics.inverse_velocity_ik_pose(
        SINGULAR_PREGRASP, np.zeros(6), joint_velocity_limits=ARM_VELOCITY_LIMITS
    )
    assert np.array_equal(result.joint_velocity_rad_s, np.zeros(6))
    assert result.nullspace_correction_norm == 0.0
    assert np.array_equal(result.residual_twist, np.zeros(6))


def test_ik_pose_rejects_invalid_damping_and_posture_inputs() -> None:
    kinematics = chain()
    desired = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        kinematics.inverse_velocity_ik_pose(
            SINGULAR_PREGRASP,
            desired,
            joint_velocity_limits=ARM_VELOCITY_LIMITS,
            base_damping=0.10,
            maximum_damping=0.01,
        )
    with pytest.raises(ValueError):
        kinematics.inverse_velocity_ik_pose(
            SINGULAR_PREGRASP,
            desired,
            joint_velocity_limits=ARM_VELOCITY_LIMITS,
            singular_value_threshold=0.0,
        )
    with pytest.raises(ValueError):
        kinematics.inverse_velocity_ik_pose(
            SINGULAR_PREGRASP,
            desired,
            joint_velocity_limits=ARM_VELOCITY_LIMITS,
            nullspace_reference=np.zeros(3),
        )
    with pytest.raises(ValueError):
        kinematics.inverse_velocity_ik_pose(
            SINGULAR_PREGRASP,
            desired,
            joint_velocity_limits=ARM_VELOCITY_LIMITS,
            nullspace_reference=np.zeros(6),
            nullspace_gains=np.full(6, -0.1),
        )
