"""Deterministic endpoint selection; this must not alter live differential IK."""
from pathlib import Path

import numpy as np
import pytest

from simulation.pose_ik import (
    ElbowPreference, _rotation_error, elbow_height, select_elbow_preferred_pose,
)
from simulation.serial_chain_kinematics import SerialChainKinematics, axis_angle_to_matrix
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
from space_arm_platform.joint_limits import load_joint_limits

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'model/SARM/platform/sarm_platform.xml'
NAMES = tuple(f'joint{i}' for i in range(1, 7))


@pytest.fixture(scope='module')
def setup():
    chain = SerialChainKinematics.from_mjcf(MODEL, base_body='cubesat_bus', joint_names=NAMES, tool_site='sarm_ee')
    q = np.array(BALANCED_TELEOP_HOME[:6])
    return chain, q, load_joint_limits(MODEL, NAMES)


def solve(setup, **kwargs):
    chain, q, limits = setup
    opts = dict(joint_position_min=limits.lower, joint_position_max=limits.upper,
                configuration_is_valid=lambda q: True)
    opts.update(kwargs)
    return select_elbow_preferred_pose(chain, *chain.forward(q), q, **opts)


def test_joint_centres_define_geometric_elbow_not_joint_sign(setup):
    chain, q, _ = setup
    origins = chain.joint_origins(q)
    assert origins.shape == (6, 3)
    assert elbow_height(chain, q) == pytest.approx(0.22416191778)
    assert elbow_height(chain, q, ElbowPreference(up_axis=(0, 0, -1))) == pytest.approx(-0.22416191778)
    assert elbow_height(chain, q, ElbowPreference(up_axis=(0, 0, 5))) == pytest.approx(0.22416191778)
    with pytest.raises(ValueError):
        chain.joint_origins(np.full(6, np.nan))


def test_prefers_higher_accurate_branch_and_is_deterministic(setup):
    chain, q, _ = setup
    result = solve(setup)
    c = result.candidate
    assert c is not None
    assert result.attempted_seeds == 19
    assert c.elbow_height_m > elbow_height(chain, q) + 0.15
    assert c.position_error_m <= 1e-4
    assert c.orientation_error_rad <= 1e-3
    assert c.minimum_singular_value > 0.02
    # The preferred SARM elbow-high branch has NEGATIVE q3: no UR5 sign rule.
    assert c.joint_position_rad[2] < 0
    np.testing.assert_array_equal(c.joint_position_rad, solve(setup).candidate.joint_position_rad)
    np.testing.assert_array_equal(q, np.asarray(BALANCED_TELEOP_HOME[:6]))


def test_zero_preference_keeps_current_branch(setup):
    c = solve(setup, preference=ElbowPreference(elbow_weight=0)).candidate
    np.testing.assert_array_equal(c.joint_position_rad, setup[1])


def test_collision_rejection_can_fall_back_to_lower_elbow(setup):
    chain, q, _ = setup
    result = solve(setup, configuration_is_valid=lambda candidate: elbow_height(chain, candidate) < 0.3)
    assert result.rejected_collision >= 1
    np.testing.assert_array_equal(result.candidate.joint_position_rad, q)
    assert solve(setup, seeds=[], configuration_is_valid=lambda _: False).candidate is None


def test_unreachable_target_not_accepted_as_low_error_score(setup):
    chain, q, limits = setup
    result = select_elbow_preferred_pose(chain, np.array([10., 10., 10.]), np.eye(3), q,
        joint_position_min=limits.lower, joint_position_max=limits.upper,
        configuration_is_valid=lambda _: True, seeds=[])
    assert result.candidate is None


def test_narrow_bounds_keep_legal_current_branch(setup):
    _, q, _ = setup
    result = solve(setup, joint_position_min=q-.01, joint_position_max=q+.01)
    np.testing.assert_array_equal(result.candidate.joint_position_rad, q)


def test_pi_rotation_error_is_not_spuriously_zero():
    axis = np.array([1., 2., 3.]); axis /= np.linalg.norm(axis)
    r = axis_angle_to_matrix(axis, np.pi)
    error = _rotation_error(r)
    assert np.linalg.norm(error) == pytest.approx(np.pi)
    np.testing.assert_allclose(axis_angle_to_matrix(error, np.linalg.norm(error)), r, atol=1e-10)


@pytest.mark.parametrize('change', [dict(up_axis=(0,0,0)), dict(elbow_weight=-1),
    dict(length_scale_m=0), dict(position_tolerance_m=float('nan')), dict(max_iterations=0)])
def test_invalid_preferences_rejected(change):
    with pytest.raises(ValueError):
        ElbowPreference(**change)


def test_invalid_targets_and_seeds_rejected(setup):
    with pytest.raises(ValueError):
        solve(setup, seeds=[np.full(6, np.nan)])
    with pytest.raises(ValueError):
        solve(setup, configuration_is_valid=None)


def test_selected_endpoint_has_no_arm_contacts_in_actual_default_scene():
    pytest.importorskip('mujoco')
    from simulation.prepare_elbow_posture import prepare, SceneConfigurationValidator
    path = ROOT / 'model/SARM/platform/sarm_ground_target_self_collision.xml'
    randomized = dict(arm_joint_position_rad=list(BALANCED_TELEOP_HOME),
        target_position_m=[.94, .039086, .4], target_orientation_wxyz=[1,0,0,0], target_hinge_position_rad=0)
    result = prepare(path, randomized)
    assert result['status'] == 'selected'
    assert result['selected_elbow_height_m'] > result['original_elbow_height_m'] + .15
    chain = SerialChainKinematics.from_mjcf(path,base_body='cubesat_bus',joint_names=NAMES,tool_site='sarm_ee')
    validator = SceneConfigurationValidator(path, randomized, chain)
    assert validator(np.array(result['joint_position_rad']))
    # A deliberately folded pose must be rejected, not only counted in metadata.
    assert not validator(np.array([0.07427746, 2.83034688, -2.23581109, 2.81894761, -1.18229786, -0.48175413]))
