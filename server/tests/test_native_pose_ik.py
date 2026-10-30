"""Compare the batched IK with the independent Python reference and target pose."""
from pathlib import Path

import numpy as np
import pytest

from simulation.native_posture import native_solve_pose
from simulation.scripted_tasks.motion import _solve_pose_python, solve_pose
from simulation.serial_chain_kinematics import SerialChainKinematics
from space_arm_platform.joint_limits import load_joint_limits


@pytest.fixture
def arm():
    model = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_platform.xml"
    names = tuple(f"joint{i}" for i in range(1, 7))
    chain = SerialChainKinematics.from_mjcf(model, base_body="cubesat_bus", joint_names=names, tool_site="sarm_ee")
    limits = load_joint_limits(model, names)
    return chain, np.array(limits.lower), np.array(limits.upper)


def test_random_reachable_poses_and_controller_dispatch(arm):
    chain, low, high = arm
    rng = np.random.default_rng(20261028)
    for i in range(150):
        target = rng.uniform(low + .03, high - .03)
        p, r = chain.forward(target)
        seed = np.clip(target + rng.normal(0, .04 if i < 100 else .2, 6), low, high)
        try:
            expected = _solve_pose_python(chain, p, r, seed, low, high)
        except ValueError:
            with pytest.raises(ValueError):
                native_solve_pose(chain, p, r, seed, low, high)
            continue
        actual = native_solve_pose(chain, p, r, seed, low, high)
        np.testing.assert_allclose(actual, expected, atol=2e-8, rtol=2e-8)
        position, rotation = chain.forward(actual)
        assert np.linalg.norm(position - p) < 2e-6
        assert np.linalg.norm(rotation - r) < 1.5e-5
        assert np.all((actual >= low) & (actual <= high))
    chain.enable_native_geometry()
    np.testing.assert_allclose(solve_pose(chain, p, r, seed, low, high), actual)


def test_unreachable_target_and_nonfinite_inputs(arm):
    chain, low, high = arm
    for p in (np.array([10., 10., 10.]), np.array([np.nan, 0., 0.])):
        with pytest.raises(ValueError):
            native_solve_pose(chain, p, np.eye(3), np.zeros(6), low, high)
