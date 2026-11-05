import copy
import numpy as np
import pytest

from simulation.policy_pose import pose_to_joint_target, validate_kinematics
from simulation.serial_chain_kinematics import rotation_matrix_to_vector
from test_serial_chain_kinematics import chain


def test_pose_target_hold_and_bounded_motion():
    k = chain()
    q = np.array([.15, -.35, .25, -.1, .2, .3])
    p, r = k.forward(q)
    row = np.r_[p, rotation_matrix_to_vector(r), .6]
    limits = np.array([.7, .7, .7, .9, 1., 1.])
    kwargs = dict(lower=np.full(6, -6.), upper=np.full(6, 6.), velocity_limits=limits, dt=.02)
    target = pose_to_joint_target(k, q, row, **kwargs)
    np.testing.assert_allclose(target[:6], q, atol=1e-10)
    assert target[6] == .6
    row[0] += .001
    target = np.asarray(pose_to_joint_target(k, q, row, **kwargs))
    assert np.all(np.abs(target[:6] - q) <= limits * .02 + 1e-10)
    moved, _ = k.forward(target[:6])
    assert np.linalg.norm(moved - row[:3]) < np.linalg.norm(p - row[:3])


def test_geometry_and_tool_frame_must_match():
    k = chain()
    declaration = dict(schema="serial_chain_fk_v1", reference_frame="cubesat_bus", tool_frame="sarm_ee",
        joint_names=list(k.joint_names), segments=[{key:value.tolist() if isinstance(value,np.ndarray) else value
            for key,value in vars(segment).items()} for segment in k.segments])
    validate_kinematics(k, declaration)
    bad = copy.deepcopy(declaration); bad["segments"][-1]["position"][0] += .01
    with pytest.raises(ValueError, match="geometry"):
        validate_kinematics(k, bad)
    bad = copy.deepcopy(declaration); bad["tool_frame"] = "wrist"
    with pytest.raises(ValueError, match="sarm_ee"):
        validate_kinematics(k, bad)
