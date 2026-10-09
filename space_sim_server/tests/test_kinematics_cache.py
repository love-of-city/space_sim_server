"""Regression coverage for shared, value-keyed kinematics geometry."""
from pathlib import Path

import numpy as np
import pytest

from simulation.serial_chain_kinematics import (
    Segment, SerialChainKinematics, _transform, _translation, axis_angle_to_matrix,
)


def legacy_geometry(chain, q):
    """Uncached pre-optimization algorithm, including nonzero hinge offsets."""
    transform = np.eye(4)
    origins = np.zeros((len(q), 3))
    axes = np.zeros_like(origins)
    for segment in chain.segments:
        transform = transform @ _transform(segment.position, segment.rotation)
        if segment.joint_index is not None:
            i = segment.joint_index
            origins[i] = (transform @ np.array([*segment.joint_position, 1.]))[:3]
            axes[i] = transform[:3, :3] @ segment.joint_axis
            transform = (transform @ _translation(segment.joint_position)
                         @ _transform(np.zeros(3), axis_angle_to_matrix(segment.joint_axis, q[i]))
                         @ _translation(-segment.joint_position))
    jac = np.zeros((6, len(q)))
    for i in range(len(q)):
        jac[:3, i] = np.cross(axes[i], transform[:3, 3] - origins[i])
        jac[3:, i] = axes[i]
    return transform, origins, axes, jac


def chain_fixture():
    rng = np.random.default_rng(42)
    segments = []
    for i in range(6):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        segments.append(Segment(rng.normal(size=3), axis_angle_to_matrix(axis, .3),
                                i, rng.normal(size=3), axis))
    segments.append(Segment(np.array([.1, .2, .3]), np.eye(3)))
    return SerialChainKinematics(segments, tuple(f"joint{i + 1}" for i in range(6)))


def test_shared_geometry_matches_uncached_chain_with_offsets_exactly():
    chain = chain_fixture()
    for q in np.random.default_rng(12).uniform(-2, 2, (100, 6)):
        transform, origins, axes, jac = legacy_geometry(chain, q)
        position, rotation = chain.forward(q)
        np.testing.assert_array_equal(position, transform[:3, 3])
        np.testing.assert_array_equal(rotation, transform[:3, :3])
        np.testing.assert_array_equal(chain.joint_geometry(q), (origins, axes))
        np.testing.assert_array_equal(chain.jacobian(q), jac)
        assert len(chain._geometry_cache) <= 8


def test_cache_keys_values_and_returns_independent_arrays():
    chain = chain_fixture()
    q = np.zeros(6)
    position, rotation = chain.forward(q)
    original_position, original_rotation = position.copy(), rotation.copy()
    expected_jac = chain.jacobian(q)
    position[:] = 999
    rotation[:] = 0
    chain.joint_origins(q)[:] = 999
    origins, axes = chain.joint_geometry(q)
    origins[:] = axes[:] = 0
    chain.jacobian(q)[:] = 999
    np.testing.assert_array_equal(chain.forward(q)[0], original_position)
    np.testing.assert_array_equal(chain.forward(q)[1], original_rotation)
    np.testing.assert_array_equal(chain.jacobian(q), expected_jac)
    # The controller mutates the SAME array each step: object identity is unsafe.
    q[1] = .4
    np.testing.assert_array_equal(chain.jacobian(q), legacy_geometry(chain, q)[3])
    assert not np.array_equal(chain.forward(q)[0], original_position)


@pytest.mark.parametrize("invalid", [np.zeros(5), np.full(6, np.nan), np.full(6, np.inf)])
def test_bad_geometry_cannot_enter_cache(invalid):
    chain = chain_fixture()
    for query in (chain.forward, chain.jacobian, chain.joint_geometry):
        with pytest.raises(ValueError):
            query(invalid)
    assert not chain._geometry_cache
