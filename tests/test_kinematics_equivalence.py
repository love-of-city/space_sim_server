"""Exact geometry optimizations must preserve the original matrix-chain math."""
from pathlib import Path
import numpy as np
import pytest
from simulation.serial_chain_kinematics import SerialChainKinematics, axis_angle_to_matrix, _transform, _translation

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_platform.xml"


def reference_geometry(chain, q):
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
    jacobian = np.vstack((np.cross(axes, transform[:3, 3] - origins).T, axes.T))
    return transform, origins, axes, jacobian


@pytest.mark.parametrize("up", [(0., 0., 1.), (.2, -.7, .4)])
def test_randomized_geometry_and_gradients_match_scalar_reference(up):
    chain = SerialChainKinematics.from_mjcf(MODEL, base_body="cubesat_bus",
        joint_names=tuple(f"joint{i}" for i in range(1, 7)), tool_site="sarm_ee")
    rng = np.random.default_rng(231126)
    axis = np.asarray(up) / np.linalg.norm(up)
    for q in rng.uniform(-np.pi, np.pi, (80, 6)):
        expected = reference_geometry(chain, q)
        for actual, want in zip(chain._geometry(q), expected):
            np.testing.assert_allclose(actual, want, rtol=0., atol=2e-14)
        for shoulder, elbow in ((1, 2), (5, 3), (0, 5), (3, 3)):
            grad = np.zeros(6)
            _, origins, axes, _ = expected
            for i in range(6):
                if i < elbow: grad[i] += axis @ np.cross(axes[i], origins[elbow] - origins[i])
                if i < shoulder: grad[i] -= axis @ np.cross(axes[i], origins[shoulder] - origins[i])
            height, actual = chain.relative_joint_height(q, shoulder_joint=f"joint{shoulder+1}",
                elbow_joint=f"joint{elbow+1}", up_axis=up)
            assert height == pytest.approx(float(axis @ (origins[elbow] - origins[shoulder])), abs=2e-14)
            np.testing.assert_allclose(actual, grad, rtol=0., atol=2e-14)
