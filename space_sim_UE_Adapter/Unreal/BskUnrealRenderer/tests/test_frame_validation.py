"""Fast fixed-shape rotation checks retain the original acceptance thresholds."""
import numpy as np
import pytest
from bsk_render_adapter.frames import dcm_to_quaternion_wxyz, mrp_to_dcm_b_from_n


def legacy_valid(r):
    return (r.shape == (3, 3) and np.all(np.isfinite(r))
            and np.allclose(r @ r.T, np.eye(3), atol=1e-9)
            and np.isclose(np.linalg.det(r), 1., atol=1e-9))


def test_fast_validation_matches_old_rules_at_tolerance_boundaries():
    rng = np.random.default_rng(230923)
    matrices = [np.diag([1., 1., -1.]), np.zeros((3, 3)), np.eye(2)]
    for _ in range(100):
        rotation = mrp_to_dcm_b_from_n(rng.normal(size=3))
        matrices.append(rotation)
        for scale in (1e-11, 1e-9, 1e-7, 1e-5, 1e-3):
            matrices.append(rotation + scale * rng.normal(size=(3, 3)))
    for delta in (4.999e-6, 5e-6, 5.001e-6, 1.00009e-5, 1.00011e-5):
        matrices.append(np.diag([1. + delta, 1., 1.]))
    for r in matrices:
        if legacy_valid(r):
            q = dcm_to_quaternion_wxyz(r)
            assert np.linalg.norm(q) == pytest.approx(1., abs=2e-15)
        else:
            with pytest.raises(ValueError): dcm_to_quaternion_wxyz(r)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_rotation_rejected(value):
    r = np.eye(3);r[0, 0] = value
    with pytest.raises(ValueError): dcm_to_quaternion_wxyz(r)
