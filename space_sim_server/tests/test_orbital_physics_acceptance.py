"""Physics acceptance using the same native Earth model as production."""
import pytest

pytest.importorskip("Basilisk")

from tools.verify_orbital_physics import check_fields, propagate


def test_native_j2_matches_independent_cartesian_acceleration():
    check_fields()


@pytest.mark.parametrize("mode", ["point_mass", "j2"])
def test_two_orbit_invariants_and_node_regression(mode):
    propagate(mode, dt=1.0, periods=2)
