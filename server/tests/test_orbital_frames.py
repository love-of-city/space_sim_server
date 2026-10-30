"""Frame interpolation is independently testable without Basilisk or MuJoCo."""
import numpy as np
import pytest

from simulation.orbital_frames import interpolate_origin


def test_hermite_reproduces_cubic_motion():
    r0 = np.array([2., 4., 6.])
    velocity = np.array([1., -3., 2.])
    acceleration = np.array([.1, .5, -.3])
    jerk = np.array([.04, -.03, .08])
    dt = .02
    position = lambda t: r0 + velocity*t + acceleration*t*t/2 + jerk*t**3/6
    v1 = velocity + acceleration*dt + jerk*dt*dt/2
    for fraction in [0., .1, .5, .9, 1.]:
        np.testing.assert_allclose(interpolate_origin(r0, velocity, position(dt), v1, dt, fraction),
                                   position(dt*fraction), atol=1e-14, rtol=0)


@pytest.mark.parametrize("dt,fraction", [(0, 0), (-1, .5), (.01, -1), (.01, 1.1),
                                        (float("nan"), .5), (.01, float("inf"))])
def test_invalid_interval_is_rejected(dt, fraction):
    zero = np.zeros(3)
    with pytest.raises(ValueError, match="origin sampling"):
        interpolate_origin(zero, zero, zero, zero, dt, fraction)
