import numpy as np
import pytest

from simulation.near_field.orbital_guidance import vbar_frame, rotating_relative_state, rotating_acceleration
from simulation.near_field.navigation import admit_handoff
from simulation.orbital_rendezvous.reference import FlightProfile


@pytest.mark.parametrize('time_s', [0., 5120., 11120.])
def test_coorbiting_stationary_vbar_is_not_rejected_for_inertial_velocity(time_s):
    r, v, _ = FlightProfile().target(time_s)
    target = np.r_[r, v]
    frame, omega = vbar_frame(target)
    delta = frame @ [110., 0., 0.]
    relative_velocity = np.cross(omega, delta)
    assert np.linalg.norm(relative_velocity) > .05
    relative = np.r_[delta, relative_velocity]
    np.testing.assert_allclose(rotating_relative_state(target, relative), [110, 0, 0, 0, 0, 0], atol=1e-12)
    handoff = {
        'guidance_frame': 'orbital-vbar', 'source_kind': 'synthetic frame test',
        'gnss_estimates_eci': [target, target-relative], 'covariances': [np.eye(6), np.eye(6)],
        'stowed_joints': [],
        'state': {'bodies': [
            {'body_id': 'rendezvous/target', 'position_m': r, 'velocity_m_s': v,
             'attitude_wxyz': [0, 1, 0, 0], 'angular_velocity_rad_s': [0, 0, .02]},
            {'body_id': 'rendezvous/chaser', 'position_m': r-delta, 'velocity_m_s': v-relative_velocity,
             'attitude_wxyz': [1, 0, 0, 0], 'angular_velocity_rad_s': [0, 0, 0]},
        ]}}
    admit_handoff(handoff)
    handoff['state']['bodies'][1]['velocity_m_s'] = v-relative_velocity-frame[:, 0]*.3
    with pytest.raises(ValueError, match='rotating-frame speed'):
        admit_handoff(handoff)


def test_frame_preserves_distances_and_rotating_acceleration_matches_circular_motion():
    r, v, _ = FlightProfile().target(700.)
    frame, omega = vbar_frame(np.r_[r, v])
    np.testing.assert_allclose(frame.T @ frame, np.eye(3), atol=1e-14)
    assert np.linalg.det(frame) == pytest.approx(1)
    delta = frame @ [110., 0., 0.]
    acceleration = rotating_acceleration(omega, delta, np.zeros(3))
    np.testing.assert_allclose(acceleration, -np.dot(omega, omega)*delta, atol=1e-15)


def test_invalid_orbit_and_unknown_guidance_mode_do_not_enable_autonomy():
    with pytest.raises(ValueError):
        vbar_frame(np.zeros(6))
    from simulation.near_field.orbital_guidance import guidance_frame
    with pytest.raises(ValueError):
        guidance_frame({'guidance_frame': 'unvalidated'})
