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


def test_maneuvering_body_origin_does_not_look_like_a_lost_gnss_link():
    from simulation.near_field.navigation import RelativeEKF
    rng = np.random.default_rng(18)
    estimator = RelativeEKF([110, 0, 0, 0, .05, 0], np.eye(6), acceleration_sigma_m_s2=.05)
    sigmas = np.array([.03]*3 + [.002]*3)
    for tick in range(601):
        t = tick*.05
        estimator.predict(.05 if tick else 0, np.zeros(3), np.zeros((3, 3)))
        if tick % 2 == 0:
            # A 0.5 m lever arm rotating at 0.1 rad/s produces a real moving
            # body origin even when its assembly COM has no acceleration.
            truth = np.array([110+.5*(np.cos(.1*t)-1), .5*np.sin(.1*t), 0,
                              -.05*np.sin(.1*t), .05*np.cos(.1*t), 0])
            estimator.update({'source':'gnss','stamp':t,'z':truth+rng.normal(size=6)*sigmas,
                              'R':np.diag(sigmas**2)}, t)
    assert estimator.accepted['gnss'] >= 295
    assert np.linalg.norm(estimator.x[:3]-truth[:3]) < .05
    assert not estimator.update({'source':'gnss','stamp':30.1,'z':truth+1000,
                                 'R':np.diag(sigmas**2)}, 30.1)


def test_settled_navigation_resolves_stationary_hold_without_relaxing_velocity_limit():
    from simulation.near_field.navigation import RelativeEKF
    rng = np.random.default_rng(31)
    estimator = RelativeEKF(np.zeros(6), np.eye(6), acceleration_sigma_m_s2=.003)
    errors = []
    for tick in range(2001):
        t = tick*.05
        estimator.predict(.05 if tick else 0, np.zeros(3), np.zeros((3, 3)))
        if tick % 2 == 0:
            for source, sigmas in [('gnss', np.array([.03]*3+[.002]*3)),
                                   ('stereo', np.full(3, .0007))]:
                estimator.update({'source':source, 'stamp':t, 'z':rng.normal(size=len(sigmas))*sigmas,
                                  'R':np.diag(sigmas**2)}, t)
        if t > 50:
            errors.append(np.linalg.norm(estimator.x[3:]))
    assert np.sqrt(np.mean(np.square(errors))) < .001
    assert np.quantile(errors, .99) < .002
    assert estimator.accepted['gnss'] > 990
