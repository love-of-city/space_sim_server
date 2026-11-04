"""Independent Kepler fixtures and statistical checks of the GNSS ROE interface."""
import json

import numpy as np
import pytest

from simulation.differential_gnss import DifferentialGNSS
from simulation.gnss_transport import DelayedGNSSNavigation
from simulation.relative_navigation import navigation_solution, near_navigation_solution, osculating_roe
from simulation.orbital_rendezvous.reference import MU
from space_arm_platform.gnss_config import GNSSConfig


def kepler(a=6_878_137., e=0., i=.9, node=.4, perigee=.7, mean=1.2):
    """Independent classical element -> Cartesian construction; no production conversion."""
    anomaly = mean
    for _ in range(20):
        anomaly -= (anomaly - e*np.sin(anomaly) - mean) / (1-e*np.cos(anomaly))
    root = np.sqrt(1-e*e)
    r = a*np.array([np.cos(anomaly)-e, root*np.sin(anomaly), 0.])
    v = np.sqrt(MU/a)/(1-e*np.cos(anomaly))*np.array([-np.sin(anomaly), root*np.cos(anomaly), 0.])
    co, so, cw, sw, ci, si = np.cos(node), np.sin(node), np.cos(perigee), np.sin(perigee), np.cos(i), np.sin(i)
    rotation = np.array([[co*cw-so*sw*ci, -co*sw-so*cw*ci, so*si],
                         [so*cw+co*sw*ci, -so*sw+co*cw*ci, -co*si], [sw*si, cw*si, ci]])
    return np.r_[rotation @ r, rotation @ v]


@pytest.mark.parametrize("eccentricity", [0., 1e-12, .001, .1, .6])
def test_elements_match_independent_classical_definition(eccentricity):
    chief = dict(a=6_878_137., e=eccentricity, i=.9, node=.4, perigee=.7, mean=1.2)
    deputy = dict(a=6_878_137.+80, e=eccentricity+.00002, i=.90003, node=.40004, perigee=.72, mean=1.19)
    values, chief_a = osculating_roe([kepler(**deputy), kepler(**chief)])
    expected = [80/chief['a'], -.01+.02+.00004*np.cos(.9),
                deputy['e']*np.cos(.72)-chief['e']*np.cos(.7),
                deputy['e']*np.sin(.72)-chief['e']*np.sin(.7), .00003, .00004*np.sin(.9)]
    np.testing.assert_allclose(values, expected, atol=3e-15, rtol=1e-9)
    assert chief_a == pytest.approx(chief['a'], abs=1e-7)


def test_circular_orbit_does_not_depend_on_undefined_perigee():
    chief = kepler(perigee=0, mean=1)
    target = kepler(perigee=2, mean=-1+1e-5)
    actual, _ = osculating_roe([target, chief])
    np.testing.assert_allclose(actual, [0, 1e-5, 0, 0, 0, 0], atol=2e-15)


def test_node_and_longitude_wrap_with_finite_covariance():
    states = [kepler(node=-np.pi+1e-8, perigee=0, mean=-np.pi+1e-8),
              kepler(node=np.pi-1e-8, perigee=0, mean=np.pi-1e-8)]
    solution = navigation_solution(states, np.eye(12)*.001, 12, reference_point='assembled_center_of_mass', source='test posterior')
    values = solution['roe']['values']
    assert values[1] == pytest.approx(2e-8*(1+np.cos(.9)), abs=2e-15)
    assert values[5] == pytest.approx(2e-8*np.sin(.9), abs=2e-15)
    assert np.max(np.diag(solution['roe']['covariance'])) < 1e-8


def test_covariance_matches_monte_carlo_with_correlated_receiver_errors():
    states = np.array([kepler(mean=1.20002), kepler()])
    sensor = DifferentialGNSS(GNSSConfig())
    packet = sensor.measure(states, 0)
    solution = navigation_solution(states, packet['covariance'], 0, reference_point='assembled_center_of_mass', source='joint filter')
    errors = np.random.default_rng(800).multivariate_normal(np.zeros(12), packet['covariance'], size=16000)
    samples, _ = osculating_roe(states + errors.reshape(-1, 2, 6))
    observed = np.cov(samples.T)
    expected = np.array(solution['roe']['covariance'])
    np.testing.assert_allclose(np.diag(observed), np.diag(expected), rtol=.04, atol=0)
    scales = np.sqrt(np.outer(np.diag(expected), np.diag(expected)))
    assert np.max(np.abs(observed-expected) / scales) < .04
    np.testing.assert_allclose(solution['covariance'], packet['relative_covariance'], atol=1e-16)
    independent_P = np.array(packet['covariance'])
    independent_P[:6, 6:] = independent_P[6:, :6] = 0
    independent = navigation_solution(states, independent_P, 0, reference_point='assembled_center_of_mass', source='test')
    # Default velocity residuals dominate some ROE components; cancellation
    # need not have the same large factor as Cartesian position covariance.
    assert np.trace(independent['roe']['covariance']) > np.trace(expected)
    assert solution['attitude']['available'] is False
    assert solution['roe']['chief'] == 'chaser' and solution['roe']['deputy'] == 'target'
    json.dumps(solution, allow_nan=False)


def test_delay_output_uses_delivered_posterior_and_current_prediction_epoch():
    channel = DelayedGNSSNavigation(GNSSConfig(delay_s=.2), seed=19)
    states = np.array([kepler(mean=1.20002), kepler()])
    assert channel.step(states, 0, np.zeros((2, 3))) is None
    channel.step(states, .1, np.zeros((2, 3)))
    channel.step(states, .2, np.zeros((2, 3)))
    solution = navigation_solution(channel.estimate, channel.covariance, channel.now,
                                   reference_point='assembled_center_of_mass', source='predicted GNSS')
    assert channel.last_packet['stamp'] == 0
    assert solution['epoch_s'] == .2
    np.testing.assert_allclose(solution['position_m'], channel.estimate[0, :3]-channel.estimate[1, :3])
    assert not np.allclose(solution['position_m'], states[0, :3]-states[1, :3], rtol=0, atol=1e-4)


def test_near_phase_matches_handoff_reference_and_covariance():
    target, chaser = kepler(mean=1.20002), kepler()
    relative_P = np.diag([.03**2]*3+[.0015**2]*3)
    solution = near_navigation_solution(target, target-chaser, np.eye(6)*.3, relative_P, 5120.2)
    np.testing.assert_allclose(solution['covariance'], relative_P, atol=1e-16)
    np.testing.assert_allclose(solution['position_m'], target[:3]-chaser[:3])
    expected, _ = osculating_roe([target, chaser])
    np.testing.assert_allclose(solution['roe']['values'], expected, atol=1e-15)
    assert solution['reference_point'] == 'body_origin'
    assert 'approximation' in solution['covariance_model']


@pytest.mark.parametrize("orbit", [dict(i=0), dict(i=np.pi), dict(i=1e-7)])
def test_equatorial_geometry_marks_roe_unavailable_without_losing_cartesian(orbit):
    states = [kepler(**orbit), kepler(mean=1.20002, **orbit)]
    solution = navigation_solution(states, np.eye(12), 0, reference_point='body_origin', source='test')
    assert not solution['roe']['available']
    assert 'equatorial' in solution['roe']['reason']
    assert np.isfinite(solution['position_m']).all()
    json.dumps(solution, allow_nan=False)


@pytest.mark.parametrize("kind", ['radial', 'unbound', 'zero'])
def test_non_orbital_geometry_does_not_publish_fake_roe(kind):
    state = kepler()
    if kind == 'radial':
        state[3:] = state[:3]
    elif kind == 'unbound':
        state[3:] *= 2
    else:
        state[:] = 0
    solution = navigation_solution([state, state], np.eye(12), 0, reference_point='body_origin', source='test')
    assert not solution['roe']['available']
    json.dumps(solution, allow_nan=False)


@pytest.mark.parametrize("kind", ['nan', 'negative', 'asymmetric', 'shape', 'time'])
def test_bad_covariance_or_epoch_is_rejected(kind):
    P, time = np.eye(12), 0.
    if kind == 'nan':
        P[0, 0] = np.nan
    elif kind == 'negative':
        P[0, 0] = -1
    elif kind == 'asymmetric':
        P[0, 1] = 1
    elif kind == 'shape':
        P = np.eye(6)
    else:
        time = -1
    with pytest.raises(ValueError, match='invalid navigation'):
        navigation_solution([kepler(), kepler()], P, time, reference_point='body_origin', source='test')
