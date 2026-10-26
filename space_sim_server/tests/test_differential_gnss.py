import copy
import numpy as np
import pytest

from space_arm_platform.gnss_config import GNSSConfig
from simulation.differential_gnss import DifferentialGNSS, JointOrbitFilter


def test_common_errors_cancel_with_the_reported_relative_covariance():
    sensor = DifferentialGNSS(GNSSConfig(), seed=18)
    truth = np.array([[6_878_137., 100., 0., 0., 7600., 0.], [6_878_137., 0., 0., 0., 7600., 0.]])
    packets = [sensor.measure(truth, t/10) for t in range(6000)]
    errors = np.array([p['relative'] - (truth[0]-truth[1]) for p in packets])
    expected = np.diag(packets[0]['relative_covariance'])
    np.testing.assert_allclose(errors.var(axis=0), expected, rtol=.07, atol=0)
    assert np.max(np.abs(errors.mean(axis=0)) / np.sqrt(expected)) < .05
    assert np.sqrt(expected[0]) < .031
    assert np.linalg.eigvalsh(packets[0]['covariance']).min() > 0


def test_distant_baselines_reduce_common_mode_cancellation():
    cfg = GNSSConfig()
    close, distant = DifferentialGNSS(cfg), DifferentialGNSS(cfg)
    truth = np.zeros((2, 6))
    near = close.measure(truth, 0)
    truth[0, 0] = 200_000
    far = distant.measure(truth, 0)
    assert far['correlation'] < near['correlation']
    assert far['relative_covariance'][0, 0] > 10 * near['relative_covariance'][0, 0]


def test_handoff_restores_receiver_rng_time_and_sequence_exactly():
    cfg = GNSSConfig(delay_s=.1)
    sensor = DifferentialGNSS(cfg)
    truth = np.zeros((2, 6))
    sensor.measure(truth, 10)
    state = copy.deepcopy(sensor.snapshot())
    resumed = DifferentialGNSS(cfg, seed=999)
    resumed.restore(state)
    a, b = sensor.measure(truth, 10.1), resumed.measure(truth, 10.1)
    np.testing.assert_array_equal(a['observations'], b['observations'])
    assert b['sequence'] == 2 and b['stamp'] == 10.1 and b['due'] == pytest.approx(10.2)
    with pytest.raises(ValueError, match='advance'):
        resumed.measure(truth, 10.1)
    with pytest.raises(ValueError, match='configuration changed'):
        DifferentialGNSS(GNSSConfig(delay_s=.2)).restore(state)


def test_joint_filter_retains_cross_covariance_and_rejects_old_packets():
    sensor, estimator = DifferentialGNSS(GNSSConfig()), JointOrbitFilter()
    truth = np.array([[6_878_137., 100., 0., 0., 7600., 0.], [6_878_137., 0., 0., 0., 7600., 0.]])
    packet = sensor.measure(truth, 0)
    estimator.update(packet, np.zeros((2, 3)))
    np.testing.assert_allclose(estimator.relative_covariance, packet['relative_covariance'], atol=1e-14)
    assert estimator.relative_covariance[0, 0] < .01 * estimator.covariance[0, 0]
    next_packet = sensor.measure(truth, .1)
    estimator.update(next_packet, np.zeros((2, 3)))
    assert np.linalg.eigvalsh(estimator.covariance).min() > 0
    with pytest.raises(ValueError, match='out-of-order'):
        estimator.update(packet, np.zeros((2, 3)))


@pytest.mark.parametrize('kwargs', [dict(delay_s=-1), dict(sample_rate_hz=True),
                                  dict(relative_position_sigma_m=2), dict(correlation_length_m=float('nan'))])
def test_invalid_receiver_contract_is_rejected(kwargs):
    with pytest.raises(ValueError):
        GNSSConfig(**kwargs)


def test_near_field_inherits_joint_uncertainty_and_receiver_state():
    from simulation.near_field.navigation import origin_navigation, SensorConfig, SensorSimulator
    config = GNSSConfig()
    receiver = DifferentialGNSS(config)
    truth = np.array([[6_878_137., 100., 0., 0., 7600., 0.], [6_878_137., 0., 0., 0., 7600., 0.]])
    packet = receiver.measure(truth, 5120)
    P = packet['covariance']
    handoff = {'source_kind': 'synthetic covariance-only test',
               'gnss_estimates_eci': packet['observations'],
               'covariances': [P[:6, :6], P[6:, 6:]], 'gnss_cross_covariance_eci': P[:6, 6:]}
    _, relative_P = origin_navigation(handoff)
    np.testing.assert_allclose(relative_P, packet['relative_covariance'], atol=1e-14)
    resumed = SensorSimulator(SensorConfig(receiver_config=config), receiver_state=receiver.snapshot(), epoch_s=5120)
    packets, _ = resumed.acquire(0, 'FarHandoff', truth[0]-truth[1], [0, 0, 110], np.eye(3), absolute_truth=truth)
    assert not any(p['source'] == 'gnss' for p in packets)
    packets, _ = resumed.acquire(.1, 'FarHandoff', truth[0]-truth[1], [0, 0, 110], np.eye(3), absolute_truth=truth)
    continued = receiver.measure(truth, 5120.1)
    observed = next(p for p in packets if p['source'] == 'gnss')
    np.testing.assert_array_equal(observed['z'], continued['relative'])
    handoff['gnss_cross_covariance_eci'] = 2 * P[:6, :6]
    with pytest.raises(ValueError, match='positive semidefinite'):
        origin_navigation(handoff)
