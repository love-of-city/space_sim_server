"""Sensor statistics, clocks and isolation without a dynamics dependency."""
import copy
import math

import numpy as np
import pytest

from simulation.gyroscope import GYRO_PERIOD_NS, GyroConfig, Gyroscope


def ideal_config(**overrides):
    return GyroConfig(**(dict(white_noise_std_rad_s=0., initial_bias_std_rad_s=0.,
                             bias_random_walk_rad_s_sqrt_s=0.) | overrides))


def test_ideal_sensor_reads_all_axes_without_modifying_truth():
    truth = np.array([.01, -.02, .03])
    sensor = Gyroscope(ideal_config())
    reading = sensor.sample(0, truth)
    assert reading['angular_velocity_body_rad_s'] == truth.tolist()
    assert reading['error_variance_rad2_s2'] == [0.] * 3
    assert reading['frame'] == 'spacecraft_body'
    assert reading['reference_frame'] == 'J2000'
    assert reading['valid']
    assert np.array_equal(truth, [.01, -.02, .03])
    assert not any('truth' in k or 'bias' in k or 'orientation' in k for k in reading)


def test_repeated_reads_and_duplicate_ticks_do_not_resample_or_mutate_sensor():
    a, b = Gyroscope(seed=123), Gyroscope(seed=123)
    for tick in range(30):
        stamp = tick * GYRO_PERIOD_NS
        expected = b.sample(stamp, [.01, .02, .03])
        assert a.sample(stamp, [.01, .02, .03]) == expected
        for _ in range(4):
            row = a.latest
            row['angular_velocity_body_rad_s'][0] = 900
            assert a.sample(stamp, [.01, .02, .03]) == expected
    a.reset()
    b.reset()
    assert a.sample(0, [0.] * 3) == b.sample(0, [0.] * 3)
    assert a.latest != Gyroscope(seed=124).sample(0, [0.] * 3)


def test_seed_does_not_advance_global_numpy_randomness():
    before = copy.deepcopy(np.random.get_state())
    sensor = Gyroscope(seed=90)
    sensor.sample(0, [0.] * 3)
    after = np.random.get_state()
    assert before[0] == after[0]
    assert np.array_equal(before[1], after[1])
    assert before[2:] == after[2:]


def test_white_noise_mean_variance_and_axis_independence():
    sigma = .002
    sensor = Gyroscope(ideal_config(white_noise_std_rad_s=sigma), seed=515)
    rows = np.array([sensor.sample(i * GYRO_PERIOD_NS, [0.] * 3)[
        'angular_velocity_body_rad_s'] for i in range(12000)])
    assert np.max(np.abs(rows.mean(axis=0))) < 5 * sigma / math.sqrt(len(rows))
    assert np.allclose(rows.var(axis=0), sigma**2, rtol=.06)
    covariance = np.cov(rows.T)
    assert np.max(np.abs(covariance - np.diag(np.diag(covariance)))) < .04 * sigma**2


def test_random_walk_increments_scale_with_physical_interval():
    walk = .002
    sensor = Gyroscope(ideal_config(bias_random_walk_rad_s_sqrt_s=walk), seed=51)
    rows = np.array([sensor.sample(i * GYRO_PERIOD_NS, [0.] * 3)[
        'angular_velocity_body_rad_s'] for i in range(12000)])
    assert np.allclose(np.diff(rows, axis=0).var(axis=0), walk**2 / 50, rtol=.06)
    assert sensor.latest['error_variance_rad2_s2'] == pytest.approx([walk**2 * 11999 / 50] * 3)


def test_initial_bias_is_constant_with_no_white_noise_or_random_walk():
    sensor = Gyroscope(ideal_config(initial_bias_std_rad_s=.001), seed=8)
    initial = sensor.sample(0, [0.] * 3)
    for tick in range(1, 20):
        reading = sensor.sample(tick * GYRO_PERIOD_NS, [0.] * 3)
        assert reading['angular_velocity_body_rad_s'] == initial['angular_velocity_body_rad_s']
    assert initial['error_variance_rad2_s2'] == [1e-6] * 3
    assert initial['angular_velocity_body_rad_s'] != [0.] * 3


def test_saturation_is_per_axis_and_invalid_for_control():
    reading = Gyroscope(ideal_config(range_rad_s=.1)).sample(0, [.2, -.3, .05])
    assert reading['angular_velocity_body_rad_s'] == [.1, -.1, .05]
    assert reading['saturated_axes'] == [True, True, False]
    assert not reading['valid']


@pytest.mark.parametrize('stamp', [-1, True, .02, 1, 40_000_000])
def test_bad_first_timestamps_are_rejected(stamp):
    with pytest.raises(ValueError):
        Gyroscope().sample(stamp, [0.] * 3)


@pytest.mark.parametrize('truth', [[0, 0], [0, 0, np.nan], [np.inf, 0, 0]])
def test_invalid_rates_do_not_advance_random_state(truth):
    a, b = Gyroscope(seed=1), Gyroscope(seed=1)
    with pytest.raises(ValueError):
        a.sample(0, truth)
    assert a.sample(0, [0.] * 3) == b.sample(0, [0.] * 3)


def test_missing_or_reversed_samples_require_explicit_reset():
    sensor = Gyroscope()
    sensor.sample(0, [0.] * 3)
    with pytest.raises(ValueError):
        sensor.sample(40_000_000, [0.] * 3)
    sensor.sample(20_000_000, [0.] * 3)
    with pytest.raises(ValueError):
        sensor.sample(0, [0.] * 3)
    sensor.reset()
    assert sensor.latest is None
    assert sensor.sample(0, [0.] * 3)['sample_time_ns'] == '0'


@pytest.mark.parametrize('config', [None, [], {'unknown': 1}, {'enabled': 1},
    {'white_noise_std_rad_s': -1}, {'initial_bias_std_rad_s': float('nan')},
    {'range_rad_s': 0}, {'bias_random_walk_rad_s_sqrt_s': True}])
def test_bad_configuration_is_rejected(config):
    with pytest.raises(ValueError):
        GyroConfig.from_mapping(config)


@pytest.mark.parametrize('seed', [-1, True, 1.2, 2**64])
def test_bad_seed_is_rejected(seed):
    with pytest.raises(ValueError):
        Gyroscope(seed=seed)


def test_disabled_sensor_has_no_measurement():
    sensor = Gyroscope(GyroConfig(enabled=False))
    assert sensor.sample(0, [0.] * 3) is None
    assert sensor.latest is None
