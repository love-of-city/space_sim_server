import copy

import numpy as np
import pytest

from space_arm_platform.gnss_config import GNSSConfig
from simulation.differential_gnss import DifferentialGNSS, JointOrbitFilter
from simulation.gnss_transport import DelayedGNSSNavigation
from simulation.orbital_rendezvous.reference import MU


def orbit(t):
    radius = 6_878_137.
    rate = np.sqrt(MU/radius**3)
    state = np.array([radius*np.cos(rate*t), radius*np.sin(rate*t), 0.,
                      -radius*rate*np.sin(rate*t), radius*rate*np.cos(rate*t), 0.])
    return np.array([state, state + [0., -110., 0., 0., 0., 0.]])


def test_delayed_prediction_uses_configured_field_at_measurement_epochs():
    from simulation.gnss_transport import _propagate
    class Field:
        def __init__(self):
            self.times = []
        def acceleration(self, position, time_s):
            self.times.append(time_s)
            return np.array([.2, -.1, .05])
        def gradient(self, position, time_s):
            return np.zeros((3, 3))
    field = Field()
    initial = orbit(0.)
    control = np.array([[.1, 0., 0.], [0., .1, 0.]])
    state, covariance = _propagate(initial, np.eye(12), .25, control, gravity_model=field, time_s=12.)
    a = control + [.2, -.1, .05]
    np.testing.assert_allclose(state[:, :3], initial[:, :3] + .25*initial[:, 3:] + .5*.25**2*a)
    np.testing.assert_allclose(state[:, 3:], initial[:, 3:] + .25*a)
    assert field.times == [12., 12.25, 12., 12.25]
    assert np.linalg.eigvalsh(covariance).min() > 0


def test_transport_never_uses_a_packet_before_its_due_time():
    nav = DelayedGNSSNavigation(GNSSConfig(delay_s=.25))
    acceleration = np.zeros((2, 3))
    for tick in range(5):
        assert nav.step(orbit(tick*.05), tick*.05, acceleration) is None
        assert nav.last_packet is None
    estimate = nav.step(orbit(.25), .25, acceleration)
    assert estimate is not None
    assert nav.last_packet['stamp'] == 0
    assert nav.last_packet['due'] == .25
    assert nav.telemetry()['gnss_delivered_sequence'] == 1
    assert nav.telemetry()['gnss_acquired_sequence'] == 3
    assert nav.telemetry()['gnss_measurement_age_s'] == .25
    assert np.linalg.norm(estimate[0, :3]-orbit(.25)[0, :3]) < 5
    # The delivered raw position is roughly 1.9 km behind the current orbit.
    assert np.linalg.norm(nav.last_packet['observations'][0, :3]-estimate[0, :3]) > 1800


def test_future_truth_cannot_change_current_prediction():
    a = DelayedGNSSNavigation(GNSSConfig(delay_s=.2), seed=31)
    b = DelayedGNSSNavigation(GNSSConfig(delay_s=.2), seed=31)
    acceleration = np.zeros((2, 3))
    for tick in range(9):
        now = tick*.05
        truth_a, truth_b = orbit(now), orbit(now)
        if now >= .3:
            truth_b[1, 0] += 100_000
        first, second = a.step(truth_a, now, acceleration), b.step(truth_b, now, acceleration)
        if first is not None:
            np.testing.assert_array_equal(first, second)
            np.testing.assert_array_equal(a.covariance, b.covariance)


@pytest.mark.parametrize('rate,delay', [(1, .25), (2, .15), (5, .25), (10, .12), (20, .05)])
def test_delayed_posteriors_equal_on_time_processing_at_their_measurement_epoch(rate, delay):
    on_time = DelayedGNSSNavigation(GNSSConfig(sample_rate_hz=rate), seed=14)
    delayed = DelayedGNSSNavigation(GNSSConfig(sample_rate_hz=rate, delay_s=delay), seed=14)
    reference = {}
    for tick in range(100):
        now = tick*.05
        control = np.array([[0., 0., 0.], [.002*np.sin(now), .001*np.cos(now), 0.]])
        on_time.step(orbit(now), now, control)
        reference[on_time.filter.last_stamp] = (on_time.filter.state.copy(), on_time.filter.covariance.copy())
        delayed.step(orbit(now), now, control)
        if delayed.filter.state is not None:
            expected_state, expected_covariance = reference[delayed.filter.last_stamp]
            np.testing.assert_array_equal(delayed.filter.state, expected_state)
            np.testing.assert_array_equal(delayed.filter.covariance, expected_covariance)
            assert delayed.telemetry()['gnss_measurement_age_s'] >= delay-1e-9
            assert np.linalg.eigvalsh(delayed.covariance).min() > 0
            assert len(delayed.controls) <= 28
    assert delayed.delivered_count + len(delayed.pending) == delayed.receiver.sequence


def test_zero_delay_at_receiver_rate_preserves_the_existing_filter_calculation():
    config = GNSSConfig()
    existing_receiver, existing_filter = DifferentialGNSS(config), JointOrbitFilter()
    channel = DelayedGNSSNavigation(config)
    acceleration = np.array([[0., 0., 0.], [.01, -.003, 0.]])
    for tick in range(40):
        now = tick*.1
        old = existing_filter.update(existing_receiver.measure(orbit(now), now), acceleration)
        result = channel.step(orbit(now), now, acceleration)
        np.testing.assert_array_equal(old, result)
        np.testing.assert_array_equal(existing_filter.covariance, channel.covariance)


def test_handoff_retains_in_flight_measurements_and_capture_time_geometry():
    channel = DelayedGNSSNavigation(GNSSConfig(delay_s=.2))
    offsets = np.array([[.1, .2, .3, .001, 0., 0.], [.4, .5, .6, 0., .002, 0.]])
    cooperative = {'rotation': np.eye(3), 'hinge': .01}
    for tick in range(5):
        now = tick*.05
        channel.step(orbit(now), now, np.zeros((2, 3)), origin_offsets=offsets, cooperative=cooperative)
    saved = copy.deepcopy(channel.snapshot())
    offsets[:] = 99
    cooperative['rotation'][:] = 99
    assert [packet['sequence'] for packet in saved['pending']] == [2, 3]
    assert all(packet['due'] > saved['epoch_s'] for packet in saved['pending'])
    np.testing.assert_array_equal(saved['pending'][0]['origin_offsets'][0], [.1, .2, .3, .001, 0., 0.])
    np.testing.assert_array_equal(saved['pending'][0]['cooperative']['rotation'], np.eye(3))
    assert saved['gnss_acquired_sequence'] == 3 and saved['gnss_delivered_sequence'] == 1


def test_transport_rejects_nonadvancing_clock_and_nonfinite_controls():
    channel = DelayedGNSSNavigation(GNSSConfig(delay_s=.1))
    channel.step(orbit(0), 0, np.zeros((2, 3)))
    with pytest.raises(ValueError, match='advance'):
        channel.step(orbit(0), 0, np.zeros((2, 3)))
    with pytest.raises(ValueError, match='commanded acceleration'):
        channel.step(orbit(.1), .1, np.full((2, 3), np.nan))


def transport_handoff():
    config = GNSSConfig(delay_s=.2)
    channel = DelayedGNSSNavigation(config, seed=19)
    offsets = np.array([[.1, .2, .3, .001, 0., 0.], [.4, .5, .6, 0., .002, 0.]])
    for tick in range(5):
        now = tick*.05
        channel.step(orbit(now), now, np.zeros((2, 3)), origin_offsets=offsets,
                     cooperative={'rotation': np.eye(3), 'hinge': .01})
    return config, channel, offsets


def test_near_handoff_delivers_pending_packets_once_in_their_original_reference_frame():
    from simulation.near_field.navigation import SensorConfig, SensorSimulator
    config, channel, offsets = transport_handoff()
    saved = channel.snapshot()
    receiver_state = copy.deepcopy(channel.receiver.snapshot())
    near = SensorSimulator(SensorConfig(receiver_config=config), receiver_state=receiver_state,
                           epoch_s=.2, transport_state=saved)
    delivered = []
    for tick in range(5):
        now = tick*.05
        truth = orbit(.2+now)-offsets
        packets, _ = near.acquire(now, 'FarHandoff', truth[0]-truth[1], [0., 0., 110.], np.eye(3),
                                  {'rotation': np.eye(3), 'hinge': .01}, absolute_truth=truth)
        delivered.extend(p for p in packets if p['source'] == 'gnss')
        assert all(p['due'] <= now+1e-8 for p in packets)
    assert [p['sequence'] for p in delivered] == [2, 3]
    for before, after in zip(saved['pending'], delivered):
        origins = np.array(before['observations'])-np.array(before['origin_offsets'])
        np.testing.assert_array_equal(after['z'], origins[0]-origins[1])
        np.testing.assert_array_equal(after['R'], before['relative_covariance'])
        assert after['stamp'] == pytest.approx(before['stamp']-.2)
        assert after['due'] == pytest.approx(before['due']-.2)
    continued = DifferentialGNSS(config)
    continued.restore(receiver_state)
    continued.measure(orbit(.3)-offsets, .3)
    continued.measure(orbit(.4)-offsets, .4)
    assert near.receiver.sequence == continued.sequence == 5
    assert near.receiver.snapshot()['rng_state'] == continued.snapshot()['rng_state']
    near_saved = near.transport_snapshot()
    assert near_saved['reference'] == 'body_origin'
    assert near_saved['gnss_delivered_count'] == 3
    assert [p['sequence'] for p in near_saved['pending']] == [4, 5]
    resumed = SensorSimulator(SensorConfig(receiver_config=config), receiver_state=near.receiver.snapshot(),
                              epoch_s=.4, transport_state=near_saved)
    assert [p['sequence'] for p in resumed.pending] == [4, 5]


@pytest.mark.parametrize('problem', ['epoch', 'sequence', 'offsets', 'early_due', 'rotation'])
def test_near_handoff_rejects_corrupt_transport_instead_of_silently_resetting_it(problem):
    from simulation.near_field.navigation import SensorConfig, SensorSimulator
    config, channel, _ = transport_handoff()
    saved = channel.snapshot()
    if problem == 'epoch':
        saved['epoch_s'] = .3
    elif problem == 'sequence':
        saved['pending'][0]['sequence'] = 1
    elif problem == 'offsets':
        saved['pending'][0]['origin_offsets'][0][0] = float('nan')
    elif problem == 'early_due':
        saved['pending'][0]['due'] = .05
    else:
        saved['pending'][0]['cooperative']['rotation'][0][0] = 2
    with pytest.raises(ValueError):
        SensorSimulator(SensorConfig(receiver_config=config), receiver_state=channel.receiver.snapshot(),
                        epoch_s=.2, transport_state=saved)


@pytest.mark.parametrize('delay', [0, .12, .25])
@pytest.mark.parametrize('rate', [1, 2, 5, 10, 20])
def test_prephysics_sensor_polling_preserves_actual_publisher_epochs(delay, rate):
    from simulation.near_field.navigation import SensorConfig, SensorSimulator
    sensor = SensorSimulator(SensorConfig(receiver_config=GNSSConfig(delay_s=delay, sample_rate_hz=rate)))
    packets = []
    for tick in range(22):
        now = tick*.05
        published = max(0, tick-1)*.05
        truth = orbit(published)
        ready, _ = sensor.acquire(now, 'FarHandoff', truth[0]-truth[1], [0., 0., 110.], np.eye(3),
                                   {'rotation':np.eye(3), 'hinge':0.}, absolute_truth=truth,
                                   acquisition_time_s=published)
        for packet in ready:
            if packet['source'] != 'gnss':
                continue
            packets.append(packet)
            assert packet['stamp'] <= published+1e-9
            assert packet['due'] == pytest.approx(packet['stamp']+delay)
            assert packet['due'] <= now+1e-9
            assert (packet['stamp']*rate) == pytest.approx(round(packet['stamp']*rate))
    assert sensor.receiver.sequence == rate+1
    assert sensor.receiver.last_time == pytest.approx(1)
    assert [p['sequence'] for p in packets] == list(range(1, len(packets)+1))
    assert len(packets)+len([p for p in sensor.pending if p['source']=='gnss']) == rate+1


def test_stereo_observations_share_physical_epoch_instead_of_callback_time():
    from simulation.near_field.navigation import SensorConfig, SensorSimulator, stereo_geometry
    sensor = SensorSimulator(SensorConfig(vision_delay_s=.15))
    grasp = [.3, .02, .6]
    assert stereo_geometry(grasp, sensor.config)['valid']
    ready, _ = sensor.acquire(.15, 'VisualFusion', np.array([.6, 0., 0., 0., 0., 0.]),
                               grasp, np.eye(3), acquisition_time_s=.1)
    assert not ready
    queued = next(p for p in sensor.pending if p['source']=='stereo')
    assert queued['stamp'] == .1 and queued['due'] == .25
    ready, _ = sensor.acquire(.25, 'VisualFusion', np.array([.6, 0., 0., 0., 0., 0.]),
                               grasp, np.eye(3), acquisition_time_s=.2)
    first = next(p for p in ready if p['source']=='stereo')
    assert first['stamp'] == .1


def test_sensor_rejects_future_or_rewound_publication_timestamps():
    from simulation.near_field.navigation import SensorConfig, SensorSimulator
    sensor = SensorSimulator(SensorConfig())
    values = (np.zeros(6), [0., 0., 110.], np.eye(3))
    sensor.acquire(.1, 'FarHandoff', *values, acquisition_time_s=.05)
    for stamp in (.2, .04, float('nan')):
        with pytest.raises(ValueError, match='source time'):
            sensor.acquire(.15, 'FarHandoff', *values, acquisition_time_s=stamp)
