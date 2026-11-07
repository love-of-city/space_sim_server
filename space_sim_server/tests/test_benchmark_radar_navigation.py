"""No-target-telemetry boundary, radar geometry, estimator observability and independent target scoring."""
import math
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError
from scipy.spatial.transform import Rotation

from simulation.benchmark_radar_navigation_methods import RadarOrbitEstimator
from simulation.benchmark_radar_sensor import measure_radar
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_external import ExternalMethod, ExternalMethodDefinition
from space_arm_platform.benchmark_navigation import NavigationEstimateAction, NavigationEstimateEvidence
from space_arm_platform.benchmark_radar_navigation import (
    RadarDelivery, RadarNavigationContext, RadarNavigationTask, RadarObservation, RadarSensorModel,
)
from space_arm_platform.benchmark_radar_navigation_score import RadarNavigationScorer
from space_arm_platform.benchmark_registry import parse_action


def task(duration=1.):
    original = load_task(SPEC_DIRECTORY/'d02-noncooperative-radar-estimation-v1.json')
    return RadarNavigationTask.model_validate(original.model_dump() | {'duration_s': duration})


def physical(tick):
    radius, mu = 6878136.6, 398600441800000.
    t, inc = tick*.02, math.radians(51.6)
    rate = math.sqrt(mu/radius**3)
    phase = math.pi+rate*t
    radial = np.array([math.cos(phase), math.sin(phase)*math.cos(inc), math.sin(phase)*math.sin(inc)])
    tangent = np.array([-math.sin(phase), math.cos(phase)*math.cos(inc), math.cos(phase)*math.sin(inc)])
    normal = np.cross(radial, tangent)
    target = np.r_[radius*radial, radius*rate*tangent]
    displacement = -500*tangent+(50+.05*t)*normal+20*radial
    velocity = np.cross(rate*normal, displacement)+.05*normal
    return dict(time_ns=tick*20_000_000, target_com_j2000=tuple(target),
        chaser_com_j2000=tuple(target+np.r_[displacement, velocity]),
        applied_motor_force_body_n=(0., 0., 0.), external_state_writes=0)


def packet(spec, row, rng=None, rotation=None):
    return measure_radar(spec.observation, row['target_com_j2000'], row['chaser_com_j2000'],
        np.eye(3) if rotation is None else rotation, np.random.default_rng(17) if rng is None else rng,
        episode_id='measurement-case', sequence=row['time_ns']//spec.decision_period_ns, nanos=row['time_ns'])


@pytest.mark.parametrize('change', [dict(target_cooperation='C'), dict(controlled_spacecraft=1),
    dict(controlled_spacecraft=False), dict(spacecraft_count=1), dict(reference_frame='chaser_minus_target_J2000'),
    dict(vision_mode='V2'), dict(source_id='D01')])
def test_radar_task_cannot_alias_cooperative_or_controlled_navigation(change):
    with pytest.raises(ValidationError):
        RadarNavigationTask.model_validate(task().model_dump() | change)


def test_target_orbit_is_a_distribution_and_sensor_envelope_covers_safety():
    for field in ('target_altitude_half_width_m', 'target_inclination_half_width_deg', 'target_anomaly_half_width_deg'):
        document = task().model_dump()
        document['initial'][field] = 0.
        with pytest.raises(ValidationError):
            RadarNavigationTask.model_validate(document)
    for key, value in (('target_link', 'cooperative_target_gnss_solution'), ('minimum_range_m', 200.), ('maximum_range_m', 900.)):
        document = task().model_dump()
        document['observation'][key] = value
        with pytest.raises(ValidationError):
            RadarNavigationTask.model_validate(document)


def test_radar_geometry_uses_observer_body_bearing_and_target_minus_observer_range_rate():
    document = task().observation.model_dump()
    for key in ('position_sigma_m', 'velocity_sigma_m_s', 'attitude_sigma_rad', 'range_sigma_m', 'bearing_sigma_rad', 'range_rate_sigma_m_s'):
        document[key] = 1e-12
    sensor = RadarSensorModel.model_validate(document)
    rotation = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    observer = (6878136.6, 0., 0., 0., 7500., 0.)
    target = (6878136.6, 500., 0., 0., 7502., 3.)
    observation = measure_radar(sensor, target, observer, rotation, np.random.default_rng(3), episode_id='case', sequence=0, nanos=0)
    assert observation.range_m == pytest.approx(500., abs=1e-8)
    assert observation.direction_body_unit == pytest.approx((1., 0., 0.), abs=1e-8)
    assert observation.range_rate_m_s == pytest.approx(2., abs=1e-8)
    assert Rotation.from_quat(observation.orientation_body_to_j2000_wxyz, scalar_first=True).apply(observation.direction_body_unit) == pytest.approx((0., 1., 0.), abs=1e-8)
    assert observation.target_measurement_j2000 is None
    with pytest.raises(ValueError, match='ranging envelope'):
        measure_radar(sensor, observer, observer, rotation, np.random.default_rng(3), episode_id='case', sequence=0, nanos=0)


@pytest.mark.parametrize('change', [dict(target_measurement_j2000=(0.,)*6), dict(target_truth_j2000=(0.,)*6),
    dict(target_estimate_j2000=(0.,)*6), dict(direction_body_unit=(2., 0., 0.)),
    dict(orientation_body_to_j2000_wxyz=(2., 0., 0., 0.)), dict(delivery_time_ns=1)])
def test_observation_whitelist_rejects_target_telemetry_truth_and_invalid_sensor_packets(change):
    with pytest.raises(ValidationError):
        RadarObservation.model_validate(packet(task(), physical(0)).model_dump() | change)


def test_private_delivery_is_reproducible_across_episode_identities():
    observation = packet(task(), physical(0))
    values = observation.model_dump(exclude={'episode_id', 'schema_version'})
    changed = observation.model_copy(update={'episode_id': 'another-run'})
    first = RadarDelivery(observation=values)
    second = RadarDelivery(observation=changed.model_dump(exclude={'episode_id', 'schema_version'}))
    assert first.model_dump_json() == second.model_dump_json()
    with pytest.raises(ValidationError):
        RadarDelivery(observation=observation.model_dump())


def scored(fault=None):
    spec = task()
    scorer = RadarNavigationScorer(spec, 170.)
    for tick in range(51):
        row = physical(tick)
        if fault == 'actuation' and tick == 40:
            row['applied_motor_force_body_n'] = (.001, 0., 0.)
        if fault == 'state_write' and tick == 40:
            row['external_state_writes'] = 1
        scorer.add(row)
        if tick % 25:
            continue
        observation = packet(spec, row)
        if not (fault == 'missing_delivery' and tick == 25):
            delivery = RadarDelivery(observation=observation.model_dump(exclude={'episode_id', 'schema_version'}))
            scorer.add(delivery)
            if fault == 'duplicate_delivery' and tick == 25:
                scorer.add(delivery)
        if tick == 50 or (fault == 'missing_estimate' and tick == 25):
            continue
        state = np.asarray(row['target_com_j2000'])
        if fault == 'wrong_sign_frame':
            state = np.asarray(row['chaser_com_j2000'])-state
        if fault == 'early_error' and tick == 0:
            state[0] += 4.
        scorer.add(NavigationEstimateEvidence(sequence=tick//25, estimate_time_ns=row['time_ns'],
            reference_frame=spec.reference_frame, estimated_state=tuple(state)))
    return scorer.result()


def test_grader_checks_absolute_target_orbit_and_complete_sensor_stream():
    result = scored()
    assert result['outcome'] == 'success'
    assert result['sample_count'] == 51 and result['estimate_count'] == 2 and result['radar_delivery_count'] == 3
    assert result['position_rmse_m'] == result['velocity_rmse_m_s'] == result['thrust_impulse_n_s'] == 0.
    assert result['target_telemetry_available'] is False
    assert result['range_measurement_rmse_m'] > 0 and result['inertial_bearing_measurement_rmse_rad'] > 0


@pytest.mark.parametrize('fault, outcome', [('missing_delivery', 'invalid_evidence'), ('duplicate_delivery', 'invalid_evidence'),
    ('missing_estimate', 'invalid_evidence'), ('state_write', 'invalid_evidence'), ('actuation', 'safety_violation'),
    ('wrong_sign_frame', 'task_failure'), ('early_error', 'task_failure')])
def test_last_accurate_sample_cannot_erase_incomplete_evidence_or_earlier_errors(fault, outcome):
    result = scored(fault)
    assert result['outcome'] == outcome
    if fault == 'early_error':
        assert result['final_position_error_m'] == 0 and result['maximum_position_error_m'] == 4.


def test_initial_target_bounds_are_scored_independently_of_estimates():
    spec = task()
    row = physical(0)
    row['target_com_j2000'] = (row['target_com_j2000'][0]-3000., *row['target_com_j2000'][1:])
    scorer = RadarNavigationScorer(spec, 170.)
    scorer.add(row)
    assert scorer.result()['outcome'] == 'invalid_initialization'


def test_estimator_uses_time_series_to_recover_unobserved_transverse_velocity_and_resets():
    spec = task(90.)
    context = RadarNavigationContext(task=spec, chaser_mass_kg=170.)
    method = RadarOrbitEstimator()
    method.reset(context)
    rng = np.random.default_rng(61)
    errors = []
    for tick in range(0, 4500, 25):
        row = physical(tick)
        observation = packet(spec, row, rng)
        action = parse_action(spec, method.act(observation), observation)
        errors.append(np.asarray(action.estimated_state)-row['target_com_j2000'])
        assert np.linalg.eigvalsh(method.covariance).min() > 0
    errors = np.asarray(errors)
    assert np.sqrt(np.mean(np.sum(errors[:, :3]**2, axis=1))) < spec.objective.position_rmse_limit_m
    assert np.sqrt(np.mean(np.sum(errors[:, 3:]**2, axis=1))) < spec.objective.velocity_rmse_limit_m_s
    assert np.linalg.norm(errors[0, 3:]) > .4 and np.linalg.norm(errors[-1, 3:]) < .03
    method.reset(context)
    assert method.state is None and method.last_time_ns is None


def test_external_radar_method_and_public_action_contract(tmp_path):
    spec = task()
    context = RadarNavigationContext(task=spec, chaser_mass_kg=170.)
    path = Path(__file__).resolve().parents[1]/'examples/autonomy_methods/radar-navigation.json'
    external = ExternalMethod(ExternalMethodDefinition(path, spec), tmp_path)
    baseline = RadarOrbitEstimator()
    baseline.reset(context)
    try:
        external.reset(context)
        for tick in (0, 25):
            observation = packet(spec, physical(tick))
            assert parse_action(spec, external.act(observation), observation) == baseline.act(observation)
        action = baseline.act(observation)
        for update in (dict(episode_id='stale'), dict(reference_frame='chaser_minus_target_J2000'), dict(force_j2000_n=(0., 0., 0.))):
            with pytest.raises(ValueError):
                parse_action(spec, action.model_dump() | update, observation)
        assert isinstance(action, NavigationEstimateAction)
    finally:
        external.close()
