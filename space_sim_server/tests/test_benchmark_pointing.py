"""Continuous tracking, target geometry, visibility, safety and public inputs."""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_pointing import PointingEphemeris, PointingObservation, PointingTask
from space_arm_platform.benchmark_pointing_score import PointingScorer
from space_arm_platform.benchmark_registry import method_input_types
from simulation.benchmark_pointing_methods import line_of_sight, reference_frame


@pytest.fixture
def task():
    data = load_task(SPEC_DIRECTORY/'s06-earth-pointing-v1.json').model_dump(mode='json')
    data['objective']['acquisition_deadline_s'] = .1
    data['duration_s'] = .2
    return PointingTask.model_validate(data)


def ephemeris(time_s=0., spin=0.):
    c, s = math.cos(spin*time_s), math.sin(spin*time_s)
    return PointingEphemeris(earth_j2000_to_fixed=((c,s,0.),(-s,c,0.),(0.,0.,1.)),
        earth_j2000_to_fixed_rate_s=((-spin*s,spin*c,0.),(-spin*c,-spin*s,0.),(0.,0.,0.)),
        sun_geocentric_j2000=(1.5e11,0.,0.,0.,30000.,0.))


def circular_sample(tick, error_rad=0., angular_rate=None):
    t, radius = tick*.02, 6878136.6
    n = math.sqrt(3.986004418e14/radius**3)
    c, s = math.cos(n*t), math.sin(n*t)
    frame = np.column_stack(([-s,c,0.],[0.,0.,-1.],[-c,-s,0.]))
    if tick == 0:
        error_rad = math.radians(10)
    orientation = Rotation.from_matrix(frame)*Rotation.from_rotvec([error_rad,0.,0.])
    omega = (0.,-n,0.) if angular_rate is None else angular_rate
    return dict(time_ns=tick*20_000_000, chaser_com_j2000=(radius*c,radius*s,0.,-radius*n*s,radius*n*c,0.),
        orientation_body_to_j2000_wxyz=tuple(orientation.as_quat()[[3,0,1,2]]),
        angular_velocity_body_rad_s=(0.,0.,0.) if tick == 0 else omega,
        wheel_speed_rad_s=(0.,0.,0.), applied_motor_torque_xyz_nm=(0.,0.,0.), external_state_writes=0,
        ephemeris=ephemeris().model_dump())


def test_entire_tracking_window_requires_dynamic_direction_not_zero_body_rate(task):
    scorer = PointingScorer(task)
    for tick in range(11):
        scorer.add(circular_sample(tick))
        if tick < 10:
            assert scorer.result()['outcome'] == 'incomplete'
    score = scorer.result()
    assert score['outcome'] == 'success' and score['tracking_samples'] == 6
    assert score['failed_tracking_samples'] == 0
    assert score['maximum_tracking_boresight_error_rad'] < 1e-12
    assert score['maximum_tracking_direction_rate_error_rad_s'] < 1e-12
    assert score['maximum_body_rate_rad_s'] > .001


@pytest.mark.parametrize('defect', ['one_pointing_excursion', 'stationary_body', 'gap', 'duplicate', 'overwrite', 'hard_rate'])
def test_late_correct_frame_cannot_erase_tracking_failure_or_invalid_evidence(task, defect):
    scorer = PointingScorer(task)
    for tick in range(11):
        if tick == 6 and defect == 'gap':
            continue
        sample = circular_sample(tick, error_rad=.02 if tick == 6 and defect == 'one_pointing_excursion' else 0.,
                                  angular_rate=(0.,0.,0.) if tick == 6 and defect == 'stationary_body' else None)
        if tick == 6 and defect == 'overwrite':
            sample['external_state_writes'] = 1
        if tick == 6 and defect == 'hard_rate':
            sample['angular_velocity_body_rad_s'] = (0.,.16,0.)
        scorer.add(sample)
        if tick == 6 and defect == 'duplicate':
            scorer.add(sample)
    score = scorer.result()
    assert not score['success'] and score['final_boresight_error_rad'] < 1e-12
    if defect in {'gap', 'duplicate', 'overwrite'}:
        assert score['outcome'] == 'invalid_evidence'
    elif defect == 'hard_rate':
        assert score['outcome'] == 'safety_violation'
    else:
        assert score['outcome'] == 'time_limit' and score['failed_tracking_samples'] == 1


def test_roll_about_boresight_is_unconstrained_but_not_hard_body_rate(task):
    data = task.model_dump(mode='json')
    data['objective'].update(kind='inertial_direction', direction_j2000_unit=[0.,0.,1.])
    task = PointingTask.model_validate(data)
    scorer = PointingScorer(task)
    for tick in range(11):
        sample = circular_sample(tick)
        quaternion = Rotation.from_rotvec([math.radians(10),0.,0.] if tick == 0 else [0.,0.,tick*.01]).as_quat()[[3,0,1,2]]
        sample.update(orientation_body_to_j2000_wxyz=tuple(quaternion), angular_velocity_body_rad_s=(0.,0.,.05) if tick else (0.,0.,0.))
        scorer.add(sample)
    assert scorer.result()['success']


def test_ground_geometry_uses_spice_fixed_frame_and_its_derivative(task):
    data = task.model_dump(mode='json')
    data.update(source_id='S05', requirement_id='ssb.attitude.ground-track')
    data['objective'].update(kind='ground_site', ground_direction_iau_earth_unit=[1.,0.,0.], minimum_elevation_rad=.1)
    goal = PointingTask.model_validate(data).objective
    state = np.array([6878136.6,0.,0.,0.,7500.,0.])
    direction, derivative = line_of_sight(goal, state, ephemeris(0., 7.292115e-5))
    dt = .001
    later = state.copy()
    later[:3] += state[3:]*dt
    next_direction, _ = line_of_sight(goal, later, ephemeris(dt, 7.292115e-5))
    assert direction == pytest.approx([-1.,0.,0.])
    assert derivative == pytest.approx((next_direction-direction)/dt, abs=2e-7)
    assert abs(derivative[1]) > .014
    frame, omega = reference_frame(direction, derivative)
    assert frame[:,2] == pytest.approx(direction)
    assert np.cross(omega,direction) == pytest.approx(derivative)


@pytest.mark.parametrize('kind', ['ground_site', 'sun'])
def test_invisible_ground_or_occulted_sun_cannot_succeed(task, kind):
    data = task.model_dump(mode='json')
    data['objective']['kind'] = kind
    if kind == 'ground_site':
        data.update(source_id='S05', requirement_id='ssb.attitude.ground-track')
        data['objective'].update(ground_direction_iau_earth_unit=[-1.,0.,0.], minimum_elevation_rad=.1)
    scorer = PointingScorer(PointingTask.model_validate(data))
    for tick in range(11):
        sample = circular_sample(tick)
        if kind == 'sun':
            sample['ephemeris']['sun_geocentric_j2000'] = (-1.5e11,0.,0.,0.,0.,0.)
        scorer.add(sample)
    score = scorer.result()
    assert not score['success']
    if kind == 'ground_site':
        assert 'ground_target_not_initially_visible' in score['evidence_errors']
    else:
        assert 'sun_occulted_by_earth' in score['quality_violations']


@pytest.mark.parametrize('defect', ['nonunit', 'no_window', 'wrong_source', 'initially_aligned', 'boolean_clock', 'left_handed', 'wrong_derivative'])
def test_invalid_contract_and_ephemeris_rejected(task, defect):
    data = task.model_dump(mode='json')
    if defect in {'left_handed', 'wrong_derivative'}:
        payload = ephemeris().model_dump(mode='json')
        payload['earth_j2000_to_fixed' if defect == 'left_handed' else 'earth_j2000_to_fixed_rate_s'][0][0] = -1. if defect == 'left_handed' else .001
        with pytest.raises(ValueError):
            PointingEphemeris.model_validate(payload)
        return
    if defect == 'nonunit':
        data['objective'].update(kind='inertial_direction', direction_j2000_unit=[2.,0.,0.])
    if defect == 'no_window':
        data['objective']['acquisition_deadline_s'] = data['duration_s']
    if defect == 'wrong_source':
        data['source_id'] = 'S05'
    if defect == 'initially_aligned':
        data['initial']['rotation_min_rad'] = .001
    if defect == 'boolean_clock':
        data['controlled_spacecraft'] = True
    with pytest.raises(ValueError):
        PointingTask.model_validate(data)


def test_public_observation_excludes_physics_and_has_separate_known_ephemeris(task):
    assert method_input_types(task)[1] is PointingObservation
    assert 'chaser_com_j2000' not in PointingObservation.model_fields
    assert 'applied_motor_torque_xyz_nm' not in PointingObservation.model_fields
    assert 'chaser_gnss_j2000' in PointingObservation.model_fields
    assert 'ephemeris' in PointingObservation.model_fields
