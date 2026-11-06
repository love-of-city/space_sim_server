"""Physical two-body geometry, full-window tracking, initial bounds and public contracts."""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_target_pointing import TargetPointingObservation, TargetPointingTask
from space_arm_platform.benchmark_target_pointing_score import TargetPointingScorer
from space_arm_platform.benchmark_registry import make_scorer, method_input_types, method_ids
from simulation.benchmark_pointing_methods import reference_frame
from simulation.benchmark_target_pointing_methods import TargetPointingPD


@pytest.fixture
def task():
    data = load_task(SPEC_DIRECTORY/'d11-cooperative-target-pointing-v1.json').model_dump(mode='json')
    data['initial'].update(inclination_deg=0.,true_anomaly_deg=0.)
    data['objective']['acquisition_deadline_s'] = .1
    data['duration_s'] = .2
    data['target_initial'].update(position_tnr_m=[500.,0.,0.],velocity_tnr_m_s=[0.,.4,0.])
    return TargetPointingTask.model_validate(data)


def physical_sample(tick, tilt=0.):
    t, radius = tick*.02,6878136.6
    n = math.sqrt(3.986004418e14/radius**3)
    c,s = math.cos(n*t),math.sin(n*t)
    radial,tangent,normal = np.array([c,s,0.]),np.array([-s,c,0.]),np.array([0.,0.,1.])
    chaser = np.r_[radius*radial,radius*n*tangent]
    dr = 500*tangent+.4*t*normal
    dv = -500*n*radial+.4*normal
    target = chaser+np.r_[dr,dv]
    direction = dr/np.linalg.norm(dr)
    derivative = (dv-direction*(direction@dv))/np.linalg.norm(dr)
    frame, omega = reference_frame(direction,derivative)
    orientation = Rotation.from_matrix(frame)*Rotation.from_rotvec([.15 if tick == 0 else tilt,0.,0.])
    return dict(time_ns=tick*20_000_000,chaser_com_j2000=tuple(chaser),target_com_j2000=tuple(target),
        orientation_body_to_j2000_wxyz=tuple(orientation.as_quat()[[3,0,1,2]]),
        angular_velocity_body_rad_s=(0.,0.,0.) if tick == 0 else tuple(orientation.inv().apply(omega)),
        wheel_speed_rad_s=(0.,0.,0.),applied_motor_torque_xyz_nm=(0.,0.,0.),external_state_writes=0,
        ephemeris=dict(earth_j2000_to_fixed=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.)),
            earth_j2000_to_fixed_rate_s=((0.,0.,0.),(0.,0.,0.),(0.,0.,0.)),
            sun_geocentric_j2000=(1.5e11,0.,0.,0.,30000.,0.)))


def test_real_relative_geometry_and_full_window_are_required(task):
    scorer = make_scorer(task,170.)
    assert isinstance(scorer,TargetPointingScorer)
    for tick in range(11):
        scorer.add(physical_sample(tick))
        if tick < 10:
            assert scorer.result()['outcome'] == 'incomplete'
    score = scorer.result()
    assert score['success'] and score['tracking_samples'] == 6
    assert score['maximum_tracking_boresight_error_rad'] < 1e-12
    assert score['maximum_tracking_direction_rate_error_rad_s'] < 1e-12
    assert score['minimum_range_m'] == pytest.approx(500.)
    assert score['maximum_relative_speed_m_s'] == pytest.approx(.4)
    assert not score['evidence_errors'] and score['controlled_spacecraft'] == 1 and score['spacecraft_count'] == 2


@pytest.mark.parametrize('defect',['excursion','wrong_rate','frozen_direction','range','speed','gap','duplicate','write','hard_rate'])
def test_late_correct_pointing_never_erases_failure(task,defect):
    scorer = TargetPointingScorer(task)
    for tick in range(11):
        if tick == 6 and defect == 'gap':
            continue
        sample = physical_sample(tick,.02 if tick == 6 and defect == 'excursion' else 0.)
        if tick == 6:
            if defect == 'wrong_rate': sample['angular_velocity_body_rad_s'] = (0.,0.,0.)
            if defect == 'frozen_direction': sample['orientation_body_to_j2000_wxyz'] = physical_sample(0)['orientation_body_to_j2000_wxyz']
            if defect == 'range': sample['target_com_j2000'] = tuple(np.asarray(sample['chaser_com_j2000'])+[0.,50.,0.,0.,0.,0.])
            if defect == 'speed': sample['target_com_j2000'] = tuple(np.asarray(sample['target_com_j2000'])+[0.,0.,0.,10.,0.,0.])
            if defect == 'write': sample['external_state_writes'] = 1
            if defect == 'hard_rate': sample['angular_velocity_body_rad_s'] = (.16,0.,0.)
        scorer.add(sample)
        if tick == 6 and defect == 'duplicate': scorer.add(sample)
    score = scorer.result()
    assert not score['success'] and score['final_boresight_error_rad'] < 1e-12
    expected = ('invalid_evidence' if defect in {'gap','duplicate','write'} else
        'safety_violation' if defect in {'range','speed','hard_rate'} else 'time_limit')
    assert score['outcome'] == expected


@pytest.mark.parametrize('defect',['target_position','target_velocity','chaser_orbit'])
def test_initial_orbit_and_relative_distribution_are_independently_checked(task,defect):
    sample = physical_sample(0)
    if defect == 'chaser_orbit':
        sample['chaser_com_j2000'] = tuple(np.asarray(sample['chaser_com_j2000'])+[0.,0.,0.,1.,0.,0.])
    else:
        offset = np.zeros(6)
        offset[0 if defect == 'target_position' else 3] = 20.
        sample['target_com_j2000'] = tuple(np.asarray(sample['target_com_j2000'])+offset)
    scorer = TargetPointingScorer(task)
    scorer.add(sample)
    assert scorer.result()['outcome'] == 'invalid_evidence'


@pytest.mark.parametrize('defect',['noncooperative','one_body','two_controlled','unsafe_distance','unsafe_velocity','wrong_source','wrong_frame','no_window'])
def test_d11_cannot_relabel_single_or_noncooperative_geometry(task,defect):
    data = task.model_dump(mode='json')
    if defect == 'noncooperative': data['target_cooperation'] = 'N'
    if defect == 'one_body': data['spacecraft_count'] = 1
    if defect == 'two_controlled': data['controlled_spacecraft'] = 2
    if defect == 'unsafe_distance': data['target_initial']['position_half_width_m'] = 400
    if defect == 'unsafe_velocity': data['target_initial']['velocity_half_width_m_s'] = 5
    if defect == 'wrong_source': data['source_id'] = 'S07'
    if defect == 'wrong_frame': data['reference_frame'] = 'RTN'
    if defect == 'no_window': data['objective']['acquisition_deadline_s'] = data['duration_s']
    with pytest.raises(ValueError): TargetPointingTask.model_validate(data)


def test_public_target_gnss_drives_reference_and_reset(task):
    assert method_input_types(task)[1] is TargetPointingObservation
    assert 'target_com_j2000' not in TargetPointingObservation.model_fields
    assert 'chaser_com_j2000' not in TargetPointingObservation.model_fields
    sample = physical_sample(1)
    observation = TargetPointingObservation(episode_id='test',sequence=1,sample_time_ns=20_000_000,delivery_time_ns=20_000_000,
        orientation_body_to_j2000_wxyz=sample['orientation_body_to_j2000_wxyz'],
        angular_velocity_body_rad_s=sample['angular_velocity_body_rad_s'],wheel_speed_rad_s=(0.,0.,0.),gyro_valid=True,
        chaser_gnss_j2000=sample['chaser_com_j2000'],target_gnss_j2000=sample['target_com_j2000'],ephemeris=sample['ephemeris'])
    method = TargetPointingPD()
    direction, derivative = method._reference(observation)
    dt = .001
    next_chaser, next_target = np.array(observation.chaser_gnss_j2000),np.array(observation.target_gnss_j2000)
    next_chaser[:3] += dt*next_chaser[3:]
    next_target[:3] += dt*next_target[3:]
    following = observation.model_copy(update=dict(chaser_gnss_j2000=tuple(next_chaser),target_gnss_j2000=tuple(next_target)))
    assert derivative == pytest.approx((method._reference(following)[0]-direction)/dt,abs=2e-9)
    moved = observation.model_copy(update=dict(target_gnss_j2000=tuple(np.asarray(observation.target_gnss_j2000)+[0.,0.,100.,0.,0.,0.])))
    assert np.linalg.norm(method._reference(moved)[0]-direction) > .1
    method.initial_reference = np.ones(4)
    method.reset(None)
    assert method.initial_reference is None
    assert method_ids(task)[0] == 'cooperative-target-pointing-v1'
