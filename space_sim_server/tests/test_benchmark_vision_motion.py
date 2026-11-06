"""Observable spin, own-motion compensation and independent full-window grading."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY,load_task
from space_arm_platform.benchmark_spin_estimator import ImageGyroAngularVelocity
from space_arm_platform.benchmark_vision import FiducialMarker
from space_arm_platform.benchmark_vision_motion import ChaserGyroPacket,VisionMotionContext,VisionMotionTask
from space_arm_platform.benchmark_vision_motion_score import VisionMotionScorer
from test_benchmark_vision import capture,estimate,truth,quaternion


@pytest.fixture
def task():
    data=load_task(SPEC_DIRECTORY/'d27-stereo-gyro-spin-v1.json').model_dump()
    data['duration_s']=3.
    data['objective']['spin_warmup_ns']=1_000_000_000
    data['initial'].update(inclination_deg=0.,true_anomaly_deg=0.,
                           chaser_rate_body_rad_s=(0.,0.,0.),chaser_rate_half_width_rad_s=0.,
                           target_rate_body_rad_s=(0.,.04,0.),target_rate_half_width_rad_s=0.)
    return VisionMotionTask.model_validate(data)


def packet(time_ns,rate=(0.,0.,0.),*,valid=True,variance=(0.,0.,0.)):
    return ChaserGyroPacket(sample_time_ns=time_ns,angular_velocity_body_rad_s=rate,
        error_variance_rad2_s2=variance,saturated_axes=(not valid,False,False),valid=valid)


def test_image_and_own_gyro_recover_target_body_spin_under_noncommuting_camera_motion():
    mount=Rotation.from_euler('xyz',[.3,-.6,1.2])
    camera=SimpleNamespace(orientation_body_from_optical_wxyz=quaternion(mount))
    estimator=ImageGyroAngularVelocity(camera,2_000_000_000)
    own=np.array([.03,.02,-.01])
    target_rate=np.array([.01,.04,-.02])
    initial=Rotation.from_euler('xyz',[-.2,.5,.6])
    rates=[]
    for index in range(9):
        now=index*500_000_000
        body=Rotation.from_rotvec(own*now*1e-9)
        target=initial*Rotation.from_rotvec(target_rate*now*1e-9)
        relative=mount.inv()*body.inv()*target
        samples=[packet(t,tuple(own)) for t in range(0 if not index else now-480_000_000,now+1,20_000_000)]
        result=estimator.update(now,samples,SimpleNamespace(orientation_optical_from_target_wxyz=quaternion(relative)))
        if index<4:
            assert result is None
        else:
            rates.append(result)
            assert np.allclose(result,target_rate,rtol=0,atol=1e-12)
    assert len(rates)==5


def test_invalid_own_gyro_cannot_silently_be_treated_as_zero_motion():
    camera=SimpleNamespace(orientation_body_from_optical_wxyz=(1.,0.,0.,0.))
    estimator=ImageGyroAngularVelocity(camera,1_000_000_000)
    for index in range(5):
        now=index*500_000_000
        pose=SimpleNamespace(orientation_optical_from_target_wxyz=quaternion(Rotation.from_rotvec([0.,.04*index*.5,0.])))
        samples=[packet(t,valid=t!=500_000_000) for t in range(0 if not index else now-480_000_000,now+1,20_000_000)]
        assert estimator.update(now,samples,pose) is None
    with pytest.raises(ValueError,match='consecutive'):
        estimator.update(now,[],pose)


def sequence(task,defect=None):
    scorer=VisionMotionScorer(task,169.4)
    for tick in range(151):
        raw=truth(tick)
        raw['target_angular_velocity_body_rad_s']=(0.,.04,0.)
        raw['target_orientation_body_to_j2000_wxyz']=quaternion(
            Rotation.from_euler('x',.2)*Rotation.from_euler('y',.1+.04*tick*.02))
        model=task.own_gyro
        variance=model.white_noise_std_rad_s**2+model.initial_bias_std_rad_s**2+model.bias_random_walk_rad_s_sqrt_s**2*tick*.02
        raw['chaser_gyro']=packet(tick*20_000_000,variance=(variance,)*3).model_dump()
        if defect=='physical_violation' and tick==40:
            raw['applied_motor_force_body_n']=(1.,0.,0.)
        scorer.add(raw)
        if tick%25==0:
            index=tick//25
            scorer.add(capture(scorer,index))
            if tick<150:
                value=estimate(scorer,index)
                value.update(record_type='vision_motion_estimate',angular_velocity_frame=task.angular_velocity_frame,
                    target_angular_velocity_body_rad_s=(0.,.04,0.) if index>=2 else None)
                if defect=='zero_spin' and index>=2:
                    value['target_angular_velocity_body_rad_s']=(0.,0.,0.)
                if defect=='missing_spin' and index==3:
                    value['target_angular_velocity_body_rad_s']=None
                if defect=='wrong_early_spin' and index==2:
                    value['target_angular_velocity_body_rad_s']=(0.,1.,0.)
                scorer.add(value)
    return scorer.result()


def test_every_spin_estimate_after_declared_warmup_is_scored_against_physics(task):
    score=sequence(task)
    assert score['success'] and score['expected_spin_estimates']==4 and score['available_spin_count']==4,score
    assert score['angular_velocity_rmse_rad_s']==0.
    assert score['estimate_count']==6 and score['capture_count']==7


@pytest.mark.parametrize('defect',['zero_spin','missing_spin','wrong_early_spin'])
def test_good_final_spin_or_pose_cannot_erase_an_earlier_angular_velocity_failure(task,defect):
    score=sequence(task,defect)
    assert score['outcome']=='task_failure' and score['quality_violations']
    assert score['position_rmse_m']<1e-10


def test_perfect_spin_does_not_override_physical_safety(task):
    assert sequence(task,'physical_violation')['outcome']=='safety_violation'


@pytest.mark.parametrize('key,value',[('target_link','target_gyro'),('vision_mode','V3'),('source_id','D28')])
def test_motion_task_cannot_claim_an_undeclared_modality_or_noncooperative_tumble_task(task,key,value):
    data=deepcopy(task.model_dump())
    data[key]=value
    with pytest.raises(ValueError):
        VisionMotionTask.model_validate(data)


def test_initial_spin_distribution_accepts_both_directions_but_excludes_a_stationary_target(task):
    scorer=VisionMotionScorer(task,169.4)
    for rate,expected in (((0.,.04,0.),True),((0.,-.04,0.),True),((0.,0.,0.),False)):
        sample=SimpleNamespace(target_angular_velocity_body_rad_s=rate,chaser_angular_velocity_body_rad_s=(0.,0.,0.))
        assert scorer.initial_rates_valid(sample)==expected


def test_existing_noisy_gyro_enters_only_as_an_own_body_measurement(task):
    from simulation.gyroscope import GyroConfig,Gyroscope
    sensor=Gyroscope(GyroConfig(**task.own_gyro.sensor_config()),seed=7,
        body_name='cubesat_bus',sensor_name='benchmark_chaser_gyro')
    first=ChaserGyroPacket.from_sensor(sensor.sample(0,[.003,-.002,.002]))
    assert first.valid and first.sample_time_ns==0
    assert first.angular_velocity_body_rad_s!=(.003,-.002,.002)
    assert set(first.model_dump())=={'sample_time_ns','angular_velocity_body_rad_s','error_variance_rad2_s2',
                                   'saturated_axes','valid','body_name','frame','reference_frame'}
    with pytest.raises(ValueError):
        ChaserGyroPacket.model_validate(dict(first.model_dump(),target_angular_velocity_body_rad_s=[0.,.04,0.]))
    with pytest.raises(ValueError):
        ChaserGyroPacket.model_validate(dict(first.model_dump(),body_name='capture_target'))
    with pytest.raises(ValueError,match='timestamp'):
        ChaserGyroPacket.model_validate(dict(first.model_dump(),sample_time_ns=1))


def test_pose_loss_discards_old_image_history_until_a_new_full_spin_window_is_observed():
    camera=SimpleNamespace(orientation_body_from_optical_wxyz=(1.,0.,0.,0.))
    estimator=ImageGyroAngularVelocity(camera,1_000_000_000)
    for index in range(8):
        now=index*500_000_000
        pose=SimpleNamespace(orientation_optical_from_target_wxyz=quaternion(Rotation.from_rotvec([0.,.04*index*.5,0.])))
        if index==3:
            pose=None
        samples=[packet(t) for t in range(0 if not index else now-480_000_000,now+1,20_000_000)]
        result=estimator.update(now,samples,pose)
        if index in (0,1,3,4,5):
            assert result is None
        else:
            assert np.allclose(result,[0.,.04,0.],rtol=0,atol=1e-12)


def test_public_context_roundtrip_retains_spin_and_gyro_contract_without_private_sensor_state(task):
    context=VisionMotionContext(task=task,chaser_mass_kg=169.4,marker_dictionary='DICT_4X4_50',
        markers=(FiducialMarker(marker_id=0,corners_target_m=((0.,0.,0.),(.05,0.,0.),(.05,.05,0.),(0.,.05,0.))),))
    restored=VisionMotionContext.model_validate_json(context.model_dump_json())
    assert restored==context and restored.task.fingerprint()==task.fingerprint()
    assert restored.task.initial.target_spin_sign=='equiprobable_both_directions'
    assert restored.task.own_gyro.sample_period_ns==20_000_000
    with pytest.raises(ValueError):
        VisionMotionContext.model_validate(dict(context.model_dump(),gyro_bias=[0.,0.,0.]))
