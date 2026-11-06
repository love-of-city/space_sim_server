"""Independent image-pose grading: frame binding, missing estimates and safety."""
from copy import deepcopy
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_camera import PinholeCamera
from space_arm_platform.benchmark_vision import VisionPoseTask
from space_arm_platform.benchmark_vision_score import VisionPoseScorer,physical_camera


def test_public_image_replay_rejects_valid_png_substitution(task,tmp_path):
    import base64
    import hashlib
    import json
    import struct
    import zlib
    from space_arm_platform.benchmark_vision_score import verify_public_image_evidence

    def chunk(kind,data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    image=(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',3,2,8,2,0,0,0))
           +chunk(b'IDAT',zlib.compress(bytes(20)))+chunk(b'IEND',b''))
    camera=task.cameras[0].model_copy(update={'width':3,'height':2})
    task=task.model_copy(update={'cameras':(camera,)})
    scorer=VisionPoseScorer(task,169.4)
    scorer.add(truth(0))
    recorded=capture(scorer,0)
    recorded['image_sha256']=[hashlib.sha256(image).hexdigest()]
    frame=dict(camera_id=camera.camera_id,renderer_session_id=recorded['renderer_session_id'],
        source_frame_id=recorded['source_frame_id'],sample_time_ns=0,width=3,height=2,
        png_sha256=recorded['image_sha256'][0],png_base64=base64.b64encode(image).decode())
    observation=dict(episode_id='unit-evidence',sequence=0,sample_time_ns=0,delivery_time_ns=0,images=[frame])
    physics_path,observations_path=tmp_path/'physics.jsonl',tmp_path/'observations.jsonl'
    physics_path.write_text(json.dumps(recorded)+'\n')
    observations_path.write_text(json.dumps(observation)+'\n')
    verify_public_image_evidence(task,observations_path,physics_path)
    recorded['image_sha256']=['b'*64]
    physics_path.write_text(json.dumps(recorded)+'\n')
    with pytest.raises(ValueError,match='authoritative'):
        verify_public_image_evidence(task,observations_path,physics_path)


def quaternion(rotation):
    return tuple(rotation.as_quat()[[3,0,1,2]])


@pytest.fixture
def task():
    camera=PinholeCamera(camera_id='team/camera/benchmark_left',width=640,height=480,horizontal_fov_rad=.7,
        position_body_m=(.1,-.9,.05),orientation_body_from_optical_wxyz=quaternion(Rotation.from_euler('x',math.pi/2)))
    return VisionPoseTask.model_validate(dict(
        schema_version='spacesim-autonomy-rendered-pose/1',configuration_id='d23-test-fixture-v1',
        requirement_id='ssb.vision.monocular-pose',source_id='D23',backend='native_team_rendered_pose_v1',
        target_cooperation='P',controlled_spacecraft=0,spacecraft_count=2,control_architecture='single_estimator',
        vision_mode='V2',action_type='target_relative_pose_estimate_only',attitude_control='none_passive_attitude',
        reference_frame='target_body_origin_and_axes_in_left_optical_camera',target_link='none',
        target_geometry_prior='known_passive_fiducial_layout_no_active_cooperation',fiducial_layout='target_aruco_flush_5cm_v2',
        cameras=[camera.model_dump()],
        initial=dict(target_altitude_m=500000.,inclination_deg=0.,true_anomaly_deg=0.,
            chaser_minus_target_com_j2000_m=(0.,3.,0.),position_half_width_m=.1,
            chaser_minus_target_velocity_j2000_m_s=(0.,0.,0.),velocity_half_width_m_s=.001,
            target_tilt_min_rad=.1,target_tilt_max_rad=.3,target_roll_half_width_rad=.2,
            attitude_distribution='tilt_about_uniform_xz_axis_then_body_y_roll',initial_body_rates='zero_both_spacecraft',
            chaser_initial_attitude='identity_body_to_j2000'),
        objective=dict(kind='full_duration_relative_pose',minimum_available_fraction=1.,position_rmse_limit_m=.01,
            orientation_rmse_limit_rad=.02,maximum_position_error_m=.03,maximum_orientation_error_rad=.05),
        safety=dict(minimum_range_m=2.,maximum_range_m=10.,maximum_relative_speed_m_s=.1,maximum_body_rate_rad_s=.2,
            maximum_force_n=25.,maximum_uncommanded_force_n=1e-9,maximum_uncommanded_wheel_torque_nm=1e-9,
            contact_policy='no_physical_contacts'),
        seeds=dict(train=[1,2,3],validation=[101,102,103],test=list(range(201,211))),
        duration_s=1.,physics_period_ns=20_000_000,physics_substeps=8,decision_period_ns=500_000_000,
        earth_gravity='j2',solar_gravity=True,drag_enabled=True,srp_enabled=True,
        disturbance_sigma_m_s2=.00002,mass_model='constant_no_propellant_model'))


def truth(tick):
    radius=6378136.6+500000.
    velocity=math.sqrt(3.986004418e14/radius)
    return dict(time_ns=tick*20_000_000,target_com_j2000=(radius,0.,0.,0.,velocity,0.),
        chaser_com_j2000=(radius,3.,0.,0.,velocity,0.),target_origin_j2000_m=(radius-.1,-.01,-.02),
        chaser_origin_j2000_m=(radius-.2,2.7,.1),
        target_orientation_body_to_j2000_wxyz=quaternion(Rotation.from_euler('x',.2)*Rotation.from_euler('y',.1)),
        chaser_orientation_body_to_j2000_wxyz=(1.,0.,0.,0.),
        target_angular_velocity_body_rad_s=(0.,0.,0.),chaser_angular_velocity_body_rad_s=(0.,0.,0.),
        applied_motor_force_body_n=(0.,0.,0.),applied_wheel_torque_xyz_nm=(0.,0.,0.),contacts=0,external_state_writes=0)


def capture(scorer,sequence):
    poses=[physical_camera(scorer.latest,camera) for camera in scorer.task.cameras]
    return dict(record_type='vision_capture',sequence=sequence,sample_time_ns=sequence*500_000_000,
        renderer_session_id='renderer-1',source_frame_id=10+sequence,
        camera_ids=[camera.camera_id for camera in scorer.task.cameras],
        calibration_sha256=[camera.fingerprint() for camera in scorer.task.cameras],
        image_sha256=['a'*64]*len(poses),camera_origins_j2000_m=[tuple(p) for p,_ in poses],
        camera_orientations_j2000_from_optical_wxyz=[quaternion(Rotation.from_matrix(r)) for _,r in poses])


def estimate(scorer,sequence,*,position_error=0.,orientation_error=0.):
    # Oracle-valued records are unit-test inputs to the private grader only;
    # this helper is never registered as an evaluated image method.
    cam_p,cam_r=physical_camera(scorer.latest,scorer.task.cameras[0])
    actual_q=scorer.latest.target_orientation_body_to_j2000_wxyz
    target_r=Rotation.from_quat(np.asarray(actual_q)[[1,2,3,0]]).as_matrix()
    p=cam_r.T@(np.asarray(scorer.latest.target_origin_j2000_m)-cam_p)+[position_error,0.,0.]
    r=Rotation.from_euler('z',orientation_error)*Rotation.from_matrix(cam_r.T@target_r)
    return dict(record_type='vision_pose_estimate',sequence=sequence,estimate_time_ns=sequence*500_000_000,
        reference_frame=scorer.task.reference_frame,image_sha256=['a'*64]*len(scorer.task.cameras),
        pose=dict(position_target_origin_optical_m=tuple(p),orientation_optical_from_target_wxyz=quaternion(r)))


def sequence(task,mutate=None):
    scorer=VisionPoseScorer(task,169.4)
    for tick in range(51):
        raw=truth(tick)
        if mutate:
            raw=mutate('physics',tick,raw)
        if raw is not None:
            scorer.add(raw)
        if tick%25==0:
            index=tick//25
            raw=capture(scorer,index)
            if mutate:
                raw=mutate('capture',index,raw)
            if raw is not None:
                scorer.add(raw)
            if tick<50:
                raw=estimate(scorer,index)
                if mutate:
                    raw=mutate('estimate',index,raw)
                if raw is not None:
                    scorer.add(raw)
    return scorer.result()


def test_pose_scoring_uses_native_origin_optical_axes_and_every_observation(task):
    score=sequence(task)
    assert score['success'] and score['capture_count']==3 and score['estimate_count']==2
    assert score['available_pose_fraction']==1. and score['minimum_range_m']==3.
    assert score['position_rmse_m']<1e-12 and score['orientation_rmse_rad']<1e-12


@pytest.mark.parametrize('defect', ['missing_pose','position','orientation','wrong_direction'])
def test_last_good_pose_does_not_erase_earlier_failure(task,defect):
    def mutate(kind,index,raw):
        if kind=='estimate' and index==0:
            if defect=='missing_pose':
                raw['pose']=None
            elif defect=='position':
                raw['pose']['position_target_origin_optical_m']=(0.,0.,0.)
            elif defect=='orientation':
                raw['pose']['orientation_optical_from_target_wxyz']=(1.,0.,0.,0.)
            else:
                q=raw['pose']['orientation_optical_from_target_wxyz']
                raw['pose']['orientation_optical_from_target_wxyz']=(q[0],-q[1],-q[2],-q[3])
        return raw
    score=sequence(task,mutate)
    assert score['outcome']=='task_failure' and score['quality_violations']
    if defect=='missing_pose':
        assert score['missing_pose_count']==1 and score['available_pose_fraction']==.5


@pytest.mark.parametrize('defect', ['contact','force','torque','body_rate','range','speed'])
def test_hard_physical_violation_vetoes_perfect_pose_estimates(task,defect):
    def mutate(kind,index,raw):
        if kind=='physics' and index==10:
            if defect=='contact':raw['contacts']=1
            elif defect=='force':raw['applied_motor_force_body_n']=(1.,0.,0.)
            elif defect=='torque':raw['applied_wheel_torque_xyz_nm']=(.1,0.,0.)
            elif defect=='body_rate':raw['target_angular_velocity_body_rad_s']=(1.,0.,0.)
            elif defect=='range':raw['chaser_com_j2000']=raw['target_com_j2000']
            else:raw['chaser_com_j2000']=(*raw['chaser_com_j2000'][:3],0.,9000.,0.)
        return raw
    assert sequence(task,mutate)['outcome']=='safety_violation'


@pytest.mark.parametrize('defect', ['physics_gap','overwritten_state','image_gap','old_image','wrong_mount',
                                  'wrong_calibration','new_session','different_image','old_estimate','missing_estimate'])
def test_truth_and_image_evidence_must_be_fresh_complete_and_consistent(task,defect):
    def mutate(kind,index,raw):
        if kind=='physics' and index==10:
            if defect=='physics_gap':return None
            if defect=='overwritten_state':raw['external_state_writes']=1
        if kind=='capture' and index==1:
            if defect=='image_gap':return None
            if defect=='old_image':raw['source_frame_id']=10
            if defect=='new_session':raw['renderer_session_id']='another-renderer'
            if defect=='wrong_calibration':raw['calibration_sha256']=['b'*64]
            if defect=='wrong_mount':raw['camera_origins_j2000_m']=[(0.,0.,0.)]
        if kind=='estimate' and index==1:
            if defect=='different_image':raw['image_sha256']=['b'*64]
            if defect=='old_estimate':raw['estimate_time_ns']=0
            if defect=='missing_estimate':return None
        return raw
    assert sequence(task,mutate)['outcome']=='invalid_evidence'


@pytest.mark.parametrize('key,value', [('target_cooperation','N'),('vision_mode','V3'),
    ('controlled_spacecraft',1),('target_link','target_truth'),('source_id','D24'),
    ('initial_body_rates','spinning')])
def test_task_cannot_mislabel_its_sensor_cooperation_or_authority(task,key,value):
    data=deepcopy(task.model_dump())
    if key=='initial_body_rates':data['initial'][key]=value
    else:data[key]=value
    with pytest.raises(ValueError):
        VisionPoseTask.model_validate(data)


def test_stereo_requires_two_distinct_cameras_and_scores_the_same_left_reference(task):
    data=task.model_dump()
    data.update(source_id='D24',requirement_id='ssb.vision.stereo-pose')
    left=data['cameras'][0]
    data['cameras']=(left,dict(left,camera_id='team/camera/benchmark_right',position_body_m=(.22,-.9,.05)))
    stereo=VisionPoseTask.model_validate(data)
    assert sequence(stereo)['success']
    data['cameras']=(left,dict(left,camera_id='team/camera/benchmark_right'))
    with pytest.raises(ValueError,match='physical baseline'):
        VisionPoseTask.model_validate(data)
