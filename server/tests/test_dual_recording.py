"""One shared clock, two sensor streams, 14D training values, three real videos."""
import json
import numpy as np
import pytest
from space_arm_platform.models import parse_simulation_observation, EpisodeStart
from space_arm_platform.lerobot_capture import LiveLeRobotWriter
from space_arm_platform.recording_config import VideoEncodingConfig
from space_arm_platform.compact_storage import read_observations, read_wrench
from test_lerobot_recording import packet


def dual_observation(index):
    stamp=index*100_000_000
    arms={}
    # Deliberately reverse incoming order: the public training order stays left/right.
    for side, angle in [('right',.2),('left',.1)]:
        def wrench(t):
            return dict(sensor_name=side+'_wrist_ft',site_name=side+'_wrist_ft_site',
                parent_body=side+'_ee_base',child_body=side+'_wrist_ft_tool',frame='sensor_site',
                sign_convention='mujoco_force_torque',measurement='raw_interface_wrench',sample_phase='last_substep_start',
                valid=True,sample_time_ns=str(t-625000),interval_end_time_ns=str(t),force_n=[angle,2.,3.],torque_nm=[0.,0.,0.])
        arms[side]=dict(joint_position_rad=[angle]*6+[.4,-.4],joint_velocity_rad_s=[0.]*8,
            target_joint_position_rad=[angle+.01]*6+[.5,-.5],end_effector_position_body_m=[angle,0.,.3],
            end_effector_orientation_body_wxyz=[1.,0.,0.,0.],end_effector_twist_body=[0.]*6,
            wrist_wrench=wrench(stamp),wrist_wrench_rate_hz=200,
            wrist_wrench_samples=[wrench(t) for t in range(stamp-95_000_000,stamp+1,5_000_000)])
    return parse_simulation_observation(dict(protocol='space-arm-control/1',type='observation',observation_layout='dual-arm/1',
        simulation_id='dual-test',render_session_id='session-a',step_id=str(index),render_frame_id=str(index),
        sim_time_ns=str(stamp),wall_time_ns='1',applied_action_sequence='0',model_sha256='a'*64,
        model_revision='dual-v2',mujoco_runtime_version=3007000,gripper_type='four_bar_tendon',arms=arms))


@pytest.mark.parametrize('storage',['compact','diagnostic'])
def test_dual_official_roundtrip_and_independent_wrenches(tmp_path,storage):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    request=EpisodeStart(fps=10,camera_ids=['overview','left_wrist','right_wrist'],instruction='Left catches blue, right catches red.')
    episode=tmp_path/'episode';root=episode/'lerobot'
    writer=LiveLeRobotWriter(root,request,video_encoding=VideoEncodingConfig(encoder_threads=1,image_writer_threads=1),storage_profile=storage)
    for i in range(1,4):
        writer.append(dual_observation(i),{c:packet(i,c) for c in request.camera_ids})
    writer.finish('success')
    info=json.loads((root/'meta/info.json').read_text());platform=json.loads((root/'meta/platform.json').read_text())
    assert info['features']['observation.state']['shape']==[14]
    assert info['features']['action']['names'][0]=='left_joint_1_rad'
    assert info['features']['action']['names'][7]=='right_joint_1_rad'
    assert platform['arm_order']==['left','right']
    assert len(list((root/'videos').rglob('*.mp4')))==3
    loaded=LeRobotDataset('local/episode',root=root,video_backend='pyav')
    np.testing.assert_allclose(loaded[0]['observation.state'][:6],[.1]*6)
    np.testing.assert_allclose(loaded[0]['observation.state'][7:13],[.2]*6)
    np.testing.assert_allclose(loaded[0]['action'][7:13],[.21]*6)
    for side in ('left','right'):
        rows=list(read_wrench(episode,side))
        assert len(rows)==41 and int(rows[0]['sim_time_ns'])==100_000_000 and int(rows[-1]['sim_time_ns'])==300_000_000
        assert platform['high_frequency_wrist_wrench'][side]['samples']==41
    if storage=='compact':
        rows=list(read_observations(episode))
        assert len(rows)==3
        assert rows[0]['observation_layout']=='dual-arm/1'
        assert 'joint_position_rad' not in rows[0]
        np.testing.assert_allclose(rows[0]['arms']['right']['joint_position_rad'][:6],[.2]*6)
        assert rows[0]['arms']['left']['wrist_wrench']['sensor_name']=='left_wrist_ft'


def test_dual_schema_rejects_missing_arm_swapped_sensor_and_mismatched_time():
    obs=dual_observation(1).model_dump(mode='json')
    del obs['arms']['right']
    with pytest.raises(ValueError):parse_simulation_observation(obs)
    obs=dual_observation(1).model_dump(mode='json');obs['arms']['left']['wrist_wrench']['sensor_name']='right_wrist_ft'
    with pytest.raises(ValueError):parse_simulation_observation(obs)
    obs=dual_observation(1).model_dump(mode='json');obs['arms']['left']['wrist_wrench']['interval_end_time_ns']='99999999'
    with pytest.raises(ValueError):parse_simulation_observation(obs)


@pytest.mark.parametrize('field,value',[('site_name','right_wrist_ft_site'),('child_body','right_wrist_ft_tool')])
def test_reject_cross_arm_sensor_attachment(field,value):
    obs=dual_observation(1).model_dump(mode='json')
    obs['arms']['left']['wrist_wrench'][field]=value
    with pytest.raises(ValueError):parse_simulation_observation(obs)
    obs=dual_observation(1).model_dump(mode='json')
    obs['arms']['left']['wrist_wrench_samples'][0][field]=value
    with pytest.raises(ValueError):parse_simulation_observation(obs)
