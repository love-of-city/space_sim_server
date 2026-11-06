"""Independent physical dwell, camera-frame kinematics and actual force credit."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY,load_task
from space_arm_platform.benchmark_external import ExternalMethodDefinition
from space_arm_platform.benchmark_vision import VisionPoseTruthSample
from space_arm_platform.benchmark_vision_approach import VisionApproachAction,VisionApproachTask
from space_arm_platform.benchmark_vision_approach_score import VisionApproachScorer,optical_relative_motion
from space_arm_platform.benchmark_vision_score import rotation
from test_benchmark_vision import truth,capture,estimate,quaternion


@pytest.fixture
def task():
    data=load_task(SPEC_DIRECTORY/'d37-stereo-body-approach-v1.json').model_dump()
    data['duration_s']=3.
    data['objective']['dwell_s']=1.
    data['initial'].update(inclination_deg=0.,true_anomaly_deg=0.)
    return VisionApproachTask.model_validate(data)


def physical(task,tick,*,at_goal=False):
    # Deliberate kinematic unit fixtures for the grader, not an environment or
    # a claimed native trajectory. Native execution is validated separately.
    raw=truth(tick)
    raw['chaser_com_j2000']=tuple(np.asarray(raw['chaser_com_j2000'])+[0.,2.,0.,0.,0.,0.])
    raw['chaser_origin_j2000_m']=tuple(np.asarray(raw['chaser_origin_j2000_m'])+[0.,2.,0.])
    if at_goal:
        camera=task.cameras[0]
        target_origin=np.asarray(raw['target_origin_j2000_m'])
        camera_origin=target_origin-rotation(camera.orientation_body_from_optical_wxyz)@task.objective.target_origin_optical_goal_m
        chaser_origin=camera_origin-camera.position_body_m
        raw['chaser_origin_j2000_m']=tuple(chaser_origin)
        com=np.asarray(raw['chaser_com_j2000'])
        com[:3]=chaser_origin+[.2,.3,-.1]
        raw['chaser_com_j2000']=tuple(com)
    raw['applied_motor_force_body_n']=(.1,0.,0.) if tick else (0.,0.,0.)
    return raw


def sequence(task,defect=None,end=100,*,return_scorer=False):
    scorer=VisionApproachScorer(task,169.4)
    for tick in range(end+1):
        raw=physical(task,tick,at_goal=tick>=50 and not(defect=='dwell_gap' and tick==80))
        if defect=='missing_motor_receipt' and tick==30:
            raw['applied_motor_force_body_n']=(0.,0.,0.)
        if defect=='contact' and tick==70:
            raw['contacts']=1
        if defect=='speed_gap' and tick==80:
            com=list(raw['chaser_com_j2000'])
            com[3]+=.1
            raw['chaser_com_j2000']=tuple(com)
        scorer.add(raw)
        if tick%25==0 and not(defect=='missing_terminal_image' and tick==end):
            index=tick//25
            scorer.add(capture(scorer,index))
            if tick<end or defect=='pending_terminal_command':
                command=estimate(scorer,index)
                command.update(record_type='vision_approach_action',force_frame=task.force_frame,force_body_n=(.1,0.,0.))
                scorer.add(command)
    return scorer if return_scorer else scorer.result()


def test_early_physical_dwell_requires_all_preceding_control_images_but_not_unused_future_ticks(task):
    result=sequence(task)
    assert result['success'],result
    assert result['dwell_s']==1. and result['completion_time_s']==2.
    assert result['expected_estimates']==result['available_pose_count']==4
    assert result['maximum_planned_estimates']==6 and result['capture_count']==5
    assert result['thrust_impulse_n_s']==pytest.approx(.2)


@pytest.mark.parametrize('end',[100,150])
def test_cancelled_terminal_capture_is_a_valid_prefix_but_cannot_certify_a_completed_dwell(task,end):
    result=sequence(task,'missing_terminal_image',end=end)
    assert result['dwell_s']>=task.objective.dwell_s
    assert result['outcome']=='incomplete' and not result['success']
    assert result['completion_time_s'] is None and not result['evidence_errors']
    assert result['capture_count']==result['estimate_count']==result['expected_estimates']


@pytest.mark.parametrize('end,defect',[(103,None),(100,'pending_terminal_command')])
def test_cancelled_native_interval_or_pending_command_remains_incomplete(task,end,defect):
    result=sequence(task,defect,end=end)
    assert result['outcome']=='incomplete' and not result['evidence_errors']
    assert not result['success'] and result['completion_time_s'] is None


def test_partial_motion_without_a_preceding_image_control_remains_invalid(task):
    scorer=sequence(task,end=100,return_scorer=True)
    # A physical interval cannot silently use the previous image's command
    # after the next decision instant, even if its force receipt matches.
    scorer.add(physical(task,101,at_goal=True))
    result=scorer.result()
    assert result['outcome']=='invalid_evidence'
    assert 'incomplete_visual_control_coverage' in result['evidence_errors']


@pytest.mark.parametrize('defect',['dwell_gap','speed_gap'])
def test_last_good_position_cannot_hide_an_interrupted_physical_dwell(task,defect):
    result=sequence(task,defect)
    assert result['outcome']=='incomplete' and result['dwell_s']<1.,result


def test_requesting_thrust_does_not_substitute_for_an_actual_motor_receipt(task):
    result=sequence(task,'missing_motor_receipt')
    assert result['outcome']=='invalid_evidence'
    assert 'body_force_receipt_differs_from_command' in result['evidence_errors']


def test_image_accuracy_and_goal_dwell_cannot_override_unplanned_contact(task):
    assert sequence(task,'contact')['outcome']=='safety_violation'


def test_camera_relative_velocity_includes_rotating_axes_and_both_body_origin_lever_arms(task):
    raw=physical(task,0)
    raw.update(target_com_j2000=(1.,4.,.3,.01,.02,-.04),chaser_com_j2000=(-.2,.1,.4,-.03,.01,.02),
        target_origin_j2000_m=(1.1,4.2,.25),chaser_origin_j2000_m=(-.4,.3,.5),
        target_orientation_body_to_j2000_wxyz=quaternion(Rotation.from_euler('xyz',[.2,-.3,.4])),
        chaser_orientation_body_to_j2000_wxyz=quaternion(Rotation.from_euler('xyz',[-.1,.25,-.3])),
        target_angular_velocity_body_rad_s=(.02,-.03,.04),chaser_angular_velocity_body_rad_s=(-.01,.04,.03))
    sample=VisionPoseTruthSample.model_validate(raw)
    position,velocity=optical_relative_motion(sample,task.cameras[0])
    dt=1e-6
    advanced=deepcopy(raw)
    for body in ('target','chaser'):
        old=np.asarray(raw[body+'_com_j2000'])
        orientation=rotation(raw[body+'_orientation_body_to_j2000_wxyz'])
        lever=orientation.T@(np.asarray(raw[body+'_origin_j2000_m'])-old[:3])
        updated=orientation@Rotation.from_rotvec(np.asarray(raw[body+'_angular_velocity_body_rad_s'])*dt).as_matrix()
        com=old.copy()
        com[:3]+=old[3:]*dt
        advanced[body+'_com_j2000']=tuple(com)
        advanced[body+'_origin_j2000_m']=tuple(com[:3]+updated@lever)
        advanced[body+'_orientation_body_to_j2000_wxyz']=quaternion(Rotation.from_matrix(updated))
    future,_=optical_relative_motion(VisionPoseTruthSample.model_validate(advanced),task.cameras[0])
    assert np.allclose((future-position)/dt,velocity,rtol=0,atol=1e-7)


@pytest.mark.parametrize('changes',[
    {'force_frame':'J2000'}, {'force_body_n':(6.,0.,0.)}, {'estimate_time_ns':500_000_000},
])
def test_body_force_action_rejects_wrong_frame_excess_thrust_and_stale_image(task,changes):
    raw=dict(episode_id='unit',observation_sequence=0,estimate_time_ns=0,
        reference_frame=task.reference_frame,pose=None,force_frame=task.force_frame,force_body_n=(0.,0.,0.))
    raw.update(changes)
    observation=SimpleNamespace(episode_id='unit',sequence=0,sample_time_ns=0,
        images=[SimpleNamespace(camera_id=camera.camera_id) for camera in task.cameras])
    with pytest.raises(ValueError):
        VisionApproachAction.model_validate(raw).validate_for(observation,task)


@pytest.mark.parametrize('name',['vision-approach-builtin.json','vision-approach-no-thrust.json',
                                'vision-approach-unsafe.json','vision-approach.json'])
def test_actual_external_definitions_admit_only_the_visual_control_schema(task,name):
    definition=ExternalMethodDefinition(SPEC_DIRECTORY.parents[2]/'examples/autonomy_methods'/name,task)
    assert definition.spec.task_schemas==(task.schema_version,)
    definition.verify_unchanged()


def test_relabeling_a_single_estimator_as_a_visual_driver_is_rejected(task):
    document=task.model_dump()
    document['controlled_spacecraft']=0
    with pytest.raises(ValueError):
        VisionApproachTask.model_validate(document)
