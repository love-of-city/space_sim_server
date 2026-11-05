"""Recovery must transport the held body and cannot succeed at capture alone."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from test_floating_capture import realized, sample_at_grasp
from test_floating_options import parse
from simulation.scripted_tasks.floating_capture import create_floating_task, FloatingCaptureTask, FloatingCaptureConfig
from simulation.scripted_tasks.floating_options import apply_floating_options
from collection.collect_scripted_tasks import validate_floating_recording


def controlled_task(tmp_path):
    scene, model = realized(tmp_path)
    task = create_floating_task('floating_bolt_capture', model)
    task.config = replace(task.config, retract_speed_m_s=.2)
    task.chain = SimpleNamespace(forward=lambda q: (q[:3].copy(), np.eye(3)), jacobian=lambda q: np.eye(6),
        inverse_velocity_ik_pose=lambda q, v, **kw: SimpleNamespace(joint_velocity_rad_s=v))
    q = np.array([.6,.35,.36,0,0,0,1,-1.])
    sample = replace(sample_at_grasp(task, scene, contacts=(1,1)), joints=q, plug_rotation_body=np.eye(3),
                     plug_position_body_m=q[:3]-task.asset.grip_point)
    task.reference[:] = q
    # Identity grasp basis makes this a transparent actuator-following fixture.
    task.grasp_pose = lambda s: (s.plug_position_body_m+task.asset.grip_point, np.eye(3))
    return task, sample


def follow(task, sample, i):
    q, v = task.step(replace(sample, time_s=i*.005))
    return replace(sample, joints=q, joint_velocities=v, plug_velocity_body_m_s=v[:3],
                   plug_position_body_m=q[:3]-task.asset.grip_point)


def test_hold_retract_settle_moves_measurements_without_resetting_grasp(tmp_path):
    task, sample = controlled_task(tmp_path)
    held = None
    entered_settle = False
    for i in range(2000):
        previous = sample.joints.copy()
        sample = follow(task,sample,i)
        if task.stage == 'settle' and not entered_settle:
            # The position servo continues smoothly rather than snapping its
            # reference back to the measured joint pose at the stage boundary.
            assert 0 < np.linalg.norm(sample.joints[:3]-previous[:3])/.005 < .003
            entered_settle = True
        if task.stage == 'retract' and held is None:
            held = (task._held_local.copy(),task._held_rotation.copy())
            assert task.result is None and task._hold_completed >= 2.
        if task.result:
            break
    assert task.result == 'success'
    assert [e['stage'] for e in task.events] == [*task.stages,'success']
    assert np.linalg.norm(sample.joints[:3]-task.config.retract_position_body_m) < .003
    np.testing.assert_array_equal(task._held_local,held[0])
    np.testing.assert_array_equal(task._held_rotation,held[1])
    events={e['stage']:e['sim_time_s'] for e in task.events}
    assert events['retract']-events['hold'] >= 2.
    assert events['success']-events['settle'] >= 1.


def test_transport_accelerates_from_rest_instead_of_applying_full_speed(tmp_path):
    task, sample = controlled_task(tmp_path)
    for i in range(1000):
        sample = follow(task, sample, i)
        if task.stage == 'retract':
            break
    assert task.stage == 'retract'
    speeds = []
    for j in range(i+1, i+21):
        sample = follow(task, sample, j)
        speeds.append(float(np.linalg.norm(sample.joint_velocities[:3])))
    # Fixture transport limit is 0.2 m/s; after 100 ms the smooth envelope is
    # only 2.8% of that value, and its first increment starts near zero.
    assert 0 < speeds[0] < .00002
    assert np.all(np.diff(speeds) > 0)
    assert speeds[-1] <= .2*.028+1e-9


@pytest.mark.parametrize('defect',['contact','slip','rotation'])
def test_retraction_immediately_rejects_lost_grasp(tmp_path,defect):
    task,sample=controlled_task(tmp_path)
    for i in range(2000):
        sample=follow(task,sample,i)
        if task.stage=='retract':break
    if defect=='contact':sample=replace(sample,finger_plug_contacts=(1,0))
    elif defect=='slip':sample=replace(sample,plug_position_body_m=sample.plug_position_body_m+[.005,0,0])
    else:sample=replace(sample,plug_rotation_body=np.diag([-1.,-1.,1.]))
    task.step(replace(sample,time_s=(i+1)*.005))
    assert task.result=='failed' and task.reason=='grasp_lost_during_retraction'


def test_settle_does_not_count_time_away_from_destination(tmp_path):
    task,sample=controlled_task(tmp_path)
    for i in range(2000):
        sample=follow(task,sample,i)
        if task.stage=='settle':break
    q=sample.joints.copy();q[0]+=.01
    sample=replace(sample,joints=q,joint_velocities=np.zeros(8),plug_velocity_body_m_s=np.zeros(3),
                   plug_position_body_m=q[:3]-task.asset.grip_point)
    for step in range(i+1,i+301):task.step(replace(sample,time_s=step*.005))
    assert task.result == 'failed' and task.reason == 'retraction_destination_lost'


def recording():
    params=FloatingCaptureConfig().snapshot();steps=[]
    start=np.array([.6,.35,.36]);end=np.array(params['retract_position_body_m'])
    for stage,count in [('open',2),('pregrasp',2),('intercept',2),('grip',2),('hold',105),('retract',80),('settle',55)]:
        for i in range(count):
            p=start if stage in ('open','pregrasp','intercept','grip','hold') else end if stage=='settle' else start+(end-start)*i/(count-1)
            task=dict(task_version=FloatingCaptureTask.version,target_body='bolt',result='success',
                task_parameters=params,held_seconds=2.,settled_seconds=1.,stage=stage,
                feedback_source='authoritative_simulation_state',finger_target_contacts=[1,1],
                finger_normal_force_n=[1.,1.],slip_m=0.,angular_slip_rad=0.,relative_speed_m_s=0.,
                target_position_body_m=(p+[0,0,.01]).tolist(),retraction_start_position_body_m=start.tolist(),
                retraction_target_position_body_m=end.tolist(),retraction_target_rotation_body=np.eye(3).tolist())
            steps.append(dict(sim_time_ns=str(len(steps)*20_000_000),scripted_task=deepcopy(task),
                end_effector_position_body_m=p.tolist(),end_effector_orientation_body_wxyz=[1.,0,0,0],end_effector_twist_body=[0.]*6))
    return steps


def test_recording_requires_real_transport_and_all_stable_windows():
    validate_floating_recording(recording(),list(FloatingCaptureTask.stages),'floating_bolt_capture')


@pytest.mark.parametrize('defect',['hold_gap','drop','slip','no_transport','object_left_behind','away','spin','too_short','missing_stage','bad_channels'])
def test_recording_rejects_false_success(defect):
    rows=recording()
    by_stage=lambda stage: [r for r in rows if r['scripted_task']['stage']==stage]
    if defect=='hold_gap':by_stage('hold')[-40]['scripted_task']['finger_normal_force_n']=[0.,1.]
    elif defect=='drop':by_stage('retract')[40]['scripted_task']['finger_target_contacts']=[1,0]
    elif defect=='slip':by_stage('retract')[40]['scripted_task']['slip_m']=.004
    elif defect=='no_transport':
        for r in rows:r['end_effector_position_body_m']=[.3,.35,.36]
    elif defect=='object_left_behind':
        for r in rows:r['scripted_task']['target_position_body_m']=[.6,.35,.37]
    elif defect=='away':by_stage('settle')[20]['end_effector_position_body_m']=[.32,.35,.36]
    elif defect=='spin':by_stage('settle')[20]['end_effector_twist_body'][-1]=.1
    elif defect=='too_short':
        for r in rows:r['scripted_task']['task_parameters']['hold_s']=1.
    elif defect=='missing_stage':rows=[r for r in rows if r['scripted_task']['stage']!='retract']
    elif defect=='bad_channels':by_stage('grip')[0]['scripted_task']['finger_target_contacts']=[1,.5]
    with pytest.raises(AssertionError):validate_floating_recording(rows,list(FloatingCaptureTask.stages),'floating_bolt_capture')


def test_cli_retraction_and_finite_destination():
    args=parse('--retract-position','.30','.35','.4','--retract-speed','.02')
    result=apply_floating_options({},args)
    assert result['floating_retraction']==dict(position_body_m=[.30,.35,.4],speed_m_s=.02)
    for pos in ([1,2],[0,0,float('nan')],[0,0,float('inf')]):
        with pytest.raises(ValueError):FloatingCaptureConfig(retract_position_body_m=pos)
    with pytest.raises(ValueError):apply_floating_options({},parse('--retract-speed','0'))
    with pytest.raises(ValueError):apply_floating_options({},args,prepared=True)
