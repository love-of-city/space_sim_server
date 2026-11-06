"""Fault injection into the real per-arm controller and sequential coordinator."""
from dataclasses import replace
import numpy as np
import pytest
from simulation.scripted_tasks.dual_floating_capture import DualFloatingTask
from simulation.scripted_tasks.floating_capture import create_floating_task
from test_floating_capture import realized,sample_at_grasp


def task_pair(tmp_path):
    scene,model=realized(tmp_path,'floating_plug_capture')
    coordinator=DualFloatingTask.__new__(DualFloatingTask)
    coordinator.tasks={side:create_floating_task('floating_plug_capture',model) for side in ('left','right')}
    samples={side:sample_at_grasp(task,scene,contacts=(1,1)) for side,task in coordinator.tasks.items()}
    for task in coordinator.tasks.values():task.stage='pending'
    coordinator.commands={side:s.joints.copy() for side,s in samples.items()}
    coordinator.velocities={side:np.zeros(8) for side in samples}
    coordinator.stage='left';coordinator.result=None;coordinator.reason=''
    coordinator.events=[];coordinator.right_start_s=None;coordinator.last_time=None
    return coordinator,samples


@pytest.mark.parametrize('fault,expected',[('sensor','wrist_sensor_unavailable'),('overload','wrist_load_limit')])
def test_parked_right_failure_stops_both_arms_before_handoff(tmp_path,fault,expected):
    task,samples=task_pair(tmp_path)
    right=samples['right']
    if fault=='sensor':right=replace(right,wrench_valid=False)
    else:right=replace(right,wrench=np.array([100.,0,0,0,0,0]))
    for tick in range(60):
        task.step({'left':replace(samples['left'],time_s=tick*.005),'right':replace(right,time_s=tick*.005)},
            right_preflight=lambda _:pytest.fail('Must not hand off an unsafe parked arm'))
        if task.result:break
    assert task.result=='failed' and task.reason=='right:'+expected and task.stage=='left'


def test_left_object_loss_during_right_motion_fails_whole_episode(tmp_path):
    task,samples=task_pair(tmp_path);left=task.tasks['left']
    sample=samples['left'];position,rotation=left.chain.forward(sample.joints[:6])
    left.stage='settle';left._stable=1.;left.completion_ready=True
    left._held_rotation=rotation.T@sample.plug_rotation_body
    left._held_local=np.array(left.metrics(sample)['grasp_point_tool_m'])
    left._retract_start=position-np.array([.1,0,0]);left._retract_rotation=rotation
    left.config=replace(left.config,retract_position_body_m=tuple(position))
    task.stage='right';task.right_start_s=0.
    samples['left']=replace(sample,finger_plug_contacts=(0,0),finger_normal_force_n=(0.,0.))
    task.step(samples)
    assert task.result=='failed' and task.reason=='left:grasp_lost_during_retraction'
    assert not any(e['stage']=='success' for e in task.events)
