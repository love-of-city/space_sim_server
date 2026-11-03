from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from simulation.scripted_tasks.plug_insert import PlugInsertTask,PlugInsertConfig,PlugSample

MODEL=Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'


@pytest.mark.parametrize('body',['aviation_plug','guide_plug','cylindrical_plug'])
def test_all_plugs_extract_with_measured_alignment_and_bounded_axial_lead(body):
    task=PlugInsertTask(MODEL,PlugInsertConfig(plug_body=body))
    assert task.profile.feedback_extraction
    position=np.r_[task.pickup[:2],.032]
    sample=PlugSample(0.,np.zeros(8),np.zeros(8),position,np.diag([1.,-1.,-1.]),np.zeros(3),np.zeros(3),np.zeros(6),True)
    task._plug_goal=position.copy();task._plug_goal[2]=.10
    task._advance_extraction_goal(sample,.005)
    assert position[2]<task._plug_goal[2]<=position[2]+.001
    task._plug_goal[2]=.10
    task._advance_extraction_goal(replace(sample,wrench=np.array([0,0,31.,0,0,0])),.005)
    assert task._plug_goal[2]<=position[2]
    task._plug_goal[2]=.10
    task._advance_extraction_goal(replace(sample,plug_position_body_m=position+np.array([.0004,0,0])),.005)
    assert task._plug_goal[2]<=position[2]


def test_seating_command_does_not_push_flange_through_coplanar_neighbor_rims():
    from test_scripted_plug_task import task_sample
    task,sample=task_sample();task._enter('seat',sample)
    gap=task._seat_z-task.hole[2]-task.config.flange_local_m
    assert gap==pytest.approx(.00025)
    assert gap<task.config.seat_tolerance_m/2
    with pytest.raises(ValueError):PlugInsertConfig(seat_clearance_m=.002)


@pytest.mark.parametrize('name',['aviation_plug_insert','guide_plug_insert','cylindrical_plug_insert'])
def test_all_connectors_unload_servo_before_opening(name):
    from test_independent_plug_tasks import make_task
    task,sample=make_task(name);task.reference[:6]+=.02
    task.reference[6:]=[.38,-.38]
    q=sample.joints.copy();q[6:]=[.44,-.44];sample=replace(sample,joints=q)
    before=task.reference.copy();task._enter('release',sample)
    assert task._release_unload_duration>=1.
    assert task._unload_before_release(sample)
    np.testing.assert_allclose(task.reference,before)
    reference,_=task.step(replace(sample,time_s=.1))
    np.testing.assert_allclose(reference[6:],before[6:])


def test_slow_measured_extraction_has_time_to_reach_transport_height():
    from test_scripted_plug_task import task_sample
    task,sample=task_sample();task._enter('lift',sample)
    assert task._timeout>100.  # Recorded 90 s timeout stopped 11.6 mm short.


def test_extraction_tracks_tilted_donor_independently_of_receiver():
    from simulation.serial_chain_kinematics import axis_angle_to_matrix
    task = PlugInsertTask(MODEL, PlugInsertConfig(plug_body='guide_plug'))
    task.pickup_rotation = axis_angle_to_matrix(np.array([1., 0., 0.]), .02)
    normal = task.pickup_rotation[:, 2]
    position = task.pickup + .027 * normal
    sample = PlugSample(0., np.zeros(8), np.zeros(8), position,
        task.pickup_rotation @ np.diag([1., -1., -1.]), np.zeros(3), np.zeros(3), np.zeros(6), True)
    task._plug_goal = position.copy()
    task._advance_extraction_goal(sample, .005)
    assert task._plug_goal[2] > position[2]
    np.testing.assert_allclose(np.cross(task._plug_goal-task.pickup, normal), 0., atol=1e-12)
    # The service-star recipient remains vertical; it must not block extraction.
    np.testing.assert_allclose(task.hole_rotation, np.eye(3), atol=1e-12)


def test_aviation_target_clears_measured_low_flange_without_changing_tolerance():
    cfg=PlugInsertConfig(plug_body='aviation_plug')
    assert cfg.seat_clearance_m==pytest.approx(.00075)
    assert cfg.seat_tolerance_m==.001
    assert cfg.seat_clearance_m-.000458>.00025


def test_guide_target_reserves_space_for_measured_seating_undershoot():
    cfg=PlugInsertConfig(plug_body='guide_plug')
    assert cfg.seat_clearance_m-.00026>.00025
    assert cfg.seat_clearance_m<cfg.seat_tolerance_m==.001


@pytest.mark.parametrize('name',['aviation_plug_insert','guide_plug_insert','cylindrical_plug_insert'])
def test_release_waits_for_the_actual_collision_checked_seating_pose(name,monkeypatch):
    from test_independent_plug_tasks import make_task
    task,sample=make_task(name)
    target=task.hole[2]+task.config.flange_local_m+task.config.seat_clearance_m
    sample=replace(sample,plug_position_body_m=np.r_[task.hole[:2],target-.0002],finger_plug_contacts=(1,1))
    task._enter('seat',sample)
    p,r=task.chain.forward(sample.joints[:6])
    task._grasp_transform=(r.T@(sample.plug_position_body_m-p),r.T@sample.plug_rotation_body)
    monkeypatch.setattr(task,'_part_feedback',lambda *args:None)
    entered=[];monkeypatch.setattr(task,'_next',lambda sample:entered.append(True))
    for i in range(1,250):task.step(replace(sample,time_s=i*.005))
    assert not entered and task.result is None
    sample=replace(sample,plug_position_body_m=np.r_[task.hole[:2],target])
    for i in range(250,500):task.step(replace(sample,time_s=i*.005))
    assert entered


def test_seating_recovery_unloads_then_realigns_before_axial_correction(monkeypatch):
    from test_independent_plug_tasks import make_task
    task,sample=make_task('guide_plug_insert')
    target=task.hole[2]+task.config.flange_local_m+task.config.seat_clearance_m
    sample=replace(sample,plug_position_body_m=np.r_[task.hole[:2],target-.0005],finger_plug_contacts=(1,1))
    task.reference[:6]+=.02;before=task.reference.copy();task._enter('seat',sample)
    p,r=task.chain.forward(sample.joints[:6]);task._grasp_transform=(r.T@(sample.plug_position_body_m-p),r.T@sample.plug_rotation_body)
    goals=[];monkeypatch.setattr(task,'_part_feedback',lambda *args:goals.append(task._plug_goal.copy()))
    command,_=task.step(replace(sample,time_s=.1))
    assert not goals
    assert np.max(abs(command[:6]-sample.joints[:6]))<np.max(abs(before[:6]-sample.joints[:6]))
    misaligned=replace(sample,time_s=1.1,plug_position_body_m=sample.plug_position_body_m+np.array([.0003,0,0]))
    task.step(misaligned)
    assert goals[-1][2]==pytest.approx(misaligned.plug_position_body_m[2])
    task.step(replace(sample,time_s=1.2))
    assert sample.plug_position_body_m[2]<goals[-1][2]<=sample.plug_position_body_m[2]+.0005
    task.step(replace(sample,time_s=1.3,plug_position_body_m=sample.plug_position_body_m+np.array([.00018,0,0])))
    assert goals[-1][2]>sample.plug_position_body_m[2]
    task.step(replace(sample,time_s=1.4,plug_position_body_m=sample.plug_position_body_m+np.array([.00023,0,0])))
    assert goals[-1][2]==pytest.approx(sample.plug_position_body_m[2])
    task.step(replace(sample,time_s=1.5,plug_position_body_m=sample.plug_position_body_m+np.array([.00018,0,0])))
    assert goals[-1][2]==pytest.approx(sample.plug_position_body_m[2])
    assert task._timeout>=60.
