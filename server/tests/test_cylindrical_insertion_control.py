from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
from simulation.scripted_tasks.scene_plug import PlugInsertTask,PlugSample,PlugInsertConfig
from simulation.serial_chain_kinematics import axis_angle_to_matrix

MODEL=Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'

def setup():
    task=PlugInsertTask(MODEL)
    task._seat_z=.03;task._plug_goal=np.array([.535,.355,.03])
    sample=PlugSample(127.,np.zeros(8),np.zeros(8),np.array([.535,.355,.038]),
                      np.diag([1.,-1.,-1.]),np.zeros(3),np.zeros(3),np.zeros(6),True)
    return task,sample

def test_axial_goal_cannot_run_far_ahead_of_stalled_part():
    task,sample=setup();task._advance_insertion_goal(sample,.005)
    assert task._plug_goal[2]>=sample.plug_position_body_m[2]-.001
    assert task.config.force_limit_n==40 and task.config.torque_limit_nm==4

@pytest.mark.parametrize('change', ['tilt','radial','torque','force'])
def test_realign_before_advancing_under_side_contact(change):
    task,sample=setup()
    if change=='tilt':sample=replace(sample,plug_rotation_body=axis_angle_to_matrix([1,0,0],.003)@sample.plug_rotation_body)
    if change=='radial':sample=replace(sample,plug_position_body_m=sample.plug_position_body_m+np.array([.0003,0,0]))
    if change=='torque':sample=replace(sample,wrench=np.array([0,0,0,0,3.1,0]))
    if change=='force':sample=replace(sample,wrench=np.array([0,0,31.,0,0,0]))
    task._advance_insertion_goal(sample,.005)
    assert task._plug_goal[2]>=sample.plug_position_body_m[2]

def test_aligned_unloaded_part_keeps_nominal_insertion_speed():
    task,sample=setup();task._plug_goal[2]=.038
    task._advance_insertion_goal(sample,.005)
    assert task._plug_goal[2]==pytest.approx(.038-.002*.005)


def test_optional_velocity_damping_brakes_a_descending_aligned_part():
    from scene_plug_fixtures import task_sample
    task,sample=task_sample()
    task.config=replace(task.config,part_velocity_damping=2.)
    task.stage='insert'
    task._plug_goal=sample.plug_position_body_m.copy()
    task.plug_rotation=sample.plug_rotation_body.copy()
    start=task.reference.copy()
    moving=replace(sample,plug_velocity_body_m_s=np.array([0.,0.,-.003]))
    task._part_feedback(moving,.005)
    velocity=(task.reference[:6]-start[:6])/.005
    assert (task.chain.jacobian(sample.joints[:6])[:3]@velocity)[2] > 0.
    assert moving.plug_velocity_body_m_s[2] == -.003
    assert task.config.force_limit_n==40. and task.config.torque_limit_nm==4.
    task.stage='lift'
    task.reference[:]=start
    task._part_feedback(moving,.005)
    assert task.reference == pytest.approx(start)


def test_axial_damping_does_not_feed_contact_spin_into_orientation_control():
    from scene_plug_fixtures import task_sample
    task,sample=task_sample()
    task.config=replace(task.config,part_velocity_damping=2.)
    task.stage='insert'
    task._plug_goal=sample.plug_position_body_m.copy()
    task.plug_rotation=sample.plug_rotation_body.copy()
    initial=task.reference.copy()
    spinning=replace(sample,plug_angular_velocity_body_rad_s=np.array([.1,-.1,1.6]))
    task._part_feedback(spinning,.005)
    assert task.reference == pytest.approx(initial)


def test_insert_step_applies_stalled_part_guard_before_feedback():
    task,sample=setup()
    task.stage='insert';task._entered=sample.time_s;task._last_time=sample.time_s-.005;task._timeout=90.
    p,r=task.chain.forward(sample.joints[:6])
    task._grasp_transform=(r.T@(sample.plug_position_body_m-p),r.T@sample.plug_rotation_body)
    task.plug_rotation=sample.plug_rotation_body.copy()
    task.step(sample)
    assert task._plug_goal[2]>=.037
    assert task.result is None

def test_guide_cancels_axial_lead_under_preload():
    _,sample=setup()
    task=PlugInsertTask(MODEL,PlugInsertConfig(plug_body='guide_plug'))
    task._seat_z=.03;task._plug_goal=np.array([.535,.275,.038])
    task._advance_insertion_goal(replace(sample,plug_position_body_m=np.array([.535,.275,.038]),
        wrench=np.array([0,0,31.,0,0,0])),.005)
    assert task._plug_goal[2]>=.038


@pytest.mark.parametrize('body',['guide_plug','aviation_plug'])
def test_keyed_head_realigns_before_descending(body):
    _,sample=setup();task=PlugInsertTask(MODEL,PlugInsertConfig(plug_body=body))
    position=np.r_[task.hole[:2],.038]
    task._seat_z=.03075;task._plug_goal=position.copy();task._plug_goal[2]=.031
    sample=replace(sample,plug_position_body_m=position,
        plug_rotation_body=axis_angle_to_matrix([0,0,1],.006)@sample.plug_rotation_body)
    task._advance_insertion_goal(sample,.005)
    assert task._plug_goal[2]>=position[2]

@pytest.mark.parametrize('preload',[.04,.12])
def test_cylinder_unloads_arm_before_opening_without_reference_jump(preload):
    from scene_plug_fixtures import task_sample
    task,sample=task_sample()
    task.reference[:6]+=np.array([preload,-.02,0,0,0,0])
    start=task.reference.copy();task._enter('release',sample)
    previous=start.copy()
    duration=task._release_unload_duration
    ticks=int(np.ceil(duration/.005))
    for stamp in np.linspace(0,duration,ticks+1)[1:]:
        reference,velocity=task.step(replace(sample,time_s=stamp))
        assert reference[6:]==pytest.approx(start[6:])
        assert np.max(np.abs(velocity[:6]))<=task.config.arm_speed_rad_s+1e-9
        previous=reference
    assert previous[:6]==pytest.approx(sample.joints[:6])
    reference,_=task.step(replace(sample,time_s=duration+.005))
    assert reference[:6]==pytest.approx(sample.joints[:6])
    assert reference[6]>start[6]


def test_slow_release_allows_commanded_opening_before_timeout():
    from scene_plug_fixtures import task_sample
    task, sample = task_sample()
    task.config = replace(task.config, release_speed_rad_s=.02)
    task.reference[6:] = [.03, -.03]
    task._enter('release', sample)
    elapsed = task._release_unload_duration + .005
    while task.reference[6] < task.config.release_open_rad:
        task.step(replace(sample, time_s=elapsed))
        assert task.result is None
        elapsed += .005
    assert elapsed > 30.
