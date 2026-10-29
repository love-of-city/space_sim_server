"""Contact location and motion handovers, including the observed jitter cases."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from simulation.scripted_tasks.aviation_bolt import AviationBoltTask
from simulation.scripted_tasks.threaded_motion import ThreadedSample, ThreadedTaskBase
from simulation.serial_chain_kinematics import axis_angle_to_matrix


MODEL = Path(__file__).resolve().parents[1]/"model/SARM/platform/sarm_20261012.xml"


def task_sample():
    task = AviationBoltTask(MODEL)
    q = np.r_[np.deg2rad([90., -60., 60., 0., -90., 0.]), 0., 0.]
    sample = ThreadedSample(0., q, np.zeros(8), task.initial_position.copy(),
                            task.initial_rotation.copy(), np.zeros(3), np.zeros(3),
                            np.zeros(6), True, (0, 0))
    task.reference[:] = q
    return task, sample


@pytest.mark.parametrize("body,flat", [("bolt", .0194855716), ("fastener", .0216506351)])
def test_outer_head_and_neck_contact_do_not_count_as_original_head_tab(body, flat):
    assert ThreadedTaskBase.is_latch_contact(body, [0., flat-.002, .009])
    assert ThreadedTaskBase.is_latch_contact(body, [0., -flat+.002, .009])
    for point in ([0., flat, .015], [0., flat-.002, .025], [.02, flat-.002, .009]):
        assert not ThreadedTaskBase.is_latch_contact(body, point)


def test_approach_drift_cannot_add_a_second_velocity_integrator(monkeypatch):
    task, sample = task_sample()
    task._enter("approach", sample)
    task._last_time = 0.
    def conflicting_feedback(*args, **kwargs):
        raise AssertionError("A running timed path must own the arm reference")
    monkeypatch.setattr(task, "_follow_pose", conflicting_feedback)
    moving = replace(sample, time_s=.005,
                     part_position_body_m=sample.part_position_body_m+[.001, 0., 0.],
                     part_rotation_body=axis_angle_to_matrix(np.array([0., 0., 1.]), .3)@sample.part_rotation_body)
    reference, _ = task.step(moving)
    np.testing.assert_allclose(reference, task.planned_path_reference(.005), atol=1e-12)


def test_stopped_path_discards_old_feedback_velocity_before_regrasp():
    task, sample = task_sample()
    task._command_rate[:] = .233
    task._command_acceleration[:] = .4
    task.velocity[:] = 0.
    task._step_finish(sample, task.reference.copy(), .005)
    position, rotation = task.chain.forward(sample.joints[:6])
    before = task.reference.copy()
    task._follow_pose(position, rotation, sample, .005)
    np.testing.assert_allclose(task.reference, before, atol=1e-12)
    np.testing.assert_allclose(task._command_rate, 0., atol=1e-12)


def test_feedback_restart_and_braking_keep_command_acceleration_and_jerk_bounded():
    task, sample = task_sample()
    position, rotation = task.chain.forward(sample.joints[:6])
    rotation = axis_angle_to_matrix(np.array([0., 0., 1.]), .04)@rotation
    velocities = [np.zeros(6)]
    for tick in range(1, 401):
        actual = replace(sample, time_s=tick*.005, joints=task.reference.copy())
        old = task.reference.copy()
        task._follow_pose(position, rotation, actual, .005)
        _, velocity = task._step_finish(actual, old, .005)
        velocities.append(velocity[:6])
    acceleration = np.diff(velocities, axis=0)/.005
    jerk = np.diff(np.vstack([np.zeros(6), acceleration]), axis=0)/.005
    assert np.max(np.abs(velocities[1])) < .00021
    assert np.max(np.abs(acceleration)) <= .700001
    assert np.max(np.abs(jerk)) <= 8.0001
    assert np.max(np.abs(velocities[-1])) < .0002
    _, final_rotation = task.chain.forward(task.reference[:6])
    assert np.linalg.norm(final_rotation-rotation) < .0002


def test_completed_turn_cannot_open_jaws_while_wrist_is_moving(monkeypatch):
    task, sample = task_sample()
    task.source_rotation_rad = task.source_required_rotation_rad
    sample = replace(sample, part_position_body_m=task.initial_position+task.outward*.059,
                     joint_velocities=np.r_[np.zeros(5), -.2, 0., 0.],
                     part_angular_velocity_body_rad_s=task.outward*.2)
    task._enter("unscrew", sample)
    task._spin_speed = .2
    task._command_rate[5] = -.2
    monkeypatch.setattr(task, "_track_part", lambda *args, **kwargs: None)
    task._spin(sample, .005)
    assert task.stage == "unscrew"
    assert not task.source_unscrewed


def test_measured_helix_feedback_rejects_servo_deflection_without_hunting():
    task, sample = task_sample()
    goal_position, goal_rotation = task.chain.forward(sample.joints[:6])
    goal_rotation=axis_angle_to_matrix(np.array([0.,0.,1.]),.04)@goal_rotation
    joints=sample.joints.copy()
    rate=np.zeros(8)
    velocities=[np.zeros(6)]
    # A loaded native-like PD servo: command-space convergence alone leaves
    # a persistent physical error. The measured helix loop must remove it.
    load=np.r_[np.zeros(5),1.]
    for tick in range(1,2001):
        actual=replace(sample,time_s=tick*.005,joints=joints.copy(),joint_velocities=rate.copy())
        old=task.reference.copy()
        task._follow_pose(goal_position,goal_rotation,actual,.005,measured_feedback=True)
        _,published=task._step_finish(actual,old,.005)
        velocities.append(published[:6])
        acceleration=500.*(task.reference[:6]-joints[:6])-50.*rate[:6]+load
        rate[:6]+=acceleration*.005
        joints[:6]+=rate[:6]*.005
    position,rotation=task.chain.forward(joints[:6])
    assert np.linalg.norm(position-goal_position)<.0001
    assert np.linalg.norm(rotation-goal_rotation)<.0002
    assert np.max(np.abs(rate[:6]))<.0002
    assert np.max(np.abs(velocities[-1]))<.0002
    acceleration=np.diff(velocities,axis=0)/.005
    jerk=np.diff(np.vstack([np.zeros(6),acceleration]),axis=0)/.005
    assert np.max(np.abs(acceleration))<=.700001
    assert np.max(np.abs(jerk))<=8.0001


def test_obstructed_helix_brakes_before_hitting_the_hard_reference_lead_limit():
    task,sample=task_sample()
    position,rotation=task.chain.forward(sample.joints[:6])
    rotation=axis_angle_to_matrix(np.array([0.,0.,1.]),.4)@rotation
    velocities=[np.zeros(6)]
    leads=[]
    # Keep the measured arm blocked. The unmet goal must not make the command
    # hit its hard +/-0.06 rad envelope and jump to a different velocity.
    for tick in range(1,1601):
        actual=replace(sample,time_s=tick*.005)
        old=task.reference.copy()
        task._follow_pose(position,rotation,actual,.005,measured_feedback=True)
        _,velocity=task._step_finish(actual,old,.005)
        velocities.append(velocity[:6])
        leads.append(task.reference[:6]-sample.joints[:6])
    acceleration=np.diff(velocities,axis=0)/.005
    jerk=np.diff(np.vstack([np.zeros(6),acceleration]),axis=0)/.005
    assert np.max(np.abs(leads))<.05
    assert np.max(np.abs(velocities[-1]))<.0002
    assert np.max(np.abs(acceleration))<=.700001
    assert np.max(np.abs(jerk))<=8.0001


@pytest.mark.parametrize('unloaded,measured_speed',[(False,0.),(True,.03)])
def test_clear_jaws_do_not_start_rewind_before_arm_unloading_finishes(monkeypatch,unloaded,measured_speed):
    task,sample=task_sample()
    task._enter('regrasp_release',sample)
    task._last_time=.495
    task._stable=.2
    sample=replace(sample,time_s=.5,joints=np.r_[sample.joints[:6],.4,-.4],
                   joint_velocities=np.r_[measured_speed,np.zeros(7)],finger_part_contacts=(0,0))
    monkeypatch.setattr(task,'_path_step',lambda *args:unloaded)
    monkeypatch.setattr(task,'_advance_opening',lambda *args:True)
    task.step(sample)
    assert task.stage=='regrasp_release'


def test_target_rotation_cannot_run_ahead_of_actual_axial_thread_engagement(monkeypatch):
    task,sample=task_sample()
    position=task.seated_position+task.target_outward*task.config.target_clearance_m
    sample=replace(sample,part_position_body_m=position)
    task._enter('screw_in',sample)
    initial_rotation=sample.part_rotation_body.copy()
    monkeypatch.setattr(task,'_track_part',lambda *args,**kwargs:None)
    # The screw follows angular motion but is axially blocked at the mouth.
    # The old controller would keep turning until its half-turn regrasp.
    for tick in range(1,1001):
        task.target_rotation_rad=task._stroke_turn
        observed=replace(sample,time_s=tick*.005,
                         part_rotation_body=axis_angle_to_matrix(task.target_outward,-task._stroke_turn)@initial_rotation)
        task._spin(observed,.005)
    assert task.stage=='screw_in'
    assert task._stroke_turn<.75
    assert task._spin_speed<.002


@pytest.mark.parametrize('stage',['unscrew','screw_in'])
def test_thread_goal_corrects_carried_tilt_instead_of_rotating_it_forever(stage):
    task,sample=task_sample()
    sample=replace(sample,part_rotation_body=axis_angle_to_matrix(np.array([1.,0.,0.]),.02)@sample.part_rotation_body)
    task._enter(stage,sample)
    axis=task.outward if stage=='unscrew' else task.target_outward
    np.testing.assert_allclose(task._spin_goal_rotation[:,2],-axis,atol=1e-12)
    np.testing.assert_allclose(task._spin_goal_rotation.T@task._spin_goal_rotation,np.eye(3),atol=1e-12)
