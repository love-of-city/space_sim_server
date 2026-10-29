"""A partially inserted round connector must finish seating before release."""

from dataclasses import replace

import numpy as np
import pytest

from test_independent_plug_tasks import make_task
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector


def inserted_sample(gap, angular_speed=0., name="cylindrical_plug_insert"):
    task, sample = make_task(name)
    position = task.hole + [0., 0., task.config.flange_local_m+gap]
    rotation = task.plug_rotation
    q = sample.joints.copy()
    q[:6] = solve_pose(
        task.chain, position+rotation @ task.config.grasp_offset_m,
        task.grasp_rotation, q[:6], task.lower, task.upper,
    )
    q[6:] = [.32, -.32]
    sample = replace(
        sample, joints=q, plug_position_body_m=position,
        plug_rotation_body=rotation, finger_plug_contacts=(3, 3),
        plug_angular_velocity_body_rad_s=np.array([0., 0., angular_speed]),
    )
    task.reference[:] = q
    task.reference[6:] = [.275, -.275]
    task._enter("extract", sample)  # Capture the actual held-part transform.
    task._enter("insert", sample)
    return task, sample


def test_observed_half_millimetre_gap_keeps_seating_feedback_active_and_jaws_closed():
    task, sample = inserted_sample(.0004837)
    before = task.reference.copy()
    start_position, _ = task.chain.forward(before[:6])
    # The failed native run satisfied the old 1-mm gate and low velocities.
    # It must now continue pushing toward the flange seat rather than open.
    for tick in range(1, 81):
        task.step(replace(sample, time_s=tick*.005))
    position, _ = task.chain.forward(task.reference[:6])
    assert task.stage == "insert" and task.result is None
    np.testing.assert_allclose(task.reference[6:], before[6:])
    assert position[2] < start_position[2]-1e-4


@pytest.mark.parametrize("angular_speed,expected_stage", [(0., "release"), (.0368, "insert")])
def test_seated_cylinder_releases_with_existing_stability_time_but_not_while_spinning(
    angular_speed, expected_stage,
):
    task, sample = inserted_sample(.000001, angular_speed)
    for tick in range(1, 35):
        task.step(replace(sample, time_s=tick*.005))
    assert task.stage == expected_stage
    assert task.result is None
    if expected_stage == "release":
        release = next(event for event in task.events if event["stage"] == "release")
        assert .145 <= release["sim_time_s"] <= .155


@pytest.mark.parametrize("angular_speed,expected_result", [(.029, "success"), (.0368, None)])
def test_final_spin_acceptance_stays_unchanged_after_seating_guard(angular_speed, expected_result):
    task, sample = make_task("cylindrical_plug_insert")
    task._ever_lifted = True
    task._enter("verify", sample)
    joints = sample.joints.copy()
    joints[6:] = [.65, -.65]
    seated = replace(
        sample, joints=joints, plug_position_body_m=task.hole+[0., 0., .025001],
        plug_angular_velocity_body_rad_s=np.array([0., 0., angular_speed]),
        finger_plug_contacts=(0, 0),
    )
    for tick in range(1, 131):
        task.step(replace(seated, time_s=tick*.005))
    assert task.result == expected_result


def release_sample(*, unloaded=True, contacts=(0, 1), name="cylindrical_plug_insert",
                   wrist_drift=True):
    task, sample = inserted_sample(.000001, name=name)
    task._enter("release", sample)
    task._release_unloaded = unloaded
    if unloaded:
        task._motion_s = task._duration
        task.reference[:6] = task.planned_path_reference(task._duration)[:6]
    if wrist_drift:
        p,r = task.chain.forward(sample.joints[:6])
        joints = sample.joints.copy()
        joints[:6] = solve_pose(
            task.chain,p+[.0003,.001,.0005],
            axis_angle_to_matrix(np.array([0.,0.,1.]),.025)@r,
            joints[:6],task.lower,task.upper,
        )
        sample = replace(sample,joints=joints)
    sample = replace(
        sample, finger_plug_contacts=contacts,
        plug_position_body_m=sample.plug_position_body_m + [0.,0.,.0005],
        plug_rotation_body=(axis_angle_to_matrix(np.array([0.,0.,1.]),.025)
                            @ sample.plug_rotation_body),
    )
    return task, sample


def test_release_does_not_counteract_part_drift_before_preload_unloads():
    task, disturbed = release_sample(unloaded=False)
    control, undisturbed = release_sample(unloaded=False)
    undisturbed = replace(
        undisturbed, plug_position_body_m=control.hole+[0.,0.,.025001],
        plug_rotation_body=control.plug_rotation,
    )
    actual,_ = task.step(replace(disturbed,time_s=.005))
    expected,_ = control.step(replace(undisturbed,time_s=.005))
    assert not task._release_unloaded
    np.testing.assert_allclose(actual,expected,atol=1e-12)


@pytest.mark.parametrize("contacts", [(1,0),(0,1),(1,1)])
def test_unloaded_cylinder_keeps_entry_pose_control_when_both_fingers_clear(contacts):
    task, sample = release_sample(contacts=contacts)
    start_p,start_r = task.chain.forward(task.reference[:6])
    first,_ = task.step(replace(sample,time_s=.005))
    p1,r1 = task.chain.forward(first[:6])
    second,_ = task.step(replace(sample,time_s=.010))
    p2,r2 = task.chain.forward(second[:6])
    # These are actual output references: compensate measured wrist deflection
    # toward the frozen release-entry pose across successive control ticks.
    assert p1[2] < start_p[2]-1e-6
    assert p2[2] < p1[2]-1e-6
    assert rotation_matrix_to_vector(r1@start_r.T)[2] < -1e-5
    assert rotation_matrix_to_vector(r2@r1.T)[2] < -1e-5
    assert second[6] > first[6]
    clear=replace(sample,finger_plug_contacts=(0,0))
    previous=second
    for tick in range(3,7):
        reference,_=task.step(replace(clear,time_s=tick*.005))
        delta=np.linalg.norm(reference[:6]-previous[:6])
        assert 1e-7<delta<np.sqrt(6)*.08*.005
        previous=reference
    assert reference[6] > second[6]
    assert task.result is None


def test_release_hold_does_not_chase_a_lifting_or_spinning_part():
    task, sample = release_sample(wrist_drift=False)
    # The sample's part moved and rotated, but the wrist is still at the safe
    # release-entry pose. It must not follow a recomputed slipping grasp frame.
    before = task.reference[:6].copy()
    for tick in range(1,5):
        reference,_ = task.step(replace(sample,time_s=tick*.005))
        np.testing.assert_allclose(reference[:6],before,atol=1e-10)


def test_release_hold_bounds_servo_preload_during_persistent_contact():
    task, sample = release_sample()
    for _ in range(1200):
        before = task.reference[:6].copy()
        task._hold_release_pose(sample,.005)
        assert np.max(np.abs(task.reference[:6]-before)) <= .08*.005+1e-12
        assert np.max(np.abs(task.reference[:6]-sample.joints[:6])) <= .06+1e-12
    assert np.all(task.reference[:6] >= task.lower)
    assert np.all(task.reference[:6] <= task.upper)


def test_stationary_blocked_release_wrist_has_finite_equilibrium_offset():
    task,sample=release_sample()
    p,r=task.chain.forward(sample.joints[:6])
    error=np.r_[task._goal-p,.2*rotation_matrix_to_vector(task._rotation@r.T)]
    jac=task.chain.jacobian(sample.joints[:6])
    jac[3:]*=.2
    correction=jac.T@np.linalg.solve(jac@jac.T+1e-6*np.eye(6),error)
    equilibrium=task._release_hold_joints+correction
    for _ in range(1200):
        task._hold_release_pose(sample,.005)
    np.testing.assert_allclose(task.reference[:6],equilibrium,atol=2e-9)
    previous=task.reference[:6].copy()
    for _ in range(1200):
        task._hold_release_pose(sample,.005)
    np.testing.assert_allclose(task.reference[:6],previous,atol=2e-9)


def test_release_velocity_feedback_opposes_measured_unloading_motion():
    stationary,sample=release_sample(wrist_drift=False)
    moving,moving_sample=release_sample(wrist_drift=False)
    velocity=np.array([.015,-.01,.01,.02,-.015,.01])
    moving_sample=replace(moving_sample,joint_velocities=np.r_[velocity,0.,0.])
    stationary._hold_release_pose(sample,.005)
    moving._hold_release_pose(moving_sample,.005)
    # At the same measured pose/reference, only measured velocity differs.
    # The additional command must oppose the real joint motion, not follow it.
    correction=moving.reference[:6]-stationary.reference[:6]
    assert correction@velocity<0.
    assert np.max(np.abs(correction))<.08*.005


def test_release_reference_preload_decays_continuously_at_the_entry_pose():
    task,sample=release_sample(wrist_drift=False,contacts=(0,0))
    preload=np.array([.006,-.004,.003,-.005,.002,-.001])
    task.reference[:6]=task._release_hold_joints+preload
    before=task.reference[:6].copy()
    task._hold_release_pose(sample,.005)
    expected=before-.005*3.*preload
    np.testing.assert_allclose(task.reference[:6],expected,atol=1e-10)
    assert np.linalg.norm(task.reference[:6]-task._release_hold_joints)>0.
    assert np.linalg.norm(task.reference[:6]-task._release_hold_joints)<np.linalg.norm(preload)


def test_last_contact_bit_does_not_switch_release_command_or_velocity():
    touching,touching_sample=release_sample(contacts=(0,1))
    clear,clear_sample=release_sample(contacts=(0,0))
    for tick in range(1,6):
        before=clear.reference.copy()
        q_touch,v_touch=touching.step(replace(touching_sample,time_s=tick*.005))
        q_clear,v_clear=clear.step(replace(clear_sample,time_s=tick*.005))
        np.testing.assert_allclose(q_clear,q_touch,atol=1e-12)
        np.testing.assert_allclose(v_clear,v_touch,atol=1e-12)
        np.testing.assert_allclose(v_clear,(q_clear-before)/.005,atol=1e-12)


@pytest.mark.parametrize("name", ["aviation_plug_insert","guide_plug_insert"])
def test_other_connector_release_does_not_gain_cylinder_contact_feedback(name):
    task, sample=release_sample(name=name)
    before=task.reference[:6].copy()
    for tick in range(1,5):
        reference,_=task.step(replace(sample,time_s=tick*.005))
        np.testing.assert_allclose(reference[:6],before,atol=1e-12)


def test_release_keeps_opening_until_geometry_gate_accepts_before_retract(monkeypatch):
    from simulation.scripted_tasks import plug_clearance
    task,sample=release_sample(contacts=(0,0),wrist_drift=False)
    opening=task.config.release_open_rad-.002
    sample=replace(sample,joints=np.r_[sample.joints[:6],opening,-opening],native_qpos=np.zeros(1))
    task.reference[6:]=[opening,-opening]
    task._gripper_timing=task._timing=task._path=None
    task._stable=.1
    checks=[]
    transitions=[]
    def geometry_gate(current,observed):
        checks.append((current.reference.copy(),observed.joints.copy()))
        return len(checks)>1
    monkeypatch.setattr(plug_clearance,'cylindrical_release_clearance',geometry_gate)
    monkeypatch.setattr(task,'_next',lambda observed:transitions.append(observed.time_s))
    first,_=task.step(replace(sample,time_s=.005))
    assert len(checks)==1 and not transitions
    second,_=task.step(replace(sample,time_s=.010))
    assert len(checks)==2 and transitions==[.010]
    assert second[6]>first[6]>opening


def test_release_geometry_failure_is_recorded_and_closes_private_probe(monkeypatch):
    from types import SimpleNamespace
    from simulation.scripted_tasks import plug_clearance
    task,sample=release_sample(contacts=(0,0),wrist_drift=False)
    opening=task.config.release_open_rad-.002
    sample=replace(sample,joints=np.r_[sample.joints[:6],opening,-opening],native_qpos=np.zeros(1))
    task.reference[6:]=[opening,-opening]
    task._gripper_timing=task._timing=task._path=None
    task._stable=.1
    closed=[]
    task._release_clearance_probe=SimpleNamespace(close=lambda:closed.append(True))
    def blocked(*args):
        raise ValueError('geometry-check budget')
    monkeypatch.setattr(plug_clearance,'cylindrical_release_clearance',blocked)
    task.step(replace(sample,time_s=.005))
    assert task.result=='failed'
    assert task.reason=='release_clearance_failed: geometry-check budget'
    assert closed==[True] and task._release_clearance_probe is None


def test_blocked_cylinder_retreat_passes_rising_alternatives_to_shared_search(monkeypatch):
    from simulation.scripted_tasks import environment_motion,plug_clearance
    task,sample=make_task('cylindrical_plug_insert')
    sample=replace(sample,native_qpos=np.zeros(1))
    task.environment_planning=True
    start,_=task.chain.forward(task.reference[:6])
    candidates=[]
    def blocked(current,observed,targets):
        candidates.extend(targets)
        current.retract_plan={'accepted':False,'attempts':[{'collision':'neighbor'}]}
        raise ValueError('direct_retracts_blocked')
    def searched(current,observed,goal,rotation,axis,**kwargs):
        goals=kwargs['fallback_positions']
        assert kwargs['error']=='direct_retracts_blocked'
        assert len(goals)==12
        np.testing.assert_allclose(goal,goals[0])
        assert all(position[2]>start[2]+.07 for position in goals)
        np.testing.assert_allclose(goals,[candidate['position'] for candidate in candidates])
        current._goal,current._rotation=goals[7],rotation
        return {'route':'searched','goal_candidate_index':7}
    monkeypatch.setattr(plug_clearance,'plan_cylindrical_retract',blocked)
    monkeypatch.setattr(environment_motion,'plan',searched)
    task._enter('retract',sample)
    assert task.retract_plan['height_m']==.12
    assert task.retract_plan['goal_candidate_index']==7
    assert task.retract_plan['attempts']==[{'collision':'neighbor'}]
    np.testing.assert_allclose(task._goal,candidates[7]['position'])
