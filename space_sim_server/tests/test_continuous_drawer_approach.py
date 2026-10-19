"""Continuous approach geometry, combined derivative limits and contact transitions."""
from dataclasses import replace
import numpy as np
import pytest
from test_scripted_drawer_cycle import setup_cycle
from simulation.serial_chain_kinematics import rotation_matrix_to_vector
from simulation.scripted_tasks.approach_path import FinishBlend
from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask


def test_blend_is_continuous_and_finishes_before_contact_without_time_stop():
    blend = FinishBlend(0.7)
    for boundary in (blend.finish-blend.width, blend.finish):
        left = np.array(blend.state(boundary-1e-9))
        right = np.array(blend.state(boundary+1e-9))
        np.testing.assert_allclose(left, right, atol=1e-4)
    assert blend.state(1) == (1.0, 0.0, 0.0, 0.0)


def test_curved_approach_rotates_and_opens_before_final_axial_entry():
    task, sample = setup_cycle()
    task._enter("approach", sample)
    profile = task._approach_profile
    p0, _ = profile.pose(0)
    for u in np.linspace(0, 1, 401):
        reference = task.approach_reference(u)
        p, r = task.chain.forward(reference[:6])
        expected_p, expected_r = profile.pose(u)
        np.testing.assert_allclose(p, expected_p, atol=5e-6)
        assert np.linalg.norm(rotation_matrix_to_vector(expected_r @ r.T)) < 5e-6
        assert 0 <= reference[6] <= 1 and reference[7] == -reference[6]
        if u >= profile.blend.finish:
            np.testing.assert_allclose((p-task.grasp)[:2], 0, atol=5e-6)
            assert reference[6] == 1
            assert np.linalg.norm(rotation_matrix_to_vector(task.grasp_rotation @ r.T)) < 5e-6
    midpoint = task.approach_reference(profile.blend.finish/2)
    p, r = task.chain.forward(midpoint[:6])
    assert 0 < midpoint[6] < 1
    assert p[2] < p0[2]-0.03  # Direct downward curve, not horizontal-then-vertical.
    assert np.linalg.norm(p[:2]-p0[:2]) > 0.01
    assert np.linalg.norm(rotation_matrix_to_vector(r @ profile.start_rotation.T)) > 0.1
    assert all(stage not in task.stages for stage in ("raise", "above_drawer", "align", "open", "descend", "hold", "seat"))


def test_all_combined_reference_derivatives_and_terminal_speed_are_bounded():
    task, sample = setup_cycle()
    task._enter("approach", sample)
    profile, timing, cfg = task._approach_profile, task._timing, task.config
    for t in np.linspace(0, timing.duration, 1201):
        u, v, a, j = timing.state(t)
        _, b1, b2, b3 = profile.blend.state(u)
        p1 = -profile.axis*profile.height-profile.lateral*b1
        p2, p3 = -profile.lateral*b2, -profile.lateral*b3
        measured = (
            (np.linalg.norm(p1*v), np.linalg.norm(p2*v*v+p1*a), np.linalg.norm(p3*v**3+3*p2*v*a+p1*j)),
            tuple(abs(profile.angle*x) for x in (b1*v, b2*v*v+b1*a, b3*v**3+3*b2*v*a+b1*j)),
            tuple(abs((1-profile.initial_opening)*x) for x in (b1*v, b2*v*v+b1*a, b3*v**3+3*b2*v*a+b1*j)),
        )
        limits = (
            (cfg.linear_speed_m_s, cfg.linear_acceleration_m_s2, cfg.linear_jerk_m_s3),
            (cfg.angular_speed_rad_s, cfg.angular_acceleration_rad_s2, cfg.angular_jerk_rad_s3),
            (cfg.gripper_speed_rad_s, cfg.gripper_acceleration_rad_s2, cfg.gripper_jerk_rad_s3),
        )
        assert np.all(np.array(measured) <= np.array(limits)+1e-8)
        _, qv, qa, qj = timing.joint_state(t)
        assert np.max(abs(qv)) <= cfg.arm_speed_rad_s+1e-8
        assert np.max(abs(qa)) <= cfg.joint_acceleration_rad_s2+1e-8
        assert np.max(abs(qj)) <= cfg.joint_jerk_rad_s3+1e-8
        if 0 < t < timing.duration:
            assert v > 0  # No interior stop at the orientation/entry handoff.
        if u >= 1-cfg.slow_distance_m/profile.height:
            assert measured[0][0] <= cfg.approach_speed_m_s+1e-8


def test_lift_requires_actual_drawer_opening_before_reversing():
    task, sample = setup_cycle()
    task.stage = "lift"
    sample = replace(sample, joints=np.r_[sample.joints[:6], 0.047, -0.047], finger_drawer_contacts=(2, 2))
    assert not task._stage_reached(sample, True, task.grasp)
    opened = replace(sample, drawer_m=task.config.lift_m)
    assert task._stage_reached(opened, True, task.grasp+task.axis*task.config.lift_m)
    assert not task._stage_reached(replace(opened, finger_drawer_contacts=(0, 2)), True, task.grasp)


def test_release_clearance_allows_retreat_while_opening_profile_continues():
    task, sample = setup_cycle()
    task._enter("approach", sample)
    task.reference[:] = task.approach_reference(1)
    task.reference[6:] = 0
    task._enter("release", sample)
    timing = task._gripper_timing
    task._step_gripper(1, timing.duration*0.7)
    before, elapsed = task.reference.copy(), task._gripper_motion_s
    free = replace(sample, time_s=elapsed, joints=before, finger_drawer_contacts=(0, 0))
    assert task.config.release_clearance_opening <= before[6] < 0.95
    assert task._stage_reached(free, True, task.grasp)
    assert not task._stage_reached(replace(free, finger_drawer_contacts=(1, 0)), True, task.grasp)
    task._enter("retract", free)
    assert task._gripper_timing is timing and task._gripper_motion_s == elapsed
    np.testing.assert_array_equal(task.reference, before)
    task._step_gripper(1, 0.005)
    assert task.reference[6] == pytest.approx(timing.joint_state(elapsed+0.005)[0][0])


def test_old_recordings_retain_their_original_stage_sequence():
    assert "align" in DrawerCycleTask.recorded_stages("drawer-cycle/3")
    assert "above_drawer" in DrawerCycleTask.recorded_stages("drawer-cycle/4")
    assert DrawerCycleTask.recorded_stages("drawer-cycle/5") == DrawerCycleTask.stages


def test_preflight_gripper_closes_real_four_bar_loops_at_every_opening():
    from simulation.scripted_tasks.gripper_preflight import ParallelogramGripper
    from test_scripted_drawer_cycle import MODEL
    gripper = ParallelogramGripper(MODEL)
    for opening in np.linspace(0, 1, 51):
        values = gripper.passive_positions([opening, -opening])
        assert values["ee_link21_joint"] == opening
        assert values["ee_finger1_joint"] == -opening
        assert values["ee_link22_joint"] == -opening
        assert values["ee_finger2_joint"] == opening
    with pytest.raises(ValueError):
        gripper.passive_positions([float("nan"), 0])


def test_different_loop_geometry_is_rejected_instead_of_assuming_link_angles(tmp_path):
    import xml.etree.ElementTree as ET
    from simulation.scripted_tasks.gripper_preflight import ParallelogramGripper
    from test_scripted_drawer_cycle import MODEL
    tree = ET.parse(MODEL)
    tree.getroot().find('.//site[@name="ee_finger1_link11_site"]').set("pos", "0.02 0 0")
    changed = tmp_path / "different-gripper.xml"
    tree.write(changed, encoding="utf-8")
    with pytest.raises(ValueError, match="does not close"):
        ParallelogramGripper(changed)
