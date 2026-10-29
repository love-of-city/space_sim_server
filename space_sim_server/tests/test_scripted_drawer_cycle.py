"""Full-cycle acceptance must require release, retreat and a stable closed drawer."""

from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask, DrawerCycleConfig
from simulation.scripted_tasks.drawer_lift import DrawerSample
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def setup_cycle():
    task = DrawerCycleTask(MODEL)
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    sample = DrawerSample(0.0, q, np.zeros(8), 0.0, 0.0, 0.0, np.zeros(6), True, (0, 0))
    task.step(sample)
    return task, sample


def test_seeded_reset_preserves_scene_and_disables_all_noise():
    source = dict(
        environment={"lighting": {"sunlight_intensity_scale": 1.0}},
        randomization={"arm_joint_position_rad": [0.0] * 8},
    )
    first = randomized_scene(source, 17)
    assert first == randomized_scene(source, 17)
    assert first != randomized_scene(source, 18)
    assert source["randomization"]["arm_joint_position_rad"] == [0.0] * 8
    assert first["environment"] == source["environment"]
    assert not any(first["collection_config"]["disturbances"].values())
    offsets = first["collection_config"]["initialization"]["offset_deg"]
    assert all(-30 <= x <= 30 for x in offsets) and max(map(abs, offsets)) > 2
    assert first["randomization"]["arm_joint_position_rad"][-2:] == [0.0, 0.0]
    with pytest.raises(ValueError):
        randomized_scene(source, 1, float("nan"))


@pytest.mark.parametrize("half_range", [30.0, 50.0])
def test_wide_initialization_preserves_independent_offsets_without_clamping(half_range):
    baseline = np.deg2rad([179., -179., 179., -179., 179., -179.])
    source = {"randomization": {"arm_joint_position_rad": [*baseline, 0., 0.]}}
    scene = randomized_scene(source, 17, half_range)
    initialization = scene["collection_config"]["initialization"]
    offsets = np.asarray(initialization["offset_deg"])
    assert initialization["half_range_deg"] == half_range
    assert initialization["distribution"] == "independent_uniform"
    assert np.max(np.abs(offsets)) > 10
    assert np.all(np.abs(offsets) <= half_range)
    assert len(set(offsets)) == 6
    np.testing.assert_allclose(
        scene["randomization"]["arm_joint_position_rad"][:6], baseline + np.deg2rad(offsets))
    # Samples outside joint limits remain intact for the native preflight to reject.
    assert any(abs(value) > np.pi for value in scene["randomization"]["arm_joint_position_rad"][:6])
    assert list(source["randomization"]["arm_joint_position_rad"][:6]) == list(baseline)


@pytest.mark.parametrize("half_range", [-1.0, 50.1, float("nan"), float("inf")])
def test_initialization_rejects_invalid_half_ranges(half_range):
    with pytest.raises(ValueError, match=r"finite and in \[0,50\]"):
        randomized_scene({"randomization": {"arm_joint_position_rad": [0.] * 8}}, 17, half_range)


def test_opening_is_not_cycle_success_and_return_allows_pressed_latch():
    task, sample = setup_cycle()
    for stage in (*task.approach_stages, "descend", "lift"):
        task._enter(stage, sample)
        if task._path is not None:
            task.reference[:6] = task._path[-1]
    task._enter("return", sample)
    task.reference[:6] = task._path[-1]
    q = np.r_[task.reference[:6], 0.047, -0.047]
    sample = replace(sample, joints=q, buckle_rad=-0.137, finger_drawer_contacts=(3, 4))
    assert task.result is None and task.open_verified
    assert not task._closed(sample)
    assert task._stage_reached(sample, True, task.grasp)
    task.stage = "verify"
    assert not task._stage_reached(sample, True, task.hover)


def test_grasp_requires_both_fingers_on_drawer_not_just_partial_closure():
    task, sample = setup_cycle()
    task.stage = "grip"
    sample = replace(sample, joints=np.r_[sample.joints[:6], 0.047, -0.047])
    assert not task._stage_reached(sample, True, task.grasp)
    assert not task._stage_reached(
        replace(sample, finger_drawer_contacts=(5, 0)), True, task.grasp
    )
    assert task._stage_reached(
        replace(sample, finger_drawer_contacts=(5, 2)), True, task.grasp
    )


def test_final_verification_requires_closed_stationary_latch_and_open_hand():
    task, sample = setup_cycle()
    task.stage = "verify"
    task.open_verified = task.closed_verified = True
    opened = replace(sample, joints=np.r_[sample.joints[:6], 1.0, -1.0])
    assert not task._stage_reached(opened, True, task.hover)
    opened = replace(opened, time_s=task.config.verify_hold_s)
    assert task._stage_reached(opened, True, task.hover)
    for bad in (
        replace(opened, drawer_m=0.01),
        replace(opened, buckle_rad=-0.1),
        replace(opened, drawer_velocity_m_s=0.02),
        sample,
    ):
        assert not task._stage_reached(bad, True, task.hover)
    assert not task._stage_reached(opened, True, task.hover + 0.03)
    task.open_verified = False
    assert not task._stage_reached(opened, True, task.hover)


def test_cycle_config_rejects_invalid_close_tolerance():
    with pytest.raises(ValueError):
        DrawerCycleConfig(close_tolerance_m=-1)
    with pytest.raises(ValueError, match="overtravel"):
        DrawerCycleConfig(seating_overtravel_m=0.01)


def test_smooth_interpolation_keeps_real_drawer_cartesian_path():
    from simulation.serial_chain_kinematics import (
        axis_angle_to_matrix,
        rotation_matrix_to_vector,
    )

    task, sample = setup_cycle()
    for stage in (
        "raise",
        "above_drawer",
        "descend",
        "lift",
        "return",
        "seat",
        "retract",
    ):
        task._enter(stage, sample)
        if task._path is None:
            assert stage == "raise" and task._goal is not None
            continue
        p0, r0 = task.chain.forward(task._path[0])
        rv = rotation_matrix_to_vector(task._rotation @ r0.T)
        angle = np.linalg.norm(rv)
        # Include the midpoint of every former straight interpolation segment.
        for u in np.linspace(0, 1, 161):
            q = task._timing.path.at(u)
            assert np.all(q >= task.lower) and np.all(q <= task.upper)
            p, r = task.chain.forward(q)
            expected_r = (
                axis_angle_to_matrix(rv / angle, u * angle) @ r0
                if angle > 1e-12
                else r0
            )
            assert np.linalg.norm(p - (p0 + u * (task._goal - p0))) < 5e-6
            assert np.linalg.norm(rotation_matrix_to_vector(expected_r @ r.T)) < 5e-6
        task.reference[:6] = task._path[-1]


def test_gripper_is_smooth_and_contact_still_does_not_imply_completion():
    task, sample = setup_cycle()
    for stage, start, target, speed in (
        ("open", 0.0, 1.0, 0.2),
        ("grip", 1.0, 0.0, 0.2),
        ("release", 0.0, 1.0, 0.2),
    ):
        task.reference[6:] = [start, -start]
        task._enter(stage, sample)
        timing = task._gripper_timing
        assert timing is not None
        for t in np.linspace(0, timing.duration, 1001):
            q, v, a, j = timing.joint_state(t)
            assert 0 <= q[0] <= 1
            assert abs(v[0]) <= speed + 1e-9
            if stage == "grip" and q[0] <= task.config.grip_contact_range_rad:
                assert abs(v[0]) <= task.config.grip_contact_speed_rad_s + 1e-9
            assert abs(a[0]) <= task.config.gripper_acceleration_rad_s2 + 1e-9
            assert abs(j[0]) <= task.config.gripper_jerk_rad_s3 + 1e-9
        task._step_gripper(target, timing.duration)
        np.testing.assert_allclose(task.reference[6:], [target, -target], atol=1e-12)
    task.reference[6:] = [1.0, -1.0]
    task._enter("grip", sample)
    task.reference[6:] = 0.0
    contact = replace(
        sample,
        joints=np.r_[sample.joints[:6], 0.047, -0.047],
        finger_drawer_contacts=(3, 4),
    )
    assert not task._stage_reached(contact, True, task.grasp)


def test_approach_translates_and_rotates_on_the_same_smooth_path():
    from simulation.serial_chain_kinematics import rotation_matrix_to_vector

    task, sample = setup_cycle()
    task._enter("above_drawer", sample)
    p0, r0 = task.chain.forward(task._path[0])
    pm, rm = task.chain.forward(task._timing.path.at(0.5))
    assert "align" not in task.stages
    assert np.linalg.norm(pm - p0) > 0.01
    assert np.linalg.norm(rotation_matrix_to_vector(rm @ r0.T)) > 0.1
    np.testing.assert_allclose(pm, (p0 + task.hover) / 2, atol=5e-6)
    assert pm[2] >= task.hover[2] - 5e-6


def test_initial_wait_enters_direct_approach_and_requires_valid_sensor():
    task, sample = setup_cycle()
    assert not task._stage_reached(replace(sample, wrench_valid=False), True, task.hover)
    for tick in range(1, 73):
        task.step(replace(sample, time_s=tick * 0.005))
    assert task.stage == "approach"
    assert [event["stage"] for event in task.events] == ["settle", "approach"]
    assert task.config.initial_settle_s == 0.25


@pytest.mark.parametrize("fault", [
    {"drawer_velocity_m_s": 0.01}, {"buckle_rad": 0.1},
    {"finger_drawer_contacts": (1, 0)}, {"finger_drawer_contacts": None},
])
def test_retreat_counts_towards_closure_window_but_contact_or_motion_resets_it(fault):
    task, sample = setup_cycle()
    task.open_verified = task.closed_verified = True
    opened = replace(sample, joints=np.r_[sample.joints[:6], 1.0, -1.0])
    task.stage = "retract"
    # The arm need not be stationary for the released drawer to be observed.
    assert not task._stage_reached(opened, False, task.hover + 0.1)
    task.stage = "verify"
    later = replace(opened, time_s=2.1)
    assert task._stage_reached(later, True, task.hover)
    assert task._stable_time() == task.config.terminal_confirm_s
    assert not task._stage_reached(replace(later, **fault), True, task.hover)
    assert not task._stage_reached(later, True, task.hover)
    assert not task._stage_reached(replace(later, time_s=4.0), True, task.hover)
    assert task._stage_reached(replace(later, time_s=4.2), True, task.hover)


def test_recorded_stage_versions_preserve_existing_episode_validation():
    assert "align" in DrawerCycleTask.recorded_stages("drawer-cycle/3")
    assert "align" not in DrawerCycleTask.recorded_stages("drawer-cycle/4")
    with pytest.raises(ValueError, match="Unsupported drawer task version"):
        DrawerCycleTask.recorded_stages("drawer-cycle/unknown")


def test_empty_raise_still_requires_measured_pose_tracking():
    task, sample = setup_cycle()
    task._enter("raise", sample)
    assert task._path is None and task._goal is not None
    wrong = sample.joints.copy()
    wrong[0] += 0.1
    for tick in range(1, 50):
        task.step(replace(sample, time_s=tick * 0.005, joints=wrong))
    assert task.stage == "raise"
    assert task._stable == 0
