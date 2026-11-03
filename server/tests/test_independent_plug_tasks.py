"""Independent part identity, tracking and geometry-sensitive acceptance."""

from dataclasses import replace
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES
from simulation.scripted_tasks.plug_insert import PlugInsertTask, PlugSample
from simulation.scripted_tasks.plug_motion_config import PlugInsertConfig
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import (
    quaternion_wxyz_to_matrix,
    axis_angle_to_matrix,
)
from space_arm_platform.collection_jobs import TASKS, CollectionCreate
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
from pathlib import Path

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def make_task(name):
    profile = PLUG_PROFILES[name]
    task = PlugInsertTask(MODEL, PlugInsertConfig(plug_body=profile.body))
    body = ET.parse(MODEL).getroot().find(f'./worldbody/body[@name="{profile.body}"]')
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    sample = PlugSample(
        0.0,
        q,
        np.zeros(8),
        np.fromstring(body.get("pos"), sep=" "),
        quaternion_wxyz_to_matrix(np.fromstring(body.get("quat"), sep=" ")),
        np.zeros(3),
        np.zeros(3),
        np.zeros(6),
        True,
    )
    task.step(sample)
    return task, sample


@pytest.mark.parametrize("name", PLUG_PROFILES)
def test_independent_target_body_and_manager_dispatch(name):
    task, sample = make_task(name)
    profile = PLUG_PROFILES[name]
    assert task.name == name
    assert task.config.target_site == profile.target_site
    assert task.config.shaft_tip_local_m == profile.tip_m
    assert np.linalg.norm(task.hole - task.pickup) > 0.05
    assert TASKS[name].arguments == ("--task", name)
    assert CollectionCreate(task=name).task == name
    with pytest.raises(ValueError, match="forward assembly hole"):
        PlugInsertConfig(plug_body=profile.body, target_site=profile.prefix + "_pickup")
    other = next(p for p in PLUG_PROFILES.values() if p.body != profile.body)
    with pytest.raises(ValueError):
        PlugInsertConfig(plug_body=profile.body, target_site=other.target_site)


@pytest.mark.parametrize("name", PLUG_PROFILES)
def test_short_aviation_and_long_plugs_require_their_actual_insertion_depth(name):
    task, sample = make_task(name)
    task._ever_lifted = True
    task._enter("verify", sample)
    q = sample.joints.copy()
    q[6:] = [0.65, -0.65]
    seated = replace(sample, joints=q, plug_position_body_m=task.hole + [0, 0, 0.025])
    for i in range(1, 65):
        task.step(replace(seated, time_s=i / 30))
    assert task.result == "success"
    assert task.metrics(seated)["insertion_depth_m"] == pytest.approx(
        task.profile.tip_m - 0.025
    )


@pytest.mark.parametrize("name", ["guide_plug_insert", "aviation_plug_insert"])
def test_keyed_part_cannot_pass_with_yaw_error(name):
    task, sample = make_task(name)
    task._ever_lifted = True
    task._enter("verify", sample)
    q = sample.joints.copy()
    q[6:] = [0.65, -0.65]
    wrong = replace(
        sample,
        joints=q,
        plug_position_body_m=task.hole + [0, 0, 0.025],
        plug_rotation_body=axis_angle_to_matrix(np.array([0.0, 0.0, 1.0]), 0.08)
        @ sample.plug_rotation_body,
    )
    for i in range(1, 90):
        task.step(replace(wrong, time_s=i / 30))
    assert task.result is None


@pytest.mark.parametrize("name", PLUG_PROFILES)
def test_seated_completion_does_not_wait_for_residual_motion(name):
    task, sample = make_task(name)
    task._ever_lifted = True
    task._enter("verify", sample)
    q = sample.joints.copy()
    q[6:] = [0.65, -0.65]
    seated = replace(
        sample, joints=q, finger_plug_contacts=(0, 0),
        plug_position_body_m=task.hole + [0, 0, 0.025],
        plug_velocity_body_m_s=np.array([0.004, 0., 0.]),
        plug_angular_velocity_body_rad_s=np.array([0.1, 0.1, 0.]),
    )
    for i in range(1, 40):
        task.step(replace(seated, time_s=i * .005))
    assert task.result is None  # short geometric confirmation is still required
    task.step(replace(seated, time_s=.2))
    assert task.result == "success"
    assert task.metrics(seated)["plug_angular_velocity_body_rad_s"] == [0.1, 0.1, 0.]


def test_geometric_confirmation_restarts_when_plug_leaves_seat():
    task, sample = make_task("cylindrical_plug_insert")
    task._ever_lifted = True
    task._enter("verify", sample)
    q = sample.joints.copy()
    q[6:] = [.65, -.65]
    seated = replace(sample, joints=q, plug_position_body_m=task.hole + [0, 0, .025])
    for i in range(1, 31):
        task.step(replace(seated, time_s=i * .005))
    task.step(replace(seated, time_s=.155, plug_position_body_m=task.hole + [0, 0, .028]))
    for i in range(32, 61):
        task.step(replace(seated, time_s=i * .005))
    assert task.result is None
    for i in range(61, 73):
        task.step(replace(seated, time_s=i * .005))
    assert task.result == "success"


@pytest.mark.parametrize("name", PLUG_PROFILES)
def test_saved_completion_accepts_residual_motion_but_keeps_geometry_and_history(name):
    from collection.recording_validation import validate_plug_recording

    task, sample = make_task(name)
    q = sample.joints.copy()
    q[6:] = [.65, -.65]
    task._ever_lifted = True
    seated = replace(sample, joints=q, plug_position_body_m=task.hole + [0, 0, .025],
                     plug_velocity_body_m_s=np.array([.004, 0., 0.]),
                     plug_angular_velocity_body_rad_s=np.array([.1, 0., .1]))
    metrics = {**task.metrics(seated), "result": "success"}
    stages = list(task.stages)
    steps = [{"scripted_task": {**metrics, "stage": stage,
                               "grasp_heading_error_rad": 0., "finger_plug_contacts": [1, 1]},
              "gripper_opening_normalized": .65} for stage in stages]
    validate_plug_recording(steps, stages, name)
    steps[-1]["scripted_task"]["seat_gap_m"] = .003
    with pytest.raises(AssertionError):
        validate_plug_recording(steps, stages, name)
    steps[-1]["scripted_task"]["seat_gap_m"] = 0.
    steps[-1]["scripted_task"]["task_version"] = 'plug-insert/14-straight-approach-gentle-release'
    with pytest.raises(AssertionError):
        validate_plug_recording(steps, stages, name)


def test_tracking_moves_reference_without_editing_sample():
    task, sample = make_task("cylindrical_plug_insert")
    task.reference[:6] = solve_pose(
        task.chain,
        task.grasp + [0, 0, 0.15],
        task.grasp_rotation,
        sample.joints[:6],
        task.lower,
        task.upper,
    )
    task._enter("grip", sample)
    before = task.reference.copy()
    moved = replace(
        sample, plug_position_body_m=sample.plug_position_body_m + [0.004, 0, 0]
    )
    original = moved.plug_position_body_m.copy()
    # The shared drift filter rejects single-tick disturbances but must follow
    # a sustained offset, without exceeding the reference tracking speed.
    for _ in range(60):
        previous = task.reference.copy()
        task._follow_pickup(moved, 0.005, previous)
        assert np.max(np.abs(task.reference[:6] - previous[:6])) <= (
            task.config.tracking_speed_rad_s * .005 + 1e-12)
    p0, _ = task.chain.forward(before[:6])
    p1, _ = task.chain.forward(task.reference[:6])
    assert p1[0] > p0[0]
    np.testing.assert_array_equal(moved.plug_position_body_m, original)


def test_contact_gate_does_not_accept_empty_grip():
    task, sample = make_task("cylindrical_plug_insert")
    task._enter("grip", sample)
    q = sample.joints.copy()
    q[6:] = [0.1, -0.1]
    task.step(replace(sample, time_s=7, joints=q, finger_plug_contacts=(0, 0)))
    assert task.stage == "grip"


@pytest.mark.parametrize("name", PLUG_PROFILES)
def test_historical_record_rejects_a_seated_but_spinning_part(name):
    from collection.recorded_task_contracts import recorded_stages
    from collection.recording_validation import validate_plug_recording

    task, sample = make_task(name)
    task._ever_lifted = True
    q = sample.joints.copy()
    q[6:] = [.65, -.65]
    moving = replace(sample, joints=q,
                     plug_position_body_m=task.hole + [0, 0, .025],
                     plug_angular_velocity_body_rad_s=np.array([0., 0., .12]))
    version = "plug-insert/14-straight-approach-gentle-release"
    stages = list(recorded_stages(name, version))
    metrics = {**task.metrics(moving), "task_version": version, "result": "success",
               "grasp_heading_error_rad": 0., "finger_plug_contacts": [1, 1]}
    steps = [{"scripted_task": {**metrics, "stage": stage},
              "gripper_opening_normalized": .65} for stage in stages]
    with pytest.raises(AssertionError):
        validate_plug_recording(steps, stages, name)


def test_headless_default_target_matches_current_scene_model():
    from collection.run_scripted_task import default_target_state

    state = default_target_state(MODEL)
    np.testing.assert_allclose(state["target_position_m"], [0.94, 0.039086, 0.4])
    assert state["target_angular_velocity_rad_s"] == [0.0, 0.0, 0.0]


def test_unreachable_next_stage_is_a_task_failure_not_a_recorder_crash(monkeypatch):
    task, sample = make_task("cylindrical_plug_insert")

    def unreachable(*args):
        raise ValueError("outside workspace")

    monkeypatch.setattr(task, "_enter", unreachable)
    task.step(replace(sample, time_s=1.0))
    assert task.result == "failed"
    assert task.reason == "planning_failed: outside workspace"
    np.testing.assert_array_equal(task.velocity, np.zeros(8))
