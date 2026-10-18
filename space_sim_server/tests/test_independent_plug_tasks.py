"""Independent part identity, tracking and geometry-sensitive acceptance."""

from dataclasses import replace
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES
from simulation.scripted_tasks.plug_insert import (
    PlugInsertTask,
    PlugInsertConfig,
    PlugSample,
)
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
    task._enter("open", sample)
    before = task.reference.copy()
    moved = replace(
        sample, plug_position_body_m=sample.plug_position_body_m + [0.004, 0, 0]
    )
    original = moved.plug_position_body_m.copy()
    task._follow_pickup(moved, 0.005, before)
    p0, _ = task.chain.forward(before[:6])
    p1, _ = task.chain.forward(task.reference[:6])
    assert p1[0] > p0[0]
    assert (
        np.max(np.abs(task.reference[:6] - before[:6]))
        <= task.config.tracking_speed_rad_s * 0.005 + 1e-12
    )
    np.testing.assert_array_equal(moved.plug_position_body_m, original)


def test_contact_gate_does_not_accept_empty_grip():
    task, sample = make_task("cylindrical_plug_insert")
    task._enter("grip", sample)
    q = sample.joints.copy()
    q[6:] = [0.1, -0.1]
    task.step(replace(sample, time_s=7, joints=q, finger_plug_contacts=(0, 0)))
    assert task.stage == "grip"


@pytest.mark.parametrize("name", PLUG_PROFILES)
def test_final_acceptance_rejects_a_seated_but_spinning_part(name):
    task, sample = make_task(name)
    task._ever_lifted = True
    task._enter("verify", sample)
    q = sample.joints.copy()
    q[6:] = [0.65, -0.65]
    moving = replace(
        sample,
        joints=q,
        plug_position_body_m=task.hole + [0, 0, 0.025],
        plug_angular_velocity_body_rad_s=np.array([0.0, 0.0, 0.12]),
    )
    for tick in range(1, 90):
        task.step(replace(moving, time_s=tick / 30))
    assert task.result is None


def test_headless_default_target_matches_current_scene_model():
    from tools.run_scripted_task import default_target_state

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
