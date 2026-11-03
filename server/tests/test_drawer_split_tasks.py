from collection.recorded_task_contracts import recorded_stages, validate_recorded_stages
"""Independent directions must preserve physical initialization and acceptance."""

from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.model_preparation.drawer_task_initialization import prepare_drawer_initial_state
from simulation.scripted_tasks.drawer_lift import DrawerSample
from simulation.scripted_tasks.drawer_tasks import DRAWER_TASKS, create_drawer_task, drawer_task_class
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def setup_task(name, model=MODEL, drawer_m=None):
    task = create_drawer_task(name, model)
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    if drawer_m is None:
        drawer_m = 0.08 if name == "drawer_close" else 0.0
    sample = DrawerSample(0., q, np.zeros(8), drawer_m, 0., 0., np.zeros(6), True, (0, 0))
    task.step(sample)
    return task, sample


def test_open_initialization_matches_joint_coordinates_and_is_reversible(tmp_path):
    root = ET.parse(MODEL).getroot()
    body = root.find('.//body[@name="drawer_base"]')
    slide = body.find('joint[@name="drawer_base_slide"]')
    original = np.fromstring(body.get("pos"), sep=" ")
    original_limit = slide.get("range")
    report = prepare_drawer_initial_state(root, "drawer_close")
    np.testing.assert_allclose(np.fromstring(body.get("pos"), sep=" "), original + [0, 0, .08])
    assert float(slide.get("ref")) == .08
    assert slide.get("range") == original_limit
    assert report["runtime_state_writes"] is False
    assert report["drawer_slide_m"] == .08
    once = ET.tostring(root)
    prepare_drawer_initial_state(root, "drawer_close")
    assert ET.tostring(root) == once
    variant = tmp_path / "opened.xml"
    ET.ElementTree(root).write(variant)
    pending = create_drawer_task("drawer_close", variant)
    assert pending._initial_drawer == .08
    initialized, _ = setup_task("drawer_close", variant)
    base, _ = setup_task("drawer_close")
    np.testing.assert_allclose(initialized.grip_origin, base.grip_origin, atol=1e-12)
    np.testing.assert_allclose(initialized.grasp, base.grasp, atol=1e-12)
    prepare_drawer_initial_state(root, "drawer_open")
    assert float(slide.get("ref")) == 0
    np.testing.assert_allclose(np.fromstring(body.get("pos"), sep=" "), original, atol=1e-12)


def test_initialization_follows_randomized_drawer_position(tmp_path):
    root = ET.parse(MODEL).getroot()
    body = root.find('.//body[@name="drawer_base"]')
    original = np.fromstring(body.get("pos"), sep=" ") + [.07, -.035, 0]
    body.set("pos", " ".join(map(str, original)))
    prepare_drawer_initial_state(root, "drawer_close")
    np.testing.assert_allclose(np.fromstring(body.get("pos"), sep=" "), original + [0, 0, .08])


def test_invalid_initialization_does_not_silently_move_an_unknown_rest_pose():
    root = ET.parse(MODEL).getroot()
    root.find('.//joint[@name="drawer_base_slide"]').set("ref", ".03")
    with pytest.raises(ValueError, match="Unrecognized"):
        prepare_drawer_initial_state(root, "drawer_close")
    for value in (-.01, .2, float("nan")):
        with pytest.raises(ValueError, match="opening"):
            prepare_drawer_initial_state(root, "drawer_close", open_height_m=value)


@pytest.mark.parametrize("name,wrong", [("drawer_open", .08), ("drawer_close", 0.)])
def test_independent_tasks_reject_wrong_initial_state(name, wrong):
    task, _ = setup_task(name, drawer_m=wrong)
    assert task.result == "failed"
    assert task.reason.startswith("drawer_not_initially_")
    assert not task.open_verified


@pytest.mark.parametrize("name,operation", [("drawer_open", "lift"), ("drawer_close", "return")])
def test_straight_approach_targets_current_handle_and_has_only_requested_stroke(name, operation):
    task, sample = setup_task(name)
    assert task.result is None
    assert task.stage == "settle"
    assert ("return" not in task.stages) if name == "drawer_open" else ("lift" not in task.stages)
    task._enter("approach", sample)
    start, _ = task.chain.forward(sample.joints[:6])
    for u in np.linspace(0, 1, 31):
        q = task.approach_reference(u)
        p, _ = task.chain.forward(q[:6])
        np.testing.assert_allclose(p, start + u * (task.grasp - start), atol=5e-6)
        assert q[7] == -q[6]
    task.reference[:] = task.approach_reference(1.)
    task._enter(operation, sample)
    actual_goal, _ = task.chain.forward(task._path[-1])
    desired = (task.grip_origin + [0, 0, .08] if name == "drawer_open"
               else task.grip_origin - task.axis * task.config.seating_overtravel_m)
    np.testing.assert_allclose(actual_goal, desired, atol=5e-6)
    assert task.path_reference(.5).shape == (8,)


def test_open_success_requires_release_and_stability_without_reclosing():
    task, sample = setup_task("drawer_open")
    task.stage = "lift"
    held = replace(sample, joints=np.r_[sample.joints[:6], .047, -.047],
                   drawer_m=.08, finger_drawer_contacts=(3, 3))
    assert task._stage_reached(held, True, task.grasp + task.axis * .08)
    task._enter("release", held)
    assert task.open_verified and not task.closed_verified
    free = replace(held, time_s=1., joints=np.r_[sample.joints[:6], .7, -.7], finger_drawer_contacts=(0, 0))
    assert task._stage_reached(free, True, task.grasp + task.axis * .08)
    task.stage = "verify"
    free = replace(free, time_s=3.1, joints=np.r_[sample.joints[:6], 1., -1.])
    assert task._stage_reached(free, True, task.hover)
    assert task.metrics(free)["released_open_seconds"] == pytest.approx(2.1)
    drifted = replace(free, drawer_m=.082)
    assert task._stage_reached(drifted, True, task.hover)
    assert task.metrics(drifted)["peak_drawer_displacement_m"] == pytest.approx(.082)
    for fault in ({"drawer_m": 0.}, {"drawer_velocity_m_s": .01}, {"finger_drawer_contacts": (1, 0)}):
        assert not task._stage_reached(replace(free, **fault), True, task.hover)
        assert not task._stage_reached(free, True, task.hover)
        free = replace(free, time_s=free.time_s + 2.1)
        assert task._stage_reached(free, True, task.hover)


def test_close_verification_uses_closed_zero_and_requires_latch_reset():
    task, sample = setup_task("drawer_close")
    assert task.open_verified
    assert not task._closed(sample)
    closed = replace(sample, drawer_m=0., joints=np.r_[sample.joints[:6], 1., -1.])
    assert task._closed(closed)
    assert not task._closed(replace(closed, buckle_rad=-.1))
    task.closed_verified = True
    task.stage = "retract"
    task._stage_reached(closed, False, task.hover + .1)
    task.stage = "verify"
    assert task._stage_reached(replace(closed, time_s=2.1), True, task.hover)
    assert task.metrics(closed)["drawer_target_m"] == 0.
    assert task.metrics(closed)["drawer_initial_m"] == .08


def test_registry_keeps_original_cycle_and_direction_specific_versions():
    assert DRAWER_TASKS == ("drawer_cycle", "drawer_open", "drawer_close")
    for name in DRAWER_TASKS:
        cls = drawer_task_class(name)
        assert recorded_stages(cls.name, cls.version) == cls.stages
        assert create_drawer_task(name, MODEL, linear_speed_m_s=.05).config.linear_speed_m_s == .05
    with pytest.raises(ValueError, match="Unknown"):
        drawer_task_class("drawer_invalid")


@pytest.mark.parametrize("name", ["drawer_open", "drawer_close"])
def test_native_recording_metrics_accept_split_task_metadata(name):
    from simulation.scripted_tasks.native_state import DrawerState

    task, sample = setup_task(name)
    reader = object.__new__(DrawerState)
    reader.task = task
    metrics = reader.metrics(sample)
    assert metrics["drawer_position_m"] == sample.drawer_m
    assert metrics["drawer_initial_m"] == sample.drawer_m
    assert metrics["drawer_velocity_m_s"] == 0.
