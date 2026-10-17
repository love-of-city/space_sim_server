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
    assert all(-10 <= x <= 10 for x in offsets) and max(map(abs, offsets)) > 2
    assert first["randomization"]["arm_joint_position_rad"][-2:] == [0.0, 0.0]
    with pytest.raises(ValueError):
        randomized_scene(source, 1, float("nan"))


def test_opening_is_not_cycle_success_and_return_allows_pressed_latch():
    task, sample = setup_cycle()
    for stage in ("raise", "above_drawer", "align", "descend", "lift"):
        task._enter(stage, sample)
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
        DrawerCycleConfig(seating_overtravel_m=.01)
