from collection.recorded_task_contracts import recorded_stages, validate_recorded_stages
"""The two rod skills have separate physical starts and completion evidence."""
from dataclasses import replace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.model_preparation.threaded_task_initialization import (
    FOLDED_HINGE_RAD, LOCKED_TRAVEL_M, apply_locking_task_initial_state,
)
from simulation.scripted_tasks.locking_rod import (
    LockingRodTask, LockingRodFoldTask, LockingRodRestoreTask,
)
from simulation.scripted_tasks.threaded_tasks import create_threaded_task
from simulation.scripted_tasks.threaded_motion import ThreadedSample
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

SOURCE = Path(__file__).resolve().parents[1]/"model/SARM/platform/sarm_20261012.xml"


def make_model(tmp_path, name, *, yaw=0.):
    root = ET.parse(SOURCE).getroot()
    base = root.find('.//body[@name="task_box_base"]')
    base.set("quat", f"{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}")
    apply_locking_task_initial_state(root, name)
    path = tmp_path/f"{name}.xml"
    ET.ElementTree(root).write(path)
    return path


def sample_for(task, *, hinge=None, slide=None, spin=None, clip=True, fingers=(0, 0), time=0.):
    hinge = task.hinge_ref if hinge is None else hinge
    slide = task.slide_ref if slide is None else slide
    spin = task.spin_ref if spin is None else spin
    p, r = task.mechanism_pose(hinge, slide, spin)
    return ThreadedSample(
        time, np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), .45, -.45],
        np.zeros(8), p, r, np.zeros(3), np.zeros(3), np.zeros(6), True, fingers,
        {"locking_rod_hinge": hinge, "fastener_slide": slide, "fastener_spin": spin,
         "locking_rod_hinge_velocity": 0., "fastener_slide_velocity": 0., "fastener_spin_velocity": 0.,
         "locking_rod_clip_contacts": float(clip), "fastener_clip_contacts": 0.,
         "fastener_hook_contacts": 0., "fastener_hook_normal_force_n": 0.},
    )


@pytest.mark.parametrize("yaw", [0., .67])
def test_restore_rest_transform_is_same_mechanism_at_every_coordinate(tmp_path, yaw):
    locked = LockingRodTask(make_model(tmp_path, "locking_rod_cycle", yaw=yaw))
    folded = LockingRodRestoreTask(make_model(tmp_path, "locking_rod_restore", yaw=yaw))
    assert folded.hinge_ref == FOLDED_HINGE_RAD
    assert folded.slide_ref == folded.spin_ref == 0.
    assert folded.locked_slide == LOCKED_TRAVEL_M
    for hinge in [FOLDED_HINGE_RAD, -.7, 0.]:
        for slide in [0., .002, LOCKED_TRAVEL_M]:
            spin = slide*2*math.pi/locked.lead_m
            for actual, expected in zip(folded.mechanism_pose(hinge, slide, spin),
                                        locked.mechanism_pose(hinge, slide, spin), strict=True):
                np.testing.assert_allclose(actual, expected, atol=1e-12)


def test_restore_model_is_idempotent_and_can_be_returned_to_upright(tmp_path):
    path = make_model(tmp_path, "locking_rod_restore")
    root = ET.parse(path).getroot()
    before = ET.tostring(root)
    result = apply_locking_task_initial_state(root, "locking_rod_restore")
    assert ET.tostring(root) == before
    assert result["runtime_state_writes"] is False
    apply_locking_task_initial_state(root, "locking_rod_fold")
    assert float(root.find('.//joint[@name="locking_rod_hinge"]').get('ref')) == 0.
    assert float(root.find('.//joint[@name="fastener_slide"]').get('ref')) == LOCKED_TRAVEL_M
    assert root.find('./custom/numeric[@name="scripted_locking_folded_initial_state"]') is None
    second = ET.tostring(root)
    apply_locking_task_initial_state(root, "locking_rod_fold")
    assert ET.tostring(root) == second


def test_restore_start_requires_real_clip_and_loose_fastener(tmp_path):
    task = LockingRodRestoreTask(make_model(tmp_path, "locking_rod_restore"))
    assert task._initially_folded(sample_for(task))
    assert not task._initially_folded(sample_for(task, clip=False))
    assert not task._initially_folded(sample_for(task, hinge=0.))
    assert not task._initially_folded(sample_for(task, slide=.002, spin=.002*2*math.pi/task.lead_m))
    assert not task._initially_locked(sample_for(task))


def test_restore_settle_approaches_folded_support_grasp_without_replaying_fold(tmp_path, monkeypatch):
    task = LockingRodRestoreTask(make_model(tmp_path, "locking_rod_restore"))
    task.step(sample_for(task))
    entered = []
    monkeypatch.setattr(task, "_enter", lambda stage, sample: entered.append(stage))
    task.step(sample_for(task, time=.30))
    assert entered == ["reapproach"]
    assert task.initial_folded_verified
    assert not task.loosen_verified and not task.folded_verified


@pytest.mark.parametrize("name,kind", [
    ("locking_rod_fold", LockingRodFoldTask), ("locking_rod_restore", LockingRodRestoreTask),
])
def test_registry_and_stage_validation_are_direction_specific(tmp_path, name, kind):
    task = create_threaded_task(name, make_model(tmp_path, name), speed_profile="fast_collection")
    assert isinstance(task, kind) and task.config.linear_speed_m_s == .08
    assert validate_recorded_stages(kind.name, [s for s in kind.stages for _ in range(2)], kind.version)
    assert not validate_recorded_stages(kind.name, LockingRodTask.stages, kind.version)
    assert not validate_recorded_stages(kind.name, kind.stages[:-1], kind.version)
    with pytest.raises(ValueError, match="Unsupported"):
        recorded_stages(kind.name, LockingRodTask.version)


def test_fold_terminates_only_with_loosen_lower_release_and_clip_retention(tmp_path):
    task = LockingRodFoldTask(make_model(tmp_path, "locking_rod_fold"))
    down = sample_for(task, hinge=FOLDED_HINGE_RAD, slide=0., spin=0.)
    task.stage, task._entered = "verify", 0.
    for time in np.arange(0., 1.2, .05):
        task.step(replace(down, time_s=float(time)))
    assert task.result is None
    task.loosen_verified = task.folded_verified = True
    for time in np.arange(1.2, 2.4, .05):
        task.step(replace(down, time_s=float(time)))
    assert task.result == "success"
    assert not task.returned_verified and not task.tightened_verified
    assert not task._terminal_evidence(sample_for(task, hinge=FOLDED_HINGE_RAD, slide=0., spin=0., clip=False))


def test_fold_clear_down_hands_off_to_verification(tmp_path, monkeypatch):
    task = LockingRodFoldTask(make_model(tmp_path, "locking_rod_fold"))
    down = sample_for(task, hinge=FOLDED_HINGE_RAD, slide=0., spin=0., time=.005)
    task.stage, task._entered, task._last_time = "clear_down", 0., 0.
    monkeypatch.setattr(task, "_path_step", lambda *args: True)
    monkeypatch.setattr(task, "_advance_opening", lambda *args: None)
    task.step(down)
    assert task.stage == "verify"


def test_restore_cannot_finish_without_actual_tightening_history(tmp_path):
    task = LockingRodRestoreTask(make_model(tmp_path, "locking_rod_restore"))
    up = sample_for(task, hinge=0., slide=task.locked_slide, spin=task.locked_spin, clip=False)
    task.initial_folded_verified = task.returned_verified = task.tightened_verified = True
    task._tighten_seated_coordinates = (task.locked_slide, task.locked_spin)
    assert not task._terminal_evidence(up)
    task._tighten_peak_hook_force_n = 5.
    assert task._terminal_evidence(up)
    assert not task.loosen_verified and not task.folded_verified
    task.stage, task._entered = "verify", 0.
    for time in np.arange(0., 1.2, .05):
        task.step(replace(up, time_s=float(time)))
    assert task.result == "success"


def test_restore_tightening_targets_locked_seat_instead_of_loose_start(tmp_path, monkeypatch):
    task = LockingRodRestoreTask(make_model(tmp_path, "locking_rod_restore"))
    up = sample_for(task, hinge=0., slide=0., spin=0., clip=False, time=.005)
    task.stage, task._entered, task._last_time = "tighten", 0., 0.
    task._initial_spin = 0.
    goals = []
    monkeypatch.setattr(task, "_rotate_fastener", lambda sample, dt, goal: goals.append(goal))
    task.step(up)
    assert goals == [pytest.approx(LOCKED_TRAVEL_M*2*math.pi/task.lead_m+.12)]


@pytest.mark.parametrize("name", ["locking_rod_fold", "locking_rod_restore"])
def test_split_preflight_contains_only_executed_physical_operations(tmp_path, monkeypatch, name):
    monkeypatch.setattr(LockingRodTask, "_contact_opening", lambda *args: .33)
    task = create_threaded_task(name, make_model(tmp_path, name), speed_profile="fast_collection")
    initial = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.]
    paths = task.preflight_paths(initial)
    assert [entry[0] for entry in paths] == list(task.stages[1:-1])
    assert task.stage == "pending"
    for _, references, allowed, overrides in paths:
        assert references.shape == (161, 8) and allowed == ("fastener",)
        assert np.isfinite(references).all()
        np.testing.assert_allclose(overrides["fastener_spin"], overrides["fastener_slide"]*2*math.pi/task.lead_m)
    if name == "locking_rod_fold":
        assert paths[-1][3]["locking_rod_hinge"][-1] == FOLDED_HINGE_RAD
    else:
        assert paths[0][3]["locking_rod_hinge"][0] == FOLDED_HINGE_RAD
        assert paths[-1][3]["fastener_slide"][-1] == LOCKED_TRAVEL_M
