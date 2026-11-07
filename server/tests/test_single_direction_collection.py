"""Independent-task dispatch and rejection of wrong-direction physical records."""
from copy import deepcopy
from dataclasses import asdict

import pytest

from collection.recording_validation import validate_drawer_recording, validate_threaded_recording
from collection.sequence import SequenceConfig, TaskConfig
from simulation.scripted_tasks.drawer_tasks import drawer_task_class
from simulation.scripted_tasks.threaded_tasks import threaded_task_class
from space_arm_platform.collection_jobs import CollectionCreate, TASKS
from test_threaded_collection_integration import rod_recording

NEW_TASKS = ("drawer_open", "drawer_close", "locking_rod_fold", "locking_rod_restore")


@pytest.mark.parametrize("name", NEW_TASKS)
def test_independent_tasks_can_be_selected_by_web_and_layout_sequence(name):
    request = CollectionCreate(task=name)
    assert TASKS[name].arguments == ("--task", name)
    assert request.linear_speed_m_s == (.1 if name.startswith("drawer_") else None)
    assert request.threaded_speed_profile == ("fast_collection" if name.startswith("locking_") else None)
    plan = SequenceConfig(layout_pool="pool.json", tasks=[TaskConfig(name=name)])
    assert [item.name for item in plan.tasks if item.enabled] == [name]
    # A saved config that omits tasks must preserve the original five-task batch.
    original = SequenceConfig(layout_pool="pool.json")
    assert len([item for item in original.tasks if item.enabled]) == 5
    assert not set(NEW_TASKS) & {item.name for item in original.tasks if item.enabled}


def drawer_recording(name):
    from simulation.scripted_tasks.drawer_close import DrawerCloseConfig
    from simulation.scripted_tasks.drawer_open import DrawerOpenConfig
    config = DrawerOpenConfig() if name == "drawer_open" else DrawerCloseConfig()
    controller = drawer_task_class(name)
    initial, target = (0., .08) if name == "drawer_open" else (.08, 0.)
    rows = []
    for stage in list(controller.stages[:-1])+["verify"]*105:
        released = stage in ("release", "retract", "verify")
        row = dict(name=name, task_version=controller.version, task_parameters=asdict(config),
                   stage=stage, result=None, drawer_initial_m=initial, drawer_target_m=target,
                   drawer_position_m=target if released else initial,
                   drawer_velocity_m_s=0., finger_drawer_contacts=[0, 0] if released else [1, 1],
                   drawer_open=name == "drawer_open", drawer_closed=name == "drawer_close",
                   open_verified=True, seated_before_release_verified=name == "drawer_close",
                   released_open_seconds=config.verify_hold_s, released_closed_seconds=config.verify_hold_s)
        rows.append(dict(sim_time_ns=str(len(rows)*20_000_000), scripted_task=row,
                         gripper_opening_normalized=1.))
    rows[-1]["scripted_task"]["result"] = "success"
    return rows, list(controller.stages)


@pytest.mark.parametrize("name", ["drawer_open", "drawer_close"])
def test_single_drawer_recording_requires_actual_direction_and_released_hold(name):
    rows, stages = drawer_recording(name)
    validate_drawer_recording(rows, stages, name)
    # Opening overlaps retreat: the physical-clear threshold starts the hold,
    # while the last frame must still reach the full opening. No extra dwell.
    clearing = deepcopy(rows)
    threshold = clearing[-1]["scripted_task"]["task_parameters"]["release_clearance_opening"]
    clearing[-10]["gripper_opening_normalized"] = threshold+.01
    validate_drawer_recording(clearing, stages, name)
    clearing[-10]["gripper_opening_normalized"] = threshold-.01
    with pytest.raises(AssertionError):
        validate_drawer_recording(clearing, stages, name)
    wrong_initial = deepcopy(rows)
    wrong_initial[0]["scripted_task"]["drawer_position_m"] = rows[-1]["scripted_task"]["drawer_target_m"]
    with pytest.raises(AssertionError):
        validate_drawer_recording(wrong_initial, stages, name)
    moving = deepcopy(rows)
    moving[-10]["scripted_task"]["drawer_velocity_m_s"] = .02
    with pytest.raises(AssertionError):
        validate_drawer_recording(moving, stages, name)
    touching = deepcopy(rows)
    touching[-10]["scripted_task"]["finger_drawer_contacts"] = [1, 0]
    with pytest.raises(AssertionError):
        validate_drawer_recording(touching, stages, name)
    wrong_name = "drawer_close" if name == "drawer_open" else "drawer_open"
    with pytest.raises((AssertionError, ValueError)):
        validate_drawer_recording(rows, stages, wrong_name)


def split_rod_recording(name):
    original, _ = rod_recording()
    by_stage = {obs["scripted_task"]["stage"]: obs for obs in original}
    controller = threaded_task_class(name)
    rows = []
    stages = list(controller.stages)
    for stage in stages[:-1]+["verify"]*55:
        obs = deepcopy(by_stage[stage])
        row = obs["scripted_task"]
        params = row["task_parameters"]
        row.update(task_version=controller.version, task_operation="fold" if name.endswith("fold") else "restore",
                   result=None, initial_folded_verified=name.endswith("restore"))
        if name.endswith("fold"):
            row.update(returned_verified=False, tightened_verified=False)
            if stage in ("release_down", "clear_down", "verify"):
                row.update(hinge_rad=params["folded_hinge_rad"], fastener_slide_m=0., fastener_spin_rad=0.,
                           locking_rod_clip_contacts=1, finger_part_contacts=[0, 0])
        else:
            row.update(loosen_verified=False, folded_verified=False)
            if stage in ("settle", "reapproach", "regrip"):
                row.update(hinge_rad=params["folded_hinge_rad"], fastener_slide_m=0., fastener_spin_rad=0.,
                           locking_rod_clip_contacts=1)
        obs["sim_time_ns"] = str(len(rows)*20_000_000)
        rows.append(obs)
    rows[-1]["scripted_task"]["result"] = "success"
    return rows, stages


@pytest.mark.parametrize("name", ["locking_rod_fold", "locking_rod_restore"])
def test_split_rod_validation_rejects_false_initial_or_final_state(name):
    rows, stages = split_rod_recording(name)
    validate_threaded_recording(rows, stages, name)
    invalid = deepcopy(rows)
    if name.endswith("fold"):
        invalid[-10]["scripted_task"]["locking_rod_clip_contacts"] = 0
    else:
        invalid[0]["scripted_task"]["hinge_rad"] = 0.
    with pytest.raises(AssertionError):
        validate_threaded_recording(invalid, stages, name)
    with pytest.raises(AssertionError):
        validate_threaded_recording(rows, list(threaded_task_class("locking_rod_cycle").stages), name)
    missing = deepcopy(rows)
    required = "loosen_verified" if name.endswith("fold") else "initial_folded_verified"
    missing[-1]["scripted_task"][required] = False
    with pytest.raises(AssertionError):
        validate_threaded_recording(missing, stages, name)


@pytest.mark.parametrize("name", ["locking_rod_fold", "locking_rod_restore"])
def test_split_rod_release_can_finish_opening_during_the_contact_free_hold(name):
    rows, stages = split_rod_recording(name)
    rows[-10]["gripper_opening_normalized"] = .415
    validate_threaded_recording(rows, stages, name)
    blocked = deepcopy(rows)
    blocked[-10]["scripted_task"]["finger_part_contacts"] = [1, 0]
    with pytest.raises(AssertionError):
        validate_threaded_recording(blocked, stages, name)
    narrow = deepcopy(rows)
    narrow[-10]["gripper_opening_normalized"] = .39
    with pytest.raises(AssertionError):
        validate_threaded_recording(narrow, stages, name)
    rows[-1]["gripper_opening_normalized"] = .415
    with pytest.raises(AssertionError):
        validate_threaded_recording(rows, stages, name)


def test_backend_catalog_imports_without_simulation_package(tmp_path):
    """An installed API must not require unbundled controller sources to start."""
    from pathlib import Path
    import subprocess
    import sys

    backend = Path(__file__).resolve().parents[1] / "backend"
    script = """
import importlib.abc
import sys
sys.path.insert(0, sys.argv[1])
class BlockSimulation(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'simulation' or fullname.startswith('simulation.'):
            raise ModuleNotFoundError('simulation is deliberately not installed')
sys.meta_path.insert(0, BlockSimulation())
from space_arm_platform.collection_jobs import CollectionCreate, TASKS
assert len(TASKS) == 11
assert 'capture_target_grasp' in TASKS
assert CollectionCreate(task='drawer_close').linear_speed_m_s == .1
assert CollectionCreate(task='locking_rod_restore').threaded_speed_profile == 'fast_collection'
"""
    subprocess.run([sys.executable, "-I", "-c", script, str(backend)], cwd=tmp_path,
                   check=True, capture_output=True, text=True)


@pytest.mark.parametrize("name", ["locking_rod_fold", "locking_rod_restore"])
def test_directional_rod_retains_stability_and_rejects_cycle_version(name):
    rows, stages = split_rod_recording(name)
    moving = deepcopy(rows)
    moving[-10]["scripted_task"]["part_velocity_body_m_s"] = [.02, 0., 0.]
    with pytest.raises(AssertionError):
        validate_threaded_recording(moving, stages, name)
    rows[-1]["scripted_task"]["task_version"] = "locking-rod-cycle/3-straight-approach"
    with pytest.raises(ValueError, match="Unsupported recorded task/version"):
        validate_threaded_recording(rows, stages, name)


def test_completion_defaults_remain_bound_to_directional_versions():
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
    from simulation.scripted_tasks.drawer_close import DrawerCloseConfig
    from simulation.scripted_tasks.drawer_open import DrawerOpenConfig
    assert DrawerCycleConfig().verify_hold_s == .3
    assert DrawerOpenConfig().verify_hold_s == DrawerCloseConfig().verify_hold_s == 2.
    assert threaded_task_class("locking_rod_cycle").config_class().verify_hold_s == .3
    for name in ("locking_rod_fold", "locking_rod_restore"):
        assert threaded_task_class(name).config_class().verify_hold_s == 1.
