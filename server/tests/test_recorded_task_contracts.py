"""Saved versions retain their own stage and parameter meanings."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import os
from pathlib import Path
import subprocess
import sys

import pytest

from collection.recorded_task_contracts import (
    recorded_contract, recorded_stages, validate_recorded_stages,
    validate_recorded_configuration,
)


@pytest.mark.parametrize("task,version,present,absent", [
    ("drawer_cycle", None, "align", "approach"),
    ("drawer_cycle", "drawer-cycle/4", "above_drawer", "align"),
    ("drawer_cycle", "drawer-cycle/7-closed-completion", "return", "seat"),
    ("plug_insert", None, "open", "extract"),
    ("guide_plug_insert", "plug-insert/3-latch-grasp", "raise", "open"),
    ("aviation_plug_insert", "plug-insert/4-coordinated-approach", "descend", "raise"),
    ("cylindrical_plug_insert", "plug-insert/7-continuous-carry", "align_hole", "descend"),
    ("cylindrical_plug_insert", "plug-insert/19-axis-contact-continuity", "extract", "seat"),
    ("locking_rod_fold", "locking-rod-fold/1-straight-approach", "lower", "tighten"),
    ("locking_rod_restore", "locking-rod-restore/1-straight-approach", "tighten", "loosen"),
])
def test_historical_stage_boundaries(task, version, present, absent):
    stages = recorded_stages(task, version)
    assert present in stages and absent not in stages
    with pytest.raises(ValueError, match="Unsupported recorded task/version"):
        recorded_stages(task, "future/unknown")


def test_contracts_preserve_completion_rules_and_are_immutable():
    historical = recorded_contract("plug_insert", "plug-insert/14-straight-approach-gentle-release")
    current = recorded_contract("plug_insert", "plug-insert/19-axis-contact-continuity")
    assert historical.released_velocity_required and not current.released_velocity_required
    assert not historical.latch_grasp_required and current.latch_grasp_required
    with pytest.raises(FrozenInstanceError):
        current.released_velocity_required = True


def test_bolt_contract_requires_every_regrasp_operation():
    stages = ["settle", "approach", "grip", "unscrew", "regrasp_release",
              "regrasp_rewind", "regrasp_grip", "unscrew", "carry", "screw_in",
              "regrasp_release", "regrasp_rewind", "regrasp_grip", "screw_in",
              "release", "retract", "verify"]
    version = "aviation-bolt/4-straight-approach"
    assert validate_recorded_stages("aviation_bolt_insert", stages, version)
    assert not validate_recorded_stages("aviation_bolt_insert", [s for s in stages if s != "regrasp_rewind"], version)


def saved_configuration():
    from simulation.scripted_tasks.plug_motion_config import config_snapshot, plug_motion_config
    task = "cylindrical_plug_insert"
    snapshot = config_snapshot(plug_motion_config(task), task)
    scene = {"collection_config": {"plug_config_snapshot": snapshot}}
    row = dict(task_parameters_schema=snapshot["schema"], task_parameters_sha256=snapshot["sha256"],
               task_parameters=deepcopy(snapshot["values"]), task_version=snapshot["task_version"])
    return task, scene, row


@pytest.mark.parametrize("corruption", ["missing_snapshot", "values", "hash", "schema", "version", "incomplete"])
def test_declared_configuration_must_match_the_saved_snapshot(corruption):
    task, scene, row = saved_configuration()
    validate_recorded_configuration(scene, task, [row])
    if corruption == "missing_snapshot":
        scene.clear()
    elif corruption == "values":
        row["task_parameters"]["force_limit_n"] = 12.
    elif corruption == "hash":
        scene["collection_config"]["plug_config_snapshot"]["sha256"] = "wrong"
    elif corruption == "schema":
        row["task_parameters_schema"] = "plug-motion-config/unknown"
    elif corruption == "version":
        row["task_version"] = "plug-insert/2"
    else:
        del scene["collection_config"]["plug_config_snapshot"]["values"]["force_limit_n"]
    with pytest.raises(ValueError):
        validate_recorded_configuration(scene, task, [row])


def test_legacy_flat_parameters_are_not_rewritten_or_reinterpreted():
    row = {"task_version": "plug-insert/2", "task_parameters": {"hold_s": 2.}}
    original = deepcopy(row)
    validate_recorded_configuration({}, "plug_insert", [row])
    assert row == original


def test_recording_reader_import_does_not_load_controllers_or_native_runtime(tmp_path):
    server = Path(__file__).resolve().parents[1]
    code = """import sys
from collection.recording_validation import validate_recording
from space_arm_platform.task_catalog import TASK_CHOICES
assert len(TASK_CHOICES) == 10
for name in ('simulation.scripted_tasks.plug_insert', 'simulation.scripted_tasks.drawer_cycle',
             'simulation.local_mujoco_stepper', 'Basilisk', 'torch'):
    assert name not in sys.modules, name
"""
    env = {**os.environ, "PYTHONPATH": os.pathsep.join((str(server), str(server / "backend")))}
    subprocess.run([sys.executable, "-c", code], cwd=tmp_path, env=env, check=True)
