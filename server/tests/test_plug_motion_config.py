"""Saved tuning inputs must not alter physical or final acceptance settings."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from simulation.scripted_tasks.plug_insert import PlugInsertTask
from simulation.scripted_tasks.plug_motion_config import PlugInsertConfig
from simulation.scripted_tasks.plug_motion_config import (
    MOTION_PARAMETERS, MOTION_PARAMETER_MAXIMUMS, plug_motion_config,
)


def test_saved_motion_overrides_are_resolved_without_mutating_input():
    scene = {"collection_config": {"plug_motion_parameters": {"insert_speed_m_s": .012}}}
    cfg = plug_motion_config("cylindrical_plug_insert", scene)
    assert cfg.insert_speed_m_s == .012
    assert cfg.radial_tolerance_m == .0004
    assert scene["collection_config"]["plug_motion_parameters"] == {"insert_speed_m_s": .012}
    assert plug_motion_config("cylindrical_plug_insert", scene, insert_speed_m_s=.02).insert_speed_m_s == .02


@pytest.mark.parametrize("parameters", [
    {"radial_tolerance_m": .01}, {"seat_tolerance_m": .01},
    {"plug_body": "guide_plug"}, {"force_limit_n": 1000}, [],
    {"insert_speed_m_s": float("nan")}, {"insert_speed_m_s": True},
])
def test_invalid_or_non_motion_overrides_are_rejected(parameters):
    with pytest.raises(ValueError):
        plug_motion_config("cylindrical_plug_insert", {"collection_config": {"plug_motion_parameters": parameters}})


@pytest.mark.parametrize("name", sorted(MOTION_PARAMETERS))
def test_each_tunable_parameter_has_a_finite_upper_bound(name):
    maximum = MOTION_PARAMETER_MAXIMUMS[name]
    scene = {"collection_config": {"plug_motion_parameters": {name: maximum + .001}}}
    with pytest.raises(ValueError, match=name):
        plug_motion_config("cylindrical_plug_insert", scene)


def test_explicit_extraction_speed_cannot_bypass_saved_configuration_bound():
    with pytest.raises(ValueError, match="extraction_speed_m_s"):
        plug_motion_config("cylindrical_plug_insert", extraction_speed_m_s=1.)
    cfg = plug_motion_config("cylindrical_plug_insert", extraction_speed_m_s=.025)
    assert cfg.extraction_speed_m_s == .025


MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


@pytest.mark.parametrize("name,lateral,velocity", [
    ("aviation_plug_insert", 4., 8.),
    ("cylindrical_plug_insert", 3., 6.),
    ("guide_plug_insert", 3., 8.),
])
def test_profile_extraction_gains_reach_default_and_runtime_bound_feedback(name, lateral, velocity):
    cfg = plug_motion_config(name)
    task = PlugInsertTask(MODEL, cfg)
    assert cfg.extraction_lateral_gain_s_inv == lateral
    assert cfg.extraction_velocity_gain == velocity
    for report in (task._contact_feedback.configuration(),
                   task.configure_contact_feedback(np.full(6, 500.), None)):
        assert report["extraction_lateral_gain_s_inv"] == lateral
        assert report["extraction_velocity_gain"] == velocity


@pytest.mark.parametrize("name", ["aviation_plug_insert", "cylindrical_plug_insert", "guide_plug_insert"])
def test_saved_extraction_gains_override_profile_defaults_and_survive_runtime_binding(name):
    parameters = {"extraction_lateral_gain_s_inv": 2.5, "extraction_velocity_gain": 4.5}
    scene = {"collection_config": {"plug_motion_parameters": parameters.copy()}}
    cfg = plug_motion_config(name, scene)
    task = PlugInsertTask(MODEL, cfg)
    retention = SimpleNamespace(sliding_friction_n=20., friction_travel_m=.008)
    report = task.configure_contact_feedback(np.full(6, 500.), retention)
    for key, value in parameters.items():
        assert getattr(cfg, key) == value
        assert report[key] == value
        assert getattr(task._contact_feedback, key) == value
    assert report["friction_force_n"] == 20.
    assert scene["collection_config"]["plug_motion_parameters"] == parameters
    explicit = plug_motion_config(name, scene, extraction_lateral_gain_s_inv=5., extraction_velocity_gain=7.)
    assert explicit.extraction_lateral_gain_s_inv == 5.
    assert explicit.extraction_velocity_gain == 7.


@pytest.mark.parametrize("name", ["extraction_lateral_gain_s_inv", "extraction_velocity_gain"])
@pytest.mark.parametrize("value", [0., -1., float("nan"), float("inf"), True, "2"])
def test_saved_and_explicit_extraction_gains_reject_invalid_values(name, value):
    with pytest.raises(ValueError, match=name):
        plug_motion_config("cylindrical_plug_insert", {"collection_config": {
            "plug_motion_parameters": {name: value}}})
    with pytest.raises(ValueError, match=name):
        plug_motion_config("cylindrical_plug_insert", **{name: value})


@pytest.mark.parametrize("name,maximum", [("extraction_lateral_gain_s_inv", 6.), ("extraction_velocity_gain", 8.)])
def test_extraction_gains_accept_upper_bound_and_direct_config_cannot_exceed_it(name, maximum):
    cfg = plug_motion_config("cylindrical_plug_insert", **{name: maximum})
    assert getattr(cfg, name) == maximum
    with pytest.raises(ValueError, match=name):
        PlugInsertConfig(**{name: maximum+.001})


@pytest.mark.parametrize("name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_frozen_configuration_roundtrips_without_overlaying_defaults(name):
    import json
    from dataclasses import asdict
    from simulation.scripted_tasks.plug_motion_config import CONFIG_SCHEMA, config_snapshot

    configured = plug_motion_config(name, insert_speed_m_s=.012)
    saved = json.loads(json.dumps(config_snapshot(configured, name)))
    restored = plug_motion_config(name, {"collection_config": {"plug_config_snapshot": saved}})
    assert asdict(restored) == asdict(configured)
    assert saved["schema"] == CONFIG_SCHEMA
    assert "hold_s" not in saved["values"]
    with pytest.raises(ValueError, match="conflicts"):
        plug_motion_config(name, {"collection_config": {"plug_config_snapshot": saved}},
                           insert_speed_m_s=.025)


def test_frozen_configuration_rejects_tampering_and_wrong_task():
    from copy import deepcopy
    from simulation.scripted_tasks.plug_motion_config import config_snapshot

    snapshot = config_snapshot(plug_motion_config("aviation_plug_insert"), "aviation_plug_insert")
    altered = deepcopy(snapshot)
    altered["values"]["insert_speed_m_s"] = .02
    with pytest.raises(ValueError, match="hash"):
        plug_motion_config("aviation_plug_insert", {"collection_config": {"plug_config_snapshot": altered}})
    with pytest.raises(ValueError, match="match"):
        plug_motion_config("guide_plug_insert", {"collection_config": {"plug_config_snapshot": snapshot}})


def test_initialization_freezes_the_same_motion_configuration_as_runtime():
    from simulation.scripted_tasks.task_initialization import randomized_scene
    from simulation.scripted_tasks.plug_motion_config import config_snapshot

    source = {"randomization": {"arm_joint_position_rad": [0.] * 8},
              "collection_config": {"plug_motion_parameters": {"insert_speed_m_s": .012}}}
    scene = randomized_scene(source, 42, task_name="guide_plug_insert")
    config = plug_motion_config("guide_plug_insert", scene)
    assert config.insert_speed_m_s == .012
    assert scene["collection_config"]["plug_config_snapshot"] == config_snapshot(config, "guide_plug_insert")
