"""Per-backend momentum acceptance bounds and the no-accumulation guard.

The scenario module is loaded without Python MuJoCo (Basilisk's MuJoCo DLL and the
offline Python MuJoCo package must not share a process), the same way
tests/test_attitude_control.py loads it.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "model/SARM/platform/scenarios/scenario_sarm_grasp.py"


@pytest.fixture
def scenario(monkeypatch):
    monkeypatch.setitem(sys.modules, "mujoco", ModuleType("mujoco"))
    spec = importlib.util.spec_from_file_location("sarm_momentum_acceptance_test", SCENARIO)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def metrics(scenario, **overrides):
    """A fully passing metric set, with overrides applied."""
    base = dict(
        max_linear_momentum_error=0.0,
        max_com_angular_momentum_error=0.0,
        max_origin_angular_momentum_error=0.0,
        max_com_uniform_motion_error=0.0,
        post_motion_linear_momentum_drift=0.0,
        post_motion_origin_angular_momentum_drift=0.0,
        initial_mechanical_energy=0.0,
        final_mechanical_energy=0.0,
        max_mechanical_energy_change=0.0,
        first_contact_time=5.0,
        bilateral_contact_duration=1.0,
        withdrawal_bilateral_coverage=1.0,
        fixed_jaw_peak_normal_force=1.0,
        moving_jaw_peak_normal_force=1.0,
        target_withdrawal_distance=0.05,
        grasp_site_drift=0.0,
        final_relative_linear_speed=0.0,
        final_relative_angular_speed=0.0,
        invalid_contact_count=0,
        max_abs_actuator_command=(0.0,),
    )
    base.update(overrides)
    return scenario.GraspMetrics(**base)


def test_a_fully_passing_metric_set_reports_nothing(scenario):
    assert scenario.acceptance_failures(metrics(scenario), "basilisk") == []
    assert scenario.acceptance_failures(metrics(scenario), "local") == []


def test_unknown_backend_is_rejected(scenario):
    with pytest.raises(ValueError, match="no momentum thresholds"):
        scenario.acceptance_failures(metrics(scenario), "rk4")


@pytest.mark.parametrize(
    "field,local_value,basilisk_value",
    [
        ("max_linear_momentum_error", 1.9e-4, 1.9e-4),
        ("max_com_angular_momentum_error", 9.0e-5, 9.0e-5),
        ("max_origin_angular_momentum_error", 2.8e-5, 2.8e-5),
    ],
)
def test_measured_local_error_passes_local_and_fails_basilisk(
    scenario, field, local_value, basilisk_value
):
    """The measured local discretization error is the local bound's business.

    basilisk's strict bounds stay in force for the basilisk path: the same value
    must still fail there, so the reference criterion cannot silently loosen.
    """
    case = metrics(scenario, **{field: local_value})
    assert scenario.acceptance_failures(case, "local") == []
    assert scenario.acceptance_failures(case, "basilisk"), (
        f"{field}={basilisk_value} must still fail the basilisk reference bounds"
    )


def test_basilisk_thresholds_are_the_original_strict_values(scenario):
    bounds = scenario.MOMENTUM_THRESHOLDS["basilisk"]
    assert bounds["linear_kg_m_s"] == 5e-5
    assert bounds["com_angular_kg_m2_s"] == 1e-5
    assert bounds["origin_angular_kg_m2_s"] == 1e-5
    assert bounds["com_uniform_motion_m"] == 5e-5


def test_local_bounds_sit_above_the_measured_one_substep_error(scenario):
    """Bounds are regression limits: comfortably above measured, still tight."""
    bounds = scenario.MOMENTUM_THRESHOLDS["local"]
    measured = {
        "linear_kg_m_s": 1.87e-4,
        "com_angular_kg_m2_s": 8.98e-5,
        "origin_angular_kg_m2_s": 2.83e-5,
    }
    for key, value in measured.items():
        assert value < bounds[key] <= 5.0 * value, key


def test_accumulating_momentum_is_rejected_on_both_backends(scenario):
    """A post-motion drift is accumulation, not discretization error."""
    for backend in ("basilisk", "local"):
        for field in (
            "post_motion_linear_momentum_drift",
            "post_motion_origin_angular_momentum_drift",
        ):
            failures = scenario.acceptance_failures(metrics(scenario, **{field: 1e-3}), backend)
            assert any("accumulating" in message for message in failures), (backend, field)


def test_the_measured_post_motion_drift_passes_local(scenario):
    """The measured 1.1e-6 / 3e-7 residual must not trip the guard."""
    case = metrics(
        scenario,
        post_motion_linear_momentum_drift=1.117e-6,
        post_motion_origin_angular_momentum_drift=3e-7,
    )
    assert scenario.acceptance_failures(case, "local") == []
