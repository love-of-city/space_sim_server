"""Regression checks for evidence-based offline plug-motion recommendations."""

import json
from pathlib import Path

import pytest

from tools.benchmarks.optimize_plug_motion import _new_output, prepare_inputs, candidate_scene
from tools.benchmarks.plug_motion_metrics import measure_trace, rejection_reasons


def row(time, stage="insert", *, speed=0.0, reference=0.0, remaining=0.01):
    return dict(sim_time_ns=str(round(time * 1e9)), stage=stage,
                plug_velocity_body_m_s=[0.0, 0.0, speed],
                axial_reference_speed_m_s=reference, contact_remaining_m=remaining,
                insertion_speed_scale=1.0 if reference else 0.0,
                wrist_wrench=[1.0, 0.0, 0.0, 0.0, 0.1, 0.0])


def test_measures_actual_stall_and_does_not_confuse_zero_reference_with_stall():
    trace = [row(0, "extract", speed=0.01, reference=0.01),
             row(.1, speed=0.0, reference=.01),
             row(.2, speed=0.0, reference=.01),
             row(.3, speed=.01, reference=0.0),
             row(.4, speed=.01, reference=0.0),
             row(.5, remaining=0.0), row(.6, remaining=0.0),
             row(.7, "release"), row(1.7, "verify")]
    result = measure_trace(trace, {"result": "success"})
    insertion = result["stages"]["insert"]
    assert result["measurement_complete"]
    assert insertion["max_stagnant_seconds"] == pytest.approx(.2)
    assert insertion["stalled_with_reference_seconds"] == pytest.approx(.2)
    assert insertion["measured_motion_with_zero_reference_seconds"] == pytest.approx(.2)
    assert insertion["zero_reference_samples"] == 2
    assert insertion["zero_target_samples"] == 2
    assert insertion["seated_confirmation_seconds"] == pytest.approx(.2)
    assert result["release_seconds"] == pytest.approx(1.0)


def test_failure_or_missing_actual_velocity_cannot_be_recommended():
    trace = [row(0, "extract"), row(.1), row(.2, "release")]
    del trace[1]["plug_velocity_body_m_s"]
    result = measure_trace(trace, {"result": "failed", "reason": "dropped"})
    reasons = rejection_reasons(result)
    assert "task_failed" in reasons
    assert "incomplete_contact_measurement" in reasons


def test_load_regression_rejects_fast_success():
    metrics = successful_metrics(insert_force=12.0, summary_peak=12.0)
    baseline = successful_metrics(insert_force=10.0, summary_peak=10.0)
    assert rejection_reasons(metrics, baseline) == ["max_wrist_force_n_regression", "insert_max_wrist_force_n_regression"]
    assert not rejection_reasons(metrics, baseline, load_multiplier=1.3)


def successful_metrics(*, insert_force=10., summary_peak=100., result="success"):
    trace = [row(0, "extract", speed=.01), row(.1, "insert", speed=-.01), row(.2, "release")]
    trace[0]["wrist_wrench"] = {"valid": True, "force_n": [3., 4., 0.], "torque_nm": [0., .3, .4]}
    trace[1]["wrist_wrench"] = {"valid": True, "force_n": [insert_force, 0., 0.], "torque_nm": [0., 0., .1]}
    return measure_trace(trace, {"result": result, "max_wrist_force_n": summary_peak})


def test_real_sensor_dict_stage_peaks_cannot_be_masked_by_whole_task_summary():
    baseline = successful_metrics(insert_force=10., summary_peak=100.)
    candidate = successful_metrics(insert_force=12., summary_peak=100.)
    assert baseline["stages"]["extract"]["max_wrist_force_n"] == pytest.approx(5.)
    assert baseline["stages"]["extract"]["max_wrist_torque_nm"] == pytest.approx(.5)
    assert candidate["max_wrist_force_n"] == baseline["max_wrist_force_n"] == 100.
    assert rejection_reasons(candidate, baseline) == ["insert_max_wrist_force_n_regression"]


def test_failed_baseline_cannot_approve_a_successful_candidate():
    baseline = successful_metrics(result="failed")
    candidate = successful_metrics()
    assert rejection_reasons(candidate, baseline) == ["baseline_failed"]


@pytest.mark.parametrize("wrench", [
    None, {}, {"valid": False, "force_n": [0., 0., 0.], "torque_nm": [0., 0., 0.]},
    {"valid": True, "force_n": [float("nan"), 0., 0.], "torque_nm": [0., 0., 0.]},
    [1., 0., 0., 0., float("inf"), 0.],
])
def test_missing_or_invalid_contact_wrench_cannot_use_summary_as_substitute(wrench):
    trace = [row(0, "extract", speed=.01), row(.1, "insert", speed=-.01), row(.2, "release")]
    trace[1]["wrist_wrench"] = wrench
    metrics = measure_trace(trace, {"result": "success", "max_wrist_force_n": 10., "max_wrist_torque_nm": 1.})
    assert not metrics["load_measurement_complete"]
    assert "incomplete_contact_load_measurement" in rejection_reasons(metrics)


def test_explicit_target_and_remaining_travel_take_precedence_over_legacy_fields():
    trace = [row(0, "extract", speed=.01), row(.1, speed=-.01, reference=.01),
             row(.2, remaining=0.), row(.3, "release")]
    trace[1]["axial_target_speed_m_s"] = 0.
    trace[1]["insertion_speed_scale"] = 1.
    trace[2]["seat_gap_m"] = .1
    metrics = measure_trace(trace, {"result": "success"})
    insertion = metrics["stages"]["insert"]
    assert insertion["zero_target_samples"] == 1
    assert insertion["zero_reference_samples"] == 0
    assert insertion["seated_confirmation_seconds"] == pytest.approx(.1)
    assert insertion["stagnant_seconds"] == 0.
    assert metrics["fallback_remaining_samples"] == 0


def test_case_inputs_preserved_and_unknown_parameters_rejected_before_output(tmp_path):
    scene = tmp_path / "scene.json"
    scene.write_text('{"collection_config":{"plug_motion_parameters":{"insert_creep_delay_s":0.2}}}')
    model = tmp_path / "model.xml"
    model.write_text("<mujoco/>")
    cases = tmp_path / "cases.json"
    cases.write_text(json.dumps([dict(name="round", task="cylindrical_plug_insert", scene="scene.json", model="model.xml")]))
    candidates = tmp_path / "candidates.json"
    candidates.write_text(json.dumps([dict(name="short-hold", parameters={"insert_creep_delay_s": .1})]))
    original = {p: p.read_bytes() for p in (scene, model, cases, candidates)}
    parsed_cases, parsed_candidates = prepare_inputs(cases, candidates)
    assert parsed_candidates[0] == dict(name="baseline", parameters={})
    assert parsed_cases[0]["scene_data"]["collection_config"]["plug_motion_parameters"]["insert_creep_delay_s"] == .2
    assert all(p.read_bytes() == contents for p, contents in original.items())
    candidates.write_text(json.dumps([dict(name="bad", parameters={"disable_collision": True})]))
    with pytest.raises(ValueError, match="Unsupported plug motion"):
        prepare_inputs(cases, candidates)


def test_existing_or_stopped_collection_output_refused(tmp_path):
    with pytest.raises(FileExistsError):
        _new_output(tmp_path)
    (tmp_path / "cancel.request").touch()
    with pytest.raises(ValueError, match="collection batch"):
        _new_output(tmp_path / "diagnostic")
    assert not (tmp_path / "diagnostic").exists()


def test_candidate_from_frozen_config_preserves_source_and_records_derivation():
    from simulation.scripted_tasks.plug_motion_config import config_snapshot, plug_motion_config
    task = "aviation_plug_insert"
    config = plug_motion_config(task, insert_creep_delay_s=.2, release_open_rad=.6)
    snapshot = config_snapshot(config, task)
    source = {"collection_config": {"plug_config_snapshot": snapshot}}
    original = json.dumps(source, sort_keys=True)
    derived = candidate_scene(task, source, {"insert_creep_delay_s": .1})
    assert json.dumps(source, sort_keys=True) == original
    result = plug_motion_config(task, derived)
    assert result.insert_creep_delay_s == .1
    assert result.release_open_rad == .6
    assert derived["collection_config"]["plug_config_derivation"] == {
        "source_resolved_config_sha256": snapshot["sha256"],
        "source_had_snapshot": True,
        "parameter_changes": {"insert_creep_delay_s": .1},
    }
    assert derived["collection_config"]["plug_config_snapshot"]["sha256"] != snapshot["sha256"]


@pytest.mark.parametrize("times", [[0, 0], [1, .5]])
def test_non_increasing_trace_times_rejected(times):
    with pytest.raises(ValueError, match="strictly increasing"):
        measure_trace([row(t) for t in times], {"result": "success"})
