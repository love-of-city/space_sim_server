"""Guard task semantics, observation boundaries and adversarial grading inputs."""
from copy import deepcopy
import json
import math

import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import (
    ForceAction, NavigationObservation, SPEC_DIRECTORY, WaypointTask, load_task,
)
from space_arm_platform.benchmark_scoring import WaypointScorer, relative_truth


@pytest.fixture
def task():
    document = load_task(SPEC_DIRECTORY / "d32-cooperative-gnss-v1.json").model_dump(mode="json")
    document["duration_s"] = 1.0
    document["objective"]["dwell_s"] = 0.1
    return WaypointTask.model_validate(document)


def sample(tick, position=(-150.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0), force=(0.0, 0.0, 0.0)):
    # Equatorial orbit at x=7e6: T is +y, N is +z, R is +x.
    radius, speed = 7e6, 7500.0
    dx, dy, dz = position[2], position[0], position[1]
    vx, vy, vz = velocity[2], velocity[0], velocity[1]
    return {"time_ns": tick * 20_000_000, "target_com_j2000": [radius, 0., 0., 0., speed, 0.],
            "chaser_com_j2000": [radius + dx, dy, dz, vx - speed / radius * dy, speed + vy + speed / radius * dx, vz],
            "applied_motor_force_body_n": list(force), "external_state_writes": 0}


@pytest.mark.parametrize("mutation", [
    {"controlled_spacecraft": 2}, {"vision_mode": "V2"}, {"source_id": "D26"},
    {"attitude_control": "truth_hold"}, {"duration_s": True}, {"duration_s": 0.11},
    {"duration_s": 0.0}, {"duration_s": float("nan")},
    {"controlled_spacecraft": True}, {"solar_gravity": 1}, {"drag_enabled": "true"},
])
def test_rejects_contract_misrepresentation(task, mutation):
    document = task.model_dump(mode="json")
    document.update(mutation)
    with pytest.raises(ValidationError):
        WaypointTask.model_validate(document)


def test_initial_envelope_and_success_region_are_safe(task):
    document = task.model_dump(mode="json")
    document["initial"]["position_half_width_m"] = 100.0
    with pytest.raises(ValidationError, match="every bounded initial"):
        WaypointTask.model_validate(document)
    document = task.model_dump(mode="json")
    document["objective"]["position_tnr_m"] = [-100.0, 0.0, 0.0]
    with pytest.raises(ValidationError, match="complete success region"):
        WaypointTask.model_validate(document)


def test_split_overlap_and_unregistered_seeds_rejected(task):
    document = task.model_dump(mode="json")
    document["seeds"]["train"].append(201)
    with pytest.raises(ValidationError, match="unique"):
        WaypointTask.model_validate(document)
    with pytest.raises(ValueError, match="outside"):
        task.seeds.split_for(999)
    assert task.seeds.split_for(201) == "test"


def test_observation_has_no_truth_or_internal_state_channel():
    observation = NavigationObservation(episode_id="episode", sequence=0, sample_time_ns=0, delivery_time_ns=0,
                                        target_estimate_j2000=(7e6, 0., 0., 0., 7500., 0.),
                                        chaser_estimate_j2000=(7e6, -150., 0., 0., 7500., 0.))
    document = observation.model_dump(mode="json")
    for forbidden in ("truth", "scene", "score", "physical_state", "target_attitude"):
        candidate = dict(document, **{forbidden: {}})
        with pytest.raises(ValidationError):
            NavigationObservation.model_validate(candidate)
    action = ForceAction(episode_id="episode", observation_sequence=0, force_j2000_n=(1., 0., 0.))
    action.validate_for(observation, 25.)
    for change in ({"episode_id": "other"}, {"observation_sequence": 1}, {"force_j2000_n": (26., 0., 0.)}):
        with pytest.raises(ValueError):
            ForceAction.model_validate({**action.model_dump(), **change}).validate_for(observation, 25.)


def test_tnr_velocity_removes_rotating_frame_motion():
    from space_arm_platform.benchmark_contract import FlightTruthSample
    position, velocity = relative_truth(FlightTruthSample.model_validate(sample(0, (-150., 12., 5.))))
    assert position == pytest.approx((-150., 12., 5.))
    assert velocity == pytest.approx((0., 0., 0.), abs=1e-10)


def test_copied_model_cannot_bypass_action_validation():
    observation = NavigationObservation(episode_id="episode", sequence=0, sample_time_ns=0, delivery_time_ns=0,
                                        target_estimate_j2000=(7e6, 0., 0., 0., 7500., 0.),
                                        chaser_estimate_j2000=(7e6, -150., 0., 0., 7500., 0.))
    action = ForceAction(episode_id="episode", observation_sequence=0, force_j2000_n=(0., 0., 0.))
    for invalid in ((math.nan, 0., 0.), (True, 0., 0.), (1., 2.), ("1", 0., 0.)):
        copied = action.model_copy(update={"force_j2000_n": invalid})
        with pytest.raises(ValidationError):
            copied.validate_for(observation, 25.)


def test_typed_evidence_is_revalidated_before_grading(task):
    from space_arm_platform.benchmark_contract import FlightTruthSample
    copied = FlightTruthSample.model_validate(sample(0)).model_copy(update={"chaser_com_j2000": (math.nan,) * 6})
    scorer = WaypointScorer(task, 100.)
    with pytest.raises(ValueError, match="invalid benchmark physical evidence"):
        scorer.add(copied)
    assert scorer.result()["outcome"] == "invalid_evidence"


def test_dwell_requires_full_continuous_duration(task):
    scorer = WaypointScorer(task, 100.)
    for tick in range(5):
        scorer.add(sample(tick))
    assert scorer.result()["outcome"] == "incomplete"
    scorer.add(sample(5))
    result = scorer.result()
    assert result["success"]
    assert result["completion_time_s"] == pytest.approx(0.1)
    assert result["fuel_kg"] is None and result["energy_j"] is None


def test_leaving_region_resets_dwell(task):
    scorer = WaypointScorer(task, 100.)
    for tick in range(5):
        scorer.add(sample(tick))
    scorer.add(sample(5, (-152., 0., 0.)))
    for tick in range(6, 11):
        scorer.add(sample(tick))
    assert not scorer.result()["success"]
    scorer.add(sample(11))
    assert scorer.result()["success"]


@pytest.mark.parametrize("tick", [0, 2, 5])
def test_duplicate_or_missing_evidence_cannot_count_as_dwell(task, tick):
    scorer = WaypointScorer(task, 100.)
    scorer.add(sample(0))
    scorer.add(sample(tick))
    for later in range(tick + 1, tick + 20):
        scorer.add(sample(later))
    assert scorer.result()["outcome"] == "invalid_evidence"


@pytest.mark.parametrize("change,reason", [
    ({"position": (-99., 0., 0.)}, "minimum_range"),
    ({"position": (-1001., 0., 0.)}, "maximum_range"),
    ({"velocity": (2.1, 0., 0.)}, "relative_speed"),
    ({"force": (25.1, 0., 0.)}, "applied_force"),
])
def test_hard_violation_vetoes_later_completion(task, change, reason):
    scorer = WaypointScorer(task, 100.)
    scorer.add(sample(0, **change))
    for tick in range(1, 10):
        scorer.add(sample(tick))
    assert scorer.result()["outcome"] == "safety_violation"
    assert reason in scorer.result()["safety_violations"]
    assert scorer.result()["completion_time_s"] is None


def test_time_limit_and_force_integral_use_entire_failed_trial(task):
    scorer = WaypointScorer(task, 100.)
    for tick in range(51):
        scorer.add(sample(tick, (-160., 0., 0.), force=(3., 4., 0.)))
    result = scorer.result()
    assert result["outcome"] == "time_limit"
    assert result["thrust_impulse_n_s"] == pytest.approx(5.)
    assert result["thrust_delta_v_m_s"] == pytest.approx(0.05)


def test_teleport_nonfinite_and_fake_controller_success_fail_closed(task):
    for mutation in ({"external_state_writes": 1}, {"controller_success": True},
                     {"chaser_com_j2000": [math.nan] * 6}):
        scorer = WaypointScorer(task, 100.)
        raw = deepcopy(sample(0))
        raw.update(mutation)
        try:
            scorer.add(raw)
        except ValueError:
            pass
        assert scorer.result()["outcome"] == "invalid_evidence"


def make_artifact(directory, task, *, outcome="success"):
    from space_arm_platform.benchmark_execution import sha256, write_json
    directory.mkdir()
    write_json(directory / "task.json", task.model_dump(mode="json"))
    write_json(directory / "manifest.json", {"task_fingerprint": task.fingerprint(),
                                            "chaser_mass_kg": 100., "physics_path": "physics.jsonl"})
    scorer = WaypointScorer(task, 100.)
    rows = [sample(tick) for tick in range(6)]
    for row in rows:
        scorer.add(row)
    (directory / "physics.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    write_json(directory / "result.json", {"outcome": outcome, "physics_score": scorer.result(),
                                          "evidence_sha256": {name: sha256(directory / name)
                                                              for name in ("task.json", "manifest.json", "physics.jsonl")}})


def test_offline_replay_matches_and_rejects_changed_evidence(task, tmp_path):
    from space_arm_platform.benchmark_execution import rescore_waypoint
    directory = tmp_path / "run"
    make_artifact(directory, task)
    result = rescore_waypoint(directory)
    assert result["success"] and result["matches_recorded_physics_score"]
    with (directory / "physics.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(sample(6)) + "\n")
    with pytest.raises(ValueError, match="changed or invalid evidence"):
        rescore_waypoint(directory)


@pytest.mark.parametrize("outcome", ["cancelled", "infrastructure_error", "method_error"])
def test_replay_does_not_erase_nonphysical_failure(task, tmp_path, outcome):
    from space_arm_platform.benchmark_execution import rescore_waypoint
    directory = tmp_path / "run"
    make_artifact(directory, task, outcome=outcome)
    result = rescore_waypoint(directory)
    assert result["outcome"] == outcome and not result["success"]
    assert result["physics_score"]["success"]


def test_batch_failures_stay_in_denominator_and_pending_has_no_interval():
    from space_arm_platform.benchmark_batch import summarize
    outcomes = ["success", "time_limit", "infrastructure_error", "method_error", "invalid_evidence"]
    result = summarize([{"outcome": outcome} for outcome in outcomes])
    assert result["planned"] == 5 and result["finished"] == 5
    assert result["success_rate_all_planned"] == 0.2
    assert result["wilson_95_interval"][0] < 0.2 < result["wilson_95_interval"][1]
    partial = summarize([{"outcome": "success"}, {"outcome": "pending"}])
    assert not partial["complete"] and partial["wilson_95_interval"] is None
    assert partial["success_rate_all_planned"] == 0.5


def test_interrupted_trial_is_not_pending_or_implicitly_retried(task, tmp_path):
    from space_arm_platform.benchmark_batch import _completed_trial
    from space_arm_platform.benchmark_execution import write_json
    directory = tmp_path / "attempt"
    directory.mkdir()
    write_json(directory / "status.json", {"outcome": "running"})
    result = _completed_trial(directory, {"trial_id": "0000-seed-1", "seed": 1}, task, "gnss-pd-v1")
    assert result["outcome"] == "infrastructure_error"
    assert "not retried" in result["detail"]
