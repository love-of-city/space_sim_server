"""Navigation timing/frame boundaries and accuracy grading over the entire window."""
import math
import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_navigation import (
    GNSSMeasurements, NavigationEstimateAction, NavigationEstimateEvidence, NavigationTask,
)
from space_arm_platform.benchmark_navigation_score import NavigationScorer
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS
from space_arm_platform.benchmark_registry import parse_action


def task(relative=False):
    filename = "d01-cooperative-gnss-estimation-v1.json" if relative else "s14-gnss-state-estimation-v1.json"
    document = load_task(SPEC_DIRECTORY/filename).model_dump()
    document["duration_s"] = 1.
    return NavigationTask.model_validate(document)


def physical(tick, relative=False, *, altitude=500000., force=(0., 0., 0.)):
    radius = RADIUS+altitude
    n = math.sqrt(MU/radius**3)
    phase, inc = math.pi+n*tick*.02, math.radians(51.6)
    radial = np.array([math.cos(phase), math.sin(phase)*math.cos(inc), math.sin(phase)*math.sin(inc)])
    tangent = np.array([-math.sin(phase), math.cos(phase)*math.cos(inc), math.cos(phase)*math.sin(inc)])
    normal = np.cross(radial, tangent)
    state = np.r_[radius*radial, radius*n*tangent]
    row = dict(time_ns=tick*20_000_000, applied_motor_force_body_n=force, external_state_writes=0)
    if relative:
        displacement = -160*tangent+3*normal+2*radial
        velocity = np.cross(n*normal, displacement)
        row.update(target_com_j2000=tuple(state), chaser_com_j2000=tuple(state+np.r_[displacement, velocity]))
    else:
        row.update(chaser_com_j2000=tuple(state), orientation_body_to_j2000_wxyz=(1., 0., 0., 0.))
    return row


def estimate(spec, tick, error=(0.,)*6):
    row = physical(tick, spec.source_id == "D01")
    state = np.asarray(row["chaser_com_j2000"])
    if spec.source_id == "D01":
        state = state-row["target_com_j2000"]
    return NavigationEstimateEvidence(sequence=tick//25, estimate_time_ns=tick*20_000_000,
        reference_frame=spec.reference_frame, estimated_state=tuple(state+error))


@pytest.mark.parametrize("change", [dict(controlled_spacecraft=1), dict(controlled_spacecraft=False),
    dict(source_id="D01"), dict(spacecraft_count=2), dict(reference_frame="chaser_minus_target_J2000"),
    dict(vision_mode="V2"), dict(duration_s=1.1)])
def test_navigation_contract_cannot_alias_control_or_different_navigation_semantics(change):
    with pytest.raises(ValidationError):
        NavigationTask.model_validate(task().model_dump() | change)


@pytest.mark.parametrize("relative", [False, True])
def test_accurate_estimates_require_the_entire_time_window(relative):
    spec = task(relative)
    scorer = NavigationScorer(spec, 100.)
    for tick in range(51):
        scorer.add(physical(tick, relative))
        if tick < 50 and tick % 25 == 0:
            scorer.add(estimate(spec, tick, (.1, .2, 0., .001, .002, 0.)))
        if tick < 50:
            assert scorer.result()["outcome"] == "incomplete"
    result = scorer.result()
    assert result["outcome"] == "success"
    assert result["estimate_count"] == result["expected_estimates"] == 2
    assert result["sample_count"] == 51
    assert result["position_rmse_m"] == pytest.approx(math.sqrt(.05), abs=1e-8)
    assert result["velocity_rmse_m_s"] == pytest.approx(math.sqrt(.000005), abs=1e-10)
    assert result["thrust_impulse_n_s"] == 0


def test_bad_early_estimate_is_not_erased_by_an_accurate_final_estimate():
    spec = task()
    scorer = NavigationScorer(spec, 100.)
    for tick in range(51):
        scorer.add(physical(tick))
        if tick in (0, 25):
            scorer.add(estimate(spec, tick, (2., 0., 0., 0., 0., 0.) if tick == 0 else (0.,)*6))
    result = scorer.result()
    assert result["final_position_error_m"] == 0
    assert result["outcome"] == "task_failure"
    assert result["position_rmse_m"] == pytest.approx(math.sqrt(2))
    assert result["quality_violations"] == ["position_rmse"]


@pytest.mark.parametrize("fault", ["missing", "duplicate", "late", "wrong_frame", "physical_gap", "state_write"])
def test_missing_or_noncontemporaneous_records_cannot_pass(fault):
    spec = task()
    scorer = NavigationScorer(spec, 100.)
    for tick in range(51):
        if fault == "physical_gap" and tick == 12:
            continue
        row = physical(tick)
        if fault == "state_write" and tick == 3:
            row["external_state_writes"] = 1
        scorer.add(row)
        if tick in (0, 25):
            record = estimate(spec, tick).model_dump()
            if tick == 25:
                if fault == "missing":
                    continue
                if fault == "late":
                    record["estimate_time_ns"] -= 20_000_000
                if fault == "wrong_frame":
                    record["reference_frame"] = "chaser_minus_target_J2000"
            scorer.add(record)
            if fault == "duplicate" and tick == 0:
                scorer.add(record)
    assert scorer.result()["outcome"] == "invalid_evidence"


def test_invalid_initialization_is_separate_from_method_accuracy_failure():
    scorer = NavigationScorer(task(), 100.)
    scorer.add(physical(0, altitude=501000.))
    assert scorer.result()["outcome"] == "invalid_initialization"
    assert scorer.result()["initialization_errors"] == ["initial_orbit_outside_declared_distribution"]


def test_navigation_has_no_propulsion_authority_even_if_estimates_are_exact():
    spec = task()
    scorer = NavigationScorer(spec, 100.)
    for tick in range(51):
        scorer.add(physical(tick, force=(.001, 0., 0.) if tick == 30 else (0., 0., 0.)))
        if tick in (0, 25):
            scorer.add(estimate(spec, tick))
    assert scorer.result()["outcome"] == "safety_violation"
    assert scorer.result()["safety_violations"] == ["navigation_actuation_not_allowed"]


def test_estimates_are_sample_tied_bounded_and_cannot_request_force():
    spec = task()
    obs = GNSSMeasurements(episode_id="case", sequence=0, sample_time_ns=0, delivery_time_ns=0,
                           chaser_measurement_j2000=physical(0)["chaser_com_j2000"], target_measurement_j2000=None)
    action = NavigationEstimateAction(episode_id="case", observation_sequence=0, estimate_time_ns=0,
                                     reference_frame=spec.reference_frame, estimated_state=obs.chaser_measurement_j2000)
    assert parse_action(spec, action, obs) == action
    for update in (dict(estimate_time_ns=1), dict(episode_id="old"), dict(observation_sequence=1),
                   dict(reference_frame="chaser_minus_target_J2000"), dict(estimated_state=(math.nan,)*6),
                   dict(estimated_state=(1e300,)*6), dict(force_j2000_n=(0., 0., 0.))):
        with pytest.raises(ValueError):
            parse_action(spec, action.model_dump() | update, obs)
    with pytest.raises(ValidationError):
        GNSSMeasurements.model_validate(obs.model_dump() | {"chaser_truth_j2000": (0.,)*6})
