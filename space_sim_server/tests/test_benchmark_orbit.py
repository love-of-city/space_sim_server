"""Reject altitude-only success, premature burns, discontinuous dwell and truth leaks."""
import math
import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, load_task
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS, OrbitObservation, OrbitTask
from space_arm_platform.benchmark_orbit_score import OrbitScorer, orbital_elements, rotate_body_force
from space_arm_platform.benchmark_registry import make_scorer, parse_action, method_ids


def task(name="s11-orbit-raise-v1", **objective):
    doc = load_task(SPEC_DIRECTORY / (name+".json")).model_dump()
    doc["objective"].update(objective)
    return OrbitTask.model_validate(doc)


def circular(altitude, speed_scale=1.):
    i = math.radians(51.6)
    v = math.sqrt(MU/(RADIUS+altitude))*speed_scale
    return (-RADIUS-altitude, 0., 0., 0., -v*math.cos(i), -v*math.sin(i))


def sample(tick, altitude=500000., force=(0., 0., 0.), speed_scale=1.):
    return dict(time_ns=tick*20_000_000, chaser_com_j2000=circular(altitude, speed_scale),
                orientation_body_to_j2000_wxyz=(1., 0., 0., 0.), applied_motor_force_body_n=force, external_state_writes=0)


@pytest.mark.parametrize("change", [dict(source_id="S09"), dict(spacecraft_count=True), dict(solar_gravity=1),
                                    dict(vision_mode="V2"), dict(duration_s=math.inf)])
def test_orbit_task_semantics_are_explicit(change):
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(task().model_dump() | change)


def test_target_must_require_an_orbit_change_and_remain_safe():
    for goal in (dict(altitude_m=500005.), dict(altitude_m=549999.), dict(maximum_eccentricity=.02)):
        with pytest.raises(ValidationError):
            task(**goal)
    with pytest.raises(ValidationError):
        task("s09-finite-burn-v1", delta_velocity_j2000_m_s=(0., 0., 0.))
    with pytest.raises(ValidationError):
        task("s09-finite-burn-v1", latest_burn_s=60.)


def test_orbital_elements_distinguish_radius_from_energy_and_shape():
    circular_orbit = orbital_elements(circular(501000.))
    assert circular_orbit["semimajor_axis_m"] == pytest.approx(RADIUS+501000., abs=1e-8)
    assert circular_orbit["eccentricity"] < 1e-14
    elliptic = orbital_elements(circular(501000., 1.001))
    assert elliptic["altitude_m"] == circular_orbit["altitude_m"]
    assert elliptic["eccentricity"] > .001
    assert elliptic["apogee_altitude_m"] > 520000.
    escape = orbital_elements(circular(501000., 1.5))
    assert escape["semimajor_axis_m"] is None


def test_success_requires_correct_orbit_unpowered_continuous_dwell():
    spec = task(dwell_s=.1)
    scorer = OrbitScorer(spec, 100.)
    scorer.add(sample(0))
    for tick in range(1, 7):
        scorer.add(sample(tick, 501000., speed_scale=1.001))
    assert scorer.result()["outcome"] == "incomplete"
    for tick in range(7, 13):
        scorer.add(sample(tick, 501000., force=(1., 0., 0.)))
    assert scorer.result()["longest_dwell_s"] == 0
    for tick in range(13, 18):
        scorer.add(sample(tick, 501000.))
    assert scorer.result()["outcome"] == "incomplete"
    scorer.add(sample(18, 501000.))
    assert scorer.result()["outcome"] == "success"
    scorer.add(sample(19, 501000., force=(26., 0., 0.)))
    assert scorer.result()["outcome"] == "safety_violation"


@pytest.mark.parametrize("invalid", [dict(altitude=501000.), dict(speed_scale=1.001)])
def test_initialized_inside_goal_cannot_pass(invalid):
    scorer = OrbitScorer(task(dwell_s=.1), 100.)
    scorer.add(sample(0, **invalid))
    for tick in range(1, 8):
        scorer.add(sample(tick, 501000.))
    assert scorer.result()["outcome"] == "invalid_evidence"


@pytest.mark.parametrize("kind", ["gap", "repeat", "write"])
def test_dwell_evidence_cannot_be_repaired_after_discontinuity(kind):
    scorer = OrbitScorer(task(dwell_s=.1), 100.)
    scorer.add(sample(0))
    wrong = sample(2 if kind == "gap" else 0 if kind == "repeat" else 1, 501000.)
    if kind == "write":
        wrong["external_state_writes"] = 1
    scorer.add(wrong)
    for tick in range(3, 11):
        scorer.add(sample(tick, 501000.))
    assert scorer.result()["outcome"] == "invalid_evidence"


def test_impulse_uses_actual_force_and_stops_before_confirmation():
    spec = task("s09-finite-burn-v1", delta_velocity_j2000_m_s=(0., -.02, 0.), vector_tolerance_m_s=.0001, dwell_s=.1)
    scorer = OrbitScorer(spec, 100.)
    for tick in range(251):
        scorer.add(sample(tick))
    for tick in range(251, 256):
        scorer.add(sample(tick, force=(0., -20., 0.)))
    assert scorer.result()["delta_velocity_j2000_m_s"] == pytest.approx([0., -.02, 0.])
    assert scorer.result()["outcome"] == "incomplete"
    for tick in range(256, 262):
        scorer.add(sample(tick))
    assert scorer.result()["outcome"] == "success"
    early = OrbitScorer(spec, 100.)
    early.add(sample(0))
    early.add(sample(1, force=(0., -20., 0.)))
    assert early.result()["safety_violations"] == ["thrust_outside_burn_window"]


def test_force_rotation_is_quaternion_sign_invariant():
    q = (math.sqrt(.5), 0., 0., math.sqrt(.5))
    assert rotate_body_force(q, (1., 0., 0.)) == pytest.approx((0., 1., 0.))
    assert np.allclose(rotate_body_force(q, (1., 2., 3.)), rotate_body_force(tuple(-v for v in q), (1., 2., 3.)))


def test_orbit_observations_do_not_expose_truth_or_target():
    spec = task()
    obs = OrbitObservation(episode_id="one", sequence=0, sample_time_ns=0, delivery_time_ns=0,
                           chaser_estimate_j2000=circular(500000.))
    for key in ("target_estimate_j2000", "chaser_com_j2000", "orientation_body_to_j2000_wxyz"):
        with pytest.raises(ValidationError):
            OrbitObservation.model_validate(obs.model_dump() | {key: (0.,)*6})
    action = ForceAction(episode_id="one", observation_sequence=0, force_j2000_n=(1., 0., 0.))
    assert parse_action(spec, action, obs) == action
    with pytest.raises(ValueError, match="stale"):
        parse_action(spec, action.model_copy(update={"episode_id": "old"}), obs)
    with pytest.raises(ValidationError):
        parse_action(spec, action.model_copy(update={"force_j2000_n": (math.nan, 0., 0.)}), obs)
    assert isinstance(make_scorer(spec, 100.), OrbitScorer)
    assert method_ids(spec)[0] == "gnss-orbit-maneuver-v1"
