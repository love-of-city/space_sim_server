"""Attitude contract, quaternion-invariant grading and family dispatch guards."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_attitude import (
    AttitudeObservation, AttitudeTask, AttitudeTruthSample, WheelTorqueAction,
)
from space_arm_platform.benchmark_attitude_score import AttitudeScorer
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_registry import make_scorer, method_ids, parse_action


@pytest.fixture
def task():
    document = load_task(SPEC_DIRECTORY / "s02-wheel-attitude-v1.json").model_dump(mode="json")
    document["duration_s"] = 1.
    document["objective"]["dwell_s"] = 0.1
    return AttitudeTask.model_validate(document)


def sample(tick, quaternion=(1., 0., 0., 0.), rate=(0., 0., 0.), wheel=(0., 0., 0.), torque=(0., 0., 0.)):
    return {"time_ns": tick*20_000_000, "orientation_body_to_j2000_wxyz": quaternion,
            "angular_velocity_body_rad_s": rate, "wheel_speed_rad_s": wheel,
            "applied_motor_torque_xyz_nm": torque, "external_state_writes": 0}


@pytest.mark.parametrize("mutation", [{"source_id": "S01"}, {"spacecraft_count": 2}, {"spacecraft_count": True},
                                      {"target_cooperation": "C"}, {"action_type": "body_torque"},
                                      {"solar_gravity": 1}, {"duration_s": math.inf}])
def test_attitude_config_does_not_alias_other_semantics(task, mutation):
    with pytest.raises(ValidationError):
        AttitudeTask.model_validate({**task.model_dump(), **mutation})


def test_maneuver_is_required_and_initial_state_is_safe(task):
    for section, key, value in [("initial", "rotation_min_rad", 0.0001),
                                ("initial", "wheel_speed_rad_s", [700., 0., 0.]),
                                ("initial", "body_rate_half_width_rad_s", 0.15),
                                ("objective", "orientation_body_to_j2000_wxyz", [2., 0., 0., 0.])]:
        document = task.model_dump(mode="json")
        document[section][key] = value
        with pytest.raises(ValidationError):
            AttitudeTask.model_validate(document)


def test_quaternion_double_cover_and_continuous_dwell(task):
    scorer = AttitudeScorer(task)
    for tick in range(5):
        scorer.add(sample(tick, quaternion=((-1.)**tick, 0., 0., 0.)))
    assert not scorer.result()["success"]
    scorer.add(sample(5))
    assert scorer.result()["success"]
    assert scorer.result()["completion_time_s"] == pytest.approx(.1)


def test_attitude_match_without_rate_stability_is_not_success(task):
    scorer = AttitudeScorer(task)
    for tick in range(51):
        scorer.add(sample(tick, rate=(.002, 0., 0.)))
    assert scorer.result()["outcome"] == "time_limit"


def test_pointing_excursion_restarts_dwell(task):
    scorer = AttitudeScorer(task)
    for tick in range(5):
        scorer.add(sample(tick))
    scorer.add(sample(5, quaternion=(math.cos(.05), math.sin(.05), 0., 0.)))
    for tick in range(6, 11):
        scorer.add(sample(tick))
    assert not scorer.result()["success"]
    scorer.add(sample(11))
    assert scorer.result()["success"]


@pytest.mark.parametrize("change,reason", [
    ({"rate": (.16, 0., 0.)}, "body_rate"),
    ({"wheel": (629., 0., 0.)}, "wheel_speed"),
    ({"torque": (.21, 0., 0.)}, "motor_torque"),
])
def test_hard_limits_veto_later_hold(task, change, reason):
    scorer = AttitudeScorer(task)
    scorer.add(sample(0, **change))
    for tick in range(1, 10):
        scorer.add(sample(tick))
    assert scorer.result()["outcome"] == "safety_violation"
    assert reason in scorer.result()["safety_violations"]
    assert scorer.result()["completion_time_s"] is None


@pytest.mark.parametrize("tick", [0, 2])
def test_bad_cadence_invalidates_attitude_evidence(task, tick):
    scorer = AttitudeScorer(task)
    scorer.add(sample(0))
    scorer.add(sample(tick))
    assert scorer.result()["outcome"] == "invalid_evidence"


def test_overwritten_and_forged_typed_physics_fail_closed(task):
    raw = sample(0)
    raw["external_state_writes"] = 1
    scorer = AttitudeScorer(task)
    scorer.add(raw)
    assert scorer.result()["outcome"] == "invalid_evidence"
    forged = AttitudeTruthSample.model_validate(sample(0)).model_copy(update={"applied_motor_torque_xyz_nm": (math.nan, 0., 0.)})
    with pytest.raises(ValueError):
        AttitudeScorer(task).add(forged)


def test_attitude_actions_are_identity_bound_and_separate_from_translation(task):
    observation = AttitudeObservation(episode_id="case", sequence=0, sample_time_ns=0, delivery_time_ns=0,
        orientation_body_to_j2000_wxyz=(1., 0., 0., 0.), angular_velocity_body_rad_s=(0., 0., 0.),
        wheel_speed_rad_s=(0., 0., 0.), gyro_valid=True)
    action = WheelTorqueAction(episode_id="case", observation_sequence=0, motor_torque_xyz_nm=(.1, 0., 0.))
    assert parse_action(task, action, observation) == action
    for update in ({"episode_id": "old"}, {"observation_sequence": 1},
                   {"motor_torque_xyz_nm": (math.nan, 0., 0.)}, {"motor_torque_xyz_nm": (.21, 0., 0.)}):
        with pytest.raises(ValueError):
            parse_action(task, action.model_copy(update=update), observation)
    with pytest.raises(ValueError):
        parse_action(task, {"episode_id": "case", "observation_sequence": 0, "force_j2000_n": [0., 0., 0.]}, observation)
    with pytest.raises(ValueError):
        AttitudeObservation.model_validate({**observation.model_dump(), "truth_quaternion": [1., 0., 0., 0.]})
    assert isinstance(make_scorer(task, 100.), AttitudeScorer)
    assert "gnss-pd-v1" not in method_ids(task)


def test_motor_impulse_is_measured_without_claiming_electrical_energy(task):
    scorer = AttitudeScorer(task)
    for tick in range(6):
        scorer.add(sample(tick, torque=(.1, -.1, .05)))
    result = scorer.result()
    assert result["absolute_motor_impulse_xyz_nm_s"] == pytest.approx([.01, .01, .005])
    assert result["fuel_kg"] is None and result["energy_j"] is None


def test_controller_tracks_a_nonidentity_target_and_ignores_quaternion_sign(task):
    from simulation.benchmark_attitude_methods import AttitudeWheelPD
    from space_arm_platform.benchmark_attitude import AttitudeContext
    h = math.sqrt(.5)
    document = task.model_dump(mode="json")
    document["objective"]["orientation_body_to_j2000_wxyz"] = [h, h, 0., 0.]
    changed = AttitudeTask.model_validate(document)
    method = AttitudeWheelPD()
    method.reset(AttitudeContext(task=changed, chaser_mass_kg=100., inertia_com_body_kg_m2=((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
        wheel_spin_inertia_kg_m2=(.01, .01, .01), hardware_torque_limits_nm=(.2, .2, .2),
        hardware_speed_limits_rad_s=(628., 628., 628.)))
    c, s = math.cos(.05), math.sin(.05)
    orientation = (h*c, h*c, -h*s, h*s)  # target followed by a +0.1 rad body-z error
    observation = AttitudeObservation(episode_id="case", sequence=0, sample_time_ns=0, delivery_time_ns=0,
        orientation_body_to_j2000_wxyz=orientation, angular_velocity_body_rad_s=(0., 0., 0.),
        wheel_speed_rad_s=(0., 0., 0.), gyro_valid=True)
    torque = method.act(observation).motor_torque_xyz_nm
    assert torque[:2] == pytest.approx((0., 0.), abs=1e-12)
    assert torque[2] > 0
    flipped = observation.model_copy(update={"orientation_body_to_j2000_wxyz": tuple(-v for v in orientation)})
    assert method.act(flipped).motor_torque_xyz_nm == pytest.approx(torque)


def test_single_spacecraft_derivative_preserves_team_geometry_and_inertia(tmp_path):
    from simulation.benchmark_attitude_model import prepare_attitude_model
    from simulation.orbital_rendezvous.model import DEFAULT_MODEL
    server = Path(__file__).resolve().parents[1]
    source = ET.parse(server / DEFAULT_MODEL).getroot()
    model, files, manifest = prepare_attitude_model(server, tmp_path / "single")
    derived = ET.parse(model).getroot()
    assert [body.get("name") for body in derived.findall("./worldbody/body")] == ["cubesat_bus"]
    before = source.find('./worldbody/body[@name="cubesat_bus"]')
    after = derived.find('./worldbody/body[@name="cubesat_bus"]')
    assert [geom.attrib for geom in before.findall(".//geom")] == [geom.attrib for geom in after.findall(".//geom")]
    assert [item.attrib for item in before.findall(".//inertial")] == [item.attrib for item in after.findall(".//inertial")]
    expected = [item.attrib for item in source.findall("./actuator/motor") if item.get("name").startswith("rw_")]
    assert [item.attrib for item in derived.findall("./actuator/motor")] == expected
    assert len(files) == manifest["mesh_count"]
    assert set(manifest["bodies"]) == {"cubesat_bus"}
    assert all("ground_validation_satellite" not in name for name in manifest["meshes"])
