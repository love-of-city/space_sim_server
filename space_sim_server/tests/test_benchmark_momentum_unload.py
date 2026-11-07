"""Physical unloading cannot be replaced by attitude hold or wheel braking alone."""
import json
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from simulation.benchmark_momentum_unload_methods import MomentumUnloadPD
from space_arm_platform.benchmark_attitude import AttitudeObservation
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_external import ExternalMethodDefinition, ExternalMethod
from space_arm_platform.benchmark_momentum_unload import MomentumUnloadTask, MomentumUnloadContext, MomentumUnloadAction
from space_arm_platform.benchmark_momentum_unload_score import MomentumUnloadScorer
from space_arm_platform.benchmark_registry import make_scorer, method_ids, parse_action


def task(short=False):
    original = load_task(SPEC_DIRECTORY/'s08-wheel-momentum-unload-v1.json')
    if not short:
        return original
    data = original.model_dump(mode='json')
    data['duration_s'] = .2
    data['objective'].update(dwell_s=.04, minimum_external_torque_impulse_nm_s=.001)
    return MomentumUnloadTask.model_validate(data)


def sample(tick, *, wheels=(300., -300., 300.), external=(0., 0., 0.), **changes):
    angle = .03 if tick == 0 else 0.
    row = dict(time_ns=tick*20_000_000,
        orientation_body_to_j2000_wxyz=(float(np.cos(angle/2)), float(np.sin(angle/2)), 0., 0.),
        angular_velocity_body_rad_s=(0., 0., 0.), wheel_speed_rad_s=(600., -580., 560.) if tick == 0 else wheels,
        applied_motor_torque_xyz_nm=(0., 0., 0.), external_state_writes=0,
        requested_external_torque_body_nm=(0., 0., 0.) if tick == 0 else external,
        applied_external_torque_body_nm=(0., 0., 0.) if tick == 0 else external)
    return row | changes


@pytest.mark.parametrize('change', [
    {'source_id': 'S02'}, {'target_cooperation': 'C'}, {'spacecraft_count': 2}, {'controlled_spacecraft': True},
    {'vision_mode': 'V2'}, {'external_torque_model': 'reaction_wheels_only'}, {'wheel_spin_inertia_kg_m2': [.1, .1, .1]},
])
def test_unload_contract_rejects_unrelated_task_and_hardware(change):
    with pytest.raises(ValidationError):
        MomentumUnloadTask.model_validate(task().model_dump() | change)


@pytest.mark.parametrize('field,change', [
    ('initial', {'wheel_speed_rad_s': [0., 0., 0.]}),
    ('objective', {'maximum_wheel_speed_rad_s': 600.}),
    ('objective', {'minimum_wheel_momentum_reduction_nm_s': 11.}),
    ('objective', {'minimum_external_torque_impulse_nm_s': 10.}),
    ('safety', {'maximum_external_torque_norm_nm': .03}),
])
def test_unload_task_requires_realistic_nontrivial_initial_and_terminal_margins(field, change):
    data = task().model_dump()
    data[field].update(change)
    with pytest.raises(ValidationError):
        MomentumUnloadTask.model_validate(data)


def test_goal_requires_reduced_wheel_momentum_actual_external_impulse_and_torque_off_hold():
    spec = task(short=True)
    score = make_scorer(spec, 170.)
    for tick in range(6):
        score.add(sample(tick, external=(.02, 0., 0.)))
        assert score.result()['outcome'] != 'success'
    score.add(sample(6))
    score.add(sample(7))
    assert score.result()['outcome'] == 'incomplete'
    score.add(sample(8))
    result = score.result()
    assert result['outcome'] == 'success' and result['longest_dwell_s'] == .04
    assert result['external_torque_norm_impulse_nm_s'] == pytest.approx(.002)
    assert result['wheel_momentum_norm_reduction_nm_s'] > 4.
    assert result['fuel_kg'] is None and result['energy_j'] is None


@pytest.mark.parametrize('external,wheels,expected', [
    ((0., 0., 0.), (300., -300., 300.), 'insufficient_actual_external_torque_impulse'),
    ((.02, 0., 0.), (600., -580., 560.), 'wheel_speed_not_unloaded'),
    ((.02, 0., 0.), (300., -300., 300.), 'external_torque_still_active'),
])
def test_no_false_success_from_wheel_speed_or_attitude_only(external, wheels, expected):
    score = MomentumUnloadScorer(task(short=True))
    for tick in range(11):
        score.add(sample(tick, external=external, wheels=wheels))
    assert score.result()['outcome'] == 'task_failure'
    assert expected in score.result()['task_quality_failures']


@pytest.mark.parametrize('change,violation', [
    ({'external_state_writes': 1}, 'physical_state_overwritten_after_initialization'),
    ({'applied_external_torque_body_nm': (.019, 0., 0.)}, 'external_torque_readback_differs_from_ideal_actuator'),
    ({'time_ns': 0}, 'missing_duplicate_or_reordered_physics_sample'),
])
def test_unloading_physical_evidence_cannot_use_writes_or_mismatched_torque(change, violation):
    score = MomentumUnloadScorer(task(short=True))
    score.add(sample(0))
    score.add(sample(1, external=(.02, 0., 0.), **change))
    assert score.result()['outcome'] == 'invalid_evidence'
    assert violation in score.result()['evidence_errors']


def test_norm_limit_is_not_three_independent_axis_limits_and_safety_vetoes_success():
    score = MomentumUnloadScorer(task(short=True))
    score.add(sample(0))
    score.add(sample(1, external=(.02, .02, .02)))
    assert 'external_torque_norm' in score.result()['safety_violations']
    score = MomentumUnloadScorer(task(short=True))
    for tick in range(9):
        score.add(sample(tick, external=(.02, 0., 0.) if tick < 6 else (0., 0., 0.),
            angular_velocity_body_rad_s=(.16, 0., 0.) if tick == 1 else (0., 0., 0.)))
    assert score.result()['outcome'] == 'safety_violation' and not score.result()['success']


def context():
    return MomentumUnloadContext(task=task(), chaser_mass_kg=170.,
        inertia_com_body_kg_m2=((50., 0., 0.), (0., 60., 0.), (0., 0., 70.)),
        wheel_spin_inertia_kg_m2=(.01, .01, .01), hardware_torque_limits_nm=(.2, .2, .2),
        hardware_speed_limits_rad_s=(628.3185307179587,)*3)


def observation(sequence=0, wheels=(600., -580., 560.)):
    return AttitudeObservation(episode_id='unload', sequence=sequence, sample_time_ns=sequence*20_000_000,
        delivery_time_ns=sequence*20_000_000, orientation_body_to_j2000_wxyz=(1., 0., 0., 0.),
        angular_velocity_body_rad_s=(0., 0., 0.), wheel_speed_rad_s=wheels, gyro_valid=True)


def test_original_hysteretic_policy_restarts_per_episode_and_compensates_bus_reaction():
    method = MomentumUnloadPD()
    method.reset(context())
    first = method.act(observation())
    assert np.linalg.norm(first.external_torque_body_nm) == pytest.approx(.02)
    np.testing.assert_allclose(first.motor_torque_xyz_nm, first.external_torque_body_nm)
    assert all(t*w < 0 for t, w in zip(first.motor_torque_xyz_nm, observation().wheel_speed_rad_s, strict=True))
    off = method.act(observation(1, wheels=(300., -300., 300.)))
    assert off.external_torque_body_nm == (0., 0., 0.)
    assert method.act(observation(2, wheels=(400., -400., 400.))).external_torque_body_nm == (0., 0., 0.)
    method.reset(context())
    assert method.act(observation()) == first
    assert method_ids(task())[0] == 'momentum-unload-pd-v1'


def test_action_norm_identity_and_no_truth_input():
    public = observation()
    action = MomentumUnloadAction(episode_id='unload', observation_sequence=0,
        motor_torque_xyz_nm=(0., 0., 0.), external_torque_body_nm=(.02, .02, .02))
    with pytest.raises(ValueError, match='norm'):
        parse_action(task(), action.model_dump(), public)
    action = action.model_copy(update={'external_torque_body_nm': (0., 0., 0.)})
    assert parse_action(task(), action.model_dump(), public) == action
    with pytest.raises(ValueError, match='stale'):
        parse_action(task(), action.model_dump(), observation(1))
    with pytest.raises(ValidationError):
        AttitudeObservation.model_validate(public.model_dump() | {'physical_state': [1., 2., 3.]})


def test_external_unload_policy_receives_only_public_contract_and_matches_in_process(tmp_path):
    path = Path(__file__).resolve().parents[1]/'examples/autonomy_methods/momentum-unload.json'
    definition = ExternalMethodDefinition(path, task())
    external = ExternalMethod(definition, tmp_path)
    internal = MomentumUnloadPD()
    try:
        external.reset(context())
        internal.reset(context())
        for tick, speed in enumerate((600., 500., 300., 400., 600.)):
            public = observation(tick, wheels=(speed, -speed, speed))
            expected = internal.act(public)
            actual = parse_action(task(), external.act(public), public)
            assert actual == expected
    finally:
        external.close()
    assert 'truth' not in json.dumps(context().model_dump()).lower()
