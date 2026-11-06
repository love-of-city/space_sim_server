"""Fault evidence predicates and public control boundary; synthetic logs are not native acceptance."""
import math

import numpy as np
import pytest

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_registry import make_scorer, method_classes, method_ids, parse_action
from space_arm_platform.benchmark_wheel_recovery import (
    MotorLossRealization, WheelRecoveryAction, WheelRecoveryContext, WheelRecoveryObservation,
    WheelRecoveryTask,
)
from space_arm_platform.benchmark_wheel_recovery_score import WheelRecoveryScorer


@pytest.fixture
def task():
    data = load_task(SPEC_DIRECTORY/'s22-wheel-effectiveness-recovery-v1.json').model_dump(mode='json')
    data['duration_s'] = 1.
    data['fault'].update(onset_earliest_s=.1, onset_latest_s=.1)
    data['objective'].update(dwell_s=.1, diagnosis_deadline_after_onset_s=.2,
        diagnosis_dwell_s=.06, minimum_lost_motor_impulse_nm_s=.001)
    return WheelRecoveryTask.model_validate(data)


def physical(tick, *, diagnosed=True, actual_loss=True, delay=0):
    active = tick > 5
    torque = [0., .1, 0.] if tick else [0., 0., 0.]
    actual = list(torque)
    if active and actual_loss:
        actual[1] *= .3
    estimate = [1., .3 if active and diagnosed and tick >= delay else 1., 1.]
    return dict(time_ns=tick*20_000_000,
        orientation_body_to_j2000_wxyz=(math.cos(.5), math.sin(.5), 0., 0.) if tick == 0 else (1., 0., 0., 0.),
        angular_velocity_body_rad_s=(0., 0., 0.), wheel_speed_rad_s=(0., 0., 0.),
        applied_motor_torque_xyz_nm=actual, external_state_writes=0,
        fault=dict(onset_time_ns=100_000_000, axis='y', effectiveness=.3),
        requested_motor_torque_xyz_nm=torque, estimated_effectiveness_xyz=estimate)


@pytest.mark.parametrize('section,key,value', [
    ('fault', 'axes', ['x', 'x']), ('fault', 'effectiveness_min', .6),
    ('fault', 'onset_latest_s', .101), ('fault', 'effectiveness_max', 1.),
    ('objective', 'diagnosis_dwell_s', .3), ('objective', 'minimum_lost_motor_impulse_nm_s', .1),
    ('safety', 'maximum_wheel_speed_rad_s', 628.), ('initial', 'rotation_min_rad', .001),
])
def test_recovery_contract_rejects_unobservable_or_incompatible_faults(task, section, key, value):
    data = task.model_dump(mode='json')
    data[section][key] = value
    with pytest.raises(ValueError):
        WheelRecoveryTask.model_validate(data)


def test_recovery_registration_is_distinct_and_legacy_attitude_fingerprint_remains():
    task = load_task(SPEC_DIRECTORY/'s22-wheel-effectiveness-recovery-v1.json')
    assert isinstance(make_scorer(task, 170.), WheelRecoveryScorer)
    assert method_ids(task)[0] == 'adaptive-wheel-recovery-v1'
    assert tuple(method_classes(task)) == method_ids(task)
    assert load_task(SPEC_DIRECTORY/'s02-wheel-attitude-v1.json').fingerprint() == 'abd19ea93eb5122b14c3fb81f3ad4812cb8c1e8f0e3d775ef0a17bffe3467fa2'
    for key, value in (('source_id', 'S02'), ('action_type', 'reaction_wheel_motor_torques_xyz')):
        with pytest.raises(ValueError):
            WheelRecoveryTask.model_validate(task.model_dump() | {key: value})


def test_nominal_hold_does_not_count_as_post_diagnosis_recovery(task):
    scorer = WheelRecoveryScorer(task)
    for tick in range(14):
        scorer.add(physical(tick, delay=8))
        assert not scorer.result()['success']
    for tick in range(14, 17):
        scorer.add(physical(tick, delay=8))
    result = scorer.result()
    assert result['outcome'] == 'success'
    assert result['diagnosis_confirmed_s'] == pytest.approx(.22)
    assert result['completion_time_s'] == pytest.approx(.32)
    assert result['longest_dwell_s'] == pytest.approx(.1)
    assert result['fault_triggered'] and result['lost_motor_impulse_nm_s'] > .001
    assert not result['quality_violations'] and not result['evidence_errors']


@pytest.mark.parametrize('kind', ['no_diagnosis', 'late_diagnosis', 'unloaded_fault', 'false_alarm'])
def test_attitude_success_cannot_hide_diagnosis_failure(task, kind):
    scorer = WheelRecoveryScorer(task)
    for tick in range(51):
        row = physical(tick, diagnosed=kind != 'no_diagnosis', delay=20 if kind == 'late_diagnosis' else 0)
        if kind == 'unloaded_fault':
            row['requested_motor_torque_xyz_nm'] = row['applied_motor_torque_xyz_nm'] = (0., 0., 0.)
        if kind == 'false_alarm' and tick <= 5:
            row['estimated_effectiveness_xyz'] = (1., .3, 1.)
        scorer.add(row)
    result = scorer.result()
    assert result['outcome'] == 'task_failure' and result['completion_time_s'] is None
    assert result['quality_violations']
    assert not result['evidence_errors']


@pytest.mark.parametrize('kind', ['no_physical_loss', 'changed_realization', 'gap', 'state_write', 'initial_target', 'pre_observation_force'])
def test_fault_flags_and_reported_diagnosis_cannot_forge_physical_evidence(task, kind):
    scorer = WheelRecoveryScorer(task)
    for tick in range(26):
        if kind == 'gap' and tick == 8:
            continue
        row = physical(tick, actual_loss=kind != 'no_physical_loss')
        if kind == 'changed_realization' and tick >= 10:
            row['fault']['axis'] = 'z'
        if kind == 'state_write' and tick == 10:
            row['external_state_writes'] = 1
        if kind == 'initial_target' and tick == 0:
            row['orientation_body_to_j2000_wxyz'] = (1., 0., 0., 0.)
        if kind == 'pre_observation_force' and tick == 0:
            row['requested_motor_torque_xyz_nm'] = row['applied_motor_torque_xyz_nm'] = (0., .1, 0.)
        scorer.add(row)
    assert scorer.result()['outcome'] == 'invalid_evidence'


def test_diagnosis_does_not_override_hard_body_rate_limit(task):
    scorer = WheelRecoveryScorer(task)
    for tick in range(26):
        row = physical(tick)
        if tick == 10:
            row['angular_velocity_body_rad_s'] = (.2, 0., 0.)
        scorer.add(row)
    result = scorer.result()
    assert result['outcome'] == 'safety_violation' and 'body_rate' in result['safety_violations']


def context(task):
    return WheelRecoveryContext(task=task, chaser_mass_kg=170.,
        inertia_com_body_kg_m2=((1., 0., 0.), (0., 2., 0.), (0., 0., 3.)),
        wheel_spin_inertia_kg_m2=(.01, .01, .01), hardware_torque_limits_nm=(.2, .2, .2),
        hardware_speed_limits_rad_s=(628., 628., 628.))


def observation(tick, torque=(0., 0., 0.)):
    return WheelRecoveryObservation(episode_id='test', sequence=tick,
        sample_time_ns=tick*20_000_000, delivery_time_ns=tick*20_000_000,
        orientation_body_to_j2000_wxyz=(math.cos(.3), 0., math.sin(.3), 0.),
        angular_velocity_body_rad_s=(0., 0., 0.), gyro_valid=True,
        wheel_speed_rad_s=(0., 0., 0.), measured_motor_torque_xyz_nm=torque)


def test_motor_diagnosis_uses_previous_command_and_current_telemetry_and_resets(task):
    from simulation.benchmark_wheel_recovery_methods import AdaptiveWheelRecovery
    method = AdaptiveWheelRecovery()
    method.reset(context(task))
    first = method.act(observation(0))
    command = first.motor_torque_xyz_nm
    for tick in range(1, 26):
        row = observation(tick, tuple(np.asarray(command)*[1., .3, 1.]))
        action = method.act(row)
        command = action.motor_torque_xyz_nm
    assert action.estimated_effectiveness_xyz == pytest.approx((1., .3, 1.))
    method.reset(context(task))
    assert method.act(observation(0)) == first
    assert not method.history


def test_public_inputs_do_not_contain_realized_fault_and_actions_remain_bound(task):
    public = observation(0)
    for secret in ('fault', 'fault_axis', 'fault_onset_time_ns', 'fault_effectiveness'):
        with pytest.raises(ValueError):
            WheelRecoveryObservation.model_validate(public.model_dump() | {secret: 1})
    action = WheelRecoveryAction(episode_id='test', observation_sequence=0,
        motor_torque_xyz_nm=(0., .1, 0.), estimated_effectiveness_xyz=(1., 1., 1.))
    assert parse_action(task, action, public) == action
    for change in ({'observation_sequence': 1}, {'estimated_effectiveness_xyz': (0., 1., 1.)},
                   {'motor_torque_xyz_nm': (0., .21, 0.)}, {'estimated_effectiveness_xyz': (math.nan, 1., 1.)}):
        with pytest.raises(ValueError):
            parse_action(task, action.model_copy(update=change), public)
    with pytest.raises(ValueError):
        MotorLossRealization(onset_time_ns=0, axis='x', effectiveness=.3).validate_for(task)
