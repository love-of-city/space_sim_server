"""GNSS loss must occur in delivered data, and recovery needs continuous accuracy."""
from copy import deepcopy

import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_navigation_recovery import (
    GNSSOutageRealization, NavigationRecoveryContext, NavigationRecoveryDelivery,
    NavigationRecoveryObservation, NavigationRecoveryTask,
)
from space_arm_platform.benchmark_navigation_recovery_score import NavigationRecoveryScorer
from space_arm_platform.benchmark_registry import make_scorer, method_classes, method_ids, method_input_types, parse_action
from test_benchmark_navigation import estimate, physical


def task():
    document = load_task(SPEC_DIRECTORY/'s23-gnss-outage-recovery-v1.json').model_dump()
    document['duration_s'] = 21.
    document['fault'].update(earliest_onset_s=5., latest_onset_s=5., minimum_duration_s=5., maximum_duration_s=5.)
    document['objective']['recovery_deadline_after_return_s'] = 8.
    return NavigationRecoveryTask.model_validate(document)


OUTAGE = GNSSOutageRealization(onset_time_ns=5_000_000_000, return_time_ns=10_000_000_000)


def delivery(tick):
    available = OUTAGE.available(tick*20_000_000)
    return NavigationRecoveryDelivery(sequence=tick//25, sample_time_ns=tick*20_000_000,
        delivery_time_ns=tick*20_000_000, receiver_available=available,
        measurement_j2000=physical(tick)['chaser_com_j2000'] if available else None)


def replay(*, fault=None, error_at=None, assert_incomplete=False):
    spec = task()
    scorer = make_scorer(spec, 170.)
    for tick in range(1051):
        if fault == 'physical_gap' and tick == 123:
            continue
        row = physical(tick)
        row.update(outage=OUTAGE.model_dump(), receiver_available=OUTAGE.available(row['time_ns']))
        if fault == 'availability' and tick == 250:
            row['receiver_available'] = True
        if fault == 'state_write' and tick == 123:
            row['external_state_writes'] = 1
        if fault == 'actuation' and tick == 123:
            row['applied_motor_force_body_n'] = (.001, 0., 0.)
        scorer.add(row)
        if tick % 25 == 0:
            delivered = delivery(tick).model_dump()
            if fault == 'solution_during_outage' and tick == 250:
                delivered.update(receiver_available=True, measurement_j2000=row['chaser_com_j2000'])
            if not ((fault == 'missing_delivery' and tick == 250) or (fault == 'terminal_delivery' and tick == 1050)):
                scorer.add(delivered)
            if fault == 'duplicate_delivery' and tick == 250:
                scorer.add(delivered)
            if tick < 1050 and not (fault == 'missing_estimate' and tick == 250):
                scorer.add(estimate(spec, tick, error_at(tick) if error_at else (0.,)*6))
        if assert_incomplete and tick < 1050:
            assert scorer.result()['outcome'] == 'incomplete'
    return scorer.result()


def test_full_window_outage_and_continuous_reacquisition_are_all_required():
    result = replay(assert_incomplete=True)
    assert result['outcome'] == 'success'
    assert result['sample_count'] == 1051 and result['estimate_count'] == 42
    assert result['receiver_delivery_count'] == 43
    assert result['missing_measurement_count'] == result['expected_missing_measurements'] == result['outage_estimate_count'] == 10
    assert result['recovery_confirmation_latency_s'] == 5.
    assert result['completion_time_s'] == 21. and result['thrust_impulse_n_s'] == 0.


def test_accurate_endpoint_does_not_erase_bad_estimates_during_loss():
    result = replay(error_at=lambda tick: (4., 0., 0., 0., 0., 0.) if tick == 250 else (0.,)*6)
    assert result['outcome'] == 'task_failure' and result['final_position_error_m'] == 0.
    assert result['quality_violations'] == ['outage_position_error']
    assert result['maximum_outage_position_error_m'] == 4.


def test_short_accurate_intervals_cannot_accumulate_recovery_confirmation():
    # Return at 10 s; one bad estimate at 14.5 s resets the first 4 s hold.
    result = replay(error_at=lambda tick: (1., 0., 0., 0., 0., 0.) if tick == 725 else (0.,)*6)
    assert result['outcome'] == 'task_failure'
    assert result['quality_violations'] == ['navigation_recovery_deadline']
    assert result['recovery_confirmation_latency_s'] == 10.


@pytest.mark.parametrize('fault', ['physical_gap', 'availability', 'state_write', 'solution_during_outage',
    'missing_delivery', 'duplicate_delivery', 'terminal_delivery', 'missing_estimate'])
def test_missing_false_or_noncontemporaneous_fault_evidence_cannot_pass(fault):
    result = replay(fault=fault)
    assert result['outcome'] == 'invalid_evidence' and result['evidence_errors']


def test_exact_estimation_cannot_grant_motor_authority():
    result = replay(fault='actuation')
    assert result['outcome'] == 'safety_violation'
    assert result['safety_violations'] == ['navigation_actuation_not_allowed']


@pytest.mark.parametrize('path,value', [
    (('fault', 'earliest_onset_s'), 0.), (('fault', 'latest_onset_s'), 4.),
    (('fault', 'minimum_duration_s'), 6.), (('fault', 'latest_onset_s'), 5.1),
    (('fault', 'maximum_duration_s'), 100.), (('objective', 'recovered_dwell_s'), 8.),
    (('objective', 'outage_maximum_position_error_m'), 6.),
    (('observation', 'target_link'), 'cooperative_target_gnss_solution'),
    (('controlled_spacecraft',), 1), (('spacecraft_count',), 2), (('source_id',), 'S14'),
])
def test_fault_contract_cannot_alias_healthy_navigation_or_unobservable_recovery(path, value):
    document = deepcopy(task().model_dump())
    parent = document
    for key in path[:-1]:
        parent = parent[key]
    parent[path[-1]] = value
    with pytest.raises(ValidationError):
        NavigationRecoveryTask.model_validate(document)


def test_private_fault_realization_is_bounded_and_cannot_change_during_episode():
    spec = load_task(SPEC_DIRECTORY/'s23-gnss-outage-recovery-v1.json')
    scorer = NavigationRecoveryScorer(spec, 170.)
    outage = GNSSOutageRealization(onset_time_ns=20_000_000_000, return_time_ns=40_000_000_000)
    scorer.add(physical(0) | dict(outage=outage.model_dump(), receiver_available=True))
    changed = GNSSOutageRealization(onset_time_ns=21_000_000_000, return_time_ns=41_000_000_000)
    scorer.add(physical(1) | dict(outage=changed.model_dump(), receiver_available=True))
    assert 'receiver_outage_realization_changed' in scorer.result()['evidence_errors']
    with pytest.raises(ValueError):
        GNSSOutageRealization(onset_time_ns=1, return_time_ns=40_000_000_000).validate_for(spec)


def observation(tick):
    record = delivery(tick)
    return NavigationRecoveryObservation(episode_id='method-case', sequence=record.sequence,
        sample_time_ns=record.sample_time_ns, delivery_time_ns=record.delivery_time_ns,
        receiver_available=record.receiver_available, chaser_measurement_j2000=record.measurement_j2000,
        target_measurement_j2000=None)


def test_public_method_propagates_loss_without_truth_and_reset_discards_old_state():
    spec = task()
    assert method_input_types(spec) == (NavigationRecoveryContext, NavigationRecoveryObservation)
    context = NavigationRecoveryContext(task=spec, chaser_mass_kg=170.)
    method = method_classes(spec)[method_ids(spec)[0]]()
    method.reset(context)
    first = method.act(observation(0))
    for tick in range(25, 751, 25):
        obs = observation(tick)
        action = parse_action(spec, method.act(obs), obs)
        assert np.linalg.norm(np.array(action.estimated_state[:3])-physical(tick)['chaser_com_j2000'][:3]) < .1
    method.reset(context)
    assert method.act(observation(0)) == first
    assert set(context.model_dump()) == {'task', 'chaser_mass_kg'}
    assert 'outage' not in context.model_dump()['task']
    missing = observation(250)
    for extra in (dict(chaser_measurement_j2000=physical(250)['chaser_com_j2000']),
                  dict(receiver_available=True), dict(receiver_available=0),
                  dict(outage=OUTAGE.model_dump()), dict(chaser_truth_j2000=(0.,)*6), dict(delivery_time_ns=missing.sample_time_ns+1)):
        with pytest.raises(ValidationError):
            NavigationRecoveryObservation.model_validate(missing.model_dump() | extra)


def test_negative_control_holds_an_obsolete_state_through_loss():
    spec = task()
    method = method_classes(spec)['freeze-during-outage-v1']()
    method.reset(NavigationRecoveryContext(task=spec, chaser_mass_kg=170.))
    for tick in range(0, 250, 25):
        action = method.act(observation(tick))
    frozen = action.estimated_state
    for tick in range(250, 500, 25):
        assert method.act(observation(tick)).estimated_state == frozen
    assert method.act(observation(500)).estimated_state != frozen
