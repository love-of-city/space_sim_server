"""Analytic risk/reversal fixtures; these do not claim propagated native success."""
from types import SimpleNamespace

import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, NavigationObservation, load_task
from space_arm_platform.benchmark_relative_flight import RelativeFlightTask
from space_arm_platform.benchmark_relative_flight_score import RelativeFlightScorer
from simulation.benchmark_relative_flight_methods import GNSSRelativeFlight
from test_benchmark_relative_flight import physical


def task():
    return load_task(SPEC_DIRECTORY/'d40-gnss-proximity-abort-v1.json')


@pytest.mark.parametrize('defect', ['outward', 'no_risk', 'already_at_goal', 'deadline', 'confirmation', 'wrong_path', 'no_retreat'])
def test_abort_requires_real_initial_closing_risk_and_a_distinct_retreat(defect):
    value = task().model_dump()
    if defect == 'outward':
        value['initial']['relative_velocity_tnr_m_s'] = (-.5, 0., 0.)
    elif defect == 'no_risk':
        value['path_requirement']['prediction_horizon_s'] = 60.
    elif defect == 'already_at_goal':
        value['initial']['relative_position_tnr_m'] = (-250., 0., 0.)
    elif defect == 'deadline':
        value['path_requirement']['reversal_deadline_s'] = 120.
    elif defect == 'confirmation':
        value['path_requirement']['reversal_confirmation_s'] = 41.
    elif defect == 'wrong_path':
        value['path_requirement'] = {'kind': 'full_duration_relative_hold'}
    else:
        value['objective']['position_tnr_m'] = (-160., 0., 0.)
    with pytest.raises(ValidationError):
        RelativeFlightTask.model_validate(value)


def short_grader_task():
    value = task().model_dump()
    value['initial'].update(position_half_width_m=0., velocity_half_width_m_s=0.)
    value['objective'].update(position_tnr_m=(-151., 0., 0.), position_tolerance_m=.02, dwell_s=.5)
    value['duration_s'] = 20.
    value['path_requirement'].update(reversal_confirmation_s=.5, reversal_deadline_s=4., minimum_retreat_distance_m=.5)
    return RelativeFlightTask.model_validate(value)


def trajectory(time, delay=0.):
    # Coast inward for delay, reverse with -0.5 m/s² for 2 s, coast outward
    # for delay, then decelerate for 4 s. Both versions end at -151 m.
    if time < delay:
        return -150.+.5*time, .5
    time -= delay
    start = -150.+.5*delay
    if time < 2.:
        return start+.5*time-.25*time*time, .5-.5*time
    time -= 2.
    if time < delay:
        return start-.5*time, -.5
    time -= delay
    if time < 4.:
        return -150.-.5*time+.0625*time*time, -.5+.125*time
    return -151., 0.


def grade(delay=0., defect=None):
    spec = short_grader_task()
    scorer = RelativeFlightScorer(spec, 170.)
    for tick in range(1001):
        position, velocity = trajectory(tick*.02, delay)
        row = physical(tick, (position, 0., 0.), (velocity, 0., 0.))
        if defect == 'gap' and tick == 70:
            continue
        if defect == 'state_write' and tick == 80:
            row['external_state_writes'] = 1
        if defect == 'safety' and tick == 200:
            row['applied_motor_force_body_n'] = (6., 0., 0.)
        scorer.add(row)
    return scorer.result()


def test_success_requires_continuous_outward_confirmation_then_actual_retreat_dwell():
    score = grade()
    assert score['success'] and score['initial_closing_speed_m_s'] == pytest.approx(.5)
    assert score['reversal_confirmed_s']-score['reversal_started_s'] == pytest.approx(.5)
    assert 1.5 <= score['reversal_confirmed_s'] < 1.7
    assert score['retreat_progress_m'] == pytest.approx(1., abs=1e-8)
    assert score['completion_time_s'] > score['reversal_confirmed_s']
    assert score['longest_dwell_s'] >= .5


def test_late_abort_cannot_be_repaired_by_eventually_reaching_the_correct_hold_point():
    score = grade(delay=5.)
    assert score['outcome'] == 'task_failure'
    assert score['reversal_confirmed_s'] > score['reversal_deadline_s']
    assert score['final_position_error_m'] < 1e-8
    assert score['quality_violations'] == ['abort_reversal_deadline']
    assert score['completion_time_s'] is None


@pytest.mark.parametrize('defect,outcome', [('gap', 'invalid_evidence'), ('state_write', 'invalid_evidence'), ('safety', 'safety_violation')])
def test_retreat_does_not_override_invalid_evidence_or_motor_safety(defect, outcome):
    assert grade(defect=defect)['outcome'] == outcome


def test_already_retreating_initialization_is_not_an_abort_success():
    spec = short_grader_task()
    scorer = RelativeFlightScorer(spec, 170.)
    scorer.add(physical(0, (-150., 0., 0.), (-.5, 0., 0.)))
    assert scorer.result()['outcome'] == 'invalid_initialization'
    assert 'abort_started_without_required_closing_motion' in scorer.result()['initialization_errors']


def test_outward_confirmation_cannot_accumulate_across_separate_short_intervals():
    spec = short_grader_task()
    scorer = RelativeFlightScorer(spec, 170.)
    # Deliberately alternate authoritative velocity samples to test the timing
    # rule, not as a claimed physically propagated trajectory.
    for tick in range(201):
        velocity = .5 if tick == 0 else (-.1 if tick % 20 < 10 else .1)
        scorer.add(physical(tick, (-150., 0., 0.), (velocity, 0., 0.)))
    result = scorer.result()
    assert result['reversal_confirmed_s'] is None
    assert result['quality_violations'] == ['abort_reversal_deadline']


def observation(sequence, position, velocity):
    raw = physical(sequence*25, (position, 0., 0.), (velocity, 0., 0.))
    return NavigationObservation(episode_id='public-gnss', sequence=sequence,
        sample_time_ns=sequence*500_000_000, delivery_time_ns=sequence*500_000_000,
        chaser_estimate_j2000=raw['chaser_com_j2000'], target_estimate_j2000=raw['target_com_j2000'])


def test_public_gnss_risk_triggers_braking_latches_retreat_and_resets_between_episodes():
    method = GNSSRelativeFlight()
    context = SimpleNamespace(task=task(), chaser_mass_kg=170.)
    method.reset(context)
    safe = observation(0, -150., -.1)
    assert method.act(safe).force_j2000_n == (0., 0., 0.)
    closing = observation(1, -150., .5)
    action = method.act(closing)
    relative = np.asarray(closing.chaser_estimate_j2000[:3])-closing.target_estimate_j2000[:3]
    assert np.asarray(action.force_j2000_n)@relative > 0
    assert np.linalg.norm(action.force_j2000_n) > 0
    recovering = observation(2, -147., -.05)
    assert np.linalg.norm(method.act(recovering).force_j2000_n) > 0
    method.reset(context)
    assert method.act(recovering).force_j2000_n == (0., 0., 0.)
