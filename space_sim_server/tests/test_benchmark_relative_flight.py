"""Analytic grading fixtures; actual integrated acceptance is separate."""
import math

import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS
from space_arm_platform.benchmark_relative_flight import RelativeFlightTask
from space_arm_platform.benchmark_relative_flight_score import RelativeFlightScorer

NAMES = {'D12': 'd12-continuous-gnss-hold-v1', 'D29': 'd29-vbar-gnss-approach-v1', 'D30': 'd30-rbar-gnss-approach-v1'}


def document(source):
    return load_task(SPEC_DIRECTORY/(NAMES[source]+'.json')).model_dump()


def short_task(source):
    value = document(source)
    value['initial'].update(position_half_width_m=0., velocity_half_width_m_s=0., relative_velocity_tnr_m_s=(0., 0., 0.))
    if source == 'D12':
        value['duration_s'] = value['objective']['dwell_s'] = 1.
    else:
        index = 0 if source == 'D29' else 2
        initial = [0., 0., 0.]
        initial[index] = -126.
        value['initial']['relative_position_tnr_m'] = initial
        value['objective'].update(position_tolerance_m=.02, dwell_s=.2)
        value['duration_s'] = 12.
        value['path_requirement']['minimum_progress_m'] = .8
    return RelativeFlightTask.model_validate(value)


def physical(tick, position, velocity=(0., 0., 0.)):
    radius, inc = RADIUS+500000., math.radians(51.6)
    n = math.sqrt(MU/radius**3)
    phase = math.pi+n*tick*.02
    radial = np.array([math.cos(phase), math.sin(phase)*math.cos(inc), math.sin(phase)*math.sin(inc)])
    tangent = np.array([-math.sin(phase), math.cos(phase)*math.cos(inc), math.cos(phase)*math.sin(inc)])
    normal = np.cross(radial, tangent)
    frame = np.stack((tangent, normal, radial), axis=1)
    target = np.r_[radius*radial, radius*n*tangent]
    displacement = frame @ position
    derivative = frame @ velocity+np.cross(n*normal, displacement)
    return dict(time_ns=tick*20_000_000, target_com_j2000=tuple(target),
        chaser_com_j2000=tuple(target+np.r_[displacement, derivative]),
        applied_motor_force_body_n=(0., 0., 0.), external_state_writes=0)


@pytest.mark.parametrize('source,mutation', [
    ('D12', 'short_hold'), ('D12', 'outside_hold'), ('D12', 'axis_requirement'),
    ('D29', 'wrong_axis'), ('D30', 'wrong_axis'), ('D29', 'inside_arrival'),
    ('D29', 'wrong_requirement'), ('D30', 'outside_corridor'), ('D30', 'off_axis_endpoint'),
])
def test_distinct_semantics_cannot_be_obtained_by_renaming_a_waypoint(source, mutation):
    value = document(source)
    if mutation == 'short_hold': value['objective']['dwell_s'] = 20.
    if mutation == 'outside_hold': value['initial']['position_half_width_m'] = 1.
    if mutation == 'axis_requirement': value['path_requirement'] = document('D29')['path_requirement']
    if mutation == 'wrong_axis': value['path_requirement']['axis'] = 'R' if source == 'D29' else 'T'
    if mutation == 'inside_arrival': value['initial']['relative_position_tnr_m'] = value['objective']['position_tnr_m']
    if mutation == 'wrong_requirement': value['requirement_id'] = 'ssb.approach.rbar'
    if mutation == 'outside_corridor': value['initial']['position_half_width_m'] = 10.
    if mutation == 'off_axis_endpoint': value['objective']['position_tnr_m'] = (-1., 0., -125.)
    with pytest.raises(ValidationError):
        RelativeFlightTask.model_validate(value)


def test_hold_requires_every_sample_over_the_full_duration():
    task = short_task('D12')
    score = RelativeFlightScorer(task, 170.)
    for tick in range(51):
        score.add(physical(tick, task.objective.position_tnr_m))
        assert score.result()['outcome'] == ('success' if tick == 50 else 'incomplete')
    assert score.result()['longest_dwell_s'] == 1.
    assert score.result()['sample_count'] == 51


def test_returning_to_the_hold_point_cannot_erase_an_earlier_excursion():
    task = short_task('D12')
    score = RelativeFlightScorer(task, 170.)
    for tick in range(51):
        position = list(task.objective.position_tnr_m)
        if tick == 25: position[1] = 1.
        score.add(physical(tick, position))
    result = score.result()
    assert result['outcome'] == 'task_failure'
    assert result['final_position_error_m'] < 1e-8
    assert result['quality_violations'] == ['continuous_hold_envelope']
    assert result['first_hold_violation_s'] == .5


@pytest.mark.parametrize('source', ['D29', 'D30'])
@pytest.mark.parametrize('excursion', [False, True])
def test_approach_must_remain_in_its_axis_corridor_even_if_arrival_is_accurate(source, excursion):
    task = short_task(source)
    score = RelativeFlightScorer(task, 170.)
    index = 0 if source == 'D29' else 2
    for tick in range(601):
        u = min(1., tick*.02/10.)
        position, velocity = np.zeros(3), np.zeros(3)
        position[index] = -126.+10*u**3-15*u**4+6*u**5
        velocity[index] = (30*u**2-60*u**3+30*u**4)/10.
        if excursion and tick == 250: position[1] = 8.5
        score.add(physical(tick, position, velocity))
    result = score.result()
    assert result['outcome'] == ('safety_violation' if excursion else 'success')
    assert result['approach_progress_m'] == pytest.approx(1., abs=1e-8)
    assert result['final_position_error_m'] < 1e-8
    assert ('approach_corridor' in result['safety_violations']) == excursion


def test_initializing_at_arrival_cannot_pass_an_approach_task():
    task = short_task('D29')
    score = RelativeFlightScorer(task, 170.)
    for tick in range(601):
        score.add(physical(tick, task.objective.position_tnr_m))
    assert score.result()['outcome'] == 'invalid_initialization'
    assert score.result()['completion_time_s'] is None


@pytest.mark.parametrize('corruption', ['gap', 'duplicate', 'state_write'])
def test_hold_cannot_accumulate_invalid_physical_time(corruption):
    task = short_task('D12')
    score = RelativeFlightScorer(task, 170.)
    for tick in range(51):
        if corruption == 'gap' and tick == 25: continue
        row = physical(tick, task.objective.position_tnr_m)
        if corruption == 'state_write' and tick == 25: row['external_state_writes'] = 1
        score.add(row)
        if corruption == 'duplicate' and tick == 25: score.add(row)
    assert score.result()['outcome'] == 'invalid_evidence'
