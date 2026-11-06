"""Continuous native-force coverage is distinct from an equal pulsed impulse."""
from copy import deepcopy

import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, load_task
from space_arm_platform.benchmark_orbit import OrbitContext, OrbitObservation, OrbitTask
from space_arm_platform.benchmark_orbit_score import OrbitScorer
from space_arm_platform.benchmark_registry import method_classes, method_ids, parse_action
from test_benchmark_orbit import circular, sample


def task():
    return load_task(SPEC_DIRECTORY/'s10-continuous-low-thrust-v1.json')


def fixture_task():
    document = task().model_dump()
    document['duration_s'] = 340.
    document['objective'].update(delta_velocity_j2000_m_s=(0.,-.6,0.),earliest_burn_s=0.,latest_burn_s=320.,
        minimum_continuous_thrust_s=300.)
    return OrbitTask.model_validate(document)


@pytest.mark.parametrize('pulsed,energy_increase,expected', [(False,True,'success'),
    (True,True,'continuous_low_thrust_duration'), (False,False,'low_thrust_orbital_energy_increase')])
def test_analytic_grading_requires_continuity_and_real_energy_change_in_addition_to_impulse(pulsed,energy_increase,expected):
    # Isolated grading fixtures, not claimed native trajectories. Native validation
    # separately integrates the real motor force and checks its orbital effects.
    scorer = OrbitScorer(fixture_task(),100.)
    for tick in range(17001):
        active = 0 < tick <= 15000
        force = .25 if pulsed and (tick-1) % 250 < 200 else 0. if pulsed else .2
        row = sample(tick,altitude=501200. if energy_increase and tick else 500000.,
            force=(0.,-force,0.) if active else (0.,0.,0.))
        scorer.add(row)
        if tick in (0,1,14999,15000):
            assert scorer.result()['outcome'] == 'incomplete'
    result = scorer.result()
    assert result['final_orbit']['impulse_vector_error_m_s'] < 1e-10
    assert result['thrust_delta_v_m_s'] == pytest.approx(.6)
    assert result['longest_continuous_thrust_s'] == (4. if pulsed else 300.)
    if expected == 'success':
        assert result['outcome'] == 'success' and result['quality_violations'] == []
    else:
        assert result['outcome'] == 'task_failure' and result['quality_violations'] == [expected]


@pytest.mark.parametrize('kind', ['first_frame','gap','duplicate','force_bound','state_write'])
def test_continuous_force_evidence_cannot_count_first_frame_or_discontinuity(kind):
    scorer = OrbitScorer(task(),100.)
    scorer.add(sample(0,force=(.2,0.,0.)))
    assert scorer.result()['longest_continuous_thrust_s'] == 0
    if kind == 'first_frame':
        return
    # Early burning is independently safety-vetoed, but its timing counter must
    # also discard gaps and duplicates, rather than accumulating fake duration.
    scorer.add(sample(1,force=(.2,0.,0.)))
    row = sample(3 if kind == 'gap' else 1 if kind == 'duplicate' else 2,
        force=(.3 if kind == 'force_bound' else .2,0.,0.))
    if kind == 'state_write':
        row['external_state_writes'] = 1
    scorer.add(row)
    result = scorer.result()
    if kind in {'gap','duplicate'}:
        assert result['longest_continuous_thrust_s'] == .02 and result['outcome'] == 'invalid_evidence'
    elif kind == 'state_write':
        assert result['outcome'] == 'invalid_evidence'
    else:
        assert result['outcome'] == 'safety_violation' and 'applied_force' in result['safety_violations']


@pytest.mark.parametrize('path,value', [
    (('safety','maximum_force_n'),1.01), (('objective','minimum_active_force_n'),.25),
    (('objective','minimum_active_force_n'),.001), (('objective','minimum_continuous_thrust_s'),299.),
    (('objective','minimum_continuous_thrust_s'),800.), (('objective','minimum_continuous_thrust_s'),600.01),
    (('objective','minimum_semimajor_axis_increase_m'),100000.),
])
def test_low_thrust_contract_has_a_feasible_continuous_window_and_bounded_actuator(path,value):
    document = deepcopy(task().model_dump())
    document[path[0]][path[1]] = value
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(document)


def test_long_low_thrust_cannot_be_relabelled_as_the_short_burn_requirement():
    document = task().model_dump() | dict(source_id='S09',requirement_id='ssb.orbit.maneuver')
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(document)


def observation(sequence):
    return OrbitObservation(episode_id='low-thrust-method',sequence=sequence,
        sample_time_ns=sequence*500_000_000,delivery_time_ns=sequence*500_000_000,
        chaser_estimate_j2000=circular(500000.))


def test_continuous_and_pulsed_methods_use_the_same_public_contract_and_reset():
    spec = task()
    assert 'pulsed-orbit-burn-v1' in method_ids(spec)
    assert 'pulsed-orbit-burn-v1' not in method_ids(load_task(SPEC_DIRECTORY/'s11-orbit-raise-v1.json'))
    context = OrbitContext(task=spec,chaser_mass_kg=169.)
    for name in ('gnss-orbit-maneuver-v1','pulsed-orbit-burn-v1'):
        method = method_classes(spec)[name]()
        method.reset(context)
        actions = [parse_action(spec,method.act(observation(tick)),observation(tick)) for tick in range(30)]
        magnitude = [np.linalg.norm(action.force_j2000_n) for action in actions]
        assert max(magnitude[:10]) == 0
        if name == 'gnss-orbit-maneuver-v1':
            assert magnitude[10:] == pytest.approx([.2]*20)
        else:
            assert magnitude[10:18] == pytest.approx([.25]*8)
            assert magnitude[18:20] == [0.,0.]
            assert magnitude[20:28] == pytest.approx([.25]*8)
        method.reset(context)
        assert method.act(observation(10)).force_j2000_n == actions[10].force_j2000_n
    with pytest.raises(ValueError):
        parse_action(spec,ForceAction(episode_id='low-thrust-method',observation_sequence=10,force_j2000_n=(.3,0.,0.)),observation(10))
