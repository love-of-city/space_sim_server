"""D19 requires range closure and simultaneous actual actuation of both craft."""
import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_formation import FormationContext, FormationObservation, FormationTask
from space_arm_platform.benchmark_formation_score import FormationScorer
from test_benchmark_formation import physical


def document():
    return load_task(SPEC_DIRECTORY/'d19-gnss-cooperative-rendezvous-v1.json').model_dump()


@pytest.mark.parametrize('defect', ['one_controlled', 'no_closure', 'near_start', 'far_end',
    'impossible_progress', 'no_activity', 'excess_impulse', 'off_grid', 'wrong_source', 'not_enough_time'])
def test_cooperation_contract_requires_a_feasible_actual_two_craft_transfer(defect):
    value = document()
    if defect == 'one_controlled': value['controlled_spacecraft'] = 1
    if defect == 'no_closure': value['objective']['minimum_range_reduction_m'] = 0
    if defect == 'near_start': value['initial']['positions_tnr_m'] = ((0, 0, 0), (-450, 0, 0))
    if defect == 'far_end': value['objective']['positions_tnr_m'] = ((0, 0, 0), (-250, 0, 0))
    if defect == 'impossible_progress': value['objective']['minimum_range_reduction_m'] = 500
    if defect == 'no_activity': value['objective']['active_force_threshold_n'] = 5
    if defect == 'excess_impulse': value['objective']['minimum_per_spacecraft_impulse_n_s'] = 2501
    if defect == 'off_grid': value['objective']['simultaneous_maneuver_dwell_s'] = 1.001
    if defect == 'wrong_source': value['source_id'] = 'D14'
    if defect == 'not_enough_time': value['duration_s'] = 100
    with pytest.raises(ValidationError):
        FormationTask.model_validate(value)


def analytic_task():
    # Short kinematic scorer fixture; it is never native flight evidence.
    value = document()
    value['duration_s'] = 14.
    value['initial'].update(position_half_width_m=0., velocity_half_width_m_s=0.)
    value['safety']['maximum_pair_speed_m_s'] = 100.
    value['objective'].update(dwell_s=1., simultaneous_maneuver_dwell_s=2., minimum_per_spacecraft_impulse_n_s=.1)
    return FormationTask.model_validate(value)


def fixture(tick, defect=None):
    u = min(1., tick*.02/10.)
    progress = 10*u**3-15*u**4+6*u**5
    speed = 450*(30*u**2-60*u**3+30*u**4)/10.
    sample = physical(tick, ((0, 0, 0), (-600+450*progress, 0, 0)), ((0, 0, 0), (speed, 0, 0)))
    forces = ((1., 0, 0), (-1., 0, 0))
    if defect == 'one_passive': forces = ((0., 0, 0), (-1., 0, 0))
    if defect == 'alternating':
        forces = ((1., 0, 0), (0., 0, 0)) if tick % 2 else ((0., 0, 0), (-1., 0, 0))
    if defect == 'brief_overlap' and tick >= 50: forces = ((0., 0, 0), (-1., 0, 0))
    sample['applied_motor_forces_body_n'] = forces if tick else ((0., 0, 0),)*2
    return sample


@pytest.mark.parametrize('defect', [None, 'one_passive', 'alternating', 'brief_overlap'])
def test_identical_arrival_does_not_replace_actual_joint_maneuver(defect):
    scorer = FormationScorer(analytic_task(), [170., 170.])
    for tick in range(701):
        scorer.add(fixture(tick, defect))
    result = scorer.result()
    assert result['formation_position_error_m'] < 1e-8
    assert result['longest_dwell_s'] > 1
    assert result['cooperative_range_reduction_m'] == pytest.approx(450)
    assert result['success'] == (defect is None)
    if defect:
        assert result['outcome'] == 'task_failure'
        assert 'simultaneous_cooperative_maneuver' in result['quality_violations']
    if defect == 'one_passive':
        assert 'both_spacecraft_active_impulse' in result['quality_violations']
        assert result['per_spacecraft_thrust_impulse_n_s'][0] == 0
    if defect == 'alternating':
        assert min(result['per_spacecraft_thrust_impulse_n_s']) > .1
        assert result['simultaneous_maneuver_s'] == 0
    if defect == 'brief_overlap':
        assert result['simultaneous_maneuver_s'] == pytest.approx(.98)


def test_gap_and_prepositioned_arrival_cannot_supply_cooperative_evidence():
    scorer = FormationScorer(analytic_task(), [170., 170.])
    first = fixture(500)
    first['time_ns'] = 0
    scorer.add(first)
    assert scorer.result()['outcome'] == 'invalid_initialization'
    scorer = FormationScorer(analytic_task(), [170., 170.])
    scorer.add(fixture(0))
    scorer.add(fixture(2))
    assert scorer.result()['outcome'] == 'invalid_evidence'
    assert scorer.result()['simultaneous_maneuver_s'] == 0
    assert scorer.result()['per_spacecraft_thrust_impulse_n_s'] == [0., 0.]


def test_method_shares_force_using_only_public_independent_gnss_and_resets():
    from simulation.benchmark_formation_methods import GNSSFormation, OneSidedFormation
    task = load_task(SPEC_DIRECTORY/'d19-gnss-cooperative-rendezvous-v1.json')
    context = FormationContext(task=task, spacecraft_masses_kg=(170., 180.))
    public = FormationObservation(episode_id='test', sequence=0, sample_time_ns=0, delivery_time_ns=0,
        estimates_j2000=fixture(0)['com_states_j2000'])
    method = GNSSFormation()
    method.reset(context)
    first = method.act(public)
    first.validate_for(public, task)
    assert np.allclose(np.sum(first.forces_j2000_n, axis=0), 0., atol=1e-12)
    assert min(np.linalg.norm(first.forces_j2000_n, axis=1)) > 0
    method.reset(context)
    assert method.act(public) == first
    one_sided = OneSidedFormation()
    one_sided.reset(context)
    control = one_sided.act(public)
    assert control.forces_j2000_n[0] == (0., 0., 0.)
    assert np.linalg.norm(control.forces_j2000_n[1]) > 0
    assert set(public.model_dump()) == {'schema_version', 'episode_id', 'sequence',
        'sample_time_ns', 'delivery_time_ns', 'estimates_j2000'}
