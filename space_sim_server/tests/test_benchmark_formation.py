"""Independent analytic geometry fixtures; real dynamics are tested separately."""
import math

import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_formation import FormationAction, FormationObservation, FormationTask
from space_arm_platform.benchmark_formation_score import FormationScorer
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS


def document():
    return load_task(SPEC_DIRECTORY/'m02-gnss-formation-hold-v1.json').model_dump()


def analytic_task():
    value = document()
    value['duration_s'] = value['objective']['dwell_s'] = 1.
    value['initial'].update(velocities_tnr_m_s=((0., 0., 0.),)*3,
                            position_half_width_m=0., velocity_half_width_m_s=0.)
    return FormationTask.model_validate(value)


def physical(tick, positions=None, velocities=None):
    radius, inc = RADIUS+500000., math.radians(51.6)
    rate = math.sqrt(MU/radius**3)
    phase = math.pi+rate*tick*.02
    radial = np.array([math.cos(phase), math.sin(phase)*math.cos(inc), math.sin(phase)*math.sin(inc)])
    tangent = np.array([-math.sin(phase), math.cos(phase)*math.cos(inc), math.cos(phase)*math.sin(inc)])
    normal = np.cross(radial, tangent)
    axes = np.stack((tangent, normal, radial), axis=1)
    reference = np.r_[radius*radial, radius*rate*tangent]
    points = np.asarray(positions if positions is not None else ((0, 0, 0), (-150, 0, 0), (75, 0, 130)))
    speeds = np.asarray(velocities if velocities is not None else ((0, 0, 0),)*3)
    offsets = points @ axes.T
    states = np.column_stack((offsets, speeds @ axes.T+np.cross(rate*normal, offsets)))+reference
    return dict(time_ns=tick*20_000_000, com_states_j2000=states.tolist(),
                applied_motor_forces_body_n=((0., 0., 0.),)*3, external_state_writes=0)


@pytest.mark.parametrize('mutation', ['source', 'count', 'uncontrolled', 'missing', 'origin', 'pair_collision',
                                     'initial_spread', 'short_hold', 'vacuous_reconfigure', 'fixed_establish'])
def test_contract_rejects_mislabeled_or_vacuous_formation(mutation):
    value = document()
    if mutation == 'source': value['source_id'] = 'D15'
    if mutation == 'count': value['spacecraft_count'] = True
    if mutation == 'uncontrolled': value['controlled_spacecraft'] = 2
    if mutation == 'missing': value['objective']['positions_tnr_m'] = ((0, 0, 0), (-150, 0, 0))
    if mutation == 'origin': value['initial']['positions_tnr_m'] = ((1, 0, 0), (-150, 0, 0), (75, 0, 130))
    if mutation == 'pair_collision': value['objective']['positions_tnr_m'] = ((0, 0, 0), (-150, 0, 0), (-150, 20, 0))
    if mutation == 'initial_spread': value['initial']['position_half_width_m'] = 1.
    if mutation == 'short_hold': value['objective']['dwell_s'] = 20.
    if mutation == 'vacuous_reconfigure':
        value = load_task(SPEC_DIRECTORY/'m03-gnss-formation-reconfigure-v1.json').model_dump()
        value['objective']['minimum_displacement_m'] = 0.
    if mutation == 'fixed_establish':
        value = load_task(SPEC_DIRECTORY/'m01-gnss-formation-establish-v1.json').model_dump()
        value['initial']['position_half_width_m'] = .1
    with pytest.raises(ValidationError): FormationTask.model_validate(value)


def test_every_member_must_remain_in_formation_for_entire_duration():
    scorer = FormationScorer(analytic_task(), [170.]*3)
    for tick in range(51):
        scorer.add(physical(tick))
        assert scorer.result()['success'] == (tick == 50)
    assert scorer.result()['longest_dwell_s'] == 1.
    assert scorer.result()['minimum_pair_distance_m'] == pytest.approx(150.)


@pytest.mark.parametrize('defect', ['position', 'velocity'])
def test_single_member_brief_departure_cannot_be_averaged_away_or_erased(defect):
    scorer = FormationScorer(analytic_task(), [170.]*3)
    for tick in range(51):
        positions = ((0, 0, 0), (-150, 0, 0), (75, 1., 130)) if tick == 20 and defect == 'position' else None
        speeds = ((0, 0, 0), (0, 0, 0), (0, .06, 0)) if tick == 20 and defect == 'velocity' else None
        scorer.add(physical(tick, positions, speeds))
    score = scorer.result()
    assert score['outcome'] == 'task_failure'
    assert score['quality_violations'] == ['continuous_formation_envelope']
    assert score['first_hold_violation_s'] == .4
    assert score['formation_position_error_m'] < 1e-8


def test_follower_to_follower_collision_is_vetoed_even_with_safe_leader_distances():
    scorer = FormationScorer(analytic_task(), [170.]*3)
    scorer.add(physical(0))
    scorer.add(physical(1, ((0, 0, 0), (-150, 0, 0), (-150, 20, 0))))
    score = scorer.result()
    assert score['outcome'] == 'safety_violation'
    assert score['safety_violations'] == ['minimum_pair_distance']
    assert score['minimum_pair_distance_m'] == pytest.approx(20.)


def test_force_limits_and_impulse_apply_to_every_individual_actuator():
    scorer = FormationScorer(analytic_task(), [100., 200., 300.])
    scorer.add(physical(0))
    sample = physical(1)
    sample['applied_motor_forces_body_n'] = ((1., 0, 0), (0, 2., 0), (0, 0, 26.))
    scorer.add(sample)
    score = scorer.result()
    assert score['outcome'] == 'safety_violation' and score['safety_violations'] == ['applied_force']
    assert score['per_spacecraft_thrust_impulse_n_s'] == pytest.approx([.02, .04, .52])
    assert score['per_spacecraft_delta_v_m_s'] == pytest.approx([.0002, .0002, .52/300])


@pytest.mark.parametrize('defect', ['gap', 'duplicate', 'overwrite', 'missing_member', 'initial'])
def test_incomplete_or_inconsistent_physical_evidence_is_not_success(defect):
    scorer = FormationScorer(analytic_task(), [170.]*3)
    first = physical(0, ((0, 0, 0), (-150, 0, 0), (75, .1, 130)) if defect == 'initial' else None)
    scorer.add(first)
    sample = physical(2 if defect == 'gap' else 0 if defect == 'duplicate' else 1)
    if defect == 'overwrite': sample['external_state_writes'] = 1
    if defect == 'missing_member':
        sample['com_states_j2000'].pop()
        with pytest.raises(ValueError): scorer.add(sample)
    else:
        scorer.add(sample)
    assert scorer.result()['outcome'] == ('invalid_initialization' if defect == 'initial' else 'invalid_evidence')


@pytest.mark.parametrize('defect', ['stale', 'previous_episode', 'missing_member', 'force_limit', 'observation_count'])
def test_joint_action_requires_current_episode_and_one_bounded_force_per_member(defect):
    observation = FormationObservation(episode_id='current', sequence=1, sample_time_ns=500000000,
        delivery_time_ns=500000000, estimates_j2000=physical(0)['com_states_j2000'])
    action = FormationAction(episode_id='previous' if defect == 'previous_episode' else 'current',
        observation_sequence=0 if defect == 'stale' else 1,
        forces_j2000_n=((0, 0, 26.),)*3 if defect == 'force_limit' else ((0, 0, 0),)*(2 if defect == 'missing_member' else 3))
    if defect == 'observation_count': observation = observation.model_copy(update={'estimates_j2000': observation.estimates_j2000[:2]})
    with pytest.raises(ValueError): action.validate_for(observation, analytic_task())
