"""Ordered physical dwell, skipped targets, hard safety and public-only switching."""
import math

import pytest

from space_arm_platform.benchmark_attitude import AttitudeContext, AttitudeObservation, AttitudeTask
from space_arm_platform.benchmark_attitude_score import AttitudeScorer
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_registry import method_ids
from simulation.benchmark_attitude_methods import SequenceAttitudePD


@pytest.fixture
def task():
    data = load_task(SPEC_DIRECTORY/'s04-sequential-wheel-slew-v1.json').model_dump(mode='json')
    data['objective']['dwell_s'] = .1
    data['duration_s'] = 1.
    return AttitudeTask.model_validate(data)


def sample(tick, quaternion, **changes):
    return dict(time_ns=tick*20_000_000, orientation_body_to_j2000_wxyz=quaternion,
        angular_velocity_body_rad_s=changes.get('rate', (0., 0., 0.)),
        wheel_speed_rad_s=(0., 0., 0.), applied_motor_torque_xyz_nm=(0., 0., 0.),
        external_state_writes=0)


def start(task):
    scorer = AttitudeScorer(task)
    scorer.add(sample(0, (math.cos(math.pi/18), math.sin(math.pi/18), 0., 0.)))
    return scorer


@pytest.mark.parametrize('defect', ['missing', 'duplicate', 'sign_duplicate', 'nonunit', 'wrong_source', 'short_deadline'])
def test_sequence_contract_requires_distinct_ordered_targets(task, defect):
    data = task.model_dump(mode='json')
    targets = data['objective']['additional_orientations_body_to_j2000_wxyz']
    if defect == 'missing':
        del data['objective']['additional_orientations_body_to_j2000_wxyz']
    if defect == 'duplicate':
        targets[1] = targets[0]
    if defect == 'sign_duplicate':
        targets[1] = [-v for v in targets[0]]
    if defect == 'nonunit':
        targets[1] = [2., 0., 0., 0.]
    if defect == 'wrong_source':
        data.update(source_id='S02', requirement_id='ssb.attitude.slew-hold')
    if defect == 'short_deadline':
        data['duration_s'] = .2
    with pytest.raises(ValueError):
        AttitudeTask.model_validate(data)


def test_every_target_requires_separate_continuous_physical_dwell(task):
    scorer = start(task)
    for index, quaternion in enumerate(scorer.targets):
        for tick in range(index*6+1, index*6+7):
            scorer.add(sample(tick, quaternion))
        score = scorer.result()
        assert len(score['completed_targets']) == index+1
        assert score['success'] == (index == 2)
        if index < 2:
            assert score['final_attitude_error_rad'] > task.objective.angle_tolerance_rad
    assert [entry['completed_at_s'] for entry in score['completed_targets']] == pytest.approx([.12, .24, .36])
    assert all(entry['continuous_dwell_s'] == .1 for entry in score['completed_targets'])
    assert method_ids(task)[0] == 'attitude-sequence-pd-v1'


@pytest.mark.parametrize('destination', ['first', 'last'])
def test_holding_one_target_cannot_erase_unvisited_targets(task, destination):
    scorer = start(task)
    for tick in range(1, 51):
        scorer.add(sample(tick, scorer.targets[0 if destination == 'first' else -1]))
    score = scorer.result()
    assert score['outcome'] == 'time_limit'
    assert len(score['completed_targets']) == (1 if destination == 'first' else 0)


def test_rate_excursion_restarts_current_dwell_and_hard_violation_vetoes_all_targets(task):
    scorer = start(task)
    for tick in range(1, 5):
        scorer.add(sample(tick, scorer.targets[0]))
    scorer.add(sample(5, scorer.targets[0], rate=(.16, 0., 0.)))
    for index, quaternion in enumerate(scorer.targets):
        for tick in range(index*6+6, index*6+12):
            scorer.add(sample(tick, quaternion))
    score = scorer.result()
    assert len(score['completed_targets']) == 3
    assert score['completed_targets'][0]['hold_start_s'] == .12
    assert score['outcome'] == 'safety_violation' and score['completion_time_s'] is None


def test_sequence_initialization_and_missing_physics_are_rejected(task):
    invalid = AttitudeScorer(task)
    invalid.add(sample(0, (1., 0., 0., 0.)))
    assert 'initial_state_outside_attitude_sequence_distribution' in invalid.result()['evidence_errors']
    scorer = start(task)
    scorer.add(sample(2, scorer.targets[0]))
    assert scorer.result()['outcome'] == 'invalid_evidence'


@pytest.mark.parametrize('interruption', ['invalid_gyro', 'missing_observation'])
def test_method_observation_hold_restarts_and_reset_forgets_sequence_progress(task, interruption):
    context = AttitudeContext(task=task, chaser_mass_kg=100.,
        inertia_com_body_kg_m2=((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
        wheel_spin_inertia_kg_m2=(.01, .01, .01), hardware_torque_limits_nm=(.2, .2, .2),
        hardware_speed_limits_rad_s=(628., 628., 628.))
    method = SequenceAttitudePD()
    method.reset(context)
    def observation(tick, valid=True):
        return AttitudeObservation(episode_id='sequence', sequence=tick, sample_time_ns=tick*20_000_000,
            delivery_time_ns=tick*20_000_000, orientation_body_to_j2000_wxyz=(1., 0., 0., 0.),
            angular_velocity_body_rad_s=(0., 0., 0.), wheel_speed_rad_s=(0., 0., 0.), gyro_valid=valid)
    for tick in range(21):
        method.act(observation(tick))
    if interruption == 'invalid_gyro':
        assert method.act(observation(21, False)).motor_torque_xyz_nm == (0., 0., 0.)
    for tick in range(22, 52):
        method.act(observation(tick))
    assert method.index == 0
    assert any(method.act(observation(52)).motor_torque_xyz_nm)
    assert method.index == 1
    method.reset(context)
    assert method.index == 0 and method.hold_start_ns is None and method.last_time_ns is None
