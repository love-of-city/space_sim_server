"""Far-to-proximity semantics and smooth public-GNSS guidance; native runs separate."""
import numpy as np
import pytest
from pydantic import ValidationError

from simulation.benchmark_relative_flight_methods import GNSSRelativeFlight
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, NavigationObservation, load_task
from space_arm_platform.benchmark_relative_flight import RelativeFlightContext, RelativeFlightTask
from space_arm_platform.benchmark_relative_flight_score import RelativeFlightScorer
from test_benchmark_relative_flight import physical


def task():
    return load_task(SPEC_DIRECTORY/'d05-cooperative-far-rendezvous-v1.json')


@pytest.mark.parametrize('kind', ['near_start','wide_start','far_goal','short_time','wrong_path','wrong_source','small_progress'])
def test_far_rendezvous_cannot_be_obtained_by_relabeling_a_near_waypoint(kind):
    document = task().model_dump()
    if kind == 'near_start': document['initial']['relative_position_tnr_m'] = [-250.,0.,0.]
    if kind == 'wide_start': document['initial']['position_half_width_m'] = 1300.
    if kind == 'far_goal': document['objective']['position_tnr_m'] = [-500.,0.,0.]
    if kind == 'short_time': document['duration_s'] = 100.
    if kind == 'wrong_path': document['path_requirement'] = {'kind':'full_duration_relative_hold'}
    if kind == 'wrong_source': document['source_id'] = 'D29'
    if kind == 'small_progress': document['path_requirement']['minimum_range_reduction_m'] = 10.
    with pytest.raises(ValidationError):
        RelativeFlightTask.model_validate(document)


def test_private_initial_evidence_rejects_prepositioned_success():
    spec = task()
    scorer = RelativeFlightScorer(spec,170.)
    scorer.add(physical(0,spec.objective.position_tnr_m))
    result = scorer.result()
    assert result['outcome'] == 'invalid_initialization'
    assert 'rendezvous_did_not_start_at_declared_far_range' in result['initialization_errors']


def test_far_transfer_does_not_impose_stationkeeping_or_axis_corridor():
    spec = task()
    scorer = RelativeFlightScorer(spec,170.)
    for tick in range(501):
        t = tick*.02
        position = np.array(spec.initial.relative_position_tnr_m)+np.array([.005*t*t,0.,0.])
        scorer.add(physical(tick,position,(.01*t,0.,0.)))
    result = scorer.result()
    assert result['outcome'] == 'incomplete'
    assert result['range_reduction_m'] > .49
    assert result['first_hold_violation_s'] is None
    assert result['maximum_cross_track_m'] is None
    assert not result['safety_violations'] and not result['initialization_errors']


def test_gnss_reference_starts_at_measured_far_state_and_reset_clears_it():
    spec = task()
    model = GNSSRelativeFlight()
    model.reset(RelativeFlightContext(task=spec,chaser_mass_kg=170.))
    first = physical(0,spec.initial.relative_position_tnr_m)
    observation = NavigationObservation(episode_id='far',sequence=0,sample_time_ns=0,delivery_time_ns=0,
        target_estimate_j2000=first['target_com_j2000'],chaser_estimate_j2000=first['chaser_com_j2000'])
    action = model.act(observation)
    # Jumping straight to the final 150 m setpoint would saturate the 25 N
    # actuator; the continuous reference begins at the measured initial state.
    assert np.linalg.norm(action.force_j2000_n) < 10
    assert np.allclose(model.initial_estimate,spec.initial.relative_position_tnr_m,atol=1e-6)
    model.reset(RelativeFlightContext(task=spec,chaser_mass_kg=170.))
    assert model.initial_estimate is None and model.act(observation) == action
