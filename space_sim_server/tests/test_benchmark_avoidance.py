"""Conjunction semantics, public-input methods and independent physical scoring."""
import math
import numpy as np
import pytest
from pydantic import ValidationError

from simulation.benchmark_avoidance_methods import CatalogAvoidance
from space_arm_platform.benchmark_avoidance import AvoidanceContext, AvoidanceObservation, AvoidanceTask
from space_arm_platform.benchmark_avoidance_score import AvoidanceScorer, tnr_frame
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, load_task
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS
from space_arm_platform.benchmark_registry import method_ids, parse_action


def task():
    return load_task(SPEC_DIRECTORY/'s16-catalog-debris-avoidance-v1.json')


def own_state():
    inc = math.radians(51.6)
    speed = math.sqrt(MU/(RADIUS+500000.))
    return np.array([-RADIUS-500000.,0.,0.,0.,-speed*math.cos(inc),-speed*math.sin(inc)])


def sample(tick, *, avoid=True):
    # Analytic straight relative trajectories test scoring, not native dynamics.
    t = tick*.02
    own = own_state()
    basis, omega = tnr_frame(own)
    burn = min(t,60.) if avoid else 0.
    offset = .015*burn**2+.03*burn*(t-burn)
    delta = basis@np.array([0.,-1500.+10*t,5.+offset])
    velocity = basis@np.array([0.,10.,.03*burn])+np.cross(omega,delta)
    debris = np.r_[own[:3]+delta,own[3:]+velocity]
    return dict(time_ns=tick*20_000_000, chaser_com_j2000=tuple(own), debris_com_j2000=tuple(debris),
        orientation_body_to_j2000_wxyz=(1.,0.,0.,0.), applied_motor_force_body_n=(-3. if 0<t<=60 and avoid else 0.,0.,0.),
        external_state_writes=0)


@pytest.mark.parametrize('key,value', [('spacecraft_count',2),('debris_count',True),('solar_gravity',1),
    ('source_id','D36'),('target_cooperation','C'),('vision_mode','V2')])
def test_explicit_single_spacecraft_and_noncooperative_debris(key,value):
    with pytest.raises(ValidationError):
        AvoidanceTask.model_validate(task().model_dump()|{key:value})


@pytest.mark.parametrize('key,value', [('relative_position_tnr_m',[0.,-1500.,200.]),
    ('relative_velocity_tnr_m_s',[0.,-10.,0.]),('velocity_half_width_m_s',1.),('position_half_width_m',100.)])
def test_entire_initial_box_must_be_closing_into_screening_sphere(key,value):
    document = task().model_dump()
    document['encounter'][key] = value
    with pytest.raises(ValidationError):
        AvoidanceTask.model_validate(document)


def observation(relative=(0.,-1500.,5.), sequence=0):
    own = own_state()
    basis, omega = tnr_frame(own)
    delta = basis@relative
    debris = np.r_[own[:3]+delta,own[3:]+basis@np.array([0.,10.,0.])+np.cross(omega,delta)]
    return AvoidanceObservation(episode_id='sample',sequence=sequence,sample_time_ns=sequence*500_000_000,
        delivery_time_ns=sequence*500_000_000,chaser_estimate_j2000=tuple(own),debris_catalog_j2000=tuple(debris),
        catalog_sample_time_ns=sequence*500_000_000,catalog_delivery_time_ns=sequence*500_000_000)


def test_method_detects_public_risk_latches_coast_and_reset_clears_history():
    context = AvoidanceContext(task=task(),chaser_mass_kg=170.)
    method = CatalogAvoidance()
    method.reset(context)
    first = observation()
    action = method.act(first)
    assert np.linalg.norm(action.force_j2000_n) == pytest.approx(5.)
    assert parse_action(context.task,action,first) == action
    assert method_ids(context.task)[0] == 'catalog-avoidance-v1'
    clear = observation((0.,-500.,300.),1)
    assert method.act(clear).force_j2000_n == (0.,0.,0.)
    assert method.act(first).force_j2000_n == (0.,0.,0.)
    method.reset(context)
    assert method.act(first) == action
    with pytest.raises(ValueError,match='stale'):
        parse_action(context.task,action,clear)
    with pytest.raises(ValueError,match='exceeds'):
        parse_action(context.task,ForceAction(episode_id='sample',observation_sequence=0,force_j2000_n=(6.,0.,0.)),first)


def test_observation_disallows_private_state_or_future_catalog():
    raw = observation().model_dump()
    for addition in ({'truth_state':[0.]*6},{'catalog_sample_time_ns':1},
                     {'catalog_delivery_time_ns':1},{'debris_catalog_j2000':[float('nan')]*6}):
        with pytest.raises(ValidationError):
            AvoidanceObservation.model_validate(raw|addition)


def test_full_horizon_and_coast_are_required_and_later_safety_vetoes_success():
    scorer = AvoidanceScorer(task(),100.)
    for tick in range(16000):
        scorer.add(sample(tick))
    assert scorer.result()['outcome'] == 'incomplete'
    scorer.add(sample(16000))
    result = scorer.result()
    assert result['outcome'] == 'success'
    assert result['minimum_range_m'] > 200
    assert result['thrust_delta_v_m_s'] == pytest.approx(1.8)
    assert result['final_coast_s'] > 250
    assert result['collision_probability'] is None
    bad = sample(16000)
    bad['applied_motor_force_body_n'] = (6.,0.,0.)
    scorer.add(bad)
    assert scorer.result()['outcome'] == 'invalid_evidence'
    assert 'applied_force' in scorer.result()['safety_violations']


def test_zero_thrust_fails_the_actual_distance_even_before_nominal_closest_time():
    scorer = AvoidanceScorer(task(),100.)
    for tick in range(7101):
        scorer.add(sample(tick,avoid=False))
        if scorer.result()['safety_violations']:
            break
    result = scorer.result()
    assert result['outcome'] == 'safety_violation'
    assert result['minimum_range_m'] < 100
    assert result['final_time_s'] < 150
    assert result['thrust_impulse_n_s'] == 0


@pytest.mark.parametrize('change', ['gap','repeat','write','safe_initialization'])
def test_bad_initialization_or_discontinuous_evidence_cannot_be_repaired(change):
    scorer = AvoidanceScorer(task(),100.)
    first = sample(0)
    if change == 'safe_initialization':
        first['debris_com_j2000'] = tuple(np.asarray(first['debris_com_j2000'])+np.array([500.,0.,0.,0.,0.,0.]))
    scorer.add(first)
    second = sample(2 if change == 'gap' else 0 if change == 'repeat' else 1)
    if change == 'write':
        second['external_state_writes'] = 1
    scorer.add(second)
    assert scorer.result()['outcome'] == 'invalid_evidence'


def test_zero_or_invalid_mass_cannot_hide_resource_consumption():
    for mass in (0.,-1.,float('inf'),float('nan')):
        with pytest.raises(ValueError):
            AvoidanceScorer(task(),mass)
