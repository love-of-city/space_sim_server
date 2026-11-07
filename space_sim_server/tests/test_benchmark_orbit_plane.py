"""An orbit-plane goal depends on r cross v, never on spacecraft attitude."""
import math

import numpy as np
import pytest
from pydantic import ValidationError

from simulation.benchmark_plane_reference import plane_reference
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_orbit import OrbitContext, OrbitObservation, OrbitTask, orbit_plane_normal
from space_arm_platform.benchmark_orbit_score import OrbitScorer, orbital_elements
from space_arm_platform.benchmark_registry import method_classes, method_ids, parse_action
from test_benchmark_orbit import circular, sample


def task():
    return load_task(SPEC_DIRECTORY/'s12-gnss-orbit-plane-v1.json')


def target_state():
    state = np.asarray(circular(500000.))
    speed = np.linalg.norm(state[3:])
    inclination = math.radians(51.65)
    return (*state[:3],0.,-speed*math.cos(inclination),-speed*math.sin(inclination))


def short_task():
    document = task().model_dump()
    document['duration_s'] = 1.
    document['objective']['dwell_s'] = .1
    return OrbitTask.model_validate(document)


def test_plane_definition_uses_oriented_J2000_angular_momentum():
    assert orbit_plane_normal(0.,0.) == (0.,0.,1.)
    assert orbit_plane_normal(90.,0.) == pytest.approx((0.,-1.,0.),abs=1e-14)
    assert orbit_plane_normal(90.,90.) == pytest.approx((1.,0.,0.),abs=1e-14)
    assert orbital_elements(target_state())['normal'] == pytest.approx(orbit_plane_normal(51.65,0.),abs=1e-14)


def test_attitude_rotation_and_correct_altitude_cannot_replace_an_orbit_plane_change():
    scorer = OrbitScorer(short_task(),170.)
    for tick in range(51):
        row = sample(tick)
        angle = math.radians(.05)
        row['orientation_body_to_j2000_wxyz'] = (math.cos(angle/2),math.sin(angle/2),0.,0.)
        scorer.add(row)
    result = scorer.result()
    assert result['outcome'] == 'task_failure' and result['quality_violations'] == ['target_orbit_plane']
    assert result['target_plane_error_rad'] == pytest.approx(math.radians(.05),abs=1e-14)
    assert result['achieved_plane_change_rad'] < 1e-14
    assert result['final_orbit']['semimajor_axis_error_m'] < 1e-8


def test_changed_orbit_requires_unpowered_continuous_confirmation_and_safety():
    scorer = OrbitScorer(short_task(),170.)
    scorer.add(sample(0))
    for tick in range(1,8):
        scorer.add(sample(tick) | dict(chaser_com_j2000=target_state(),applied_motor_force_body_n=(1.,0.,0.)))
    assert scorer.result()['outcome'] == 'incomplete' and scorer.result()['longest_dwell_s'] == 0
    for tick in range(8,14):
        scorer.add(sample(tick) | dict(chaser_com_j2000=target_state()))
    result = scorer.result()
    assert result['outcome'] == 'success' and result['target_plane_error_rad'] < 1e-14
    assert result['achieved_plane_change_rad'] == pytest.approx(math.radians(.05),abs=1e-14)
    scorer.add(sample(14) | dict(chaser_com_j2000=target_state(),applied_motor_force_body_n=(26.,0.,0.)))
    assert scorer.result()['outcome'] == 'safety_violation'


def test_starting_in_the_target_plane_is_invalid_initialization_evidence():
    scorer = OrbitScorer(short_task(),170.)
    scorer.add(sample(0) | dict(chaser_com_j2000=target_state()))
    assert scorer.result()['outcome'] == 'invalid_evidence'
    assert 'initial_orbit_outside_declared_distribution' in scorer.result()['evidence_errors']


@pytest.mark.parametrize('change', [dict(inclination_deg=51.6),dict(plane_tolerance_rad=.001),
    dict(inclination_deg=-1.),dict(raan_deg=360.),dict(plane_definition='body_attitude')])
def test_target_must_require_a_nontrivial_oriented_orbit_change(change):
    document = task().model_dump()
    document['objective'].update(change)
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(document)


def test_plane_objective_cannot_alias_raise_lower():
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(task().model_dump() | dict(source_id='S11',requirement_id='ssb.orbit.raise-lower'))


@pytest.mark.parametrize('time', [50.,200.,350.])
def test_reference_velocity_and_acceleration_are_derivatives_of_its_position(time):
    initial = np.asarray(circular(500020.))
    target = np.asarray(orbit_plane_normal(51.65,.01))
    retained = target.copy()
    step = .1
    minus = plane_reference(initial,500000.,target,time-step,406.)
    current = plane_reference(initial,500000.,target,time,406.)
    plus = plane_reference(initial,500000.,target,time+step,406.)
    assert np.array_equal(target,retained)
    assert (plus[0]-minus[0])/(2*step) == pytest.approx(current[1],abs=3e-5)
    assert (plus[1]-minus[1])/(2*step) == pytest.approx(current[2],abs=1e-5)


@pytest.mark.parametrize('target', [orbit_plane_normal(51.65,0.),orbit_plane_normal(51.6,0.),
    tuple(-value for value in orbit_plane_normal(51.6,0.))])
def test_reference_endpoints_cover_parallel_and_opposite_normals_without_state_writes(target):
    initial = np.asarray(circular(500000.))
    original = initial.copy()
    start = plane_reference(initial,500000.,target,0.,406.)
    finish = plane_reference(initial,500000.,target,406.,406.)
    assert np.array_equal(initial,original)
    assert np.r_[start[0],start[1]] == pytest.approx(initial,abs=1e-8)
    assert orbital_elements(np.r_[finish[0],finish[1]])['normal'] == pytest.approx(target,abs=1e-13)


def test_public_guidance_requests_a_real_plane_maneuver_and_reset_restores_reference():
    spec = task()
    context = OrbitContext(task=spec,chaser_mass_kg=169.)
    baseline = method_classes(spec)[method_ids(spec)[0]]()
    unchanged = method_classes(spec)['unchanged-orbit-plane-v1']()
    for method in (baseline,unchanged):
        method.reset(context)
    first = OrbitObservation(episode_id='plane-test',sequence=0,sample_time_ns=0,delivery_time_ns=0,
        chaser_estimate_j2000=circular(500000.))
    initial_action = baseline.act(first)
    unchanged.act(first)
    initial = np.asarray(first.chaser_estimate_j2000)
    r,v,_ = plane_reference(initial,500000.,orbit_plane_normal(51.6,0.),150.,406.)
    later = first.model_copy(update=dict(sequence=300,sample_time_ns=150_000_000_000,delivery_time_ns=150_000_000_000,
        chaser_estimate_j2000=tuple(np.r_[r,v])))
    maneuver = parse_action(spec,baseline.act(later),later)
    keep = parse_action(spec,unchanged.act(later),later)
    assert np.linalg.norm(maneuver.force_j2000_n) > 1.
    assert np.linalg.norm(keep.force_j2000_n) < 1e-6
    baseline.reset(context)
    assert baseline.act(first) == initial_action
    assert 'unchanged-orbit-plane-v1' in method_ids(spec)
    assert 'unchanged-orbit-plane-v1' not in method_ids(load_task(SPEC_DIRECTORY/'s10-continuous-low-thrust-v1.json'))
    assert tuple(method_classes(spec)) == method_ids(spec)
    assert 'pulsed-orbit-burn-v1' not in method_classes(spec)
