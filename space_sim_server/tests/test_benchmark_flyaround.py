"""Analytic path fixtures test winding semantics; native acceptance is separate."""
import math

import numpy as np
import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_flyaround import FlyAroundTask
from space_arm_platform.benchmark_flyaround_score import FlyAroundScorer
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS


def document():
    return load_task(SPEC_DIRECTORY/'d33-gnss-flyaround-v1.json').model_dump()


def analytic_task():
    value = document()
    value['duration_s'] = 60.
    value['safety']['maximum_relative_speed_m_s'] = 20.
    value['initial'].update(position_half_width_m=0., velocity_half_width_m_s=0.,
                            relative_velocity_tnr_m_s=(0., 0., -150*2*math.pi/50))
    return FlyAroundTask.model_validate(value)


def physical(tick, angle, radius=150., normal=0., angular_rate=2*math.pi/50):
    orbit_radius, inc = RADIUS+500000., math.radians(51.6)
    rate = math.sqrt(MU/orbit_radius**3)
    phase = math.pi+rate*tick*.02
    radial = np.array([math.cos(phase), math.sin(phase)*math.cos(inc), math.sin(phase)*math.sin(inc)])
    tangent = np.array([-math.sin(phase), math.cos(phase)*math.cos(inc), math.cos(phase)*math.sin(inc)])
    orbit_normal = np.cross(radial, tangent)
    frame = np.stack((tangent, orbit_normal, radial), axis=1)
    target = np.r_[orbit_radius*radial, orbit_radius*rate*tangent]
    position = np.array([-radius*math.cos(angle), normal, -radius*math.sin(angle)])
    velocity = radius*angular_rate*np.array([math.sin(angle), 0., -math.cos(angle)])
    displacement = frame @ position
    derivative = frame @ velocity+np.cross(rate*orbit_normal, displacement)
    return dict(time_ns=tick*20_000_000, target_com_j2000=tuple(target),
        chaser_com_j2000=tuple(target+np.r_[displacement, derivative]),
        applied_motor_force_body_n=(0., 0., 0.), external_state_writes=0)


@pytest.mark.parametrize('mutation', ['shell', 'initial_annulus', 'initial_plane', 'too_short', 'loose_closure', 'boolean_lap'])
def test_contract_rejects_nonphysical_or_vacuous_revolutions(mutation):
    value = document()
    if mutation == 'shell': value['objective']['radial_tolerance_m'] = 51.
    if mutation == 'initial_annulus': value['initial']['position_half_width_m'] = 4.
    if mutation == 'initial_plane': value['initial']['relative_position_tnr_m'] = (-150., 4., 0.)
    if mutation == 'too_short': value['duration_s'] = 100.
    if mutation == 'loose_closure': value['objective']['closure_tolerance_m'] = 200.
    if mutation == 'boolean_lap': value['objective']['revolutions'] = True
    with pytest.raises(ValidationError): FlyAroundTask.model_validate(value)


def test_full_signed_revolution_crosses_atan_branch_cut_and_returns_to_initial_location():
    scorer = FlyAroundScorer(analytic_task(), 170.)
    for tick in range(2502):
        scorer.add(physical(tick, tick*.02*2*math.pi/50))
        if tick < 2500: assert scorer.result()['outcome'] == 'incomplete'
    result = scorer.result()
    assert result['outcome'] == 'success'
    assert 1. <= result['signed_revolutions'] < 1.001
    assert result['reverse_travel_rad'] == 0.
    assert result['final_closure_error_m'] < .4


@pytest.mark.parametrize('path', ['stationary', 'clockwise', 'oscillation'])
def test_stationkeeping_reverse_motion_and_backtracking_cannot_count_as_a_lap(path):
    scorer = FlyAroundScorer(analytic_task(), 170.)
    for tick in range(3001):
        phase = tick*.02*2*math.pi/50
        angle = 0. if path == 'stationary' else -phase if path == 'clockwise' else .5*math.sin(phase)
        # Match the declared initial state for this grading fixture. Later
        # estimates of velocity do not determine winding; physical positions do.
        scorer.add(physical(tick, angle))
    result = scorer.result()
    assert not result['success']
    assert result['signed_revolutions'] < 1.
    if path != 'stationary': assert 'flyaround_reverse_travel' in result['quality_violations']


@pytest.mark.parametrize('excursion', ['radius', 'plane', 'safety'])
def test_returning_inside_tolerances_does_not_erase_path_or_safety_breach(excursion):
    scorer = FlyAroundScorer(analytic_task(), 170.)
    for tick in range(3001):
        sample = physical(tick, tick*.02*2*math.pi/50,
                          radius=156. if tick == 500 and excursion == 'radius' else 150.,
                          normal=4. if tick == 500 and excursion == 'plane' else 0.)
        if tick == 500 and excursion == 'safety': sample['applied_motor_force_body_n'] = (26., 0., 0.)
        scorer.add(sample)
    result = scorer.result()
    assert result['outcome'] == ('safety_violation' if excursion == 'safety' else 'task_failure')
    assert result['completion_time_s'] is None


@pytest.mark.parametrize('defect', ['gap', 'duplicate', 'overwrite', 'teleport', 'initial'])
def test_inconsistent_physical_evidence_is_rejected(defect):
    scorer = FlyAroundScorer(analytic_task(), 170.)
    first = physical(0, 0., normal=1. if defect == 'initial' else 0.)
    scorer.add(first)
    sample = physical(2 if defect == 'gap' else 0 if defect == 'duplicate' else 1,
                      .5 if defect == 'teleport' else .02*2*math.pi/50)
    if defect == 'overwrite': sample['external_state_writes'] = 1
    scorer.add(sample)
    assert scorer.result()['outcome'] == ('invalid_initialization' if defect == 'initial' else 'invalid_evidence')
