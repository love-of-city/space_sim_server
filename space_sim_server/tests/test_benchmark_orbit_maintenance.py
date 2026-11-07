"""Whole-window orbit grading; fixtures test the grader, not propagated motion."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from pydantic import ValidationError
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2, EARTH_RADIUS_M, OrbitObservation, OrbitTask
from space_arm_platform.benchmark_orbit_score import OrbitScorer
from space_arm_platform.benchmark_external import ExternalMethodDefinition
from test_benchmark_orbit import sample
from simulation.benchmark_orbit_methods import GNSSOrbitManeuver


def task():
    return load_task(SPEC_DIRECTORY/'s13-gnss-orbit-maintenance-v1.json')


def short_grader_fixture():
    # Deliberately compress a mathematical scoring fixture. Production schema
    # validation separately rejects a maintenance window shorter than 600 s.
    spec = task()
    return spec.model_copy(update={'duration_s': .5, 'objective': spec.objective.model_copy(
        update={'acquisition_deadline_s': .1, 'dwell_s': .4})})


@pytest.mark.parametrize('change', ['point_mass', 'short_window', 'late_end', 'transfer', 'wrong_source'])
def test_maintenance_cannot_be_a_renamed_short_transfer_or_unperturbed_dwell(change):
    document = task().model_dump()
    if change == 'point_mass':
        document['earth_gravity'] = 'point_mass'
    elif change == 'short_window':
        document['objective']['dwell_s'] = 20.
        document['duration_s'] = 140.
    elif change == 'late_end':
        document['duration_s'] = 1201.
    elif change == 'transfer':
        document['objective']['altitude_m'] = 501000.
    else:
        document.update(source_id='S11', requirement_id='ssb.orbit.raise-lower')
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(document)


def score(defect=None):
    scorer = OrbitScorer(short_grader_fixture(), 100.)
    for tick in range(26):
        altitude = 500030. if (defect == 'early_acquisition' and tick == 4) or (defect == 'mid_window' and tick == 10) else 500000.
        raw = sample(tick, altitude, force=(1., 0., 0.))
        if defect == 'safety' and tick == 20:
            raw['applied_motor_force_body_n'] = (6., 0., 0.)
        if defect == 'gap' and tick == 10:
            continue
        scorer.add(raw)
        if tick < 25 and defect is None:
            assert scorer.result()['outcome'] == 'incomplete'
    return scorer.result()


def test_maintenance_may_apply_corrections_but_must_cover_the_entire_fixed_window():
    result = score()
    assert result['success'] and result['completion_time_s'] == .5
    assert result['longest_dwell_s'] == .4
    assert result['maintenance_samples'] == 21 and result['failed_maintenance_samples'] == 0
    assert result['thrust_impulse_n_s'] == pytest.approx(.5)
    assert score('early_acquisition')['success']


def test_a_good_final_orbit_cannot_erase_one_bad_sample_in_the_maintenance_window():
    result = score('mid_window')
    assert result['outcome'] == 'task_failure'
    assert result['final_orbit']['semimajor_axis_error_m'] < 1e-6
    assert result['failed_maintenance_samples'] == 1
    assert result['maximum_maintenance_axis_error_m'] == pytest.approx(30.)
    assert result['quality_violations'] == ['continuous_orbit_band']


def test_maintenance_cannot_override_missing_evidence_or_physical_force_safety():
    assert score('gap')['outcome'] == 'invalid_evidence'
    assert score('safety')['outcome'] == 'safety_violation'


def test_original_external_orbit_interface_can_load_the_new_public_contract():
    spec = task()
    definition = ExternalMethodDefinition(SPEC_DIRECTORY.parents[2]/'examples/autonomy_methods/orbit-pd.json', spec)
    assert definition.spec.task_schemas == (spec.schema_version,)
    definition.verify_unchanged()
    original = load_task(SPEC_DIRECTORY/'s11-orbit-raise-v1.json').model_dump()
    mislabeled = deepcopy(original)
    mislabeled.update(source_id='S13', requirement_id='ssb.orbit.stationkeeping')
    with pytest.raises(ValidationError):
        OrbitTask.model_validate(mislabeled)


def test_controller_keeps_correcting_gnss_error_inside_the_maintenance_window():
    spec = task()
    method = GNSSOrbitManeuver()
    method.reset(SimpleNamespace(task=spec, chaser_mass_kg=100.))
    initial = np.asarray(sample(0)['chaser_com_j2000'])
    observation = OrbitObservation(episode_id='public', sequence=0, sample_time_ns=0,
        delivery_time_ns=0, chaser_estimate_j2000=tuple(initial))
    method.act(observation)
    normal = np.cross(initial[:3], initial[3:])
    normal /= np.linalg.norm(normal)
    angle = 500.*np.sqrt(EARTH_MU_M3_S2/(EARTH_RADIUS_M+500000.)**3)
    rotated = Rotation.from_rotvec(normal*angle)
    position, velocity = rotated.apply(initial[:3]), rotated.apply(initial[3:])
    radial = position/np.linalg.norm(position)
    displaced = np.r_[position+3.*radial, velocity]
    observation = observation.model_copy(update=dict(sequence=1000, sample_time_ns=500_000_000_000,
        delivery_time_ns=500_000_000_000, chaser_estimate_j2000=tuple(displaced)))
    force = np.asarray(method.act(observation).force_j2000_n)
    assert force@radial < -.5
    assert np.linalg.norm(force) < spec.safety.maximum_force_n
