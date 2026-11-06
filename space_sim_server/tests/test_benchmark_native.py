"""Real native lifecycle test, isolated from Python-MuJoCo test processes.

Requires the team's hydrated assets, Basilisk and a source-matched native build.
The cloud acceptance command runs this explicitly; a skipped basic CI test is
not counted as native validation.
"""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

SERVER = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("Basilisk") is None,
    reason="native SpaceSim acceptance requires Basilisk on the simulation host",
)


def test_native_receiver_outage_withholds_solutions_propagates_and_resets(tmp_path):
    code = r'''
import json
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_registry import native_environment, method_classes, method_ids

task = load_task(SPEC_DIRECTORY/'s23-gnss-outage-recovery-v1.json')
env = native_environment(task, Path(sys.argv[1]))
method = method_classes(task)[method_ids(task)[0]]()
try:
    first = env.reset(3)
    initial = env._physical_state(0).copy()
    realization = env._outage
    assert first.receiver_available and first.target_measurement_j2000 is None
    assert not np.array_equal(first.chaser_measurement_j2000, initial)
    assert not hasattr(env, '_filter') and not hasattr(env, '_filters')
    method.reset(env.context)
    old = method.act(first)
    method.reset(env.context)
    obs, missing, returned = first, 0, False
    while obs.sample_time_ns < realization.return_time_ns+500_000_000:
        assert 'outage' not in obs.model_dump() and 'chaser_truth_j2000' not in obs.model_dump()
        if not obs.receiver_available:
            missing += 1
            assert obs.chaser_measurement_j2000 is None
        elif missing:
            returned = True
            assert obs.chaser_measurement_j2000 is not None
        obs = env.step(method.act(obs))
    assert returned and missing == (realization.return_time_ns-realization.onset_time_ns)//task.decision_period_ns
    score = env.evaluation()
    assert score['outcome'] == 'incomplete' and score['thrust_impulse_n_s'] == 0
    assert score['missing_measurement_count'] == score['outage_estimate_count'] == missing
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    env._truth_stream.flush()
    rows = [json.loads(line) for line in (env.episode_directory/'physics.jsonl').read_text().splitlines()]
    delivered = [row for row in rows if row.get('record_type') == 'gnss_receiver_delivery']
    assert len(delivered) == obs.sequence+1 and sum(not row['receiver_available'] for row in delivered) == missing
    assert all(row['measurement_j2000'] is None for row in delivered if not row['receiver_available'])
    assert len([row for row in rows if 'time_ns' in row]) == env._time_ns//task.physics_period_ns+1
    again = env.reset(3)
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert np.array_equal(env._physical_state(0), initial) and env._outage == realization
    assert env.evaluation()['missing_measurement_count'] == env.evaluation()['estimate_count'] == 0
    try: env.step(old)
    except ValueError as error: assert 'stale' in str(error)
    else: raise AssertionError('old recovery estimate accepted after reset')
    method.reset(env.context)
    new = method.act(again)
    assert new.estimated_state == old.estimated_state
finally:
    env.close()
    method.close()
print('native receiver outage, actual withheld solutions, propagation, return and reset verified')
'''
    result = subprocess.run([sys.executable, '-c', code, str(tmp_path)], cwd=SERVER,
                            capture_output=True, text=True, timeout=480)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'native receiver outage' in result.stdout


def test_native_low_thrust_motor_changes_orbit_and_reset_restores_free_flight(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, load_task
from space_arm_platform.benchmark_registry import native_environment

task = load_task(SPEC_DIRECTORY/'s10-continuous-low-thrust-v1.json')
env = native_environment(task,Path(sys.argv[1]))
def action(observation,force=0.):
    return ForceAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
        force_j2000_n=(0.,-force,0.))
try:
    first = env.reset(3)
    initial = env._physical_state(0).copy()
    observation = first
    for _ in range(10):
        observation = env.step(action(observation))
    old = action(observation,.2)
    env.step(old)
    propelled = env._physical_state(env._time_ns).copy()
    score = env.evaluation()
    assert abs(score['thrust_impulse_n_s']-.1) < 1e-12
    assert score['longest_continuous_thrust_s'] == .5
    assert not score['evidence_errors'] and not score['safety_violations']
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    again = env.reset(3)
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert np.array_equal(env._physical_state(0),initial)
    assert env.evaluation()['longest_continuous_thrust_s'] == env.evaluation()['thrust_impulse_n_s'] == 0
    try: env.step(old)
    except ValueError as error: assert 'stale' in str(error)
    else: raise AssertionError('old low-thrust command accepted after reset')
    observation = again
    for _ in range(11):
        observation = env.step(action(observation))
    displacement = np.linalg.norm(propelled[:3]-env._physical_state(env._time_ns)[:3])
    assert 1e-5 < displacement < .001,displacement
    assert env.evaluation()['thrust_impulse_n_s'] == 0
finally:
    env.close()
print('native low-force actuation, orbital response, sensor and force reset verified')
'''
    result = subprocess.run([sys.executable,'-c',code,str(tmp_path)],cwd=SERVER,
        capture_output=True,text=True,timeout=240)
    assert result.returncode == 0,result.stdout+result.stderr
    assert 'native low-force actuation' in result.stdout


def test_native_orbit_plane_changes_with_normal_thrust_and_resets(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, load_task
from space_arm_platform.benchmark_registry import native_environment

task = load_task(SPEC_DIRECTORY/'s12-gnss-orbit-plane-v1.json')
env = native_environment(task,Path(sys.argv[1]))
def normal(state):
    value = np.cross(state[:3],state[3:])
    return value/np.linalg.norm(value)
def action(obs,force):
    return ForceAction(episode_id=obs.episode_id,observation_sequence=obs.sequence,force_j2000_n=tuple(force))
try:
    first = env.reset(3)
    initial = env._physical_state(0).copy()
    before = env.evaluation()['target_plane_error_rad']
    force = -10*normal(initial)
    obs = first
    for _ in range(10):
        old = action(obs,force)
        obs = env.step(old)
    changed = normal(env._physical_state(env._time_ns))
    score = env.evaluation()
    assert abs(score['thrust_impulse_n_s']-50.) < 1e-9
    assert score['achieved_plane_change_rad'] > 2e-5 and score['target_plane_error_rad'] < before-2e-5
    assert not score['evidence_errors'] and not score['safety_violations']
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    again = env.reset(3)
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert np.array_equal(env._physical_state(0),initial)
    assert env.evaluation()['achieved_plane_change_rad'] < 1e-14 and env.evaluation()['thrust_impulse_n_s'] == 0
    try: env.step(old)
    except ValueError as error: assert 'stale' in str(error)
    else: raise AssertionError('old plane control action accepted after reset')
    obs = again
    for _ in range(10):
        obs = env.step(action(obs,(0.,0.,0.)))
    assert np.linalg.norm(normal(env._physical_state(env._time_ns))-changed) > 2e-5
    assert env.evaluation()['thrust_impulse_n_s'] == 0
finally:
    env.close()
print('native normal force changes angular momentum, with deterministic sensor and orbit reset')
'''
    result = subprocess.run([sys.executable,'-c',code,str(tmp_path)],cwd=SERVER,
        capture_output=True,text=True,timeout=240)
    assert result.returncode == 0,result.stdout+result.stderr
    assert 'native normal force changes angular momentum' in result.stdout


def test_native_debris_is_independent_force_moves_spacecraft_and_reset_restores_sensors(tmp_path):
    code = r'''
import json
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, load_task
from space_arm_platform.benchmark_registry import native_environment

task = load_task(SPEC_DIRECTORY/'s16-catalog-debris-avoidance-v1.json')
env = native_environment(task,Path(sys.argv[1]))
def physical():
    env._truth_stream.flush()
    return [json.loads(line) for line in (env.episode_directory/'physics.jsonl').read_text('utf-8').splitlines()]
def action(obs):
    return ForceAction(episode_id=obs.episode_id,observation_sequence=obs.sequence,force_j2000_n=(1.,0.,0.))
try:
    first = env.reset(3)
    initial = physical()
    assert len(initial) == 1
    manifest = env.model_manifest
    assert manifest['spacecraft_count'] == manifest['debris_count'] == 1
    assert manifest['bodies']['benchmark_debris']['actuators'] == []
    assert env.context.task.observation.target_link == 'none_external_catalog_only'
    assert not np.array_equal(first.debris_catalog_j2000,initial[0]['debris_com_j2000'])
    old = action(first)
    observation = first
    for _ in range(4):
        observation = env.step(action(observation))
    rows = physical()
    assert len(rows) == 101 and env.evaluation()['thrust_impulse_n_s'] > 1.99
    assert not env.evaluation()['evidence_errors']
    assert np.linalg.norm(np.asarray(rows[-1]['debris_com_j2000'][:3])-rows[0]['debris_com_j2000'][:3]) > 14000
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    repeated = env.reset(3)
    assert repeated.episode_id != first.episode_id
    assert repeated.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert physical() == initial
    try:
        env.step(old)
    except ValueError:
        pass
    else:
        raise AssertionError('stale action accepted after reset')
    for _ in range(4):
        repeated = env.step(action(repeated))
    assert physical() == rows
    assert repeated.model_dump(exclude={'episode_id'}) == observation.model_dump(exclude={'episode_id'})
    coast = env.reset(3)
    for _ in range(4):
        coast = env.step(ForceAction(episode_id=coast.episode_id,observation_sequence=coast.sequence,force_j2000_n=(0.,0.,0.)))
    quiet = physical()
    dv = np.asarray(rows[-1]['chaser_com_j2000'][3:])-quiet[-1]['chaser_com_j2000'][3:]
    assert np.linalg.norm(dv) > .005
    assert env.evaluation()['thrust_impulse_n_s'] == 0
finally:
    env.close()
print('native debris, real force, independent catalog and reset verified')
'''
    result = subprocess.run([sys.executable,'-c',code,str(tmp_path/'avoidance-native')],
                            cwd=SERVER,capture_output=True,text=True,timeout=240)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'native debris' in result.stdout


def test_native_wheel_loss_changes_motor_output_and_reset_restores_history(tmp_path):
    code = r'''
import json
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_registry import native_environment
from space_arm_platform.benchmark_wheel_recovery import WheelRecoveryAction

task = load_task(SPEC_DIRECTORY/'s22-wheel-effectiveness-recovery-v1.json')
env = native_environment(task, Path(sys.argv[1]))
def action(observation):
    return WheelRecoveryAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
        motor_torque_xyz_nm=(.04, .04, .04), estimated_effectiveness_xyz=(1., 1., 1.))
def physical():
    env._truth_stream.flush()
    return [json.loads(line) for line in (env.episode_directory/'physics.jsonl').read_text('utf-8').splitlines()]
try:
    first = env.reset(3)
    initial = physical()
    old_action = action(first)
    observation = first
    while observation.sample_time_ns < 8_500_000_000:
        observation = env.step(action(observation))
    rows = physical()
    fault = rows[0]['fault']
    axis = 'xyz'.index(fault['axis'])
    assert len(rows) == 426 and env.evaluation()['fault_triggered']
    assert not env.evaluation()['evidence_errors']
    assert env.evaluation()['lost_motor_impulse_nm_s'] > 0
    for row in rows[1:]:
        expected = np.full(3, .04)
        if row['time_ns'] > fault['onset_time_ns']:
            expected[axis] *= fault['effectiveness']
        assert np.allclose(row['applied_motor_torque_xyz_nm'], expected, atol=1e-12, rtol=0)
    assert not np.array_equal(rows[-1]['angular_velocity_body_rad_s'], rows[0]['angular_velocity_body_rad_s'])
    assert not np.array_equal(observation.measured_motor_torque_xyz_nm, rows[-1]['applied_motor_torque_xyz_nm'])
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    again = env.reset(3)
    assert again.episode_id != first.episode_id
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert physical() == initial
    assert env.evaluation()['fault_samples'] == 0 and not env.evaluation()['fault_triggered']
    assert env.evaluation()['lost_motor_impulse_nm_s'] == 0
    assert env.evaluation()['diagnosis_confirmed_s'] is None
    try:
        env.step(old_action)
    except ValueError:
        pass
    else:
        raise AssertionError('stale action accepted after fault reset')
    for _ in range(10):
        again = env.step(action(again))
    assert physical() == rows[:11]
finally:
    env.close()
print('actual wheel loss, torque telemetry and fault reset verified')
'''
    result = subprocess.run([sys.executable, '-c', code, str(tmp_path/'recovery-native')],
                            cwd=SERVER, capture_output=True, text=True, timeout=240)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'actual wheel loss' in result.stdout


def test_native_pointing_gnss_ephemeris_motor_action_and_reset(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_attitude import WheelTorqueAction
from space_arm_platform.benchmark_registry import native_environment
task = load_task(SPEC_DIRECTORY/'s05-ground-pointing-v1.json')
env = native_environment(task, Path(sys.argv[1]))
try:
    first = env.reset(201)
    physical = env._com_sample(0)
    assert .001 < np.linalg.norm(np.asarray(first.chaser_gnss_j2000[:3])-physical[:3]) < 2.
    assert not np.array_equal(first.chaser_gnss_j2000[3:], physical[3:])
    assert env.evaluation()['outcome'] == 'incomplete'
    assert env.evaluation()['minimum_ground_elevation_rad'] > task.objective.minimum_elevation_rad
    initial_earth = np.asarray(first.ephemeris.earth_j2000_to_fixed)
    initial_rates = np.asarray(env._body_reader().omega_BN_B)
    action = WheelTorqueAction(episode_id=first.episode_id, observation_sequence=0, motor_torque_xyz_nm=(.1,0.,0.))
    following = env.step(action)
    assert following.sample_time_ns == task.decision_period_ns
    assert not np.array_equal(initial_earth, following.ephemeris.earth_j2000_to_fixed)
    assert env._body_reader().omega_BN_B[0] < initial_rates[0]
    assert np.allclose(env.evaluation()['absolute_motor_impulse_xyz_nm_s'], [.002,0.,0.], atol=1e-12)
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    repeated = env.reset(201)
    assert repeated.episode_id != first.episode_id
    assert first.model_dump(exclude={'episode_id'}) == repeated.model_dump(exclude={'episode_id'})
    assert env.evaluation()['sample_count'] == 1
    try:
        env.step(action)
    except ValueError:
        pass
    else:
        raise AssertionError('a stale episode action was accepted after reset')
finally:
    env.close()
print('native pointing sensors, shared ephemeris, motor reaction and reset verified')
'''
    result = subprocess.run([sys.executable, '-c', code, str(tmp_path/'pointing-native')],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'native pointing sensors' in result.stdout


@pytest.mark.parametrize('configuration', ['d15-gnss-formation-hold-v1', 'm02-gnss-formation-hold-v1', 'd19-gnss-cooperative-rendezvous-v1'])
def test_each_formation_member_has_independent_native_actuation_and_reset(tmp_path, configuration):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_formation import FormationAction
from space_arm_platform.benchmark_registry import native_environment
task = load_task(SPEC_DIRECTORY/(sys.argv[2]+'.json'))
env = native_environment(task, Path(sys.argv[1]))
try:
    seed = 3 if task.source_id == 'D19' else 201
    first = env.reset(seed)
    count = task.spacecraft_count
    initial = env._read_physical_states(0).copy()
    assert len(env.model_manifest['crafts']) == count
    assert env.model_manifest['mesh_count'] == 9
    assert all(mass > 169 for mass in env.context.spacecraft_masses_kg)
    assert not np.array_equal(np.asarray(first.estimates_j2000), initial)
    force = tuple((float(i+1), 0., 0.) for i in range(count))
    action = FormationAction(episode_id=first.episode_id, observation_sequence=0, forces_j2000_n=force)
    env.step(action)
    propelled = env._read_physical_states(task.decision_period_ns).copy()
    assert np.allclose(env.evaluation()['per_spacecraft_thrust_impulse_n_s'], .5*np.arange(1, count+1), atol=1e-12)
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    again = env.reset(seed)
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert np.array_equal(env._read_physical_states(0), initial)
    assert env.evaluation()['sample_count'] == 1
    try: env.step(action)
    except ValueError as error: assert 'stale' in str(error)
    else: raise AssertionError('old formation action accepted after reset')
    env.step(FormationAction(episode_id=again.episode_id, observation_sequence=0, forces_j2000_n=((0.,0.,0.),)*count))
    drift = env._read_physical_states(task.decision_period_ns)
    displacement = np.linalg.norm(propelled[:,:3]-drift[:,:3], axis=1)
    assert np.all(displacement > 1e-4) and np.all(displacement < .01), displacement
    assert np.all(np.diff(displacement) > 0), displacement
    assert env.evaluation()['per_spacecraft_thrust_impulse_n_s'] == [0.]*count
finally:
    env.close()
print('every original SARM independently actuated and reset')
'''
    result = subprocess.run([sys.executable, '-c', code, str(tmp_path/configuration), configuration],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'every original SARM independently' in result.stdout


@pytest.mark.parametrize('configuration', ['d05-cooperative-far-rendezvous-v1', 'd12-continuous-gnss-hold-v1', 'd29-vbar-gnss-approach-v1', 'd30-rbar-gnss-approach-v1', 'd33-gnss-flyaround-v1', 'd40-gnss-proximity-abort-v1'])
def test_relative_flight_contracts_use_actual_actuators_and_reset(tmp_path, configuration):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task, ForceAction
from space_arm_platform.benchmark_registry import native_environment

task = load_task(SPEC_DIRECTORY/(sys.argv[2]+'.json'))
env = native_environment(task, Path(sys.argv[1]))
try:
    seed = 3 if task.source_id in {'D05', 'D40'} else 201
    first = env.reset(seed)
    initial = env._read_physical_states(0).copy()
    assert env.model_manifest['mesh_count'] == 113
    action = ForceAction(episode_id=first.episode_id, observation_sequence=0, force_j2000_n=(1., 0., 0.))
    env.step(action)
    propelled = env._read_physical_states(task.decision_period_ns).copy()
    assert abs(env.evaluation()['thrust_impulse_n_s']-.5) < 1e-12
    if hasattr(task, 'path_requirement'):
        assert env.evaluation()['path_requirement'] == task.path_requirement.kind
    else:
        assert env.evaluation()['signed_revolutions'] > 0
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    again = env.reset(seed)
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert np.array_equal(env._read_physical_states(0), initial)
    assert env.evaluation()['sample_count'] == 1
    if task.source_id == 'D40':
        assert env.evaluation()['initial_closing_speed_m_s'] >= task.path_requirement.minimum_initial_closing_speed_m_s
        assert env.evaluation()['reversal_confirmed_s'] is None
    try: env.step(action)
    except ValueError as error: assert 'stale' in str(error)
    else: raise AssertionError('old relative-flight action accepted after reset')
    env.step(ForceAction(episode_id=again.episode_id, observation_sequence=0, force_j2000_n=(0., 0., 0.)))
    difference = np.linalg.norm(propelled[1,:3]-env._read_physical_states(task.decision_period_ns)[1,:3])
    assert 1e-4 < difference < .01, difference
    assert env.evaluation()['thrust_impulse_n_s'] == 0
finally:
    env.close()
print('native relative-flight actuation, evidence and reset verified')
'''
    result = subprocess.run([sys.executable, '-c', code, str(tmp_path/configuration), configuration],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'native relative-flight actuation' in result.stdout


@pytest.mark.parametrize("configuration", ["s14-gnss-state-estimation-v1", "d01-cooperative-gnss-estimation-v1"])
def test_native_navigation_uses_unfiltered_sensors_and_zero_actuation(tmp_path, configuration):
    code = r'''
import json, sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_navigation import NavigationEstimateAction
from simulation.benchmark_native_navigation import NativeNavigationEnvironment

task = load_task(SPEC_DIRECTORY/(sys.argv[2]+".json"))
output = Path(sys.argv[1])
while len(str(output)) < 275:
    output /= 'deep result directory with spaces'
env = NativeNavigationEnvironment(task, output)
def physical():
    return env._physical_state(env._time_ns)[None,:] if task.source_id == "S14" else env._read_physical_states(env._time_ns)
try:
    first = env.reset(201)
    assert len(str(env.episode_directory/'model/team_flight.xml')) > 260
    initial = physical().copy()
    assert not hasattr(env, '_filter') and not hasattr(env, '_filters')
    assert not np.array_equal(first.chaser_measurement_j2000, initial[-1])
    assert (first.target_measurement_j2000 is None) == (task.source_id == "S14")
    assert len(env.model_manifest['bodies']) == task.spacecraft_count
    state = np.array(first.chaser_measurement_j2000)
    if task.source_id == "D01": state -= first.target_measurement_j2000
    action = NavigationEstimateAction(episode_id=first.episode_id, observation_sequence=0, estimate_time_ns=0,
        reference_frame=task.reference_frame, estimated_state=tuple(state))
    env.step(action)
    assert np.linalg.norm(physical()[0,:3]-initial[0,:3]) > 1000
    assert env.evaluation()['estimate_count'] == 1
    assert env.evaluation()['thrust_impulse_n_s'] == 0
    assert env.provenance()['native_dynamics']['external_state_writes'] == 0
    env._truth_stream.flush()
    records = [json.loads(line) for line in (env.episode_directory/'physics.jsonl').read_text().splitlines()]
    assert len(records) == 27 and records[1]['record_type'] == 'navigation_estimate'
    assert records[1]['estimate_time_ns'] == records[0]['time_ns'] == 0
    again = env.reset(201)
    assert again.model_dump(exclude={'episode_id'}) == first.model_dump(exclude={'episode_id'})
    assert np.array_equal(physical(), initial)
    assert env.evaluation()['estimate_count'] == 0 and env.evaluation()['sample_count'] == 1
    try: env.step(action)
    except ValueError as error: assert 'stale' in str(error)
    else: raise AssertionError('old navigation action accepted after reset')
finally:
    env.close()
print('native navigation propagation, raw sensors, estimate timing and reset verified')
'''
    result = subprocess.run([sys.executable, "-c", code, str(tmp_path / configuration), configuration],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "native navigation propagation" in result.stdout


def test_single_orbit_motor_response_and_reset_are_native(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import ForceAction, SPEC_DIRECTORY, load_task
from simulation.benchmark_native_orbit import NativeOrbitEnvironment

task = load_task(SPEC_DIRECTORY / "s11-orbit-raise-v1.json")
env = NativeOrbitEnvironment(task, Path(sys.argv[1]))
try:
    first = env.reset(201)
    initial = env._physical_state(0).copy()
    action = ForceAction(episode_id=first.episode_id, observation_sequence=0, force_j2000_n=(1., 0., 0.))
    env.step(action)
    propelled = env._physical_state(task.decision_period_ns).copy()
    assert abs(env.evaluation()["thrust_impulse_n_s"]-.5) < 1e-12
    assert set(env.model_manifest["bodies"]) == {"cubesat_bus"}
    assert env.model_manifest["mesh_count"] == 9
    assert env.provenance()["native_dynamics"]["external_state_writes"] == 0
    assert not np.array_equal(first.chaser_estimate_j2000, initial)
    again = env.reset(201)
    assert again.episode_id != first.episode_id
    assert again.model_dump(exclude={"episode_id"}) == first.model_dump(exclude={"episode_id"})
    assert np.array_equal(env._physical_state(0), initial)
    assert env.evaluation()["thrust_impulse_n_s"] == 0
    try:
        env.step(action)
    except ValueError as error:
        assert "stale" in str(error)
    else:
        raise AssertionError("stale orbital action accepted after reset")
    env.step(ForceAction(episode_id=again.episode_id, observation_sequence=0, force_j2000_n=(0., 0., 0.)))
    displacement = np.linalg.norm(propelled[:3]-env._physical_state(task.decision_period_ns)[:3])
    assert 1e-4 < displacement < .01, displacement
    assert env.evaluation()["thrust_impulse_n_s"] == 0
    assert env.provenance()["native_dynamics"]["external_state_writes"] == 0
finally:
    env.close()
print("single native orbit force, sensor boundary and reset verified")
'''
    result = subprocess.run([sys.executable, "-c", code, str(tmp_path / "native-orbit")],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "single native orbit force" in result.stdout


def test_native_action_changes_motion_and_reset_restores_rng_and_physics(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import ForceAction, SPEC_DIRECTORY, load_task
from simulation.benchmark_native_flight import NativeWaypointEnvironment

task = load_task(SPEC_DIRECTORY / "d32-cooperative-gnss-v1.json")
env = NativeWaypointEnvironment(task, Path(sys.argv[1]))
try:
    first = env.reset(201)
    initial = env._read_physical_states(0).copy()
    action = ForceAction(episode_id=first.episode_id, observation_sequence=first.sequence,
                         force_j2000_n=(1., 0., 0.))
    env.step(action)
    propelled = env._read_physical_states(task.decision_period_ns).copy()
    assert abs(env.evaluation()["thrust_impulse_n_s"] - 0.5) < 1e-12
    assert env.provenance()["native_dynamics"]["external_state_writes"] == 0
    assert env.model_manifest["mesh_count"] == 113

    again = env.reset(201)
    assert again.episode_id != first.episode_id
    assert again.model_dump(exclude={"episode_id"}) == first.model_dump(exclude={"episode_id"})
    assert np.array_equal(env._read_physical_states(0), initial)
    assert env.evaluation()["sample_count"] == 1
    assert env.evaluation()["thrust_impulse_n_s"] == 0.
    try:
        env.step(action)
    except ValueError as error:
        assert "stale action" in str(error)
    else:
        raise AssertionError("old episode action accepted after reset")
    env.step(ForceAction(episode_id=again.episode_id, observation_sequence=0,
                         force_j2000_n=(0., 0., 0.)))
    passive = env._read_physical_states(task.decision_period_ns)
    displacement = np.linalg.norm(propelled[1, :3] - passive[1, :3])
    assert 1e-4 < displacement < 0.01, displacement
    assert env.evaluation()["thrust_impulse_n_s"] == 0.
    assert env.provenance()["native_dynamics"]["external_state_writes"] == 0
finally:
    env.close()
try:
    env.step(action)
except RuntimeError:
    pass
else:
    raise AssertionError("closed native environment accepted an action")
print("native reset, action readback, state continuity and seeded observations verified")
'''
    result = subprocess.run([sys.executable, "-c", code, str(tmp_path / "native-reset")],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "native reset, action readback" in result.stdout


def test_single_spacecraft_wheels_and_sensor_reset_use_native_physics(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_attitude import AttitudeTask, WheelTorqueAction
from simulation.benchmark_native_attitude import NativeAttitudeEnvironment

task = load_task(SPEC_DIRECTORY / "s02-wheel-attitude-v1.json")
env = NativeAttitudeEnvironment(task, Path(sys.argv[1]))
try:
    first = env.reset(201)
    assert env.model_manifest["spacecraft_count"] == 1
    assert set(env.model_manifest["bodies"]) == {"cubesat_bus"}
    assert "capture_target" not in list(env._scene.getBodyNames())
    assert set(env._stepper.actuator_names) == {"rw_x_motor", "rw_y_motor", "rw_z_motor"}
    physical = env._body_reader()
    physical_q = np.asarray(rbk.MRP2EP(physical.sigma_BN))
    assert 1e-8 < np.linalg.norm(np.asarray(first.orientation_body_to_j2000_wxyz)-physical_q) < .005
    assert not np.array_equal(first.angular_velocity_body_rad_s, physical.omega_BN_B)
    center = np.asarray(env.model_manifest["bodies"]["cubesat_bus"]["center_body_m"])
    rotation = rbk.MRP2C(physical.sigma_BN).T
    state = env.initial_states_j2000[0]
    assert np.allclose(np.asarray(physical.r_BN_N)+rotation@center, state[:3], atol=1e-8, rtol=0)
    assert np.allclose(np.asarray(physical.v_BN_N)+rotation@np.cross(physical.omega_BN_B,center), state[3:], atol=1e-8, rtol=0)
    initial_rate = np.array(physical.omega_BN_B)
    env.step(WheelTorqueAction(episode_id=first.episode_id, observation_sequence=0, motor_torque_xyz_nm=(.1, 0., 0.)))
    assert env._body_reader().omega_BN_B[0] < initial_rate[0]
    assert env._wheel_readers[0]().state > .1
    assert np.allclose(env.evaluation()["absolute_motor_impulse_xyz_nm_s"], [.002, 0., 0.], atol=1e-12)
    assert env.provenance()["native_dynamics"]["external_state_writes"] == 0
    again = env.reset(201)
    assert again.episode_id != first.episode_id
    assert again.model_dump(exclude={"episode_id"}) == first.model_dump(exclude={"episode_id"})
    assert env.evaluation()["sample_count"] == 1
finally:
    env.close()

# Exercise the actual original drive guard in the derived single-body scene.
document = task.model_dump(mode="json")
document["initial"]["wheel_speed_rad_s"] = [.99*task.safety.maximum_wheel_speed_rad_s, 0., 0.]
guarded = NativeAttitudeEnvironment(AttitudeTask.model_validate(document), Path(sys.argv[1]) / "guard")
try:
    observation = guarded.reset(201)
    observation = guarded.step(WheelTorqueAction(episode_id=observation.episode_id, observation_sequence=0,
                                                motor_torque_xyz_nm=(.1, 0., 0.)))
    index = guarded._motor_indices[0]
    assert guarded._stepper.actuator_force[index] == 0.
    guarded.step(WheelTorqueAction(episode_id=observation.episode_id, observation_sequence=1,
                                  motor_torque_xyz_nm=(-.1, 0., 0.)))
    assert abs(guarded._stepper.actuator_force[index] + .1) < 1e-12
finally:
    guarded.close()
print("single-spacecraft native wheel reaction, COM initialization, sensors and speed protection verified")
'''
    result = subprocess.run([sys.executable, "-c", code, str(tmp_path / "attitude-native")],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "single-spacecraft native wheel reaction" in result.stdout


def test_native_detumble_initial_spin_is_bounded_and_resettable(tmp_path):
    code = r'''
import sys
from pathlib import Path
import numpy as np
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, load_task
from simulation.benchmark_native_attitude import NativeAttitudeEnvironment

task = load_task(SPEC_DIRECTORY / "s01-wheel-detumble-v1.json")
env = NativeAttitudeEnvironment(task, Path(sys.argv[1]))
for seed in (*task.seeds.train, *task.seeds.validation, *task.seeds.test):
    state, quaternion, omega = env._initial_state(seed)
    assert .04 <= np.linalg.norm(omega) <= np.sqrt(3)*.04
    assert np.all(np.abs(omega) <= .04)
try:
    first = env.reset(201)
    omega = env._body_reader().omega_BN_B
    assert np.linalg.norm(omega) >= .04
    assert env.evaluation()["outcome"] == "incomplete"
    assert env.provenance()["native_dynamics"]["external_state_writes"] == 0
    again = env.reset(201)
    assert again.model_dump(exclude={"episode_id"}) == first.model_dump(exclude={"episode_id"})
finally:
    env.close()
print("native nonzero spin and reproducible reset verified")
'''
    result = subprocess.run([sys.executable, "-c", code, str(tmp_path / "detumble-native")],
                            cwd=SERVER, capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "native nonzero spin" in result.stdout
