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
