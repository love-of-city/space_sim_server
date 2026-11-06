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
