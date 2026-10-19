"""Real Linux runtime isolation: Basilisk and offline MuJoCo run in separate processes."""
from pathlib import Path
import importlib.util
import os
import subprocess
import sys

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux ELF runtime test")
ROOT = Path(__file__).resolve().parents[1]


def run_isolated(source):
    env = dict(os.environ, PYTHONPATH=str(ROOT))
    return subprocess.run([sys.executable, "-c", source], env=env, cwd=ROOT,
                          capture_output=True, text=True, timeout=60)


def test_basilisk_runtime_is_loaded_without_python_mujoco():
    if importlib.util.find_spec("Basilisk") is None:
        pytest.skip("Basilisk is not installed")
    result = run_isolated('''
import sys
from simulation.mujoco_runtime import basilisk_mujoco_runtime
from bsk_render_adapter.native_plugins import loaded_mujoco_library
p = basilisk_mujoco_runtime()
assert p.is_file()
assert "mujoco" not in sys.modules
assert loaded_mujoco_library().mj_version() == 3007000
''')
    assert result.returncode == 0, result.stderr


def test_rejects_second_runtime():
    if any(importlib.util.find_spec(name) is None for name in ("Basilisk", "mujoco")):
        pytest.skip("Basilisk and offline MuJoCo are required")
    result = run_isolated('''
from Basilisk.simulation import mujoco as bsk_mujoco
import mujoco
from bsk_render_adapter.native_plugins import loaded_mujoco_library
try:
    loaded_mujoco_library()
except RuntimeError as error:
    assert "Expected one loaded MuJoCo runtime" in str(error)
else:
    raise AssertionError("two runtime copies were accepted")
''')
    assert result.returncode == 0, result.stderr
