from pathlib import Path
import json
import os
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SHELL = shutil.which("powershell.exe")
pytestmark = pytest.mark.skipif(not SHELL, reason="Windows PowerShell required")


def make_workspace(tmp_path):
    root = tmp_path / "workspace with spaces"
    scripts = root / "scripts"
    scripts.mkdir(parents=True)
    shutil.copy(ROOT / "scripts/run_simulation.ps1", scripts)
    shutil.copy(ROOT / "scripts/python_runtime.ps1", scripts)
    modules = root / "fake_modules"
    basilisk = modules / "Basilisk" / "simulation"
    basilisk.mkdir(parents=True)
    for module in (modules / "numpy.py", basilisk.parent / "__init__.py",
                   basilisk / "__init__.py", basilisk / "mujoco.py"):
        module.write_text("", encoding="utf-8")
    simulation = root / "simulation"
    simulation.mkdir()
    (simulation / "teleop_grasp_unreal.py").write_text(
        "import json, sys; print(json.dumps(sys.argv[1:]))", encoding="utf-8"
    )
    return root


def test_explicit_python_preserves_model_paths_and_control_port(tmp_path):
    root = make_workspace(tmp_path)
    environment = os.environ.copy()
    environment["SPACE_SIM_PYTHON"] = sys.executable
    environment["PYTHONPATH"] = str(root / "fake_modules")
    model = root / "models with spaces" / "platform"
    adapter = root / "adapter with spaces"
    result = subprocess.run(
        [SHELL, "-NoProfile", "-File", str(root / "scripts/run_simulation.ps1"),
         "-ModelRoot", str(model), "-AdapterRoot", str(adapter), "-ControlPort", "18766"],
        capture_output=True, text=True, env=environment, timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    arguments = json.loads(result.stdout)
    assert arguments[arguments.index("--model-root") + 1] == str(model)
    assert arguments[arguments.index("--control-port") + 1] == "18766"
    assert arguments[arguments.index("--duration") + 1] == "0"
    assert arguments[arguments.index("--catalog") + 1].endswith("sarm_platform.catalog.json")


def test_invalid_explicit_python_fails_without_conda_fallback(tmp_path):
    root = make_workspace(tmp_path)
    environment = os.environ.copy()
    environment["SPACE_SIM_PYTHON"] = str(root / "missing-python.exe")
    result = subprocess.run(
        [SHELL, "-NoProfile", "-File", str(root / "scripts/run_simulation.ps1"),
         "-ModelRoot", str(root / "platform"), "-AdapterRoot", str(root / "adapter")],
        capture_output=True, text=True, env=environment, timeout=20,
    )
    assert result.returncode != 0
    assert "Selected Python does not exist" in result.stderr
