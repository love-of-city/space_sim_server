"""Test offline worker selection without starting services or native dynamics."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

pytestmark = pytest.mark.skipif(
    sys.platform != "win32" or not shutil.which("pwsh"),
    reason="Windows PowerShell entrypoint",
)
ROOT = Path(__file__).resolve().parents[1]
RESOLVER = ROOT / "scripts/posture_runtime.ps1"


def quote(value):
    return "'" + str(value).replace("'", "''") + "'"


def select(repo, *, override="", validate=False, pythonpath=""):
    env = os.environ.copy()
    env["SPACE_SIM_POSTURE_PYTHON"] = str(override)
    env["PYTHONPATH"] = str(pythonpath)
    env.pop("PYTHONHOME", None)
    script = (
        "$ErrorActionPreference='Stop'; "
        f". {quote(RESOLVER)}; "
        f"Resolve-SpaceSimPosturePython -RepositoryRoot {quote(repo)}"
        + (" -ValidateRuntime" if validate else "")
    )
    script = "try { " + script + " } catch { [Console]::Error.WriteLine($_.Exception.Message); exit 1 }"
    return subprocess.run(
        [shutil.which("pwsh"), "-NoProfile", "-Command", script],
        cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8",
        timeout=30,
    )


def test_local_selection_works_without_inherited_environment(tmp_path):
    repo = tmp_path / "repository with spaces"
    repo.mkdir()
    exe = repo / "offline worker/python.exe"
    exe.parent.mkdir()
    exe.touch()
    (repo / ".space-sim-posture-python").write_text(
        "offline worker/python.exe\n", encoding="utf-8-sig",
    )
    result = select(repo)
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == exe


def test_process_override_takes_precedence(tmp_path):
    (tmp_path / ".space-sim-posture-python").write_text("missing.exe", encoding="utf-8")
    result = select(tmp_path, override=sys.executable)
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == Path(sys.executable)


@pytest.mark.parametrize("selection", ["", "missing.exe"])
def test_invalid_local_selection_never_silently_falls_back(tmp_path, selection):
    (tmp_path / ".space-sim-posture-python").write_text(selection, encoding="utf-8")
    result = select(tmp_path)
    assert result.returncode != 0
    assert "fallback" in result.stderr.lower()


def test_invalid_override_does_not_use_valid_local_selection(tmp_path):
    (tmp_path / ".space-sim-posture-python").write_text(sys.executable, encoding="utf-8")
    result = select(tmp_path, override="missing.exe")
    assert result.returncode != 0
    assert "fallback" in result.stderr.lower()


def test_unconfigured_selection_preserves_existing_behavior(tmp_path):
    result = select(tmp_path)
    assert result.returncode == 0, result.stderr
    assert not result.stdout.strip()


@pytest.mark.parametrize("version,success", [("3.7.0", True), ("0.0.0", False)])
def test_isolated_preflight_checks_required_mujoco_version(tmp_path, version, success):
    # Stubs live only in this test's child-process import path.
    (tmp_path / "mujoco.py").write_text(f"__version__ = {version!r}\n", encoding="utf-8")
    (tmp_path / "numpy.py").write_text("", encoding="utf-8")
    result = select(tmp_path, override=sys.executable, validate=True, pythonpath=tmp_path)
    assert (result.returncode == 0) is success, result.stderr
    if not success:
        assert "preflight failed" in result.stderr
        assert "no fallback" in result.stderr


def test_backend_resolves_worker_before_frontend_build_and_server_start():
    source = (ROOT / "scripts/run_backend.ps1").read_text(encoding="utf-8")
    selection = source.index("Resolve-SpaceSimPosturePython")
    assert selection < source.index("npm.cmd run build")
    assert selection < source.index("& $pythonExe @arguments")
    assert "$env:SPACE_SIM_POSTURE_PYTHON = $posturePython" in source


def test_platform_validates_worker_before_stopping_existing_services():
    source = (ROOT / "scripts/run_platform.ps1").read_text(encoding="utf-8")
    selection = source.index("Resolve-SpaceSimPosturePython")
    stop = source.index("& (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet -KeepPendingPublic")
    assert selection < stop < source.index("$cleanupOnStartupFailure = $true")
