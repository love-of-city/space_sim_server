"""Exercise the real PowerShell selector with isolated environments, without UE."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

pytestmark = pytest.mark.skipif(sys.platform != "win32" or not shutil.which("pwsh"), reason="Windows PowerShell entrypoints")
ROOT = Path(__file__).resolve().parents[1]
RESOLVER = ROOT / "scripts/python_runtime.ps1"


def quote(value):
    return "'" + str(value).replace("'", "''") + "'"


@pytest.fixture(scope="module")
def runtime(tmp_path_factory):
    root = tmp_path_factory.mktemp("runtime with spaces")
    subprocess.run([sys.executable, "-m", "venv", "--without-pip", str(root / "base")], check=True)
    repo = root / "repository"
    repo.mkdir()
    for path in (repo / ".venv", repo / "venv", root / ".venv", root / "venv", root / "active"):
        shutil.copytree(root / "base", path)
    return root, repo


def run_selector(runtime, *, explicit="", override="", active="", conda="", modules="", repo=None):
    root, default_repo = runtime
    env = os.environ.copy()
    for key in ("SPACE_SIM_PYTHON", "VIRTUAL_ENV", "CONDA_PREFIX", "PYTHONHOME", "PYTHONPATH"):
        env.pop(key, None)
    env.update(SPACE_SIM_PYTHON=str(override), VIRTUAL_ENV=str(active), CONDA_PREFIX=str(conda))
    env["PATH"] = str(root / "base/Scripts") + os.pathsep + env.get("PATH", "")
    script = f"$ErrorActionPreference='Stop'; . {quote(RESOLVER)}; Resolve-SpaceSimPython -RepositoryRoot {quote(repo or default_repo)} -RequestedPython {quote(explicit)}"
    if modules:
        script += " -RequiredModules " + quote(modules)
    script = "try { " + script + " } catch { [Console]::Error.WriteLine($_.Exception.Message); exit 1 }"
    return subprocess.run([shutil.which("pwsh"), "-NoProfile", "-Command", script], env=env, cwd=root, capture_output=True, text=True, encoding="utf-8")


def test_selection_priority_and_space_paths(runtime):
    root, repo = runtime
    exe = root / "base/Scripts/python.exe"
    active = root / "active"
    cases = [
        (dict(explicit=exe, override="missing", active="missing"), exe),
        (dict(override=exe, active="missing"), exe),
        (dict(active=active, conda="missing"), active / "Scripts/python.exe"),
        (dict(conda=exe.parent), exe),
        (dict(), repo / ".venv/Scripts/python.exe"),
    ]
    for args, expected in cases:
        result = run_selector(runtime, **args)
        assert result.returncode == 0, result.stderr
        assert Path(result.stdout.strip()) == expected


def test_invalid_environment_never_falls_back(runtime):
    for args in (dict(explicit="missing"), dict(override="missing"), dict(active="missing"), dict(conda="missing")):
        result = run_selector(runtime, **args)
        assert result.returncode != 0
        assert "no fallback" in result.stderr


def test_missing_dependency_never_falls_back(runtime):
    result = run_selector(runtime, modules="space_sim_deliberately_missing_dependency")
    assert result.returncode != 0
    assert "preflight failed" in result.stderr
    assert "no fallback" in result.stderr


def test_path_fallback(runtime):
    root, _ = runtime
    isolated = root / "isolated/repository"
    isolated.mkdir(parents=True)
    result = run_selector(runtime, repo=isolated)
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == root / "base/Scripts/python.exe"


def test_local_directory_priority(runtime):
    root, _ = runtime
    parent = root / "local order"
    repo = parent / "repository"
    repo.mkdir(parents=True)
    candidates = [repo / ".venv", repo / "venv", parent / ".venv", parent / "venv"]
    for path in candidates:
        shutil.copytree(root / "base", path)
    for path in candidates:
        result = run_selector(runtime, repo=repo)
        assert result.returncode == 0, result.stderr
        assert Path(result.stdout.strip()) == path / "Scripts/python.exe"
        shutil.rmtree(path)
