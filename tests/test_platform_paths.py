"""Exercise real PowerShell path discovery without starting UE or services."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh") or r"C:\Program Files\PowerShell\7\pwsh.exe"
pytestmark = pytest.mark.skipif(os.name != "nt" or not Path(PWSH).is_file(), reason="Windows PowerShell 7 required")


def make_adapter(root):
    project = root / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject"
    project.parent.mkdir(parents=True)
    project.write_text("{}", encoding="utf-8")
    return root


def resolve(tmp_path, project, requested=""):
    harness = tmp_path / "resolve.ps1"
    harness.write_text("""
param($Helpers,$Project,$Requested)
$ErrorActionPreference='Stop'
. $Helpers
Resolve-PlatformAdapterRoot $Requested $Project
""", encoding="utf-8")
    return subprocess.run([PWSH, "-NoProfile", "-File", str(harness),
        str(ROOT / "scripts/platform_paths.ps1"), str(project), requested],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=15)


@pytest.mark.parametrize("server_wrapped", [False, True])
@pytest.mark.parametrize("adapter_wrapped", [False, True])
def test_discovers_sibling_and_wrapped_checkouts(tmp_path, server_wrapped, adapter_wrapped):
    workspace = tmp_path / "workspace with spaces"
    project = workspace / "space_sim_server"
    if server_wrapped:
        project /= "space_sim_server"
    project.mkdir(parents=True)
    adapter = workspace / "space_sim_UE_adapter"
    if adapter_wrapped:
        adapter /= "space_sim_UE_Adapter"
    make_adapter(adapter)
    result = resolve(tmp_path, project)
    assert result.returncode == 0, result.stdout + result.stderr
    assert Path(result.stdout.strip()) == adapter


def test_prefers_nearest_valid_checkout(tmp_path):
    project = tmp_path / "outer/server/server"
    project.mkdir(parents=True)
    nearest = make_adapter(project.parent / "space_sim_UE_adapter")
    make_adapter(project.parent.parent / "space_sim_UE_adapter")
    result = resolve(tmp_path, project)
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == nearest


def test_explicit_relative_path_overrides_discovery(tmp_path):
    project = tmp_path / "server"
    project.mkdir()
    chosen = make_adapter(project / "custom adapter")
    make_adapter(project.parent / "space_sim_UE_adapter")
    result = resolve(tmp_path, project, "custom adapter")
    assert result.returncode == 0, result.stderr
    assert Path(result.stdout.strip()) == chosen


def test_invalid_explicit_path_does_not_silently_fall_back(tmp_path):
    project = tmp_path / "server"
    project.mkdir()
    make_adapter(project.parent / "space_sim_UE_adapter")
    result = resolve(tmp_path, project, "missing adapter")
    assert result.returncode != 0
    assert "not a UE adapter checkout" in result.stderr


def test_empty_adapter_folder_is_not_a_checkout(tmp_path):
    project = tmp_path / "server"
    project.mkdir()
    (project.parent / "space_sim_UE_adapter").mkdir()
    result = resolve(tmp_path, project)
    assert result.returncode != 0
    assert "UE adapter checkout was not found" in result.stderr
