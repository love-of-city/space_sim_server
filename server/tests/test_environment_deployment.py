"""Protect saved environment edits across preparation and runtime synchronization."""
import importlib.util
from pathlib import Path

import pytest

from space_arm_platform import unreal_runtime
from tools.deployment import sync_unreal_runtime as sync

ROOT = Path(__file__).resolve().parents[2]
PREPARATION = ROOT / "ue-adapter/Unreal/BskUnrealRenderer/scripts/prepare_environment_level.py"
spec = importlib.util.spec_from_file_location("prepare_environment_level", PREPARATION)
preparation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preparation)


@pytest.fixture(autouse=True)
def isolate_environment_package(monkeypatch):
    # Map/synchronization tests use tiny package fixtures; external package
    # validation itself is covered by the dedicated preflight tests.
    monkeypatch.setattr(preparation, "check_external_assets", lambda project: None)


def put(root, relative, content=b"fixture"):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


def fixture_source(tmp_path):
    adapter = tmp_path / "source"
    project = adapter / sync.PROJECT_RELATIVE
    put(project, sync.PROJECT_FILE, b'{"FileVersion": 3}')
    put(project, "Config/DefaultEngine.ini", b"source settings")
    put(project, "Source/module.cpp", b"source code")
    put(project, sync.LEVEL, preparation.PACKAGE_MAGIC[0] + b"saved source environment" * 20)
    return adapter, project


def test_map_preparation_creates_once_and_preserves_artist_edits(tmp_path, monkeypatch):
    project = tmp_path / "project"
    put(project, "BskUnrealRenderer.uproject")
    put(project, "scripts/create_environment_level.py")
    unreal = tmp_path / "unreal"
    import sys
    platform = "Win64" if sys.platform == "win32" else "Linux"
    executable = "UnrealEditor-Cmd.exe" if sys.platform == "win32" else "UnrealEditor-Cmd"
    put(unreal, f"Engine/Binaries/{platform}/{executable}")
    calls = []
    def commandlet(command, **kwargs):
        calls.append(command)
        assert kwargs["check"]
        put(project, preparation.MAP_RELATIVE, preparation.PACKAGE_MAGIC[0] + b"created environment" * 20)
    monkeypatch.setattr(preparation.subprocess, "run", commandlet)
    assert preparation.prepare(project, unreal) is True
    saved_content = preparation.PACKAGE_MAGIC[0] + b"hand edited transform material exposure" * 20
    saved = put(project, preparation.MAP_RELATIVE, saved_content)
    assert preparation.prepare(project, unreal) is False
    assert saved.read_bytes() == saved_content
    assert len(calls) == 1
    assert "-run=pythonscript" in calls[0]


@pytest.mark.parametrize("content", [b"", b"truncated", b"not a UE package" * 100,
                                     b"version https://git-lfs.github.com/spec/v1\noid sha256:abc\nsize 900\n"])
def test_invalid_existing_map_is_never_regenerated(tmp_path, monkeypatch, content):
    saved = put(tmp_path, preparation.MAP_RELATIVE, content)
    monkeypatch.setattr(preparation.subprocess, "run", lambda *a, **k: pytest.fail("must not overwrite an existing map"))
    with pytest.raises(RuntimeError):
        preparation.prepare(tmp_path, tmp_path)
    assert saved.read_bytes() == content


def test_runtime_prepares_missing_map_before_launch_and_keeps_saved_map(tmp_path, monkeypatch):
    monkeypatch.setattr(unreal_runtime, "check_external_environment_assets", lambda adapter: None)
    project = tmp_path / sync.PROJECT_RELATIVE
    put(project, "scripts/prepare_environment_level.py")
    calls = []
    def prepare(command, **kwargs):
        calls.append(command)
        put(project, preparation.MAP_RELATIVE, preparation.PACKAGE_MAGIC[0] + b"saved renderer environment" * 20)
    monkeypatch.setattr(unreal_runtime.subprocess, "run", prepare)
    level = unreal_runtime.ensure_environment_level(tmp_path, tmp_path / "engine")
    assert level.is_file()
    assert unreal_runtime.ensure_environment_level(tmp_path, tmp_path / "engine") == level
    assert len(calls) == 1


def test_explicit_verify_loads_existing_map_without_replacing_art(tmp_path, monkeypatch):
    project = tmp_path / "project"
    put(project, "BskUnrealRenderer.uproject")
    put(project, "scripts/create_environment_level.py")
    content = preparation.PACKAGE_MAGIC[0] + b"saved artist environment" * 20
    level = put(project, preparation.MAP_RELATIVE, content)
    unreal = tmp_path / "unreal"
    import sys
    platform = "Win64" if sys.platform == "win32" else "Linux"
    executable = "UnrealEditor-Cmd.exe" if sys.platform == "win32" else "UnrealEditor-Cmd"
    put(unreal, f"Engine/Binaries/{platform}/{executable}")
    calls = []
    monkeypatch.setattr(preparation.subprocess, "run", lambda command, **kwargs: calls.append(command))
    assert preparation.prepare(project, unreal, verify=True) is False
    assert len(calls) == 1
    assert level.read_bytes() == content


def test_sync_preserves_generated_state_machine_config_and_unrelated_files(tmp_path):
    source, project = fixture_source(tmp_path)
    for relative in ("Saved/AssetImport/local.json", "Plugins/Runtime/Binaries/Linux/runtime.so",
                     "Config/machine.local.json", "Config/UserEngine.ini", "Source/__pycache__/module.pyc"):
        put(project, relative, b"must not deploy")
    destination = tmp_path / "runtime"
    runtime = destination / sync.PROJECT_RELATIVE
    saved = put(runtime, "Saved/AssetImport/local.json", b"machine catalog")
    config = put(runtime, "Config/machine.local.json", b"machine settings")
    unrelated = put(runtime, "Content/Artist/Custom.uasset", b"artist asset")
    report = sync.synchronize(source, destination, dry_run=True)
    assert report["counts"]["create"] == 4
    assert not (runtime / sync.PROJECT_FILE).exists()
    sync.synchronize(source, destination)
    assert saved.read_bytes() == b"machine catalog"
    assert config.read_bytes() == b"machine settings"
    assert unrelated.read_bytes() == b"artist asset"
    assert not (runtime / "Plugins/Runtime/Binaries/Linux/runtime.so").exists()
    assert (runtime / sync.LEVEL).read_bytes() == (project / sync.LEVEL).read_bytes()


def test_sync_updates_unchanged_baseline_but_rejects_runtime_edits_before_copying(tmp_path):
    source, project = fixture_source(tmp_path)
    destination = tmp_path / "runtime"
    sync.synchronize(source, destination)
    runtime = destination / sync.PROJECT_RELATIVE
    put(project, "Source/module.cpp", b"updated source")
    sync.synchronize(source, destination)
    assert (runtime / "Source/module.cpp").read_bytes() == b"updated source"
    put(runtime, "Source/module.cpp", b"runtime edit")
    put(project, "Source/new.cpp", b"new source")
    with pytest.raises(RuntimeError, match="before copying"):
        sync.synchronize(source, destination)
    assert not (runtime / "Source/new.cpp").exists()
    assert (runtime / "Source/module.cpp").read_bytes() == b"runtime edit"


@pytest.mark.parametrize("protected", [sync.LEVEL, "Content/BSK/Environment/MI_SunPreview.uasset"])
def test_environment_art_has_separate_explicit_overwrite_permission(tmp_path, protected):
    source, project = fixture_source(tmp_path)
    put(project, protected, preparation.PACKAGE_MAGIC[0] + b"saved environment art" * 20)
    destination = tmp_path / "runtime"
    sync.synchronize(source, destination)
    runtime = destination / sync.PROJECT_RELATIVE
    put(project, protected, preparation.PACKAGE_MAGIC[0] + b"new authored source art" * 20)
    with pytest.raises(RuntimeError):
        sync.synchronize(source, destination, overwrite=True)
    sync.synchronize(source, destination, overwrite_level=True)
    assert (runtime / protected).read_bytes() == (project / protected).read_bytes()


def test_sync_rejects_lfs_pointer_before_mutating_destination(tmp_path):
    source, project = fixture_source(tmp_path)
    put(project, "Content/Unresolved.uasset", b"version https://git-lfs.github.com/spec/v1\n")
    destination = tmp_path / "runtime"
    with pytest.raises(RuntimeError, match="Git LFS pointer"):
        sync.synchronize(source, destination)
    assert not destination.exists()


def test_sync_rejects_overlapping_roots(tmp_path):
    source, _ = fixture_source(tmp_path)
    for destination in (source, source / "runtime", source.parent):
        with pytest.raises(ValueError, match="non-overlapping"):
            sync.synchronize(source, destination)


def test_runtime_checks_external_assets_even_when_map_is_saved(tmp_path, monkeypatch):
    project = tmp_path / sync.PROJECT_RELATIVE
    checker = put(project, "scripts/check_external_environment_assets.py")
    level = put(project, sync.LEVEL, preparation.PACKAGE_MAGIC[0] + b"artist map" * 30)
    calls = []
    monkeypatch.setattr(unreal_runtime.subprocess, "run", lambda command, **kw: calls.append((command, kw)))
    assert unreal_runtime.ensure_environment_level(tmp_path, tmp_path / "engine") == level
    assert len(calls) == 1
    assert calls[0][0][1:] == [str(checker), "--project", str(project)]
    assert calls[0][1]["check"] is True


def test_runtime_rejects_missing_external_assets_before_loading_saved_map(tmp_path, monkeypatch):
    import subprocess
    project = tmp_path / sync.PROJECT_RELATIVE
    put(project, "scripts/check_external_environment_assets.py")
    level = put(project, sync.LEVEL, preparation.PACKAGE_MAGIC[0] + b"artist map" * 30)
    original = level.read_bytes()
    def missing_assets(command, **kwargs):
        raise subprocess.CalledProcessError(1, command)
    monkeypatch.setattr(unreal_runtime.subprocess, "run", missing_assets)
    with pytest.raises(subprocess.CalledProcessError):
        unreal_runtime.ensure_environment_level(tmp_path, tmp_path / "engine")
    assert level.read_bytes() == original


def test_read_only_preflight_never_creates_missing_map(tmp_path, monkeypatch):
    monkeypatch.setattr(unreal_runtime, "check_external_environment_assets", lambda adapter: None)
    monkeypatch.setattr(unreal_runtime.subprocess, "run", lambda *a, **kw: pytest.fail("must not launch a commandlet"))
    with pytest.raises(FileNotFoundError, match="build the Editor target"):
        unreal_runtime.ensure_environment_level(tmp_path, tmp_path / "engine", prepare=False)


def test_linux_build_uses_source_tools_for_a_separate_runtime_project(tmp_path, monkeypatch):
    from tools.assets import build_unreal_linux as build
    import sys
    adapter = tmp_path / "runtime adapter"
    engine = tmp_path / "engine"
    calls, prepared = [], []
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(sys, "argv", ["build_unreal_linux.py", "--adapter", str(adapter), "--unreal", str(engine)])
    monkeypatch.setattr(build.subprocess, "run", lambda command, **kw: calls.append(command))
    monkeypatch.setattr(build, "ensure_environment_level", lambda *args: prepared.append(args))
    build.main()
    command = calls[0]
    assert Path(command[1]) == ROOT / "ue-adapter/scripts/build_ue_project.py"
    assert Path(command[command.index("--project") + 1]) == adapter.resolve() / sync.PROJECT_RELATIVE / sync.PROJECT_FILE
    assert prepared == [(adapter.resolve(), engine.resolve())]
