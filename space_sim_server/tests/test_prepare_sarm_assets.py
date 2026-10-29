"""Unattended imports must not accept exit zero without verified mesh settings."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.assets import prepare_sarm_assets as prep


@pytest.fixture
def asset_environment(tmp_path, monkeypatch):
    project = tmp_path / "adapter/Unreal/BskUnrealRenderer"
    (project / "scripts").mkdir(parents=True)
    (project / "BskUnrealRenderer.uproject").touch()
    (project / "scripts/configure_imported_meshes.py").write_text("# fixture")
    mesh = project / "Content/BSK/Test/part.uasset"
    catalog = {"assets": {"part.obj": {"asset_path": "/Game/BSK/Test/part.part"}}}
    calls = []
    behavior = {"create_asset": True, "acknowledge": True}

    def run(command, **kwargs):
        calls.append(command)
        if "--catalog" in command:
            path = Path(command[command.index("--catalog") + 1])
            path.write_text(json.dumps(catalog), encoding="utf-8")
        elif "-run=ImportAssets" in command and behavior["create_asset"]:
            mesh.parent.mkdir(parents=True, exist_ok=True)
            mesh.write_bytes(b"test asset")
        elif "env" in kwargs and behavior["acknowledge"]:
            env = kwargs["env"]
            source = Path(env["BSK_MJCF_ASSET_CATALOG"])
            Path(env["BSK_MJCF_RESULT_PATH"]).write_text(json.dumps({
                "configured": 1, "catalog_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            }))

    monkeypatch.setattr(prep.subprocess, "run", run)
    monkeypatch.setattr(prep, "selected_scene", lambda *a, **k: tmp_path / "model.xml")
    monkeypatch.setattr(prep, "unreal_editor", lambda *a: tmp_path / "UnrealEditor")
    args = SimpleNamespace(adapter=tmp_path / "adapter", model_root=tmp_path,
                           template_id=prep.DEFAULT_TEMPLATE, unreal=tmp_path,
                           catalog_only=False, force=False)
    return args, project, calls, behavior


def test_successful_import_cached_but_missing_asset_requires_reimport(asset_environment):
    args, project, calls, _ = asset_environment
    prep.prepare(args)
    assert len(calls) == 3
    prep.prepare(args)
    assert len(calls) == 4  # only catalog regeneration
    (project / "Content/BSK/Test/part.uasset").unlink()
    prep.prepare(args)
    assert len(calls) == 7


@pytest.mark.parametrize("failure", ["create_asset", "acknowledge"])
def test_zero_exit_without_import_or_configuration_is_failure(asset_environment, failure):
    args, project, _, behavior = asset_environment
    behavior[failure] = False
    with pytest.raises((RuntimeError, FileNotFoundError)):
        prep.prepare(args)
    assert not list((project / "Saved/AssetImport").glob("*.linux-import.json"))


def test_catalog_only_does_not_require_ue(asset_environment, monkeypatch):
    args, project, calls, _ = asset_environment
    args.catalog_only = True
    monkeypatch.setattr(prep, "unreal_editor", lambda *a: pytest.fail("UE not installed"))
    prep.prepare(args)
    assert len(calls) == 1
    assert not list((project / "Saved/AssetImport").glob("*.linux-import.json"))


def test_failed_forced_reimport_invalidates_previous_success(asset_environment):
    args, project, _, behavior = asset_environment
    prep.prepare(args)
    args.force = True
    behavior["acknowledge"] = False
    with pytest.raises(FileNotFoundError):
        prep.prepare(args)
    assert not list((project / "Saved/AssetImport").glob("*.linux-import.json"))


def test_external_package_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        prep.asset_files(tmp_path, {"assets": {"mesh": {"asset_path": "/Engine/part.part"}}})
