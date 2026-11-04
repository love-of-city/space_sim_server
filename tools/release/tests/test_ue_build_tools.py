"""Hermetic release inputs and report guards; these tests do not launch UE."""
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "ue-adapter/scripts"))
import build_ue_project as build
import check_test_report as junit
import package_ue_project as package
import prepare_cook_staging as cook
import run_ue_automation as automation


def fixture_project(tmp_path):
    project = tmp_path / "project"
    (project / "Config").mkdir(parents=True)
    (project / "BskUnrealRenderer.uproject").write_text("{}")
    (project / "Config/DefaultGame.ini").write_text(
        '[/Script/UnrealEd.ProjectPackagingSettings]\n+DirectoriesToAlwaysCook=(Path="/Game/BSK")\n')
    (project / "Source").mkdir()
    (project / "Source/main.cpp").write_text("// tracked source")
    asset = project / "Content/BSK/Generated/Base/mesh.uasset"
    asset.parent.mkdir(parents=True)
    asset.write_bytes(b"example asset")
    dirty = project / "Content/BSK/Generated/Random150_42/local.uasset"
    dirty.parent.mkdir(parents=True)
    dirty.write_bytes(b"undeclared local data")
    subprocess.run(["git", "init", "-q", str(project)], check=True)
    subprocess.run(["git", "-C", str(project), "add", "Source", "Config", "BskUnrealRenderer.uproject"], check=True)
    manifest = tmp_path / "assets.json"
    manifest.write_text(json.dumps({"schema": "ue-cook-inputs/1", "files": {
        asset.relative_to(project).as_posix(): hashlib.sha256(asset.read_bytes()).hexdigest()},
        "cook_directories": ["/Game/BSK/Generated/Base"], "runtime_assets": []}))
    return project, manifest, asset


def test_cook_excludes_local_generated_assets_and_copies_only_declared_inputs(tmp_path):
    project, manifest, asset = fixture_project(tmp_path)
    output = tmp_path / "isolated staging"
    result = cook.prepare(project, output, [manifest])
    assert (output / asset.relative_to(project)).read_bytes() == asset.read_bytes()
    assert not (output / "Content/BSK/Generated/Random150_42").exists()
    assert '/Game/BSK")' not in (output / "Config/DefaultGame.ini").read_text()
    assert result["validation"]["packaged_runtime"] == "not-run"
    runtime = json.loads((output / "cook-runtime-assets.json").read_text())
    assert runtime["assets"] == [{"object_path": "/Game/BSK/Generated/Base/mesh.mesh"}]
    with pytest.raises(ValueError, match="empty"):
        cook.prepare(project, output, [manifest])


def test_cook_staging_is_content_identical_after_input_relocation(tmp_path):
    project, manifest, _ = fixture_project(tmp_path)
    first = cook.prepare(project, tmp_path / "one", [manifest])
    relocated = tmp_path / "different user" / "relocated project"
    shutil.copytree(project, relocated)
    second = cook.prepare(relocated, tmp_path / "two", [manifest])
    assert first == second


def test_cook_runtime_objects_can_select_compiled_blueprint_class(tmp_path):
    project, manifest, asset = fixture_project(tmp_path)
    value = json.loads(manifest.read_text())
    value["runtime_objects"] = {asset.relative_to(project).as_posix(): "/Game/BSK/Generated/Base/mesh.mesh_C"}
    manifest.write_text(json.dumps(value))
    _, _, runtime = cook.load_assets([manifest], project)
    assert runtime == ["/Game/BSK/Generated/Base/mesh.mesh_C"]
    value["runtime_objects"][asset.relative_to(project).as_posix()] = "/Game/Other.Other_C"
    manifest.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="stay in its asset package"):
        cook.load_assets([manifest], project)


def test_cook_rejects_mutated_asset_before_writing_staging(tmp_path):
    project, manifest, asset = fixture_project(tmp_path)
    asset.write_bytes(b"changed")
    output = tmp_path / "staging"
    with pytest.raises(ValueError, match="hash mismatch"):
        cook.prepare(project, output, [manifest])
    assert not output.exists()


def test_cook_rejects_manifest_selection_that_omits_the_default_map(tmp_path):
    project, manifest, _ = fixture_project(tmp_path)
    (project / "Config/DefaultEngine.ini").write_text(
        '[/Script/EngineSettings.GameMapsSettings]\nGameDefaultMap=/Game/BSK/Maps/BskEnvironment\n')
    output = tmp_path / "staging"
    with pytest.raises(ValueError, match="omit the configured default map"):
        cook.prepare(project, output, [manifest])
    assert not output.exists()


@pytest.mark.parametrize("value", ["../outside", "/absolute", "Content/../../outside", "C:\\outside"])
def test_cook_paths_must_be_package_relative(tmp_path, value):
    with pytest.raises(ValueError):
        cook.contained(tmp_path, value)


@pytest.mark.parametrize("states", [[], ["Skipped"], ["NotRun"], ["Success", "Fail"]])
def test_automation_refuses_empty_or_incomplete_reports(tmp_path, states):
    (tmp_path / "index.json").write_text(json.dumps({"tests": [{"state": state} for state in states]}))
    with pytest.raises(RuntimeError):
        automation.assert_report(tmp_path)


def test_success_report_requires_actual_executed_cases(tmp_path):
    (tmp_path / "index.json").write_text(json.dumps({"tests": [{"state": "Success"}], "failed": 0, "notRun": 0}))
    assert automation.assert_report(tmp_path)["tests"] == 1


@pytest.mark.parametrize("body", ["", '<testcase><skipped/></testcase>', '<testcase><failure/></testcase>'])
def test_junit_layer_does_not_accept_silent_skip(tmp_path, body):
    path = tmp_path / "report.xml"
    path.write_text("<testsuite>" + body + "</testsuite>")
    with pytest.raises(ValueError):
        junit.check(path)


def test_engine_policy_checks_patch_and_changelist(tmp_path):
    (tmp_path / "Engine/Build").mkdir(parents=True)
    version = {"MajorVersion": 5, "MinorVersion": 6, "PatchVersion": 1, "Changelist": 44394996, "CompatibleChangelist": 43139311}
    (tmp_path / "Engine/Build/Build.version").write_text(json.dumps(version))
    policy = tmp_path / "policy.json"
    policy.write_text(json.dumps({"engine": {**version, "Changelist": 1}}))
    with pytest.raises(ValueError, match="Changelist mismatch"):
        build.validate_engine(tmp_path, policy)


def test_noop_build_reuses_only_unchanged_compiler_identity(tmp_path):
    engine = tmp_path / "Engine/Build"
    engine.mkdir(parents=True)
    (engine / "Build.version").write_text('{}')
    compiler = tmp_path / "Engine/Extras/bin/clang++"
    compiler.parent.mkdir(parents=True)
    compiler.write_bytes(b"compiler v1")
    full = build.toolchain_identity(f"Using clang ({compiler}) version 'clang 18.1.0'\nTotal time: 10 seconds", tmp_path)
    assert full["status"] == "recorded"
    assert build.toolchain_identity("Target is up to date\nTotal time: 2 seconds", tmp_path, full) == full
    compiler.write_bytes(b"compiler v2")
    assert build.toolchain_identity("Target is up to date", tmp_path, full)["status"] == "not_recorded"


@pytest.mark.parametrize("actual", [[], ["/Game/A.A"], ["/Game/A.A", "/Game/A.A"], ["/Game/A.A", "/Game/C.C"]])
def test_package_report_refuses_missing_duplicate_or_unexpected_assets(tmp_path, actual):
    manifest = tmp_path / "assets.json"
    manifest.write_text(json.dumps({"assets": [{"object_path": item} for item in ["/Game/A.A", "/Game/B.B"]]}))
    report = tmp_path / "report.json"
    report.write_text(json.dumps({"schema": "ue-cook-runtime-validation/1", "passed": True, "errors": [],
                                 "assets": [{"object_path": item, "loaded": True} for item in actual]}))
    with pytest.raises(RuntimeError, match="incomplete"):
        package.assert_runtime_report(report, manifest)
