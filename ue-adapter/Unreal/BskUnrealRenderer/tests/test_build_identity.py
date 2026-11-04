"""Content and loaded-module provenance must reject stale capture builds."""
import importlib.util
import json
from pathlib import Path

import pytest

_path = Path(__file__).resolve().parents[3] / "Adapters/bsk_render_adapter/build_identity.py"
_spec = importlib.util.spec_from_file_location("identity_under_test", _path)
identity = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(identity)


@pytest.fixture
def build(tmp_path):
    project, engine = tmp_path / "project", tmp_path / "engine"
    plugin = project / "Plugins/BskUnrealRuntime"
    (plugin / "Source").mkdir(parents=True)
    (plugin / "Shaders").mkdir()
    (plugin / "BskUnrealRuntime.uplugin").write_text("{}")
    (plugin / "Source/Runtime.cpp").write_text("source")
    (plugin / "Shaders/Camera.usf").write_text("shader")
    (project / "BskUnrealRenderer.uproject").write_text("{}")
    (engine / "Engine/Build").mkdir(parents=True)
    (engine / "Engine/Build/Build.version").write_text(json.dumps(
        {"MajorVersion": 5, "MinorVersion": 6, "PatchVersion": 1, "Changelist": 44394996}))
    engine_binaries = engine / "Engine/Binaries/Linux"
    engine_binaries.mkdir(parents=True)
    (engine_binaries / "UnrealEditor.modules").write_text('{"BuildId":"test-engine"}')
    for directory, modules in [(project / "Binaries/Linux", {"BskUnrealRenderer": "project.so"}),
                               (plugin / "Binaries/Linux", {name: name + ".so" for name in identity.MODULE_NAMES})]:
        directory.mkdir(parents=True)
        for filename in modules.values():
            (directory / filename).write_bytes(filename.encode())
        (directory / "UnrealEditor.modules").write_text(json.dumps({"BuildId": "test-engine", "Modules": modules}))
    identity.write_build_manifest(project, engine)
    return project, engine


@pytest.mark.parametrize("relative", ["Plugins/BskUnrealRuntime/Source/Runtime.cpp",
                                     "Plugins/BskUnrealRuntime/Shaders/Camera.usf",
                                     "Plugins/BskUnrealRuntime/BskUnrealRuntime.uplugin"])
def test_source_shader_and_descriptor_changes_require_rebuild(build, relative):
    project, engine = build
    assert identity.verify_build_manifest(project, engine, identity.default_manifest_path(project, "Linux"))["schema"] == identity.SCHEMA
    (project / relative).write_text("modified after build")
    with pytest.raises(RuntimeError, match="rebuild"):
        identity.verify_build_manifest(project, engine, identity.default_manifest_path(project, "Linux"))


def test_binary_or_engine_replacement_is_rejected(build):
    project, engine = build
    binary = project / "Plugins/BskUnrealRuntime/Binaries/Linux/BskCameraShaders.so"
    binary.write_bytes(b"another build")
    with pytest.raises(RuntimeError, match="binary"):
        identity.verify_build_manifest(project, engine, identity.default_manifest_path(project, "Linux"))
    identity.write_build_manifest(project, engine)
    (engine / "Engine/Build/Build.version").write_text('{"MajorVersion": 6}')
    with pytest.raises(RuntimeError, match="Engine"):
        identity.verify_build_manifest(project, engine, identity.default_manifest_path(project, "Linux"))


def test_documents_do_not_invalidate_build_and_paths_cannot_escape(build):
    project, engine = build
    (project / "README.md").write_text("Updated documentation")
    assert identity.verify_build_manifest(project, engine, identity.default_manifest_path(project, "Linux"))
    with pytest.raises(ValueError, match="escapes"):
        identity.contained_path(project, "../outside.so")


def test_build_log_timing_does_not_change_content_identity(build):
    project, engine = build
    first = identity.create_build_manifest(project, engine, toolchain={"compiler": "clang-18", "build_log_sha256": "a" * 64})
    second = identity.create_build_manifest(project, engine, toolchain={"compiler": "clang-18", "build_log_sha256": "b" * 64})
    assert first["build_id"] == second["build_id"]
    assert first["evidence"] != second["evidence"]


def test_runtime_report_checks_pid_and_actual_module_paths(build, tmp_path):
    project, engine = build
    manifest = identity.verify_build_manifest(project, engine, identity.default_manifest_path(project, "Linux"))
    report = {"schema": identity.RUNTIME_SCHEMA, "pid": 123, "engine_version": "5.6.1-44394996+++UE5+Release-5.6", "monolithic": False,
              "modules": {name: str(project / manifest["identity"]["modules"][name]["path"])
                          for name in identity.MODULE_NAMES}}
    path = tmp_path / "runtime.json"
    path.write_text(json.dumps(report))
    assert identity.verify_runtime_report(path, manifest, project, expected_pid=123, check_process_maps=False)
    with pytest.raises(RuntimeError, match="different process"):
        identity.verify_runtime_report(path, manifest, project, expected_pid=456, check_process_maps=False)
    report["modules"]["BskCameraShaders"] = str(tmp_path / "old/BskCameraShaders.so")
    path.write_text(json.dumps(report))
    with pytest.raises(RuntimeError, match="module path"):
        identity.verify_runtime_report(path, manifest, project, expected_pid=123, check_process_maps=False)


def test_shipping_identity_requires_shipping_product_and_checks_monolithic_report(build, tmp_path):
    project, engine = build
    directory = project / "Binaries/Linux"
    (directory / "BskUnrealRenderer").write_bytes(b"development executable")
    with pytest.raises(FileNotFoundError, match="Shipping"):
        identity.create_build_manifest(project, engine, target="BskUnrealRenderer", configuration="Shipping")
    shipping = directory / "BskUnrealRenderer-Linux-Shipping"
    shipping.write_bytes(b"shipping executable")
    manifest = identity.create_build_manifest(project, engine, target="BskUnrealRenderer", configuration="Shipping")
    assert list(manifest["identity"]["binaries"]) == ["Binaries/Linux/" + shipping.name]
    report = {"schema": identity.RUNTIME_SCHEMA, "pid": 123, "engine_version": "5.6.1-44394996+++UE5+Release-5.6",
              "monolithic": True, "modules": {name: str(shipping) for name in identity.MODULE_NAMES}}
    path = tmp_path / "game-runtime.json"
    path.write_text(json.dumps(report))
    assert identity.verify_runtime_report(path, manifest, project, expected_pid=123, check_process_maps=False)
    report["modules"]["BskUnrealRuntime"] = str(directory / "BskUnrealRenderer")
    path.write_text(json.dumps(report))
    with pytest.raises(RuntimeError, match="module path"):
        identity.verify_runtime_report(path, manifest, project, expected_pid=123, check_process_maps=False)


def test_editor_debuggame_never_certifies_or_overwrites_development_products(build):
    project, engine = build
    development = identity.default_manifest_path(project, "Linux")
    previous = development.read_bytes()
    debug = identity.default_manifest_path(project, "Linux", configuration="DebugGame")
    assert debug != development
    with pytest.raises(FileNotFoundError, match="UnrealEditor-Linux-DebugGame.modules"):
        identity.write_build_manifest(project, engine, configuration="DebugGame")
    assert development.read_bytes() == previous
    registries = [project / "Binaries/Linux/UnrealEditor.modules", *project.glob("Plugins/**/Binaries/Linux/UnrealEditor.modules")]
    for registry in registries:
        document = json.loads(registry.read_text())
        for name, filename in list(document["Modules"].items()):
            debug_name = "debug-" + filename
            (registry.parent / debug_name).write_bytes(debug_name.encode())
            document["Modules"][name] = debug_name
        registry.with_name("UnrealEditor-Linux-DebugGame.modules").write_text(json.dumps(document))
    assert identity.write_build_manifest(project, engine, configuration="DebugGame") == debug
    result = identity.verify_build_manifest(project, engine, debug)
    assert all(Path(item["path"]).name.startswith("debug-") for item in result["identity"]["modules"].values())
    assert development.read_bytes() == previous
