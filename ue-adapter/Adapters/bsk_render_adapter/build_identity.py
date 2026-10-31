"""Content identities for UE builds and the modules loaded by a capture process.

This module uses the standard library only. Build tools may load it by file path
without importing the simulation adapter or a locally installed Basilisk.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform as host_platform
import sys
from datetime import datetime, timezone


SCHEMA = "ue-build-manifest/1"
RUNTIME_SCHEMA = "ue-runtime-report/1"
MODULE_NAMES = ("BskUnrealRuntime", "BskCameraShaders")


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def content_id(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode("utf-8")).hexdigest()


def project_directory(project) -> Path:
    path = Path(project).expanduser().resolve()
    return path.parent if path.suffix == ".uproject" else path


def contained_path(root: Path, relative: str) -> Path:
    """Manifest paths are portable and cannot read files outside their root."""
    if not isinstance(relative, str) or not relative or "\\" in relative:
        raise ValueError(f"Invalid manifest path: {relative!r}")
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Manifest path escapes its root: {relative}")
    result = (root / path).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError(f"Manifest symlink escapes its root: {relative}")
    return result


def source_files(project) -> dict[str, str]:
    root = project_directory(project)
    files = list(root.glob("*.uproject"))
    for directory in (root / "Source", root / "Config"):
        if directory.is_dir():
            files.extend(p for p in directory.rglob("*") if p.is_file())
    for plugin in (root / "Plugins").glob("**/*.uplugin"):
        files.append(plugin)
        for name in ("Source", "Shaders", "Config"):
            directory = plugin.parent / name
            if directory.is_dir():
                files.extend(p for p in directory.rglob("*") if p.is_file())
    files = [p for p in files if not any(part in {"__pycache__", ".git"} for part in p.parts)]
    if not any(p.suffix == ".uproject" for p in files):
        raise FileNotFoundError(f"UE project is missing: {root}")
    return {p.relative_to(root).as_posix(): file_sha256(p) for p in sorted(set(files))}


def engine_identity(engine, platform: str) -> dict:
    root = Path(engine).expanduser().resolve()
    version_file = root / "Engine/Build/Build.version"
    version = json.loads(version_file.read_text(encoding="utf-8-sig"))
    registry = root / "Engine/Binaries" / platform / "UnrealEditor.modules"
    plugins = {}
    for relative in ("Media/PixelStreaming2", "Media/PixelCapture", "Experimental/CelestialVault"):
        plugin = root / "Engine/Plugins" / relative
        candidates = list(plugin.glob("*.uplugin"))
        binary_dir = plugin / "Binaries" / platform
        candidates.extend(binary_dir.glob("UnrealEditor.modules"))
        for suffix in ("*.so", "*.dll"):
            candidates.extend(binary_dir.glob(suffix))
        for path in sorted(candidates):
            plugins[path.relative_to(root).as_posix()] = file_sha256(path)
    return {"version": version, "build_version_sha256": file_sha256(version_file),
            "module_build_id": json.loads(registry.read_text()).get("BuildId") if registry.exists() else None,
            "plugin_files": plugins}


def default_manifest_path(project, platform=None, target="BskUnrealRendererEditor",
                          configuration="Development") -> Path:
    platform = platform or ("Win64" if os.name == "nt" else "Linux")
    name = ("ue-build-manifest.json" if target.endswith("Editor") else
            f"ue-build-{target}-{configuration}.json")
    return project_directory(project) / "Binaries" / platform / name


def build_products(project, platform: str, target: str, configuration: str) -> tuple[dict, dict]:
    root = project_directory(project)
    binaries, modules = {}, {}
    if target.endswith("Editor"):
        registries = [root / "Binaries" / platform / "UnrealEditor.modules"]
        registries += sorted((root / "Plugins").glob(f"**/Binaries/{platform}/UnrealEditor.modules"))
        for registry in registries:
            document = json.loads(registry.read_text(encoding="utf-8-sig"))
            binaries[registry.relative_to(root).as_posix()] = file_sha256(registry)
            for name, filename in document.get("Modules", {}).items():
                library = contained_path(registry.parent, filename)
                relative = library.relative_to(root).as_posix()
                binaries[relative] = file_sha256(library)
                modules[name] = {"path": relative, "sha256": binaries[relative],
                                 "engine_build_id": document.get("BuildId")}
        if not set(MODULE_NAMES).issubset(modules):
            raise FileNotFoundError("Build both BskUnrealRuntime and BskCameraShaders before recording provenance")
    else:
        directory = root / "Binaries" / platform
        suffix = ".exe" if platform == "Win64" else ""
        name = (target + suffix if configuration == "Development" else
                f"{target}-{platform}-{configuration}{suffix}")
        executable = directory / name
        if not executable.is_file():
            raise FileNotFoundError(f"Game executable not built: {directory} ({configuration})")
        binaries[executable.relative_to(root).as_posix()] = file_sha256(executable)
    return dict(sorted(binaries.items())), modules


def create_build_manifest(project, engine, platform="Linux", target="BskUnrealRendererEditor",
                          configuration="Development", toolchain=None) -> dict:
    if platform not in {"Linux", "Win64"}:
        raise ValueError("Supported build platforms: Linux, Win64")
    root = project_directory(project)
    sources = source_files(root)
    binaries, modules = build_products(root, platform, target, configuration)
    engine_info = engine_identity(engine, platform)
    for name, module in modules.items():
        if engine_info["module_build_id"] and module["engine_build_id"] != engine_info["module_build_id"]:
            raise ValueError(f"Module {name} was built for another Engine Build ID")
    toolchain_identity = dict(toolchain or {"status": "not_recorded"})
    build_log_sha256 = toolchain_identity.pop("build_log_sha256", None)
    identity = dict(platform=platform, architecture=host_platform.machine(), target=target,
                    configuration=configuration, engine=engine_info,
                    sources=sources, binaries=binaries, modules=modules,
                    toolchain=toolchain_identity)
    return {"schema": SCHEMA, "build_id": content_id(identity), "identity": identity,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "evidence": {"build_log_sha256": build_log_sha256}}


def write_build_manifest(project, engine, platform="Linux", target="BskUnrealRendererEditor",
                         configuration="Development", toolchain=None, output=None) -> Path:
    document = create_build_manifest(project, engine, platform, target, configuration, toolchain)
    path = Path(output) if output else default_manifest_path(project, platform, target, configuration)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return path


def verify_build_manifest(project, engine, manifest_path=None) -> dict:
    root = project_directory(project)
    path = Path(manifest_path) if manifest_path else default_manifest_path(root)
    if not path.is_file():
        raise RuntimeError(f"UE build manifest missing: {path}; run the full UE build before capture")
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema") != SCHEMA or not isinstance(document.get("identity"), dict):
        raise ValueError("Unsupported UE build manifest; rebuild the project")
    identity = document["identity"]
    if content_id(identity) != document.get("build_id"):
        raise ValueError("UE build manifest identity is corrupt")
    if source_files(root) != identity["sources"]:
        raise RuntimeError("UE source/Shader/config changed since the build; rebuild before capture")
    if engine_identity(engine, identity["platform"]) != identity["engine"]:
        raise RuntimeError("UE Engine Build ID/version differs from the build manifest")
    binaries, modules = build_products(root, identity["platform"], identity["target"], identity["configuration"])
    if binaries != identity["binaries"] or modules != identity["modules"]:
        raise RuntimeError("UE binary or module registry changed since the build; rebuild before capture")
    return document


def verify_runtime_report(report_path, manifest, project, *, expected_pid=None,
                          check_process_maps=True) -> dict:
    """Check actual module paths reported by the newly started UE process.

    Linux additionally rejects replaced/deleted mapped libraries. On Windows the
    UE module manager report supplies the paths; immutable builds are required.
    """
    report = json.loads(Path(report_path).read_text(encoding="utf-8-sig"))
    if report.get("schema") != RUNTIME_SCHEMA or not isinstance(report.get("pid"), int):
        raise RuntimeError("Invalid UE runtime identity report")
    pid = report["pid"]
    if pid <= 0 or (expected_pid is not None and pid != expected_pid):
        raise RuntimeError("UE runtime identity belongs to a different process")
    version = manifest["identity"]["engine"]["version"]
    expected_version = f"{version['MajorVersion']}.{version['MinorVersion']}.{version['PatchVersion']}"
    if version.get("Changelist"):
        expected_version += f"-{version['Changelist']}"
    actual_version = str(report.get("engine_version", ""))
    if (actual_version != expected_version and
            not actual_version.startswith((expected_version + "-", expected_version + "+"))):
        raise RuntimeError("Loaded UE engine version differs from the build")
    root = project_directory(project)
    is_editor = manifest["identity"]["target"].endswith("Editor")
    if report.get("monolithic") is not (not is_editor):
        raise RuntimeError("UE runtime linkage differs from the build target")
    expected_modules = manifest["identity"]["modules"]
    if not is_editor:
        binaries = manifest["identity"]["binaries"]
        if len(binaries) != 1:
            raise RuntimeError("Monolithic build must identify its single executable")
        executable, sha256 = next(iter(binaries.items()))
        expected_modules = {name: {"path": executable, "sha256": sha256} for name in MODULE_NAMES}
    maps = None
    if sys.platform.startswith("linux") and check_process_maps:
        maps = Path(f"/proc/{pid}/maps").read_text()
    for name in MODULE_NAMES:
        expected = expected_modules[name]
        actual = Path(report.get("modules", {}).get(name, ""))
        if not actual.is_absolute() or actual.resolve() != contained_path(root, expected["path"]):
            raise RuntimeError(f"Loaded UE module path differs: {name}")
        if file_sha256(actual) != expected["sha256"]:
            raise RuntimeError(f"Loaded UE module file changed: {name}")
        if maps is not None:
            paths = [fields[5].replace("\\040", " ") for line in maps.splitlines()
                     if len(fields := line.split(maxsplit=5)) == 6]
            if str(actual.resolve()) not in paths or str(actual.resolve()) + " (deleted)" in paths:
                raise RuntimeError(f"UE module is not mapped from the certified file: {name}")
    return report
