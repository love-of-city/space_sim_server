"""Build an explicit UE project and write its verified build identity.

Uses an installed engine; never downloads an engine or changes a Python environment.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ADAPTER = Path(__file__).resolve().parents[1]
ENGINE_POLICY = ADAPTER.parent / "manifests/environments" / ("windows-runtime.json" if sys.platform == "win32" else "linux-runtime.json")


def host_platform() -> str:
    if sys.platform.startswith("linux"):
        return "Linux"
    if sys.platform == "win32":
        return "Win64"
    raise RuntimeError("Supported build hosts are Linux and Windows")


def validate_engine(engine: Path, policy: Path | None = None) -> dict:
    version = json.loads((engine / "Engine/Build/Build.version").read_text(encoding="utf-8-sig"))
    if (version.get("MajorVersion"), version.get("MinorVersion")) != (5, 6):
        raise ValueError("SpaceSim requires the validated UE 5.6 engine family")
    if policy is not None:
        settings = json.loads(policy.read_text(encoding="utf-8"))
        expected = settings["engine"]
        for key in ("MajorVersion", "MinorVersion", "PatchVersion", "Changelist", "CompatibleChangelist"):
            if version.get(key) != expected.get(key):
                raise ValueError(f"Engine {key} mismatch: {version.get(key)} != {expected.get(key)}")
        descriptors = {"CelestialVault": "Experimental/CelestialVault/CelestialVault.uplugin",
                       "PixelStreaming2": "Media/PixelStreaming2/PixelStreaming2.uplugin"}
        for name in settings.get("required_engine_plugins", []):
            descriptor = engine / "Engine/Plugins" / descriptors[name]
            if not descriptor.is_file():
                raise FileNotFoundError(descriptor)
            recorded = settings.get("engine_plugin_descriptor_sha256", {}).get(name)
            if recorded and hashlib.sha256(descriptor.read_bytes()).hexdigest() != recorded:
                raise ValueError(f"Engine plugin descriptor mismatch: {name}")
    return version


def build_command(engine: Path, project: Path, platform: str, target: str, configuration: str) -> list[str]:
    script = engine / "Engine/Build/BatchFiles" / ("Linux/Build.sh" if platform == "Linux" else "Build.bat")
    if not script.is_file():
        raise FileNotFoundError(script)
    return (["bash", str(script)] if platform == "Linux" else [str(script)]) + [
        target, platform, configuration, f"-Project={project}", "-WaitMutex", "-NoHotReloadFromIDE", "-NoUBA"]


def file_digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def toolchain_identity(log_text: str, engine: Path, previous: dict | None = None) -> dict:
    """Retain proven compiler identity on an incremental no-op, not log timing."""
    engine_sha = file_digest(engine / "Engine/Build/Build.version")
    lines = sorted(set(line.strip().replace(str(engine), "<engine>") for line in log_text.splitlines()
                       if any(term in line.lower() for term in ("using clang", "using compiler", "toolchain", "using sdk", "visual studio", "windows sdk"))))
    binaries = {}
    for value in re.findall(r"Using (?:clang|compiler|llvm-ar) \(([^)]+)\)", log_text):
        path = Path(value)
        if path.is_file() and path.resolve().is_relative_to(engine.resolve()):
            binaries[path.resolve().relative_to(engine.resolve()).as_posix()] = file_digest(path)
    if not lines and previous and previous.get("engine_build_version_sha256") == engine_sha and previous.get("compiler_binaries"):
        if all(not Path(rel).is_absolute() and ".." not in Path(rel).parts
               and (engine / rel).resolve().is_relative_to(engine.resolve())
               and (engine / rel).is_file() and file_digest(engine / rel) == sha
               for rel, sha in previous["compiler_binaries"].items()):
            return previous
    return {"status": "recorded" if lines else "not_recorded", "source": "UBT build output",
            "details": lines, "engine_build_version_sha256": engine_sha,
            "compiler_binaries": dict(sorted(binaries.items()))}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", "--unreal", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=ADAPTER / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject")
    parser.add_argument("--target", choices=("BskUnrealRendererEditor", "BskUnrealRenderer"), default="BskUnrealRendererEditor")
    parser.add_argument("--configuration", choices=("Development", "DebugGame", "Shipping"), default="Development")
    parser.add_argument("--engine-policy", type=Path, default=ENGINE_POLICY)
    args = parser.parse_args(argv)
    engine, project = args.engine.expanduser().resolve(), args.project.resolve()
    if not project.is_file():
        parser.error(f"Project does not exist: {project}")
    platform = host_platform()
    validate_engine(engine, args.engine_policy)
    log_dir = project.parent / "Saved/BuildEvidence"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{args.target}-{platform}-{args.configuration}.log"
    toolchain_path = log_path.with_suffix(".toolchain.json")
    previous = json.loads(toolchain_path.read_text()) if toolchain_path.exists() else None
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(build_command(engine, project, platform, args.target, args.configuration),
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        if process.wait():
            raise subprocess.CalledProcessError(process.returncode, process.args)
    log_text = log_path.read_text(encoding="utf-8")
    toolchain_path.write_text(json.dumps(toolchain_identity(log_text, engine, previous), indent=2) + "\n")
    log_path.with_suffix(".evidence.json").write_text(json.dumps({
        "build_log_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest()}, indent=2) + "\n")
    subprocess.run([sys.executable, str(ADAPTER / "scripts/write_ue_build_manifest.py"),
                    "--project", str(project), "--engine", str(engine), "--platform", platform,
                    "--target", args.target, "--configuration", args.configuration,
                    "--toolchain-json", str(toolchain_path)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
