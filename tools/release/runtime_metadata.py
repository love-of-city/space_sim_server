"""Inspect actual release inputs without importing packages into the build process."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path


def sha256(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def python_metadata(executable: Path) -> dict:
    script = """import importlib.metadata,json,platform,sys,sysconfig
print(json.dumps(dict(version=platform.python_version(),implementation=platform.python_implementation(),
    abi=sysconfig.get_config_var('SOABI'),platform=sys.platform,machine=platform.machine(),
    packages=sorted((dict(name=d.metadata['Name'],version=d.version) for d in importlib.metadata.distributions()),key=lambda d:d['name'].lower()))))
"""
    completed = subprocess.run([str(executable), "-I", "-c", script], check=True, capture_output=True,
                               text=True, timeout=60)
    data = json.loads(completed.stdout)
    data["executable_sha256"] = sha256(executable.resolve())
    return data


def required_glibc(elf_roots: list[Path]) -> dict:
    readelf = shutil.which("readelf")
    if not readelf:
        raise RuntimeError("readelf is required to establish release ELF requirements")
    maximum: tuple[int, ...] = ()
    scanned = 0
    required_by = []
    seen = set()
    for root in elf_roots:
        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.is_symlink():
                continue
            identity = (path.stat().st_dev, path.stat().st_ino)
            if identity in seen:
                continue
            seen.add(identity)
            with path.open("rb") as stream:
                if stream.read(4) != b"\x7fELF":
                    continue
            output = subprocess.run([readelf, "--version-info", "--wide", str(path)],
                                    check=True, capture_output=True, text=True, timeout=30).stdout
            # Definitions identify provided versions, not the consumer's minimum.
            needs = output.split("Version needs section", 1)[1] if "Version needs section" in output else ""
            versions = [tuple(map(int, item.split("."))) for item in re.findall(r"Name: GLIBC_([0-9.]+)", needs)]
            scanned += 1
            if versions and max(versions) > maximum:
                maximum = max(versions)
                required_by = [str(path.relative_to(root))]
            elif versions and max(versions) == maximum:
                required_by.append(str(path.relative_to(root)))
    if not scanned:
        raise ValueError("No ELF input was scanned; cannot claim a glibc minimum")
    return {"minimum": ".".join(map(str, maximum)) if maximum else None,
            "elf_files_scanned": scanned, "required_by": required_by,
            "scope": "ELF symbol requirements; GPU driver and kernel compatibility require runtime validation"}


def collect(executable: Path, engine: Path, elf_roots: list[Path] | None = None) -> dict:
    version_path = engine / "Engine/Build/Build.version"
    engine_version = json.loads(version_path.read_text(encoding="utf-8-sig"))
    plugins = {}
    for name, rel in (("CelestialVault", "Experimental/CelestialVault/CelestialVault.uplugin"),
                      ("PixelStreaming2", "Media/PixelStreaming2/PixelStreaming2.uplugin")):
        path = engine / "Engine/Plugins" / rel
        if not path.is_file():
            raise FileNotFoundError(f"Required engine plugin descriptor missing: {path}")
        plugins[name] = {"sha256": sha256(path), "descriptor": json.loads(path.read_text(encoding="utf-8-sig"))}
    return {"schema": "release-runtime-inputs/1", "python": python_metadata(executable),
            "engine": engine_version, "engine_build_version_sha256": sha256(version_path),
            "engine_plugins": plugins, "glibc": required_glibc(elf_roots) if elf_roots else {"status": "not-scanned"},
            "certification": {"clean_install": "not-run", "gpu": "not-run"}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, required=True)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--elf-root", type=Path, action="append")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    data = collect(args.python, args.engine, args.elf_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "python": data["python"]["version"], "engine": data["engine"]}))


if __name__ == "__main__":
    main()
