"""Create the editable environment map once, after building the Editor target.

Uses UE's own Python commandlet, never the system Python's unreal module.
An existing map is always preserved, including artist edits.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time

MAP_PACKAGE = "/Game/BSK/Maps/BskEnvironment"
MAP_RELATIVE = Path("Content/BSK/Maps/BskEnvironment.umap")
PACKAGE_MAGIC = (bytes.fromhex("c1832a9e"), bytes.fromhex("9e2a83c1"))
check_external_assets = runpy.run_path(str(Path(__file__).with_name("check_external_environment_assets.py")))["check_external_assets"]


def map_ready(project: Path) -> bool:
    path = project / MAP_RELATIVE
    if not path.exists():
        return False
    if not path.is_file():
        raise RuntimeError(f"Environment map is not a file: {path}")
    with path.open("rb") as stream:
        header = stream.read(200)
    if header.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise RuntimeError(f"Environment map is a Git LFS pointer: {path}; run git lfs pull")
    if len(header) < 128 or header[:4] not in PACKAGE_MAGIC:
        raise RuntimeError(f"Environment map is not a valid UE package header: {path}; refusing to overwrite it")
    return True


def prepare(project: Path, unreal: Path, *, verify=False) -> bool:
    project = project.expanduser().resolve()
    check_external_assets(project)
    existed = map_ready(project)
    if existed and not verify:
        print(f"Preserving saved environment map: {project / MAP_RELATIVE}")
        return False
    uproject = project / "BskUnrealRenderer.uproject"
    script = project / "scripts/create_environment_level.py"
    for path in (uproject, script):
        if not path.is_file():
            raise FileNotFoundError(path)
    platform = "Win64" if sys.platform == "win32" else "Linux"
    executable = "UnrealEditor-Cmd.exe" if sys.platform == "win32" else "UnrealEditor-Cmd"
    editor = unreal.expanduser().resolve() / "Engine/Binaries" / platform / executable
    if not editor.is_file() and sys.platform.startswith("linux"):
        editor = editor.with_name("UnrealEditor")
    if not editor.is_file():
        raise FileNotFoundError(f"UE editor commandlet missing: {editor}")
    # A first batch launch may have several workers. Only one UE process may
    # author the package; the others wait for its saved result.
    lock = project / "Saved/EnvironmentPreparation.lock"
    lock.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + 600
    while True:
        try:
            descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            break
        except FileExistsError:
            if map_ready(project) and not verify:
                return False
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Environment preparation is locked: {lock}; stop the authoring process and remove a stale lock")
            time.sleep(0.2)
    try:
        with os.fdopen(descriptor, "w") as stream:
            stream.write(str(os.getpid()))
        if map_ready(project) and not verify:
            return False
        subprocess.run([str(editor), str(uproject), "-run=pythonscript", f"-script={script}",
                        "-unattended", "-nop4", "-nosplash", "-NullRHI"], check=True)
        if not map_ready(project):
            raise RuntimeError("UE did not create BskEnvironment.umap; build the Editor target first and inspect the UE log")
    finally:
        lock.unlink(missing_ok=True)
    print(f"{'Verified' if existed else 'Created'} editable environment map: {project / MAP_RELATIVE}")
    return not existed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--unreal", type=Path, required=True)
    parser.add_argument("--verify", action="store_true",
                        help="Load an existing level in UE and validate environment bindings without replacing artist edits")
    args = parser.parse_args()
    prepare(args.project, args.unreal, verify=args.verify)


if __name__ == "__main__":
    main()
