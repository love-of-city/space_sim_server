"""Resolve the local UE installation without assuming a Windows executable."""
from __future__ import annotations

import os
from pathlib import Path
import sys
import subprocess

ENVIRONMENT_MAP = "/Game/BSK/Maps/BskEnvironment"


def check_external_environment_assets(adapter: Path) -> None:
    """Check the independently distributed environment before launching UE."""
    project = adapter / "Unreal/BskUnrealRenderer"
    checker = project / "scripts/check_external_environment_assets.py"
    if not checker.is_file():
        raise FileNotFoundError(f"External environment preflight missing: {checker}; synchronize the adapter source")
    subprocess.run([sys.executable, str(checker), "--project", str(project)], check=True)


def ensure_environment_level(adapter: Path, unreal: Path, *, prepare: bool = True) -> Path:
    """Validate external assets and prepare only a missing, never an authored, map."""
    check_external_environment_assets(adapter)
    project = adapter / "Unreal/BskUnrealRenderer"
    map_path = project / "Content/BSK/Maps/BskEnvironment.umap"
    if map_path.is_file():
        with map_path.open("rb") as stream:
            header = stream.read(200)
        if header.startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise RuntimeError(f"Environment map is a Git LFS pointer: {map_path}; run git lfs pull")
        if len(header) < 128 or header[:4] not in (bytes.fromhex("c1832a9e"), bytes.fromhex("9e2a83c1")):
            raise RuntimeError(f"Environment map is not a valid UE package header: {map_path}; refusing to overwrite it")
        return map_path
    if not prepare:
        raise FileNotFoundError("Editable environment level missing; build the Editor target with "
                                "tools/assets/build_unreal_linux.py first")
    preparation = project / "scripts/prepare_environment_level.py"
    if not preparation.is_file():
        raise FileNotFoundError(f"Environment preparation missing: {preparation}; synchronize the adapter source and rebuild")
    subprocess.run([sys.executable, str(preparation), "--project", str(project),
                    "--unreal", str(unreal)], check=True)
    if not map_path.is_file():
        raise RuntimeError(f"Environment map was not created: {map_path}; build the Editor target first")
    return ensure_environment_level(adapter, unreal)


def default_unreal_root() -> Path:
    if os.environ.get("UE56_ROOT"):
        return Path(os.environ["UE56_ROOT"]).expanduser()
    if sys.platform == "win32":
        return Path("C:/Program Files/Epic Games/UE_5.6")
    return Path.home() / "engines/UE_5.6"


def unreal_editor(root: Path) -> Path:
    root = root.expanduser().resolve()
    if sys.platform == "win32":
        candidates = [root / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"]
    elif sys.platform.startswith("linux"):
        candidates = [root / "Engine/Binaries/Linux" / name
                      for name in ("UnrealEditor-Cmd", "UnrealEditor")]
    else:
        raise RuntimeError("UE launcher supports Windows and Linux only")
    for candidate in candidates:
        if candidate.is_file():
            if os.name != "nt" and not os.access(candidate, os.X_OK):
                raise PermissionError(f"UE executable lacks execute permission: {candidate}")
            return candidate
    raise FileNotFoundError(f"UE editor missing under {root}; set UE56_ROOT or pass --unreal")
