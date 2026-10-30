"""Resolve the local UE installation without assuming a Windows executable."""
from __future__ import annotations

import os
from pathlib import Path
import sys


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
