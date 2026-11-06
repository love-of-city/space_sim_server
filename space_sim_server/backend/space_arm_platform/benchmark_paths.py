"""Absolute evidence paths, including deep Windows output directories."""
from pathlib import Path
import sys


def windows_extended_path(path: str) -> str:
    """Convert an already absolute native Windows path without changing its target."""
    if path.startswith("\\\\?\\"):
        return path
    if path.startswith("\\\\"):
        return "\\\\?\\UNC\\" + path[2:]
    return "\\\\?\\" + path


def evidence_path(path) -> Path:
    resolved = Path(path).resolve()
    return Path(windows_extended_path(str(resolved))) if sys.platform == "win32" else resolved
