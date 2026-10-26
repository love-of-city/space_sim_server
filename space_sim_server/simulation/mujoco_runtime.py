"""Find the MuJoCo runtime already loaded by Basilisk; never load a second copy."""
from __future__ import annotations

from pathlib import Path
import sys


def basilisk_mujoco_runtime() -> Path:
    import Basilisk
    from Basilisk.simulation import mujoco as _bsk_mujoco  # noqa: F401

    if sys.platform == "win32":
        runtime = Path(Basilisk.__file__).resolve().parent / "mujoco.dll"
        if not runtime.is_file():
            raise FileNotFoundError(f"Basilisk MuJoCo runtime not found: {runtime}")
        return runtime
    if not sys.platform.startswith("linux"):
        raise RuntimeError("Native MuJoCo runtime discovery supports Windows and Linux only")

    from bsk_render_adapter.native_plugins import loaded_mujoco_library
    return Path(loaded_mujoco_library()._name)
