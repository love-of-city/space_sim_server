"""Load only locally built, source-matched optional native helpers."""
from __future__ import annotations

import ctypes
from functools import lru_cache
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@lru_cache(maxsize=4)
def load_native_library(name: str):
    if name not in {"posture", "mjscene_probe", "local_mujoco_stepper"}:
        raise ValueError("unknown native acceleration helper")
    manifest = ROOT / "run/native_acceleration" / f"{name}.json"
    if not manifest.is_file():
        raise FileNotFoundError(f"Native {name} helper is not built. Run tools/build_native_acceleration.py.")
    document = json.loads(manifest.read_text(encoding="utf-8"))
    expected = hashlib.sha256((ROOT / "native" / f"{name}.cpp").read_bytes()).hexdigest()
    if document.get("abi") != 1 or document.get("source_sha256") != expected:
        raise RuntimeError(f"Native {name} helper is stale; rebuild it (no fallback attempted).")
    library = (ROOT / document["library"]).resolve()
    if not library.is_relative_to((ROOT / "run/native_acceleration").resolve()):
        raise RuntimeError("Native helper path must stay inside run/native_acceleration.")
    if hashlib.sha256(library.read_bytes()).hexdigest() != document.get("library_sha256"):
        raise RuntimeError("Native helper fingerprint mismatch; rebuild it.")
    return ctypes.CDLL(str(library))
