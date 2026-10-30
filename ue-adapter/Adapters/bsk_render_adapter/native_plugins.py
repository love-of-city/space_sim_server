"""Linux plugins bind to the one MuJoCo runtime already present in the process."""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import re
import sys


def loaded_mujoco_library(*, global_symbols: bool = False):
    if not sys.platform.startswith("linux"):
        raise RuntimeError("ELF runtime discovery requires Linux")
    candidates = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/"):
            continue
        name = re.sub(r"\\([0-7]{3})", lambda m: chr(int(m[1], 8)), fields[5])
        path = Path(name)
        if re.match(r"libmujoco(?:[.-])", path.name) and ".so" in path.name:
            candidates.add(path.resolve())
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one loaded MuJoCo runtime, found {sorted(map(str, candidates))}")
    runtime = candidates.pop()
    mode = os.RTLD_NOW | os.RTLD_NOLOAD
    if global_symbols:
        mode |= os.RTLD_GLOBAL
    library = ctypes.CDLL(str(runtime), mode=mode)
    library.mj_version.restype = ctypes.c_int
    if library.mj_version() != 3007000:
        raise RuntimeError("SARM native helpers require MuJoCo 3.7.0")
    return library
