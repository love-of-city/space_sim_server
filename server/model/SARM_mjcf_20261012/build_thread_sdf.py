#!/usr/bin/env python3
"""Compile the local MuJoCo SDF plugin against the active mujoco wheel."""

from pathlib import Path
import subprocess

import mujoco


ROOT = Path(__file__).resolve().parent
PACKAGE = Path(mujoco.__file__).resolve().parent
LIBRARY = PACKAGE / ("libmujoco.so." + mujoco.__version__)
OUTPUT = ROOT / "libtwo_start_thread.so"

if not LIBRARY.exists():
    raise FileNotFoundError(LIBRARY)

command = [
    "g++", "-std=c++17", "-O2", "-fPIC", "-shared",
    str(ROOT / "thread_sdf.cpp"),
    "-I", str(PACKAGE / "include"),
    "-L", str(PACKAGE),
    "-l:" + LIBRARY.name,
    "-Wl,-rpath," + str(PACKAGE),
    "-o", str(OUTPUT),
]
subprocess.run(command, check=True)
print("built", OUTPUT)
