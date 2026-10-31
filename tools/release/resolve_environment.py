"""Resolve hashed dependencies and optionally verify a new isolated Python environment.

Outputs are candidate evidence until installation and dependency checks succeed.
Never targets an existing environment and never changes the active simulation env.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from runtime_metadata import python_metadata, sha256

ROOT = Path(__file__).resolve().parents[2]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--uv", default="uv")
    parser.add_argument("--platform", choices=("linux", "windows"), required=True)
    parser.add_argument("--constraint", type=Path)
    parser.add_argument("--index", action="append", default=[])
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--install", action="store_true", help="Also create and validate output/environment; host platform must match")
    parser.add_argument("--timeout", type=int, default=600)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("Use an empty output directory; existing environments are never replaced")
    output.mkdir(parents=True, exist_ok=True)
    result = {"schema": "environment-reconstruction/1", "platform": args.platform,
              "resolution": "not-run", "clean_install": "not-run", "native_physics": "not-run", "ue_gpu": "not-run"}
    status = output / "environment-validation.json"
    lock = output / "requirements.lock"
    native = output / "native-requirements.in"
    native.write_text("bsk==2.11.1\nsetuptools>=68\n")
    command = [args.uv, "pip", "compile", "--generate-hashes", "--all-extras", "--python-version", "3.13",
               "--python-platform", "x86_64-manylinux_2_35" if args.platform == "linux" else "x86_64-pc-windows-msvc",
               str(ROOT / "server/pyproject.toml"), str(ROOT / "ue-adapter/pyproject.toml"), str(native), "-o", str(lock)]
    if args.constraint:
        command.extend(["--constraint", str(args.constraint.resolve())])
    if args.offline:
        command.append("--offline")
    for index in args.index:
        command.extend(["--index", index])
    try:
        with (output / "resolve.log").open("w") as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
        result.update(resolution="passed", lock_sha256=sha256(lock))
        if args.install:
            if (args.platform == "windows") != (sys.platform == "win32"):
                raise RuntimeError("Clean installation must run on the requested target platform")
            environment = output / "environment"
            subprocess.run([args.uv, "venv", "--python", str(args.python), str(environment)], check=True, timeout=60)
            executable = environment / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
            subprocess.run([args.uv, "pip", "sync", "--python", str(executable), "--require-hashes", str(lock)], check=True, timeout=args.timeout)
            subprocess.run([args.uv, "pip", "install", "--python", str(executable), "--no-deps", "--no-build-isolation",
                            "-e", str(ROOT / "server"), "-e", str(ROOT / "ue-adapter")], check=True, timeout=120)
            subprocess.run([args.uv, "pip", "check", "--python", str(executable)], check=True, timeout=60)
            subprocess.run([str(executable), str(ROOT / "server/scripts/check_basilisk.py")], check=True, timeout=60)
            result.update(clean_install="passed", environment=python_metadata(executable))
    except (subprocess.SubprocessError, OSError, RuntimeError) as error:
        result["error"] = str(error)
        raise
    finally:
        status.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
