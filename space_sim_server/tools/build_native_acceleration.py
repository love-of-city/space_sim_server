"""Build optional local acceleration DLLs; never installs/replaces Basilisk.

No third-party build dependencies. Uses the installed MSVC x64 toolchain.
Artifacts live under ignored run/native_acceleration and are source-hash named.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BUILD_FLAGS = ["/nologo", "/LD", "/O2", "/fp:strict", "/EHsc", "/std:c++17", "/MD"]
# Vendored MuJoCo 3.7.0 headers (the version Basilisk 2.11.1 embeds), so a clean
# checkout can build the MuJoCo-linked components without an offline Python
# environment. --mujoco-include still overrides this.
VENDORED_MUJOCO_INCLUDE = ROOT / "third_party/mujoco/include"


def source_digest(source: Path) -> str:
    return hashlib.sha256(source.read_bytes()).hexdigest()


def compiler_environment() -> dict[str, str]:
    if sys.platform != "win32":
        raise RuntimeError("This build helper currently supports Windows/MSVC x64 only.")
    where = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / (
        "Microsoft Visual Studio/Installer/vswhere.exe"
    )
    install = subprocess.check_output(
        [str(where), "-latest", "-products", "*", "-requires",
         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
        text=True,
    ).strip()
    if not install:
        raise RuntimeError("MSVC x64 build tools were not found.")
    vcvars = Path(install) / "VC/Auxiliary/Build/vcvars64.bat"
    # This shell call only reads the compiler environment; all file operations
    # and compiler arguments below are direct, not string-built filesystem commands.
    text = subprocess.check_output(
        f'cmd.exe /d /s /c ""{vcvars}" >nul && set"',
        text=True, errors="replace",
    )
    env = os.environ.copy()
    env.update(line.split("=", 1) for line in text.splitlines() if "=" in line and not line.startswith("="))
    return env


def build(name: str, include: Path | None = None) -> Path:
    source = ROOT / "native" / f"{name}.cpp"
    digest = source_digest(source)
    out = ROOT / "run/native_acceleration"
    out.mkdir(parents=True, exist_ok=True)
    work = out / f"{name}_{digest[:16]}"
    work.mkdir(exist_ok=True)
    dll = work / f"{name}.dll"
    env = compiler_environment()
    compiler = Path(env["VCToolsInstallDir"]) / "bin/Hostx64/x64/cl.exe"
    command = [str(compiler), *BUILD_FLAGS, str(source), f"/Fe:{dll}", f"/Fo:{work / (name+'.obj')}"]
    if include is not None:
        command.append(f"/I{include.resolve()}")
    subprocess.run(command, cwd=work, env=env, check=True)
    manifest = {
        "name": name, "abi": 1, "source_sha256": digest, "library": str(dll.relative_to(ROOT)),
        "library_sha256": hashlib.sha256(dll.read_bytes()).hexdigest(),
        "compiler_flags": BUILD_FLAGS, "python": sys.executable,
    }
    if include is not None:
        manifest["include"] = str(include.resolve())
        for filename in ("mujoco.h", "mjdata.h", "mjmodel.h"):
            manifest[f"{filename}_sha256"] = hashlib.sha256(
                (include / "mujoco" / filename).read_bytes()).hexdigest()
    (out / f"{name}.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return dll


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component", choices=("posture", "mjscene_probe", "local_mujoco_stepper", "all"),
                        default="posture",
                        help="'all' builds every component this machine can build")
    parser.add_argument("--mujoco-include", type=Path,
                        help="MuJoCo 3.7.0 include root; defaults to the vendored copy")
    options = parser.parse_args()
    if options.component == "all":
        # Components the running simulation needs. mjscene_probe is an offline
        # diagnostic and is deliberately excluded.
        for name in ("posture", "local_mujoco_stepper"):
            print(build(name, VENDORED_MUJOCO_INCLUDE))
        raise SystemExit(0)
    include = options.mujoco_include
    if options.component in {"mjscene_probe", "local_mujoco_stepper"} and include is None:
        include = VENDORED_MUJOCO_INCLUDE
        if not (include / "mujoco" / "mujoco.h").is_file():
            parser.error("--mujoco-include (exact MuJoCo 3.7.0 headers) is required for this component")
    print(build(options.component, include))
