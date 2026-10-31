"""Compatibility entry point for the shared UE build/identity tool on Linux."""
from __future__ import annotations
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend"))
from space_arm_platform.unreal_runtime import default_unreal_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unreal", type=Path, default=default_unreal_root())
    parser.add_argument("--adapter", type=Path, default=ROOT.parent / "ue-adapter")
    args = parser.parse_args()
    if not sys.platform.startswith("linux"):
        parser.error("Run this helper on Linux; Windows uses build.ps1")
    adapter = args.adapter.resolve()
    subprocess.run([sys.executable, str(adapter / "scripts/build_ue_project.py"),
                    "--engine", str(args.unreal.expanduser().resolve()),
                    "--project", str(adapter / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject")], check=True)


if __name__ == "__main__":
    main()
