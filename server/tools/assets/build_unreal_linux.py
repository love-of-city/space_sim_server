"""Compatibility entry point for the shared UE build/identity tool on Linux."""
from __future__ import annotations
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend"))
from space_arm_platform.unreal_runtime import default_unreal_root, ensure_environment_level


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unreal", type=Path, default=default_unreal_root())
    parser.add_argument("--adapter", type=Path, default=ROOT.parent / "ue-adapter")
    args = parser.parse_args()
    if not sys.platform.startswith("linux"):
        parser.error("Run this helper on Linux; Windows uses build.ps1")
    adapter = args.adapter.resolve()
    # The target may be a deployed UE-only copy. Use the source checkout
    # for the build helper, engine policy and identity implementation.
    build_tool = ROOT.parent / "ue-adapter/scripts/build_ue_project.py"
    subprocess.run([sys.executable, str(build_tool),
                    "--engine", str(args.unreal.expanduser().resolve()),
                    "--project", str(adapter / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject")], check=True)
    ensure_environment_level(adapter, args.unreal.expanduser().resolve())


if __name__ == "__main__":
    main()
