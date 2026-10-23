"""Build this project's editor target using an existing Linux UE 5.6 installation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))
from space_arm_platform.unreal_runtime import default_unreal_root, unreal_editor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unreal", type=Path, default=default_unreal_root())
    parser.add_argument("--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter")
    args = parser.parse_args()
    if not sys.platform.startswith("linux"):
        parser.error("Run this helper on Linux; Windows uses the existing build.ps1")
    engine = args.unreal.expanduser().resolve()
    unreal_editor(engine)
    version = json.loads((engine / "Engine/Build/Build.version").read_text())
    if (version["MajorVersion"], version["MinorVersion"]) != (5, 6):
        parser.error("This project requires UE 5.6")
    project = args.adapter.resolve() / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject"
    if not project.is_file():
        parser.error(f"UE project missing: {project}")
    build = engine / "Engine/Build/BatchFiles/Linux/Build.sh"
    subprocess.run(["bash", str(build), "BskUnrealRendererEditor", "Linux", "Development",
                    f"-Project={project}", "-WaitMutex", "-NoHotReloadFromIDE"], check=True)


if __name__ == "__main__":
    main()
