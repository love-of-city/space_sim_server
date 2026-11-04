"""Command line entry point for authoritative SARM teleoperation."""
from __future__ import annotations
from pathlib import Path
import sys
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "backend"))
import argparse
import json
import os
from space_arm_platform.sampling import DYNAMICS_HZ, DEFAULT_IK_HZ, DEFAULT_CAPTURE_HZ
from simulation.local_mujoco_stepper import DYNAMICS_BACKENDS, MAX_SUBSTEPS
from simulation.teleop_control import (
    IK_MODES, IK_MODE_IK_POSE
)
from simulation.teleop_session import (
    run
)



def main() -> None:
    workspace = PROJECT_ROOT.parent
    default_adapter = workspace / "ue-adapter"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-root", type=Path, default=default_adapter)
    parser.add_argument(
        "--model-root",
        type=Path,
        default=PROJECT_ROOT / "model" / "SARM" / "platform",
    )
    parser.add_argument(
        "--catalog",
        type=Path,
        help="Prepared UE catalog; defaults to the selected adapter/model catalog.",
    )
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8766)
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=5558)
    parser.add_argument("--duration", type=float, default=0.0, help="Optional finite runtime in seconds; 0 runs until stopped.")
    parser.add_argument("--simulation-rate", type=float, default=1.0)
    parser.add_argument("--capture-rate", type=float, default=DEFAULT_CAPTURE_HZ)
    parser.add_argument("--ik-rate", type=float, default=DEFAULT_IK_HZ)
    parser.add_argument("--dynamics-step", type=float, default=None,
                        help="Online rational clock period: 1/400 seconds; other periods are rejected")
    parser.add_argument(
        "--ik-mode",
        choices=IK_MODES,
        default=os.environ.get("SPACE_SIM_IK_MODE", IK_MODE_IK_POSE),
        help=(
            "ik_pose (default: single-pass damped differential IK); "
            "strict (hold an infeasible Cartesian direction rather than project it)"
        ),
    )
    parser.add_argument("--scene-instance", type=Path)
    parser.add_argument("--disable-attitude-control", action="store_true",
                        help="Leave rotors installed but command zero motor torque (A/B diagnostics)")
    parser.add_argument("--dynamics-backend", choices=DYNAMICS_BACKENDS, default=None,
                        help="local (default): fixed-step MuJoCo implicitfast in the orbital frame. "
                             "basilisk: MJScene + RKF45, kept as the accuracy reference. "
                             "Defaults to SPACE_SIM_DYNAMICS_BACKEND, else local.")
    parser.add_argument("--local-substeps", type=int, choices=range(1, MAX_SUBSTEPS + 1), default=None,
                        metavar=f"1..{MAX_SUBSTEPS}",
                        help="MuJoCo substeps per 400 Hz step for the local backend (SPACE_SIM_LOCAL_SUBSTEPS)")
    parser.add_argument("--orbital-mode", choices=("bsk", "linear_tidal"), default=None,
                        help="Local gravity: BSK field models (default) or legacy Earth tidal baseline. "
                             "Defaults to SPACE_SIM_ORBITAL_MODE, else bsk.")
    parser.add_argument("--atmospheric-drag", action="store_true",
                        help="Require explicitly configured drag parameters; no uncalibrated defaults are applied.")
    parser.add_argument("--orbital-config", type=Path,
                        help="JSON OrbitalPhysics override; otherwise use saved environment.physics (new scenes: J2, old scenes: point mass).")
    args = parser.parse_args()
    if args.ik_mode not in IK_MODES:
        parser.error("SPACE_SIM_IK_MODE must be ik_pose or strict; constrained has been removed. "
                     "Unset the old environment override or select --ik-mode ik_pose.")
    if args.catalog is None:
        catalog_name = "sarm_platform.catalog.json" if args.model_root.name == "platform" else "cubesat_so101.catalog.json"
        if args.scene_instance and args.scene_instance.is_file():
            selected_instance = json.loads(args.scene_instance.read_text(encoding="utf-8"))
            if selected_instance.get('template_id') in ('sarm-task-box-20261012', 'sarm-task-box-20261012-sdf', 'sarm-task-box-20261012-demo'):
                catalog_name = 'sarm_20261012.catalog.json'
            if selected_instance.get("template_id") == "sarm-task-box-contacts":
                catalog_name = "sarm_task_box.catalog.json"
            elif selected_instance.get("template_id") == "sarm-task-box-free-plugs":
                catalog_name = "sarm_task_box_plugs.catalog.json"
        args.catalog = args.adapter_root / "Unreal" / "BskUnrealRenderer" / "Saved" / "AssetImport" / catalog_name
    if (
        args.duration < 0
        or args.simulation_rate <= 0
        or args.capture_rate <= 0
        or args.ik_rate <= 0
        or args.ik_rate > DYNAMICS_HZ
    ):
        parser.error(
            "duration must be non-negative, simulation-rate and capture-rate must be positive; "
            "ik-rate must use the current 200 Hz control grid"
        )
    if not args.adapter_root.is_dir() or not args.model_root.is_dir() or not args.catalog.is_file():
        parser.error("adapter-root, model-root or prepared asset catalog is missing")
    if args.scene_instance and not args.scene_instance.is_file():
        parser.error("scene-instance file is missing")
    try:
        run(args)
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
