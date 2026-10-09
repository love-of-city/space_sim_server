"""Render the native CubeSat + SO-101 Basilisk grasp maneuver in Unreal."""

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
import sys
import time
from types import ModuleType

def load_native_grasp_module(model_root: Path) -> ModuleType:
    """Load the original scenario without preloading Python MuJoCo on Windows.

    The scenario imports ``mujoco`` for post-run numerical analysis and MP4
    rendering.  The live UE path needs only its Basilisk/MJScene construction,
    controllers, trajectory and initialization helpers.  Loading both Python
    MuJoCo and Basilisk's MuJoCo extension into one Windows process currently
    produces a DLL-symbol collision, so a placeholder defers that optional API.
    """

    scenario_path = model_root / "scenarios" / ("scenario_sarm_grasp.py" if (model_root / "scenarios" / "scenario_sarm_grasp.py").is_file() else "scenario_cubesat_so101_grasp.py")
    if not scenario_path.is_file():
        raise FileNotFoundError(f"native grasp scenario does not exist: {scenario_path}")
    previous_mujoco = sys.modules.get("mujoco")
    if previous_mujoco is not None and hasattr(previous_mujoco, "MjModel"):
        raise RuntimeError("Python MuJoCo was loaded before the Basilisk grasp compatibility wrapper")
    placeholder = ModuleType("mujoco")
    sys.modules["mujoco"] = placeholder
    try:
        specification = importlib.util.spec_from_file_location("bsk_native_cubesat_so101_grasp", scenario_path)
        if specification is None or specification.loader is None:
            raise ImportError(f"cannot load native grasp scenario: {scenario_path}")
        module = importlib.util.module_from_spec(specification)
        sys.modules[specification.name] = module
        specification.loader.exec_module(module)
        return module
    except Exception:
        if previous_mujoco is None:
            sys.modules.pop("mujoco", None)
        else:
            sys.modules["mujoco"] = previous_mujoco
        raise


def run(
    model_root: Path,
    catalog: Path,
    host: str,
    port: int,
    duration: float,
    simulation_rate: float,
) -> None:
    if duration <= 0.0 or simulation_rate <= 0.0:
        raise ValueError("duration and simulation_rate must be positive")
    if not catalog.is_file():
        raise FileNotFoundError(f"asset catalog is missing: {catalog}")

    native = load_native_grasp_module(model_root)
    # Import the renderer only after the native scenario established the
    # Windows-safe MuJoCo/Basilisk DLL load order.
    from Basilisk.utilities import macros
    from bsk_render_adapter import BasiliskRenderBridge, CameraVisual, SceneSettings

    simulation, scene, dynamics_models, recorders = native._build_simulation()
    # Keep the original Python-owned controller/recorder wrappers alive for the
    # complete run, exactly as the native scenario's _run_recorded() does.
    keep_alive = (dynamics_models, recorders)
    bridge = BasiliskRenderBridge(
        host=host,
        port=port,
        origin_object="grasp/cubesat_bus",
        frame_period_ns=macros.sec2nano(1.0 / 30.0),
    )
    try:
        body_ids = bridge.add_mj_scene(
            scene,
            namespace="grasp",
            source_path=native.MODEL_PATH,
            mesh_asset_catalog=catalog,
            semantic_label="spacecraft_robot_link",
            camera_picture_in_picture=True,
            camera_capture_rate_hz=15.0,
            camera_pip_resolution=(480, 270),
            camera_picture_in_picture_start_slot=2,
            camera_display_names={"so101_wrist_cam": "SO-101 Wrist Camera"},
        )
        # Fixed body-mounted overview.  The wire camera basis is +X forward,
        # +Y left, +Z up; this attitude looks from the CubeSat toward the arm's
        # grasp workspace while remaining rigidly installed on the bus.
        bridge.add_camera(
            CameraVisual(
                camera_id="grasp/camera/spacecraft_overview",
                display_name="Spacecraft Overview",
                parent_id=body_ids["cubesat_bus"],
                position_body_m=(0.0, -0.22, 0.34),
                orientation_body_from_camera_wxyz=(
                    0.9602216126462713,
                    0.0191150104507704,
                    -0.0679365250286891,
                    0.2701734619637677,
                ),
                field_of_view_rad=math.radians(70.0),
                resolution=(480, 270),
                semantic_label="spacecraft_overview_camera",
                picture_in_picture=True,
                capture_rate_hz=15.0,
                picture_in_picture_slot=1,
            )
        )
        bridge.set_scene_settings(
            SceneSettings(
                origin_object_id="grasp/cubesat_bus",
                default_camera_target="grasp/so101_gripper",
                default_camera_distance_m=1.25,
                orbit_lines=False,
                trajectory_history=False,
                interpolation_delay_ms=67.0,
                max_extrapolation_ms=67.0,
            )
        )
        simulation.AddModelToTask("graspTask", bridge, -10_000)
        native._initialize_state(simulation, scene)

        wall_start = time.monotonic()
        frame_count = int(math.ceil(duration * 30.0))
        for frame in range(1, frame_count + 1):
            sim_seconds = min(frame / 30.0, duration)
            deadline = wall_start + sim_seconds / simulation_rate
            time.sleep(max(0.0, deadline - time.monotonic()))
            simulation.ConfigureStopTime(macros.sec2nano(sim_seconds))
            simulation.ExecuteSimulation()
        _ = keep_alive
    finally:
        bridge.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-root", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--simulation-rate", type=float, default=1.0)
    args = parser.parse_args()
    run(
        args.model_root.resolve(),
        args.catalog.resolve(),
        args.host,
        args.port,
        args.duration,
        args.simulation_rate,
    )
    print("Native CubeSat + SO-101 Basilisk grasp stream finished")


if __name__ == "__main__":
    main()
