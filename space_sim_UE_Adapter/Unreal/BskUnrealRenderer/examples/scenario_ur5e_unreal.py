"""Stream the repository UR5e MJScene model to Unreal in real time."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import time

from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass, macros

from bsk_render_adapter import BasiliskRenderBridge, SceneSettings


ACTUATORS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
HOME = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)
INITIAL_HOLD_SECONDS = 1.0


def sequential_joint_targets(
    sim_seconds: float,
    joint_angle_rad: float,
    move_seconds: float,
    hold_seconds: float,
) -> tuple[float, ...]:
    """Move each joint once by a fixed angle, in order, then hold it there."""

    if not math.isfinite(sim_seconds) or sim_seconds < 0.0:
        raise ValueError("sim_seconds must be finite and non-negative")
    if not math.isfinite(joint_angle_rad) or abs(joint_angle_rad) > math.radians(45.0):
        raise ValueError("joint_angle_rad must be finite and within +/-45 degrees")
    if not math.isfinite(move_seconds) or move_seconds <= 0.0:
        raise ValueError("move_seconds must be finite and positive")
    if not math.isfinite(hold_seconds) or hold_seconds < 0.0:
        raise ValueError("hold_seconds must be finite and non-negative")

    elapsed = sim_seconds - INITIAL_HOLD_SECONDS
    stage_seconds = move_seconds + hold_seconds
    targets = list(HOME)
    for index, home in enumerate(HOME):
        stage_time = elapsed - index * stage_seconds
        if stage_time <= 0.0:
            continue
        progress = min(stage_time / move_seconds, 1.0)
        # Cubic smoothstep has zero velocity at the start and end, avoiding an
        # actuator command step that could excite the position servo.
        smooth_progress = progress * progress * (3.0 - 2.0 * progress)
        targets[index] = home + joint_angle_rad * smooth_progress
    return tuple(targets)


def run(
    workspace: Path,
    host: str,
    port: int,
    duration: float,
    simulation_rate: float,
    joint_angle_degrees: float,
    move_seconds: float,
    hold_seconds: float,
) -> None:
    if duration <= 0.0 or simulation_rate <= 0.0:
        raise ValueError("duration and simulation_rate must be positive")
    joint_angle_rad = math.radians(joint_angle_degrees)
    # Validate the motion parameters before constructing the simulation.
    sequential_joint_targets(0.0, joint_angle_rad, move_seconds, hold_seconds)
    model_root = workspace / "test" / "model" / "arm" / "universal_robots_ur5e"
    mjcf = model_root / "scene.xml"
    catalog = workspace / "Unreal" / "BskUnrealRenderer" / "Config" / "BskAssets" / "ur5e.json"
    if not catalog.is_file():
        raise FileNotFoundError(f"asset catalog is missing: {catalog}; run prepare_ur5e_assets.ps1")

    previous_directory = Path.cwd()
    os.chdir(model_root)  # Basilisk MJScene currently resolves <include> relative to cwd.
    bridge = None
    try:
        simulation = SimulationBaseClass.SimBaseClass()
        process = simulation.CreateNewProcess("ur5e_process")
        task_name = "ur5e_task"
        task_period_ns = macros.sec2nano(1.0 / 30.0)
        process.addTask(simulation.CreateNewTask(task_name, task_period_ns))

        scene = mujoco.MJScene.fromFile("scene.xml")
        scene.ModelTag = "ur5e_mjscene"
        simulation.AddModelToTask(task_name, scene, 100)

        command_messages = []
        for actuator_name, value in zip(ACTUATORS, HOME):
            message = messaging.SingleActuatorMsg()
            message.write(messaging.SingleActuatorMsgPayload(input=value))
            scene.getSingleActuator(actuator_name).actuatorInMsg.subscribeTo(message)
            command_messages.append(message)

        bridge = BasiliskRenderBridge(host=host, port=port, origin_object="ur5e/base")
        bridge.add_mj_scene(
            scene,
            namespace="ur5e",
            source_path=mjcf,
            mesh_asset_catalog=catalog,
            semantic_label="robot_link",
        )
        bridge.set_scene_settings(
            SceneSettings(
                origin_object_id="ur5e/base",
                default_camera_target="ur5e/upper_arm_link",
                default_camera_distance_m=1.75,
                orbit_lines=False,
                trajectory_history=False,
                interpolation_delay_ms=67.0,
                max_extrapolation_ms=67.0,
            )
        )
        simulation.AddModelToTask(task_name, bridge, -100)
        simulation.InitializeSimulation()
        for body_name, joint_name, value in zip(
            ("shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"),
            JOINTS,
            HOME,
        ):
            scene.getBody(body_name).getScalarJoint(joint_name).setPosition(value)

        wall_start = time.monotonic()
        frame = 1
        total_frames = int(math.ceil(duration * 30.0))
        while frame <= total_frames:
            sim_seconds = frame / 30.0
            targets = sequential_joint_targets(
                sim_seconds,
                joint_angle_rad,
                move_seconds,
                hold_seconds,
            )
            for message, target in zip(command_messages, targets):
                message.write(messaging.SingleActuatorMsgPayload(input=target))
            deadline = wall_start + sim_seconds / simulation_rate
            time.sleep(max(0.0, deadline - time.monotonic()))
            simulation.ConfigureStopTime(frame * task_period_ns)
            simulation.ExecuteSimulation()
            frame += 1
    finally:
        if bridge is not None:
            bridge.close()
        os.chdir(previous_directory)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument("--duration", type=float, default=22.0)
    parser.add_argument("--simulation-rate", type=float, default=1.0)
    parser.add_argument(
        "--joint-angle-deg",
        type=float,
        default=20.0,
        help="fixed angle added to each joint, in sequence (default: 20 degrees)",
    )
    parser.add_argument(
        "--move-seconds",
        type=float,
        default=2.0,
        help="smooth movement duration for each joint (default: 2 seconds)",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.75,
        help="pause after each joint movement (default: 0.75 seconds)",
    )
    args = parser.parse_args()
    run(
        args.workspace.resolve(),
        args.host,
        args.port,
        args.duration,
        args.simulation_rate,
        args.joint_angle_deg,
        args.move_seconds,
        args.hold_seconds,
    )
    print("UR5e Basilisk/MJScene live stream finished")


if __name__ == "__main__":
    main()
