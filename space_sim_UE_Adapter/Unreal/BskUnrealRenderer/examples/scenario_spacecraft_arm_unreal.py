"""Stream the external free-floating CubeSat + SO-101 MJScene model to UE."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass, macros

from bsk_render_adapter import BasiliskRenderBridge, SceneSettings


JOINTS = (
    ("so101_shoulder", "so101_shoulder_pan"),
    ("so101_upper_arm", "so101_shoulder_lift"),
    ("so101_lower_arm", "so101_elbow_flex"),
    ("so101_wrist", "so101_wrist_flex"),
    ("so101_gripper", "so101_wrist_roll"),
    ("so101_moving_jaw_so101_v1", "so101_gripper"),
)
ACTUATORS = (
    "so101_shoulder_pan_motor",
    "so101_shoulder_lift_motor",
    "so101_elbow_flex_motor",
    "so101_wrist_flex_motor",
    "so101_wrist_roll_motor",
    "so101_gripper_motor",
)
INITIAL_JOINT_POSITIONS = (0.0, -0.18, 0.22, -0.04, 0.0, 0.25)
TORQUE_AMPLITUDES = (0.12, 0.12, 0.10, 0.07, 0.05, 0.025)


def joint_torques(sim_seconds: float) -> tuple[float, ...]:
    """Return smooth bounded open-loop torques for a visual motion demo."""

    envelope = min(max(sim_seconds / 0.5, 0.0), 1.0)
    return tuple(
        envelope * amplitude * math.sin(2.0 * math.pi * 0.18 * sim_seconds + index * 0.65)
        for index, amplitude in enumerate(TORQUE_AMPLITUDES)
    )


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
    mjcf = model_root / "assets" / "cubesat_so101" / "cubesat_so101.xml"
    mesh_root = model_root / "assets" / "robotstudio_so101" / "assets"
    if not mjcf.is_file():
        raise FileNotFoundError(f"combined MJCF does not exist: {mjcf}")
    mesh_files = [str(path.resolve()) for path in sorted(mesh_root.glob("*.stl"))]

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("spacecraftArmProcess")
    task_name = "spacecraftArmTask"
    dynamics_period_ns = macros.sec2nano(0.001)
    process.addTask(simulation.CreateNewTask(task_name, dynamics_period_ns))

    scene = mujoco.MJScene.fromFile(str(mjcf), files=mesh_files)
    scene.ModelTag = "cubeSatSO101Scene"
    simulation.AddModelToTask(task_name, scene, 100)

    command_messages = []
    for actuator_name in ACTUATORS:
        message = messaging.SingleActuatorMsg()
        message.write(messaging.SingleActuatorMsgPayload(input=0.0))
        scene.getSingleActuator(actuator_name).actuatorInMsg.subscribeTo(message)
        command_messages.append(message)

    bridge = BasiliskRenderBridge(
        host=host,
        port=port,
        origin_object="so101/cubesat_bus",
        frame_period_ns=macros.sec2nano(1.0 / 30.0),
    )
    try:
        bridge.add_mj_scene(
            scene,
            namespace="so101",
            source_path=mjcf,
            mesh_asset_catalog=catalog,
            semantic_label="spacecraft_robot_link",
        )
        bridge.set_scene_settings(
            SceneSettings(
                origin_object_id="so101/cubesat_bus",
                default_camera_target="so101/so101_gripper",
                default_camera_distance_m=1.5,
                orbit_lines=False,
                trajectory_history=False,
                interpolation_delay_ms=67.0,
                max_extrapolation_ms=67.0,
            )
        )
        simulation.AddModelToTask(task_name, bridge, -100)
        simulation.InitializeSimulation()
        scene.getBody("cubesat_bus").setVelocity([0.01, -0.004, 0.002])
        for (body_name, joint_name), position in zip(JOINTS, INITIAL_JOINT_POSITIONS, strict=True):
            scene.getBody(body_name).getScalarJoint(joint_name).setPosition(position)

        wall_start = time.monotonic()
        frame_count = int(math.ceil(duration * 30.0))
        for frame in range(1, frame_count + 1):
            sim_seconds = min(frame / 30.0, duration)
            for message, torque in zip(command_messages, joint_torques(sim_seconds), strict=True):
                message.write(messaging.SingleActuatorMsgPayload(input=torque))
            deadline = wall_start + sim_seconds / simulation_rate
            time.sleep(max(0.0, deadline - time.monotonic()))
            simulation.ConfigureStopTime(macros.sec2nano(sim_seconds))
            simulation.ExecuteSimulation()
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
    print("CubeSat + SO-101 Basilisk/MJScene live stream finished")


if __name__ == "__main__":
    main()
