"""Run the authoritative CubeSat + SO-101 MJScene under live operator control.

This process owns all joint/contact dynamics.  The browser never moves UE
actors directly: it sends a bounded six-dimensional end-effector twist and a
gripper command to the platform backend.  This process projects the Cartesian
command through damped least-squares inverse kinematics into PID joint
references, and the existing bsk_render_adapter publishes the resulting world
state to UE.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import socket
import sys
import threading
import time
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "backend"))

from simulation.serial_chain_kinematics import (  # noqa: E402
    SerialChainKinematics,
    matrix_to_quaternion_wxyz,
)
from space_arm_platform.protocol import CONTROL_PROTOCOL, encode_packet, recv_socket  # noqa: E402


JOINT_MIN = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533])
JOINT_MAX = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.74385, 1.74533])
ARM_JOINT_VELOCITY_LIMIT = np.array([0.70, 0.70, 0.70, 0.90, 1.00])
ARM_JOINT_NAMES = (
    "so101_shoulder_pan",
    "so101_shoulder_lift",
    "so101_elbow_flex",
    "so101_wrist_flex",
    "so101_wrist_roll",
)


class SimulationControlClient:
    """Reconnectable latest-action client; its receive thread never touches BSK."""

    def __init__(self, host: str, port: int, simulation_id: str) -> None:
        self.host = host
        self.port = int(port)
        self.simulation_id = simulation_id
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._action: dict[str, Any] = self._neutral_action()
        self._received_monotonic = 0.0
        self._socket: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._socket is not None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._worker, name="teleop-action-receiver", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            connection = self._socket
            self._socket = None
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def latest_action(self) -> tuple[dict[str, Any], bool]:
        with self._lock:
            action = dict(self._action)
            action["end_effector_linear_velocity_body_m_s"] = list(
                self._action["end_effector_linear_velocity_body_m_s"]
            )
            action["end_effector_angular_velocity_body_rad_s"] = list(
                self._action["end_effector_angular_velocity_body_rad_s"]
            )
            stale = time.monotonic() - self._received_monotonic > 0.25
        if stale:
            action["deadman"] = False
            action["end_effector_linear_velocity_body_m_s"] = [0.0] * 3
            action["end_effector_angular_velocity_body_rad_s"] = [0.0] * 3
            action["gripper_velocity_rad_s"] = 0.0
        return action, stale

    def send_observation(self, message: dict[str, Any]) -> bool:
        with self._lock:
            connection = self._socket
        if connection is None:
            return False
        try:
            with self._send_lock:
                connection.sendall(encode_packet(message))
            return True
        except OSError:
            self._drop(connection)
            return False

    def _worker(self) -> None:
        while not self._stop.is_set():
            connection: socket.socket | None = None
            try:
                connection = socket.create_connection((self.host, self.port), timeout=1.0)
                connection.settimeout(1.0)
                with self._send_lock:
                    connection.sendall(
                        encode_packet(
                            {
                                "protocol": CONTROL_PROTOCOL,
                                "type": "sim_hello",
                                "simulation_id": self.simulation_id,
                                "capabilities": [
                                    "cartesian_twist_6d",
                                    "damped_least_squares_ik",
                                    "gripper_velocity",
                                    "joint_observation",
                                    "cartesian_observation",
                                ],
                            }
                        )
                    )
                with self._lock:
                    self._socket = connection
                while not self._stop.is_set():
                    try:
                        message = recv_socket(connection)
                    except socket.timeout:
                        continue
                    if self._valid_action(message):
                        with self._lock:
                            self._action = message
                            self._received_monotonic = time.monotonic()
            except (OSError, EOFError, ValueError):
                pass
            finally:
                if connection:
                    self._drop(connection)
            self._stop.wait(0.5)

    def _drop(self, connection: socket.socket) -> None:
        with self._lock:
            if self._socket is connection:
                self._socket = None
        try:
            connection.close()
        except OSError:
            pass

    @staticmethod
    def _valid_action(message: dict[str, Any]) -> bool:
        linear = message.get("end_effector_linear_velocity_body_m_s")
        angular = message.get("end_effector_angular_velocity_body_rad_s")
        return (
            message.get("protocol") == CONTROL_PROTOCOL
            and message.get("type") == "action"
            and message.get("control_frame", "spacecraft_body") == "spacecraft_body"
            and isinstance(linear, list)
            and len(linear) == 3
            and isinstance(angular, list)
            and len(angular) == 3
            and all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in [*linear, *angular]
            )
            and isinstance(message.get("gripper_velocity_rad_s"), (int, float))
        )

    @staticmethod
    def _neutral_action() -> dict[str, Any]:
        return {
            "protocol": CONTROL_PROTOCOL,
            "type": "action",
            "server_sequence": "0",
            "deadman": False,
            "control_frame": "spacecraft_body",
            "end_effector_linear_velocity_body_m_s": [0.0] * 3,
            "end_effector_angular_velocity_body_rad_s": [0.0] * 3,
            "gripper_velocity_rad_s": 0.0,
        }


class CartesianTeleopTarget:
    """Project Cartesian twist commands into bounded PID joint references."""

    def __init__(
        self,
        initial_position: np.ndarray,
        client: SimulationControlClient,
        kinematics: SerialChainKinematics,
    ) -> None:
        self.position = np.asarray(initial_position, dtype=float).copy()
        self.velocity = np.zeros(6)
        self.client = client
        self.kinematics = kinematics
        self.last_sim_seconds: float | None = None
        self.applied_sequence = "0"
        self.command_stale = True
        self.desired_twist = np.zeros(6)
        self.achieved_twist = np.zeros(6)
        self.residual_twist = np.zeros(6)
        self.jacobian_rank = 0

    def reference(self, sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        if self.last_sim_seconds is None:
            self.last_sim_seconds = sim_seconds
            return self.position.copy(), self.velocity.copy()
        dt = max(0.0, min(0.02, sim_seconds - self.last_sim_seconds))
        self.last_sim_seconds = sim_seconds
        action, stale = self.client.latest_action()
        enabled = bool(action.get("deadman")) and not stale
        self.desired_twist = np.array(
            [
                *action["end_effector_linear_velocity_body_m_s"],
                *action["end_effector_angular_velocity_body_rad_s"],
            ],
            dtype=float,
        )
        if enabled:
            result = self.kinematics.inverse_velocity(
                self.position[:5],
                self.desired_twist,
                joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
            )
            self.velocity[:5] = result.joint_velocity_rad_s
            self.velocity[5] = float(action["gripper_velocity_rad_s"])
            self.achieved_twist = result.achieved_twist
            self.residual_twist = result.residual_twist
            self.jacobian_rank = result.jacobian_rank
        else:
            self.velocity.fill(0.0)
            self.achieved_twist.fill(0.0)
            self.residual_twist.fill(0.0)
            self.jacobian_rank = int(np.linalg.matrix_rank(self.kinematics.jacobian(self.position[:5])))
        proposed = self.position + self.velocity * dt
        clipped = np.clip(proposed, JOINT_MIN, JOINT_MAX)
        at_limit = clipped != proposed
        self.velocity[at_limit] = 0.0
        self.position = clipped
        self.applied_sequence = str(action.get("server_sequence", "0"))
        self.command_stale = stale
        return self.position.copy(), self.velocity.copy()


def run(args: argparse.Namespace) -> None:
    adapter_root = args.adapter_root.resolve()
    ue_examples = adapter_root / "Unreal" / "BskUnrealRenderer" / "examples"
    sys.path.insert(0, str(adapter_root / "Adapters"))
    sys.path.insert(0, str(ue_examples))

    # This wrapper establishes the Windows-safe Basilisk/MuJoCo DLL load order.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    native = load_native_grasp_module(args.model_root.resolve())
    native.TIME_STEP = 0.002  # One authoritative 500 Hz dynamics step.

    client = SimulationControlClient(args.control_host, args.control_port, "so101-teleop")
    kinematics = SerialChainKinematics.from_mjcf(
        native.MODEL_PATH,
        base_body="cubesat_bus",
        joint_names=ARM_JOINT_NAMES,
        tool_site="so101_gripperframe",
    )
    targets = CartesianTeleopTarget(native.PREGRASP, client, kinematics)
    original_reference = native.JointTrajectoryPublisher.__dict__["reference"]
    native.JointTrajectoryPublisher.reference = classmethod(
        lambda _cls, seconds: targets.reference(seconds)
    )

    from Basilisk.utilities import macros
    from bsk_render_adapter import BasiliskRenderBridge, CameraVisual, SceneSettings

    client.start()
    bridge: BasiliskRenderBridge | None = None
    try:
        simulation, scene, dynamics_models, recorders = native._build_simulation()
        keep_alive = (dynamics_models, recorders)
        bridge = BasiliskRenderBridge(
            host=args.render_host,
            port=args.render_port,
            origin_object="teleop/cubesat_bus",
            frame_period_ns=macros.sec2nano(1.0 / 30.0),
        )
        body_ids = bridge.add_mj_scene(
            scene,
            namespace="teleop",
            source_path=native.MODEL_PATH,
            mesh_asset_catalog=args.catalog.resolve(),
            semantic_label="spacecraft_robot_link",
            camera_picture_in_picture=True,
            camera_capture_rate_hz=args.capture_rate,
            camera_capture_products=("rgb", "depth", "segmentation"),
            camera_pip_resolution=(640, 360),
            camera_picture_in_picture_start_slot=2,
            camera_display_names={"so101_wrist_cam": "SO-101 Wrist Camera"},
        )
        bridge.add_camera(
            CameraVisual(
                camera_id="teleop/camera/spacecraft_overview",
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
                resolution=(640, 360),
                semantic_label="spacecraft_overview_camera",
                picture_in_picture=True,
                capture_rate_hz=args.capture_rate,
                capture_products=("rgb", "depth", "segmentation"),
                picture_in_picture_slot=1,
            )
        )
        bridge.set_scene_settings(
            SceneSettings(
                origin_object_id="teleop/cubesat_bus",
                default_camera_target="teleop/so101_gripper",
                default_camera_distance_m=1.25,
                orbit_lines=False,
                trajectory_history=False,
                # 预览通道只保留很短的插值缓冲，并允许最多 50 ms 的视觉外推；
                # 训练采集由 UE 在权威帧上单独完成，不会记录这些平滑后的姿态。
                interpolation_delay_ms=15.0,
                max_extrapolation_ms=50.0,
            )
        )
        simulation.AddModelToTask("graspTask", bridge, -10_000)
        native._initialize_state(simulation, scene)
        joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in native.JOINTS]

        wall_start = time.monotonic()
        frame_count = int(math.ceil(args.duration * 30.0))
        for frame in range(1, frame_count + 1):
            sim_seconds = min(frame / 30.0, args.duration)
            deadline = wall_start + sim_seconds / args.simulation_rate
            time.sleep(max(0.0, deadline - time.monotonic()))
            simulation.ConfigureStopTime(macros.sec2nano(sim_seconds))
            simulation.ExecuteSimulation()
            render_frame_id = bridge.last_published_frame_id
            render_sim_time_ns = bridge.last_published_sim_time_ns
            joint_position = [float(joint.stateOutMsg.read().state) for joint in joints]
            joint_velocity = [float(joint.stateDotOutMsg.read().state) for joint in joints]
            end_effector_position, end_effector_rotation = kinematics.forward(
                np.asarray(joint_position[:5])
            )
            end_effector_twist = kinematics.jacobian(
                np.asarray(joint_position[:5])
            ) @ np.asarray(joint_velocity[:5])
            client.send_observation(
                {
                    "protocol": CONTROL_PROTOCOL,
                    "type": "observation",
                    "simulation_id": "so101-teleop",
                    "step_id": str(frame),
                    "render_frame_id": str(render_frame_id),
                    # 使用权威渲染帧的离散时间，保证状态和相机产品可逐帧严格配对。
                    "sim_time_ns": str(render_sim_time_ns),
                    "wall_time_ns": str(time.time_ns()),
                    "applied_action_sequence": targets.applied_sequence,
                    "joint_position_rad": joint_position,
                    "joint_velocity_rad_s": joint_velocity,
                    "target_joint_position_rad": targets.position.tolist(),
                    "end_effector_position_body_m": end_effector_position.tolist(),
                    "end_effector_orientation_body_wxyz": matrix_to_quaternion_wxyz(
                        end_effector_rotation
                    ).tolist(),
                    "end_effector_twist_body": end_effector_twist.tolist(),
                    "cartesian_command_residual": targets.residual_twist.tolist(),
                    "jacobian_rank": targets.jacobian_rank,
                    "command_stale": targets.command_stale,
                }
            )
        _ = keep_alive
    finally:
        native.JointTrajectoryPublisher.reference = original_reference
        if bridge:
            bridge.close()
        client.close()


def main() -> None:
    workspace = PROJECT_ROOT.parent
    default_adapter = workspace / "space_sim_UE_adapter"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-root", type=Path, default=default_adapter)
    parser.add_argument("--model-root", type=Path, default=workspace / "test" / "model" / "spacecraft_and_arm")
    parser.add_argument(
        "--catalog",
        type=Path,
        default=default_adapter / "Unreal" / "BskUnrealRenderer" / "Saved" / "AssetImport" / "cubesat_so101.catalog.json",
    )
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8766)
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=5558)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--simulation-rate", type=float, default=1.0)
    parser.add_argument("--capture-rate", type=float, default=10.0)
    args = parser.parse_args()
    if args.duration <= 0 or args.simulation_rate <= 0 or args.capture_rate <= 0:
        parser.error("duration, simulation-rate and capture-rate must be positive")
    if not args.adapter_root.is_dir() or not args.model_root.is_dir() or not args.catalog.is_file():
        parser.error("adapter-root, model-root or prepared asset catalog is missing")
    run(args)


if __name__ == "__main__":
    main()
