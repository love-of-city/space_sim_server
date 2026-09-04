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
import json
import math
from pathlib import Path
import socket
import sys
import threading
import time
from typing import Any

import numpy as np
from Basilisk.architecture import sysModel


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "backend"))

from simulation.serial_chain_kinematics import (  # noqa: E402
    SerialChainKinematics,
    matrix_to_quaternion_wxyz,
)
from space_arm_platform.protocol import CONTROL_PROTOCOL, encode_packet, recv_socket  # noqa: E402
from simulation.architecture import BasiliskModuleRegistry  # noqa: E402


JOINT_MIN = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533])
JOINT_MAX = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.74385, 1.74533])
ARM_JOINT_VELOCITY_LIMIT = np.array([0.70, 0.70, 0.70, 0.90, 1.00])
DEFAULT_EPHEMERIS_EPOCH_UTC = "2026 SEPTEMBER 02 00:00:00.000"
SUPPORTED_EPHEMERIS_CENTER = "Earth"
SUPPORTED_EPHEMERIS_FRAME = "J2000"
SUN_REFERENCE_DISTANCE_M = 149_597_870_700.0
EARTH_EQUATORIAL_RADIUS_M = 6_378_136.6
MINIMUM_PERIAPSIS_ALTITUDE_M = 120_000.0
DEFAULT_ORBIT = {
    "altitude_m": 500_000.0,
    "eccentricity": 0.0,
    "inclination_deg": 51.6,
    "raan_deg": 0.0,
    "argument_of_periapsis_deg": 0.0,
    "true_anomaly_deg": 180.0,
}


ARM_JOINT_NAMES = (
    "so101_shoulder_pan",
    "so101_shoulder_lift",
    "so101_elbow_flex",
    "so101_wrist_flex",
    "so101_wrist_roll",
)


def _load_scene_instance(path: Path | None) -> dict[str, Any] | None:
    """Load and validate one reproducible scene-instance document."""

    if path is None:
        return None
    resolved = path.resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as error:
        raise ValueError(f"unable to read scene instance {resolved}: {error}") from error
    if document.get("schema") != "space-arm-scene-instance/1":
        raise ValueError(f"unsupported scene instance schema: {document.get('schema')}")
    if document.get("template_id") != "spacecraft-arm-teleop":
        raise ValueError(f"unsupported scene template: {document.get('template_id')}")
    runtime = document.get("runtime")
    randomization = document.get("randomization")
    environment = document.get("environment", {})
    if not isinstance(runtime, dict) or not isinstance(randomization, dict):
        raise ValueError("scene instance requires runtime and randomization objects")
    if not isinstance(environment, dict):
        raise ValueError("scene instance environment must be an object")
    epoch = str(environment.get("ephemeris_epoch_utc", DEFAULT_EPHEMERIS_EPOCH_UTC)).strip()
    center = str(environment.get("ephemeris_center", SUPPORTED_EPHEMERIS_CENTER)).strip()
    frame = str(environment.get("ephemeris_frame", SUPPORTED_EPHEMERIS_FRAME)).strip()
    if not epoch:
        raise ValueError("scene environment.ephemeris_epoch_utc must not be empty")
    if center.casefold() != SUPPORTED_EPHEMERIS_CENTER.casefold():
        raise ValueError("scene environment.ephemeris_center must be Earth")
    if frame.casefold() != SUPPORTED_EPHEMERIS_FRAME.casefold():
        raise ValueError("scene environment.ephemeris_frame must be J2000")
    orbit = environment.get("orbit", DEFAULT_ORBIT)
    if not isinstance(orbit, dict):
        raise ValueError("scene environment.orbit must be an object")
    normalized_orbit: dict[str, float] = {}
    for field, default in DEFAULT_ORBIT.items():
        try:
            value = float(orbit.get(field, default))
        except (TypeError, ValueError) as error:
            raise ValueError(f"scene environment.orbit.{field} must be a number") from error
        if not math.isfinite(value):
            raise ValueError(f"scene environment.orbit.{field} must be finite")
        normalized_orbit[field] = value
    eccentricity = normalized_orbit["eccentricity"]
    inclination_deg = normalized_orbit["inclination_deg"]
    altitude_m = normalized_orbit["altitude_m"]
    if altitude_m <= 0.0:
        raise ValueError("scene environment.orbit.altitude_m must be positive")
    if eccentricity < 0.0 or eccentricity >= 1.0:
        raise ValueError("scene environment.orbit.eccentricity must be in [0, 1)")
    if inclination_deg < 0.0 or inclination_deg > 180.0:
        raise ValueError("scene environment.orbit.inclination_deg must be in [0, 180]")
    periapsis_altitude_m = (
        (EARTH_EQUATORIAL_RADIUS_M + altitude_m) * (1.0 - eccentricity)
        - EARTH_EQUATORIAL_RADIUS_M
    )
    if periapsis_altitude_m < MINIMUM_PERIAPSIS_ALTITUDE_M:
        raise ValueError(
            "scene environment.orbit periapsis altitude must be at least "
            f"{MINIMUM_PERIAPSIS_ALTITUDE_M:.0f} m"
        )
    document["environment"] = {
        "ephemeris_epoch_utc": epoch,
        "ephemeris_center": SUPPORTED_EPHEMERIS_CENTER,
        "ephemeris_frame": SUPPORTED_EPHEMERIS_FRAME,
        "orbit": normalized_orbit,
    }
    runtime_limits = {
        "simulation_rate": (0.0, 100.0),
        "capture_rate_hz": (0.0, 60.0),
        "ik_rate_hz": (0.0, 500.0),
    }
    for field, (minimum, maximum) in runtime_limits.items():
        try:
            value = float(runtime[field])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"scene runtime.{field} must be a number") from error
        if not math.isfinite(value) or value <= minimum or value > maximum:
            raise ValueError(
                f"scene runtime.{field} must be in ({minimum}, {maximum}]"
            )
        runtime[field] = value
    expected_lengths = {
        "target_position_m": 3,
        "target_orientation_wxyz": 4,
        "target_linear_velocity_m_s": 3,
        "target_angular_velocity_rad_s": 3,
        "arm_joint_position_rad": 6,
    }
    for field, length in expected_lengths.items():
        values = randomization.get(field)
        if not isinstance(values, list) or len(values) != length:
            raise ValueError(f"scene randomization.{field} must contain {length} numbers")
        try:
            converted = [float(value) for value in values]
        except (TypeError, ValueError) as error:
            raise ValueError(f"scene randomization.{field} must contain only numbers") from error
        if not all(math.isfinite(value) for value in converted):
            raise ValueError(f"scene randomization.{field} must contain finite numbers")
        randomization[field] = converted
    quaternion_norm = float(np.linalg.norm(randomization["target_orientation_wxyz"]))
    if quaternion_norm <= 1.0e-12:
        raise ValueError("scene target quaternion must have non-zero norm")
    joints = np.asarray(randomization["arm_joint_position_rad"], dtype=float)
    if np.any(joints < JOINT_MIN) or np.any(joints > JOINT_MAX):
        raise ValueError("scene arm joint positions exceed the SO-101 joint limits")
    return document


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
        self.initial_position = np.asarray(initial_position, dtype=float).copy()
        self.position = self.initial_position.copy()
        self.velocity = np.zeros(6)
        self.client = client
        self.kinematics = kinematics
        self.last_sim_seconds: float | None = None
        self.applied_sequence = "0"
        self.command_stale = True
        self.desired_twist = np.zeros(6)
        self.achieved_twist = np.zeros(6)
        self.residual_twist = np.zeros(6)
        self.jacobian_rank = int(
            np.linalg.matrix_rank(self.kinematics.jacobian(self.position[:5]))
        )
        self.update_count = 0
        self.ik_solve_count = 0

    def reset(self, sim_seconds: float) -> None:
        """Reset the cached command state at the given simulation time."""

        self.position = self.initial_position.copy()
        self.velocity.fill(0.0)
        self.last_sim_seconds = float(sim_seconds)
        self.applied_sequence = "0"
        self.command_stale = True
        self.desired_twist.fill(0.0)
        self.achieved_twist.fill(0.0)
        self.residual_twist.fill(0.0)
        self.update_count = 0
        self.ik_solve_count = 0

    def update(self, sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Consume the latest action and update the cached joint reference once."""

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
            self.ik_solve_count += 1
        else:
            self.velocity.fill(0.0)
            self.achieved_twist.fill(0.0)
            self.residual_twist.fill(0.0)
        proposed = self.position + self.velocity * dt
        clipped = np.clip(proposed, JOINT_MIN, JOINT_MAX)
        at_limit = clipped != proposed
        self.velocity[at_limit] = 0.0
        self.position = clipped
        self.applied_sequence = str(action.get("server_sequence", "0"))
        self.command_stale = stale
        self.update_count += 1
        return self.position.copy(), self.velocity.copy()

    def cached_reference(self, _sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the held target without running input handling or IK."""

        return self.position.copy(), self.velocity.copy()

    def reference(self, sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Compatibility wrapper for callers that explicitly request an update."""

        return self.update(sim_seconds)


class CartesianIkControlModel(sysModel.SysModel):
    """Run endpoint IK on a scheduled BSK control task, outside MJScene RK stages."""

    def __init__(self, target: CartesianTeleopTarget) -> None:
        super().__init__()
        self.ModelTag = "so101CartesianIkController"
        self.target = target

    def Reset(self, CurrentSimNanos: int) -> None:
        """Reset the held reference when the Basilisk simulation resets."""

        self.target.reset(CurrentSimNanos * 1.0e-9)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Update the joint target once at the configured control-task rate."""


        self.target.update(CurrentSimNanos * 1.0e-9)


def run(args: argparse.Namespace) -> None:
    scene_instance = _load_scene_instance(args.scene_instance)
    if scene_instance:
        runtime = scene_instance["runtime"]
        args.simulation_rate = float(runtime["simulation_rate"])
        args.capture_rate = float(runtime["capture_rate_hz"])
        args.ik_rate = float(runtime["ik_rate_hz"])
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
    initial_joints = (
        np.asarray(scene_instance["randomization"]["arm_joint_position_rad"], dtype=float)
        if scene_instance
        else np.asarray(native.PREGRASP, dtype=float)
    )
    targets = CartesianTeleopTarget(initial_joints, client, kinematics)
    original_reference = native.JointTrajectoryPublisher.__dict__["reference"]
    native.JointTrajectoryPublisher.reference = classmethod(
        lambda _cls, seconds: targets.cached_reference(seconds)
    )

    from Basilisk.simulation import NBodyGravity, pointMassGravityModel
    from Basilisk.utilities import macros, orbitalMotion, simIncludeGravBody
    from bsk_render_adapter import BasiliskRenderBridge, CameraVisual, SceneSettings

    client.start()
    bridge: BasiliskRenderBridge | None = None
    gravity_factory = None
    module_registry = BasiliskModuleRegistry()
    try:
        simulation, scene, dynamics_models, recorders = native._build_simulation()
        ephemeris_environment = scene_instance.get("environment", {}) if scene_instance else {}
        ephemeris_epoch = str(
            ephemeris_environment.get("ephemeris_epoch_utc", DEFAULT_EPHEMERIS_EPOCH_UTC)
        )
        gravity_factory = simIncludeGravBody.gravBodyFactory()
        earth = gravity_factory.createEarth()
        sun = gravity_factory.createSun()
        ephemeris = gravity_factory.createSpiceInterface(
            time=ephemeris_epoch,
            spicePlanetFrames=[SUPPORTED_EPHEMERIS_FRAME, SUPPORTED_EPHEMERIS_FRAME],
            epochInMsg=True,
        )
        ephemeris.zeroBase = SUPPORTED_EPHEMERIS_CENTER

        # Register the environmental models with the MJScene dynamics task.
        # Basilisk computes the accelerations and MJScene performs the unified
        # multibody integration together with joints and contact.
        scene.extraEoMCall = True
        scene.AddModelToDynamicsTask(ephemeris, 20_000)
        gravity = NBodyGravity.NBodyGravity()
        gravity.ModelTag = "earthSunGravity"
        scene.AddModelToDynamicsTask(gravity, 19_000)

        earth.isCentralBody = True
        # Keep the source strengths disabled until after the initial MJScene
        # state has been assigned below.  Gravity targets remain natively bound
        # to MJScene; no Python per-step state forwarding is required.
        earth_gravity_model = pointMassGravityModel.PointMassGravityModel()
        earth_gravity_model.muBody = 0.0
        gravity.addGravitySource("earth", earth_gravity_model, True)
        gravity.getGravitySource("earth").stateInMsg.subscribeTo(
            ephemeris.planetStateOutMsgs[0]
        )

        sun_gravity_model = pointMassGravityModel.PointMassGravityModel()
        sun_gravity_model.muBody = 0.0
        gravity.addGravitySource("sun", sun_gravity_model, False)
        gravity.getGravitySource("sun").stateInMsg.subscribeTo(
            ephemeris.planetStateOutMsgs[1]
        )
        gravity_target_names = list(scene.getBodyNames())
        for body_name in gravity_target_names:
            gravity.addGravityTarget(body_name, scene.getBody(body_name))

        # The MJBody overload installs Basilisk's native subscriptions to the
        # MJScene state and mass-property messages.  Keep this direct binding
        # so the 500 Hz dynamics loop does not cross a Python bridge.
        print(json.dumps({"type": "ephemeris_configuration", "epoch_utc": ephemeris_epoch, "center": SUPPORTED_EPHEMERIS_CENTER, "frame": SUPPORTED_EPHEMERIS_FRAME, "gravity_sources": ["earth", "sun"], "gravity_targets": gravity_target_names}, sort_keys=True), flush=True)
        control_task_name = "teleopIkTask"
        control_task = simulation.CreateNewTask(
            control_task_name, macros.sec2nano(1.0 / args.ik_rate)
        )
        grasp_process = next(
            process for process in simulation.procList if process.Name == "graspProcess"
        )
        grasp_process.addTask(control_task, 100)
        ik_controller = CartesianIkControlModel(targets)
        module_registry.register("teleop_ik", ik_controller, task_name=control_task_name)
        keep_alive = (
            dynamics_models,
            recorders,
            module_registry,
            gravity_factory,
            earth_gravity_model,
            sun_gravity_model,
            gravity,
            earth,
            sun,
            ephemeris,
        )
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
        bridge.add_celestial_bodies(
            [sun],
            visual_overrides={
                "sun": {
                    "visual_role": "star",
                    "luminous": True,
                    "drives_directional_light": True,
                    "light_color_rgb": (1.0, 0.98, 0.92),
                    # Keep the current exposure calibration; UE applies the 1/r^2 correction.
                    "light_illuminance_lux_at_reference_distance": 8.0,
                    "light_reference_distance_m": SUN_REFERENCE_DISTANCE_M,
                }
            },
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
        module_registry.register("render_state_publisher", bridge, task_name="graspTask", priority=-10_000)
        module_registry.attach(simulation)
        native._initialize_state(simulation, scene)
        environment = scene_instance.get("environment", {}) if scene_instance else {}
        orbit = environment.get("orbit", DEFAULT_ORBIT)
        elements = orbitalMotion.ClassicElements()
        elements.a = float(earth.radEquator) + float(orbit["altitude_m"])
        elements.e = float(orbit["eccentricity"])
        elements.i = math.radians(float(orbit["inclination_deg"]))
        elements.Omega = math.radians(float(orbit["raan_deg"]))
        elements.omega = math.radians(float(orbit["argument_of_periapsis_deg"]))
        elements.f = math.radians(float(orbit["true_anomaly_deg"]))
        orbital_position, orbital_velocity = orbitalMotion.elem2rv(float(earth.mu), elements)
        orbital_position = np.asarray(orbital_position, dtype=float)
        orbital_velocity = np.asarray(orbital_velocity, dtype=float)
        randomized = scene_instance["randomization"] if scene_instance else {
            "target_position_m": list(native.TARGET_POS),
            "target_orientation_wxyz": list(native.TARGET_QUAT),
            "target_linear_velocity_m_s": list(native.COMMON_VELOCITY),
            "target_angular_velocity_rad_s": list(native.TARGET_SPIN),
        }
        common_velocity = np.asarray(native.COMMON_VELOCITY, dtype=float)
        bus_velocity = orbital_velocity + common_velocity
        bus = scene.getBody("cubesat_bus")
        target = scene.getBody("capture_target")
        bus.setPosition(orbital_position)
        bus.setVelocity(bus_velocity)
        target.setPosition(orbital_position + np.asarray(randomized["target_position_m"], dtype=float))
        target.setVelocity(
            orbital_velocity
            + np.asarray(randomized["target_linear_velocity_m_s"], dtype=float)
        )
        target.setAttitude(
            native._quaternion_to_mrp(
                np.asarray(randomized["target_orientation_wxyz"], dtype=float)
            )
        )
        target.setAttitudeRate(np.asarray(randomized["target_angular_velocity_rad_s"], dtype=float))
        for (body_name, joint_name), position in zip(native.JOINTS, initial_joints, strict=True):
            joint = scene.getBody(body_name).getScalarJoint(joint_name)
            joint.setPosition(float(position))
            joint.setVelocity(0.0)

        orbit_state = {
            "semi_major_axis_m": float(elements.a),
            "initial_position_m": orbital_position.tolist(),
            "initial_velocity_m_s": bus_velocity.tolist(),
            "initial_altitude_m": float(np.linalg.norm(orbital_position) - float(earth.radEquator)),
            "initial_speed_m_s": float(np.linalg.norm(bus_velocity)),
        }
        print(
            json.dumps(
                {
                    "type": "gravity_configuration",
                    "sources": ["earth", "sun"],
                    "central_body": "earth",
                    "gravity_targets": gravity_target_names,
                    "orbit": orbit,
                    **orbit_state,
                },
                sort_keys=True,
            ),
            flush=True,
        )

        # Enable the physical Earth/Sun fields after the explicit initial
        # MJScene state has been assigned.  NBodyGravity now reads the native
        # MJScene messages at each integration/substep.
        earth_gravity_model.muBody = float(earth.mu)
        sun_gravity_model.muBody = float(sun.mu)
        print(
            json.dumps(
                {
                    "type": "gravity_initialization",
                    "native_targets_bound": len(gravity_target_names),
                    "gravity_enabled": True,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in native.JOINTS]

        wall_start = time.monotonic()
        processing_seconds = 0.0
        frame_count = int(math.ceil(args.duration * 30.0)) if args.duration > 0.0 else None
        frame = 0
        while frame_count is None or frame < frame_count:
            frame += 1
            sim_seconds = frame / 30.0
            if frame_count is not None:
                sim_seconds = min(sim_seconds, args.duration)
            deadline = wall_start + sim_seconds / args.simulation_rate
            time.sleep(max(0.0, deadline - time.monotonic()))
            processing_start = time.monotonic()
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
                    "scene_instance_id": scene_instance.get("instance_id") if scene_instance else None,
                    "scene_seed": scene_instance.get("seed") if scene_instance else None,
                    "scene_template_id": scene_instance.get("template_id") if scene_instance else "spacecraft-arm-teleop",
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
                    "ik_control_rate_hz": args.ik_rate,
                    "ik_update_count": str(targets.update_count),
                    "ik_solve_count": str(targets.ik_solve_count),
                }
            )
            processing_seconds += time.monotonic() - processing_start
        if args.duration > 0.0:
            wall_seconds = time.monotonic() - wall_start
            print(
                json.dumps(
                    {
                        "type": "performance_summary",
                        "simulated_seconds": args.duration,
                        "wall_seconds": wall_seconds,
                        "processing_seconds": processing_seconds,
                        "wall_real_time_factor": args.duration / wall_seconds,
                        "processing_real_time_factor": args.duration / processing_seconds,
                        "ik_control_rate_hz": args.ik_rate,
                        "ik_update_count": targets.update_count,
                        "ik_solve_count": targets.ik_solve_count,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        _ = keep_alive
    finally:
        native.JointTrajectoryPublisher.reference = original_reference
        if bridge:
            bridge.close()
        if gravity_factory is not None:
            gravity_factory.unloadSpiceKernels()
        client.close()


def main() -> None:
    workspace = PROJECT_ROOT.parent
    default_adapter = workspace / "space_sim_UE_adapter"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-root", type=Path, default=default_adapter)
    parser.add_argument(
        "--model-root",
        type=Path,
        default=default_adapter / "test" / "model" / "spacecraft_and_arm",
    )
    parser.add_argument(
        "--catalog",
        type=Path,
        default=default_adapter / "Unreal" / "BskUnrealRenderer" / "Saved" / "AssetImport" / "cubesat_so101.catalog.json",
    )
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8766)
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=5558)
    parser.add_argument("--duration", type=float, default=0.0, help="Optional finite runtime in seconds; 0 runs until stopped.")
    parser.add_argument("--simulation-rate", type=float, default=1.0)
    parser.add_argument("--capture-rate", type=float, default=10.0)
    parser.add_argument("--ik-rate", type=float, default=100.0)
    parser.add_argument("--scene-instance", type=Path)
    args = parser.parse_args()
    if (
        args.duration < 0
        or args.simulation_rate <= 0
        or args.capture_rate <= 0
        or args.ik_rate <= 0
        or args.ik_rate > 500
    ):
        parser.error(
            "duration must be non-negative, simulation-rate and capture-rate must be positive; "
            "ik-rate must be in (0, 500]"
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
