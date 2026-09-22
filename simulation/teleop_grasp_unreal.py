"""Run the authoritative SARM MJScene under live operator control.

This process owns all joint/contact dynamics.  The browser never moves UE
actors directly: it sends a bounded six-dimensional end-effector twist and a
gripper command to the platform backend.  This process projects the Cartesian
command through direction-preserving, speed-bounded six-dimensional inverse
kinematics into PID joint references, and the existing bsk_render_adapter
publishes the resulting world
state to UE.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import gc
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time
from typing import Any, Callable

import numpy as np
from Basilisk.architecture import sysModel


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "backend"))

from simulation.serial_chain_kinematics import (  # noqa: E402
    SerialChainKinematics,
    IkResult,
    axis_angle_to_matrix,
    matrix_to_quaternion_wxyz,
    rotation_matrix_to_vector,
)
from space_arm_platform.protocol import CONTROL_PROTOCOL, encode_packet, recv_socket  # noqa: E402
from simulation.observation_capture import AuthoritativeObservationModel
from simulation.physics_clock import RationalPhysicsClock
from simulation.native_integration import configure_scene_integrator
from space_arm_platform.sampling import (
    DYNAMICS_HZ, DEFAULT_IK_HZ, DEFAULT_CAPTURE_HZ, RENDER_HZ, SUPPORTED_FPS,
    ik_step_stride, tick_time_ns,
)
from simulation.architecture import BasiliskModuleRegistry  # noqa: E402
from space_arm_platform.lighting import (  # noqa: E402
    DEFAULT_SUNLIGHT_INTENSITY_SCALE, validate_sunlight_intensity_scale,
)


from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, capture_target
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME, ZERO_TELEOP_HOME, DEFAULT_OPERATING_JOINT_DEG
from simulation.arm_preparation import ArmPreparation
from space_arm_platform.joint_limits import load_joint_limits
from simulation.motion_diagnostics import MotionSpeedMonitor
from simulation.online_elbow_ik import (
    OnlineElbowPreference, ElbowStepDiagnostics, ElbowDeviationState, apply_online_elbow_preference,
)
from space_arm_platform.control_defaults import DEFAULT_DYNAMICS_STEP_S, MIN_DYNAMICS_STEP_S, MAX_DYNAMICS_STEP_S
from simulation.reference_governor import JointReferenceGovernor
from simulation.reference_recovery import ReferenceRecoverySettings
from simulation.runtime_progress import runtime_stage
from simulation.joint_reference_publisher import HeldJointReferencePublisher


def validate_dynamics_step(value: float) -> float:
    step = float(value)
    if not math.isfinite(step) or not MIN_DYNAMICS_STEP_S <= step <= MAX_DYNAMICS_STEP_S:
        raise ValueError(f"dynamics_step_s must be in [{MIN_DYNAMICS_STEP_S}, {MAX_DYNAMICS_STEP_S}]")
    return step


SARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7)) + ("joint_finger1", "joint_finger2")
# Compatibility aliases only. Each live target receives its selected model limits.
_DEFAULT_LIMITS = load_joint_limits(PROJECT_ROOT / "model/SARM/platform/sarm_platform.xml", SARM_JOINT_NAMES)
JOINT_MIN = np.array(_DEFAULT_LIMITS.lower)
JOINT_MAX = np.array(_DEFAULT_LIMITS.upper)
ARM_JOINT_VELOCITY_LIMIT = np.array([0.70, 0.70, 0.70, 0.90, 1.00, 1.00])
# Differential IK kernels selectable for A/B evaluation.
IK_MODE_IK_POSE = "ik_pose"
IK_MODE_STRICT = "strict"
IK_MODES = (IK_MODE_IK_POSE, IK_MODE_STRICT)
# robosuite ``IK_POSE`` damping schedule: well-conditioned poses keep near-exact
# task tracking (base damping) while the approach to a singularity ramps the
# damping up, so the command slows down smoothly instead of freezing outright.
IK_POSE_BASE_DAMPING = 1.0e-3
IK_POSE_MAXIMUM_DAMPING = 5.0e-2
IK_POSE_SINGULAR_VALUE_THRESHOLD = 2.0e-2
MAX_LINEAR_COMMAND_ACCELERATION_M_S2 = 0.20
MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 = 2.0
TELEOP_ARM_KP = np.array([32.0, 32.0, 32.0, 30.0, 30.0, 15.0])
TELEOP_ARM_KD = np.array([2.0, 2.0, 2.0, 0.7, 0.5, 0.25])
TELEOP_ARM_TORQUE_LIMIT = np.array([2.0, 2.0, 2.0, 1.0, 1.0, 0.35])
DEFAULT_EPHEMERIS_EPOCH_UTC = "2026 SEPTEMBER 02 00:00:00.000"
SUPPORTED_EPHEMERIS_CENTER = "Earth"
SUPPORTED_EPHEMERIS_FRAME = "J2000"
# Inertial translation and planet-fixed attitude are different SPICE outputs.
CELESTIAL_FIXED_FRAMES = ("IAU_EARTH", "IAU_SUN")
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
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6",
)


def _load_scene_instance(path: Path | None, model_root: Path | None = None) -> dict[str, Any] | None:
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
    target = capture_target(document.get("template_id"))
    randomize_orbit_phase = document.get("randomize_orbit_phase", False)
    if not isinstance(randomize_orbit_phase, bool):
        raise ValueError("scene randomize_orbit_phase must be a boolean")
    document["randomize_orbit_phase"] = randomize_orbit_phase
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
    if randomize_orbit_phase and ("orbit" not in environment or "true_anomaly_deg" not in orbit):
        raise ValueError("randomized scene requires a saved environment.orbit.true_anomaly_deg")
    # Replay the stored angle, never resample from Seed during loading.
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
    lighting = environment.get("lighting", {})
    if not isinstance(lighting, dict):
        raise ValueError("scene environment.lighting must be an object")
    sunlight_scale = validate_sunlight_intensity_scale(
        lighting.get("sunlight_intensity_scale", DEFAULT_SUNLIGHT_INTENSITY_SCALE)
    )
    document["environment"] = {
        "ephemeris_epoch_utc": epoch,
        "ephemeris_center": SUPPORTED_EPHEMERIS_CENTER,
        "ephemeris_frame": SUPPORTED_EPHEMERIS_FRAME,
        "orbit": normalized_orbit,
        "lighting": {"sunlight_intensity_scale": sunlight_scale},
    }
    runtime_limits = {
        "simulation_rate": (0.0, 100.0),
        "capture_rate_hz": (0.0, 60.0),
        "ik_rate_hz": (0.0, 240.0),
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
        "arm_joint_position_rad": 8,
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
    scene_model = target.resolve_model(model_root) if model_root is not None else PROJECT_ROOT / target.runtime_model
    scene_limits = load_joint_limits(scene_model, SARM_JOINT_NAMES)
    if np.any(joints < np.array(scene_limits.lower)) or np.any(joints > np.array(scene_limits.upper)):
        raise ValueError("scene arm joint positions exceed the SARM joint limits")
    if document.get("arm_preparation_required", False):
        if not np.allclose(joints[:6], 0., atol=1e-12, rtol=0):
            raise ValueError("zero-start preparation scene must initialize six arm joints at zero")
        goal = np.deg2rad(np.asarray(document.get("operating_arm_joint_position_deg", DEFAULT_OPERATING_JOINT_DEG), dtype=float))
        if goal.shape != (6,) or not np.all(np.isfinite(goal)) or np.any(goal < np.asarray(scene_limits.lower[:6])) or np.any(goal > np.asarray(scene_limits.upper[:6])):
            raise ValueError("invalid saved operating joint angles")
    if target.hinge_joint:
        angle = randomization.get("target_hinge_position_rad", 0.0)
        if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or angle != 0.0:
            raise ValueError("target_hinge_position_rad must be 0 at startup; only the CAD zero pose has validated clearance")
        randomization["target_hinge_position_rad"] = 0.0
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
        self._reset_generation = ""
        self._pending_reset: str | None = None
        self._resetting = False
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
            stale = self._resetting or time.monotonic() - self._received_monotonic > 0.25
        if stale:
            action["deadman"] = False
            action["end_effector_linear_velocity_body_m_s"] = [0.0] * 3
            action["end_effector_angular_velocity_body_rad_s"] = [0.0] * 3
            action["gripper_velocity_m_s"] = 0.0
            action["gripper_velocity_rad_s"] = 0.0
        return action, stale

    @property
    def reset_generation(self) -> str:
        with self._lock:
            return self._reset_generation

    def take_reset(self) -> str | None:
        """Only the simulation thread may consume a reset, between advances."""
        with self._lock:
            request_id = self._pending_reset
            if request_id is not None:
                self._pending_reset = None
                self._reset_generation = request_id
            return request_id

    def complete_reset(self) -> None:
        with self._lock:
            # A reset received during initialization must stay pending.
            if self._pending_reset is None:
                self._resetting = False
            self._action = self._neutral_action()
            self._received_monotonic = 0.0

    def _accept_message(self, message: dict[str, Any]) -> None:
        with self._lock:
            if message.get("protocol") == CONTROL_PROTOCOL and message.get("type") == "reset":
                request_id = message.get("request_id")
                if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                    return
                if request_id == self._reset_generation or self._resetting:
                    return  # Reconnect/retry cannot reset an already applied generation twice.
                self._pending_reset = request_id
                self._resetting = True
                self._action = self._neutral_action()
                self._received_monotonic = 0.0
            elif (not self._resetting and self._valid_action(message)
                  and message.get("reset_generation", "") == self._reset_generation):
                self._action = message
                self._received_monotonic = time.monotonic()

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
                                "reset_generation": self.reset_generation,
                                "capabilities": [
                                    "scene_reset", "arm_preparation",
                                    "cartesian_twist_6d",
                                    "damped_least_squares_ik",
                                    "gripper_velocity",
                                    "joint_observation",
                                    "cartesian_observation",
                                    "sarm_si_joint_observation", "reaction_wheel_attitude_control",
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
                    self._accept_message(message)
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
                self._action = self._neutral_action()
                self._received_monotonic = 0.0
        try:
            connection.close()
        except OSError:
            pass

    @staticmethod
    def _valid_action(message: dict[str, Any]) -> bool:
        preparation = message.get("arm_preparation")
        if preparation is not None:
            if not isinstance(preparation, dict):
                return False
            goal = preparation.get("joint_position_deg")
            request_id = preparation.get("request_id")
            if (not isinstance(request_id, str) or not 1 <= len(request_id) <= 80
                    or not isinstance(goal, list) or len(goal) != 6
                    or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in goal)):
                return False
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
            and (
                isinstance(message.get("gripper_velocity_m_s"), (int, float))
                or isinstance(message.get("gripper_velocity_rad_s"), (int, float))
            )
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
            "gripper_velocity_m_s": 0.0,
            "gripper_velocity_rad_s": 0.0,
        }


class CartesianTeleopTarget:
    """Integrate bounded joint references from operator twist; telemetry is passive."""

    def __init__(
        self,
        initial_position: np.ndarray,
        client: SimulationControlClient,
        kinematics: SerialChainKinematics,
        *,
        ik_mode: str = IK_MODE_IK_POSE,
        joint_limits=None,
        elbow_preference: OnlineElbowPreference | None = None,
        preparation_required: bool = False,
        operating_joint_deg=None,
    ) -> None:
        if ik_mode not in IK_MODES:
            raise ValueError(f"unsupported IK mode {ik_mode!r}; expected one of {IK_MODES}")
        self.initial_position = np.asarray(initial_position, dtype=float).copy()
        self.position = self.initial_position.copy()
        self.velocity = np.zeros_like(self.initial_position)
        self.client = client
        self.kinematics = kinematics
        self.ik_mode = ik_mode
        if elbow_preference is None:
            elbow_mode = os.environ.get("SPACE_SIM_ONLINE_ELBOW_MODE", "prefer")
            if elbow_mode not in {"off", "prefer"}:
                raise ValueError("SPACE_SIM_ONLINE_ELBOW_MODE must be off or prefer")
            wrist_mode = os.environ.get("SPACE_SIM_ONLINE_WRIST_MODE", "prefer")
            if wrist_mode not in {"off", "prefer"}:
                raise ValueError("SPACE_SIM_ONLINE_WRIST_MODE must be off or prefer")
            joint3_mode = os.environ.get("SPACE_SIM_ONLINE_JOINT3_MODE", "prefer")
            if joint3_mode not in {"off", "prefer"}:
                raise ValueError("SPACE_SIM_ONLINE_JOINT3_MODE must be off or prefer")
            elbow_preference = OnlineElbowPreference(
                enabled=elbow_mode == "prefer", wrist_enabled=wrist_mode == "prefer",
                joint3_enabled=joint3_mode == "prefer")
        self.elbow_preference = elbow_preference
        self.elbow_deviation = ElbowDeviationState()
        self.elbow_diagnostics = ElbowStepDiagnostics()
        limits = joint_limits or _DEFAULT_LIMITS
        if limits.names != SARM_JOINT_NAMES:
            raise ValueError("joint limit order does not match the SARM chain")
        self.joint_min, self.joint_max = np.array(limits.lower), np.array(limits.upper)
        if np.any(self.initial_position < self.joint_min) or np.any(self.initial_position > self.joint_max):
            raise ValueError("initial joint positions exceed selected model limits")
        self.preparation = ArmPreparation(
            preparation_required,
            np.deg2rad(DEFAULT_OPERATING_JOINT_DEG if operating_joint_deg is None else operating_joint_deg),
            self.joint_min[:6], self.joint_max[:6], ARM_JOINT_VELOCITY_LIMIT)
        self.speed_monitor = MotionSpeedMonitor()
        self.solver_status = "idle"
        self.solver_reasons = []
        self.solve_time_ms = 0.
        self.raw_operator_twist = np.zeros(6)
        self._joint_velocity_provider = None
        self._actuator_state_provider = None
        self.actuator_diagnostics = None
        self._joint_state_provider: Callable[[], np.ndarray] | None = None
        self._effort_provider: Callable[[], np.ndarray] | None = None
        self.governor = JointReferenceGovernor()
        self.governor_state = "holding"
        self.governor_limited_joints: tuple[int, ...] = ()
        self.arm_effort_ratio = np.zeros(6)
        self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(
            self.initial_position[:6]
        )
        self.actual_tool_position = self.target_tool_position.copy()
        self.actual_tool_rotation = self.target_tool_rotation.copy()
        self.position_error = np.zeros(3)
        self.orientation_error = np.zeros(3)
        self.commanded_linear_velocity = np.zeros(3)
        self.commanded_angular_velocity = np.zeros(3)
        self.tracking_scale = 1.0
        self.velocity_scale = 1.0
        self.minimum_singular_value = 0.0
        self.condition_number = math.inf
        self.ik_damping = 0.0
        self.nullspace_correction_norm = 0.0
        self.last_sim_seconds: float | None = None
        self.applied_sequence = "0"
        self.command_stale = True
        self.desired_twist = np.zeros(6)
        self.achieved_twist = np.zeros(6)
        self.residual_twist = np.zeros(6)
        self.jacobian_rank = int(
            np.linalg.matrix_rank(self.kinematics.jacobian(self.position[:6]))
        )
        self.update_count = 0
        self.ik_solve_count = 0

    def bind_joint_state_provider(self, provider: Callable[[], np.ndarray]) -> None:
        """Use measured arm positions for telemetry; do not gate operator motion."""

        self._joint_state_provider = provider

    def bind_joint_velocity_provider(self, provider: Callable[[], np.ndarray]) -> None:
        self._joint_velocity_provider = provider

    def bind_actuator_state_provider(self, provider) -> None:
        self._actuator_state_provider = provider

    def _actual_arm_position(self) -> np.ndarray:
        if self._joint_state_provider is None:
            return self.position[:6].copy()
        measured = np.asarray(self._joint_state_provider(), dtype=float)
        if measured.shape != (6,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured arm joint state must contain six finite values")
        return measured

    def bind_effort_provider(self, provider: Callable[[], np.ndarray]) -> None:
        """Read absolute applied effort divided by each arm actuator's limit."""

        self._effort_provider = provider

    @staticmethod
    def _limit_vector(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= maximum_norm or norm <= 1.0e-14:
            return vector
        return vector * (maximum_norm / norm)

    @classmethod
    def _approach_vector(
        cls, current: np.ndarray, target: np.ndarray, maximum_delta: float
    ) -> np.ndarray:
        return current + cls._limit_vector(target - current, maximum_delta)

    def _advance_target(self, linear_velocity: np.ndarray, angular_velocity: np.ndarray, dt: float) -> None:
        self.target_tool_position = self.target_tool_position + linear_velocity * dt
        angular_speed = float(np.linalg.norm(angular_velocity))
        if angular_speed > 1.0e-14:
            delta = axis_angle_to_matrix(angular_velocity / angular_speed, angular_speed * dt)
            # Commands and Jacobian angular velocity are expressed in the
            # spacecraft body frame, so apply the spatial rotation on the left.
            self.target_tool_rotation = delta @ self.target_tool_rotation

    def reset(self, sim_seconds: float) -> None:
        """Reset the held joint and Cartesian references at simulation start."""

        self.position = self.initial_position.copy()
        self.preparation.reset()
        self.elbow_deviation.reset()
        self.elbow_diagnostics = ElbowStepDiagnostics()
        self.raw_operator_twist.fill(0.)
        self.actuator_diagnostics = None
        self.speed_monitor.reset()
        self.solver_reasons = []
        self.solver_status = "idle"
        self.solve_time_ms = 0.
        self.governor.reset()
        self.governor_state = "holding"
        self.governor_limited_joints = ()
        self.arm_effort_ratio.fill(0.0)
        self.velocity.fill(0.0)
        self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(
            self.initial_position[:6]
        )
        self.actual_tool_position = self.target_tool_position.copy()
        self.actual_tool_rotation = self.target_tool_rotation.copy()
        self.position_error.fill(0.0)
        self.orientation_error.fill(0.0)
        self.commanded_linear_velocity.fill(0.0)
        self.commanded_angular_velocity.fill(0.0)
        self.tracking_scale = 1.0
        self.velocity_scale = 1.0
        self.minimum_singular_value = 0.0
        self.condition_number = math.inf
        self.ik_damping = 0.0
        self.nullspace_correction_norm = 0.0
        self.last_sim_seconds = float(sim_seconds)
        self.applied_sequence = "0"
        self.command_stale = True
        self.desired_twist.fill(0.0)
        self.achieved_twist.fill(0.0)
        self.residual_twist.fill(0.0)
        self.update_count = 0
        self.ik_solve_count = 0

    def update(self, sim_seconds: float) -> tuple[np.ndarray, np.ndarray]:
        """Update bounded joint references; hold them on release or stale input."""

        if self.last_sim_seconds is None:
            self.last_sim_seconds = sim_seconds
            return self.position.copy(), self.velocity.copy()
        dt = max(0.0, min(0.02, sim_seconds - self.last_sim_seconds))
        self.last_sim_seconds = sim_seconds
        if dt <= 0.0:
            return self.position.copy(), self.velocity.copy()

        action, stale = self.client.latest_action()
        if self.preparation.required:
            try:
                measured = self._actual_arm_position() if self._joint_state_provider is not None else np.full(6, np.nan)
                measured_velocity = (np.asarray(self._joint_velocity_provider(), dtype=float)
                                     if self._joint_velocity_provider is not None else np.full(6, np.nan))
            except (ValueError, TypeError, RuntimeError):
                measured = measured_velocity = np.full(6, np.nan)
            preparation_reference = self.preparation.step(
                dt, action, stale, self.position[:6], measured, measured_velocity)
            if preparation_reference is not None:
                self.position[:6], self.velocity[:6] = preparation_reference
                self.velocity[6:] = 0.
                self.commanded_linear_velocity.fill(0.)
                self.commanded_angular_velocity.fill(0.)
                self.raw_operator_twist.fill(0.)
                self.desired_twist.fill(0.)
                self.achieved_twist.fill(0.)
                self.residual_twist.fill(0.)
                self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])
                self.actual_tool_position, self.actual_tool_rotation = self.kinematics.forward(measured) if np.all(np.isfinite(measured)) else (self.target_tool_position.copy(), self.target_tool_rotation.copy())
                self.position_error = self.target_tool_position - self.actual_tool_position
                self.orientation_error = rotation_matrix_to_vector(self.target_tool_rotation @ self.actual_tool_rotation.T)
                self.elbow_deviation.reset()
                self.elbow_diagnostics = ElbowStepDiagnostics(status="preparation")
                self.solver_status = "preparation_" + self.preparation.status
                self.solver_reasons = []
                self.solve_time_ms = 0.
                self.applied_sequence = str(action.get("server_sequence", "0"))
                self.command_stale = stale
                self.update_count += 1
                return self.position.copy(), self.velocity.copy()
        enabled = bool(action.get("deadman")) and not stale
        requested_linear = np.asarray(
            action["end_effector_linear_velocity_body_m_s"], dtype=float
        )
        requested_angular = np.asarray(
            action["end_effector_angular_velocity_body_rad_s"], dtype=float
        )
        self.raw_operator_twist = np.concatenate((requested_linear, requested_angular))
        requested_gripper = float(
            action.get("gripper_velocity_m_s", action.get("gripper_velocity_rad_s", 0.0))
        )
        # The packet deadman authorizes both channels, but a gripper action must
        # never enable arm IK. Numerical zero only: not a new joystick dead zone.
        arm_enabled = enabled and bool(np.any(np.abs(self.raw_operator_twist) > 1.0e-14))
        gripper_enabled = enabled and abs(requested_gripper) > 1.0e-14
        if arm_enabled:
            self.commanded_linear_velocity = self._approach_vector(
                self.commanded_linear_velocity,
                requested_linear,
                MAX_LINEAR_COMMAND_ACCELERATION_M_S2 * dt,
            )
            self.commanded_angular_velocity = self._approach_vector(
                self.commanded_angular_velocity,
                requested_angular,
                MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 * dt,
            )
        else:
            self.commanded_linear_velocity.fill(0.0)
            self.commanded_angular_velocity.fill(0.0)
        operator_linear = self.commanded_linear_velocity
        operator_angular = self.commanded_angular_velocity

        actual_arm = self._actual_arm_position()
        self.actual_tool_position, self.actual_tool_rotation = self.kinematics.forward(actual_arm)
        self.desired_twist = np.concatenate((operator_linear, operator_angular))
        # First solve the original bounded DLS task, then apply a bounded
        # geometric preference (no initial/home joint reference). Measured
        # plant error remains diagnostic; this is not plant pose feedback.
        self.elbow_diagnostics = ElbowStepDiagnostics(
            enabled=self.elbow_preference.enabled and self.ik_mode == IK_MODE_IK_POSE,
            status="idle" if self.elbow_preference.enabled and self.ik_mode == IK_MODE_IK_POSE else "off",
            wrist_enabled=self.elbow_preference.wrist_enabled and self.elbow_preference.enabled
                and self.ik_mode == IK_MODE_IK_POSE,
            joint3_enabled=self.elbow_preference.joint3_enabled and self.elbow_preference.enabled
                and self.ik_mode == IK_MODE_IK_POSE,
            position_offset_m=self.elbow_diagnostics.position_offset_m,
            orientation_offset_rad=self.elbow_diagnostics.orientation_offset_rad,
        )
        if not arm_enabled:
            # Keep measured telemetry running below, but skip reference-chain
            # Jacobian/SVD/IK work. Geometry diagnostics retain the last sample.
            result = IkResult(
                joint_velocity_rad_s=np.zeros(6), achieved_twist=np.zeros(6),
                residual_twist=np.zeros(6), jacobian_rank=self.jacobian_rank,
                minimum_singular_value=self.minimum_singular_value,
                condition_number=self.condition_number,
            )
            self.solve_time_ms = 0.0
        else:
            solve_start = time.perf_counter()
            if self.ik_mode == IK_MODE_STRICT:
                result = self.kinematics.inverse_velocity_bounded(
                    self.position[:6],
                    self.desired_twist,
                    joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
                    joint_position_min=self.joint_min[:6],
                    joint_position_max=self.joint_max[:6],
                    dt=dt,
                )
            else:
                # Task-only DLS is the baseline; never pass a home reference.
                result = self.kinematics.inverse_velocity_ik_pose(
                    self.position[:6],
                    self.desired_twist,
                    joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
                    joint_position_min=self.joint_min[:6],
                    joint_position_max=self.joint_max[:6],
                    dt=dt,
                    base_damping=IK_POSE_BASE_DAMPING,
                    maximum_damping=IK_POSE_MAXIMUM_DAMPING,
                    singular_value_threshold=IK_POSE_SINGULAR_VALUE_THRESHOLD,
                )
                result, self.elbow_diagnostics = apply_online_elbow_preference(
                    self.kinematics, self.position[:6], self.desired_twist, result,
                    joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
                    joint_position_min=self.joint_min[:6], joint_position_max=self.joint_max[:6],
                    dt=dt, preference=self.elbow_preference, deviation_state=self.elbow_deviation,
                )
            self.solve_time_ms = (time.perf_counter() - solve_start) * 1000.0
            self.ik_solve_count += 1
        self.solver_reasons = [dict(reason) for reason in getattr(result, "limit_reasons", ())]
        self.solver_status = (
            "holding" if not arm_enabled else
            "limited" if result.velocity_scale < 1.0 - 1e-6 else "tracking"
        )
        if self.elbow_diagnostics.status == "active":
            self.solver_reasons.append({"code": "elbow_preference", "joints": []})
        if arm_enabled and np.linalg.norm(result.residual_twist) > 1e-5:
            if result.damping > IK_POSE_BASE_DAMPING + 1e-8:
                self.solver_reasons.append({"code": "damped_task_error", "joints": []})
            if result.minimum_singular_value < IK_POSE_SINGULAR_VALUE_THRESHOLD:
                self.solver_reasons.append({"code": "singularity_nearby", "joints": []})
            if not self.solver_reasons:
                self.solver_reasons.append({"code": "task_residual", "joints": []})
            if self.solver_status == "tracking":
                self.solver_status = "approximate"
        self.arm_effort_ratio = self._effort_provider() if self._effort_provider else np.zeros(6)
        governed = self.governor.apply(
            self.position[:6], actual_arm, result.joint_velocity_rad_s, dt,
            enabled=arm_enabled,
            effort_ratio=self.arm_effort_ratio,
            allow_recovery=(
                action.get("allow_reference_recovery") is True and not stale
                and not enabled and not action.get("reason")
            ),
        )
        self.velocity[:6] = governed.velocity
        self.tracking_scale = governed.scale
        self.governor_state = governed.state
        self.governor_limited_joints = governed.limited_joints
        self.achieved_twist = (
            self.kinematics.jacobian(self.position[:6]) @ governed.velocity
            if governed.reference_rebased else result.achieved_twist * governed.scale
        )
        self.residual_twist = self.desired_twist - self.achieved_twist
        self.jacobian_rank = result.jacobian_rank
        self.velocity_scale = result.velocity_scale
        self.minimum_singular_value = result.minimum_singular_value
        self.condition_number = result.condition_number
        self.ik_damping = result.damping
        self.nullspace_correction_norm = result.nullspace_correction_norm
        if arm_enabled:
            self._advance_target(self.achieved_twist[:3], self.achieved_twist[3:], dt)
        self.position_error = self.target_tool_position - self.actual_tool_position
        self.orientation_error = rotation_matrix_to_vector(
            self.target_tool_rotation @ self.actual_tool_rotation.T
        )

        gripper_velocity = requested_gripper if gripper_enabled else 0.0
        self.velocity[6] = gripper_velocity
        self.velocity[7] = gripper_velocity

        proposed_arm = self.position[:6] + self.velocity[:6] * dt
        joint_limited_arm = np.clip(proposed_arm, self.joint_min[:6], self.joint_max[:6])
        at_arm_limit = joint_limited_arm != proposed_arm
        self.velocity[:6][at_arm_limit] = 0.0
        self.position[:6] = joint_limited_arm
        if governed.reference_rebased:
            self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])
            self.position_error = self.target_tool_position - self.actual_tool_position
            self.orientation_error = rotation_matrix_to_vector(self.target_tool_rotation @ self.actual_tool_rotation.T)
        if self._joint_velocity_provider is not None:
            measured_velocity = np.asarray(self._joint_velocity_provider(), dtype=float)
            if measured_velocity.shape != (6,):
                raise ValueError("measured arm velocity must have six entries")
            measured_twist = self.kinematics.jacobian(actual_arm) @ measured_velocity
        else:
            # Never mislabel the kinematic prediction as a physical measurement.
            measured_velocity = np.full(6, np.nan)
            measured_twist = np.full(6, np.nan)
        self.actuator_diagnostics = self._actuator_state_provider() if self._actuator_state_provider is not None else None
        self.speed_monitor.update(sim_seconds, self.desired_twist, measured_twist, self.achieved_twist,
            enabled=arm_enabled, stale=stale, solver_reasons=self.solver_reasons,
            measured_joints=actual_arm, target_joints=self.position[:6],
            measured_joint_velocity=measured_velocity, target_joint_velocity=self.velocity[:6],
            actuator_evidence=self.actuator_diagnostics)

        proposed_gripper = self.position[6:] + self.velocity[6:] * dt
        clipped_gripper = np.clip(proposed_gripper, self.joint_min[6:], self.joint_max[6:])
        at_gripper_limit = clipped_gripper != proposed_gripper
        self.velocity[6:][at_gripper_limit] = 0.0
        self.position[6:] = clipped_gripper

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

    def __init__(self, target: CartesianTeleopTarget, *, clock=None, stride: int = 1) -> None:
        super().__init__()
        self.ModelTag = "so101CartesianIkController"
        self.target = target
        self.clock = clock
        self.stride = stride

    def Reset(self, CurrentSimNanos: int) -> None:
        """Reset the held reference when the Basilisk simulation resets."""

        self.target.reset(CurrentSimNanos * 1.0e-9)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Hold targets between integer-divided physics steps (default: every 2)."""
        if self.clock is not None and self.clock.step_index % self.stride:
            return
        self.target.update(CurrentSimNanos * 1.0e-9)


def _create_ephemeris_interface(gravity_factory: Any, epoch_utc: str) -> Any:
    """Use one clock/frame for gravity and rendering, with physical planet spin."""

    ephemeris = gravity_factory.createSpiceInterface(
        time=epoch_utc,
        spicePlanetFrames=list(CELESTIAL_FIXED_FRAMES),
        epochInMsg=True,
    )
    ephemeris.referenceBase = SUPPORTED_EPHEMERIS_FRAME
    ephemeris.zeroBase = SUPPORTED_EPHEMERIS_CENTER
    return ephemeris


def _register_celestial_bodies(bridge: Any, earth: Any, sun: Any) -> None:
    """Share the gravity-bound body readers with the renderer; no backdrop Earth."""

    # Render the same Earth/Sun state messages used by native gravity.
    # The bridge subtracts the moving spacecraft origin for BOTH bodies.
    bridge.add_celestial_bodies(
        [earth, sun],
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


def _apply_orbital_initial_state(
    scene: Any,
    native: Any,
    earth: Any,
    orbit: dict[str, float],
    randomized: dict[str, Any],
    initial_joints: np.ndarray,
) -> dict[str, Any]:
    """Apply one saved orbital phase to the authoritative MJScene bodies.

    The render bridge reads these same bodies. Local grasp offsets, joint
    states and the legacy common drift are retained, while both free bodies
    receive the orbital position AND velocity for the selected phase.
    """
    from Basilisk.utilities import orbitalMotion

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
    common_velocity = np.asarray(native.COMMON_VELOCITY, dtype=float)
    bus_velocity = orbital_velocity + common_velocity
    bus = scene.getBody("cubesat_bus")
    target = scene.getBody("capture_target")
    bus.setPosition(orbital_position)
    bus.setVelocity(bus_velocity)
    target.setPosition(orbital_position + np.asarray(randomized["target_position_m"], dtype=float))
    # Both free bodies use the same inertial frame.  The randomized
    # target velocity is only a local offset, not its full orbital speed.
    target.setVelocity(
        orbital_velocity + np.asarray(randomized["target_linear_velocity_m_s"], dtype=float)
    )
    target.setAttitude(
        native._quaternion_to_mrp(
            np.asarray(randomized["target_orientation_wxyz"], dtype=float)
        )
    )
    target_spin = np.asarray(randomized["target_angular_velocity_rad_s"], dtype=float)
    if float(np.linalg.norm(target_spin)) > 1.0e-12:
        # Older scene files may still contain the pre-fix spin value.
        # Normalize it so they cannot reintroduce the MJScene startup NaN.
        target_spin = np.zeros(3)
    target.setAttitudeRate(target_spin)
    for (body_name, joint_name), position in zip(native.JOINTS, initial_joints, strict=True):
        joint = scene.getBody(body_name).getScalarJoint(joint_name)
        joint.setPosition(float(position))
        joint.setVelocity(0.0)

    # Extra target DOFs never enter the SARM arm/IK/actuator arrays.
    if "target_hinge_position_rad" in randomized:
        hinge = scene.getBody("satellite_outer_panel").getScalarJoint("outer_panel_hinge")
        hinge.setPosition(float(randomized["target_hinge_position_rad"]))
        hinge.setVelocity(0.0)

    orbit_state = {
        "semi_major_axis_m": float(elements.a),
        "initial_position_m": orbital_position.tolist(),
        "initial_velocity_m_s": bus_velocity.tolist(),
        "initial_altitude_m": float(np.linalg.norm(orbital_position) - float(earth.radEquator)),
        "initial_speed_m_s": float(np.linalg.norm(bus_velocity)),
    }
    return orbit_state


def run(args: argparse.Namespace) -> None:
    scene_instance = _load_scene_instance(args.scene_instance, args.model_root)
    if scene_instance:
        runtime = scene_instance["runtime"]
        args.simulation_rate = float(runtime["simulation_rate"])
        args.capture_rate = float(runtime["capture_rate_hz"])
        args.ik_rate = float(runtime["ik_rate_hz"])
    requested_step = getattr(args, "dynamics_step", None)
    if requested_step is None:
        requested_step = scene_instance["runtime"].get("dynamics_step_s", DEFAULT_DYNAMICS_STEP_S) if scene_instance else DEFAULT_DYNAMICS_STEP_S
    args.dynamics_step = validate_dynamics_step(requested_step)
    if (scene_instance is None or scene_instance.get("runtime", {}).get("dataset_capture", True)) and args.capture_rate not in SUPPORTED_FPS:
        raise ValueError("LeRobot capture rate must align with the 240 Hz dynamics / 30 Hz render grid: 1, 2, 5, 10, 30 Hz")
    adapter_root = args.adapter_root.resolve()
    ue_examples = adapter_root / "Unreal" / "BskUnrealRenderer" / "examples"
    sys.path.insert(0, str(adapter_root / "Adapters"))
    sys.path.insert(0, str(ue_examples))

    # This wrapper establishes the Windows-safe Basilisk/MuJoCo DLL load order.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    native = load_native_grasp_module(args.model_root.resolve())
    template_id = scene_instance["template_id"] if scene_instance else DEFAULT_TEMPLATE
    target_spec = capture_target(template_id)
    native.MODEL_PATH = target_spec.resolve_model(args.model_root)
    if not native.MODEL_PATH.is_file():
        raise FileNotFoundError(f"Selected target scene is missing: {native.MODEL_PATH}")
    print(json.dumps({"type": "capture_target_configuration", "template_id": template_id,
                      "runtime_model": target_spec.runtime_model, "collision_model": target_spec.collision_model,
                      "runtime_warning": target_spec.runtime_warning}, ensure_ascii=True), flush=True)
    native.TARGET_POS = np.asarray(target_spec.position_m, dtype=float)
    native.TARGET_QUAT = np.asarray(target_spec.orientation_wxyz, dtype=float)
    ik_step_stride(args.ik_rate)  # Reject stale 100 Hz scene configurations.
    native.TIME_STEP = 1.0 / DYNAMICS_HZ  # Initial period; RationalPhysicsClock drives every actual step.
    native.KP = np.asarray(native.KP, dtype=float).copy()
    native.KD = np.asarray(native.KD, dtype=float).copy()
    native.TORQUE_LIMITS = np.asarray(native.TORQUE_LIMITS, dtype=float).copy()
    native.KP[:6] = TELEOP_ARM_KP
    native.KD[:6] = TELEOP_ARM_KD
    native.TORQUE_LIMITS[:6] = TELEOP_ARM_TORQUE_LIMIT
    print(json.dumps({"type": "teleop_control_configuration", "dynamics_step_s": native.TIME_STEP,
                      "integrator": "RKF45", "ik_rate_hz": args.ik_rate,
                      "reference_governor": "bounded_reference_recovery_v2",
                      "reference_governor_settings": asdict(JointReferenceGovernor().settings),
                      "reference_recovery_settings": asdict(ReferenceRecoverySettings())}), flush=True)

    client = SimulationControlClient(args.control_host, args.control_port, "sarm-teleop")
    kinematics = SerialChainKinematics.from_mjcf(
        native.MODEL_PATH,
        base_body="cubesat_bus",
        joint_names=ARM_JOINT_NAMES,
        tool_site="sarm_ee",
    )
    initial_joints = (
        np.asarray(scene_instance["randomization"]["arm_joint_position_rad"], dtype=float)
        if scene_instance
        else np.asarray(BALANCED_TELEOP_HOME, dtype=float)
    )
    if scene_instance is None:
        initial_joints = np.asarray(ZERO_TELEOP_HOME, dtype=float)
        posture_report = {"status": "skipped", "reason": "zero_start_waiting_for_preparation"}
    else:
        posture_report = scene_instance.get("ik_initialization", {
            "status": "skipped", "reason": "saved_initial_state",
        })
    print(json.dumps({"type": "ik_initialization", **posture_report}, ensure_ascii=True), flush=True)
    client.start()
    try:
        while True:
            reset_request = _run_session(
                args, native, scene_instance, target_spec, initial_joints, kinematics, client
            )
            if reset_request is None:
                break
            # Dispose of the old native graph before creating a fresh one.
            # UE/Pixel Streaming and the backend control connection stay alive.
            with runtime_stage("release_previous_session"):
                gc.collect()
    finally:
        client.close()


def _run_session(
    args: argparse.Namespace, native: Any, scene_instance: dict[str, Any] | None,
    target_spec: Any, initial_joints: np.ndarray, kinematics: SerialChainKinematics,
    client: SimulationControlClient,
) -> str | None:
    """Build one clean physics/controller/ephemeris/render session from the saved initial conditions."""
    template_id = scene_instance["template_id"] if scene_instance else DEFAULT_TEMPLATE
    targets = CartesianTeleopTarget(
        initial_joints, client, kinematics, ik_mode=args.ik_mode,
        joint_limits=load_joint_limits(native.MODEL_PATH, SARM_JOINT_NAMES),
        preparation_required=scene_instance.get("arm_preparation_required", False) if scene_instance else True,
        operating_joint_deg=scene_instance.get("operating_arm_joint_position_deg", DEFAULT_OPERATING_JOINT_DEG) if scene_instance else DEFAULT_OPERATING_JOINT_DEG,
    )
    # Display-only metadata, prepared once from the exact limits used by this session.
    arm_joint_limits_rad = [
        [float(lo), float(hi)] if np.isfinite(lo) and np.isfinite(hi) else [None, None]
        for lo, hi in zip(targets.joint_min[:6], targets.joint_max[:6])
    ]
    reference_publisher = HeldJointReferencePublisher(targets.cached_reference, len(native.JOINTS))

    from Basilisk.simulation import NBodyGravity, pointMassGravityModel
    from Basilisk.utilities import macros, simIncludeGravBody
    from bsk_render_adapter import BasiliskRenderBridge, SceneSettings

    bridge: BasiliskRenderBridge | None = None
    gravity_factory = None
    module_registry = BasiliskModuleRegistry()
    try:
        with runtime_stage("build_physics"):
            simulation, scene, dynamics_models, recorders = native._build_simulation(
                attitude_control_enabled=False if getattr(args, "disable_attitude_control", False) else None,
                external_reference=reference_publisher,
                record_history=False,
            )
        integration = configure_scene_integrator(scene)
        print(json.dumps({"type": "native_integration_configuration", **integration,
                          "dynamics_rate_hz": DYNAMICS_HZ}, sort_keys=True), flush=True)
        print(json.dumps({
            "type": "attitude_control_configuration",
            "settings": simulation.attitude_control.settings,
            "enabled": simulation.attitude_control.enabled,
            "hardware_source": str(native.MODEL_PATH),
            "settings_source": str(native.MODEL_PATH.with_name("attitude_control.json")),
            "wheel_axes_body": [list(w.axis) for w in simulation.attitude_control.wheels],
            "spin_inertia_kg_m2": [w.spin_inertia for w in simulation.attitude_control.wheels],
            "max_torque_nm": [w.max_torque for w in simulation.attitude_control.wheels],
            "max_speed_rad_s": [w.max_speed for w in simulation.attitude_control.wheels],
        }, sort_keys=True), flush=True)
        ephemeris_environment = scene_instance.get("environment", {}) if scene_instance else {}
        ephemeris_epoch = str(
            ephemeris_environment.get("ephemeris_epoch_utc", DEFAULT_EPHEMERIS_EPOCH_UTC)
        )
        gravity_factory = simIncludeGravBody.gravBodyFactory()
        earth = gravity_factory.createEarth()
        sun = gravity_factory.createSun()
        with runtime_stage("load_ephemeris"):
            ephemeris = _create_ephemeris_interface(gravity_factory, ephemeris_epoch)

        # Register the environmental models with the MJScene dynamics task.
        # Basilisk computes the accelerations and MJScene performs the unified
        # multibody integration together with joints and contact.
        scene.extraEoMCall = True
        scene.AddModelToDynamicsTask(ephemeris, 20_000)
        gravity = NBodyGravity.NBodyGravity()
        gravity.ModelTag = "earthSunGravity"
        # Higher priorities run first.  MJScene forward kinematics (10_000)
        # must publish this substep's body states before gravity reads them;
        # trajectory/controllers (9_000 and below) then consume the fresh state.
        scene.AddModelToDynamicsTask(gravity, 9_500)

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
        # so the 240 Hz dynamics loop does not cross a Python bridge.
        print(json.dumps({"type": "ephemeris_configuration", "epoch_utc": ephemeris_epoch, "center": SUPPORTED_EPHEMERIS_CENTER, "frame": SUPPORTED_EPHEMERIS_FRAME, "planet_fixed_frames": dict(zip(("earth", "sun"), CELESTIAL_FIXED_FRAMES)), "gravity_sources": ["earth", "sun"], "gravity_targets": gravity_target_names}, sort_keys=True), flush=True)
        physics_task = next(task for task in simulation.TaskList if task.Name == "graspTask")
        physics_clock = RationalPhysicsClock(physics_task)
        module_registry.register("physics_clock", physics_clock, task_name="graspTask", priority=20_000)
        ik_controller = CartesianIkControlModel(
            targets, clock=physics_clock, stride=ik_step_stride(args.ik_rate))
        module_registry.register("teleop_ik", ik_controller, task_name="graspTask", priority=10_000)
        reference_publisher.clock = physics_clock
        reference_publisher.stride = ik_step_stride(args.ik_rate)
        module_registry.register("joint_reference_publisher", reference_publisher, task_name="graspTask", priority=9_999)
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
        dataset_capture = bool(scene_instance is None or scene_instance.get("runtime", {}).get("dataset_capture", True))
        bridge = BasiliskRenderBridge(
            reliable_frames=dataset_capture,
            host=args.render_host,
            port=args.render_port,
            origin_object="teleop/cubesat_bus",
            frame_rate_hz=RENDER_HZ,
        )
        bridge.add_mj_scene(
            scene,
            namespace="teleop",
            source_path=native.MODEL_PATH,
            mesh_asset_catalog=args.catalog.resolve(),
            semantic_label="spacecraft_robot_link",
            camera_picture_in_picture=True,
            camera_capture_rate_hz=args.capture_rate,
            camera_capture_products=("rgb",),
            camera_pip_resolution=(640, 360),
            camera_picture_in_picture_start_slot=1,
            camera_display_names={
                "spacecraft_overview": "Spacecraft Overview",
                "sarm_wrist_cam": "SARM Wrist Camera",
            },
        )
        _register_celestial_bodies(bridge, earth, sun)
        sunlight_scale = ephemeris_environment.get("lighting", {}).get(
            "sunlight_intensity_scale", DEFAULT_SUNLIGHT_INTENSITY_SCALE
        )
        print(json.dumps({"type": "lighting_configuration",
                          "sunlight_intensity_scale": sunlight_scale,
                          "scope": "rendering_only"}, sort_keys=True), flush=True)
        bridge.set_scene_settings(
            SceneSettings(
                sunlight_intensity_scale=sunlight_scale,
                origin_object_id="teleop/cubesat_bus",
                # Focus targets must be registered body IDs, not MJCF sites.
                default_camera_target="teleop/cubesat_bus",
                default_camera_distance_m=2.8,
                orbit_lines=False,
                trajectory_history=False,
                # 预览通道只保留很短的插值缓冲，并允许最多 50 ms 的视觉外推；
                # 训练采集由 UE 在权威帧上单独完成，不会记录这些平滑后的姿态。
                interpolation_delay_ms=15.0,
                max_extrapolation_ms=50.0,
            )
        )
        joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in native.JOINTS]

        def snapshot_observation(render_frame_id: int, render_sim_time_ns: int) -> dict[str, Any]:
            joint_position = [float(joint.stateOutMsg.read().state) for joint in joints]
            joint_velocity = [float(joint.stateDotOutMsg.read().state) for joint in joints]
            end_effector_position, end_effector_rotation = kinematics.forward(
                np.asarray(joint_position[:6])
            )
            end_effector_twist = kinematics.jacobian(
                np.asarray(joint_position[:6])
            ) @ np.asarray(joint_velocity[:6])
            return (
                {
                    "protocol": CONTROL_PROTOCOL,
                    "type": "observation",
                    "simulation_id": "sarm-teleop",
                    "reset_generation": client.reset_generation,
                    "render_session_id": bridge.session_id,
                    "scene_instance_id": scene_instance.get("instance_id") if scene_instance else None,
                    "scene_seed": scene_instance.get("seed") if scene_instance else None,
                    "capture_target": {
                        "model_file": target_spec.model_file,
                        "runtime_model": target_spec.runtime_model,
                        "collision_model": target_spec.collision_model,
                        "runtime_warning": target_spec.runtime_warning,
                        "synthetic_mass_kg": target_spec.synthetic_mass_kg,
                        "hinge_position_rad": float(scene.getBody(target_spec.hinge_body).getScalarJoint(target_spec.hinge_joint).stateOutMsg.read().state) if target_spec.hinge_joint else None,
                    },
                    "scene_template_id": template_id,
                    "step_id": str(render_frame_id + 1),
                    "observation_source": "authoritative_render_snapshot",
                    "render_frame_id": str(render_frame_id),
                    # 使用权威渲染帧的离散时间，保证状态和相机产品可逐帧严格配对。
                    "sim_time_ns": str(render_sim_time_ns),
                    "wall_time_ns": str(time.time_ns()),
                    "applied_action_sequence": targets.applied_sequence,
                    # Legacy aliases retain 8 mixed-unit entries for older clients.
                    # Explicit SI fields are authoritative for new consumers.
                    "arm_joint_position_rad": joint_position[:6],
                    "arm_joint_velocity_rad_s": joint_velocity[:6],
                    "target_arm_joint_position_rad": targets.position[:6].tolist(),
                    "arm_preparation": targets.preparation.telemetry(),
                    "gripper_position_m": joint_position[6:],
                    "gripper_velocity_m_s": joint_velocity[6:],
                    "target_gripper_position_m": targets.position[6:].tolist(),
                    **simulation.attitude_control.telemetry(),
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
                    "ik_mode": targets.ik_mode,
                    "ik_elbow_preference": {
                        **vars(targets.elbow_diagnostics),
                        "reference_height_m": float(kinematics.relative_joint_height(
                            targets.position[:6], up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.shoulder_joint,
                            elbow_joint=targets.elbow_preference.elbow_joint)[0]),
                        "measured_height_m": float(kinematics.relative_joint_height(
                            np.asarray(joint_position[:6]), up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.shoulder_joint,
                            elbow_joint=targets.elbow_preference.elbow_joint)[0]),
                        "preferred_height_m": targets.elbow_preference.preferred_height_m,
                        "reference_wrist_drop_m": float(kinematics.relative_joint_height(
                            targets.position[:6], up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.wrist_lower_joint,
                            elbow_joint=targets.elbow_preference.wrist_upper_joint)[0]),
                        "measured_wrist_drop_m": float(kinematics.relative_joint_height(
                            np.asarray(joint_position[:6]), up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.wrist_lower_joint,
                            elbow_joint=targets.elbow_preference.wrist_upper_joint)[0]),
                        "preferred_wrist_drop_m": targets.elbow_preference.preferred_wrist_drop_m,
                        "reference_joint3_rad": float(targets.position[ARM_JOINT_NAMES.index("joint3")]),
                        "measured_joint3_rad": float(joint_position[ARM_JOINT_NAMES.index("joint3")]),
                        "preferred_joint3_upper_rad": -targets.elbow_preference.joint3_negative_margin_rad,
                        "joint3_angle_scale_rad": targets.elbow_preference.joint3_angle_scale_rad,
                        "joint3_length_scale_m": targets.elbow_preference.joint3_length_scale_m,
                        "up_frame": "cubesat_bus",
                        "up_axis": list(targets.elbow_preference.up_axis),
                    },
                    'ik_status': targets.solver_status,
                    'ik_reasons': targets.solver_reasons,
                    'ik_solve_time_ms': targets.solve_time_ms,
                    'operator_twist_body': targets.raw_operator_twist.tolist(),
                    'expected_twist_body': targets.desired_twist.tolist(),
                    'predicted_twist_body': targets.achieved_twist.tolist(),
                    'arm_joint_limits_rad': arm_joint_limits_rad,
                    'joint_limit_margin_rad': [
                        float(min(q-lo, hi-q)) if np.isfinite(lo) or np.isfinite(hi) else None
                        for q, lo, hi in zip(joint_position[:6], targets.joint_min[:6], targets.joint_max[:6])],
                    'motion_speed_diagnostics': targets.speed_monitor.latest,
                    'arm_actuator_diagnostics': targets.actuator_diagnostics,
                    "ik_damping": float(targets.ik_damping),
                    "ik_velocity_scale": float(targets.velocity_scale),
                    "ik_minimum_singular_value": float(targets.minimum_singular_value),
                    "ik_condition_number": float(min(targets.condition_number, 1.0e9)),
                    "ik_nullspace_correction_norm": float(targets.nullspace_correction_norm),
                    "command_stale": targets.command_stale,
                    "ik_control_rate_hz": args.ik_rate,
                    "dynamics_rate_hz": DYNAMICS_HZ,
                    "ik_update_count": str(targets.update_count),
                    "ik_solve_count": str(targets.ik_solve_count),
                    "tracking_scale": float(targets.tracking_scale),
                    "reference_governor_state": targets.governor_state,
                    "reference_controller_version": "bounded_reference_recovery_v2",
                    "reference_joint_error_rad": (targets.position[:6] - np.asarray(joint_position[:6])).tolist(),
                    "reference_effort_ratio": targets.arm_effort_ratio.tolist(),
                    "reference_blocked_joints": (np.flatnonzero(getattr(targets.governor, "blocked", np.zeros(6))) + 1).tolist(),
                    "reference_saturation_seconds": getattr(targets.governor, "saturation_seconds", np.zeros(6)).tolist(),
                    "reference_limited_joints": list(targets.governor_limited_joints),
                    "dynamics_step_s": native.TIME_STEP,
                    "position_tracking_error_m": float(
                        np.linalg.norm(targets.position_error)
                    ),
                    "orientation_tracking_error_rad": float(
                        np.linalg.norm(targets.orientation_error)
                    ),
                }
            )

        observation_snapshots = AuthoritativeObservationModel(bridge, snapshot_observation)
        module_registry.register("authoritative_observation_snapshot", observation_snapshots,
                                 task_name="graspTask", priority=-10_001)

        module_registry.register("render_state_publisher", bridge, task_name="graspTask", priority=-10_000)
        module_registry.attach(simulation)
        with runtime_stage("initialize_state"):
            native._initialize_state(simulation, scene)
        environment = scene_instance.get("environment", {}) if scene_instance else {}
        orbit = environment.get("orbit", DEFAULT_ORBIT)
        randomized = scene_instance["randomization"] if scene_instance else {
            "target_position_m": list(native.TARGET_POS),
            "target_orientation_wxyz": list(native.TARGET_QUAT),
            "target_linear_velocity_m_s": list(native.COMMON_VELOCITY),
            "target_angular_velocity_rad_s": list(native.TARGET_SPIN),
        }
        if target_spec.hinge_joint:
            randomized.setdefault("target_hinge_position_rad", 0.0)
        orbit_state = _apply_orbital_initial_state(
            scene, native, earth, orbit, randomized, initial_joints
        )
        print(
            json.dumps(
                {
                    "type": "gravity_configuration",
                    "sources": ["earth", "sun"],
                    "central_body": "earth",
                    "gravity_targets": gravity_target_names,
                    "orbit": orbit,
                    "randomize_orbit_phase": bool(scene_instance and scene_instance["randomize_orbit_phase"]),
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
        targets.bind_joint_state_provider(lambda: np.asarray(
            [float(joint.stateOutMsg.read().state) for joint in joints[:6]]))
        targets.bind_joint_velocity_provider(lambda: np.asarray(
            [float(joint.stateDotOutMsg.read().state) for joint in joints[:6]]))
        models_by_tag = {model.ModelTag: model for model in dynamics_models}
        pid_models = [models_by_tag[f"{name}PID"] for name in ARM_JOINT_NAMES]
        limiter_models = [models_by_tag[f"{name}TorqueLimiter"] for name in ARM_JOINT_NAMES]
        targets.bind_actuator_state_provider(lambda: {
            "requested_torque_nm": [float(model.outputOutMsg.read().input) for model in pid_models],
            "applied_torque_nm": [float(model.actuatorOutMsg.read().input) for model in limiter_models],
            "torque_limits_nm": TELEOP_ARM_TORQUE_LIMIT.tolist(),
        })
        arm_actuators = [scene.getSingleActuator(name) for name in native.ACTUATORS[:6]]
        targets.bind_effort_provider(
            lambda: np.asarray([abs(float(actuator.actuatorInMsg().input)) for actuator in arm_actuators]) / native.TORQUE_LIMITS[:6]
        )

        wall_start = time.monotonic()
        processing_seconds = 0.0
        frame_count = int(math.ceil(args.duration * RENDER_HZ)) if args.duration > 0.0 else None
        if client.reset_generation:
            bridge.publish_event("scene_reset", {"request_id": client.reset_generation, "sim_time_ns": "0"})
        frame = 0
        while frame_count is None or frame < frame_count:
            reset_request = client.take_reset()
            if reset_request is not None:
                print(json.dumps({"type": "reset_accepted", "request_id": reset_request,
                                  "wall_time_ns": str(time.time_ns())}), flush=True)
                return reset_request
            frame += 1
            stop_ns = tick_time_ns(frame, RENDER_HZ)
            sim_seconds = stop_ns * 1.0e-9
            if frame_count is not None:
                stop_ns = min(stop_ns, macros.sec2nano(args.duration))
                sim_seconds = stop_ns * 1.0e-9
            deadline = wall_start + sim_seconds / args.simulation_rate
            time.sleep(max(0.0, deadline - time.monotonic()))
            processing_start = time.monotonic()
            simulation.ConfigureStopTime(stop_ns)
            simulation.ExecuteSimulation()
            if frame == 1:
                client.complete_reset()
            for observation_snapshot in observation_snapshots.drain():
                client.send_observation(observation_snapshot)
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
                        "dynamics_rate_hz": DYNAMICS_HZ,
                        "ik_update_count": targets.update_count,
                        "ik_solve_count": targets.ik_solve_count,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        _ = keep_alive
    finally:
        if bridge:
            with runtime_stage("close_render_bridge"):
                bridge.close()
        if gravity_factory is not None:
            with runtime_stage("unload_ephemeris"):
                gravity_factory.unloadSpiceKernels()


def main() -> None:
    workspace = PROJECT_ROOT.parent
    default_adapter = workspace / "space_sim_UE_adapter"
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
                        help="Online rational clock period: 1/240 seconds; other periods are rejected")
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
    args = parser.parse_args()
    if args.ik_mode not in IK_MODES:
        parser.error("SPACE_SIM_IK_MODE must be ik_pose or strict; constrained has been removed. "
                     "Unset the old environment override or select --ik-mode ik_pose.")
    if args.catalog is None:
        catalog_name = "sarm_platform.catalog.json" if args.model_root.name == "platform" else "cubesat_so101.catalog.json"
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
            "ik-rate must divide 240 Hz (default 120 Hz)"
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
