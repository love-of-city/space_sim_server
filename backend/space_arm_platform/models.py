"""Validated API and simulation-wire models."""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, field_validator, model_validator

from .lighting import DEFAULT_SUNLIGHT_INTENSITY_SCALE, MAX_SUNLIGHT_INTENSITY_SCALE
from .control_defaults import (
    DEFAULT_RANDOMIZATION_PROFILE, DEFAULT_OPERATING_JOINT_DEG, DEFAULT_DYNAMICS_STEP_S,
    MIN_DYNAMICS_STEP_S, MAX_DYNAMICS_STEP_S,
)
from .scene_targets import DEFAULT_TEMPLATE
from .sampling import DEFAULT_CAPTURE_HZ, DEFAULT_IK_HZ, SUPPORTED_FPS, ik_step_stride


CONTROL_PROTOCOL = "space-arm-control/1"


class ArmPreparationRequest(BaseModel):
    """Repeated heartbeat for one explicit, cancellable preparation request."""
    model_config = ConfigDict(extra="forbid")
    request_id: str = Field(min_length=1, max_length=80)
    joint_position_deg: list[Annotated[FiniteFloat, Field(strict=True)]] = Field(min_length=6, max_length=6)


class OperatorAction(BaseModel):
    """Normalized browser input before safety scaling."""

    model_config = ConfigDict(extra="forbid")

    type: Literal["operator_action"] = "operator_action"
    client_sequence: int = Field(ge=0)
    client_time_ns: str
    deadman: bool
    arm_preparation: ArmPreparationRequest | None = None
    allow_reference_recovery: bool = Field(default=False, strict=True)
    end_effector_linear_speed_m_s: float = Field(default=0.05, gt=0.0)
    end_effector_linear_velocity: list[float] = Field(min_length=3, max_length=3)
    end_effector_angular_velocity: list[float] = Field(min_length=3, max_length=3)
    gripper_velocity: float = 0.0
    input_source: Literal["keyboard", "gamepad", "unknown"] = "unknown"

    @field_validator("client_time_ns")
    @classmethod
    def validate_time(cls, value: str) -> str:
        if not value.isdecimal():
            raise ValueError("client_time_ns must be a decimal string")
        return value


class AppliedAction(BaseModel):
    """Safety-filtered command sent to the authoritative simulator."""

    protocol: Literal[CONTROL_PROTOCOL] = CONTROL_PROTOCOL
    type: Literal["action"] = "action"
    reset_generation: str = ""
    episode_id: str | None = None
    server_sequence: str
    server_time_ns: str
    client_sequence: str
    client_time_ns: str
    deadman: bool
    arm_preparation: ArmPreparationRequest | None = None
    allow_reference_recovery: bool = False
    control_frame: Literal["spacecraft_body"] = "spacecraft_body"
    requested_end_effector_linear_velocity_normalized: list[float] = Field(
        default_factory=lambda: [0.0] * 3, min_length=3, max_length=3
    )
    requested_end_effector_angular_velocity_normalized: list[float] = Field(
        default_factory=lambda: [0.0] * 3, min_length=3, max_length=3
    )
    requested_gripper_velocity_normalized: float = 0.0
    requested_end_effector_linear_speed_m_s: float = 0.05
    applied_end_effector_linear_speed_m_s: float = 0.05
    end_effector_linear_velocity_body_m_s: list[float] = Field(min_length=3, max_length=3)
    end_effector_angular_velocity_body_rad_s: list[float] = Field(min_length=3, max_length=3)
    gripper_velocity_rad_s: float
    gripper_velocity_m_s: float = 0.0
    input_source: str
    limited: bool = False
    reason: str = ""


class SimulationHello(BaseModel):
    model_config = ConfigDict(extra="allow")

    protocol: Literal[CONTROL_PROTOCOL]
    type: Literal["sim_hello"]
    simulation_id: str
    capabilities: list[str] = []
    reset_generation: str = ""


Vector3 = Annotated[list[FiniteFloat], Field(min_length=3, max_length=3)]
Quaternion = Annotated[list[FiniteFloat], Field(min_length=4, max_length=4)]
ArmVector = Annotated[list[FiniteFloat], Field(min_length=6, max_length=6)]
FingerVector = Annotated[list[FiniteFloat], Field(min_length=2, max_length=2)]
WheelFlags = Annotated[list[bool], Field(min_length=3, max_length=3)]


class AttitudeControlObservation(BaseModel):
    """Truth attitude and controller status; quaternions use w-x-y-z order."""
    model_config = ConfigDict(extra="forbid")
    enabled: bool
    mode: Literal["initial_inertial_hold"]
    reference_frame: Literal["J2000"]
    reference_initialized: bool
    state_time_ns: str = Field(pattern=r"^[0-9]+$")
    control_time_ns: str = Field(pattern=r"^[0-9]+$")
    orientation_inertial_wxyz: Quaternion
    reference_orientation_inertial_wxyz: Quaternion
    angular_velocity_body_rad_s: Vector3
    attitude_error_angle_rad: FiniteFloat = Field(ge=0.0, le=3.141592653589794)
    control_rate_hz: FiniteFloat = Field(gt=0.0, le=500.0)
    saturated: bool


class ReactionWheelObservation(BaseModel):
    """Wheel-relative speeds/momenta and drive torques, never arm joints."""
    model_config = ConfigDict(extra="forbid")
    names: list[str] = Field(min_length=3, max_length=3)
    speed_rad_s: Vector3
    relative_momentum_nms: Vector3
    requested_motor_torque_nm: Vector3
    applied_motor_torque_nm: Vector3
    max_motor_torque_nm: Vector3
    max_speed_rad_s: Vector3
    torque_limited: WheelFlags
    speed_limited: WheelFlags
    overspeed: WheelFlags


class MotionLimitReason(BaseModel):
    code: str
    joints: list[int] = Field(default_factory=list)
    detail: str = ""


class MotionSpeedChannel(BaseModel):
    active: bool = False
    warning: bool = False
    expected_speed: FiniteFloat = 0.0
    actual_speed_along_command: FiniteFloat = 0.0
    predicted_speed_along_command: FiniteFloat = 0.0
    lateral_speed: FiniteFloat = 0.0
    ratio: FiniteFloat | None = None
    reasons: list[MotionLimitReason] = Field(default_factory=list)
    state: str = "idle"


class MotionSpeedDiagnostics(BaseModel):
    frame: Literal["spacecraft_body"] = "spacecraft_body"
    control_point: str = "sarm_ee"
    threshold: FiniteFloat = 0.8
    enabled: bool = False
    command_stale: bool = False
    measurement_valid: bool = False
    linear: MotionSpeedChannel | None = None
    angular: MotionSpeedChannel | None = None


class SimulationObservation(BaseModel):
    model_config = ConfigDict(extra="allow")

    protocol: Literal[CONTROL_PROTOCOL]
    type: Literal["observation"]
    reset_generation: str = ""
    render_session_id: str = ""
    simulation_id: str
    step_id: str
    render_frame_id: str
    sim_time_ns: str
    wall_time_ns: str
    applied_action_sequence: str
    joint_position_rad: list[FiniteFloat] = Field(min_length=6, max_length=8)
    joint_velocity_rad_s: list[FiniteFloat] = Field(min_length=6, max_length=8)
    target_joint_position_rad: list[FiniteFloat] = Field(min_length=6, max_length=8)
    end_effector_position_body_m: list[float] = Field(default_factory=lambda: [0.0] * 3, min_length=3, max_length=3)
    end_effector_orientation_body_wxyz: list[float] = Field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0], min_length=4, max_length=4)
    end_effector_twist_body: list[float] = Field(default_factory=lambda: [0.0] * 6, min_length=6, max_length=6)
    cartesian_command_residual: list[float] = Field(default_factory=lambda: [0.0] * 6, min_length=6, max_length=6)
    jacobian_rank: int = Field(default=0, ge=0, le=6)
    # Differential IK diagnostics published by the SARM simulator.
    ik_mode: str = ""
    ik_elbow_preference: dict[str, Any] = Field(default_factory=dict)
    arm_preparation: dict[str, Any] = Field(default_factory=dict)
    ik_status: str = ""
    ik_reasons: list[MotionLimitReason] = Field(default_factory=list)
    ik_solve_time_ms: FiniteFloat = 0.0
    operator_twist_body: list[FiniteFloat] = Field(default_factory=lambda: [0.0]*6, min_length=6, max_length=6)
    expected_twist_body: list[FiniteFloat] = Field(default_factory=lambda: [0.0]*6, min_length=6, max_length=6)
    predicted_twist_body: list[FiniteFloat] = Field(default_factory=lambda: [0.0]*6, min_length=6, max_length=6)
    joint_limit_margin_rad: list[FiniteFloat | None] | None = Field(default=None, min_length=6, max_length=6)
    # J1-J6, model-zero coordinates; [None, None] denotes a continuous joint.
    arm_joint_limits_rad: list[tuple[FiniteFloat | None, FiniteFloat | None]] | None = Field(
        default=None, min_length=6, max_length=6
    )
    motion_speed_diagnostics: MotionSpeedDiagnostics | None = None
    ik_damping: FiniteFloat = 0.0
    ik_velocity_scale: FiniteFloat = 1.0
    ik_minimum_singular_value: FiniteFloat = 0.0
    ik_condition_number: FiniteFloat = 0.0
    ik_nullspace_correction_norm: FiniteFloat = 0.0
    command_stale: bool = False
    # Optional only for pre-SARM peers. New SARM publishes all six SI fields.
    arm_joint_position_rad: ArmVector | None = None
    arm_joint_velocity_rad_s: ArmVector | None = None
    target_arm_joint_position_rad: ArmVector | None = None
    gripper_position_m: FingerVector | None = None
    gripper_velocity_m_s: FingerVector | None = None
    target_gripper_position_m: FingerVector | None = None
    attitude_control: AttitudeControlObservation | None = None
    reaction_wheels: ReactionWheelObservation | None = None

    @model_validator(mode="after")
    def validate_joint_layout(self):
        """Accept legacy six or SARM eight, but never seven/mismatched arrays."""
        count = len(self.joint_position_rad)
        if count not in (6, 8) or any(len(v) != count for v in (
            self.joint_velocity_rad_s, self.target_joint_position_rad
        )):
            raise ValueError("joint arrays must have the same length: legacy 6 or SARM 8")
        pairs = (
            (self.arm_joint_position_rad, self.gripper_position_m, self.joint_position_rad),
            (self.arm_joint_velocity_rad_s, self.gripper_velocity_m_s, self.joint_velocity_rad_s),
            (self.target_arm_joint_position_rad, self.target_gripper_position_m, self.target_joint_position_rad),
        )
        if any(arm is not None or fingers is not None for arm, fingers, _ in pairs):
            if count != 8 or any(arm is None or fingers is None or arm + fingers != legacy
                                 for arm, fingers, legacy in pairs):
                raise ValueError("SARM SI fields must be complete and match legacy aliases")
        if (self.attitude_control is None) != (self.reaction_wheels is None):
            raise ValueError("attitude and wheel telemetry must be supplied together")
        return self

    @field_validator("arm_joint_limits_rad")
    @classmethod
    def validate_arm_limits(cls, value):
        if value is not None:
            for lower, upper in value:
                if (lower is None) != (upper is None):
                    raise ValueError("joint limits must be a finite pair or both null")
                if lower is not None and lower >= upper:
                    raise ValueError("joint lower limit must be less than upper limit")
        return value

    @field_validator("step_id", "render_frame_id", "sim_time_ns", "wall_time_ns", "applied_action_sequence")
    @classmethod
    def validate_decimal_identifier(cls, value: str) -> str:
        if not value.isdecimal():
            raise ValueError("step/frame/time identifiers must be decimal strings")
        return value


class EpisodeStart(BaseModel):
    fps: Literal[1, 2, 5, 10, 30] = DEFAULT_CAPTURE_HZ
    camera_ids: list[str] = Field(default_factory=lambda: [
        "teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam"])
    capture_products: list[Literal["rgb"]] = Field(
        default_factory=lambda: ["rgb"])
    max_frames: int = Field(default=18000, ge=1, le=108000)

    task_id: str | None = None
    task: str = "spacecraft arm teleoperation"
    instruction: str = "控制太空机械臂接近并抓取目标"
    operator: str = "operator"
    seed: int | None = None
    tags: list[str] = []
    scene_instance: dict[str, Any] | None = None
    operator_user_id: str | None = None
    operator_username: str | None = None
    operator_role: Literal["admin", "operator"] | None = None

    @model_validator(mode="after")
    def validate_dataset_features(self):
        from .lerobot_capture import camera_key
        keys = [camera_key(camera) for camera in self.camera_ids]
        if any(not camera.strip() for camera in self.camera_ids) or len(set(keys)) != len(keys):
            raise ValueError("camera IDs must be nonempty and have unique dataset keys")
        if len(set(self.capture_products)) != len(self.capture_products):
            raise ValueError("duplicate capture products")
        if self.camera_ids and "rgb" not in self.capture_products:
            raise ValueError("RGB is required for camera datasets")
        return self


class EpisodeStop(BaseModel):
    outcome: Literal["success", "failure", "aborted", "unknown"] = "unknown"
    note: str = ""


class SceneInstanceCreate(BaseModel):
    model_config = ConfigDict(extra="forbid")

    template_id: str = DEFAULT_TEMPLATE
    randomization_profile: str = DEFAULT_RANDOMIZATION_PROFILE
    randomize_orbit_phase: bool = Field(
        default=False, strict=True,
        description="Sample a seed-reproducible starting phase on the current circular orbit, independently of local grasp randomization.",
    )
    initial_arm_joint_position_deg: list[Annotated[FiniteFloat, Field(strict=True)]] | None = Field(
        default=None, min_length=6, max_length=6,
        description="Optional J1-J6 initial angles in model-zero degrees. Overrides arm randomization only; fingers are unchanged.",
    )
    operating_arm_joint_position_deg: list[Annotated[FiniteFloat, Field(strict=True)]] = Field(
        default_factory=lambda: list(DEFAULT_OPERATING_JOINT_DEG), min_length=6, max_length=6,
        description="Operating target, not the startup state. Degrees, no angle wrapping.",
    )
    seed: int | None = Field(default=None, ge=0, le=2**31 - 1)
    simulation_rate: float = Field(default=1.0, gt=0.0, le=100.0)
    capture_rate_hz: float = Field(default=DEFAULT_CAPTURE_HZ, gt=0.0, le=60.0)
    ik_rate_hz: float = Field(default=DEFAULT_IK_HZ, ge=1.0, le=240.0)
    dynamics_step_s: FiniteFloat = Field(
        default=DEFAULT_DYNAMICS_STEP_S, ge=MIN_DYNAMICS_STEP_S, le=MAX_DYNAMICS_STEP_S,
    )
    dataset_capture: bool = True
    sunlight_intensity_scale: FiniteFloat = Field(
        default=DEFAULT_SUNLIGHT_INTENSITY_SCALE, strict=True,
        ge=0.0, le=MAX_SUNLIGHT_INTENSITY_SCALE,
        description="Scene solar illumination multiplier; 1 preserves current lighting, 0 disables direct sunlight.",
    )

    @model_validator(mode="after")
    def validate_dataset_sampling(self):
        ik_step_stride(self.ik_rate_hz)
        if self.dataset_capture and self.capture_rate_hz not in SUPPORTED_FPS:
            raise ValueError(f"LeRobot capture FPS must be representable by the dynamics and render clocks: {SUPPORTED_FPS}")
        return self


class LoginRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    username: str = Field(min_length=1, max_length=64)
    password: str = Field(min_length=1, max_length=256)


class OperatorCreate(BaseModel):
    model_config = ConfigDict(extra="forbid")
    username: str = Field(min_length=3, max_length=64)
    password: str = Field(min_length=8, max_length=256)


class PasswordReset(BaseModel):
    model_config = ConfigDict(extra="forbid")
    password: str = Field(min_length=8, max_length=256)


class PasswordChange(BaseModel):
    model_config = ConfigDict(extra="forbid")
    current_password: str = Field(min_length=1, max_length=256)
    new_password: str = Field(min_length=8, max_length=256)


class TaskCreate(BaseModel):
    instruction: str
    scene_id: str = "orbital-grasp"
    seed: int | None = None
    tags: list[str] = []


class TaskComplete(BaseModel):
    outcome: Literal["success", "failure", "aborted"]
    note: str = ""
