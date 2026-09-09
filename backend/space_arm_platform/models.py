"""Validated API and simulation-wire models."""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, field_validator, model_validator

from .lighting import DEFAULT_SUNLIGHT_INTENSITY_SCALE, MAX_SUNLIGHT_INTENSITY_SCALE


CONTROL_PROTOCOL = "space-arm-control/1"


class OperatorAction(BaseModel):
    """Normalized browser input before safety scaling."""

    model_config = ConfigDict(extra="forbid")

    type: Literal["operator_action"] = "operator_action"
    client_sequence: int = Field(ge=0)
    client_time_ns: str
    deadman: bool
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
    episode_id: str | None = None
    server_sequence: str
    server_time_ns: str
    client_sequence: str
    client_time_ns: str
    deadman: bool
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


class SimulationObservation(BaseModel):
    model_config = ConfigDict(extra="allow")

    protocol: Literal[CONTROL_PROTOCOL]
    type: Literal["observation"]
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

    @field_validator("step_id", "render_frame_id", "sim_time_ns", "wall_time_ns", "applied_action_sequence")
    @classmethod
    def validate_decimal_identifier(cls, value: str) -> str:
        if not value.isdecimal():
            raise ValueError("step/frame/time identifiers must be decimal strings")
        return value


class EpisodeStart(BaseModel):
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


class EpisodeStop(BaseModel):
    outcome: Literal["success", "failure", "aborted", "unknown"] = "unknown"
    note: str = ""


class SceneInstanceCreate(BaseModel):
    model_config = ConfigDict(extra="forbid")

    template_id: str = "spacecraft-arm-teleop"
    randomization_profile: str = "training-v1"
    randomize_orbit_phase: bool = Field(
        default=False, strict=True,
        description="Sample a seed-reproducible starting phase on the current circular orbit, independently of local grasp randomization.",
    )
    seed: int | None = Field(default=None, ge=0, le=2**31 - 1)
    simulation_rate: float = Field(default=1.0, gt=0.0, le=100.0)
    capture_rate_hz: float = Field(default=10.0, gt=0.0, le=60.0)
    ik_rate_hz: float = Field(default=100.0, ge=1.0, le=500.0)
    dataset_capture: bool = False
    sunlight_intensity_scale: FiniteFloat = Field(
        default=DEFAULT_SUNLIGHT_INTENSITY_SCALE, strict=True,
        ge=0.0, le=MAX_SUNLIGHT_INTENSITY_SCALE,
        description="Scene solar illumination multiplier; 1 preserves current lighting, 0 disables direct sunlight.",
    )


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
