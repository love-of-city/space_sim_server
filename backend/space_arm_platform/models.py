"""Validated API and simulation-wire models."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


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
    input_source: str
    limited: bool = False
    reason: str = ""


class SimulationHello(BaseModel):
    model_config = ConfigDict(extra="allow")

    protocol: Literal[CONTROL_PROTOCOL]
    type: Literal["sim_hello"]
    simulation_id: str
    capabilities: list[str] = []


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
    joint_position_rad: list[float] = Field(min_length=6, max_length=6)
    joint_velocity_rad_s: list[float] = Field(min_length=6, max_length=6)
    target_joint_position_rad: list[float] = Field(min_length=6, max_length=6)
    end_effector_position_body_m: list[float] = Field(default_factory=lambda: [0.0] * 3, min_length=3, max_length=3)
    end_effector_orientation_body_wxyz: list[float] = Field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0], min_length=4, max_length=4)
    end_effector_twist_body: list[float] = Field(default_factory=lambda: [0.0] * 6, min_length=6, max_length=6)
    cartesian_command_residual: list[float] = Field(default_factory=lambda: [0.0] * 6, min_length=6, max_length=6)
    jacobian_rank: int = Field(default=0, ge=0, le=5)
    command_stale: bool = False

    @field_validator("step_id", "render_frame_id", "sim_time_ns", "wall_time_ns", "applied_action_sequence")
    @classmethod
    def validate_decimal_identifier(cls, value: str) -> str:
        if not value.isdecimal():
            raise ValueError("step/frame/time identifiers must be decimal strings")
        return value


class EpisodeStart(BaseModel):
    task: str = "spacecraft arm teleoperation"
    instruction: str = "控制太空机械臂接近并抓取目标"
    operator: str = "operator"
    seed: int | None = None
    tags: list[str] = []


class EpisodeStop(BaseModel):
    outcome: Literal["success", "failure", "aborted", "unknown"] = "unknown"
    note: str = ""
