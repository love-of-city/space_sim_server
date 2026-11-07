"""Public capture-arm commands and ideal encoder measurements, without truth."""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import Contract, Count, Number, Vector3

Arm6 = tuple[Number, Number, Number, Number, Number, Number]
Jaws4 = tuple[Number, Number, Number, Number]


class CaptureArmCommand(Contract):
    """TCP twist plus original axial/radial mechanism rates.

    The Cartesian limits are an explicit benchmark command envelope, not a
    claim about flight hardware. The original IK/joint-rate/lead and servo-force
    limits still apply separately. No direct state, target pose or force write.
    """
    schema_version: Literal['spacesim-autonomy-capture-arm-command/1'] = 'spacesim-autonomy-capture-arm-command/1'
    enabled: StrictBool = False
    tool_site: Literal['grip_frame'] = 'grip_frame'
    base_body: Literal['cubesat_bus'] = 'cubesat_bus'
    linear_velocity_tcp_in_bus_m_s: Vector3 = (0., 0., 0.)
    angular_velocity_in_bus_rad_s: Vector3 = (0., 0., 0.)
    angular_velocity_in_tool_rad_s: Vector3 = (0., 0., 0.)
    joint6_velocity_rad_s: Annotated[Number, Field(ge=-.25, le=.25)] = 0.
    axial_velocity_m_s: Annotated[Number, Field(ge=-.003, le=.003)] = 0.
    radial_velocity_m_s: Annotated[Number, Field(ge=-.003, le=.003)] = 0.

    @field_validator('linear_velocity_tcp_in_bus_m_s')
    @classmethod
    def bounded_translation(cls, value):
        if math.hypot(*value) > .02:
            raise ValueError('capture TCP translation command exceeds 0.02 m/s')
        return value

    @field_validator('angular_velocity_in_bus_rad_s', 'angular_velocity_in_tool_rad_s')
    @classmethod
    def bounded_rotation(cls, value):
        if math.hypot(*value) > .1:
            raise ValueError('capture angular command exceeds 0.1 rad/s per declared frame')
        return value

    @model_validator(mode='after')
    def disabled_has_no_motion(self):
        if not self.enabled and any((*self.linear_velocity_tcp_in_bus_m_s,
                *self.angular_velocity_in_bus_rad_s, *self.angular_velocity_in_tool_rad_s,
                self.joint6_velocity_rad_s, self.axial_velocity_m_s, self.radial_velocity_m_s)):
            raise ValueError('disabled capture command must contain zero motion')
        return self

    def legacy_action(self):
        """Only the original command fields; ownership remains capture-only."""
        return dict(deadman=self.enabled, active_arm='capture', arm_preparation=False,
            end_effector_linear_velocity_body_m_s=list(self.linear_velocity_tcp_in_bus_m_s),
            end_effector_angular_velocity_body_rad_s=list(self.angular_velocity_in_bus_rad_s),
            end_effector_angular_velocity_tool_rad_s=list(self.angular_velocity_in_tool_rad_s),
            joint6_velocity_rad_s=self.joint6_velocity_rad_s,
            capture_axial_velocity_m_s=self.axial_velocity_m_s,
            capture_radial_velocity_m_s=self.radial_velocity_m_s)


class CaptureEncoders(Contract):
    """Instantaneous ideal joint encoders; not body pose or contact feedback.

    Six arm channels are bottom_joint1..6; jaw channels are jaw_slide_0..3.
    Constraint overshoot is retained in measurements for independent safety
    grading, rather than clipped into an apparently valid configuration.
    """
    schema_version: Literal['spacesim-autonomy-capture-encoders/1'] = 'spacesim-autonomy-capture-encoders/1'
    episode_id: Annotated[str, Field(strict=True, min_length=1, max_length=128)]
    sample_time_ns: Count
    sensor_model: Literal['ideal_joint_encoders_no_noise_no_delay'] = 'ideal_joint_encoders_no_noise_no_delay'
    arm_position_rad: Arm6
    arm_velocity_rad_s: Arm6
    axial_position_m: Number
    axial_velocity_m_s: Number
    cam_angle_rad: Number
    cam_velocity_rad_s: Number
    jaw_position_m: Jaws4
    jaw_velocity_m_s: Jaws4

    def positions(self):
        return (*self.arm_position_rad, self.axial_position_m, self.cam_angle_rad, *self.jaw_position_m)

    def velocities(self):
        return (*self.arm_velocity_rad_s, self.axial_velocity_m_s, self.cam_velocity_rad_s, *self.jaw_velocity_m_s)
