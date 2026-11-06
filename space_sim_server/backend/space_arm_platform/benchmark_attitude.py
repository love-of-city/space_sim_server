"""Single-spacecraft attitude task and message contracts, without simulator imports."""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import Contract, Count, Number, Nonnegative, Positive, SeedSplits, Vector3

Quaternion = tuple[Number, Number, Number, Number]
Tensor3 = tuple[Vector3, Vector3, Vector3]


def unit_quaternion(value):
    if not math.isclose(math.sqrt(sum(component**2 for component in value)), 1., abs_tol=1e-8, rel_tol=0):
        raise ValueError("quaternion must have unit norm; component order is w,x,y,z")
    return value


class AttitudeInitialState(Contract):
    altitude_m: Annotated[Number, Field(ge=200_000, le=2_000_000)]
    inclination_deg: Annotated[Number, Field(ge=0, le=180)]
    true_anomaly_deg: Number
    # Uniform angle and isotropic axis relative to the commanded orientation.
    rotation_min_rad: Annotated[Number, Field(gt=0, le=math.pi)]
    rotation_max_rad: Annotated[Number, Field(gt=0, le=math.pi)]
    body_rate_half_width_rad_s: Nonnegative
    wheel_speed_rad_s: Vector3


class AttitudeObjective(Contract):
    orientation_body_to_j2000_wxyz: Quaternion
    angle_tolerance_rad: Annotated[Number, Field(gt=0, lt=math.pi)]
    body_rate_tolerance_rad_s: Positive
    dwell_s: Positive

    _normalized = field_validator("orientation_body_to_j2000_wxyz")(unit_quaternion)


class AttitudeSafety(Contract):
    maximum_body_rate_rad_s: Positive
    maximum_wheel_speed_rad_s: Positive
    maximum_motor_torque_nm: Positive


class AttitudeObservationModel(Contract):
    kind: Literal["noisy_absolute_attitude_telemetry_and_gyroscope"]
    orientation_noise_std_rad: Positive
    gyro_white_noise_std_rad_s: Positive
    gyro_initial_bias_std_rad_s: Nonnegative
    gyro_bias_random_walk_rad_s_sqrt_s: Nonnegative
    gyro_range_rad_s: Positive
    wheel_encoder: Literal["ideal_joint_rate_encoder"]
    delay_s: Literal[0.0]

    @field_validator("delay_s", mode="before")
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (float, int):
            raise ValueError("delay must be numeric")
        return value


class AttitudeTask(Contract):
    schema_version: Literal["spacesim-autonomy-attitude/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.attitude.slew-hold", "ssb.attitude.large-angle-slew"]
    source_id: Literal["S02", "S03"]
    backend: Literal["native_team_single_attitude_v1"]
    target_cooperation: Literal["none"]
    controlled_spacecraft: Literal[1]
    spacecraft_count: Literal[1]
    control_architecture: Literal["single_agent"]
    vision_mode: Literal["V0"]
    action_type: Literal["reaction_wheel_motor_torques_xyz"]
    reference_frame: Literal["body_to_J2000_quaternion_wxyz"]
    initial: AttitudeInitialState
    objective: AttitudeObjective
    safety: AttitudeSafety
    observation: AttitudeObservationModel
    seeds: SeedSplits
    duration_s: Positive
    physics_period_ns: Literal[20_000_000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[20_000_000]
    earth_gravity: Literal["point_mass", "j2"]
    solar_gravity: Literal[True]
    drag_enabled: StrictBool
    srp_enabled: StrictBool
    disturbance_sigma_m_s2: Annotated[Number, Field(ge=0, le=0.001)]
    mass_model: Literal["constant_no_propellant_model"]

    @field_validator("controlled_spacecraft", "spacecraft_count", "physics_period_ns",
                     "physics_substeps", "decision_period_ns", mode="before")
    @classmethod
    def integer_counts(cls, value):
        if type(value) is not int:
            raise ValueError("counts and clock periods must be integers")
        return value

    @field_validator("solar_gravity", mode="before")
    @classmethod
    def explicit_solar_gravity(cls, value):
        if value is not True:
            raise ValueError("this backend requires explicit solar gravity")
        return value

    @model_validator(mode="after")
    def feasible_contract(self):
        initial, goal, safety = self.initial, self.objective, self.safety
        requirements = {"S02": "ssb.attitude.slew-hold", "S03": "ssb.attitude.large-angle-slew"}
        if self.requirement_id != requirements[self.source_id]:
            raise ValueError("source and semantic requirement must agree")
        if self.source_id == "S03" and (initial.rotation_min_rad < math.pi/2 or self.duration_s > 90):
            raise ValueError("large-angle slew requires at least 90 degrees and a deadline no longer than 90 seconds")
        if not goal.angle_tolerance_rad < initial.rotation_min_rad <= initial.rotation_max_rad:
            raise ValueError("every initial attitude must require a maneuver")
        if math.sqrt(3) * initial.body_rate_half_width_rad_s >= safety.maximum_body_rate_rad_s:
            raise ValueError("initial body rates must satisfy the hard bound")
        if any(abs(value) >= safety.maximum_wheel_speed_rad_s for value in initial.wheel_speed_rad_s):
            raise ValueError("initial wheel speeds must satisfy the hard bound")
        if not goal.body_rate_tolerance_rad_s < safety.maximum_body_rate_rad_s < self.observation.gyro_range_rad_s:
            raise ValueError("success rate, hard rate bound and gyro range must be ordered")
        if goal.dwell_s > self.duration_s:
            raise ValueError("dwell exceeds episode duration")
        for seconds in (goal.dwell_s, self.duration_s):
            ticks = seconds * 1e9 / self.physics_period_ns
            if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError("dwell and duration must lie on the physics grid")
        return self

    def validate_catalogue(self, catalogue):
        task = catalogue.task(self.requirement_id)
        if (task.source_id != self.source_id or self.vision_mode not in task.requested_vision_modes
                or task.target_cooperation != ("none",)):
            raise ValueError("attitude configuration does not match its catalogue requirement")


class AttitudeObservation(Contract):
    schema_version: Literal["spacesim-autonomy-attitude-observation/1"] = "spacesim-autonomy-attitude-observation/1"
    episode_id: str = Field(min_length=1)
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    orientation_body_to_j2000_wxyz: Quaternion
    angular_velocity_body_rad_s: Vector3
    wheel_speed_rad_s: Vector3
    gyro_valid: StrictBool

    _normalized = field_validator("orientation_body_to_j2000_wxyz")(unit_quaternion)

    @model_validator(mode="after")
    def causal(self):
        if self.delivery_time_ns < self.sample_time_ns:
            raise ValueError("observation cannot arrive before sampling")
        return self


class WheelTorqueAction(Contract):
    schema_version: Literal["spacesim-autonomy-wheel-torque/1"] = "spacesim-autonomy-wheel-torque/1"
    episode_id: str
    observation_sequence: Count
    motor_torque_xyz_nm: Vector3

    def validate_for(self, observation: AttitudeObservation, limit_nm: float):
        value = WheelTorqueAction.model_validate(self)
        if value.episode_id != observation.episode_id or value.observation_sequence != observation.sequence:
            raise ValueError("stale action: episode or observation identity does not match")
        if any(abs(torque) > limit_nm + 1e-12 for torque in value.motor_torque_xyz_nm):
            raise ValueError("requested wheel motor torque exceeds the action contract")


class AttitudeTruthSample(Contract):
    time_ns: Count
    orientation_body_to_j2000_wxyz: Quaternion
    angular_velocity_body_rad_s: Vector3
    wheel_speed_rad_s: Vector3
    applied_motor_torque_xyz_nm: Vector3
    external_state_writes: Count

    _normalized = field_validator("orientation_body_to_j2000_wxyz")(unit_quaternion)


class AttitudeContext(Contract):
    task: AttitudeTask
    chaser_mass_kg: Positive
    inertia_com_body_kg_m2: Tensor3
    wheel_spin_inertia_kg_m2: Vector3
    hardware_torque_limits_nm: Vector3
    hardware_speed_limits_rad_s: Vector3
