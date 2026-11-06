"""Moving line-of-sight pointing, using declared telemetry and SPICE ephemerides."""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_attitude import (
    AttitudeInitialState, AttitudeObservation, AttitudeObservationModel, AttitudeSafety,
    AttitudeTruthSample, Tensor3,
)
from .benchmark_contract import Contract, Number, Positive, SeedSplits, State6, Vector3


def unit_direction(value):
    if not math.isclose(math.sqrt(sum(v*v for v in value)), 1., abs_tol=1e-8, rel_tol=0):
        raise ValueError("pointing direction must have unit norm")
    return value


class PointingWindow(Contract):
    # All samples from this deadline until episode end must satisfy the limits.
    acquisition_deadline_s: Positive
    angle_tolerance_rad: Annotated[Number, Field(gt=0, lt=.1)]
    direction_rate_tolerance_rad_s: Positive
    boresight_body: Literal["+Z"]


class EarthPointingObjective(PointingWindow):
    kind: Literal["earth_center"]


class SunPointingObjective(PointingWindow):
    kind: Literal["sun"]


class InertialPointingObjective(PointingWindow):
    kind: Literal["inertial_direction"]
    direction_j2000_unit: Vector3
    _unit = field_validator("direction_j2000_unit")(unit_direction)


class GroundPointingObjective(PointingWindow):
    kind: Literal["ground_site"]
    # Geocentric direction in the actual IAU_EARTH SPICE frame, on a sphere.
    ground_direction_iau_earth_unit: Vector3
    minimum_elevation_rad: Annotated[Number, Field(gt=0, lt=math.pi/2)]
    _unit = field_validator("ground_direction_iau_earth_unit")(unit_direction)


PointingObjective = Annotated[
    EarthPointingObjective | SunPointingObjective | InertialPointingObjective | GroundPointingObjective,
    Field(discriminator="kind"),
]


class PointingObservationModel(AttitudeObservationModel):
    kind: Literal["noisy_attitude_gyro_gnss_and_known_ephemeris"]
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    ephemeris: Literal["shared_SPICE_IAU_EARTH_and_Sun_no_spacecraft_truth"]


class PointingTask(Contract):
    schema_version: Literal["spacesim-autonomy-pointing/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.attitude.ground-track", "ssb.attitude.reference-pointing"]
    source_id: Literal["S05", "S06"]
    backend: Literal["native_team_single_pointing_v1"]
    target_cooperation: Literal["none"]
    controlled_spacecraft: Literal[1]
    spacecraft_count: Literal[1]
    control_architecture: Literal["single_agent"]
    vision_mode: Literal["V0"]
    action_type: Literal["reaction_wheel_motor_torques_xyz"]
    reference_frame: Literal["J2000_spacecraft_COM_IAU_EARTH_ground_body_plus_Z_boresight"]
    initial: AttitudeInitialState
    objective: PointingObjective
    safety: AttitudeSafety
    observation: PointingObservationModel
    seeds: SeedSplits
    duration_s: Positive
    physics_period_ns: Literal[20_000_000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[20_000_000]
    earth_gravity: Literal["point_mass", "j2"]
    solar_gravity: Literal[True]
    drag_enabled: StrictBool
    srp_enabled: StrictBool
    disturbance_sigma_m_s2: Annotated[Number, Field(ge=0, le=.001)]
    mass_model: Literal["constant_no_propellant_model"]
    earth_surface_model: Literal["geocentric_sphere_6378136.6_m_no_terrain"]
    ephemeris_epoch_utc: Literal["2026 SEPTEMBER 02 00:00:00.000"]

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
    def feasible(self):
        initial, goal, safety = self.initial, self.objective, self.safety
        ground = isinstance(goal, GroundPointingObjective)
        expected = "ssb.attitude.ground-track" if ground else "ssb.attitude.reference-pointing"
        if self.source_id != ("S05" if ground else "S06") or self.requirement_id != expected:
            raise ValueError("pointing target and source requirement must agree")
        if not 5*goal.angle_tolerance_rad < initial.rotation_min_rad <= initial.rotation_max_rad < math.pi/2:
            raise ValueError("initial boresight tilt must require a distinct acquisition maneuver")
        if math.sqrt(3)*initial.body_rate_half_width_rad_s >= safety.maximum_body_rate_rad_s:
            raise ValueError("initial rates exceed the hard bound")
        if any(abs(v) >= safety.maximum_wheel_speed_rad_s for v in initial.wheel_speed_rad_s):
            raise ValueError("initial wheel speeds exceed the hard bound")
        if not goal.direction_rate_tolerance_rad_s < safety.maximum_body_rate_rad_s < self.observation.gyro_range_rad_s:
            raise ValueError("tracking tolerance, hard rate and gyro range must be ordered")
        if goal.acquisition_deadline_s >= self.duration_s:
            raise ValueError("a nonempty required tracking window must follow acquisition")
        for seconds in (goal.acquisition_deadline_s, self.duration_s):
            ticks = seconds*1e9/self.physics_period_ns
            if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError("tracking window must lie on the physical evidence grid")
        return self

    def validate_catalogue(self, catalogue):
        entry = catalogue.task(self.requirement_id)
        if entry.source_id != self.source_id or self.vision_mode not in entry.requested_vision_modes or entry.target_cooperation != ("none",):
            raise ValueError("pointing configuration does not match its source requirement")


class PointingEphemeris(Contract):
    earth_j2000_to_fixed: Tensor3
    earth_j2000_to_fixed_rate_s: Tensor3
    sun_geocentric_j2000: State6

    @model_validator(mode="after")
    def valid_frame(self):
        c, derivative = self.earth_j2000_to_fixed, self.earth_j2000_to_fixed_rate_s
        for i in range(3):
            for j in range(3):
                if abs(sum(c[i][k]*c[j][k] for k in range(3))-(1. if i == j else 0.)) > 1e-7:
                    raise ValueError("SPICE Earth rotation must be orthonormal")
                if abs(sum(derivative[i][k]*c[j][k]+c[i][k]*derivative[j][k] for k in range(3))) > 1e-8:
                    raise ValueError("Earth rotation derivative must preserve orthonormality")
        determinant = sum(c[0][i]*(c[1][(i+1)%3]*c[2][(i+2)%3]-c[1][(i+2)%3]*c[2][(i+1)%3]) for i in range(3))
        if abs(determinant-1.) > 1e-7 or max(abs(v) for row in derivative for v in row) > .001:
            raise ValueError("invalid Earth frame handedness or spin rate")
        if not 1e11 < math.sqrt(sum(v*v for v in self.sun_geocentric_j2000[:3])) < 2e11:
            raise ValueError("Sun ephemeris must be geocentric SI state")
        return self


class PointingObservation(AttitudeObservation):
    schema_version: Literal["spacesim-autonomy-pointing-observation/1"] = "spacesim-autonomy-pointing-observation/1"
    chaser_gnss_j2000: State6
    ephemeris: PointingEphemeris


class PointingTruthSample(AttitudeTruthSample):
    chaser_com_j2000: State6
    ephemeris: PointingEphemeris


class PointingContext(Contract):
    task: PointingTask
    chaser_mass_kg: Positive
    inertia_com_body_kg_m2: Tensor3
    wheel_spin_inertia_kg_m2: Vector3
    hardware_torque_limits_nm: Vector3
    hardware_speed_limits_rad_s: Vector3
