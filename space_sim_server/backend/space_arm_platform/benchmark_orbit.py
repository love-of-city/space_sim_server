"""Single-spacecraft orbital maneuver contracts, without simulator imports."""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import Contract, Count, Number, Nonnegative, Positive, SeedSplits, State6, Vector3
from .benchmark_attitude import Quaternion, unit_quaternion

EARTH_MU_M3_S2 = 3.986004418e14
EARTH_RADIUS_M = 6378136.6


class OrbitInitialState(Contract):
    altitude_m: Annotated[Number, Field(ge=200_000, le=2_000_000)]
    altitude_half_width_m: Nonnegative
    inclination_deg: Annotated[Number, Field(ge=0, le=180)]
    true_anomaly_deg: Number
    anomaly_half_width_deg: Annotated[Number, Field(ge=0, le=180)]
    orbit_shape: Literal["circular_two_body_osculating"]


class ImpulseObjective(Contract):
    kind: Literal["finite_burn_impulse"]
    delta_velocity_j2000_m_s: Vector3
    vector_tolerance_m_s: Positive
    earliest_burn_s: Nonnegative
    latest_burn_s: Positive
    coast_force_tolerance_n: Positive
    dwell_s: Positive


class CircularOrbitObjective(Contract):
    kind: Literal["circular_orbit"]
    altitude_m: Annotated[Number, Field(ge=200_000, le=2_000_000)]
    semimajor_axis_tolerance_m: Positive
    maximum_eccentricity: Annotated[Number, Field(gt=0, lt=1)]
    plane_tolerance_rad: Annotated[Number, Field(gt=0, lt=math.pi)]
    coast_force_tolerance_n: Positive
    dwell_s: Positive


class OrbitMaintenanceObjective(Contract):
    kind: Literal["continuous_orbit_maintenance"]
    altitude_m: Annotated[Number, Field(ge=200_000, le=2_000_000)]
    semimajor_axis_tolerance_m: Positive
    maximum_eccentricity: Annotated[Number, Field(gt=0, lt=1)]
    plane_tolerance_rad: Annotated[Number, Field(gt=0, lt=math.pi)]
    acquisition_deadline_s: Nonnegative
    dwell_s: Positive


class OrbitSafety(Contract):
    minimum_altitude_m: Annotated[Number, Field(ge=200_000)]
    maximum_altitude_m: Positive
    maximum_eccentricity: Annotated[Number, Field(gt=0, lt=1)]
    maximum_force_n: Positive
    maximum_delta_v_m_s: Positive


class SingleGNSSModel(Contract):
    kind: Literal["single_gnss_solution_and_orbit_filter"]
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    delay_s: Literal[0.0]
    reference_point: Literal["assembled_center_of_mass"]
    frame: Literal["J2000"]

    @field_validator("delay_s", mode="before")
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (float, int):
            raise ValueError("delay must be numeric")
        return value


class OrbitTask(Contract):
    schema_version: Literal["spacesim-autonomy-orbit/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.orbit.maneuver", "ssb.orbit.raise-lower", "ssb.orbit.stationkeeping"]
    source_id: Literal["S09", "S11", "S13"]
    backend: Literal["native_team_single_orbit_v1"]
    target_cooperation: Literal["none"]
    controlled_spacecraft: Literal[1]
    spacecraft_count: Literal[1]
    control_architecture: Literal["single_agent"]
    vision_mode: Literal["V0"]
    action_type: Literal["ideal_force_j2000_at_assembly_com"]
    attitude_control: Literal["none_passive_attitude"]
    reference_frame: Literal["Earth_centered_J2000"]
    element_convention: Literal["instantaneous_two_body_osculating_fixed_earth_mu"]
    initial: OrbitInitialState
    objective: Annotated[ImpulseObjective | CircularOrbitObjective | OrbitMaintenanceObjective, Field(discriminator="kind")]
    safety: OrbitSafety
    observation: SingleGNSSModel
    seeds: SeedSplits
    duration_s: Positive
    physics_period_ns: Literal[20_000_000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[500_000_000]
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
    def explicit_solar(cls, value):
        if value is not True:
            raise ValueError("this backend requires explicit solar gravity")
        return value

    @model_validator(mode="after")
    def feasible_contract(self):
        initial, goal, limits = self.initial, self.objective, self.safety
        requirements = {"S09": "ssb.orbit.maneuver", "S11": "ssb.orbit.raise-lower", "S13": "ssb.orbit.stationkeeping"}
        if self.requirement_id != requirements[self.source_id]:
            raise ValueError("source and semantic requirement must agree")
        if not limits.minimum_altitude_m < initial.altitude_m-initial.altitude_half_width_m <= initial.altitude_m+initial.altitude_half_width_m < limits.maximum_altitude_m:
            raise ValueError("all initial circular orbits must lie inside the safety shell")
        if not isinstance(goal, OrbitMaintenanceObjective) and not goal.coast_force_tolerance_n < limits.maximum_force_n:
            raise ValueError("coast thrust must be stricter than the actuator bound")
        objective_types = {"S09": ImpulseObjective, "S11": CircularOrbitObjective, "S13": OrbitMaintenanceObjective}
        if not isinstance(goal, objective_types[self.source_id]):
            raise ValueError("source requires its declared impulse, transfer or maintenance objective")
        if isinstance(goal, ImpulseObjective):
            magnitude = math.dist(goal.delta_velocity_j2000_m_s, (0, 0, 0))
            if not goal.vector_tolerance_m_s < magnitude < limits.maximum_delta_v_m_s:
                raise ValueError("commanded impulse must be nontrivial and below the delta-v budget")
            if not goal.earliest_burn_s < goal.latest_burn_s <= self.duration_s-goal.dwell_s:
                raise ValueError("burn window must leave enough time for coasting confirmation")
        else:
            if isinstance(goal, CircularOrbitObjective) and abs(goal.altitude_m-initial.altitude_m) <= initial.altitude_half_width_m+goal.semimajor_axis_tolerance_m:
                raise ValueError("all initial orbits must require a change of semimajor axis")
            low, high = EARTH_RADIUS_M+goal.altitude_m-goal.semimajor_axis_tolerance_m, EARTH_RADIUS_M+goal.altitude_m+goal.semimajor_axis_tolerance_m
            if not (limits.minimum_altitude_m < low*(1-goal.maximum_eccentricity)-EARTH_RADIUS_M
                    and high*(1+goal.maximum_eccentricity)-EARTH_RADIUS_M < limits.maximum_altitude_m):
                raise ValueError("the complete target orbit must lie inside the safety shell")
            if goal.maximum_eccentricity >= limits.maximum_eccentricity:
                raise ValueError("target eccentricity must be stricter than the hard bound")
            if isinstance(goal, OrbitMaintenanceObjective):
                if abs(goal.altitude_m-initial.altitude_m)+initial.altitude_half_width_m > goal.semimajor_axis_tolerance_m:
                    raise ValueError("maintenance starts inside the declared orbital band")
                if self.earth_gravity != "j2" or goal.dwell_s < 600:
                    raise ValueError("maintenance requires J2 and at least 600 seconds of continuous coverage")
                if not math.isclose(goal.acquisition_deadline_s+goal.dwell_s, self.duration_s, rel_tol=0, abs_tol=1e-8):
                    raise ValueError("maintenance window must extend from acquisition deadline to episode end")
        if goal.dwell_s > self.duration_s:
            raise ValueError("dwell exceeds episode duration")
        grids = [(self.duration_s, self.decision_period_ns), (goal.dwell_s, self.physics_period_ns)]
        if isinstance(goal, ImpulseObjective):
            grids += [(goal.earliest_burn_s, self.decision_period_ns), (goal.latest_burn_s, self.decision_period_ns)]
        if isinstance(goal, OrbitMaintenanceObjective):
            grids += [(goal.acquisition_deadline_s, self.decision_period_ns)]
        for seconds, grid in grids:
            if not math.isclose(seconds*1e9/grid, round(seconds*1e9/grid), rel_tol=0, abs_tol=1e-8):
                raise ValueError("times must lie on their declared grids")
        return self

    def validate_catalogue(self, catalogue):
        requirement = catalogue.task(self.requirement_id)
        if (requirement.source_id != self.source_id or self.vision_mode not in requirement.requested_vision_modes
                or requirement.target_cooperation != ("none",)):
            raise ValueError("orbital configuration does not match its catalogue requirement")


class OrbitObservation(Contract):
    schema_version: Literal["spacesim-autonomy-orbit-observation/1"] = "spacesim-autonomy-orbit-observation/1"
    episode_id: str = Field(min_length=1)
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    chaser_estimate_j2000: State6

    @model_validator(mode="after")
    def causal(self):
        if self.delivery_time_ns < self.sample_time_ns:
            raise ValueError("observation cannot arrive before sampling")
        return self


class OrbitTruthSample(Contract):
    time_ns: Count
    chaser_com_j2000: State6
    orientation_body_to_j2000_wxyz: Quaternion
    applied_motor_force_body_n: Vector3
    external_state_writes: Count

    _normalized = field_validator("orientation_body_to_j2000_wxyz")(unit_quaternion)


class OrbitContext(Contract):
    task: OrbitTask
    chaser_mass_kg: Positive
