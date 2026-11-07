"""A complete signed revolution through a physical T-R annulus, not waypoint arrival."""
import math
from typing import Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import (
    Contract, FlightSafety, GNSSObservationModel, Nonnegative, Positive, SeedSplits,
    WaypointInitialState,
)


class FlyAroundObjective(Contract):
    kind: Literal["complete_signed_revolution"]
    plane: Literal["target_TR"]
    radius_m: Positive
    radial_tolerance_m: Positive
    plane_tolerance_m: Positive
    direction: Literal["positive_T_towards_R"]
    revolutions: Literal[1]
    maximum_reverse_travel_rad: Nonnegative
    closure_tolerance_m: Positive

    @field_validator("revolutions", mode="before")
    @classmethod
    def integer_revolutions(cls, value):
        if type(value) is not int:
            raise ValueError("revolution count must be an integer")
        return value


class FlyAroundTask(Contract):
    schema_version: Literal["spacesim-autonomy-flyaround/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.relative-motion.flyaround"]
    source_id: Literal["D33"]
    backend: Literal["native_team_flyaround_v1"]
    target_cooperation: Literal["C"]
    controlled_spacecraft: Literal[1]
    spacecraft_count: Literal[2]
    control_architecture: Literal["single_agent"]
    vision_mode: Literal["V0"]
    action_type: Literal["ideal_force_j2000_at_assembly_com"]
    attitude_control: Literal["none_passive_attitude"]
    reference_frame: Literal["target_TNR_tangent_normal_radial"]
    initial: WaypointInitialState
    objective: FlyAroundObjective
    safety: FlightSafety
    observation: GNSSObservationModel
    seeds: SeedSplits
    duration_s: Positive
    physics_period_ns: Literal[20_000_000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[500_000_000]
    earth_gravity: Literal["point_mass", "j2"]
    solar_gravity: Literal[True]
    drag_enabled: StrictBool
    srp_enabled: StrictBool
    disturbance_sigma_m_s2: Nonnegative = Field(le=.001)
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
    def feasible_annulus(self):
        goal, initial, safety = self.objective, self.initial, self.safety
        lower, upper = goal.radius_m-goal.radial_tolerance_m, goal.radius_m+goal.radial_tolerance_m
        if not (safety.minimum_range_m < lower
                and math.hypot(upper, goal.plane_tolerance_m) < safety.maximum_range_m):
            raise ValueError("the entire flyaround annulus must be inside the safety shell")
        radial = math.hypot(initial.relative_position_tnr_m[0], initial.relative_position_tnr_m[2])
        if (abs(radial-goal.radius_m)+math.sqrt(2)*initial.position_half_width_m >= goal.radial_tolerance_m
                or abs(initial.relative_position_tnr_m[1])+initial.position_half_width_m >= goal.plane_tolerance_m):
            raise ValueError("the complete initial position distribution must be inside the flyaround annulus")
        speed = math.dist(initial.relative_velocity_tnr_m_s, (0, 0, 0))+math.sqrt(3)*initial.velocity_half_width_m_s
        if speed >= safety.maximum_relative_speed_m_s:
            raise ValueError("all initial velocities must be safe")
        if self.duration_s*safety.maximum_relative_speed_m_s < 2*math.pi*lower:
            raise ValueError("duration is too short for one safe revolution")
        if goal.closure_tolerance_m >= goal.radius_m or goal.maximum_reverse_travel_rad >= math.pi:
            raise ValueError("closure and reversal tolerances must constrain a complete directed lap")
        if safety.maximum_relative_speed_m_s*self.physics_period_ns*1e-9/lower >= .1:
            raise ValueError("physical cadence must resolve angular travel without aliasing")
        ticks = self.duration_s*1e9/self.decision_period_ns
        if not math.isclose(ticks, round(ticks), abs_tol=1e-8, rel_tol=0):
            raise ValueError("duration must lie on the decision time grid")
        return self

    def validate_catalogue(self, catalogue):
        requirement = catalogue.task(self.requirement_id)
        if (requirement.source_id != self.source_id or self.vision_mode not in requirement.requested_vision_modes
                or self.target_cooperation not in requirement.target_cooperation):
            raise ValueError("flyaround configuration does not match its catalogue requirement")


class FlyAroundContext(Contract):
    task: FlyAroundTask
    chaser_mass_kg: Positive
