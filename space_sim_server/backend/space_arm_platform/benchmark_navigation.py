"""GNSS navigation evaluation: sensor solutions in, method estimates out."""
from __future__ import annotations

import math
from typing import Annotated, Literal
from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import Contract, Count, Number, Positive, SeedSplits, State6, WaypointInitialState
from .benchmark_orbit import OrbitInitialState

EstimateFrame = Literal["Earth_centered_J2000", "chaser_minus_target_J2000"]
EstimatePosition = Annotated[Number, Field(ge=-1e12, le=1e12)]
EstimateVelocity = Annotated[Number, Field(ge=-1e7, le=1e7)]
EstimateState = tuple[EstimatePosition, EstimatePosition, EstimatePosition, EstimateVelocity, EstimateVelocity, EstimateVelocity]


class NavigationAccuracy(Contract):
    kind: Literal["navigation_accuracy"]
    position_rmse_limit_m: Positive
    velocity_rmse_limit_m_s: Positive
    maximum_position_error_m: Positive
    maximum_velocity_error_m_s: Positive

    @model_validator(mode="after")
    def ordered_limits(self):
        if (self.position_rmse_limit_m > self.maximum_position_error_m
                or self.velocity_rmse_limit_m_s > self.maximum_velocity_error_m_s):
            raise ValueError("RMSE limits must not exceed peak error limits")
        return self


class NavigationGNSSModel(Contract):
    kind: Literal["independent_gnss_solutions_without_environment_filter"]
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    delay_s: Literal[0.0]
    target_link: Literal["none", "cooperative_target_gnss_solution"]
    reference_point: Literal["assembled_center_of_mass"]
    frame: Literal["J2000"]

    @field_validator("delay_s", mode="before")
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (float, int):
            raise ValueError("delay must be numeric")
        return value


class AbsoluteNavigationSafety(Contract):
    kind: Literal["single_spacecraft_orbit"]
    minimum_altitude_m: Annotated[Number, Field(ge=200_000)]
    maximum_altitude_m: Positive
    maximum_eccentricity: Annotated[Number, Field(gt=0, lt=1)]
    # These motors exist in the physical model, but are held at zero. An
    # estimator is never given actuation authority through this contract.
    maximum_force_n: Positive
    maximum_uncommanded_force_n: Annotated[Number, Field(ge=0, le=1e-6)]


class RelativeNavigationSafety(Contract):
    kind: Literal["two_spacecraft_relative_motion"]
    minimum_range_m: Annotated[Number, Field(ge=100)]
    maximum_range_m: Positive
    maximum_relative_speed_m_s: Positive
    maximum_force_n: Positive
    maximum_uncommanded_force_n: Annotated[Number, Field(ge=0, le=1e-6)]


class NavigationTask(Contract):
    schema_version: Literal["spacesim-autonomy-navigation-evaluation/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.navigation.absolute", "ssb.relative-navigation.cooperative"]
    source_id: Literal["S14", "D01"]
    backend: Literal["native_team_gnss_estimation_v1"]
    target_cooperation: Literal["none", "C"]
    controlled_spacecraft: Literal[0]
    spacecraft_count: Literal[1, 2]
    control_architecture: Literal["single_estimator"]
    vision_mode: Literal["V0"]
    action_type: Literal["navigation_state_estimate_only"]
    attitude_control: Literal["none_passive_attitude"]
    reference_frame: EstimateFrame
    initial: OrbitInitialState | WaypointInitialState
    objective: NavigationAccuracy
    safety: Annotated[AbsoluteNavigationSafety | RelativeNavigationSafety, Field(discriminator="kind")]
    observation: NavigationGNSSModel
    seeds: SeedSplits
    duration_s: Annotated[Number, Field(ge=1)]
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
    def semantic_contract(self):
        single = self.source_id == "S14"
        expected = ("ssb.navigation.absolute", "none", 1, "Earth_centered_J2000", "none") if single else (
            "ssb.relative-navigation.cooperative", "C", 2, "chaser_minus_target_J2000", "cooperative_target_gnss_solution")
        actual = (self.requirement_id, self.target_cooperation, self.spacecraft_count,
                  self.reference_frame, self.observation.target_link)
        if actual != expected:
            raise ValueError("navigation source, actors, cooperation, estimate frame and link must agree")
        validate_navigation_physics(self)
        return self

    @property
    def expected_estimates(self):
        # Estimates for t=0,...,duration-dt; the final observation is terminal.
        return round(self.duration_s*1e9/self.decision_period_ns)

    def validate_catalogue(self, catalogue):
        requirement = catalogue.task(self.requirement_id)
        if (requirement.source_id != self.source_id or self.vision_mode not in requirement.requested_vision_modes
                or self.target_cooperation not in requirement.target_cooperation):
            raise ValueError("navigation configuration does not match its catalogue requirement")


class GNSSMeasurements(Contract):
    schema_version: Literal["spacesim-autonomy-gnss-measurements/1"] = "spacesim-autonomy-gnss-measurements/1"
    episode_id: str = Field(min_length=1)
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    chaser_measurement_j2000: State6
    target_measurement_j2000: State6 | None

    @model_validator(mode="after")
    def causal(self):
        if self.delivery_time_ns < self.sample_time_ns:
            raise ValueError("observation cannot arrive before sampling")
        return self


class NavigationEstimateAction(Contract):
    schema_version: Literal["spacesim-autonomy-navigation-estimate/1"] = "spacesim-autonomy-navigation-estimate/1"
    episode_id: str
    observation_sequence: Count
    estimate_time_ns: Count
    reference_frame: EstimateFrame
    estimated_state: EstimateState

    def validate_for(self, observation, task):
        value = NavigationEstimateAction.model_validate(self)
        if (value.episode_id != observation.episode_id or value.observation_sequence != observation.sequence
                or value.estimate_time_ns != observation.sample_time_ns):
            raise ValueError("stale navigation estimate: episode, sequence or sample time differs")
        if value.reference_frame != task.reference_frame:
            raise ValueError("navigation estimate has the wrong reference frame")


class NavigationEstimateEvidence(Contract):
    record_type: Literal["navigation_estimate"] = "navigation_estimate"
    sequence: Count
    estimate_time_ns: Count
    reference_frame: EstimateFrame
    estimated_state: EstimateState


class NavigationContext(Contract):
    task: NavigationTask
    chaser_mass_kg: Positive


def validate_navigation_physics(task):
    """Shared physical envelope, without assigning another task's semantics."""
    initial, limits = task.initial, task.safety
    if task.spacecraft_count == 1:
        if not isinstance(initial, OrbitInitialState) or not isinstance(limits, AbsoluteNavigationSafety):
            raise ValueError("single navigation requires an absolute initial orbit and safety envelope")
        if not limits.minimum_altitude_m < initial.altitude_m-initial.altitude_half_width_m <= initial.altitude_m+initial.altitude_half_width_m < limits.maximum_altitude_m:
            raise ValueError("initial orbits must lie inside the altitude safety shell")
    else:
        if not isinstance(initial, WaypointInitialState) or not isinstance(limits, RelativeNavigationSafety):
            raise ValueError("relative navigation requires two spacecraft initial states and relative safety")
        radius = math.dist(initial.relative_position_tnr_m, (0, 0, 0))
        spread = math.sqrt(3)*initial.position_half_width_m
        if not limits.minimum_range_m < radius-spread <= radius+spread < limits.maximum_range_m:
            raise ValueError("initial relative states must lie inside the safety shell")
        if math.dist(initial.relative_velocity_tnr_m_s, (0, 0, 0))+math.sqrt(3)*initial.velocity_half_width_m_s >= limits.maximum_relative_speed_m_s:
            raise ValueError("initial relative speed must satisfy its hard bound")
    if limits.maximum_uncommanded_force_n >= limits.maximum_force_n:
        raise ValueError("navigation cannot grant motor authority")
    ticks = task.duration_s*1e9/task.decision_period_ns
    if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
        raise ValueError("navigation duration must lie on the decision grid")
