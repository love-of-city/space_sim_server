"""Serializable method/environment contracts; no native simulator imports.

The first executable specification is a cooperative GNSS waypoint task. Other
catalogue requirements are not implicitly registered by sharing these types.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Annotated, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, StrictBool, field_validator, model_validator

from .benchmark_catalog import AutonomyCatalog, load_catalog

Number = Annotated[FiniteFloat, Field(strict=True)]
Positive = Annotated[Number, Field(gt=0)]
Nonnegative = Annotated[Number, Field(ge=0)]
Count = Annotated[int, Field(strict=True, ge=0)]
Seed = Annotated[int, Field(strict=True, ge=0, lt=2**32)]
Vector3 = tuple[Number, Number, Number]
State6 = tuple[Number, Number, Number, Number, Number, Number]
SPEC_DIRECTORY = Path(__file__).resolve().parents[2] / "config/benchmarks/tasks"


class Contract(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, revalidate_instances="always")

    def fingerprint(self) -> str:
        payload = json.dumps(self.model_dump(mode="json"), sort_keys=True,
                             separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()


class SeedSplits(Contract):
    train: tuple[Seed, ...] = Field(min_length=1)
    validation: tuple[Seed, ...] = Field(min_length=1)
    test: tuple[Seed, ...] = Field(min_length=10)

    @model_validator(mode="after")
    def disjoint(self):
        values = self.train + self.validation + self.test
        if len(set(values)) != len(values):
            raise ValueError("seeds must be unique within and across frozen splits")
        return self

    def split_for(self, seed: int) -> str:
        if type(seed) is not int or not 0 <= seed < 2**32:
            raise ValueError("seed must be an integer in [0, 2^32)")
        for name in ("train", "validation", "test"):
            if seed in getattr(self, name):
                return name
        raise ValueError("seed is outside the frozen task splits")


class WaypointInitialState(Contract):
    target_altitude_m: Annotated[Number, Field(ge=200_000, le=2_000_000)]
    inclination_deg: Annotated[Number, Field(ge=0, le=180)]
    true_anomaly_deg: Number
    relative_position_tnr_m: Vector3
    relative_velocity_tnr_m_s: Vector3
    # Independent bounded uniform perturbations; cannot initialize inside the
    # hard safety sphere through an unbounded Gaussian tail.
    position_half_width_m: Nonnegative
    velocity_half_width_m_s: Nonnegative


class WaypointObjective(Contract):
    position_tnr_m: Vector3
    position_tolerance_m: Positive
    speed_tolerance_m_s: Positive
    dwell_s: Positive


class FlightSafety(Contract):
    minimum_range_m: Annotated[Number, Field(ge=100)]
    maximum_range_m: Positive
    maximum_relative_speed_m_s: Positive
    maximum_force_n: Positive


class GNSSObservationModel(Contract):
    kind: Literal["independent_gnss_solutions_and_orbit_filters"]
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    delay_s: Literal[0.0]
    target_link: Literal["cooperative_target_filtered_state"]
    reference_point: Literal["assembled_center_of_mass"]
    frame: Literal["J2000"]

    @field_validator("delay_s", mode="before")
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (float, int):
            raise ValueError("delay must be numeric, not a boolean or string")
        return value


class WaypointTask(Contract):
    schema_version: Literal["spacesim-autonomy-waypoint/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.approach.waypoint-dwell"]
    source_id: Literal["D32"]
    backend: Literal["native_team_stowed_flight_v1"]
    target_cooperation: Literal["C"]
    controlled_spacecraft: Literal[1]
    spacecraft_count: Literal[2]
    control_architecture: Literal["single_agent"]
    vision_mode: Literal["V0"]
    action_type: Literal["ideal_force_j2000_at_assembly_com"]
    attitude_control: Literal["none_passive_attitude"]
    reference_frame: Literal["target_TNR_tangent_normal_radial"]
    initial: WaypointInitialState
    objective: WaypointObjective
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
        low, high = self.safety.minimum_range_m, self.safety.maximum_range_m
        radius = math.dist(self.objective.position_tnr_m, (0, 0, 0))
        tolerance = self.objective.position_tolerance_m
        initial_radius = math.dist(self.initial.relative_position_tnr_m, (0, 0, 0))
        spread = math.sqrt(3) * self.initial.position_half_width_m
        if not (low < radius - tolerance and radius + tolerance < high):
            raise ValueError("the complete success region must lie within the safety shell")
        if not (low < initial_radius - spread and initial_radius + spread < high):
            raise ValueError("every bounded initial position must lie within the safety shell")
        initial_speed = math.dist(self.initial.relative_velocity_tnr_m_s, (0, 0, 0))
        initial_speed += math.sqrt(3) * self.initial.velocity_half_width_m_s
        if initial_speed >= self.safety.maximum_relative_speed_m_s:
            raise ValueError("initial velocities must satisfy the hard speed bound")
        if self.objective.speed_tolerance_m_s >= self.safety.maximum_relative_speed_m_s:
            raise ValueError("success speed must be stricter than the hard safety bound")
        if self.objective.dwell_s > self.duration_s:
            raise ValueError("dwell exceeds episode duration")
        for seconds, grid in ((self.duration_s, self.decision_period_ns),
                              (self.objective.dwell_s, self.physics_period_ns)):
            ticks = seconds * 1e9 / grid
            if not math.isclose(ticks, round(ticks), abs_tol=1e-8, rel_tol=0):
                raise ValueError("duration and dwell must lie on their declared time grids")
        return self

    def validate_catalogue(self, catalogue: AutonomyCatalog) -> None:
        task = catalogue.task(self.requirement_id)
        if (task.source_id != self.source_id or self.vision_mode not in task.requested_vision_modes
                or self.target_cooperation not in task.target_cooperation):
            raise ValueError("executable configuration does not match its catalogue requirement")


def load_task(path: Path, catalogue: AutonomyCatalog | None = None):
    from .benchmark_attitude import AttitudeTask
    from .benchmark_orbit import OrbitTask
    from .benchmark_navigation import NavigationTask
    from .benchmark_relative_flight import RelativeFlightTask
    from .benchmark_flyaround import FlyAroundTask
    document = json.loads(path.read_text(encoding="utf-8"))
    schemas = {"spacesim-autonomy-waypoint/1": WaypointTask, "spacesim-autonomy-attitude/1": AttitudeTask,
               "spacesim-autonomy-orbit/1": OrbitTask, "spacesim-autonomy-navigation-evaluation/1": NavigationTask,
               "spacesim-autonomy-relative-flight/1": RelativeFlightTask,
               "spacesim-autonomy-flyaround/1": FlyAroundTask}
    if not isinstance(document, dict) or document.get("schema_version") not in schemas:
        raise ValueError("unknown executable benchmark task schema")
    task = schemas[document["schema_version"]].model_validate(document)
    task.validate_catalogue(catalogue or load_catalog())
    return task


class NavigationObservation(Contract):
    """Method-visible whitelist. Contains no scene handle or physical truth."""
    schema_version: Literal["spacesim-autonomy-navigation/1"] = "spacesim-autonomy-navigation/1"
    episode_id: str = Field(min_length=1)
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    target_estimate_j2000: State6
    chaser_estimate_j2000: State6

    @model_validator(mode="after")
    def causal(self):
        if self.delivery_time_ns < self.sample_time_ns:
            raise ValueError("an observation cannot arrive before its sample time")
        return self


class ForceAction(Contract):
    schema_version: Literal["spacesim-autonomy-force/1"] = "spacesim-autonomy-force/1"
    episode_id: str
    observation_sequence: Count
    force_j2000_n: Vector3

    def validate_for(self, observation: NavigationObservation, limit_n: float) -> None:
        # Pydantic model_copy(update=...) intentionally bypasses construction
        # validation. A method returning such an object must not inject NaN or
        # malformed vectors into the native dynamics process.
        validated = ForceAction.model_validate(self)
        if (validated.episode_id != observation.episode_id
                or validated.observation_sequence != observation.sequence):
            raise ValueError("stale action: episode or observation identity does not match")
        if math.dist(validated.force_j2000_n, (0, 0, 0)) > limit_n + 1e-9:
            raise ValueError("requested force exceeds the action contract")


class FlightTruthSample(Contract):
    """Private grading evidence. COM states and native motor force readbacks."""
    time_ns: Count
    target_com_j2000: State6
    chaser_com_j2000: State6
    applied_motor_force_body_n: Vector3
    external_state_writes: Count


class MethodContext(Contract):
    task: WaypointTask
    chaser_mass_kg: Positive


class FlightMethod(Protocol):
    def reset(self, context: MethodContext) -> None: ...
    def act(self, observation: NavigationObservation) -> ForceAction: ...
    def close(self) -> None: ...


class FlightEnvironment(Protocol):
    def reset(self, seed: int) -> NavigationObservation: ...
    def step(self, action: ForceAction) -> NavigationObservation: ...
    def close(self) -> None: ...
