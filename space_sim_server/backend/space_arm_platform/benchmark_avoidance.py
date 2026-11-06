"""Single spacecraft avoiding a passive debris object using GNSS and a catalog."""
import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import Contract, Count, Number, Positive, SeedSplits, State6, Vector3
from .benchmark_orbit import OrbitInitialState, OrbitObservation, OrbitSafety, OrbitTruthSample


class DebrisEncounter(Contract):
    object_type: Literal['synthetic_passive_spherical_debris']
    radius_m: Annotated[Number, Field(ge=.01, le=1)]
    mass_kg: Positive
    relative_position_tnr_m: Vector3
    relative_velocity_tnr_m_s: Vector3
    position_half_width_m: Annotated[Number, Field(ge=0)]
    velocity_half_width_m_s: Annotated[Number, Field(ge=0)]
    frame: Literal['spacecraft_TNR_rotating']


class AvoidanceObjective(Contract):
    kind: Literal['pass_single_conjunction_and_coast']
    screening_horizon_s: Positive
    initial_predicted_miss_max_m: Positive
    encounter_window_start_s: Positive
    encounter_window_end_s: Positive
    final_range_min_m: Positive
    outward_speed_min_m_s: Positive
    outward_dwell_s: Positive
    final_coast_s: Positive
    coast_force_tolerance_n: Positive
    minimum_maneuver_delta_v_m_s: Positive


class AvoidanceSafety(OrbitSafety):
    minimum_range_m: Annotated[Number, Field(ge=100)]
    maximum_range_m: Positive
    maximum_relative_speed_m_s: Positive


class CatalogObservationModel(Contract):
    kind: Literal['self_gnss_filter_and_external_noisy_debris_catalog']
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    catalog_position_sigma_m: Positive
    catalog_velocity_sigma_m_s: Positive
    catalog_period_ns: Literal[500_000_000]
    delay_s: Literal[0.0]
    target_link: Literal['none_external_catalog_only']
    reference_point: Literal['assembled_center_of_mass_and_debris_center']
    frame: Literal['J2000']

    @field_validator('catalog_period_ns', mode='before')
    @classmethod
    def integer_period(cls, value):
        if type(value) is not int:
            raise ValueError('catalog cadence must be an integer')
        return value

    @field_validator('delay_s', mode='before')
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (int, float):
            raise ValueError('delay must be numeric')
        return value


class AvoidanceTask(Contract):
    schema_version: Literal['spacesim-autonomy-debris-avoidance/1']
    configuration_id: str = Field(pattern=r'^[a-z0-9]+(?:-[a-z0-9]+)*$')
    requirement_id: Literal['ssb.safety.single-avoidance']
    source_id: Literal['S16']
    backend: Literal['native_team_single_debris_avoidance_v1']
    target_cooperation: Literal['none']
    controlled_spacecraft: Literal[1]
    spacecraft_count: Literal[1]
    debris_count: Literal[1]
    control_architecture: Literal['single_agent']
    vision_mode: Literal['V0']
    action_type: Literal['ideal_force_j2000_at_assembly_com']
    attitude_control: Literal['none_passive_attitude']
    reference_frame: Literal['Earth_centered_J2000']
    initial: OrbitInitialState
    encounter: DebrisEncounter
    objective: AvoidanceObjective
    safety: AvoidanceSafety
    observation: CatalogObservationModel
    seeds: SeedSplits
    duration_s: Positive
    physics_period_ns: Literal[20_000_000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[500_000_000]
    earth_gravity: Literal['j2']
    solar_gravity: Literal[True]
    drag_enabled: StrictBool
    srp_enabled: StrictBool
    disturbance_sigma_m_s2: Annotated[Number, Field(ge=0, le=.001)]
    mass_model: Literal['constant_no_propellant_model']

    @field_validator('controlled_spacecraft', 'spacecraft_count', 'debris_count',
                     'physics_period_ns', 'physics_substeps', 'decision_period_ns', mode='before')
    @classmethod
    def integer_counts(cls, value):
        if type(value) is not int:
            raise ValueError('counts and clock periods must be integers')
        return value

    @field_validator('solar_gravity', mode='before')
    @classmethod
    def explicit_solar(cls, value):
        if value is not True:
            raise ValueError('explicit solar gravity is required')
        return value

    @model_validator(mode='after')
    def feasible(self):
        e, g, s = self.encounter, self.objective, self.safety
        p, v = e.relative_position_tnr_m, e.relative_velocity_tnr_m_s
        def norm(x):
            return math.sqrt(sum(y*y for y in x))
        pspread, vspread = math.sqrt(3)*e.position_half_width_m, math.sqrt(3)*e.velocity_half_width_m_s
        if not s.minimum_range_m < norm(p)-pspread < norm(p)+pspread < s.maximum_range_m:
            raise ValueError('every initial object must lie in the permitted separation shell')
        if norm(v)+vspread >= s.maximum_relative_speed_m_s or norm(v) <= vspread:
            raise ValueError('initial relative velocities must be nonzero and bounded')
        tca = -sum(a*b for a,b in zip(p, v, strict=True))/norm(v)**2
        # At this fixed time every position/velocity in the entire box is inside
        # the screening sphere. This is an upper bound, not corner sampling.
        miss_bound = norm(tuple(a+tca*b for a,b in zip(p,v,strict=True)))+pspread+tca*vspread
        if not (g.encounter_window_start_s < tca < g.encounter_window_end_s <= g.screening_horizon_s
                and miss_bound < g.initial_predicted_miss_max_m < s.minimum_range_m):
            raise ValueError('the complete initial distribution must contain a real screening threat')
        dot = sum(a*b for a,b in zip(p,v,strict=True))
        dot_error = norm(p)*vspread+norm(v)*pspread+pspread*vspread
        closing_dot_bound = dot+dot_error
        if closing_dot_bound >= 0:
            raise ValueError('every initial object must be closing')
        tca_low = (-dot-dot_error)/(norm(v)+vspread)**2
        tca_high = (-dot+dot_error)/(norm(v)-vspread)**2
        if not g.encounter_window_start_s < tca_low <= tca_high < g.encounter_window_end_s:
            raise ValueError('the complete initial distribution must meet inside the encounter window')
        if not (s.minimum_range_m < g.final_range_min_m < s.maximum_range_m
                and g.outward_speed_min_m_s < s.maximum_relative_speed_m_s
                and g.coast_force_tolerance_n < s.maximum_force_n
                and g.minimum_maneuver_delta_v_m_s < s.maximum_delta_v_m_s
                and g.encounter_window_end_s+max(g.outward_dwell_s,g.final_coast_s) < self.duration_s):
            raise ValueError('the full encounter and final confirmation must fit the episode')
        if not s.minimum_altitude_m < self.initial.altitude_m-self.initial.altitude_half_width_m <= self.initial.altitude_m+self.initial.altitude_half_width_m < s.maximum_altitude_m:
            raise ValueError('all initial spacecraft orbits must be safe')
        for seconds in (self.duration_s, g.screening_horizon_s, g.encounter_window_start_s,
                        g.encounter_window_end_s, g.outward_dwell_s, g.final_coast_s):
            ticks = seconds*1e9/self.decision_period_ns
            if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError('task times must lie on the declared decision grid')
        return self

    def validate_catalogue(self, catalogue):
        row = catalogue.task(self.requirement_id)
        if row.source_id != self.source_id or self.vision_mode not in row.requested_vision_modes or row.target_cooperation != ('none',):
            raise ValueError('debris avoidance configuration differs from its source requirement')


class AvoidanceObservation(OrbitObservation):
    schema_version: Literal['spacesim-autonomy-debris-observation/1'] = 'spacesim-autonomy-debris-observation/1'
    debris_catalog_j2000: State6
    catalog_sample_time_ns: Count
    catalog_delivery_time_ns: Count

    @model_validator(mode='after')
    def catalog_causal(self):
        if not self.catalog_sample_time_ns <= self.catalog_delivery_time_ns <= self.delivery_time_ns:
            raise ValueError('catalog cannot be sampled or delivered in the future')
        return self


class AvoidanceTruthSample(OrbitTruthSample):
    debris_com_j2000: State6


class AvoidanceContext(Contract):
    task: AvoidanceTask
    chaser_mass_kg: Positive
