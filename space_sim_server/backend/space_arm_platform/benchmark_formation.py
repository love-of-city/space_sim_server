"""Contracts for independently actuated cooperative two/three-spacecraft formations."""
import math
from typing import Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_contract import Contract, Count, Nonnegative, Number, Positive, SeedSplits, State6, Vector3


class FormationInitial(Contract):
    reference_altitude_m: Number = Field(ge=200000, le=2000000)
    inclination_deg: Number = Field(ge=0, le=180)
    true_anomaly_deg: Number
    positions_tnr_m: tuple[Vector3, ...] = Field(min_length=2, max_length=3)
    velocities_tnr_m_s: tuple[Vector3, ...] = Field(min_length=2, max_length=3)
    position_half_width_m: Nonnegative
    velocity_half_width_m_s: Nonnegative


class FormationObjective(Contract):
    kind: Literal['establish_formation', 'hold_formation', 'reconfigure_formation']
    positions_tnr_m: tuple[Vector3, ...] = Field(min_length=2, max_length=3)
    position_tolerance_m: Positive
    speed_tolerance_m_s: Positive
    dwell_s: Positive
    minimum_displacement_m: Nonnegative


class CooperativeRendezvousObjective(FormationObjective):
    kind: Literal['cooperative_rendezvous']
    minimum_initial_range_m: Positive = Field(ge=500)
    maximum_terminal_range_m: Positive = Field(le=300)
    minimum_range_reduction_m: Positive = Field(ge=100)
    minimum_per_spacecraft_impulse_n_s: Positive
    active_force_threshold_n: Positive
    simultaneous_maneuver_dwell_s: Positive


class FormationSafety(Contract):
    minimum_pair_distance_m: Number = Field(ge=100)
    maximum_pair_distance_m: Positive
    maximum_pair_speed_m_s: Positive
    maximum_force_n: Positive


class FormationSensors(Contract):
    kind: Literal['independent_gnss_solutions_and_orbit_filters']
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    delay_s: Literal[0.0]
    inter_spacecraft_link: Literal['cooperative_shared_filtered_states']
    reference_point: Literal['assembled_center_of_mass']
    frame: Literal['J2000']

    @field_validator('delay_s', mode='before')
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (float, int):
            raise ValueError('delay must be numeric')
        return value


class FormationTask(Contract):
    schema_version: Literal['spacesim-autonomy-formation/1']
    configuration_id: str = Field(pattern=r'^[a-z0-9]+(?:-[a-z0-9]+)*$')
    requirement_id: Literal['ssb.formation.pair-establish', 'ssb.formation.pair-maintain',
                            'ssb.formation.pair-reconfigure', 'ssb.formation.swarm-establish',
                            'ssb.formation.swarm-maintain', 'ssb.formation.swarm-reconfigure',
                            'ssb.rendezvous.dual-controlled']
    source_id: Literal['D14', 'D15', 'D16', 'D19', 'M01', 'M02', 'M03']
    backend: Literal['native_original_sarm_formation_v1']
    target_cooperation: Literal['C']
    spacecraft_count: Literal[2, 3]
    controlled_spacecraft: Literal[2, 3]
    control_architecture: Literal['centralized']
    vision_mode: Literal['V0']
    action_type: Literal['per_spacecraft_ideal_force_j2000_at_assembly_com']
    attitude_control: Literal['none_passive_attitude']
    reference_frame: Literal['spacecraft0_TNR_tangent_normal_radial']
    initial: FormationInitial
    objective: FormationObjective | CooperativeRendezvousObjective
    safety: FormationSafety
    observation: FormationSensors
    seeds: SeedSplits
    duration_s: Positive
    physics_period_ns: Literal[20000000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[500000000]
    earth_gravity: Literal['point_mass', 'j2']
    solar_gravity: Literal[True]
    drag_enabled: StrictBool
    srp_enabled: StrictBool
    disturbance_sigma_m_s2: Nonnegative = Field(le=.001)
    mass_model: Literal['constant_no_propellant_model']

    @field_validator('spacecraft_count', 'controlled_spacecraft', 'physics_period_ns',
                     'physics_substeps', 'decision_period_ns', mode='before')
    @classmethod
    def integer_counts(cls, value):
        if type(value) is not int:
            raise ValueError('counts and periods must be integers')
        return value

    @field_validator('solar_gravity', mode='before')
    @classmethod
    def explicit_solar(cls, value):
        if value is not True:
            raise ValueError('explicit solar gravity is required')
        return value

    @model_validator(mode='after')
    def formation_geometry(self):
        initial, goal, safety = self.initial, self.objective, self.safety
        source = {'D14': ('pair-establish', 'establish_formation'), 'D15': ('pair-maintain', 'hold_formation'),
                  'D16': ('pair-reconfigure', 'reconfigure_formation'), 'M01': ('swarm-establish', 'establish_formation'),
                  'M02': ('swarm-maintain', 'hold_formation'), 'M03': ('swarm-reconfigure', 'reconfigure_formation'),
                  'D19': ('dual-controlled', 'cooperative_rendezvous')}
        requirement, kind = source[self.source_id]
        count = 2 if self.source_id.startswith('D') else 3
        namespace = 'ssb.rendezvous.' if self.source_id == 'D19' else 'ssb.formation.'
        if (self.requirement_id != namespace+requirement or goal.kind != kind
                or self.spacecraft_count != count or self.controlled_spacecraft != count):
            raise ValueError('source, task kind and independently controlled spacecraft must agree')
        arrays = (initial.positions_tnr_m, initial.velocities_tnr_m_s, goal.positions_tnr_m)
        if any(len(array) != count or array[0] != (0., 0., 0.) for array in arrays):
            raise ValueError('all spacecraft must be listed; spacecraft zero defines the TNR origin')
        for i in range(count):
            for j in range(i+1, count):
                coefficient = 1 if i == 0 else 2
                spread = coefficient*math.sqrt(3)*initial.position_half_width_m
                distance = math.dist(initial.positions_tnr_m[i], initial.positions_tnr_m[j])
                arrival_distance = math.dist(goal.positions_tnr_m[i], goal.positions_tnr_m[j])
                if not (safety.minimum_pair_distance_m < distance-spread
                        and distance+spread < safety.maximum_pair_distance_m):
                    raise ValueError('every initial pair must satisfy separation limits over its complete distribution')
                tolerance = coefficient*goal.position_tolerance_m
                if not (safety.minimum_pair_distance_m < arrival_distance-tolerance
                        and arrival_distance+tolerance < safety.maximum_pair_distance_m):
                    raise ValueError('the entire formation success region must satisfy pairwise separation limits')
                speed = math.dist(initial.velocities_tnr_m_s[i], initial.velocities_tnr_m_s[j])
                if speed+coefficient*math.sqrt(3)*initial.velocity_half_width_m_s >= safety.maximum_pair_speed_m_s:
                    raise ValueError('every initial pair speed must be safe')
        if 2*goal.speed_tolerance_m_s >= safety.maximum_pair_speed_m_s:
            raise ValueError('all formation velocity tolerances must imply safe pairwise speeds')
        if goal.kind == 'hold_formation':
            if goal.dwell_s != self.duration_s or goal.minimum_displacement_m != 0:
                raise ValueError('maintenance requires continuous hold for the full episode')
            for i in range(1, count):
                if (math.dist(initial.positions_tnr_m[i], goal.positions_tnr_m[i])+math.sqrt(3)*initial.position_half_width_m >= goal.position_tolerance_m
                        or math.dist(initial.velocities_tnr_m_s[i], (0, 0, 0))+math.sqrt(3)*initial.velocity_half_width_m_s >= goal.speed_tolerance_m_s):
                    raise ValueError('every maintained spacecraft must start inside the hold envelope')
        else:
            if goal.dwell_s >= self.duration_s or goal.minimum_displacement_m <= 0:
                raise ValueError('establishment/reconfiguration must require actual motion before dwell')
            for i in range(1, count):
                change = math.dist(initial.positions_tnr_m[i], goal.positions_tnr_m[i])
                if change-math.sqrt(3)*initial.position_half_width_m-goal.position_tolerance_m < goal.minimum_displacement_m:
                    raise ValueError('every follower must make the declared minimum formation change')
            if goal.kind == 'establish_formation' and initial.position_half_width_m < 5.:
                raise ValueError('establishment must sample an initially dispersed constellation')
            if goal.kind == 'reconfigure_formation' and initial.position_half_width_m > .5:
                raise ValueError('reconfiguration starts in a bounded known initial formation')
            if isinstance(goal, CooperativeRendezvousObjective):
                initial_minimum = math.dist(initial.positions_tnr_m[1], (0, 0, 0))-math.sqrt(3)*initial.position_half_width_m
                terminal_maximum = math.dist(goal.positions_tnr_m[1], (0, 0, 0))+goal.position_tolerance_m
                if initial_minimum < goal.minimum_initial_range_m or terminal_maximum > goal.maximum_terminal_range_m:
                    raise ValueError('cooperative rendezvous must start outside and finish inside the declared ranges')
                if goal.minimum_range_reduction_m > initial_minimum-terminal_maximum:
                    raise ValueError('every initial and terminal pair must imply the required range reduction')
                if goal.minimum_range_reduction_m > safety.maximum_pair_speed_m_s*(self.duration_s-goal.dwell_s):
                    raise ValueError('the range reduction must fit the speed limit and final dwell')
                if not goal.active_force_threshold_n < safety.maximum_force_n:
                    raise ValueError('simultaneous activity must be distinguishable below the force limit')
                if goal.minimum_per_spacecraft_impulse_n_s >= safety.maximum_force_n*self.duration_s:
                    raise ValueError('per-spacecraft impulse requirement must be physically attainable')
                if not goal.simultaneous_maneuver_dwell_s < self.duration_s-goal.dwell_s:
                    raise ValueError('simultaneous maneuver must fit before the final dwell')
                ticks = goal.simultaneous_maneuver_dwell_s*1e9/self.physics_period_ns
                if not math.isclose(ticks, round(ticks), abs_tol=1e-8, rel_tol=0):
                    raise ValueError('simultaneous maneuver dwell must lie on the physical grid')
        for seconds, period in ((self.duration_s, self.decision_period_ns), (goal.dwell_s, self.physics_period_ns)):
            ticks = seconds*1e9/period
            if not math.isclose(ticks, round(ticks), abs_tol=1e-8, rel_tol=0):
                raise ValueError('duration and dwell must lie on declared clock grids')
        return self

    def validate_catalogue(self, catalogue):
        requirement = catalogue.task(self.requirement_id)
        if (requirement.source_id != self.source_id or self.vision_mode not in requirement.requested_vision_modes
                or self.target_cooperation not in requirement.target_cooperation):
            raise ValueError('formation does not match its catalogue requirement')


class FormationObservation(Contract):
    schema_version: Literal['spacesim-autonomy-formation-observation/1'] = 'spacesim-autonomy-formation-observation/1'
    episode_id: str = Field(min_length=1)
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    estimates_j2000: tuple[State6, ...] = Field(min_length=2, max_length=3)

    @model_validator(mode='after')
    def causal(self):
        if self.delivery_time_ns < self.sample_time_ns:
            raise ValueError('observation cannot arrive before sampling')
        return self


class FormationAction(Contract):
    schema_version: Literal['spacesim-autonomy-formation-force/1'] = 'spacesim-autonomy-formation-force/1'
    episode_id: str
    observation_sequence: Count
    forces_j2000_n: tuple[Vector3, ...] = Field(min_length=2, max_length=3)

    def validate_for(self, observation, task):
        action = FormationAction.model_validate(self)
        if action.episode_id != observation.episode_id or action.observation_sequence != observation.sequence:
            raise ValueError('stale formation action')
        if len(action.forces_j2000_n) != task.spacecraft_count or len(observation.estimates_j2000) != task.spacecraft_count:
            raise ValueError('exactly one force vector and observation per spacecraft is required')
        if any(math.dist(force, (0, 0, 0)) > task.safety.maximum_force_n+1e-9 for force in action.forces_j2000_n):
            raise ValueError('requested formation force exceeds the per-spacecraft limit')


class FormationTruthSample(Contract):
    time_ns: Count
    com_states_j2000: tuple[State6, ...] = Field(min_length=2, max_length=3)
    applied_motor_forces_body_n: tuple[Vector3, ...] = Field(min_length=2, max_length=3)
    external_state_writes: Count


class FormationContext(Contract):
    task: FormationTask
    spacecraft_masses_kg: tuple[Positive, ...] = Field(min_length=2, max_length=3)

    @model_validator(mode='after')
    def all_masses(self):
        if len(self.spacecraft_masses_kg) != self.task.spacecraft_count:
            raise ValueError('context must identify every physical spacecraft mass')
        return self
