"""Independent all-spacecraft geometry, pairwise safety and continuous dwell grading."""
import math

import numpy as np

from .benchmark_formation import CooperativeRendezvousObjective, FormationTruthSample
from .benchmark_navigation_score import initial_orbit_errors


def relative_formation(sample):
    states = np.asarray(sample.com_states_j2000)
    radius = np.linalg.norm(states[0, :3])
    momentum = np.cross(states[0, :3], states[0, 3:])
    h = np.linalg.norm(momentum)
    if radius < 6000000 or h < 1:
        raise ValueError('nondegenerate Earth orbit required for formation evidence')
    radial, normal = states[0, :3]/radius, momentum/h
    axes = np.stack((np.cross(normal, radial), normal, radial))
    offsets = states[:, :3]-states[0, :3]
    velocities = states[:, 3:]-states[0, 3:]-np.cross(momentum/radius**2, offsets)
    return offsets @ axes.T, velocities @ axes.T


class FormationScorer:
    def __init__(self, task, spacecraft_masses_kg):
        self.task = task
        self.masses = np.asarray(spacecraft_masses_kg, dtype=float)
        if self.masses.shape != (task.spacecraft_count,) or not np.isfinite(self.masses).all() or np.any(self.masses <= 0):
            raise ValueError('one finite positive physical mass per spacecraft is required')
        self.count = 0
        self.last_time_ns = self.hold_start_ns = self.completion_time_ns = None
        self.first_hold_violation_ns = None
        self.longest_hold_ns = 0
        self.initial_positions = None
        self.max_position_error = self.max_pair_speed = 0.
        self.minimum_distance = math.inf
        self.final_errors = self.final_speeds = self.displacements = None
        self.impulses = np.zeros(task.spacecraft_count)
        self.initial_range = self.final_range = None
        self.active_interval_ns = self.longest_active_interval_ns = 0
        self.evidence_errors, self.initialization_errors = set(), set()
        self.safety_violations, self.quality_violations = set(), set()

    def add(self, raw):
        try:
            sample = FormationTruthSample.model_validate(raw)
            if (len(sample.com_states_j2000) != self.task.spacecraft_count
                    or len(sample.applied_motor_forces_body_n) != self.task.spacecraft_count):
                raise ValueError('physical evidence must include every declared spacecraft')
            positions, velocities = relative_formation(sample)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_physical_sample')
            raise ValueError('invalid formation physical evidence') from error
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        if sample.time_ns != expected:
            self.evidence_errors.add('missing_duplicate_or_reordered_physics_sample')
            self.hold_start_ns = None
        if sample.time_ns > round(self.task.duration_s*1e9):
            self.evidence_errors.add('evidence_beyond_episode_time_limit')
        if sample.external_state_writes:
            self.evidence_errors.add('physical_state_overwritten_after_initialization')
        initial, goal, safety = self.task.initial, self.task.objective, self.task.safety
        if self.last_time_ns is None:
            if initial_orbit_errors(sample.com_states_j2000[0], initial.reference_altitude_m, 0.,
                                    initial.inclination_deg, initial.true_anomaly_deg, 0.):
                self.initialization_errors.add('initial_reference_orbit_outside_declared_distribution')
            if (np.any(abs(positions[1:]-np.asarray(initial.positions_tnr_m)[1:]) > initial.position_half_width_m+1e-5)
                    or np.any(abs(velocities[1:]-np.asarray(initial.velocities_tnr_m_s)[1:]) > initial.velocity_half_width_m_s+1e-8)):
                self.initialization_errors.add('initial_formation_outside_declared_distribution')
            self.initial_positions = positions.copy()
            self.initial_range = float(np.linalg.norm(positions[1]))
        self.final_range = float(np.linalg.norm(positions[1]))
        forces = np.linalg.norm(np.asarray(sample.applied_motor_forces_body_n), axis=1)
        if np.any(forces > safety.maximum_force_n+1e-8):
            self.safety_violations.add('applied_force')
        if sample.time_ns != 0 and sample.time_ns == expected:
            self.impulses += forces*self.task.physics_period_ns*1e-9
        cooperative = isinstance(goal, CooperativeRendezvousObjective)
        if cooperative:
            # Native force readback at t describes the interval ending at t.
            # Repeated/missing timestamps cannot contribute a maneuver interval.
            if (sample.time_ns != 0 and sample.time_ns == expected
                    and np.all(forces >= goal.active_force_threshold_n)):
                self.active_interval_ns += self.task.physics_period_ns
                self.longest_active_interval_ns = max(self.longest_active_interval_ns, self.active_interval_ns)
            else:
                self.active_interval_ns = 0
        for i in range(self.task.spacecraft_count):
            for j in range(i+1, self.task.spacecraft_count):
                distance = float(np.linalg.norm(positions[i]-positions[j]))
                speed = float(np.linalg.norm(velocities[i]-velocities[j]))
                self.minimum_distance = min(self.minimum_distance, distance)
                self.max_pair_speed = max(self.max_pair_speed, speed)
                if distance < safety.minimum_pair_distance_m:
                    self.safety_violations.add('minimum_pair_distance')
                if distance > safety.maximum_pair_distance_m:
                    self.safety_violations.add('maximum_pair_distance')
                if speed > safety.maximum_pair_speed_m_s:
                    self.safety_violations.add('pair_relative_speed')
        self.final_errors = np.linalg.norm(positions-np.asarray(goal.positions_tnr_m), axis=1)
        self.final_speeds = np.linalg.norm(velocities, axis=1)
        self.displacements = np.linalg.norm(positions-self.initial_positions, axis=1)
        motion_valid = np.all(self.displacements[1:] >= goal.minimum_displacement_m)
        if cooperative:
            motion_valid = bool(motion_valid
                and self.initial_range-self.final_range >= goal.minimum_range_reduction_m
                and self.final_range <= goal.maximum_terminal_range_m
                and np.all(self.impulses >= goal.minimum_per_spacecraft_impulse_n_s)
                and self.longest_active_interval_ns >= round(goal.simultaneous_maneuver_dwell_s*1e9))
        self.max_position_error = max(self.max_position_error, float(max(self.final_errors)))
        in_formation = bool(np.all(self.final_errors <= goal.position_tolerance_m)
                            and np.all(self.final_speeds <= goal.speed_tolerance_m_s))
        if goal.kind == 'hold_formation' and not in_formation:
            self.quality_violations.add('continuous_formation_envelope')
            if self.first_hold_violation_ns is None:
                self.first_hold_violation_ns = sample.time_ns
        if in_formation and not self.evidence_errors:
            if self.hold_start_ns is None:
                self.hold_start_ns = sample.time_ns
            held = sample.time_ns-self.hold_start_ns
            self.longest_hold_ns = max(self.longest_hold_ns, held)
            if (held >= round(goal.dwell_s*1e9) and self.completion_time_ns is None
                    and motion_valid):
                self.completion_time_ns = sample.time_ns
        else:
            self.hold_start_ns = None
        self.count += 1
        self.last_time_ns = sample.time_ns
        if cooperative and sample.time_ns == round(self.task.duration_s*1e9):
            if np.any(self.impulses < goal.minimum_per_spacecraft_impulse_n_s):
                self.quality_violations.add('both_spacecraft_active_impulse')
            if self.longest_active_interval_ns < round(goal.simultaneous_maneuver_dwell_s*1e9):
                self.quality_violations.add('simultaneous_cooperative_maneuver')
            if self.initial_range-self.final_range < goal.minimum_range_reduction_m:
                self.quality_violations.add('cooperative_range_reduction')

    def result(self):
        if not self.count or self.evidence_errors:
            outcome = 'invalid_evidence'
        elif self.initialization_errors:
            outcome = 'invalid_initialization'
        elif self.safety_violations:
            outcome = 'safety_violation'
        elif self.completion_time_ns is not None and not self.quality_violations:
            outcome = 'success'
        elif self.last_time_ns == round(self.task.duration_s*1e9):
            outcome = 'task_failure' if self.quality_violations else 'time_limit'
        else:
            outcome = 'incomplete'
        values = lambda array: None if array is None else array.tolist()
        result = dict(schema_version='spacesim-autonomy-formation-score/1', task_fingerprint=self.task.fingerprint(),
            outcome=outcome, success=outcome == 'success', spacecraft_count=self.task.spacecraft_count,
            sample_count=self.count, final_time_s=None if self.last_time_ns is None else self.last_time_ns*1e-9,
            completion_time_s=self.completion_time_ns*1e-9 if outcome == 'success' else None,
            longest_dwell_s=self.longest_hold_ns*1e-9,
            first_hold_violation_s=None if self.first_hold_violation_ns is None else self.first_hold_violation_ns*1e-9,
            formation_position_error_m=None if self.final_errors is None else float(max(self.final_errors)),
            maximum_formation_position_error_m=self.max_position_error if self.count else None,
            per_spacecraft_position_errors_m=values(self.final_errors), per_spacecraft_speeds_m_s=values(self.final_speeds),
            per_spacecraft_displacements_m=values(self.displacements),
            minimum_pair_distance_m=self.minimum_distance if self.count else None,
            minimum_pair_distance_margin_m=self.minimum_distance-self.task.safety.minimum_pair_distance_m if self.count else None,
            maximum_pair_speed_m_s=self.max_pair_speed if self.count else None,
            per_spacecraft_thrust_impulse_n_s=self.impulses.tolist(), per_spacecraft_delta_v_m_s=(self.impulses/self.masses).tolist(),
            thrust_impulse_n_s=float(sum(self.impulses)), thrust_delta_v_m_s=float(sum(self.impulses/self.masses)),
            fuel_kg=None, energy_j=None,
            resource_model='constant per-spacecraft masses; actual motor impulse; fuel and electrical energy not modeled',
            evidence_errors=sorted(self.evidence_errors), initialization_errors=sorted(self.initialization_errors),
            safety_violations=sorted(self.safety_violations), quality_violations=sorted(self.quality_violations),
            safety_sampling='50 Hz all-pairs COM separation and rotating-frame speed; no contact task')
        if isinstance(self.task.objective, CooperativeRendezvousObjective):
            result.update(initial_range_m=self.initial_range, final_range_m=self.final_range,
                cooperative_range_reduction_m=None if self.final_range is None else self.initial_range-self.final_range,
                simultaneous_maneuver_s=self.longest_active_interval_ns*1e-9)
        return result
