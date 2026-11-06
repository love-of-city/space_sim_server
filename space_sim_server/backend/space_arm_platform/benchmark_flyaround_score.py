"""Independent winding, path and safety grading from native COM evidence."""
import math

import numpy as np

from .benchmark_contract import FlightTruthSample
from .benchmark_navigation_score import initial_orbit_errors
from .benchmark_scoring import relative_truth


class FlyAroundScorer:
    def __init__(self, task, chaser_mass_kg):
        if not math.isfinite(chaser_mass_kg) or chaser_mass_kg <= 0:
            raise ValueError("a finite positive model mass is required")
        self.task, self.mass = task, chaser_mass_kg
        self.count = 0
        self.last_time_ns = None
        self.last_angle = None
        self.initial_position = None
        self.signed_angle = self.reverse_travel = 0.
        self.max_radial_error = self.max_plane_error = self.max_speed = 0.
        self.min_range = math.inf
        self.closure_error = None
        self.impulse_ns = 0.
        self.completion_time_ns = None
        self.evidence_errors, self.initialization_errors = set(), set()
        self.safety_violations, self.quality_violations = set(), set()

    def add(self, raw):
        try:
            sample = FlightTruthSample.model_validate(raw)
            position, velocity = relative_truth(sample)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_physical_sample')
            raise ValueError('invalid flyaround physical evidence') from error
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        if sample.time_ns != expected:
            self.evidence_errors.add('missing_duplicate_or_reordered_physics_sample')
        if sample.time_ns > round(self.task.duration_s*1e9):
            self.evidence_errors.add('evidence_beyond_episode_time_limit')
        if sample.external_state_writes:
            self.evidence_errors.add('physical_state_overwritten_after_initialization')
        initial, goal, safety = self.task.initial, self.task.objective, self.task.safety
        if self.last_time_ns is None:
            if initial_orbit_errors(sample.target_com_j2000, initial.target_altitude_m, 0.,
                                    initial.inclination_deg, initial.true_anomaly_deg, 0.):
                self.initialization_errors.add('initial_target_orbit_outside_declared_distribution')
            if (np.any(abs(position-initial.relative_position_tnr_m) > initial.position_half_width_m+1e-5)
                    or np.any(abs(velocity-initial.relative_velocity_tnr_m_s) > initial.velocity_half_width_m_s+1e-8)):
                self.initialization_errors.add('initial_relative_state_outside_declared_distribution')
            self.initial_position = position.copy()
        distance, speed = float(np.linalg.norm(position)), float(np.linalg.norm(velocity))
        radial_error = abs(math.hypot(position[0], position[2])-goal.radius_m)
        plane_error = abs(float(position[1]))
        force = math.dist(sample.applied_motor_force_body_n, (0, 0, 0))
        for violated, name in (
            (distance < safety.minimum_range_m, 'minimum_range'),
            (distance > safety.maximum_range_m, 'maximum_range'),
            (speed > safety.maximum_relative_speed_m_s, 'relative_speed'),
            (force > safety.maximum_force_n+1e-8, 'applied_force'),
        ):
            if violated: self.safety_violations.add(name)
        if radial_error > goal.radial_tolerance_m:
            self.quality_violations.add('flyaround_radius')
        if plane_error > goal.plane_tolerance_m:
            self.quality_violations.add('flyaround_plane')
        angle = math.atan2(position[2], position[0])
        if self.last_angle is not None and sample.time_ns == expected:
            increment = math.atan2(math.sin(angle-self.last_angle), math.cos(angle-self.last_angle))
            # A sub-cadence teleport cannot gain winding even if its logged
            # velocity looks safe. Position evidence must obey the speed bound.
            lower = goal.radius_m-goal.radial_tolerance_m
            if abs(increment) > safety.maximum_relative_speed_m_s*self.task.physics_period_ns*1e-9/lower+1e-7:
                self.evidence_errors.add('angular_travel_exceeds_physical_speed_bound')
            self.signed_angle += increment
            self.reverse_travel += max(0., -increment)
            self.impulse_ns += force*self.task.physics_period_ns*1e-9
        if self.reverse_travel > goal.maximum_reverse_travel_rad:
            self.quality_violations.add('flyaround_reverse_travel')
        self.closure_error = float(np.linalg.norm(position-self.initial_position))
        if (self.signed_angle >= 2*math.pi and self.closure_error <= goal.closure_tolerance_m
                and not self.evidence_errors and not self.initialization_errors
                and not self.quality_violations and not self.safety_violations
                and self.completion_time_ns is None):
            self.completion_time_ns = sample.time_ns
        self.count += 1
        self.last_time_ns, self.last_angle = sample.time_ns, angle
        self.min_range = min(self.min_range, distance)
        self.max_speed = max(self.max_speed, speed)
        self.max_radial_error = max(self.max_radial_error, radial_error)
        self.max_plane_error = max(self.max_plane_error, plane_error)

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
        return dict(schema_version='spacesim-autonomy-flyaround-score/1', task_fingerprint=self.task.fingerprint(),
            outcome=outcome, success=outcome == 'success', sample_count=self.count,
            final_time_s=None if self.last_time_ns is None else self.last_time_ns*1e-9,
            completion_time_s=self.completion_time_ns*1e-9 if outcome == 'success' else None,
            signed_revolutions=self.signed_angle/(2*math.pi), reverse_travel_rad=self.reverse_travel,
            maximum_radial_error_m=self.max_radial_error if self.count else None,
            maximum_plane_error_m=self.max_plane_error if self.count else None,
            final_closure_error_m=self.closure_error,
            minimum_range_m=self.min_range if self.count else None,
            minimum_range_margin_m=self.min_range-self.task.safety.minimum_range_m if self.count else None,
            maximum_relative_speed_m_s=self.max_speed if self.count else None,
            thrust_impulse_n_s=self.impulse_ns, thrust_delta_v_m_s=self.impulse_ns/self.mass,
            fuel_kg=None, energy_j=None,
            resource_model='constant mass; thrust integral only; fuel and electrical energy not modeled',
            evidence_errors=sorted(self.evidence_errors), initialization_errors=sorted(self.initialization_errors),
            safety_violations=sorted(self.safety_violations), quality_violations=sorted(self.quality_violations),
            safety_sampling='50 Hz COM shell and rotating-relative-speed checks; not a contact task')
