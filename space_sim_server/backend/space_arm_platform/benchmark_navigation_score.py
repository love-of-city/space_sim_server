"""Grade method estimates against contemporaneous private physical states."""
from __future__ import annotations

import math
import numpy as np

from .benchmark_contract import Contract, FlightTruthSample
from .benchmark_navigation import NavigationEstimateEvidence
from .benchmark_orbit import OrbitTruthSample
from .benchmark_orbit_score import orbital_elements
from .benchmark_scoring import relative_truth


def initial_orbit_errors(state, altitude, half_width, inclination_deg, anomaly_deg, anomaly_half_width):
    orbit = orbital_elements(state)
    inc = math.radians(inclination_deg)
    normal = np.array([0., -math.sin(inc), math.cos(inc)])
    p = np.asarray(state[:3])
    phase = math.degrees(math.atan2(p[1]*math.cos(inc)+p[2]*math.sin(inc), p[0]))
    phase_error = abs((phase-anomaly_deg+180) % 360-180)
    return (abs(orbit["altitude_m"]-altitude) > half_width+1e-5 or orbit["eccentricity"] > 1e-8
            or np.linalg.norm(orbit["normal"]-normal) > 1e-8 or phase_error > anomaly_half_width+1e-8)


class NavigationScorer:
    def __init__(self, task, chaser_mass_kg):
        if not math.isfinite(chaser_mass_kg) or chaser_mass_kg <= 0:
            raise ValueError("a finite positive model mass is required")
        self.task, self.mass = task, chaser_mass_kg
        self.last_time_ns = None
        self.latest_truth = None
        self.count = self.estimate_count = 0
        self.squared_position_error = self.squared_velocity_error = 0.
        self.peak_position_error = self.peak_velocity_error = 0.
        self.final_position_error = self.final_velocity_error = None
        self.total_impulse = 0.
        self.initialization_errors, self.evidence_errors, self.safety_violations = set(), set(), set()

    def add(self, raw):
        document = raw.model_dump() if isinstance(raw, Contract) else raw
        if isinstance(document, dict) and document.get("record_type") == "navigation_estimate":
            self._estimate(document)
            return
        try:
            sample = (OrbitTruthSample if self.task.spacecraft_count == 1 else FlightTruthSample).model_validate(document)
            self._physical(sample)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add("invalid_physical_sample")
            raise ValueError("invalid navigation physical evidence") from error

    def _physical(self, sample):
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        if sample.time_ns != expected:
            self.evidence_errors.add("missing_duplicate_or_reordered_physics_sample")
        if sample.time_ns > round(self.task.duration_s*1e9):
            self.evidence_errors.add("evidence_beyond_episode_time_limit")
        if sample.external_state_writes:
            self.evidence_errors.add("physical_state_overwritten_after_initialization")
        limits, initial = self.task.safety, self.task.initial
        force = math.dist(sample.applied_motor_force_body_n, (0, 0, 0))
        if force > limits.maximum_uncommanded_force_n:
            self.safety_violations.add("navigation_actuation_not_allowed")
        if sample.time_ns and sample.time_ns == expected:
            self.total_impulse += force*self.task.physics_period_ns*1e-9
        if self.task.spacecraft_count == 1:
            state = sample.chaser_com_j2000
            orbit = orbital_elements(state)
            if not limits.minimum_altitude_m <= orbit["altitude_m"] <= limits.maximum_altitude_m:
                self.safety_violations.add("altitude")
            if orbit["semimajor_axis_m"] is None or orbit["eccentricity"] > limits.maximum_eccentricity:
                self.safety_violations.add("orbital_eccentricity_or_escape")
            if self.last_time_ns is None and initial_orbit_errors(state, initial.altitude_m,
                    initial.altitude_half_width_m, initial.inclination_deg, initial.true_anomaly_deg, initial.anomaly_half_width_deg):
                self.initialization_errors.add("initial_orbit_outside_declared_distribution")
            self.latest_truth = np.asarray(state)
        else:
            position, velocity = relative_truth(sample)
            radius = float(np.linalg.norm(position))
            if not limits.minimum_range_m <= radius <= limits.maximum_range_m:
                self.safety_violations.add("relative_range")
            if np.linalg.norm(velocity) > limits.maximum_relative_speed_m_s:
                self.safety_violations.add("relative_speed")
            if self.last_time_ns is None:
                if self._initial_target_orbit_invalid(sample):
                    self.initialization_errors.add("initial_target_orbit_outside_declared_distribution")
                if (np.any(abs(position-initial.relative_position_tnr_m) > initial.position_half_width_m+1e-5)
                        or np.any(abs(velocity-initial.relative_velocity_tnr_m_s) > initial.velocity_half_width_m_s+1e-8)):
                    self.initialization_errors.add("initial_relative_state_outside_declared_distribution")
            # Inertial vector difference; not a rotating-frame velocity.
            self.latest_truth = self._relative_estimate_truth(sample)
        self.last_time_ns = sample.time_ns
        self.count += 1
        if sample.time_ns == round(self.task.duration_s*1e9) and self.estimate_count != self.task.expected_estimates:
            self.evidence_errors.add("incomplete_navigation_estimate_coverage")

    def _relative_estimate_truth(self, sample):
        return np.asarray(sample.chaser_com_j2000)-sample.target_com_j2000

    def _initial_target_orbit_invalid(self, sample):
        initial = self.task.initial
        return initial_orbit_errors(sample.target_com_j2000, initial.target_altitude_m, 0.,
                                    initial.inclination_deg, initial.true_anomaly_deg, 0.)

    def _estimate(self, raw):
        try:
            estimate = NavigationEstimateEvidence.model_validate(raw)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add("invalid_navigation_estimate_record")
            raise ValueError("invalid navigation estimate evidence") from error
        expected_time = self.estimate_count*self.task.decision_period_ns
        if (self.latest_truth is None or estimate.sequence != self.estimate_count
                or estimate.estimate_time_ns != expected_time or self.last_time_ns != expected_time
                or self.estimate_count >= self.task.expected_estimates):
            self.evidence_errors.add("missing_duplicate_reordered_or_noncontemporaneous_estimate")
            return
        if estimate.reference_frame != self.task.reference_frame:
            self.evidence_errors.add("wrong_estimate_reference_frame")
            return
        error = np.asarray(estimate.estimated_state)-self.latest_truth
        position_error, velocity_error = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        if not math.isfinite(position_error**2+velocity_error**2):
            self.evidence_errors.add("nonfinite_navigation_error")
            raise ValueError("navigation error exceeds finite scoring range")
        self.squared_position_error += position_error**2
        self.squared_velocity_error += velocity_error**2
        self.peak_position_error = max(self.peak_position_error, position_error)
        self.peak_velocity_error = max(self.peak_velocity_error, velocity_error)
        self.final_position_error, self.final_velocity_error = position_error, velocity_error
        self.estimate_count += 1

    def result(self):
        position_rmse = math.sqrt(self.squared_position_error/self.estimate_count) if self.estimate_count else None
        velocity_rmse = math.sqrt(self.squared_velocity_error/self.estimate_count) if self.estimate_count else None
        goal = self.task.objective
        violations = [name for name, value, limit in (
            ("position_rmse", position_rmse, goal.position_rmse_limit_m),
            ("velocity_rmse", velocity_rmse, goal.velocity_rmse_limit_m_s),
            ("maximum_position_error", self.peak_position_error, goal.maximum_position_error_m),
            ("maximum_velocity_error", self.peak_velocity_error, goal.maximum_velocity_error_m_s))
            if value is not None and value > limit]
        complete = self.last_time_ns == round(self.task.duration_s*1e9)
        if self.initialization_errors:
            outcome = "invalid_initialization"
        elif not self.count or self.evidence_errors:
            outcome = "invalid_evidence"
        elif self.safety_violations:
            outcome = "safety_violation"
        elif complete:
            outcome = "task_failure" if violations else "success"
        else:
            outcome = "incomplete"
        return {"schema_version": "spacesim-autonomy-navigation-score/1", "task_fingerprint": self.task.fingerprint(),
                "outcome": outcome, "success": outcome == "success", "sample_count": self.count,
                "estimate_count": self.estimate_count, "expected_estimates": self.task.expected_estimates,
                "final_time_s": self.last_time_ns*1e-9 if self.count else None,
                "completion_time_s": self.task.duration_s if outcome == "success" else None,
                "position_rmse_m": position_rmse, "velocity_rmse_m_s": velocity_rmse,
                "maximum_position_error_m": self.peak_position_error if self.estimate_count else None,
                "maximum_velocity_error_m_s": self.peak_velocity_error if self.estimate_count else None,
                "final_position_error_m": self.final_position_error, "final_velocity_error_m_s": self.final_velocity_error,
                "estimate_frame": self.task.reference_frame, "quality_violations": violations,
                "initialization_errors": sorted(self.initialization_errors), "evidence_errors": sorted(self.evidence_errors),
                "safety_violations": sorted(self.safety_violations), "thrust_impulse_n_s": self.total_impulse,
                "thrust_delta_v_m_s": self.total_impulse/self.mass, "fuel_kg": None, "energy_j": None,
                "evaluation_scope": "full-duration state estimation; unfiltered GNSS solutions; no actuation authority"}
