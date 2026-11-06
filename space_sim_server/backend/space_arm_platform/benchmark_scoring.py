"""Independent replayable grading of physical flight evidence.

No controller phases, navigation estimates or success flags enter this module.
The TNR projection is reconstructed from raw J2000 COM states, independently of
the guidance helpers used by methods. Cadence is part of the evidence contract.
"""
from __future__ import annotations

import math

import numpy as np

from .benchmark_contract import FlightTruthSample, WaypointTask


def relative_truth(sample: FlightTruthSample) -> tuple[np.ndarray, np.ndarray]:
    target = np.asarray(sample.target_com_j2000)
    chaser = np.asarray(sample.chaser_com_j2000)
    radius = np.linalg.norm(target[:3])
    momentum = np.cross(target[:3], target[3:])
    momentum_norm = np.linalg.norm(momentum)
    if radius < 6_000_000 or momentum_norm < 1:
        raise ValueError("nondegenerate Earth orbit required for TNR evidence")
    radial = target[:3] / radius
    normal = momentum / momentum_norm
    tangent = np.cross(normal, radial)
    axes = np.stack((tangent, normal, radial))
    displacement = chaser[:3] - target[:3]
    velocity = chaser[3:] - target[3:] - np.cross(momentum / radius**2, displacement)
    return axes @ displacement, axes @ velocity


class WaypointScorer:
    def __init__(self, task: WaypointTask, chaser_mass_kg: float):
        if not math.isfinite(chaser_mass_kg) or chaser_mass_kg <= 0:
            raise ValueError("a finite positive model mass is required")
        self.task, self.mass = task, chaser_mass_kg
        self.count = 0
        self.last_time_ns = None
        self.hold_start_ns = None
        self.longest_hold_ns = 0
        self.completion_time_ns = None
        self.min_range = math.inf
        self.max_speed = 0.0
        self.final_error = None
        self.impulse_ns = 0.0
        self.evidence_errors: set[str] = set()
        self.safety_violations: set[str] = set()

    def add(self, raw: FlightTruthSample | dict) -> None:
        try:
            sample = FlightTruthSample.model_validate(raw)
            position, velocity = relative_truth(sample)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add("invalid_physical_sample")
            raise ValueError("invalid benchmark physical evidence") from error
        expected = 0 if self.last_time_ns is None else self.last_time_ns + self.task.physics_period_ns
        if sample.time_ns != expected:
            self.evidence_errors.add("missing_duplicate_or_reordered_physics_sample")
            self.hold_start_ns = None
        if sample.time_ns > round(self.task.duration_s * 1e9):
            self.evidence_errors.add("evidence_beyond_episode_time_limit")
        if sample.external_state_writes:
            self.evidence_errors.add("physical_state_overwritten_after_initialization")
        distance = float(np.linalg.norm(position))
        speed = float(np.linalg.norm(velocity))
        error = float(np.linalg.norm(position - self.task.objective.position_tnr_m))
        force = math.dist(sample.applied_motor_force_body_n, (0, 0, 0))
        limits = self.task.safety
        if distance < limits.minimum_range_m:
            self.safety_violations.add("minimum_range")
        if distance > limits.maximum_range_m:
            self.safety_violations.add("maximum_range")
        if speed > limits.maximum_relative_speed_m_s:
            self.safety_violations.add("relative_speed")
        if force > limits.maximum_force_n + 1e-8:
            self.safety_violations.add("applied_force")
        if sample.time_ns != 0 and sample.time_ns == expected:
            # Native motor force is constant within each 20 ms outer interval;
            # the actuator adapter rotates its vector without changing its norm.
            self.impulse_ns += force * self.task.physics_period_ns * 1e-9
        in_region = error <= self.task.objective.position_tolerance_m and speed <= self.task.objective.speed_tolerance_m_s
        if in_region and not self.evidence_errors:
            if self.hold_start_ns is None:
                self.hold_start_ns = sample.time_ns
            dwell = sample.time_ns - self.hold_start_ns
            self.longest_hold_ns = max(self.longest_hold_ns, dwell)
            if dwell >= round(self.task.objective.dwell_s * 1e9) and self.completion_time_ns is None:
                self.completion_time_ns = sample.time_ns
        else:
            self.hold_start_ns = None
        self.count += 1
        self.min_range = min(self.min_range, distance)
        self.max_speed = max(self.max_speed, speed)
        self.final_error = error
        self.last_time_ns = sample.time_ns

    def result(self) -> dict:
        if not self.count or self.evidence_errors:
            outcome = "invalid_evidence"
        elif self.safety_violations:
            outcome = "safety_violation"
        elif self.completion_time_ns is not None:
            outcome = "success"
        elif self.last_time_ns == round(self.task.duration_s * 1e9):
            outcome = "time_limit"
        else:
            outcome = "incomplete"
        return {
            "schema_version": "spacesim-autonomy-waypoint-score/1",
            "task_fingerprint": self.task.fingerprint(),
            "outcome": outcome,
            "success": outcome == "success",
            "sample_count": self.count,
            "final_time_s": None if self.last_time_ns is None else self.last_time_ns * 1e-9,
            "completion_time_s": (self.completion_time_ns * 1e-9 if outcome == "success" else None),
            "longest_dwell_s": self.longest_hold_ns * 1e-9,
            "minimum_range_m": self.min_range if self.count else None,
            "minimum_range_margin_m": self.min_range - self.task.safety.minimum_range_m if self.count else None,
            "maximum_relative_speed_m_s": self.max_speed if self.count else None,
            "final_position_error_m": self.final_error,
            "thrust_impulse_n_s": self.impulse_ns,
            "thrust_delta_v_m_s": self.impulse_ns / self.mass,
            "fuel_kg": None,
            "energy_j": None,
            "resource_model": "constant mass; thrust integral only; fuel and electrical energy not modeled",
            "evidence_errors": sorted(self.evidence_errors),
            "safety_violations": sorted(self.safety_violations),
            "safety_sampling": "50 Hz COM shell and rotating-relative-speed checks; not a contact task",
        }
