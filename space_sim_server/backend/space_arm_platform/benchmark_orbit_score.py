"""Independent orbital elements and native-force impulse grading at 50 Hz."""
from __future__ import annotations

import math
import numpy as np

from .benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS, ImpulseObjective, LowThrustObjective, OrbitMaintenanceObjective, OrbitTruthSample


def orbital_elements(state):
    """Osculating two-body diagnostics; force propagation stays in Basilisk."""
    value = np.asarray(state, dtype=float)
    r, v = value[:3], value[3:]
    radius = np.linalg.norm(r)
    h = np.cross(r, v)
    hnorm = np.linalg.norm(h)
    if value.shape != (6,) or not np.isfinite(value).all() or radius < 1 or hnorm < 1:
        raise ValueError("nondegenerate Earth orbit required")
    energy = float(v@v/2-MU/radius)
    eccentricity = float(np.linalg.norm(np.cross(v, h)/MU-r/radius))
    # Do not silently turn an unbound escape trajectory into a circular orbit.
    semimajor = -MU/(2*energy) if energy < 0 else None
    return {"altitude_m": float(radius-RADIUS), "semimajor_axis_m": semimajor,
            "eccentricity": eccentricity, "normal": h/hnorm,
            "perigee_altitude_m": float(hnorm**2/MU/(1+eccentricity)-RADIUS),
            "apogee_altitude_m": semimajor*(1+eccentricity)-RADIUS if semimajor else None}


def rotate_body_force(quaternion, force):
    # Independent of the native adapter's attitude library.
    w, x, y, z = quaternion
    q = np.array([x, y, z])
    f = np.asarray(force)
    return f + 2*np.cross(q, np.cross(q, f)+w*f)


class OrbitScorer:
    def __init__(self, task, chaser_mass_kg):
        if not math.isfinite(chaser_mass_kg) or chaser_mass_kg <= 0:
            raise ValueError("a finite positive model mass is required")
        self.task, self.mass = task, chaser_mass_kg
        self.count = 0
        self.last_time_ns = self.hold_start_ns = self.completion_time_ns = None
        self.longest_hold_ns = 0
        self.impulse = np.zeros(3)
        self.total_impulse = 0.
        self.initial_normal = None
        self.minimum_altitude = math.inf
        self.maximum_eccentricity = 0.
        self.final = None
        self.evidence_errors, self.safety_violations = set(), set()
        self.maintenance_samples = self.failed_maintenance_samples = 0
        self.maintenance_axis_error = self.maintenance_eccentricity = self.maintenance_plane_error = 0.
        self.initial_semimajor_axis = self.semimajor_axis_increase = None
        self.active_thrust_ns = self.longest_active_thrust_ns = 0

    def add(self, raw):
        try:
            sample = OrbitTruthSample.model_validate(raw)
            elements = orbital_elements(sample.chaser_com_j2000)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add("invalid_physical_sample")
            raise ValueError("invalid orbital physical evidence") from error
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        if sample.time_ns != expected:
            self.evidence_errors.add("missing_duplicate_or_reordered_physics_sample")
            self.hold_start_ns = None
        if sample.time_ns > round(self.task.duration_s*1e9):
            self.evidence_errors.add("evidence_beyond_episode_time_limit")
        if sample.external_state_writes:
            self.evidence_errors.add("physical_state_overwritten_after_initialization")
        if self.initial_normal is None:
            self.initial_semimajor_axis = elements['semimajor_axis_m']
            initial = self.task.initial
            inclination = math.radians(initial.inclination_deg)
            normal = np.array([0., -math.sin(inclination), math.cos(inclination)])
            p = np.asarray(sample.chaser_com_j2000[:3])
            phase = math.degrees(math.atan2(p[1]*math.cos(inclination)+p[2]*math.sin(inclination), p[0]))
            phase_error = abs((phase-initial.true_anomaly_deg+180) % 360-180)
            if (abs(elements["altitude_m"]-initial.altitude_m) > initial.altitude_half_width_m+1e-5
                    or elements["eccentricity"] > 1e-8 or np.linalg.norm(elements["normal"]-normal) > 1e-8
                    or phase_error > initial.anomaly_half_width_deg+1e-8):
                self.evidence_errors.add("initial_orbit_outside_declared_distribution")
            self.initial_normal = elements["normal"]
        force = rotate_body_force(sample.orientation_body_to_j2000_wxyz, sample.applied_motor_force_body_n)
        magnitude = float(np.linalg.norm(force))
        if sample.time_ns and sample.time_ns == expected:
            dt = self.task.physics_period_ns*1e-9
            self.impulse += force*dt
            self.total_impulse += magnitude*dt
        goal, limits = self.task.objective, self.task.safety
        low_thrust = isinstance(goal, LowThrustObjective)
        if low_thrust:
            if sample.time_ns and sample.time_ns == expected and magnitude >= goal.minimum_active_force_n:
                self.active_thrust_ns += self.task.physics_period_ns
                self.longest_active_thrust_ns = max(self.longest_active_thrust_ns, self.active_thrust_ns)
            else:
                self.active_thrust_ns = 0
            self.semimajor_axis_increase = (None if elements['semimajor_axis_m'] is None or self.initial_semimajor_axis is None
                else elements['semimajor_axis_m']-self.initial_semimajor_axis)
        if elements["altitude_m"] < limits.minimum_altitude_m:
            self.safety_violations.add("minimum_altitude")
        if elements["altitude_m"] > limits.maximum_altitude_m:
            self.safety_violations.add("maximum_altitude")
        if elements["eccentricity"] > limits.maximum_eccentricity or elements["semimajor_axis_m"] is None:
            self.safety_violations.add("orbital_eccentricity_or_escape")
        if magnitude > limits.maximum_force_n+1e-8:
            self.safety_violations.add("applied_force")
        if self.total_impulse/self.mass > limits.maximum_delta_v_m_s+1e-9:
            self.safety_violations.add("delta_v_budget")
        plane_error = float(math.atan2(np.linalg.norm(np.cross(elements["normal"], self.initial_normal)),
                                      np.dot(elements["normal"], self.initial_normal)))
        vector_error = axis_error = None
        maintenance = isinstance(goal, OrbitMaintenanceObjective)
        coasting = maintenance or magnitude <= goal.coast_force_tolerance_n
        if isinstance(goal, ImpulseObjective):
            vector_error = float(np.linalg.norm(self.impulse/self.mass-goal.delta_velocity_j2000_m_s))
            # A sample at t reports the force used over (t-dt,t].
            if sample.time_ns and magnitude > goal.coast_force_tolerance_n and (
                    sample.time_ns <= round(goal.earliest_burn_s*1e9) or sample.time_ns > round(goal.latest_burn_s*1e9)):
                self.safety_violations.add("thrust_outside_burn_window")
            in_region = vector_error <= goal.vector_tolerance_m_s and coasting
            if low_thrust:
                in_region = (in_region and self.longest_active_thrust_ns >= round(goal.minimum_continuous_thrust_s*1e9)
                    and self.semimajor_axis_increase is not None
                    and self.semimajor_axis_increase >= goal.minimum_semimajor_axis_increase_m)
        else:
            axis_error = abs(elements["semimajor_axis_m"]-RADIUS-goal.altitude_m) if elements["semimajor_axis_m"] else None
            in_region = (axis_error is not None and axis_error <= goal.semimajor_axis_tolerance_m
                         and elements["eccentricity"] <= goal.maximum_eccentricity
                         and plane_error <= goal.plane_tolerance_rad and coasting)
        if maintenance:
            if sample.time_ns >= round(goal.acquisition_deadline_s*1e9):
                self.maintenance_samples += 1
                self.failed_maintenance_samples += int(not in_region)
                self.maintenance_axis_error = max(self.maintenance_axis_error, axis_error or 0.)
                self.maintenance_eccentricity = max(self.maintenance_eccentricity, elements['eccentricity'])
                self.maintenance_plane_error = max(self.maintenance_plane_error, plane_error)
            else:
                in_region = False
        if in_region and not self.evidence_errors:
            if self.hold_start_ns is None:
                self.hold_start_ns = sample.time_ns
            dwell = sample.time_ns-self.hold_start_ns
            self.longest_hold_ns = max(self.longest_hold_ns, dwell)
            if dwell >= round(goal.dwell_s*1e9) and self.completion_time_ns is None:
                self.completion_time_ns = sample.time_ns
        else:
            self.hold_start_ns = None
        self.final = {key: value for key, value in elements.items() if key != "normal"}
        self.final.update(plane_error_rad=plane_error, impulse_vector_error_m_s=vector_error, semimajor_axis_error_m=axis_error)
        self.count += 1
        self.last_time_ns = sample.time_ns
        self.minimum_altitude = min(self.minimum_altitude, elements["altitude_m"])
        self.maximum_eccentricity = max(self.maximum_eccentricity, elements["eccentricity"])

    def result(self):
        maintenance = isinstance(self.task.objective, OrbitMaintenanceObjective)
        if not self.count or self.evidence_errors:
            outcome = "invalid_evidence"
        elif self.safety_violations:
            outcome = "safety_violation"
        elif maintenance and self.last_time_ns == round(self.task.duration_s*1e9) and self.failed_maintenance_samples:
            outcome = "task_failure"
        elif self.completion_time_ns is not None:
            outcome = "success"
        elif self.last_time_ns == round(self.task.duration_s*1e9):
            outcome = "time_limit"
        else:
            outcome = "incomplete"
        result = {"schema_version": "spacesim-autonomy-orbit-score/1", "task_fingerprint": self.task.fingerprint(),
                "outcome": outcome, "success": outcome == "success", "sample_count": self.count,
                "final_time_s": self.last_time_ns*1e-9 if self.count else None,
                "completion_time_s": self.completion_time_ns*1e-9 if outcome == "success" else None,
                "longest_dwell_s": self.longest_hold_ns*1e-9,
                "minimum_altitude_m": self.minimum_altitude if self.count else None,
                "maximum_eccentricity": self.maximum_eccentricity if self.count else None,
                "final_orbit": self.final, "thrust_impulse_n_s": self.total_impulse,
                "thrust_delta_v_m_s": self.total_impulse/self.mass,
                "delta_velocity_j2000_m_s": (self.impulse/self.mass).tolist(),
                "fuel_kg": None, "energy_j": None,
                "resource_model": "constant mass; 50 Hz native motor-force integral; fuel and electrical energy not modeled",
                "element_convention": self.task.element_convention, "earth_mu_m3_s2": MU, "earth_radius_m": RADIUS,
                "evidence_errors": sorted(self.evidence_errors), "safety_violations": sorted(self.safety_violations)}
        if maintenance:
            result.update(path_requirement='continuous_orbit_maintenance',
                maintenance_samples=self.maintenance_samples, failed_maintenance_samples=self.failed_maintenance_samples,
                maximum_maintenance_axis_error_m=self.maintenance_axis_error if self.maintenance_samples else None,
                maximum_maintenance_eccentricity=self.maintenance_eccentricity if self.maintenance_samples else None,
                maximum_maintenance_plane_error_rad=self.maintenance_plane_error if self.maintenance_samples else None,
                quality_violations=['continuous_orbit_band'] if self.failed_maintenance_samples else [])
        if isinstance(self.task.objective, LowThrustObjective):
            goal = self.task.objective
            quality = []
            if self.longest_active_thrust_ns < round(goal.minimum_continuous_thrust_s*1e9):
                quality.append('continuous_low_thrust_duration')
            if self.semimajor_axis_increase is None or self.semimajor_axis_increase < goal.minimum_semimajor_axis_increase_m:
                quality.append('low_thrust_orbital_energy_increase')
            if outcome == 'time_limit' and quality:
                result['outcome'] = 'task_failure'
            result.update(path_requirement='continuous_low_thrust', quality_violations=quality,
                longest_continuous_thrust_s=self.longest_active_thrust_ns*1e-9,
                semimajor_axis_increase_m=self.semimajor_axis_increase)
        return result
