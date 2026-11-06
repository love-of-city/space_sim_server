"""Independent continuous pointing grade from physical COM, attitude and ephemeris."""
from __future__ import annotations

import math
import numpy as np

from .benchmark_pointing import PointingTruthSample


class PointingScorer:
    def __init__(self, task):
        self.task = task
        self.last_time_ns = None
        self.count = self.window_samples = self.failed_window_samples = 0
        self.final_angle = self.final_tracking_rate = None
        self.window_angle = self.window_rate = self.peak_rate = self.peak_wheel = self.peak_torque = 0.
        self.minimum_elevation = None
        self.first_good_s = None
        self.impulse = [0., 0., 0.]
        self.evidence_errors, self.safety_violations, self.precision_violations = set(), set(), set()

    def add(self, raw):
        try:
            sample = PointingTruthSample.model_validate(raw)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add("invalid_physical_sample")
            raise ValueError("invalid pointing physical evidence") from error
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        if sample.time_ns != expected:
            self.evidence_errors.add("missing_duplicate_or_reordered_physics_sample")
        if sample.time_ns > round(self.task.duration_s*1e9):
            self.evidence_errors.add("evidence_beyond_episode_time_limit")
        if sample.external_state_writes:
            self.evidence_errors.add("physical_state_overwritten_after_initialization")
        state, goal, ephemeris = sample.chaser_com_j2000, self.task.objective, sample.ephemeris
        position, velocity = np.asarray(state[:3]), np.asarray(state[3:])
        if np.linalg.norm(position) <= 6378136.6:
            self.safety_violations.add("earth_surface_intersection")
        elevation, occulted = None, False
        if goal.kind == "inertial_direction":
            desired = np.asarray(goal.direction_j2000_unit)
            desired_rate = np.zeros(3)
        else:
            if goal.kind == "earth_center":
                destination, destination_velocity = np.zeros(3), np.zeros(3)
            elif goal.kind == "sun":
                destination = np.asarray(ephemeris.sun_geocentric_j2000[:3])
                destination_velocity = np.asarray(ephemeris.sun_geocentric_j2000[3:])
            else:
                fixed = 6378136.6*np.asarray(goal.ground_direction_iau_earth_unit)
                matrix, derivative = np.asarray(ephemeris.earth_j2000_to_fixed), np.asarray(ephemeris.earth_j2000_to_fixed_rate_s)
                destination = matrix.T@fixed
                # A fixed surface point has v_N = Cdot_PN.T r_P.
                # The policy uses the equivalent -C.T Cdot r_N expression.
                destination_velocity = derivative.T@fixed
            offset = destination-position
            distance = np.linalg.norm(offset)
            if distance < 1.:
                self.evidence_errors.add("undefined_target_line_of_sight")
                return
            desired = offset/distance
            relative_velocity = destination_velocity-velocity
            desired_rate = (np.eye(3)-np.outer(desired, desired))@relative_velocity/distance
            if goal.kind == "ground_site":
                elevation = math.asin(float(np.clip(np.dot(destination/6378136.6, -desired), -1., 1.)))
                self.minimum_elevation = elevation if self.minimum_elevation is None else min(self.minimum_elevation, elevation)
                if self.count == 0 and elevation < goal.minimum_elevation_rad:
                    self.evidence_errors.add("ground_target_not_initially_visible")
            if goal.kind == "sun":
                along = float(-position@desired)
                occulted = 0 < along < distance and np.linalg.norm(position+along*desired) <= 6378136.6
        w, x, y, z = sample.orientation_body_to_j2000_wxyz
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        boresight = rotation[:, 2]
        angle = math.atan2(float(np.linalg.norm(np.cross(boresight, desired))), float(boresight@desired))
        omega = np.asarray(sample.angular_velocity_body_rad_s)
        pointing_rate = np.cross(rotation@omega, boresight)
        tracking_rate = float(np.linalg.norm(pointing_rate-desired_rate))
        rate = float(np.linalg.norm(omega))
        wheel, torque = max(abs(v) for v in sample.wheel_speed_rad_s), max(abs(v) for v in sample.applied_motor_torque_xyz_nm)
        limits, initial = self.task.safety, self.task.initial
        if self.count == 0:
            if (not initial.rotation_min_rad-1e-8 <= angle <= initial.rotation_max_rad+1e-8
                    or any(abs(v) > initial.body_rate_half_width_rad_s+1e-12 for v in omega)
                    or any(abs(a-b) > 1e-10 for a,b in zip(sample.wheel_speed_rad_s, initial.wheel_speed_rad_s, strict=True))):
                self.evidence_errors.add("initial_state_outside_pointing_distribution")
            if abs(np.linalg.norm(position)-(6378136.6+initial.altitude_m)) > .001:
                self.evidence_errors.add("initial_altitude_outside_pointing_distribution")
        if rate > limits.maximum_body_rate_rad_s:
            self.safety_violations.add("body_rate")
        if wheel > limits.maximum_wheel_speed_rad_s:
            self.safety_violations.add("wheel_speed")
        if torque > limits.maximum_motor_torque_nm+1e-12:
            self.safety_violations.add("motor_torque")
        if sample.time_ns and sample.time_ns == expected:
            for i, value in enumerate(sample.applied_motor_torque_xyz_nm):
                self.impulse[i] += abs(value)*self.task.physics_period_ns*1e-9
        errors = set()
        if angle > goal.angle_tolerance_rad:
            errors.add("boresight_error")
        if tracking_rate > goal.direction_rate_tolerance_rad_s:
            errors.add("direction_rate_error")
        if elevation is not None and elevation < goal.minimum_elevation_rad:
            errors.add("ground_target_below_elevation_limit")
        if occulted:
            errors.add("sun_occulted_by_earth")
        if not errors and self.first_good_s is None:
            self.first_good_s = sample.time_ns/1e9
        if sample.time_ns >= round(goal.acquisition_deadline_s*1e9):
            self.window_samples += 1
            self.failed_window_samples += bool(errors)
            self.precision_violations.update(errors)
            self.window_angle, self.window_rate = max(self.window_angle, angle), max(self.window_rate, tracking_rate)
        self.count += 1
        self.last_time_ns = sample.time_ns
        self.final_angle, self.final_tracking_rate = angle, tracking_rate
        self.peak_rate, self.peak_wheel, self.peak_torque = max(self.peak_rate, rate), max(self.peak_wheel, wheel), max(self.peak_torque, torque)

    def result(self):
        if not self.count or self.evidence_errors:
            outcome = "invalid_evidence"
        elif self.safety_violations:
            outcome = "safety_violation"
        elif self.last_time_ns == round(self.task.duration_s*1e9):
            expected = round((self.task.duration_s-self.task.objective.acquisition_deadline_s)*1e9/self.task.physics_period_ns)+1
            outcome = "success" if self.window_samples == expected and not self.failed_window_samples else "time_limit"
        else:
            outcome = "incomplete"
        return dict(schema_version="spacesim-autonomy-pointing-score/1", task_fingerprint=self.task.fingerprint(),
            outcome=outcome, success=outcome == "success", sample_count=self.count,
            final_time_s=None if self.last_time_ns is None else self.last_time_ns/1e9,
            completion_time_s=self.task.duration_s if outcome == "success" else None,
            required_tracking_start_s=self.task.objective.acquisition_deadline_s,
            required_tracking_end_s=self.task.duration_s, tracking_samples=self.window_samples,
            failed_tracking_samples=self.failed_window_samples, first_within_tolerance_s=self.first_good_s,
            final_boresight_error_rad=self.final_angle, final_direction_rate_error_rad_s=self.final_tracking_rate,
            maximum_tracking_boresight_error_rad=self.window_angle if self.window_samples else None,
            maximum_tracking_direction_rate_error_rad_s=self.window_rate if self.window_samples else None,
            minimum_ground_elevation_rad=self.minimum_elevation,
            maximum_body_rate_rad_s=self.peak_rate, maximum_wheel_speed_rad_s=self.peak_wheel,
            maximum_motor_torque_nm=self.peak_torque, absolute_motor_impulse_xyz_nm_s=self.impulse.copy(),
            fuel_kg=None, energy_j=None,
            resource_model="native wheel torque integrals; no electrical or solar power model",
            safety_sampling="50 Hz physical attitude, COM, rotor and native motor evidence; 400 Hz dynamics",
            evidence_errors=sorted(self.evidence_errors), safety_violations=sorted(self.safety_violations),
            quality_violations=sorted(self.precision_violations))
