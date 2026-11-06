"""Independent quaternion/rate dwell grading, reading only raw native evidence."""
from __future__ import annotations

import math

from .benchmark_attitude import AttitudeSequenceObjective, AttitudeTask, AttitudeTruthSample, DetumbleInitialState


class AttitudeScorer:
    def __init__(self, task: AttitudeTask):
        self.task = task
        self.last_time_ns = None
        self.count = 0
        self.hold_start_ns = None
        self.longest_hold_ns = 0
        self.completion_time_ns = None
        self.final_angle = None
        self.final_rate = None
        self.peak_rate = 0.
        self.peak_wheel_speed = 0.
        self.peak_torque = 0.
        self.motor_impulse = [0., 0., 0.]
        self.evidence_errors = set()
        self.safety_violations = set()
        self.targets = (task.objective.orientation_body_to_j2000_wxyz,)
        if isinstance(task.objective, AttitudeSequenceObjective):
            self.targets += task.objective.additional_orientations_body_to_j2000_wxyz
        self.target_index = 0
        self.completed_targets = []

    def add(self, raw: AttitudeTruthSample | dict):
        try:
            sample = AttitudeTruthSample.model_validate(raw)
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
        # q and -q describe the same physical orientation. This computation is
        # independent of the controller's quaternion product/MRP error code.
        graded_target_index = self.target_index
        dot = sum(a*b for a, b in zip(sample.orientation_body_to_j2000_wxyz,
                                     self.targets[self.target_index], strict=True))
        angle = 2 * math.acos(min(1., abs(dot)))
        rate = math.sqrt(sum(v*v for v in sample.angular_velocity_body_rad_s))
        wheel_speed = max(abs(v) for v in sample.wheel_speed_rad_s)
        torque = max(abs(v) for v in sample.applied_motor_torque_xyz_nm)
        initial = self.task.initial
        if self.count == 0 and len(self.targets) > 1:
            if (not initial.rotation_min_rad-1e-8 <= angle <= initial.rotation_max_rad+1e-8
                    or any(abs(v) > initial.body_rate_half_width_rad_s+1e-12 for v in sample.angular_velocity_body_rad_s)
                    or any(abs(a-b) > 1e-10 for a,b in zip(sample.wheel_speed_rad_s, initial.wheel_speed_rad_s, strict=True))):
                self.evidence_errors.add("initial_state_outside_attitude_sequence_distribution")
        if self.count == 0 and isinstance(initial, DetumbleInitialState):
            if (rate < initial.body_rate_min_norm_rad_s - 1e-12
                    or any(abs(v) > initial.body_rate_half_width_rad_s + 1e-12 for v in sample.angular_velocity_body_rad_s)):
                self.evidence_errors.add("initial_body_rate_outside_detumble_distribution")
            if not initial.rotation_min_rad - 1e-8 <= angle <= initial.rotation_max_rad + 1e-8:
                self.evidence_errors.add("initial_attitude_outside_detumble_distribution")
            if any(abs(actual-wanted) > 1e-10 for actual, wanted in zip(sample.wheel_speed_rad_s, initial.wheel_speed_rad_s, strict=True)):
                self.evidence_errors.add("initial_wheels_outside_detumble_distribution")
        limits = self.task.safety
        if rate > limits.maximum_body_rate_rad_s:
            self.safety_violations.add("body_rate")
        if wheel_speed > limits.maximum_wheel_speed_rad_s:
            self.safety_violations.add("wheel_speed")
        if torque > limits.maximum_motor_torque_nm + 1e-12:
            self.safety_violations.add("motor_torque")
        if sample.time_ns != 0 and sample.time_ns == expected:
            for i, value in enumerate(sample.applied_motor_torque_xyz_nm):
                self.motor_impulse[i] += abs(value) * self.task.physics_period_ns * 1e-9
        in_region = angle <= self.task.objective.angle_tolerance_rad and rate <= self.task.objective.body_rate_tolerance_rad_s
        if in_region and not self.evidence_errors:
            if self.hold_start_ns is None:
                self.hold_start_ns = sample.time_ns
            dwell = sample.time_ns - self.hold_start_ns
            self.longest_hold_ns = max(self.longest_hold_ns, dwell)
            if dwell >= round(self.task.objective.dwell_s * 1e9) and self.completion_time_ns is None:
                if len(self.targets) > 1:
                    self.completed_targets.append(dict(target_index=self.target_index,
                        hold_start_s=self.hold_start_ns/1e9, completed_at_s=sample.time_ns/1e9,
                        continuous_dwell_s=dwell/1e9))
                if self.target_index+1 == len(self.targets):
                    self.completion_time_ns = sample.time_ns
                else:
                    self.target_index += 1
                    # This physical sample belongs to the target just completed.
                    # The next target can only start accumulating on a later tick.
                    self.hold_start_ns = None
        else:
            self.hold_start_ns = None
        self.count += 1
        self.last_time_ns = sample.time_ns
        if self.target_index != graded_target_index:
            dot = sum(a*b for a,b in zip(sample.orientation_body_to_j2000_wxyz, self.targets[self.target_index], strict=True))
            angle = 2*math.acos(min(1., abs(dot)))
        self.final_angle, self.final_rate = angle, rate
        self.peak_rate = max(self.peak_rate, rate)
        self.peak_wheel_speed = max(self.peak_wheel_speed, wheel_speed)
        self.peak_torque = max(self.peak_torque, torque)

    def result(self):
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
        result = {"schema_version": "spacesim-autonomy-attitude-score/1", "task_fingerprint": self.task.fingerprint(),
                "outcome": outcome, "success": outcome == "success", "sample_count": self.count,
                "final_time_s": None if self.last_time_ns is None else self.last_time_ns / 1e9,
                "completion_time_s": self.completion_time_ns / 1e9 if outcome == "success" else None,
                "longest_dwell_s": self.longest_hold_ns / 1e9,
                "final_attitude_error_rad": self.final_angle, "final_body_rate_rad_s": self.final_rate,
                "maximum_body_rate_rad_s": self.peak_rate, "maximum_wheel_speed_rad_s": self.peak_wheel_speed,
                "maximum_motor_torque_nm": self.peak_torque, "absolute_motor_impulse_xyz_nm_s": self.motor_impulse.copy(),
                "fuel_kg": None, "energy_j": None,
                "resource_model": "reaction-wheel motor torque integrals; electrical losses and energy not modeled",
                "safety_sampling": "50 Hz physical body rates, rotor speeds and native motor torque readbacks",
                "evidence_errors": sorted(self.evidence_errors), "safety_violations": sorted(self.safety_violations)}
        if len(self.targets) > 1:
            result.update(target_count=len(self.targets), completed_targets=list(self.completed_targets),
                          current_target_index=self.target_index)
        return result
