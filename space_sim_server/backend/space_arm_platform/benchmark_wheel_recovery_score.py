"""Independent fault realization, actual motor loss, diagnosis and recovery grading."""
from __future__ import annotations

import math

from .benchmark_attitude import AttitudeTruthSample
from .benchmark_attitude_score import AttitudeScorer
from .benchmark_wheel_recovery import WheelRecoveryTruthSample


class WheelRecoveryScorer(AttitudeScorer):
    def __init__(self, task):
        super().__init__(task)
        self.fault = None
        self.fault_samples = 0
        self.lost_impulse = self.lost_impulse_before_deadline = 0.
        self.diagnosis_hold_ns = self.diagnosis_confirmed_ns = None
        self.nominal_error_start_ns = None
        self.false_alarm = False
        self.final_effectiveness_error = None

    def add(self, raw):
        try:
            sample = WheelRecoveryTruthSample.model_validate(raw)
            sample.fault.validate_for(self.task)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_motor_loss_evidence')
            raise ValueError('invalid motor loss evidence') from error
        if self.fault is None:
            self.fault = sample.fault
        elif self.fault != sample.fault:
            self.evidence_errors.add('motor_loss_realization_changed')
        goal, fault = self.task.objective, self.fault
        axis = 'xyz'.index(fault.axis)
        active = sample.time_ns > fault.onset_time_ns
        gains = [1., 1., 1.]
        if active:
            gains[axis] = fault.effectiveness
        if self.count == 0:
            dot = abs(sum(a*b for a, b in zip(sample.orientation_body_to_j2000_wxyz,
                        goal.orientation_body_to_j2000_wxyz, strict=True)))
            angle = 2*math.acos(min(1., dot))
            initial = self.task.initial
            if (not initial.rotation_min_rad-1e-8 <= angle <= initial.rotation_max_rad+1e-8
                    or any(abs(value) > initial.body_rate_half_width_rad_s+1e-12 for value in sample.angular_velocity_body_rad_s)
                    or any(abs(a-b) > 1e-10 for a, b in zip(sample.wheel_speed_rad_s, initial.wheel_speed_rad_s, strict=True))):
                self.evidence_errors.add('initial_state_outside_recovery_distribution')
            if any(abs(value) > 1e-12 for value in sample.requested_motor_torque_xyz_nm):
                self.evidence_errors.add('motor_request_before_first_observation')
        requested = sample.requested_motor_torque_xyz_nm
        if any(abs(value) > self.task.safety.maximum_motor_torque_nm+1e-12 for value in requested):
            self.evidence_errors.add('motor_request_exceeds_contract')
        # Task wheel limits are strictly below the existing hardware speed
        # guard. Loss must reach native joints, not just a declared fault flag.
        if any(abs(actual-command*gain) > 1e-9 for actual, command, gain in
               zip(sample.applied_motor_torque_xyz_nm, requested, gains, strict=True)):
            self.evidence_errors.add('native_motor_torque_does_not_match_declared_loss')
        expected_time = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        continuous = sample.time_ns == expected_time
        deadline = fault.onset_time_ns+round(goal.diagnosis_deadline_after_onset_s*1e9)
        if active:
            self.fault_samples += 1
            if continuous:
                lost = abs(requested[axis]-sample.applied_motor_torque_xyz_nm[axis])*self.task.physics_period_ns/1e9
                self.lost_impulse += lost
                if sample.time_ns <= deadline:
                    self.lost_impulse_before_deadline += lost
        error = max(abs(a-b) for a, b in zip(sample.estimated_effectiveness_xyz, gains, strict=True))
        correct = error <= goal.effectiveness_tolerance
        self.final_effectiveness_error = error
        if not active:
            if not correct and continuous:
                if self.nominal_error_start_ns is None:
                    self.nominal_error_start_ns = sample.time_ns
                if sample.time_ns-self.nominal_error_start_ns >= round(goal.diagnosis_dwell_s*1e9):
                    self.false_alarm = True
            else:
                self.nominal_error_start_ns = None
        if active and correct and continuous and not self.evidence_errors:
            if self.diagnosis_hold_ns is None:
                self.diagnosis_hold_ns = sample.time_ns
            if sample.time_ns-self.diagnosis_hold_ns >= round(goal.diagnosis_dwell_s*1e9) and self.diagnosis_confirmed_ns is None:
                self.diagnosis_confirmed_ns = sample.time_ns
        else:
            self.diagnosis_hold_ns = None
        super().add(sample.model_dump(include=set(AttitudeTruthSample.model_fields)))
        # Attitude recovery is a new continuous physical hold after an observed,
        # continuously correct diagnosis. Nominal pre-fault dwell cannot count.
        if not active or self.diagnosis_confirmed_ns is None or not correct:
            self.hold_start_ns = self.completion_time_ns = None
            self.longest_hold_ns = 0

    def result(self):
        result = super().result()
        quality = []
        if self.false_alarm:
            quality.append('false_motor_loss_diagnosis')
        if self.fault is not None:
            deadline = self.fault.onset_time_ns+round(self.task.objective.diagnosis_deadline_after_onset_s*1e9)
            if self.last_time_ns is not None and self.last_time_ns >= deadline:
                if self.diagnosis_confirmed_ns is None or self.diagnosis_confirmed_ns > deadline:
                    quality.append('motor_loss_diagnosis_deadline')
                if self.lost_impulse_before_deadline < self.task.objective.minimum_lost_motor_impulse_nm_s:
                    quality.append('motor_loss_not_observed_under_load')
        if result['outcome'] in {'success', 'time_limit'} and quality:
            result.update(outcome='task_failure', success=False, completion_time_s=None)
        result.update(schema_version='spacesim-autonomy-wheel-recovery-score/1',
            fault_triggered=self.fault_samples > 0, fault_samples=self.fault_samples,
            fault=self.fault.model_dump(mode='json') if self.fault else None,
            diagnosis_confirmed_s=self.diagnosis_confirmed_ns/1e9 if self.diagnosis_confirmed_ns is not None else None,
            diagnosis_latency_s=(self.diagnosis_confirmed_ns-self.fault.onset_time_ns)/1e9
                if self.diagnosis_confirmed_ns is not None else None,
            final_effectiveness_error=self.final_effectiveness_error,
            lost_motor_impulse_nm_s=self.lost_impulse,
            lost_motor_impulse_before_diagnosis_deadline_nm_s=self.lost_impulse_before_deadline,
            quality_violations=quality,
            evaluation_scope='actual single-wheel partial torque loss; observation-based diagnosis and post-diagnosis attitude hold')
        return result
