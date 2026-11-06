"""Independent rotor momentum reduction, physical external torque and coast dwell."""
from __future__ import annotations

import math

from .benchmark_attitude import AttitudeTruthSample
from .benchmark_attitude_score import AttitudeScorer
from .benchmark_momentum_unload import MomentumUnloadTruthSample


def norm(values):
    return math.sqrt(sum(value*value for value in values))


class MomentumUnloadScorer(AttitudeScorer):
    def __init__(self, task):
        super().__init__(task)
        self.initial_momentum = self._momentum(task.initial.wheel_speed_rad_s)
        self.final_momentum = self.final_wheel_speed = None
        self.external_impulse = self.external_peak = self.external_norm = 0.
        self.external_impulse_xyz = [0., 0., 0.]

    def _momentum(self, speeds):
        return norm(i*w for i, w in zip(self.task.wheel_spin_inertia_kg_m2, speeds, strict=True))

    def add(self, raw):
        try:
            sample = MomentumUnloadTruthSample.model_validate(raw)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_external_torque_evidence')
            raise ValueError('invalid unloading physical evidence') from error
        if self.count == 0:
            initial, goal = self.task.initial, self.task.objective
            dot = abs(sum(a*b for a, b in zip(sample.orientation_body_to_j2000_wxyz,
                goal.orientation_body_to_j2000_wxyz, strict=True)))
            angle = 2*math.acos(min(1., dot))
            if (not initial.rotation_min_rad-1e-8 <= angle <= initial.rotation_max_rad+1e-8
                    or any(abs(value) > initial.body_rate_half_width_rad_s+1e-12 for value in sample.angular_velocity_body_rad_s)
                    or any(abs(a-b) > 1e-10 for a, b in zip(sample.wheel_speed_rad_s, initial.wheel_speed_rad_s, strict=True))):
                self.evidence_errors.add('initial_state_outside_unloading_distribution')
            if norm(sample.requested_external_torque_body_nm) > 1e-12 or norm(sample.applied_motor_torque_xyz_nm) > 1e-12:
                self.evidence_errors.add('actuation_before_first_observation')
        self.external_norm = norm(sample.applied_external_torque_body_nm)
        self.external_peak = max(self.external_peak, self.external_norm)
        if self.external_norm > self.task.safety.maximum_external_torque_norm_nm+1e-12:
            self.safety_violations.add('external_torque_norm')
        if norm(sample.requested_external_torque_body_nm) > self.task.safety.maximum_external_torque_norm_nm+1e-12:
            self.evidence_errors.add('external_torque_request_exceeds_contract')
        if any(abs(a-b) > 1e-10 for a, b in zip(sample.applied_external_torque_body_nm,
                sample.requested_external_torque_body_nm, strict=True)):
            self.evidence_errors.add('external_torque_readback_differs_from_ideal_actuator')
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        if sample.time_ns != 0 and sample.time_ns == expected:
            seconds = self.task.physics_period_ns/1e9
            self.external_impulse += self.external_norm*seconds
            for axis, value in enumerate(sample.applied_external_torque_body_nm):
                self.external_impulse_xyz[axis] += abs(value)*seconds
        self.final_momentum = self._momentum(sample.wheel_speed_rad_s)
        self.final_wheel_speed = max(abs(value) for value in sample.wheel_speed_rad_s)
        super().add(sample.model_dump(include=set(AttitudeTruthSample.model_fields)))

    def _in_goal_region(self, sample, angle, rate):
        goal = self.task.objective
        return (super()._in_goal_region(sample, angle, rate)
            and self.final_wheel_speed <= goal.maximum_wheel_speed_rad_s
            and self.initial_momentum-self.final_momentum >= goal.minimum_wheel_momentum_reduction_nm_s
            and self.external_impulse >= goal.minimum_external_torque_impulse_nm_s
            and self.external_norm <= goal.maximum_coast_external_torque_norm_nm)

    def result(self):
        result = super().result()
        quality = []
        if self.final_wheel_speed is not None:
            goal = self.task.objective
            if self.final_wheel_speed > goal.maximum_wheel_speed_rad_s:
                quality.append('wheel_speed_not_unloaded')
            if self.initial_momentum-self.final_momentum < goal.minimum_wheel_momentum_reduction_nm_s:
                quality.append('insufficient_stored_wheel_momentum_reduction')
            if self.external_impulse < goal.minimum_external_torque_impulse_nm_s:
                quality.append('insufficient_actual_external_torque_impulse')
            if self.external_norm > goal.maximum_coast_external_torque_norm_nm:
                quality.append('external_torque_still_active')
        if result['outcome'] == 'time_limit' and quality:
            result.update(outcome='task_failure', success=False)
        result.update(schema_version='spacesim-autonomy-momentum-unload-score/1',
            initial_wheel_momentum_norm_nm_s=self.initial_momentum,
            final_wheel_momentum_norm_nm_s=self.final_momentum,
            wheel_momentum_norm_reduction_nm_s=None if self.final_momentum is None else self.initial_momentum-self.final_momentum,
            final_maximum_wheel_speed_rad_s=self.final_wheel_speed,
            external_torque_norm_impulse_nm_s=self.external_impulse,
            absolute_external_torque_impulse_xyz_nm_s=self.external_impulse_xyz.copy(),
            maximum_external_torque_norm_nm=self.external_peak,
            final_external_torque_norm_nm=self.external_norm, task_quality_failures=quality,
            resource_model='native wheel and ideal pure external torque integrals; no thruster geometry, fuel or electrical energy model')
        return result
