"""Native reaction-wheel loss at the existing motor drive; no body-state edits."""
from __future__ import annotations

import numpy as np

from simulation.attitude_control import WheelDrive
from simulation.benchmark_native_attitude import NativeAttitudeEnvironment
from space_arm_platform.benchmark_wheel_recovery import (
    MotorLossRealization, WheelRecoveryContext, WheelRecoveryObservation, WheelRecoveryTruthSample,
)
from space_arm_platform.benchmark_wheel_recovery_score import WheelRecoveryScorer


class _LossWheelDrive(WheelDrive):
    def __init__(self, environment, index, message):
        super().__init__(environment._wheel_owner, index, message)
        self.environment = environment

    def UpdateState(self, nanos):
        super().UpdateState(nanos)
        fault = self.environment._fault
        if nanos > fault.onset_time_ns and self.index == 'xyz'.index(fault.axis):
            self.applied *= fault.effectiveness
            self._payload.input = self.applied
            self._actuator_writer(self._payload, self.moduleID, nanos)


class NativeWheelRecoveryEnvironment(NativeAttitudeEnvironment):
    context_type = WheelRecoveryContext
    scorer_type = WheelRecoveryScorer

    def _make_wheel_drive(self, index, message):
        return _LossWheelDrive(self, index, message)

    def _setup_additional_observation(self, ctx, seed):
        fault = self.task.fault
        rng = np.random.default_rng(np.random.SeedSequence([seed, 7]))
        period = self.task.physics_period_ns
        first, last = (round(seconds*1e9/period) for seconds in (fault.onset_earliest_s, fault.onset_latest_s))
        self._fault = MotorLossRealization(onset_time_ns=int(rng.integers(first, last+1))*period,
            axis=str(rng.choice(fault.axes)), effectiveness=float(rng.uniform(fault.effectiveness_min, fault.effectiveness_max)))
        self._fault.validate_for(self.task)
        self._torque_sensor_rng = np.random.default_rng(np.random.SeedSequence([seed, 8]))
        self._requested_torques = (0., 0., 0.)
        self._reported_effectiveness = (1., 1., 1.)
        if any(self.task.safety.maximum_wheel_speed_rad_s >= .9*limit for limit in self.context.hardware_speed_limits_rad_s):
            raise ValueError('recovery task wheel bound must be safely below the original hardware guard')

    def _truth_sample(self, nanos):
        physical = super()._truth_sample(nanos).model_dump()
        return WheelRecoveryTruthSample(**physical, fault=self._fault,
            requested_motor_torque_xyz_nm=self._requested_torques,
            estimated_effectiveness_xyz=self._reported_effectiveness)

    def _measure(self, sequence):
        measured = super()._measure(sequence).model_dump(exclude={'schema_version'})
        noise = self._torque_sensor_rng.normal(0., self.task.observation.motor_torque_noise_std_nm, 3)
        torque = np.asarray([self._stepper.actuator_force[i] for i in self._motor_indices])+noise
        return WheelRecoveryObservation(**measured, measured_motor_torque_xyz_nm=tuple(torque))

    def step(self, action):
        if self._observation is None:
            raise RuntimeError('reset must precede step')
        action.validate_for(self._observation, self.task.safety.maximum_motor_torque_nm)
        self._requested_torques = action.motor_torque_xyz_nm
        self._reported_effectiveness = action.estimated_effectiveness_xyz
        return super().step(action)

    def provenance(self):
        result = super().provenance()
        result.update(fault=self._fault.model_dump(mode='json'),
            observation_model='noisy attitude, gyro and native motor torque telemetry; ideal wheel encoders; no fault flag or realized schedule',
            actuation_model='original speed-protected wheel joint motors; one persistent multiplicative output loss after the sampled onset')
        return result
