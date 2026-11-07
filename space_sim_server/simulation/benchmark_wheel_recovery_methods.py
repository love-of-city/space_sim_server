"""Causal motor-telemetry diagnosis and bounded gain-compensated attitude control."""
from collections import deque

import numpy as np

from simulation.benchmark_attitude_methods import RateLimitedAttitudePD
from space_arm_platform.benchmark_wheel_recovery import WheelRecoveryAction


class AdaptiveWheelRecovery(RateLimitedAttitudePD):
    def reset(self, context):
        super().reset(context)
        self.gains = np.ones(3)
        self.history = deque(maxlen=20)
        self.previous_command = None
        self.previous_time_ns = None
        self.probe_stop_time_ns = None

    def act(self, observation):
        if self.previous_time_ns is not None and observation.sample_time_ns != self.previous_time_ns+self.context.task.decision_period_ns:
            self.history.clear()
            self.previous_command = None
        if self.previous_command is not None:
            self.history.append((self.previous_command.copy(), np.asarray(observation.measured_motor_torque_xyz_nm)))
            for axis in range(3):
                pairs = [(command[axis], measured[axis]) for command, measured in self.history if abs(command[axis]) >= .025]
                if len(pairs) >= 5:
                    commands, measured = np.asarray(pairs).T
                    # Least squares through the origin over the last 0.4 s.
                    # No realized fault axis, onset, effectiveness or score is read.
                    self.gains[axis] = np.clip(commands@measured/(commands@commands), .1, 1.)
        desired = np.asarray(super().act(observation).motor_torque_xyz_nm)
        limit = self.context.task.safety.maximum_motor_torque_nm
        # Quiet axes otherwise have no usable input/output excitation. Use a
        # bounded zero-mean motor probe, independent of the hidden fault draw.
        # Continue for one second after observed loss so its physical effect is
        # measurable, then remove it for the final attitude/rate dwell.
        if self.probe_stop_time_ns is None and np.min(self.gains) < .8:
            self.probe_stop_time_ns = observation.sample_time_ns+1_000_000_000
        if self.probe_stop_time_ns is None or observation.sample_time_ns < self.probe_stop_time_ns:
            sign = 1. if (observation.sample_time_ns//100_000_000)%2 == 0 else -1.
            desired = desired+min(.05, limit/4)*sign*np.array([1., -1., 1.])
        command = np.clip(desired/self.gains, -limit, limit)
        self.previous_command = command
        self.previous_time_ns = observation.sample_time_ns
        return WheelRecoveryAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            motor_torque_xyz_nm=tuple(command), estimated_effectiveness_xyz=tuple(self.gains))


class UnawareWheelRecovery(RateLimitedAttitudePD):
    def act(self, observation):
        action = super().act(observation)
        return WheelRecoveryAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            motor_torque_xyz_nm=action.motor_torque_xyz_nm, estimated_effectiveness_xyz=(1., 1., 1.))


class NoRecoveryTorque(UnawareWheelRecovery):
    def act(self, observation):
        return WheelRecoveryAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            motor_torque_xyz_nm=(0., 0., 0.), estimated_effectiveness_xyz=(1., 1., 1.))


class UnsafeRecoverySpin(UnawareWheelRecovery):
    def act(self, observation):
        limit = self.context.task.safety.maximum_motor_torque_nm
        return WheelRecoveryAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            motor_torque_xyz_nm=(limit, limit, limit), estimated_effectiveness_xyz=(1., 1., 1.))


METHODS = {'adaptive-wheel-recovery-v1': AdaptiveWheelRecovery,
           'unaware-wheel-recovery-v1': UnawareWheelRecovery,
           'no-recovery-torque-v1': NoRecoveryTorque,
           'unsafe-recovery-spin-v1': UnsafeRecoverySpin}
