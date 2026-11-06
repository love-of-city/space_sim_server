"""S22: diagnose motor effectiveness loss and recover a physical attitude maneuver."""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .benchmark_attitude import (
    AttitudeContext, AttitudeInitialState, AttitudeObjective, AttitudeObservation,
    AttitudeObservationModel, AttitudeTask, AttitudeTruthSample, WheelTorqueAction,
    validate_attitude_limits,
)
from .benchmark_contract import Contract, Count, Number, Positive, Vector3

Effectiveness = Annotated[Number, Field(gt=0, le=1)]
Effectiveness3 = tuple[Effectiveness, Effectiveness, Effectiveness]


class MotorLossDistribution(Contract):
    kind: Literal['permanent_single_wheel_motor_effectiveness_loss']
    onset_earliest_s: Positive
    onset_latest_s: Positive
    axes: tuple[Literal['x', 'y', 'z'], ...] = Field(min_length=1, max_length=3)
    effectiveness_min: Annotated[Number, Field(ge=.1, le=.8)]
    effectiveness_max: Annotated[Number, Field(ge=.1, le=.8)]

    @model_validator(mode='after')
    def ordered(self):
        if (self.onset_earliest_s > self.onset_latest_s or self.effectiveness_min > self.effectiveness_max
                or len(set(self.axes)) != len(self.axes)):
            raise ValueError('motor-loss distribution bounds must be ordered and axes unique')
        return self


class RecoveryObjective(AttitudeObjective):
    diagnosis_deadline_after_onset_s: Positive
    diagnosis_dwell_s: Positive
    effectiveness_tolerance: Annotated[Number, Field(gt=0, lt=.1)]
    minimum_lost_motor_impulse_nm_s: Positive


class RecoveryObservationModel(AttitudeObservationModel):
    motor_torque_telemetry: Literal['noisy_native_motor_torque_readback']
    motor_torque_noise_std_nm: Positive


class WheelRecoveryTask(AttitudeTask):
    schema_version: Literal['spacesim-autonomy-wheel-recovery/1']
    requirement_id: Literal['ssb.recovery.actuator-reconfigure']
    source_id: Literal['S22']
    backend: Literal['native_team_single_wheel_recovery_v1']
    action_type: Literal['wheel_motor_torques_and_effectiveness_estimate']
    initial: AttitudeInitialState
    objective: RecoveryObjective
    observation: RecoveryObservationModel
    fault: MotorLossDistribution

    @model_validator(mode='after')
    def feasible_contract(self):
        # Overrides the ordinary S01--S04 semantic validator; physical limits
        # and all inherited strict clock/boolean validators remain shared.
        validate_attitude_limits(self)
        fault, goal = self.fault, self.objective
        if not goal.diagnosis_dwell_s < goal.diagnosis_deadline_after_onset_s:
            raise ValueError('continuous diagnosis must fit before its deadline')
        if fault.onset_latest_s+goal.diagnosis_deadline_after_onset_s+goal.dwell_s >= self.duration_s:
            raise ValueError('fault, diagnosis and post-diagnosis hold must fit before the deadline')
        if self.safety.maximum_wheel_speed_rad_s > 500.:
            raise ValueError('recovery evidence requires a wheel bound below the original hardware speed guard')
        if goal.effectiveness_tolerance >= (1-fault.effectiveness_max)/2:
            raise ValueError('diagnosis must distinguish the loss from nominal effectiveness')
        if goal.minimum_lost_motor_impulse_nm_s >= ((1-fault.effectiveness_max)
                * self.safety.maximum_motor_torque_nm*goal.diagnosis_deadline_after_onset_s):
            raise ValueError('required loaded fault cannot exceed the available lost impulse')
        for seconds in (fault.onset_earliest_s, fault.onset_latest_s, goal.diagnosis_dwell_s,
                        goal.diagnosis_deadline_after_onset_s):
            ticks = seconds*1e9/self.physics_period_ns
            if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError('fault and diagnosis times must lie on the physics grid')
        return self


class WheelRecoveryObservation(AttitudeObservation):
    schema_version: Literal['spacesim-autonomy-wheel-recovery-observation/1'] = 'spacesim-autonomy-wheel-recovery-observation/1'
    # Measurement of the just-completed physical interval. It contains neither
    # the hidden fault realization nor the next interval's torque.
    measured_motor_torque_xyz_nm: Vector3


class WheelRecoveryAction(WheelTorqueAction):
    schema_version: Literal['spacesim-autonomy-wheel-recovery-action/1'] = 'spacesim-autonomy-wheel-recovery-action/1'
    estimated_effectiveness_xyz: Effectiveness3

    def validate_for(self, observation, limit_nm):
        action = type(self).model_validate(self)
        if action.episode_id != observation.episode_id or action.observation_sequence != observation.sequence:
            raise ValueError('stale action: episode or observation identity does not match')
        if any(abs(torque) > limit_nm+1e-12 for torque in action.motor_torque_xyz_nm):
            raise ValueError('requested wheel motor torque exceeds the action contract')


class MotorLossRealization(Contract):
    """Private evidence only; onset is the boundary before the first affected interval."""
    onset_time_ns: Count
    axis: Literal['x', 'y', 'z']
    effectiveness: Effectiveness

    def validate_for(self, task):
        distribution = task.fault
        if (self.onset_time_ns % task.physics_period_ns
                or not round(distribution.onset_earliest_s*1e9) <= self.onset_time_ns <= round(distribution.onset_latest_s*1e9)
                or self.axis not in distribution.axes
                or not distribution.effectiveness_min <= self.effectiveness <= distribution.effectiveness_max):
            raise ValueError('private motor loss lies outside the declared distribution')


class WheelRecoveryTruthSample(AttitudeTruthSample):
    fault: MotorLossRealization
    requested_motor_torque_xyz_nm: Vector3
    estimated_effectiveness_xyz: Effectiveness3


class WheelRecoveryContext(AttitudeContext):
    task: WheelRecoveryTask
