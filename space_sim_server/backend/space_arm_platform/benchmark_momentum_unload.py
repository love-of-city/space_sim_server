"""S08: unload stored wheel momentum with an explicit external torque actuator."""
from __future__ import annotations

import math
from typing import Literal

from pydantic import model_validator

from .benchmark_attitude import (
    AttitudeContext, AttitudeInitialState, AttitudeObjective, AttitudeSafety,
    AttitudeTask, AttitudeTruthSample, WheelTorqueAction, validate_attitude_limits,
)
from .benchmark_contract import Positive, Vector3


class UnloadObjective(AttitudeObjective):
    maximum_wheel_speed_rad_s: Positive
    minimum_wheel_momentum_reduction_nm_s: Positive
    minimum_external_torque_impulse_nm_s: Positive
    maximum_coast_external_torque_norm_nm: Positive


class UnloadSafety(AttitudeSafety):
    maximum_external_torque_norm_nm: Positive


class MomentumUnloadTask(AttitudeTask):
    schema_version: Literal['spacesim-autonomy-momentum-unload/1']
    requirement_id: Literal['ssb.attitude.momentum-unload']
    source_id: Literal['S08']
    backend: Literal['native_team_single_momentum_unload_v1']
    action_type: Literal['wheel_motor_torques_and_external_body_torque']
    initial: AttitudeInitialState
    objective: UnloadObjective
    safety: UnloadSafety
    external_torque_model: Literal['ideal_pure_body_torque_site_motors']
    wheel_spin_inertia_kg_m2: Vector3

    @model_validator(mode='after')
    def feasible_contract(self):
        validate_attitude_limits(self)
        goal, safety = self.objective, self.safety
        if self.wheel_spin_inertia_kg_m2 != (.01, .01, .01):
            raise ValueError('unloading uses the three original SARM wheel spin inertias')
        if not 0 < goal.maximum_wheel_speed_rad_s < .7*safety.maximum_wheel_speed_rad_s:
            raise ValueError('wheel goal must restore margin below the unloading start threshold')
        if any(abs(value) < .7*safety.maximum_wheel_speed_rad_s for value in self.initial.wheel_speed_rad_s):
            raise ValueError('every initial wheel must require momentum unloading')
        if not goal.maximum_coast_external_torque_norm_nm < safety.maximum_external_torque_norm_nm <= .02:
            raise ValueError('external torque must respect the existing 0.02 N m unloading model')
        initial_momentum = math.sqrt(sum((i*w)**2 for i, w in zip(
            self.wheel_spin_inertia_kg_m2, self.initial.wheel_speed_rad_s, strict=True)))
        if goal.minimum_wheel_momentum_reduction_nm_s >= initial_momentum:
            raise ValueError('required unloading cannot exceed the initial stored wheel momentum')
        if goal.minimum_external_torque_impulse_nm_s >= safety.maximum_external_torque_norm_nm*(self.duration_s-goal.dwell_s):
            raise ValueError('external torque impulse and coast hold must fit before the deadline')
        return self


class MomentumUnloadAction(WheelTorqueAction):
    schema_version: Literal['spacesim-autonomy-momentum-unload-action/1'] = 'spacesim-autonomy-momentum-unload-action/1'
    external_torque_body_nm: Vector3

    def validate_for(self, observation, task):
        action = type(self).model_validate(self)
        wheel = WheelTorqueAction(**action.model_dump(exclude={'schema_version', 'external_torque_body_nm'}))
        wheel.validate_for(observation, task.safety.maximum_motor_torque_nm)
        if math.sqrt(sum(value*value for value in action.external_torque_body_nm)) > task.safety.maximum_external_torque_norm_nm+1e-12:
            raise ValueError('external torque vector exceeds the action norm contract')


class MomentumUnloadTruthSample(AttitudeTruthSample):
    requested_external_torque_body_nm: Vector3
    applied_external_torque_body_nm: Vector3


class MomentumUnloadContext(AttitudeContext):
    task: MomentumUnloadTask

    @model_validator(mode='after')
    def original_inertia(self):
        if self.wheel_spin_inertia_kg_m2 != self.task.wheel_spin_inertia_kg_m2:
            raise ValueError('method hardware context differs from declared physical wheel inertia')
        return self
