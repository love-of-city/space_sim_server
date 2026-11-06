"""Cooperative satellite COM pointing; two propagated bodies, only one controlled."""
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .benchmark_attitude import AttitudeSafety
from .benchmark_contract import Contract, Number, Positive, State6, Vector3
from .benchmark_pointing import (
    PointingContext, PointingObservation, PointingObservationModel, PointingTask,
    PointingTruthSample, PointingWindow,
)


class TargetPointingInitial(Contract):
    frame: Literal['target_relative_to_chaser_COM_in_chaser_TNR_rotating']
    position_tnr_m: Vector3
    velocity_tnr_m_s: Vector3
    position_half_width_m: Annotated[Number, Field(ge=0)]
    velocity_half_width_m_s: Annotated[Number, Field(ge=0)]


class TargetPointingObjective(PointingWindow):
    kind: Literal['spacecraft_com']
    target_reference: Literal['passive_target_COM']


class TargetPointingSafety(AttitudeSafety):
    minimum_range_m: Annotated[Number, Field(ge=100)]
    maximum_range_m: Positive
    maximum_relative_speed_m_s: Positive


class TargetPointingObservationModel(PointingObservationModel):
    kind: Literal['noisy_attitude_gyro_dual_gnss_and_known_ephemeris']
    target_link: Literal['cooperative_noisy_COM_GNSS_zero_delay']


class TargetPointingTask(PointingTask):
    schema_version: Literal['spacesim-autonomy-target-pointing/1']
    requirement_id: Literal['ssb.attitude.target-tracking']
    source_id: Literal['D11']
    backend: Literal['native_team_dual_pointing_v1']
    target_cooperation: Literal['C']
    spacecraft_count: Literal[2]
    reference_frame: Literal['J2000_two_spacecraft_COM_chaser_TNR_initial_body_plus_Z_boresight']
    target_initial: TargetPointingInitial
    objective: TargetPointingObjective
    safety: TargetPointingSafety
    observation: TargetPointingObservationModel
    target_control: Literal['passive_free_flight_no_target_actuators']

    @model_validator(mode='after')
    def feasible(self):
        self._validate_tracking_control()
        import math
        target, limits = self.target_initial, self.safety
        distance = math.sqrt(sum(v*v for v in target.position_tnr_m))
        width = math.sqrt(3)*target.position_half_width_m
        if not limits.minimum_range_m < distance-width <= distance+width < limits.maximum_range_m:
            raise ValueError('entire target initialization must lie strictly inside the separation shell')
        speed = math.sqrt(sum(v*v for v in target.velocity_tnr_m_s))+math.sqrt(3)*target.velocity_half_width_m_s
        if speed >= limits.maximum_relative_speed_m_s:
            raise ValueError('target relative velocity initialization exceeds its safety bound')
        return self

    def validate_catalogue(self, catalogue):
        entry = catalogue.task(self.requirement_id)
        if (entry.source_id != self.source_id or self.vision_mode not in entry.requested_vision_modes
                or self.target_cooperation not in entry.target_cooperation):
            raise ValueError('target-pointing configuration does not match D11')


class TargetPointingObservation(PointingObservation):
    schema_version: Literal['spacesim-autonomy-target-pointing-observation/1'] = 'spacesim-autonomy-target-pointing-observation/1'
    target_gnss_j2000: State6


class TargetPointingTruthSample(PointingTruthSample):
    target_com_j2000: State6


class TargetPointingContext(PointingContext):
    task: TargetPointingTask
