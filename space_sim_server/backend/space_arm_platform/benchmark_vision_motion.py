"""Image and own-gyro spin estimation; target rates remain private evidence."""
import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, model_validator

from .benchmark_contract import Contract, Count, Number, Positive, Vector3
from .benchmark_vision import (
    PoseAccuracy, VisionInitialState, VisionPoseAction, VisionPoseContext,
    VisionPoseEstimateEvidence, VisionPoseObservation, VisionPoseTask, VisionPoseTruthSample,
)


class MotionInitialState(VisionInitialState):
    initial_body_rates: Literal['seeded_target_spin_and_chaser_rates']
    target_spin_sign: Literal['equiprobable_both_directions']
    target_rate_body_rad_s: Vector3
    target_rate_half_width_rad_s: Annotated[Number,Field(ge=0,le=.02)]
    chaser_rate_body_rad_s: Vector3
    chaser_rate_half_width_rad_s: Annotated[Number,Field(ge=0,le=.005)]


class SpinAccuracy(PoseAccuracy):
    kind: Literal['full_duration_pose_and_spin']
    spin_warmup_ns: Count
    angular_velocity_rmse_limit_rad_s: Positive
    maximum_angular_velocity_error_rad_s: Positive


class OwnGyroModel(Contract):
    kind: Literal['own_body_gyro_no_target_rate_link']
    sample_period_ns: Literal[20_000_000]
    white_noise_std_rad_s: Annotated[Number,Field(gt=0,le=.1)]
    initial_bias_std_rad_s: Annotated[Number,Field(ge=0,le=.1)]
    bias_random_walk_rad_s_sqrt_s: Annotated[Number,Field(ge=0,le=.01)]
    range_rad_s: Annotated[Number,Field(gt=0,le=10)]

    def sensor_config(self):
        return self.model_dump(exclude={'kind','sample_period_ns'})


class VisionMotionTask(VisionPoseTask):
    schema_version: Literal['spacesim-autonomy-rendered-spin/1']
    requirement_id: Literal['ssb.vision.spin-estimation']
    source_id: Literal['D27']
    backend: Literal['native_team_rendered_spin_v1']
    action_type: Literal['target_pose_and_body_angular_velocity_estimate_only']
    angular_velocity_frame: Literal['target_body_axes_relative_to_J2000']
    initial: MotionInitialState
    objective: SpinAccuracy
    own_gyro: OwnGyroModel

    def source_camera_requirement(self):
        return 'ssb.vision.spin-estimation',2

    @model_validator(mode='after')
    def motion_contract(self):
        initial,goal=self.initial,self.objective
        for name in ('target','chaser'):
            mean=getattr(initial,name+'_rate_body_rad_s')
            spread=math.sqrt(3)*getattr(initial,name+'_rate_half_width_rad_s')
            if math.dist(mean,(0.,0.,0.))+spread>=self.safety.maximum_body_rate_rad_s:
                raise ValueError('every initial body rate must be inside the physical safety limit')
        if math.dist(initial.target_rate_body_rad_s,(0.,0.,0.))-math.sqrt(3)*initial.target_rate_half_width_rad_s<=.01:
            raise ValueError('spin scenarios must exclude stationary targets')
        if (goal.spin_warmup_ns<2*self.decision_period_ns or goal.spin_warmup_ns%self.decision_period_ns
                or goal.spin_warmup_ns>=self.duration_s*1e9):
            raise ValueError('spin warmup must end on a camera tick before the scoring window ends')
        if goal.angular_velocity_rmse_limit_rad_s>goal.maximum_angular_velocity_error_rad_s:
            raise ValueError('spin RMSE limit must not exceed the peak limit')
        if self.own_gyro.range_rad_s<=self.safety.maximum_body_rate_rad_s:
            raise ValueError('gyro range must exceed the declared physical rate bound')
        return self

    @property
    def expected_spin_estimates(self):
        return self.expected_estimates-self.objective.spin_warmup_ns//self.decision_period_ns


class ChaserGyroPacket(Contract):
    sample_time_ns: Count
    angular_velocity_body_rad_s: Vector3
    error_variance_rad2_s2: tuple[Annotated[Number,Field(ge=0)],Annotated[Number,Field(ge=0)],Annotated[Number,Field(ge=0)]]
    saturated_axes: tuple[StrictBool,StrictBool,StrictBool]
    valid: StrictBool
    body_name: Literal['cubesat_bus'] = 'cubesat_bus'
    frame: Literal['spacecraft_body'] = 'spacecraft_body'
    reference_frame: Literal['J2000'] = 'J2000'

    @model_validator(mode='after')
    def gyro_contract(self):
        if self.sample_time_ns%20_000_000 or self.valid==any(self.saturated_axes):
            raise ValueError('gyro must have a physical 50 Hz timestamp and consistent saturation status')
        return self

    @classmethod
    def from_sensor(cls,raw):
        if (raw['measurement']!='simulated_body_angular_rate' or raw['sample_rate_hz']!=50
                or raw['sensor_name']!='benchmark_chaser_gyro'):
            raise ValueError('unexpected gyroscope measurement source')
        return cls(**{key:int(raw[key]) if key=='sample_time_ns' else raw[key] for key in cls.model_fields})


class VisionMotionContext(VisionPoseContext):
    task: VisionMotionTask


class VisionMotionObservation(VisionPoseObservation):
    schema_version: Literal['spacesim-autonomy-rendered-spin-observation/1'] = 'spacesim-autonomy-rendered-spin-observation/1'
    gyro_samples: tuple[ChaserGyroPacket,...] = Field(min_length=1,max_length=25)

    @model_validator(mode='after')
    def contiguous_own_gyro(self):
        start=0 if self.sequence==0 else self.sample_time_ns-500_000_000+20_000_000
        expected=tuple(range(start,self.sample_time_ns+1,20_000_000))
        if tuple(sample.sample_time_ns for sample in self.gyro_samples)!=expected:
            raise ValueError('own-gyro history must cover every fresh physical tick since the preceding image')
        return self


class VisionMotionAction(VisionPoseAction):
    schema_version: Literal['spacesim-autonomy-spin-estimate/1'] = 'spacesim-autonomy-spin-estimate/1'
    angular_velocity_frame: Literal['target_body_axes_relative_to_J2000']
    target_angular_velocity_body_rad_s: Vector3 | None

    def validate_for(self,observation,task):
        value=VisionMotionAction.model_validate(self)
        if (value.episode_id!=observation.episode_id or value.observation_sequence!=observation.sequence
                or value.estimate_time_ns!=observation.sample_time_ns or value.reference_frame!=task.reference_frame
                or value.angular_velocity_frame!=task.angular_velocity_frame):
            raise ValueError('spin estimate identity, exposure time or frame differs from the observation')
        if tuple(image.camera_id for image in observation.images)!=tuple(camera.camera_id for camera in task.cameras):
            raise ValueError('spin observation has missing, unexpected or reordered cameras')
        if value.pose is None and value.target_angular_velocity_body_rad_s is not None:
            raise ValueError('target-body angular velocity requires a contemporaneous target pose')


class VisionMotionEstimateEvidence(VisionPoseEstimateEvidence):
    record_type: Literal['vision_motion_estimate'] = 'vision_motion_estimate'
    angular_velocity_frame: Literal['target_body_axes_relative_to_J2000']
    target_angular_velocity_body_rad_s: Vector3 | None


class VisionMotionTruthSample(VisionPoseTruthSample):
    chaser_gyro: ChaserGyroPacket
