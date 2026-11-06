"""Rendered monocular/stereo pose evaluation, without target telemetry."""
from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, StrictBool, field_validator, model_validator

from .benchmark_attitude import Quaternion, unit_quaternion
from .benchmark_camera import CameraImage, PinholeCamera, Sha256
from .benchmark_contract import Contract, Count, Number, Positive, SeedSplits, State6, Vector3


class VisionInitialState(Contract):
    target_altitude_m: Annotated[Number, Field(ge=200_000, le=2_000_000)]
    inclination_deg: Annotated[Number, Field(ge=0, le=180)]
    true_anomaly_deg: Number
    chaser_minus_target_com_j2000_m: Vector3
    position_half_width_m: Annotated[Number, Field(ge=0, le=.5)]
    chaser_minus_target_velocity_j2000_m_s: Vector3
    velocity_half_width_m_s: Annotated[Number, Field(ge=0, le=.01)]
    target_tilt_min_rad: Annotated[Number, Field(gt=0, lt=.5)]
    target_tilt_max_rad: Annotated[Number, Field(gt=0, lt=.5)]
    target_roll_half_width_rad: Annotated[Number, Field(gt=0, le=math.pi)]
    attitude_distribution: Literal['tilt_about_uniform_xz_axis_then_body_y_roll']
    initial_body_rates: Literal['zero_both_spacecraft']
    chaser_initial_attitude: Literal['identity_body_to_j2000']


class PoseAccuracy(Contract):
    kind: Literal['full_duration_relative_pose']
    minimum_available_fraction: Annotated[Number, Field(gt=0, le=1)]
    position_rmse_limit_m: Positive
    orientation_rmse_limit_rad: Annotated[Number, Field(gt=0, lt=math.pi)]
    maximum_position_error_m: Positive
    maximum_orientation_error_rad: Annotated[Number, Field(gt=0, lt=math.pi)]

    @model_validator(mode='after')
    def ordered_limits(self):
        if (self.position_rmse_limit_m > self.maximum_position_error_m
                or self.orientation_rmse_limit_rad > self.maximum_orientation_error_rad):
            raise ValueError('pose RMSE bounds must not exceed peak bounds')
        return self


class VisionSafety(Contract):
    minimum_range_m: Annotated[Number, Field(ge=2)]
    maximum_range_m: Positive
    maximum_relative_speed_m_s: Positive
    maximum_body_rate_rad_s: Positive
    # Model motors exist but the evaluated estimator has no actuation authority.
    maximum_force_n: Positive
    maximum_uncommanded_force_n: Annotated[Number, Field(ge=0, le=1e-6)]
    maximum_uncommanded_wheel_torque_nm: Annotated[Number, Field(ge=0, le=1e-6)]
    contact_policy: Literal['no_physical_contacts']


class VisionPoseTask(Contract):
    schema_version: Literal['spacesim-autonomy-rendered-pose/1']
    configuration_id: str = Field(pattern=r'^[a-z0-9]+(?:-[a-z0-9]+)*$')
    requirement_id: Literal['ssb.vision.monocular-pose','ssb.vision.stereo-pose']
    source_id: Literal['D23','D24']
    backend: Literal['native_team_rendered_pose_v1']
    target_cooperation: Literal['P']
    controlled_spacecraft: Literal[0]
    spacecraft_count: Literal[2]
    control_architecture: Literal['single_estimator']
    vision_mode: Literal['V2']
    action_type: Literal['target_relative_pose_estimate_only']
    attitude_control: Literal['none_passive_attitude']
    reference_frame: Literal['target_body_origin_and_axes_in_left_optical_camera']
    target_link: Literal['none']
    target_geometry_prior: Literal['known_passive_fiducial_layout_no_active_cooperation']
    fiducial_layout: Literal['target_aruco_flush_5cm_v2']
    cameras: tuple[PinholeCamera, ...] = Field(min_length=1, max_length=2)
    initial: VisionInitialState
    objective: PoseAccuracy
    safety: VisionSafety
    seeds: SeedSplits
    duration_s: Annotated[Number, Field(ge=1, le=120)]
    physics_period_ns: Literal[20_000_000]
    physics_substeps: Literal[8]
    decision_period_ns: Literal[500_000_000]
    earth_gravity: Literal['point_mass','j2']
    solar_gravity: Literal[True]
    drag_enabled: StrictBool
    srp_enabled: StrictBool
    disturbance_sigma_m_s2: Annotated[Number, Field(ge=0, le=.001)]
    mass_model: Literal['constant_no_propellant_model']

    @field_validator('controlled_spacecraft','spacecraft_count','physics_period_ns',
                     'physics_substeps','decision_period_ns',mode='before')
    @classmethod
    def integer_counts(cls,value):
        if type(value) is not int:
            raise ValueError('counts and periods must be integers')
        return value

    @field_validator('solar_gravity',mode='before')
    @classmethod
    def explicit_solar(cls,value):
        if value is not True:
            raise ValueError('rendered pose requires explicit solar gravity')
        return value

    @model_validator(mode='after')
    def semantic_contract(self):
        expected = ('ssb.vision.monocular-pose',1) if self.source_id == 'D23' else ('ssb.vision.stereo-pose',2)
        if (self.requirement_id,len(self.cameras)) != expected:
            raise ValueError('source ID, pose task and actual camera count must agree')
        expected_ids = ('team/camera/benchmark_left','team/camera/benchmark_right')[:len(self.cameras)]
        if tuple(camera.camera_id for camera in self.cameras) != expected_ids:
            raise ValueError('camera order must be calibrated left, then right if stereo')
        if any(camera.sample_rate_hz*self.decision_period_ns != 1_000_000_000 for camera in self.cameras):
            raise ValueError('camera sampling must equal the declared decision cadence')
        if len(self.cameras) == 2 and math.dist(self.cameras[0].position_body_m,self.cameras[1].position_body_m) < .01:
            raise ValueError('stereo requires at least a one-centimetre physical baseline')
        initial,limits = self.initial,self.safety
        radius = math.dist(initial.chaser_minus_target_com_j2000_m,(0.,0.,0.))
        spread = math.sqrt(3)*initial.position_half_width_m
        if not limits.minimum_range_m < radius-spread <= radius+spread < limits.maximum_range_m:
            raise ValueError('all initial ranges must lie inside the safety shell')
        speed = math.dist(initial.chaser_minus_target_velocity_j2000_m_s,(0.,0.,0.))
        if speed+math.sqrt(3)*initial.velocity_half_width_m_s >= limits.maximum_relative_speed_m_s:
            raise ValueError('all initial relative velocities must satisfy the safety bound')
        if initial.target_tilt_min_rad > initial.target_tilt_max_rad:
            raise ValueError('target tilt interval is reversed')
        if limits.maximum_uncommanded_force_n >= limits.maximum_force_n:
            raise ValueError('pose estimation cannot grant motor authority')
        ticks = self.duration_s*1e9/self.decision_period_ns
        if not math.isclose(ticks,round(ticks),abs_tol=1e-8,rel_tol=0):
            raise ValueError('duration must lie on the camera decision grid')
        return self

    @property
    def expected_estimates(self):
        return round(self.duration_s*1e9/self.decision_period_ns)

    def validate_catalogue(self,catalogue):
        requirement = catalogue.task(self.requirement_id)
        if (requirement.source_id != self.source_id or self.vision_mode not in requirement.requested_vision_modes
                or self.target_cooperation not in requirement.target_cooperation):
            raise ValueError('rendered pose configuration differs from the source requirement')


class FiducialMarker(Contract):
    marker_id: Annotated[int,Field(strict=True,ge=0,lt=50)]
    corners_target_m: tuple[Vector3,Vector3,Vector3,Vector3]

    @model_validator(mode='after')
    def physical_square(self):
        a,b,c,d = self.corners_target_m
        u,v = [b[i]-a[i] for i in range(3)],[d[i]-a[i] for i in range(3)]
        side = math.dist(a,b)
        if (side < 1e-4 or abs(math.dist(a,d)-side) > 1e-8
                or abs(sum(x*y for x,y in zip(u,v,strict=True))) > side**2*1e-8
                or math.dist(c,tuple(a[i]+u[i]+v[i] for i in range(3))) > 1e-8):
            raise ValueError('marker corners must form an ordered square in metres')
        return self


class VisionPoseContext(Contract):
    task: VisionPoseTask
    chaser_mass_kg: Positive
    marker_dictionary: Literal['DICT_4X4_50']
    markers: tuple[FiducialMarker,...] = Field(min_length=1,max_length=50)

    @model_validator(mode='after')
    def unique_markers(self):
        if len({marker.marker_id for marker in self.markers}) != len(self.markers):
            raise ValueError('fiducial IDs must be unique')
        return self


class VisionPoseObservation(Contract):
    schema_version: Literal['spacesim-autonomy-rendered-pose-observation/1'] = 'spacesim-autonomy-rendered-pose-observation/1'
    episode_id: str = Field(min_length=1)
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    images: tuple[CameraImage,...] = Field(min_length=1,max_length=2)

    @model_validator(mode='after')
    def simultaneous_public_images(self):
        if self.delivery_time_ns != self.sample_time_ns:
            raise ValueError('this sensor contract has zero simulated delivery delay; wall processing is separate')
        first = self.images[0]
        if (len({image.camera_id for image in self.images}) != len(self.images)
                or any(image.sample_time_ns != self.sample_time_ns
                       or image.source_frame_id != first.source_frame_id
                       or image.renderer_session_id != first.renderer_session_id for image in self.images)):
            raise ValueError('images must be unique cameras from one simultaneous authoritative frame')
        return self


class TargetPose(Contract):
    position_target_origin_optical_m: tuple[
        Annotated[Number,Field(ge=-1e6,le=1e6)],
        Annotated[Number,Field(ge=-1e6,le=1e6)],
        Annotated[Number,Field(ge=-1e6,le=1e6)],
    ]
    orientation_optical_from_target_wxyz: Quaternion

    @field_validator('orientation_optical_from_target_wxyz')
    @classmethod
    def normalized(cls,value):
        return unit_quaternion(value)


class VisionPoseAction(Contract):
    schema_version: Literal['spacesim-autonomy-pose-estimate/1'] = 'spacesim-autonomy-pose-estimate/1'
    episode_id: str
    observation_sequence: Count
    estimate_time_ns: Count
    reference_frame: Literal['target_body_origin_and_axes_in_left_optical_camera']
    pose: TargetPose | None

    def validate_for(self,observation,task):
        value = VisionPoseAction.model_validate(self)
        if (value.episode_id != observation.episode_id or value.observation_sequence != observation.sequence
                or value.estimate_time_ns != observation.sample_time_ns or value.reference_frame != task.reference_frame):
            raise ValueError('pose estimate identity, sample time or reference frame differs from its observation')
        if tuple(image.camera_id for image in observation.images) != tuple(camera.camera_id for camera in task.cameras):
            raise ValueError('pose observation has missing, unexpected or reordered cameras')


class VisionPoseEstimateEvidence(Contract):
    record_type: Literal['vision_pose_estimate'] = 'vision_pose_estimate'
    sequence: Count
    estimate_time_ns: Count
    reference_frame: Literal['target_body_origin_and_axes_in_left_optical_camera']
    image_sha256: tuple[Sha256,...] = Field(min_length=1,max_length=2)
    pose: TargetPose | None


class VisionCaptureEvidence(Contract):
    """Private renderer/physics correspondence, never part of method inputs."""
    record_type: Literal['vision_capture'] = 'vision_capture'
    sequence: Count
    sample_time_ns: Count
    renderer_session_id: str = Field(min_length=1,max_length=128)
    source_frame_id: Count
    camera_ids: tuple[str,...] = Field(min_length=1,max_length=2)
    calibration_sha256: tuple[Sha256,...] = Field(min_length=1,max_length=2)
    image_sha256: tuple[Sha256,...] = Field(min_length=1,max_length=2)
    camera_origins_j2000_m: tuple[Vector3,...] = Field(min_length=1,max_length=2)
    camera_orientations_j2000_from_optical_wxyz: tuple[Quaternion,...] = Field(min_length=1,max_length=2)

    @model_validator(mode='after')
    def same_cameras(self):
        if len({len(self.camera_ids),len(self.calibration_sha256),len(self.image_sha256),
                len(self.camera_origins_j2000_m),len(self.camera_orientations_j2000_from_optical_wxyz)}) != 1:
            raise ValueError('capture evidence must describe the same cameras in every field')
        for q in self.camera_orientations_j2000_from_optical_wxyz:
            unit_quaternion(q)
        return self


class VisionPoseTruthSample(Contract):
    time_ns: Count
    target_com_j2000: State6
    chaser_com_j2000: State6
    target_origin_j2000_m: Vector3
    chaser_origin_j2000_m: Vector3
    target_orientation_body_to_j2000_wxyz: Quaternion
    chaser_orientation_body_to_j2000_wxyz: Quaternion
    target_angular_velocity_body_rad_s: Vector3
    chaser_angular_velocity_body_rad_s: Vector3
    applied_motor_force_body_n: Vector3
    applied_wheel_torque_xyz_nm: Vector3
    contacts: Count
    external_state_writes: Count

    @field_validator('target_orientation_body_to_j2000_wxyz','chaser_orientation_body_to_j2000_wxyz')
    @classmethod
    def normalized(cls,value):
        return unit_quaternion(value)
