"""Real image feedback with explicit body-force authority and physical dwell."""
import math
from typing import Literal

from pydantic import model_validator

from .benchmark_contract import Positive,Vector3
from .benchmark_vision import (
    PoseAccuracy,VisionPoseAction,VisionPoseContext,VisionPoseEstimateEvidence,
    VisionPoseObservation,VisionPoseTask,
)


class VisualApproachObjective(PoseAccuracy):
    kind: Literal['camera_relative_waypoint_dwell']
    target_origin_optical_goal_m: Vector3
    position_tolerance_m: Positive
    optical_relative_speed_tolerance_m_s: Positive
    dwell_s: Positive
    minimum_initial_position_error_m: Positive


class VisionApproachTask(VisionPoseTask):
    schema_version: Literal['spacesim-autonomy-visual-approach/1']
    requirement_id: Literal['ssb.approach.visual-feedback']
    source_id: Literal['D37']
    backend: Literal['native_team_visual_approach_v1']
    controlled_spacecraft: Literal[1]
    control_architecture: Literal['single_controller']
    action_type: Literal['body_force_and_image_pose_estimate']
    force_frame: Literal['chaser_body_axes_at_force_application']
    objective: VisualApproachObjective

    def source_camera_requirement(self):
        return 'ssb.approach.visual-feedback',2

    @model_validator(mode='after')
    def approach_contract(self):
        goal=self.objective
        if (goal.target_origin_optical_goal_m[2]<=0
                or goal.position_tolerance_m>=goal.minimum_initial_position_error_m
                or goal.optical_relative_speed_tolerance_m_s>=self.safety.maximum_relative_speed_m_s
                or goal.dwell_s>=self.duration_s):
            raise ValueError('approach must require initial progress and a bounded positive-depth dwell')
        if not math.isclose(goal.dwell_s*1e9/self.physics_period_ns,
                            round(goal.dwell_s*1e9/self.physics_period_ns),rel_tol=0,abs_tol=1e-8):
            raise ValueError('approach dwell must lie on the physical sampling grid')
        return self


class VisionApproachContext(VisionPoseContext):
    task: VisionApproachTask


class VisionApproachObservation(VisionPoseObservation):
    schema_version: Literal['spacesim-autonomy-visual-approach-observation/1'] = 'spacesim-autonomy-visual-approach-observation/1'


class VisionApproachAction(VisionPoseAction):
    schema_version: Literal['spacesim-autonomy-visual-approach-action/1'] = 'spacesim-autonomy-visual-approach-action/1'
    force_frame: Literal['chaser_body_axes_at_force_application']
    force_body_n: Vector3

    def validate_for(self,observation,task):
        value=VisionApproachAction.model_validate(self)
        if (value.episode_id!=observation.episode_id or value.observation_sequence!=observation.sequence
                or value.estimate_time_ns!=observation.sample_time_ns or value.reference_frame!=task.reference_frame
                or value.force_frame!=task.force_frame):
            raise ValueError('visual approach action identity, exposure time or frame differs')
        if tuple(image.camera_id for image in observation.images)!=tuple(camera.camera_id for camera in task.cameras):
            raise ValueError('visual approach requires the declared stereo cameras in calibration order')
        if math.dist(value.force_body_n,(0.,0.,0.))>task.safety.maximum_force_n+1e-12:
            raise ValueError('requested body force exceeds the declared vector magnitude limit')


class VisionApproachEstimateEvidence(VisionPoseEstimateEvidence):
    record_type: Literal['vision_approach_action'] = 'vision_approach_action'
    force_frame: Literal['chaser_body_axes_at_force_application']
    force_body_n: Vector3
