"""Estimator process isolation and explicit image-pose negative controls."""
from pathlib import Path

from space_arm_platform.benchmark_external import ExternalMethod, ExternalMethodDefinition
from space_arm_platform.benchmark_vision import TargetPose, VisionPoseAction


class FiducialProcess(ExternalMethod):
    def __init__(self,task,output):
        spec=Path(__file__).resolve().parents[1]/'examples/autonomy_methods/vision-pose-builtin.json'
        super().__init__(ExternalMethodDefinition(spec,task),output)


class MissingPose:
    def __init__(self,task,output):
        pass

    def reset(self,context):
        self.context=context

    def act(self,observation):
        return VisionPoseAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns,reference_frame=self.context.task.reference_frame,pose=None)

    def close(self):
        pass


class ZeroPose(MissingPose):
    def act(self,observation):
        action=super().act(observation)
        return action.model_copy(update={'pose':TargetPose(position_target_origin_optical_m=(0.,0.,0.),
            orientation_optical_from_target_wxyz=(1.,0.,0.,0.))})


METHODS={'image-fiducial-pose-v1':FiducialProcess,'missing-image-pose-v1':MissingPose,'zero-image-pose-v1':ZeroPose}
