"""Separate image workers and a missing-estimate control for target spin."""
from pathlib import Path

from space_arm_platform.benchmark_external import ExternalMethod, ExternalMethodDefinition
from space_arm_platform.benchmark_vision_motion import VisionMotionAction


class SpinProcess(ExternalMethod):
    spec_name='vision-spin-builtin.json'

    def __init__(self,task,output):
        spec=Path(__file__).resolve().parents[1]/'examples/autonomy_methods'/self.spec_name
        super().__init__(ExternalMethodDefinition(spec,task),output)


class ZeroSpinProcess(SpinProcess):
    spec_name='vision-spin-zero.json'


class MissingSpin:
    def __init__(self,task,output):
        pass

    def reset(self,context):
        self.context=context

    def act(self,observation):
        return VisionMotionAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns,reference_frame=self.context.task.reference_frame,
            pose=None,angular_velocity_frame=self.context.task.angular_velocity_frame,target_angular_velocity_body_rad_s=None)

    def close(self):
        pass


METHODS={'image-gyro-spin-v1':SpinProcess,'zero-image-spin-v1':ZeroSpinProcess,'missing-image-spin-v1':MissingSpin}
