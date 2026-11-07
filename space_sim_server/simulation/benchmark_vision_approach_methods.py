"""Image controller workers plus an explicit missing-image negative control."""
from pathlib import Path

from space_arm_platform.benchmark_external import ExternalMethod,ExternalMethodDefinition
from space_arm_platform.benchmark_vision_approach import VisionApproachAction


class VisualApproachProcess(ExternalMethod):
    spec_name='vision-approach-builtin.json'

    def __init__(self,task,output):
        spec=Path(__file__).resolve().parents[1]/'examples/autonomy_methods'/self.spec_name
        super().__init__(ExternalMethodDefinition(spec,task),output)


class NoVisualThrustProcess(VisualApproachProcess):
    spec_name='vision-approach-no-thrust.json'


class UnsafeVisualApproachProcess(VisualApproachProcess):
    spec_name='vision-approach-unsafe.json'


class MissingVisualApproach:
    def __init__(self,task,output):
        pass

    def reset(self,context):
        self.context=context

    def act(self,observation):
        task=self.context.task
        return VisionApproachAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns,reference_frame=task.reference_frame,pose=None,
            force_frame=task.force_frame,force_body_n=(0.,0.,0.))

    def close(self):
        pass


METHODS={'image-stereo-approach-v1':VisualApproachProcess,'no-visual-thrust-v1':NoVisualThrustProcess,
         'unsafe-visual-close-v1':UnsafeVisualApproachProcess,'missing-visual-control-v1':MissingVisualApproach}
