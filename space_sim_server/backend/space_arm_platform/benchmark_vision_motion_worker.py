"""Image/gyro-only spin baseline, loaded in an isolated OpenCV process."""
from .benchmark_spin_estimator import ImageGyroAngularVelocity
from .benchmark_vision_motion import VisionMotionAction
from .benchmark_vision_worker import ImagePoseMethod


class ImageSpinMethod:
    def __init__(self,*,zero_rate=False):
        self.zero_rate=zero_rate

    def reset(self,context):
        self.context=context
        self.images=ImagePoseMethod()
        self.images.reset(context)
        self.spin=ImageGyroAngularVelocity(context.task.cameras[0],context.task.objective.spin_warmup_ns)

    def act(self,observation):
        pose=self.images.act(observation).pose
        rate=self.spin.update(observation.sample_time_ns,observation.gyro_samples,pose)
        if self.zero_rate and rate is not None:
            rate=(0.,0.,0.)
        return VisionMotionAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns,reference_frame=self.context.task.reference_frame,
            pose=pose,angular_velocity_frame=self.context.task.angular_velocity_frame,target_angular_velocity_body_rad_s=rate)

    def close(self):
        self.images.close()
