"""Seeded free rotation with real images and the existing physical gyro model."""
import numpy as np
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_vision_motion import (
    ChaserGyroPacket, VisionMotionContext, VisionMotionEstimateEvidence,
    VisionMotionObservation, VisionMotionTruthSample,
)
from space_arm_platform.benchmark_vision_motion_score import VisionMotionScorer
from simulation.benchmark_native_vision import NativeVisionPoseEnvironment, public_context
from simulation.gyroscope import GyroConfig, Gyroscope


def motion_context(task,chaser_mass_kg):
    return public_context(task,chaser_mass_kg,context_type=VisionMotionContext)


class NativeVisionMotionEnvironment(NativeVisionPoseEnvironment):
    context_type=staticmethod(motion_context)
    scorer_type=VisionMotionScorer

    def _initialize_bodies(self,states,seed):
        super()._initialize_bodies(states,seed)
        rng=np.random.default_rng(np.random.SeedSequence([seed,3]))
        initial=self.task.initial
        self._gyro=Gyroscope(GyroConfig(**self.task.own_gyro.sensor_config()),seed=seed,
            body_name='cubesat_bus',sensor_name='benchmark_chaser_gyro')
        self._gyro_interval=[]
        for index,name in enumerate(('target','chaser')):
            half_width=getattr(initial,name+'_rate_half_width_rad_s')
            sign=(-1. if rng.integers(0,2)==0 else 1.) if name=='target' else 1.
            rate=sign*np.asarray(getattr(initial,name+'_rate_body_rad_s'))+rng.uniform(-half_width,half_width,3)
            record=self.initial_attitudes[index]
            q=np.asarray(record['orientation_body_to_j2000_wxyz'])
            rotation=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
            center=np.asarray(self.model_manifest['bodies'][record['body']]['center_body_m'])
            body=self._scene.getBody(record['body'])
            body.setAttitudeRate(rate.tolist())
            # MJScene expects the body-origin velocity. Keep the independently
            # sampled COM orbit unchanged when introducing a finite body spin.
            body.setVelocity((states[index,3:]-np.cross(rotation@rate,rotation@center)).tolist())
            record['angular_velocity_body_rad_s']=rate.tolist()

    def _truth_sample(self,nanos):
        physical=super()._truth_sample(nanos)
        packet=ChaserGyroPacket.from_sensor(self._gyro.sample(int(nanos),physical.chaser_angular_velocity_body_rad_s))
        self._gyro_interval.append(packet)
        return VisionMotionTruthSample(**physical.model_dump(),chaser_gyro=packet)

    def _measure(self,sequence):
        image_observation=super()._measure(sequence)
        observation=VisionMotionObservation(**image_observation.model_dump(exclude={'schema_version'}),
            gyro_samples=tuple(self._gyro_interval))
        self._gyro_interval=[]
        return observation

    def _estimate_evidence(self,action):
        return VisionMotionEstimateEvidence(sequence=action.observation_sequence,estimate_time_ns=action.estimate_time_ns,
            reference_frame=action.reference_frame,image_sha256=tuple(image.png_sha256 for image in self._observation.images),
            pose=action.pose,angular_velocity_frame=action.angular_velocity_frame,
            target_angular_velocity_body_rad_s=action.target_angular_velocity_body_rad_s)

    def provenance(self):
        value=super().provenance()
        value.update(observation_model='actual stereo PNG and own noisy 50 Hz gyro; no target telemetry or target gyro',
            own_gyro_model=self.task.own_gyro.model_dump(mode='json'),
            angular_velocity_estimation='current target body rate relative to J2000 after declared image-history warmup')
        return value
