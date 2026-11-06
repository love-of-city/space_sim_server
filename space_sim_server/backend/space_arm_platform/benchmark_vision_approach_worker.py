"""Causal image feedback, isolated from Basilisk and private physical records."""
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from .benchmark_vision_approach import VisionApproachAction
from .benchmark_vision_worker import ImagePoseMethod


class ImageApproachMethod:
    def __init__(self,*,mode='feedback'):
        if mode not in {'feedback','no_thrust','unsafe_close'}:
            raise ValueError('unknown image approach baseline')
        self.mode=mode

    def reset(self,context):
        self.context=context
        self.images=ImagePoseMethod()
        self.images.reset(context)
        q=np.asarray(context.task.cameras[0].orientation_body_from_optical_wxyz)
        self.body_from_camera=Rotation.from_quat(q[[1,2,3,0]])
        self.history=deque(maxlen=5)
        self.last_time=None

    def act(self,observation):
        if observation.sample_time_ns!=(0 if self.last_time is None else self.last_time+500_000_000):
            raise ValueError('image feedback requires consecutive exposure times')
        self.last_time=observation.sample_time_ns
        pose=self.images.act(observation).pose
        force=np.zeros(3)
        task=self.context.task
        if pose is None:
            self.history.clear()
        else:
            position=np.asarray(pose.position_target_origin_optical_m)
            self.history.append((observation.sample_time_ns,position))
            if self.mode=='feedback' and len(self.history)>=3:
                times=np.asarray([(stamp-self.history[0][0])*1e-9 for stamp,_ in self.history])
                centered=times-times.mean()
                velocity=np.sum(centered[:,None]*np.asarray([point for _,point in self.history]),axis=0)/np.dot(centered,centered)
                error=position-task.objective.target_origin_optical_goal_m
                camera_force=self.context.chaser_mass_kg*(.012*error+.22*velocity)
                force=self.body_from_camera.apply(camera_force)
                magnitude=float(np.linalg.norm(force))
                if magnitude>task.safety.maximum_force_n:
                    force*=task.safety.maximum_force_n/magnitude
        if self.mode=='unsafe_close':
            force=np.array([0.,-task.safety.maximum_force_n,0.])
        return VisionApproachAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns,reference_frame=task.reference_frame,pose=pose,
            force_frame=task.force_frame,force_body_n=tuple(float(value) for value in force))

    def close(self):
        self.images.close()
