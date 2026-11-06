"""Image-history rate fit with own-gyro compensation; no native truth inputs."""
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation


class ImageGyroAngularVelocity:
    def __init__(self,camera,window_ns):
        self.window_ns=int(window_ns)
        q=np.asarray(camera.orientation_body_from_optical_wxyz)
        self.body_from_camera=Rotation.from_quat(q[[1,2,3,0]])
        self.body_zero_from_current=Rotation.identity()
        self.previous_gyro=None
        self.last_image_ns=None
        self.gyro_valid=True
        self.history=deque()

    def update(self,time_ns,gyro_samples,pose):
        if time_ns!=(0 if self.last_image_ns is None else self.last_image_ns+500_000_000):
            raise ValueError('spin estimator requires consecutive image exposure times')
        self.last_image_ns=time_ns
        for sample in gyro_samples:
            expected=0 if self.previous_gyro is None else self.previous_gyro.sample_time_ns+20_000_000
            if sample.sample_time_ns!=expected or sample.sample_time_ns>time_ns:
                raise ValueError('spin estimator requires causal contiguous gyro samples')
            self.gyro_valid=self.gyro_valid and sample.valid
            if self.previous_gyro is not None and self.gyro_valid:
                rate=.5*(np.asarray(self.previous_gyro.angular_velocity_body_rad_s)+sample.angular_velocity_body_rad_s)
                self.body_zero_from_current=self.body_zero_from_current*Rotation.from_rotvec(rate*.02)
            self.previous_gyro=sample
        if self.previous_gyro is None or self.previous_gyro.sample_time_ns!=time_ns:
            raise ValueError('own gyro must reach the current image exposure')
        if pose is None or not self.gyro_valid:
            self.history.clear()
            return None
        q=np.asarray(pose.orientation_optical_from_target_wxyz)
        target=self.body_zero_from_current*self.body_from_camera*Rotation.from_quat(q[[1,2,3,0]])
        self.history.append((time_ns,target))
        while self.history and self.history[0][0]<time_ns-self.window_ns:
            self.history.popleft()
        if len(self.history)<3 or self.history[0][0]!=time_ns-self.window_ns:
            return None
        reference=self.history[0][1]
        times=np.asarray([(tick-self.history[0][0])*1e-9 for tick,_ in self.history])
        logs=np.asarray([(reference.inv()*orientation).as_rotvec() for _,orientation in self.history])
        centered=times-times.mean()
        derivative=np.sum(centered[:,None]*logs,axis=0)/np.dot(centered,centered)
        # The regression is in the first target axes of the window; report
        # its tangent in the current target-body axes declared by the task.
        return tuple(float(value) for value in (target.inv()*reference).apply(derivative))
