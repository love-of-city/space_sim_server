"""Smooth commanded grasp corrections; never filter or rewrite observations."""

import math
import numpy as np
from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector


class PoseTargetFilter:
    """Three cascaded real poles, integrated exactly for each held measurement.

    A fixed local rotation chart follows small drift around the initial pickup
    orientation, including pickups whose world yaw is near +/-pi. At 2 Hz per
    pole, 21.5 Hz socket vibration is attenuated by roughly three orders of
    magnitude. Slow drift is still followed; raw poses remain available for
    contact and final acceptance checks.
    """

    def __init__(self, position, rotation, pole_hz=2.0):
        if not math.isfinite(pole_hz) or pole_hz <= 0:
            raise ValueError("Tracking pole must be finite and positive")
        self.position = np.asarray(position).copy()
        self.rotation = np.asarray(rotation).copy()
        self.rate = 2 * math.pi * pole_hz
        self.state = np.zeros((3, 6))

    def update(self, position, rotation, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Tracking step must be finite and nonnegative")
        target = np.r_[np.asarray(position) - self.position,
                       rotation_matrix_to_vector(np.asarray(rotation) @ self.rotation.T)]
        h = self.rate * dt
        e1, e2, e3 = self.state - target
        self.state[:] = target + math.exp(-h) * np.array([
            e1, e2 + h * e1, e3 + h * e2 + .5 * h * h * e1,
        ])
        p, rv = self.state[-1, :3], self.state[-1, 3:]
        angle = np.linalg.norm(rv)
        rotation = self.rotation
        if angle > 1e-12:
            rotation = axis_angle_to_matrix(rv / angle, angle) @ rotation
        return self.position + p, rotation.copy()
