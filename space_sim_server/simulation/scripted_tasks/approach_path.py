"""Straight TCP approach with coordinated orientation and finger opening.

Position stays on the start-to-grasp line. Rotation and opening finish smoothly
before contact without bending the position path or introducing a time stop.
The complete arm/finger trajectory must still pass the environment planner.
"""

from dataclasses import dataclass
import math
import numpy as np
from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector


@dataclass(frozen=True)
class FinishBlend:
    finish: float
    blend_fraction: float = 0.35

    def __post_init__(self):
        if not 0 < self.finish <= 1 or not 0 < self.blend_fraction <= 1:
            raise ValueError("Invalid approach blend")

    @property
    def width(self):
        return self.finish * self.blend_fraction

    @property
    def speed(self):
        return 1 / (self.finish - self.width / 2)

    def state(self, progress):
        u = float(np.clip(progress, 0, 1))
        width, speed = self.width, self.speed
        begin = self.finish - width
        if u >= self.finish:
            return 1.0, 0.0, 0.0, 0.0
        if u <= begin:
            return speed * u, speed, 0.0, 0.0
        x = (u - begin) / width
        integral = 2.5*x**4 - 3*x**5 + x**6
        return (
            speed * (begin + width * (x - integral)),
            speed * (1 - (10*x**3 - 15*x**4 + 6*x**5)),
            -speed / width * (30*x**2 - 60*x**3 + 30*x**4),
            -speed / width**2 * (60*x - 180*x**2 + 120*x**3),
        )

    def derivative_bounds(self):
        return self.speed * np.array([1, 1.875/self.width, (10/math.sqrt(3))/self.width**2])


class ApproachPosePath:
    def __init__(self, position, rotation, goal, goal_rotation, axis, opening, clearance,
                 goal_opening=1.):
        self.start_rotation = np.asarray(rotation).copy()
        self.goal = np.asarray(goal).copy()
        self.start_position = np.asarray(position).copy()
        self.displacement = self.goal - self.start_position
        self.distance = float(np.linalg.norm(self.displacement))
        if not math.isfinite(clearance) or clearance <= 0:
            raise ValueError("Approach orientation clearance must be positive")
        # Clearance is distance along the direct line, not an axial waypoint.
        # Short/lateral reapproaches are valid candidates too: let collision
        # and IK checks decide rather than requiring the start above the grasp.
        finish = max(.5, 1-clearance/self.distance) if self.distance > 1e-12 else 1.
        self.blend = FinishBlend(finish)
        self.rotation_vector = rotation_matrix_to_vector(goal_rotation @ self.start_rotation.T)
        self.angle = float(np.linalg.norm(self.rotation_vector))
        self.initial_opening = float(np.clip(opening, 0, 1))
        if not math.isfinite(goal_opening) or not 0 <= goal_opening <= 1:
            raise ValueError("Invalid approach opening")
        self.goal_opening = float(goal_opening)

    def pose(self, progress):
        u = float(np.clip(progress, 0, 1))
        s = self.blend.state(u)[0]
        position = self.start_position + self.displacement*u
        rotation = self.start_rotation
        if self.angle > 1e-12:
            rotation = axis_angle_to_matrix(self.rotation_vector/self.angle, self.angle*s) @ rotation
        return position, rotation

    def opening(self, progress):
        return self.initial_opening + (self.goal_opening-self.initial_opening)*self.blend.state(progress)[0]

    def derivative_bounds(self):
        b = self.blend.derivative_bounds()
        translation = np.array([self.distance, 0., 0.])
        return translation, self.angle*b, abs(self.goal_opening-self.initial_opening)*b
