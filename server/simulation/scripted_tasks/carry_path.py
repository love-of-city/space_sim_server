"""Arc-length sampled, C2 rounded lift/transfer/lower route for a held part."""

import math
import numpy as np
from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector


class CarryPosePath:
    def __init__(self, start, start_rotation, goal, goal_rotation, height, radius=.04):
        start, goal = np.asarray(start), np.asarray(goal)
        span = np.linalg.norm(goal[:2] - start[:2])
        height = max(height, start[2] + 2*radius, goal[2] + 2*radius)
        corners = [start, np.r_[start[:2], height], np.r_[goal[:2], height], goal]
        trim = min(radius, span*.4)
        if trim < .005:
            raise ValueError("Carry route requires separated pickup and destination")
        pieces, markers = [], []
        cursor = start
        for i in (1, 2):
            corner = corners[i]
            incoming = corner-corners[i-1]
            outgoing = corners[i+1]-corner
            incoming /= np.linalg.norm(incoming)
            outgoing /= np.linalg.norm(outgoing)
            a, b = corner-trim*incoming, corner+trim*outgoing
            pieces.extend(np.linspace(cursor, a, 401)[:-1])
            # Equal first/last control edges give zero curvature at straight joins.
            controls = [a, a+.4*trim*incoming, a+.8*trim*incoming,
                        b-.8*trim*outgoing, b-.4*trim*outgoing, b]
            t = np.linspace(0., 1., 1601)[:-1]
            curve = sum(math.comb(5, j)*((1-t)**(5-j)*t**j)[:, None]*p
                        for j, p in enumerate(controls))
            pieces.extend(curve)
            markers.append(len(pieces)-1)
            cursor = b
        pieces.extend(np.linspace(cursor, goal, 401))
        self.points = np.asarray(pieces)
        self.lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.length = float(self.lengths[-1])
        self.boundaries = self.lengths[markers]/self.length
        self.start_rotation = start_rotation
        self.rv = rotation_matrix_to_vector(goal_rotation @ start_rotation.T)
        self.angle = float(np.linalg.norm(self.rv))

    def pose(self, progress):
        u = float(np.clip(progress, 0., 1.))
        p = np.array([np.interp(u*self.length, self.lengths, self.points[:, i]) for i in range(3)])
        blend = 10*u**3-15*u**4+6*u**5
        r = self.start_rotation
        if self.angle > 1e-12:
            r = axis_angle_to_matrix(self.rv/self.angle, blend*self.angle) @ r
        return p, r
