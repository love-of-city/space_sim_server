"""Wheel pointing using only public noisy telemetry from the two spacecraft."""
import numpy as np

from simulation.benchmark_attitude_methods import NoWheelTorque, UnsafeSpin
from simulation.benchmark_pointing_methods import PointingWheelPD


class TargetPointingPD(PointingWheelPD):
    def _reference(self, observation):
        relative = np.asarray(observation.target_gnss_j2000)-observation.chaser_gnss_j2000
        distance = float(np.linalg.norm(relative[:3]))
        if distance < 1.:
            raise ValueError('target GNSS cannot define a line of sight')
        direction = relative[:3]/distance
        return direction, (relative[3:]-direction*(direction@relative[3:]))/distance


class FrozenTargetPointingPD(TargetPointingPD):
    frozen_reference = True


METHODS = {'cooperative-target-pointing-v1': TargetPointingPD,
    'frozen-target-pointing-v1': FrozenTargetPointingPD,
    'no-target-pointing-torque-v1': NoWheelTorque, 'unsafe-target-pointing-spin-v1': UnsafeSpin}
