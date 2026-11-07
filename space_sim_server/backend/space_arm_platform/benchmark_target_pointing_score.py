"""Independent two-body pointing geometry, initialization and separation safety."""
import math
import numpy as np

from .benchmark_pointing_score import PointingScorer
from .benchmark_target_pointing import TargetPointingTruthSample


class TargetPointingScorer(PointingScorer):
    sample_type = TargetPointingTruthSample

    def __init__(self, task):
        super().__init__(task)
        self.minimum_range = math.inf
        self.maximum_range = self.maximum_relative_speed = 0.

    def _spacecraft_target(self, sample):
        target, chaser = np.asarray(sample.target_com_j2000), np.asarray(sample.chaser_com_j2000)
        radius = np.linalg.norm(chaser[:3])
        angular_momentum = np.cross(chaser[:3], chaser[3:])
        hnorm = np.linalg.norm(angular_momentum)
        if radius < 1 or hnorm < 1:
            self.evidence_errors.add('degenerate_chaser_orbit')
            return target[:3], target[3:]
        radial, normal = chaser[:3]/radius, angular_momentum/hnorm
        frame = np.column_stack((np.cross(normal, radial), normal, radial))
        dr, dv = target[:3]-chaser[:3], target[3:]-chaser[3:]
        omega = angular_momentum/radius**2
        position, velocity = frame.T@dr, frame.T@(dv-np.cross(omega, dr))
        distance, speed = float(np.linalg.norm(dr)), float(np.linalg.norm(velocity))
        limits, initial = self.task.safety, self.task.target_initial
        if not self.count:
            initial_chaser = self.task.initial
            theta, inclination = np.deg2rad([initial_chaser.true_anomaly_deg,initial_chaser.inclination_deg])
            initial_radius = 6378136.6+initial_chaser.altitude_m
            expected_position = initial_radius*np.array([np.cos(theta),np.sin(theta)*np.cos(inclination),np.sin(theta)*np.sin(inclination)])
            expected_velocity = np.sqrt(3.986004418e14/initial_radius)*np.array([-np.sin(theta),np.cos(theta)*np.cos(inclination),np.cos(theta)*np.sin(inclination)])
            if np.linalg.norm(chaser[:3]-expected_position) > .001 or np.linalg.norm(chaser[3:]-expected_velocity) > 1e-6:
                self.evidence_errors.add('chaser_orbit_initialization_outside_declared_distribution')
            if (np.any(np.abs(position-initial.position_tnr_m) > initial.position_half_width_m+1e-5)
                    or np.any(np.abs(velocity-initial.velocity_tnr_m_s) > initial.velocity_half_width_m_s+1e-7)):
                self.evidence_errors.add('target_initialization_outside_declared_distribution')
        if distance < limits.minimum_range_m:
            self.safety_violations.add('minimum_range')
        if distance > limits.maximum_range_m:
            self.safety_violations.add('maximum_range')
        if speed > limits.maximum_relative_speed_m_s:
            self.safety_violations.add('relative_speed')
        if np.linalg.norm(target[:3]) <= 6378136.6:
            self.safety_violations.add('target_earth_surface_intersection')
        self.minimum_range = min(self.minimum_range, distance)
        self.maximum_range = max(self.maximum_range, distance)
        self.maximum_relative_speed = max(self.maximum_relative_speed, speed)
        return target[:3], target[3:]

    def result(self):
        result = super().result()
        result.update(path_requirement='continuous_cooperative_spacecraft_pointing',
            target_reference='actual_passive_target_COM', minimum_range_m=self.minimum_range if self.count else None,
            maximum_range_m=self.maximum_range if self.count else None,
            maximum_relative_speed_m_s=self.maximum_relative_speed if self.count else None,
            spacecraft_count=2, controlled_spacecraft=1)
        return result
