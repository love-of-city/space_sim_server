"""Public-measurement target orbit EKF; no simulator, target telemetry or realized initial states."""
import numpy as np
from scipy.spatial.transform import Rotation

from simulation.navigation_gravity import PointMassNavigationGravity
from space_arm_platform.benchmark_navigation import NavigationEstimateAction


class RadarOrbitEstimator:
    def reset(self, context):
        self.context = context
        self.gravity = PointMassNavigationGravity()
        self.state = self.covariance = self.last_time_ns = None

    def _estimate(self, observation):
        sensor = self.context.task.observation
        observer = np.asarray(observation.chaser_measurement_j2000)
        direction = Rotation.from_quat(observation.orientation_body_to_j2000_wxyz, scalar_first=True).apply(observation.direction_body_unit)
        position = observer[:3]+observation.range_m*direction
        radial = np.outer(direction, direction)
        position_covariance = (np.eye(3)*sensor.position_sigma_m**2+radial*sensor.range_sigma_m**2
            +(np.eye(3)-radial)*observation.range_m**2*(sensor.bearing_sigma_rad**2+sensor.attitude_sigma_rad**2))
        if self.state is None:
            # Radial velocity is observed immediately; transverse velocity is
            # learned from successive bearings, with an explicit wide prior.
            self.state = np.r_[position, observer[3:]+observation.range_rate_m_s*direction]
            self.covariance = np.zeros((6, 6))
            self.covariance[:3, :3] = position_covariance
            self.covariance[3:, 3:] = np.eye(3)
        else:
            dt = (observation.sample_time_ns-self.last_time_ns)*1e-9
            time_s = observation.sample_time_ns*1e-9
            r, v = self.state[:3], self.state[3:]
            acceleration = self.gravity.acceleration(r, time_s-dt)
            new_r = r+v*dt+.5*acceleration*dt**2
            new_v = v+.5*(acceleration+self.gravity.acceleration(new_r, time_s))*dt
            self.state = np.r_[new_r, new_v]
            gradient = self.gravity.gradient(r, time_s-dt)
            transition = np.eye(6)
            transition[:3, 3:] = dt*np.eye(3)
            transition[3:, :3] = dt*gradient
            transition[:3, :3] += .5*dt**2*gradient
            noise = np.vstack([.5*dt**2*np.eye(3), dt*np.eye(3)])
            self.covariance = transition@self.covariance@transition.T+noise@noise.T*.003**2
            # Measured position and line-of-sight velocity are the observations.
            # No fictitious three-dimensional target velocity measurement.
            measurement_matrix = np.zeros((4, 6))
            measurement_matrix[:3, :3] = np.eye(3)
            measurement_matrix[3, 3:] = direction
            measurement = np.r_[position, observation.range_rate_m_s+direction@observer[3:]]
            measurement_covariance = np.zeros((4, 4))
            measurement_covariance[:3, :3] = position_covariance
            relative_speed = float(np.linalg.norm(self.state[3:]-observer[3:]))
            measurement_covariance[3, 3] = (sensor.range_rate_sigma_m_s**2+sensor.velocity_sigma_m_s**2
                +relative_speed**2*(sensor.bearing_sigma_rad**2+sensor.attitude_sigma_rad**2))
            cross = self.covariance@measurement_matrix.T
            gain = np.linalg.solve(measurement_matrix@cross+measurement_covariance, cross.T).T
            self.state += gain@(measurement-measurement_matrix@self.state)
            left = np.eye(6)-gain@measurement_matrix
            self.covariance = left@self.covariance@left.T+gain@measurement_covariance@gain.T
        self.last_time_ns = observation.sample_time_ns
        return self.state.copy()

    def act(self, observation):
        return NavigationEstimateAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns, reference_frame=self.context.task.reference_frame,
            estimated_state=tuple(self._estimate(observation)))

    def close(self):
        pass


class FrozenRadarEstimate(RadarOrbitEstimator):
    def _estimate(self, observation):
        if self.state is None:
            return super()._estimate(observation)
        return self.state.copy()


class ZeroRadarEstimate(RadarOrbitEstimator):
    def _estimate(self, observation):
        return np.zeros(6)


METHODS = {'radar-orbit-estimator-v1': RadarOrbitEstimator, 'frozen-radar-estimate-v1': FrozenRadarEstimate,
    'zero-radar-estimate-v1': ZeroRadarEstimate}
