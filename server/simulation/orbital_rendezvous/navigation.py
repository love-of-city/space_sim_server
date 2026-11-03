"""Two independent GNSS receivers and six-state measurement-driven filters."""

import numpy as np

from simulation.navigation_gravity import PointMassNavigationGravity


class OrbitFilter:
    def __init__(self, position_sigma_m=0.2, velocity_sigma_m_s=0.002, *, gravity_model=None):
        self.gravity = gravity_model or PointMassNavigationGravity()
        self.state = None
        self.covariance = np.diag([position_sigma_m**2] * 3 + [velocity_sigma_m_s**2] * 3)
        self.measurement_covariance = self.covariance.copy()

    def update(self, observation, dt, acceleration, *, time_s=0.):
        z = np.asarray(observation, dtype=float)
        if z.shape != (6,) or not np.isfinite(z).all():
            raise ValueError("Invalid GNSS observation")
        if self.state is None:
            self.state = z.copy()
            return self.state.copy()
        r, v = self.state[:3], self.state[3:]
        a = self.gravity.acceleration(r, time_s-dt) + acceleration
        rnew = r + v * dt + 0.5 * a * dt**2
        vnew = v + 0.5 * (a + self.gravity.acceleration(rnew, time_s) + acceleration) * dt
        self.state = np.r_[rnew, vnew]
        gradient = self.gravity.gradient(r, time_s-dt)
        transition = np.eye(6)
        transition[:3, 3:] = dt * np.eye(3)
        transition[3:, :3] = dt * gradient
        transition[:3, :3] += 0.5 * dt**2 * gradient
        noise = np.vstack([0.5 * dt**2 * np.eye(3), dt * np.eye(3)])
        prior = transition @ self.covariance @ transition.T + noise @ noise.T * 0.003**2
        gain = np.linalg.solve((prior + self.measurement_covariance).T, prior.T).T
        self.state += gain @ (z - self.state)
        left = np.eye(6) - gain
        self.covariance = (
            left @ prior @ left.T + gain @ self.measurement_covariance @ gain.T
        )
        return self.state.copy()
