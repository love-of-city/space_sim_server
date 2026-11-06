"""Measurement-driven orbital coasting and reacquisition, without private outage times."""
import numpy as np

from simulation.benchmark_navigation_methods import GNSSOrbitEstimator, ZeroNavigationEstimate


class GNSSOutageRecovery(GNSSOrbitEstimator):
    def _states(self, observation):
        if observation.receiver_available:
            return super()._states(observation)
        model = self.filters[0]
        if model.state is None or self.last_time_ns is None:
            raise ValueError('coasting requires a prior public navigation solution')
        dt = (observation.sample_time_ns-self.last_time_ns)*1e-9
        if dt <= 0:
            raise ValueError('navigation recovery requires increasing observation times')
        now = observation.sample_time_ns*1e-9
        r, v = model.state[:3], model.state[3:]
        a = model.gravity.acceleration(r, now-dt)
        next_position = r+v*dt+.5*a*dt**2
        next_velocity = v+.5*(a+model.gravity.acceleration(next_position, now))*dt
        transition = np.eye(6)
        gradient = model.gravity.gradient(r, now-dt)
        transition[:3, 3:] = dt*np.eye(3)
        transition[3:, :3] = dt*gradient
        transition[:3, :3] += .5*dt**2*gradient
        noise = np.vstack([.5*dt**2*np.eye(3), dt*np.eye(3)])
        model.covariance = transition@model.covariance@transition.T+noise@noise.T*.003**2
        model.state = np.r_[next_position, next_velocity]
        self.last_time_ns = observation.sample_time_ns
        return [model.state.copy()]


class FrozenDuringOutage(GNSSOutageRecovery):
    """Retain an obsolete state during loss, then accept healthy measurements again."""
    def _states(self, observation):
        if observation.receiver_available:
            return super()._states(observation)
        model = self.filters[0]
        if model.state is None:
            raise ValueError('a prior public solution is required')
        self.last_time_ns = observation.sample_time_ns
        return [model.state.copy()]


METHODS = {'gnss-outage-recovery-v1': GNSSOutageRecovery,
           'freeze-during-outage-v1': FrozenDuringOutage, 'zero-recovery-estimate-v1': ZeroNavigationEstimate}
