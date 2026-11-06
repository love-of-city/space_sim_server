"""Replaceable navigation algorithms consume only public GNSS measurements."""
import numpy as np

from simulation.orbital_rendezvous.navigation import OrbitFilter
from space_arm_platform.benchmark_navigation import NavigationEstimateAction


class GNSSOrbitEstimator:
    def reset(self, context):
        self.context = context
        sensor = context.task.observation
        self.filters = [OrbitFilter(sensor.position_sigma_m, sensor.velocity_sigma_m_s)
                        for _ in range(context.task.spacecraft_count)]
        self.last_time_ns = None

    def _states(self, observation):
        measurements = [observation.chaser_measurement_j2000]
        if self.context.task.source_id == "D01":
            if observation.target_measurement_j2000 is None:
                raise ValueError("cooperative target GNSS solution is missing")
            measurements.append(observation.target_measurement_j2000)
        dt = 0. if self.last_time_ns is None else (observation.sample_time_ns-self.last_time_ns)*1e-9
        states = [model.update(value, dt, np.zeros(3), time_s=observation.sample_time_ns*1e-9)
                  for model, value in zip(self.filters, measurements, strict=True)]
        self.last_time_ns = observation.sample_time_ns
        return states

    def act(self, observation):
        states = self._states(observation)
        state = states[0] if len(states) == 1 else states[0]-states[1]
        return NavigationEstimateAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns, reference_frame=self.context.task.reference_frame,
            estimated_state=tuple(state))

    def close(self):
        pass


class GNSSMeasurementOnly(GNSSOrbitEstimator):
    """Unfiltered measurement ablation, with the same inputs and output contract."""
    def _states(self, observation):
        values = [np.asarray(observation.chaser_measurement_j2000)]
        if self.context.task.source_id == "D01":
            values.append(np.asarray(observation.target_measurement_j2000))
        return values


class ZeroNavigationEstimate(GNSSOrbitEstimator):
    """Negative control: an empty (zero) estimate, never a physics state write."""
    def _states(self, observation):
        return [np.zeros(6) for _ in range(self.context.task.spacecraft_count)]


METHODS = {"gnss-orbit-estimator-v1": GNSSOrbitEstimator, "gnss-measurement-only-v1": GNSSMeasurementOnly,
           "zero-navigation-estimate-v1": ZeroNavigationEstimate}
