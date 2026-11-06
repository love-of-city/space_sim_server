"""A real receiver-delivery outage over the original freely propagated SARM."""
import numpy as np

from simulation.benchmark_native_navigation import NativeAbsoluteNavigation
from space_arm_platform.benchmark_navigation_recovery import (
    GNSSOutageRealization, NavigationRecoveryContext, NavigationRecoveryDelivery,
    NavigationRecoveryObservation, NavigationRecoveryTruthSample,
)
from space_arm_platform.benchmark_navigation_recovery_score import NavigationRecoveryScorer


class NativeNavigationRecoveryEnvironment(NativeAbsoluteNavigation):
    context_type = NavigationRecoveryContext
    scorer_type = NavigationRecoveryScorer

    def reset(self, seed):
        self.task.seeds.split_for(seed)
        rng = np.random.default_rng(np.random.SeedSequence([seed, 7]))
        fault, period = self.task.fault, self.task.decision_period_ns
        onset = int(rng.integers(round(fault.earliest_onset_s*1e9/period), round(fault.latest_onset_s*1e9/period)+1))*period
        duration = int(rng.integers(round(fault.minimum_duration_s*1e9/period), round(fault.maximum_duration_s*1e9/period)+1))*period
        self._outage = GNSSOutageRealization(onset_time_ns=onset, return_time_ns=onset+duration)
        self._outage.validate_for(self.task)
        return super().reset(seed)

    def _truth_sample(self, nanos):
        sample = super()._truth_sample(nanos)
        return NavigationRecoveryTruthSample(**sample.model_dump(), outage=self._outage,
            receiver_available=self._outage.available(nanos))

    def _measure(self, sequence):
        physical = self._physical_state(self._time_ns)
        sigma = np.array([self.task.observation.position_sigma_m]*3+[self.task.observation.velocity_sigma_m_s]*3)
        # Keep the receiver random stream advancing while solutions are absent.
        # Neither the hidden sample nor a posterior is sent to the method.
        measured = physical+self._sensor_rng.normal(size=6)*sigma
        available = self._outage.available(self._time_ns)
        observation = NavigationRecoveryObservation(episode_id=self.episode_id, sequence=sequence,
            sample_time_ns=self._time_ns, delivery_time_ns=self._time_ns, receiver_available=available,
            chaser_measurement_j2000=tuple(measured) if available else None, target_measurement_j2000=None)
        delivered = NavigationRecoveryDelivery(sequence=sequence, sample_time_ns=self._time_ns,
            delivery_time_ns=self._time_ns, receiver_available=available,
            measurement_j2000=observation.chaser_measurement_j2000)
        self._scorer.add(delivered)
        self._truth_stream.write(delivered.model_dump_json()+'\n')
        return observation

    def provenance(self):
        value = super().provenance()
        value.update(observation_model='one noisy GNSS receiver; realized complete solution outage with null delivery; no environment filter',
            receiver_outage=self._outage.model_dump(mode='json'),
            fault_scope='temporary GNSS solution unavailability; not a physical receiver hardware/thermal model')
        return value
