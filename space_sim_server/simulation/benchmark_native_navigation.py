"""Navigation-only adapters over the existing single/two-spacecraft native scenes."""
import numpy as np

from simulation.benchmark_native_orbit import NativeOrbitEnvironment
from simulation.benchmark_native_flight import NativeWaypointEnvironment
from space_arm_platform.benchmark_contract import ForceAction
from space_arm_platform.benchmark_navigation import GNSSMeasurements, NavigationContext, NavigationEstimateEvidence
from space_arm_platform.benchmark_navigation_score import NavigationScorer


class _NavigationEvaluation:
    context_type = NavigationContext
    scorer_type = NavigationScorer

    def _setup_observation(self, ctx):
        # Filtering belongs to the evaluated method. No environment filter or
        # truth-initialized posterior is constructed for these configurations.
        pass

    def _measure(self, sequence):
        physical = (self._physical_state(self._time_ns)[None, :] if self.task.source_id == "S14"
                    else self._read_physical_states(self._time_ns))
        noise = np.array([self.task.observation.position_sigma_m]*3+[self.task.observation.velocity_sigma_m_s]*3)
        measured = physical+self._sensor_rng.normal(size=physical.shape)*noise
        chaser = measured[0] if self.task.source_id == "S14" else measured[1]
        target = None if self.task.source_id == "S14" else tuple(measured[0])
        return GNSSMeasurements(episode_id=self.episode_id, sequence=sequence,
            sample_time_ns=self._time_ns, delivery_time_ns=self._time_ns,
            chaser_measurement_j2000=tuple(chaser), target_measurement_j2000=target)

    def step(self, action):
        if self._simulation is None or self._observation is None:
            raise RuntimeError("reset must precede step")
        if self.evaluation()["outcome"] != "incomplete":
            raise RuntimeError("episode has terminated; reset before another step")
        action.validate_for(self._observation, self.task)
        record = NavigationEstimateEvidence(sequence=action.observation_sequence, estimate_time_ns=action.estimate_time_ns,
                                             reference_frame=action.reference_frame, estimated_state=action.estimated_state)
        # Log the method output next to the already-recorded, same-time truth.
        # The method never receives this private mixed evidence stream.
        self._scorer.add(record)
        self._truth_stream.write(record.model_dump_json()+"\n")
        return super().step(ForceAction(episode_id=self.episode_id, observation_sequence=self._observation.sequence,
                                       force_j2000_n=(0., 0., 0.)))

    def provenance(self):
        value = super().provenance()
        value.update(observation_model="independent noisy GNSS COM solutions; no environment filter; cooperative target solution only in D01",
                     actuation_model="state-estimation evaluation only; all translation and wheel motor commands held at zero",
                     estimate_frame=self.task.reference_frame,
                     estimate_clock="one estimate per observation from t=0 through duration-dt, before physical advance")
        return value


class NativeAbsoluteNavigation(_NavigationEvaluation, NativeOrbitEnvironment):
    pass


class NativeRelativeNavigation(_NavigationEvaluation, NativeWaypointEnvironment):
    pass


class NativeNavigationEnvironment:
    def __new__(cls, task, output_directory):
        environment = NativeAbsoluteNavigation if task.source_id == "S14" else NativeRelativeNavigation
        return environment(task, output_directory)
