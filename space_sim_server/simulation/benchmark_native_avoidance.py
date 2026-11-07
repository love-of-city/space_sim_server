"""Actual two-body propagation, self GNSS and independently noisy ground catalog."""
import math
import numpy as np

from simulation.benchmark_avoidance_model import prepare_avoidance_model
from simulation.benchmark_native_orbit import NativeOrbitEnvironment
from space_arm_platform.benchmark_avoidance import AvoidanceContext, AvoidanceObservation, AvoidanceTruthSample
from space_arm_platform.benchmark_avoidance_score import AvoidanceScorer, tnr_frame


class NativeAvoidanceEnvironment(NativeOrbitEnvironment):
    context_type = AvoidanceContext
    scorer_type = AvoidanceScorer

    def _prepare_model(self):
        return prepare_avoidance_model(self.repository, self.episode_directory/'model', self.task)

    def _environment_bodies(self, options):
        return super()._environment_bodies(options)+[('benchmark_debris', 'benchmark_debris_center',
            self.task.encounter.mass_kg, math.pi*self.task.encounter.radius_m**2)]

    def _setup_observation(self, ctx):
        super()._setup_observation(ctx)
        self._debris_reader = ctx.ports.site_state('benchmark_debris_center').addSubscriber()

    def _initialize_additional_bodies(self, state, seed):
        settings = self.task.encounter
        rng = np.random.default_rng(np.random.SeedSequence([seed,2]))
        self._catalog_rng = np.random.default_rng(np.random.SeedSequence([seed,3]))
        p = np.asarray(settings.relative_position_tnr_m)+rng.uniform(-settings.position_half_width_m,settings.position_half_width_m,3)
        v = np.asarray(settings.relative_velocity_tnr_m_s)+rng.uniform(-settings.velocity_half_width_m_s,settings.velocity_half_width_m_s,3)
        basis, omega = tnr_frame(state)
        delta = basis@p
        debris = np.r_[state[:3]+delta,state[3:]+basis@v+np.cross(omega,delta)]
        body = self._scene.getBody('benchmark_debris')
        body.setPosition(debris[:3].tolist())
        body.setVelocity(debris[3:].tolist())
        body.setAttitude([0.,0.,0.])
        body.setAttitudeRate([0.,0.,0.])
        return [debris.tolist()]

    def _debris_state(self, nanos):
        if int(self._debris_reader.timeWritten()) != nanos:
            raise RuntimeError('stale debris publication cannot be used as evidence or sensor input')
        state = self._debris_reader()
        return np.r_[state.r_BN_N,state.v_BN_N]

    def _truth_sample(self, nanos):
        original = super()._truth_sample(nanos)
        return AvoidanceTruthSample(**original.model_dump(), debris_com_j2000=tuple(self._debris_state(nanos)))

    def _measure(self, sequence):
        own = super()._measure(sequence)
        sensor = self.task.observation
        noise = np.array([sensor.catalog_position_sigma_m]*3+[sensor.catalog_velocity_sigma_m_s]*3)
        catalog = self._debris_state(self._time_ns)+self._catalog_rng.normal(size=6)*noise
        return AvoidanceObservation(**own.model_dump(exclude={'schema_version'}),
            debris_catalog_j2000=tuple(catalog), catalog_sample_time_ns=self._time_ns,
            catalog_delivery_time_ns=self._time_ns)

    def provenance(self):
        report = super().provenance()
        report.update(observation_model='self noisy GNSS and orbit filter; independent 2 Hz external noisy debris catalog; zero catalog latency; no debris transmitter',
            catalog_model='instantaneous isotropic Gaussian position/velocity errors; continuous ground coverage assumed; no CDM covariance or collision probability model',
            passive_object='separate native free body with BSK gravity, drag and SRP; no actuators',
            risk_model='fixed deterministic miss-distance safety shell over the entire declared horizon')
        return report
