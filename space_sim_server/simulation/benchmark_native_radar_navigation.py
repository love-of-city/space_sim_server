"""Two real passive spacecraft; only observer navigation and active ranging leave the environment."""
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from simulation.benchmark_native_navigation import NativeRelativeNavigation
from simulation.benchmark_radar_sensor import measure_radar
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_radar_navigation import RadarDelivery, RadarNavigationContext
from space_arm_platform.benchmark_radar_navigation_score import RadarNavigationScorer


class NativeRadarNavigationEnvironment(NativeRelativeNavigation):
    context_type = RadarNavigationContext
    scorer_type = RadarNavigationScorer

    def _setup_observation(self, ctx):
        self._observer_body_reader = ctx.ports.body_state('cubesat_bus').addSubscriber()

    def _initial_states(self, seed):
        settings = self.task.initial
        rng = np.random.default_rng(np.random.SeedSequence([seed, 8]))
        radius = EARTH_RADIUS+settings.target_altitude_m+rng.uniform(-settings.target_altitude_half_width_m, settings.target_altitude_half_width_m)
        phase = np.deg2rad(settings.true_anomaly_deg+rng.uniform(-settings.target_anomaly_half_width_deg, settings.target_anomaly_half_width_deg))
        inclination = np.deg2rad(settings.inclination_deg+rng.uniform(-settings.target_inclination_half_width_deg, settings.target_inclination_half_width_deg))
        radial = np.array([np.cos(phase), np.sin(phase)*np.cos(inclination), np.sin(phase)*np.sin(inclination)])
        tangent = np.array([-np.sin(phase), np.cos(phase)*np.cos(inclination), np.cos(phase)*np.sin(inclination)])
        target = np.r_[radius*radial, np.sqrt(MU/radius)*tangent]
        frame, omega = vbar_frame(target)
        rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))
        position = np.asarray(settings.relative_position_tnr_m)+rng.uniform(-settings.position_half_width_m, settings.position_half_width_m, 3)
        velocity = np.asarray(settings.relative_velocity_tnr_m_s)+rng.uniform(-settings.velocity_half_width_m_s, settings.velocity_half_width_m_s, 3)
        displacement = frame@position
        return np.array([target, target+np.r_[displacement, frame@velocity+np.cross(omega, displacement)]])

    def _measure(self, sequence):
        states = self._read_physical_states(self._time_ns)
        if int(self._observer_body_reader.timeWritten()) != self._time_ns:
            raise RuntimeError('ranging requires contemporaneous observer orientation')
        observation = measure_radar(self.task.observation, states[0], states[1],
            rbk.MRP2C(self._observer_body_reader().sigma_BN).T, self._sensor_rng,
            episode_id=self.episode_id, sequence=sequence, nanos=self._time_ns)
        delivered = RadarDelivery(observation=observation.model_dump(exclude={'episode_id', 'schema_version'}))
        self._scorer.add(delivered)
        self._truth_stream.write(delivered.model_dump_json()+'\n')
        return observation

    def provenance(self):
        value = super().provenance()
        value.update(observation_model='noisy own GNSS and attitude; active range, body bearing and range rate; target transmits nothing',
            radar_scope='idealized omnidirectional target COM scatterer; no RF power, cross-section, multipath, antenna or detection model',
            target_telemetry_available=False, target_actuation='passive target with no command interface')
        return value
