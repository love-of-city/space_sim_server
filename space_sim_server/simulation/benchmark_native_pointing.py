"""Original SARM wheel dynamics, with separate public GNSS and private COM evidence."""
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from simulation.benchmark_native_attitude import NativeAttitudeEnvironment, rotation_quaternion
from simulation.benchmark_pointing_methods import line_of_sight, matrix_to_quaternion, reference_frame
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_pointing import PointingContext, PointingEphemeris, PointingObservation, PointingTruthSample
from space_arm_platform.benchmark_pointing_score import PointingScorer


class NativePointingEnvironment(NativeAttitudeEnvironment):
    context_type = PointingContext
    scorer_type = PointingScorer

    def _setup_additional_observation(self, ctx, seed):
        if self._orbit.epoch_utc != self.task.ephemeris_epoch_utc:
            raise ValueError("pointing task epoch differs from the physical SPICE epoch")
        self._com_reader = ctx.ports.site_state("orbital_thrust_center").addSubscriber()
        self._earth_reader = ctx.ports.planet_state("earth").addSubscriber()
        self._sun_reader = ctx.ports.planet_state("sun").addSubscriber()
        self._gnss_rng = np.random.default_rng(np.random.SeedSequence([seed, 0]))

    def _ephemeris_sample(self, nanos):
        if int(self._earth_reader.timeWritten()) != nanos or int(self._sun_reader.timeWritten()) != nanos:
            raise RuntimeError("stale shared ephemeris cannot guide or grade pointing")
        earth, sun = self._earth_reader(), self._sun_reader()
        if max(abs(v) for v in (*earth.PositionVector, *earth.VelocityVector)) > 1e-8:
            raise RuntimeError("pointing requires the declared geocentric J2000 origin")
        return PointingEphemeris(earth_j2000_to_fixed=earth.J20002Pfix,
            earth_j2000_to_fixed_rate_s=earth.J20002Pfix_dot,
            sun_geocentric_j2000=(*sun.PositionVector, *sun.VelocityVector))

    def _com_sample(self, nanos):
        if int(self._com_reader.timeWritten()) != nanos:
            raise RuntimeError("stale COM publication cannot be a pointing GNSS sample")
        state = self._com_reader()
        return np.r_[state.r_BN_N, state.v_BN_N]

    def _initial_state(self, seed):
        initial = self.task.initial
        radius = EARTH_RADIUS+initial.altitude_m
        theta, inclination = np.deg2rad([initial.true_anomaly_deg, initial.inclination_deg])
        radial = np.array([np.cos(theta), np.sin(theta)*np.cos(inclination), np.sin(theta)*np.sin(inclination)])
        tangent = np.array([-np.sin(theta), np.cos(theta)*np.cos(inclination), np.cos(theta)*np.sin(inclination)])
        state = np.r_[radius*radial, np.sqrt(MU/radius)*tangent]
        direction, rate = line_of_sight(self.task.objective, state, self._ephemeris_sample(0))
        frame, _ = reference_frame(direction, rate)
        reference = matrix_to_quaternion(frame)
        rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))
        azimuth = rng.uniform(-np.pi, np.pi)
        axis_body = np.array([np.cos(azimuth), np.sin(azimuth), 0.])
        delta = rotation_quaternion(axis_body*rng.uniform(initial.rotation_min_rad, initial.rotation_max_rad))
        quaternion = rbk.C2EP(rbk.EP2C(delta)@rbk.EP2C(reference))
        quaternion /= np.linalg.norm(quaternion)
        omega = rng.uniform(-initial.body_rate_half_width_rad_s, initial.body_rate_half_width_rad_s, 3)
        return state, quaternion, omega

    def _truth_sample(self, nanos):
        base = super()._truth_sample(nanos)
        return PointingTruthSample(**base.model_dump(), chaser_com_j2000=tuple(self._com_sample(nanos)),
                                   ephemeris=self._ephemeris_sample(nanos))

    def _measure(self, sequence):
        attitude = super()._measure(sequence)
        physical = self._com_sample(self._time_ns)
        settings = self.task.observation
        noise = self._gnss_rng.normal(size=6)*np.array([settings.position_sigma_m]*3+[settings.velocity_sigma_m_s]*3)
        return PointingObservation(**attitude.model_dump(exclude={"schema_version"}),
            chaser_gnss_j2000=tuple(physical+noise), ephemeris=self._ephemeris_sample(self._time_ns))

    def provenance(self):
        result = super().provenance()
        result.update(observation_model="noisy attitude, gyro and independent COM GNSS; known shared SPICE Sun/Earth ephemeris; no spacecraft truth or images",
            pointing_model="body +Z boresight; geocentric spherical ground without terrain; actual SPICE IAU_EARTH rotation; continuous required window")
        return result
