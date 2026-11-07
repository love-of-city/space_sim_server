"""Two real native spacecraft, independent GNSS sensors and chaser wheel actuation."""
import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from simulation.benchmark_native_attitude import rotation_quaternion
from simulation.benchmark_native_pointing import NativePointingEnvironment
from simulation.benchmark_pointing_methods import matrix_to_quaternion, reference_frame
from simulation.benchmark_target_pointing_model import prepare_target_pointing_model
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_target_pointing import (
    TargetPointingContext, TargetPointingObservation, TargetPointingTruthSample,
)
from space_arm_platform.benchmark_target_pointing_score import TargetPointingScorer


class NativeTargetPointingEnvironment(NativePointingEnvironment):
    context_type = TargetPointingContext
    scorer_type = TargetPointingScorer
    model_builder = staticmethod(prepare_target_pointing_model)

    def _environment_bodies(self, options):
        return [*super()._environment_bodies(options),
            ('capture_target','target_flight_center',self.model_manifest['bodies']['capture_target']['mass_kg'],options.target_area_m2)]

    def _setup_additional_observation(self, ctx, seed):
        super()._setup_additional_observation(ctx, seed)
        self._target_reader = ctx.ports.site_state('target_flight_center').addSubscriber()
        self._target_gnss_rng = np.random.default_rng(np.random.SeedSequence([seed, 8]))

    def _target_sample(self, nanos):
        if int(self._target_reader.timeWritten()) != nanos:
            raise RuntimeError('stale target COM publication cannot be a GNSS or physical sample')
        sample = self._target_reader()
        return np.r_[sample.r_BN_N,sample.v_BN_N]

    def _initial_state(self, seed):
        initial, target_initial = self.task.initial, self.task.target_initial
        radius = EARTH_RADIUS+initial.altitude_m
        theta, inclination = np.deg2rad([initial.true_anomaly_deg,initial.inclination_deg])
        radial = np.array([np.cos(theta),np.sin(theta)*np.cos(inclination),np.sin(theta)*np.sin(inclination)])
        tangent = np.array([-np.sin(theta),np.cos(theta)*np.cos(inclination),np.cos(theta)*np.sin(inclination)])
        normal = np.cross(radial,tangent)
        state = np.r_[radius*radial,np.sqrt(MU/radius)*tangent]
        frame = np.column_stack((tangent,normal,radial))
        rng = np.random.default_rng(np.random.SeedSequence([seed,7]))
        position = np.asarray(target_initial.position_tnr_m)+rng.uniform(-target_initial.position_half_width_m,target_initial.position_half_width_m,3)
        velocity = np.asarray(target_initial.velocity_tnr_m_s)+rng.uniform(-target_initial.velocity_half_width_m_s,target_initial.velocity_half_width_m_s,3)
        dr = frame@position
        dv = frame@velocity+np.cross(np.sqrt(MU/radius**3)*normal,dr)
        self._initial_target = state+np.r_[dr,dv]
        direction = dr/np.linalg.norm(dr)
        direction_rate = (dv-direction*(direction@dv))/np.linalg.norm(dr)
        reference, _ = reference_frame(direction,direction_rate)
        reference = matrix_to_quaternion(reference)
        rng = np.random.default_rng(np.random.SeedSequence([seed,1]))
        azimuth = rng.uniform(-np.pi,np.pi)
        axis = np.array([np.cos(azimuth),np.sin(azimuth),0.])
        delta = rotation_quaternion(axis*rng.uniform(initial.rotation_min_rad,initial.rotation_max_rad))
        quaternion = rbk.C2EP(rbk.EP2C(delta)@rbk.EP2C(reference))
        quaternion /= np.linalg.norm(quaternion)
        omega = rng.uniform(-initial.body_rate_half_width_rad_s,initial.body_rate_half_width_rad_s,3)
        return state,quaternion,omega

    def _initialize_additional_bodies(self, seed, chaser_state):
        target = self._scene.getBody('capture_target')
        center = np.asarray(self.model_manifest['bodies']['capture_target']['center_body_m'])
        target.setPosition((self._initial_target[:3]-center).tolist())
        target.setVelocity(self._initial_target[3:].tolist())
        target.setAttitude([0.,0.,0.])
        target.setAttitudeRate([0.,0.,0.])
        return [self._initial_target.tolist()]

    def _truth_sample(self, nanos):
        sample = super()._truth_sample(nanos)
        return TargetPointingTruthSample(**sample.model_dump(),target_com_j2000=tuple(self._target_sample(nanos)))

    def _measure(self, sequence):
        sample = super()._measure(sequence)
        settings = self.task.observation
        noise = self._target_gnss_rng.normal(size=6)*np.array([settings.position_sigma_m]*3+[settings.velocity_sigma_m_s]*3)
        return TargetPointingObservation(**sample.model_dump(exclude={'schema_version'}),
            target_gnss_j2000=tuple(self._target_sample(self._time_ns)+noise))

    def provenance(self):
        result = super().provenance()
        result.update(observation_model='noisy chaser attitude/gyro and two independent noisy COM GNSS solutions; cooperative zero-delay target link',
            pointing_model='body +Z tracks actual freely propagated target COM; target has no actuators; neither spacecraft has translation actuators')
        return result
