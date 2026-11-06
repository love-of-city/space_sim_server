"""GNSS-only hold, approach and latched risk-triggered retreat guidance."""
import numpy as np

from simulation.benchmark_methods import NoThrust
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import gravity
from space_arm_platform.benchmark_contract import ForceAction


class GNSSRelativeFlight:
    def reset(self, context):
        self.context = context
        self.initial_estimate = None
        self.abort_time_ns = None

    def act(self, observation):
        task = self.context.task
        target = np.asarray(observation.target_estimate_j2000)
        chaser = np.asarray(observation.chaser_estimate_j2000)
        frame, omega = vbar_frame(target)
        if self.initial_estimate is None:
            self.initial_estimate = frame.T @ (chaser[:3]-target[:3])
        elapsed_s = observation.sample_time_ns*1e-9
        if task.path_requirement.kind == 'abort_and_retreat':
            displacement = chaser[:3]-target[:3]
            position = frame.T @ displacement
            relative_velocity = frame.T @ (chaser[3:]-target[3:]-np.cross(omega, displacement))
            # Linear closest-approach warning from the public GNSS estimates.
            # It is a controller heuristic, never the physical success grader.
            dot = float(position@relative_velocity)
            closest_time = np.clip(-dot/max(float(relative_velocity@relative_velocity), 1e-15),
                                   0., task.path_requirement.prediction_horizon_s)
            if (self.abort_time_ns is None and dot < 0
                    and np.linalg.norm(position+closest_time*relative_velocity) < task.safety.minimum_range_m):
                self.abort_time_ns = observation.sample_time_ns
                self.initial_estimate = position.copy()
            if self.abort_time_ns is None:
                return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                                   force_j2000_n=(0., 0., 0.))
            elapsed_s = (observation.sample_time_ns-self.abort_time_ns)*1e-9
        reference = np.asarray(task.objective.position_tnr_m)
        velocity, acceleration = np.zeros(3), np.zeros(3)
        if task.path_requirement.kind in {'axis_approach_corridor', 'abort_and_retreat', 'far_to_proximity_rendezvous'}:
            duration = .75*(task.duration_s-task.objective.dwell_s)
            u = min(1., elapsed_s/duration)
            s = 10*u**3-15*u**4+6*u**5
            ds = (30*u**2-60*u**3+30*u**4)/duration
            dds = (60*u-180*u**2+120*u**3)/duration**2
            change = reference-self.initial_estimate
            reference = self.initial_estimate+s*change
            velocity, acceleration = ds*change, dds*change
        offset, rotating_velocity = frame @ reference, frame @ velocity
        desired_position = target[:3]+offset
        desired_velocity = target[3:]+np.cross(omega, offset)+rotating_velocity
        feedforward = (gravity(target[:3])+np.cross(omega, np.cross(omega, offset))
                       +2*np.cross(omega, rotating_velocity)+frame @ acceleration-gravity(chaser[:3]))
        force = self.context.chaser_mass_kg*(feedforward+.0025*(desired_position-chaser[:3])+.1*(desired_velocity-chaser[3:]))
        force *= min(1., task.safety.maximum_force_n/max(np.linalg.norm(force), 1e-15))
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))

    def close(self):
        pass


class UnsafeRelativeFlight(GNSSRelativeFlight):
    """Legal force in the normal direction: breaks corridor or speed safety."""
    def act(self, observation):
        frame, _ = vbar_frame(np.asarray(observation.target_estimate_j2000))
        force = frame[:, 1]*self.context.task.safety.maximum_force_n
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))


METHODS = {'gnss-relative-flight-v1': GNSSRelativeFlight, 'no-relative-thrust-v1': NoThrust,
           'unsafe-relative-flight-v1': UnsafeRelativeFlight}
