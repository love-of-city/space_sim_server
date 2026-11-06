"""Centralized GNSS formation guidance with active force sharing across all craft."""
import numpy as np

from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import gravity
from space_arm_platform.benchmark_formation import FormationAction


def _formation_progress(time_s, duration_s):
    """A shared trapezoidal speed profile with smooth cosine acceleration ramps.

    All pairwise relative displacements follow the same progress, including
    follower-to-follower motion. Peak normalized speed is 1/(1-ramp), rather
    than the 1.875 peak of a quintic position interpolation.
    """
    u, ramp = max(0., min(1., time_s/duration_s)), .15
    if u == 1.:
        return 1., 0., 0.
    peak = 1/(1-ramp)
    remaining = 1-u if u > 1-ramp else u
    if remaining < ramp:
        angle = np.pi*remaining/ramp
        step = peak*(remaining/2-ramp*np.sin(angle)/(2*np.pi))
        rate = peak*(1-np.cos(angle))/(2*duration_s)
        accel = peak*np.pi*np.sin(angle)/(2*ramp*duration_s**2)
        return (1-step, rate, -accel) if u > 1-ramp else (step, rate, accel)
    return peak*(u-ramp/2), peak/duration_s, 0.


class GNSSFormation:
    def reset(self, context):
        self.context = context
        self.initial_estimate = None

    def act(self, observation):
        task = self.context.task
        states = np.asarray(observation.estimates_j2000)
        frame, omega = vbar_frame(states[0])
        if self.initial_estimate is None:
            self.initial_estimate = (states[:, :3]-states[0, :3]) @ frame
        references = np.asarray(task.objective.positions_tnr_m)
        velocities, accelerations = np.zeros_like(references), np.zeros_like(references)
        if task.objective.kind != 'hold_formation':
            duration = .94*(task.duration_s-task.objective.dwell_s)
            step, rate, accel = _formation_progress(observation.sample_time_ns*1e-9, duration)
            change = references-self.initial_estimate
            references = self.initial_estimate+step*change
            velocities, accelerations = rate*change, accel*change
        relative_control = np.zeros_like(references)
        for i in range(1, task.spacecraft_count):
            offset, rotating_velocity = frame @ references[i], frame @ velocities[i]
            desired_position = states[0, :3]+offset
            desired_velocity = states[0, 3:]+np.cross(omega, offset)+rotating_velocity
            feedforward = (gravity(states[0, :3])+np.cross(omega, np.cross(omega, offset))
                           +2*np.cross(omega, rotating_velocity)+frame @ accelerations[i]-gravity(states[i, :3]))
            relative_control[i] = feedforward+.0025*(desired_position-states[i, :3])+.1*(desired_velocity-states[i, 3:])
        masses = np.asarray(self.context.spacecraft_masses_kg)
        # Subtract one common acceleration from all participants. Relative
        # control is preserved while total requested formation force is zero;
        # spacecraft zero shares the maneuver instead of being a truth anchor.
        common = np.sum(relative_control*masses[:, None], axis=0)/sum(masses)
        forces = (relative_control-common)*masses[:, None]
        forces *= np.minimum(1., task.safety.maximum_force_n/np.maximum(np.linalg.norm(forces, axis=1), 1e-15))[:, None]
        return FormationAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                               forces_j2000_n=tuple(tuple(force) for force in forces))

    def close(self):
        pass


class NoFormationThrust(GNSSFormation):
    def act(self, observation):
        return FormationAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                               forces_j2000_n=((0., 0., 0.),)*self.context.task.spacecraft_count)


class OneSidedFormation(GNSSFormation):
    """Keep spacecraft zero passive while preserving the requested relative acceleration."""
    def act(self, observation):
        action = super().act(observation)
        masses = np.asarray(self.context.spacecraft_masses_kg)
        forces = np.asarray(action.forces_j2000_n)
        forces -= forces[0].copy()*masses[:, None]/masses[0]
        limit = self.context.task.safety.maximum_force_n
        forces *= np.minimum(1., limit/np.maximum(np.linalg.norm(forces, axis=1), 1e-15))[:, None]
        return FormationAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                               forces_j2000_n=tuple(tuple(force) for force in forces))


class UnsafeFormation(GNSSFormation):
    def act(self, observation):
        states = np.asarray(observation.estimates_j2000)
        direction = states[0, :3]-states[1, :3]
        direction /= np.linalg.norm(direction)
        forces = np.zeros((self.context.task.spacecraft_count, 3))
        forces[1] = direction*self.context.task.safety.maximum_force_n
        return FormationAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                               forces_j2000_n=tuple(tuple(force) for force in forces))


METHODS = {'gnss-formation-v1': GNSSFormation, 'no-formation-thrust-v1': NoFormationThrust,
           'unsafe-formation-v1': UnsafeFormation, 'one-sided-formation-v1': OneSidedFormation}
