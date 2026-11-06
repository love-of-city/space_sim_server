"""GNSS feedback around a circular T-R reference; no physical-truth inputs."""
import math

import numpy as np

from simulation.benchmark_methods import NoThrust
from simulation.benchmark_relative_flight_methods import UnsafeRelativeFlight
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import gravity
from space_arm_platform.benchmark_contract import ForceAction


class GNSSFlyAround:
    def reset(self, context):
        self.context = context
        self.start_angle = None

    def act(self, observation):
        task = self.context.task
        target = np.asarray(observation.target_estimate_j2000)
        chaser = np.asarray(observation.chaser_estimate_j2000)
        frame, omega = vbar_frame(target)
        if self.start_angle is None:
            position = frame.T @ (chaser[:3]-target[:3])
            self.start_angle = math.atan2(position[2], position[0])
        # Guidance choice only: competitors may choose any safe timing/path
        # satisfying the annulus and signed winding contract.
        rate = 2*math.pi/(self.context.task.duration_s*5/6)
        phase = self.start_angle+rate*observation.sample_time_ns*1e-9
        radius = task.objective.radius_m
        reference = radius*np.array([math.cos(phase), 0., math.sin(phase)])
        velocity = radius*rate*np.array([-math.sin(phase), 0., math.cos(phase)])
        acceleration = -rate**2*reference
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


METHODS = {'gnss-flyaround-v1': GNSSFlyAround, 'no-flyaround-thrust-v1': NoThrust,
           'unsafe-flyaround-v1': UnsafeRelativeFlight}
