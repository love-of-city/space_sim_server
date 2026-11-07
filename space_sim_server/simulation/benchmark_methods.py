"""Replaceable methods using only the public autonomy observation contract."""
from __future__ import annotations

import numpy as np

from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import gravity
from space_arm_platform.benchmark_contract import ForceAction, MethodContext, NavigationObservation


class GNSSWaypointPD:
    """Model-based feedforward plus critically damped PD on GNSS estimates.

Point-mass feedforward is an explicit baseline approximation even when the
environment contains J2, Sun gravity, drag and solar pressure. No scene or truth
provider is accepted. Attitude is not controlled by this method.
"""
    def reset(self, context: MethodContext) -> None:
        self.context = context

    def act(self, observation: NavigationObservation) -> ForceAction:
        target = np.asarray(observation.target_estimate_j2000)
        chaser = np.asarray(observation.chaser_estimate_j2000)
        frame, omega = vbar_frame(target)
        offset = frame @ self.context.task.objective.position_tnr_m
        desired_position = target[:3] + offset
        desired_velocity = target[3:] + np.cross(omega, offset)
        acceleration = (
            gravity(target[:3]) + np.cross(omega, np.cross(omega, offset))
            - gravity(chaser[:3])
            + 0.0025 * (desired_position - chaser[:3])
            + 0.1 * (desired_velocity - chaser[3:])
        )
        force = self.context.chaser_mass_kg * acceleration
        force *= min(1.0, self.context.task.safety.maximum_force_n / max(np.linalg.norm(force), 1e-15))
        return ForceAction(episode_id=observation.episode_id,
                           observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))

    def close(self) -> None:
        pass


class NoThrust(GNSSWaypointPD):
    def act(self, observation: NavigationObservation) -> ForceAction:
        return ForceAction(episode_id=observation.episode_id,
                           observation_sequence=observation.sequence,
                           force_j2000_n=(0.0, 0.0, 0.0))


class UnsafeApproach(GNSSWaypointPD):
    """Negative control: legal thrust aimed at the target, ignoring safety."""
    def act(self, observation: NavigationObservation) -> ForceAction:
        direction = np.subtract(observation.target_estimate_j2000[:3], observation.chaser_estimate_j2000[:3])
        force = direction * self.context.task.safety.maximum_force_n / np.linalg.norm(direction)
        return ForceAction(episode_id=observation.episode_id,
                           observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))


METHODS = {"gnss-pd-v1": GNSSWaypointPD, "no-thrust-v1": NoThrust,
           "unsafe-approach-v1": UnsafeApproach}
