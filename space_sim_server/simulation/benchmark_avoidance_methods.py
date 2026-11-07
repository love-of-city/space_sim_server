"""Public-observation collision screening and radial avoidance baseline."""
import numpy as np

from space_arm_platform.benchmark_contract import ForceAction


class CatalogAvoidance:
    def reset(self, context):
        self.context = context
        self.active = self.coasting = False

    def act(self, observation):
        task = self.context.task
        own, debris = np.asarray(observation.chaser_estimate_j2000), np.asarray(observation.debris_catalog_j2000)
        p, v = debris[:3]-own[:3], debris[3:]-own[3:]
        speed2 = float(v@v)
        tca = max(0., min(task.objective.screening_horizon_s, -float(p@v)/max(speed2,1e-12)))
        miss = float(np.linalg.norm(p+tca*v))
        force = np.zeros(3)
        if 0 < tca <= task.objective.screening_horizon_s and miss < 1.3*task.safety.minimum_range_m:
            self.active = True
        if self.active and miss >= 2.*task.safety.minimum_range_m:
            self.coasting = True
        if self.active and not self.coasting:
            # Radial impulse separates this declared cross-plane encounter.
            # No sampled initial realization, private state or outcome is read.
            force = task.safety.maximum_force_n*own[:3]/np.linalg.norm(own[:3])
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))

    def close(self):
        pass


class NoAvoidanceThrust(CatalogAvoidance):
    def act(self, observation):
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=(0.,0.,0.))


class UnsafeAvoidanceBurn(CatalogAvoidance):
    def act(self, observation):
        radial = np.asarray(observation.chaser_estimate_j2000[:3])
        force = self.context.task.safety.maximum_force_n*radial/np.linalg.norm(radial)
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))


METHODS = {'catalog-avoidance-v1': CatalogAvoidance, 'no-avoidance-thrust-v1': NoAvoidanceThrust,
           'unsafe-avoidance-burn-v1': UnsafeAvoidanceBurn}
