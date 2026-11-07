"""Declared finite-burn execution and GNSS-only circular-orbit transfer baselines."""
import numpy as np

from simulation.orbital_rendezvous.reference import blend, gravity
from space_arm_platform.benchmark_contract import ForceAction
from space_arm_platform.benchmark_orbit import EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS, ImpulseObjective, OrbitPlaneObjective, OrbitMaintenanceObjective, orbit_plane_normal

_X, _W = np.polynomial.legendre.leggauss(32)


class GNSSOrbitManeuver:
    def reset(self, context):
        self.context = context
        self.initial = None
        self.commanded_delta_v = np.zeros(3)
        self.coasting = False

    def _plane_reference_normal(self):
        goal = self.context.task.objective
        return orbit_plane_normal(goal.inclination_deg, goal.raan_deg)

    def act(self, observation):
        task, mass = self.context.task, self.context.chaser_mass_kg
        goal = task.objective
        state = np.asarray(observation.chaser_estimate_j2000)
        time = observation.sample_time_ns*1e-9
        dt = task.decision_period_ns*1e-9
        if isinstance(goal, ImpulseObjective):
            # An explicitly ideal calibrated actuator: no private force telemetry.
            # The independent grader uses actual native readbacks, not this integral.
            force = np.zeros(3)
            if goal.earliest_burn_s <= time < goal.latest_burn_s:
                residual = np.asarray(goal.delta_velocity_j2000_m_s)-self.commanded_delta_v
                force = mass*residual/dt
                force *= min(1., 0.8*task.safety.maximum_force_n/max(np.linalg.norm(force), 1e-15))
                self.commanded_delta_v += force*dt/mass
        elif isinstance(goal, OrbitPlaneObjective):
            from simulation.benchmark_plane_reference import plane_reference
            if self.initial is None:
                self.initial = state.copy()
            duration = .7*(task.duration_s-goal.dwell_s)
            position, velocity, acceleration = plane_reference(self.initial, goal.altitude_m,
                self._plane_reference_normal(), time, duration)
            force = mass*(acceleration-gravity(state[:3])+.0025*(position-state[:3])+.1*(velocity-state[3:]))
            if time >= duration:
                radius = np.linalg.norm(state[:3])
                momentum = np.cross(state[:3], state[3:])
                normal = momentum/np.linalg.norm(momentum)
                target = orbit_plane_normal(goal.inclination_deg, goal.raan_deg)
                plane_error = np.arctan2(np.linalg.norm(np.cross(normal,target)),normal@target)
                energy = state[3:]@state[3:]/2-MU/radius
                semimajor = -MU/(2*energy) if energy < 0 else np.inf
                eccentricity = np.linalg.norm(np.cross(state[3:],momentum)/MU-state[:3]/radius)
                if (abs(semimajor-RADIUS-goal.altitude_m) < .5*goal.semimajor_axis_tolerance_m
                        and eccentricity < .5*goal.maximum_eccentricity and plane_error < .5*goal.plane_tolerance_rad):
                    self.coasting = True
            if self.coasting:
                force = np.zeros(3)
            force *= min(1.,task.safety.maximum_force_n/max(np.linalg.norm(force),1e-15))
        elif isinstance(goal, OrbitMaintenanceObjective):
            if self.initial is None:
                self.initial = state.copy()
            radial0 = self.initial[:3]/np.linalg.norm(self.initial[:3])
            normal = np.cross(self.initial[:3], self.initial[3:])
            normal /= np.linalg.norm(normal)
            tangent0 = np.cross(normal, radial0)
            radius = RADIUS+goal.altitude_m
            rate = np.sqrt(MU/radius**3)
            phase = rate*time
            radial = np.cos(phase)*radial0+np.sin(phase)*tangent0
            tangent = -np.sin(phase)*radial0+np.cos(phase)*tangent0
            desired_position, desired_velocity = radius*radial, radius*rate*tangent
            # Continuous maintenance needs stronger disturbance rejection than
            # a finite transfer. These gains were selected on training episodes;
            # the public GNSS state is still the only feedback measurement.
            acceleration = (-radius*rate**2*radial-gravity(state[:3])
                            +.01*(desired_position-state[:3])+.2*(desired_velocity-state[3:]))
            force = mass*acceleration
            force *= min(1., task.safety.maximum_force_n/max(np.linalg.norm(force), 1e-15))
        else:
            if self.initial is None:
                self.initial = state.copy()
            duration = 0.7*(task.duration_s-goal.dwell_s)
            r0 = np.linalg.norm(self.initial[:3])
            r1 = RADIUS+goal.altitude_m
            radial0 = self.initial[:3]/r0
            normal = np.cross(self.initial[:3], self.initial[3:])
            normal /= np.linalg.norm(normal)
            tangent0 = np.cross(normal, radial0)
            u = min(time, duration)
            b, db, ddb = blend(u, duration)
            r, dr, ddr = r0+(r1-r0)*b, (r1-r0)*db, (r1-r0)*ddb
            nodes = 0.5*u*(1+_X)
            radii = r0+(r1-r0)*blend(nodes, duration)[0]
            theta = 0.5*u*np.dot(_W, np.sqrt(MU/radii**3))+max(0., time-duration)*np.sqrt(MU/r1**3)
            radial = np.cos(theta)*radial0+np.sin(theta)*tangent0
            tangent = -np.sin(theta)*radial0+np.cos(theta)*tangent0
            n = np.sqrt(MU/r**3)
            desired_position = r*radial
            desired_velocity = dr*radial+r*n*tangent
            desired_acceleration = (ddr-r*n*n)*radial+0.5*dr*n*tangent
            acceleration = desired_acceleration-gravity(state[:3])+0.0025*(desired_position-state[:3])+0.1*(desired_velocity-state[3:])
            force = mass*acceleration
            # Confirm an unpowered orbit. The latch is based solely on noisy GNSS;
            # the grader independently checks the physical orbit throughout dwell.
            if time >= duration:
                radius = np.linalg.norm(state[:3])
                h = np.cross(state[:3], state[3:])
                energy = state[3:]@state[3:]/2-MU/radius
                a = -MU/(2*energy) if energy < 0 else np.inf
                e = np.linalg.norm(np.cross(state[3:], h)/MU-state[:3]/radius)
                if abs(a-r1) < 0.5*goal.semimajor_axis_tolerance_m and e < 0.5*goal.maximum_eccentricity:
                    self.coasting = True
            if self.coasting:
                force = np.zeros(3)
            force *= min(1., task.safety.maximum_force_n/max(np.linalg.norm(force), 1e-15))
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))

    def close(self):
        pass


class NoOrbitThrust(GNSSOrbitManeuver):
    def act(self, observation):
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=(0., 0., 0.))


class UnsafeOrbitBurn(GNSSOrbitManeuver):
    def act(self, observation):
        # Legal force, deliberately ignores burn-window and resource constraints.
        velocity = np.asarray(observation.chaser_estimate_j2000[3:])
        force = velocity*self.context.task.safety.maximum_force_n/np.linalg.norm(velocity)
        return ForceAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                           force_j2000_n=tuple(force))


class PulsedOrbitBurn(GNSSOrbitManeuver):
    """Same impulse with four-second pulses and one-second interruptions."""
    def act(self, observation):
        task, mass = self.context.task, self.context.chaser_mass_kg
        goal = task.objective
        if not isinstance(goal, ImpulseObjective):
            raise ValueError('pulsed-burn control requires an impulse objective')
        time = observation.sample_time_ns*1e-9
        force = np.zeros(3)
        ticks = (observation.sample_time_ns-round(goal.earliest_burn_s*1e9))//task.decision_period_ns
        if goal.earliest_burn_s <= time < goal.latest_burn_s and ticks % 10 < 8:
            dt = task.decision_period_ns*1e-9
            force = mass*(np.asarray(goal.delta_velocity_j2000_m_s)-self.commanded_delta_v)/dt
            force *= min(1.,task.safety.maximum_force_n/max(np.linalg.norm(force),1e-15))
            self.commanded_delta_v += force*dt/mass
        return ForceAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,force_j2000_n=tuple(force))


class UnchangedOrbitPlane(GNSSOrbitManeuver):
    """Maintain the original orbit normal while ignoring the requested plane change."""
    def _plane_reference_normal(self):
        normal = np.cross(self.initial[:3],self.initial[3:])
        return normal/np.linalg.norm(normal)


METHODS = {"gnss-orbit-maneuver-v1": GNSSOrbitManeuver, "no-orbit-thrust-v1": NoOrbitThrust,
           "unsafe-orbit-burn-v1": UnsafeOrbitBurn, "pulsed-orbit-burn-v1": PulsedOrbitBurn,
           "unchanged-orbit-plane-v1": UnchangedOrbitPlane}
