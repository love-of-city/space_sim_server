"""Estimated-state optical-depth approach and platform release state machine.

This module owns references, never physical state. Nodes must first be frozen
by real-image prevalidation; a candidate configuration cannot release a flight.
"""
import math
import numpy as np

from simulation.capture26_mission import Capture26Mission, desired_geometry, stopping_distance
from simulation.pose_navigation import log
from simulation.platform_stability import PlatformStability


class Capture26Approach:
    def __init__(self, service_com, target_com, vision, *, initial_depth_m=110., config=None):
        self.config = config or Capture26Mission()
        self.service_com, self.target_com, self.vision = np.array(service_com), np.array(target_com), vision
        c = self.config
        offset, _ = desired_geometry(0., np.eye(3), self.service_com, self.target_com, vision)
        # Solve |offset - depth*Y| = 50 m, using the real compiled COMs.
        transverse = offset[0]**2+offset[2]**2
        if transverse >= c.com_hold_m**2:
            raise ValueError('50 m COM node is incompatible with model geometry')
        com_depth = -offset[1]+math.sqrt(c.com_hold_m**2-transverse)
        self.nodes = [('COM50m', com_depth), ('GNSS17p4m', c.gnss_hold_depth_m),
                      ('VisualAcquire', c.acquisition_depth_m), ('VisualReady', c.readiness_depth_m),
                      ('PlatformStandoff', c.terminal_depth_m if c.terminal_depth_m is not None else c.readiness_depth_m)]
        if initial_depth_m < com_depth:
            raise ValueError('near-flight entry must begin outside the 50 m COM node')
        self.depth, self.rate = float(initial_depth_m), 0.
        self.node = 0
        self.phase, self.reason = self.nodes[0][0], 'approaching node'
        self.dwell_start = self.wait_start = self.last_time_ns = None
        self.released = False
        self.safe_hold = False
        self.audit = []
        self.stability = PlatformStability(c.dwell_s)

    def _hold(self, reason):
        self.safe_hold, self.released = True, False
        self.phase, self.reason = 'SafeHold', reason
        self.dwell_start = None

    def snapshot(self):
        from copy import deepcopy
        return dict(schema='space-sim-capture26-approach/1', configuration=self.config.to_dict(),
            stability=self.stability.snapshot(), state=deepcopy({k: getattr(self, k) for k in ('depth', 'rate', 'node', 'phase', 'reason',
                'dwell_start', 'wait_start', 'last_time_ns', 'released', 'safe_hold', 'audit')}))

    def restore(self, document):
        from copy import deepcopy
        if document.get('schema') != 'space-sim-capture26-approach/1' or document.get('configuration') != self.config.to_dict():
            raise ValueError('capture26 approach configuration changed')
        state = deepcopy(document['state'])
        self.stability.restore(document['stability'])
        if (set(state) != set(self.snapshot()['state']) or type(state['node']) is not int
                or not 0 <= state['node'] < len(self.nodes)
                or not np.isfinite([state['depth'], state['rate']]).all() or state['depth'] <= 0
                or any(t is not None and (type(t) is not int or t < 0)
                       for t in (state['dwell_start'], state['wait_start'], state['last_time_ns']))):
            raise ValueError('invalid capture26 approach checkpoint')
        for key, value in state.items():
            setattr(self, key, value)

    def step(self, now_ns, state, *, position_sigma_m, visual_ready, visual_available,
             gyro_healthy, gnss_healthy, communication_healthy, observation_age_s,
             mass_kg, attitude_sigma_rad=0., process_healthy=True):
        now_ns = int(now_ns)
        if self.last_time_ns is not None and now_ns <= self.last_time_ns:
            raise ValueError('guidance must consume distinct increasing physical epochs')
        dt = 0. if self.last_time_ns is None else (now_ns-self.last_time_ns)*1e-9
        if dt > .15:
            self._hold('guidance updates interrupted')
        self.last_time_ns = now_ns
        c = self.config
        healthy = bool(all(gyro_healthy) and gnss_healthy and communication_healthy)
        healthy = healthy and bool(process_healthy)
        if not communication_healthy:
            self._hold('cooperative communication unhealthy')
        if self.node >= 2 and (not c.vision_nodes_validated or c.terminal_depth_m is None):
            self._hold('actual UE camera/node prevalidation is pending')
        name, target_depth = self.nodes[self.node]
        nominal_offset, r_desired = desired_geometry(self.depth, state['r'][1], self.service_com, self.target_com, self.vision)
        axis = -state['r'][1][:, 1]
        actual_offset = state['p'][0]-state['p'][1]
        optical_depth = self.depth+float((nominal_offset-actual_offset)@axis)
        omega = state['r'][1]@(state['rates'][1]-state['b'][1])
        com_rates = state.get('com_velocity_body', np.zeros((2, 3)))
        lever_velocity = r_desired@com_rates[0]-state['r'][1]@com_rates[1]
        relative_velocity = state['v'][0]-state['v'][1]-np.cross(omega, actual_offset)-lever_velocity
        closing_speed = max(0., float(relative_velocity@axis))
        latency = max(0., float(observation_age_s))
        stop = stopping_distance(closing_speed, position_sigma_m, latency,
                                 c.force_limit_n/mass_kg, reserve=c.thrust_reserve)
        # GNSS may take the platform to the acquisition reference without a
        # successful image. Preserve braking margin to the next, nearer node.
        boundary = c.readiness_depth_m if not visual_ready else self.nodes[-1][1]
        visual_limited = (self.node >= 3 and not visual_ready or
                          self.node == 2 and optical_depth-boundary <= stop and not visual_available)
        if visual_limited or not healthy:
            # Ordinary visual loss keeps the parked reference and platform dwell.
            # Brake in place; a fresh qualified streak can recover immediately.
            self.rate = 0.
            self.reason = 'visual readiness restricted' if visual_limited else 'platform health restricted'
            if self.wait_start is None:
                self.wait_start = now_ns
        elif not self.safe_hold:
            remaining = max(0., self.depth-target_depth)
            max_rate = .1 if self.node == 0 else .04 if self.node == 1 else .01
            acceleration = min(.01, c.force_limit_n/mass_kg*c.thrust_reserve)
            desired_rate = -min(max_rate, math.sqrt(2*acceleration*remaining))
            self.rate += float(np.clip(desired_rate-self.rate, -acceleration*dt, acceleration*dt))
            self.depth = max(target_depth, self.depth+self.rate*dt)
            if self.depth == target_depth:
                self.rate = 0.
        elif self.safe_hold:
            self.rate = 0.
        offset, r_desired = desired_geometry(self.depth, state['r'][1], self.service_com, self.target_com, self.vision)
        position_goal = state['p'][1]+offset
        velocity_goal = state['v'][1]+np.cross(omega, offset)+lever_velocity-self.rate*axis
        error = position_goal-state['p'][0]
        attitude_error = float(np.linalg.norm(log(r_desired@state['r'][0].T)))
        platform = self.stability.update(now_ns, ['optical_depth_m', self.depth], dict(
            position=float(np.linalg.norm(error)) <= (.006 if self.node == 4 else .05),
            velocity=float(np.linalg.norm(relative_velocity)) <= .002,
            attitude=attitude_error < math.radians(.3),
            position_uncertainty=position_sigma_m < .04,
            attitude_uncertainty=attitude_sigma_rad < math.radians(.3),
            health=healthy, reference_at_node=self.depth == target_depth, safety=not self.safe_hold))
        self.dwell_start = self.stability.start_ns
        quality = visual_ready if self.node >= 3 else visual_available if self.node == 2 else True
        qualified = platform['platform_stable'] and quality and not self.safe_hold
        self.released = False
        if qualified:
            if not self.audit or self.audit[-1]['node'] != name:
                self.audit.append(dict(node=name, release_time_ns=now_ns,
                    dwell_s=platform['platform_stable_duration_s'], optical_depth_m=optical_depth))
            if self.node == 4:
                self.released = True
                self.phase, self.reason = 'NearParked', 'platform stable and current visual ready'
            else:
                self.node += 1
                self.phase, self.reason = self.nodes[self.node][0], 'approaching node'
                self.wait_start = None
        elif not self.safe_hold:
            self.phase = 'VisualRestricted' if visual_limited else name
            if not healthy:
                self.phase = 'PlatformRestricted'
        return dict(position_goal=position_goal, velocity_goal=velocity_goal, rotation_goal=r_desired,
                    phase=self.phase, reason=self.reason, node=name, optical_depth_m=optical_depth,
                    stopping_distance_m=stop, visual_safety_boundary_m=boundary,
                    released=self.released, healthy=healthy, visual_ready=bool(visual_ready),
                    **platform,
                    dwell_s=0. if self.dwell_start is None else (now_ns-self.dwell_start)*1e-9)
