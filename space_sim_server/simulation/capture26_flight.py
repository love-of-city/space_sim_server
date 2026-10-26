"""Distant flight references and physical-independent release decisions."""
import numpy as np

from simulation.orbital_rendezvous.reference import FlightProfile


class Capture26Flight:
    def __init__(self, scenario, *, handoff_distance_m=110.):
        self.profile = FlightProfile(scenario=scenario, handoff_distance_m=handoff_distance_m)
        self.dwell_start = None
        self.safe_hold = False
        self.hold_offset = None
        self.reason = 'following bounded distant-flight reference'

    def hold(self, reason):
        self.safe_hold = True
        self.reason = reason
        self.dwell_start = None

    def step(self, now_ns, state, *, healthy):
        t = now_ns*1e-9
        target_r, target_v, target_a = self.profile.target(t)
        desired_r, desired_v, desired_a = self.profile.chaser(t)
        # The nominal profile supplies geometry. Tracking follows the estimated
        # passive target, without reading a propagated spacecraft state.
        p = state['p'][1]+desired_r-target_r
        v = state['v'][1]+desired_v-target_v
        if not healthy:
            self.hold('distant-flight navigation or cooperative gyro unavailable')
        if self.safe_hold:
            # Latch once in the inertial relative frame. Referencing the current
            # chaser state on every update cannot brake a closing vehicle.
            if self.hold_offset is None:
                self.hold_offset = (state['p'][0]-state['p'][1]).copy()
            p = state['p'][1]+self.hold_offset
            v = state['v'][1].copy()
        stable = (healthy and not self.safe_hold
                  and t >= self.profile.start_approach_s+self.profile.approach_duration_s
                  and np.linalg.norm(p-state['p'][0]) < 2.
                  and np.linalg.norm(v-state['v'][0]) < .02)
        self.dwell_start = now_ns if stable and self.dwell_start is None else self.dwell_start if stable else None
        released = self.dwell_start is not None and now_ns-self.dwell_start >= 120_000_000_000
        return dict(position_goal=p, velocity_goal=v,
                    differential_acceleration=np.zeros(3) if self.safe_hold else desired_a-target_a,
                    phase='SafeHold' if self.safe_hold else 'FarReady' if released else self.profile.phase(t),
                    reason=self.reason, released=released,
                    dwell_s=0. if self.dwell_start is None else (now_ns-self.dwell_start)*1e-9)
