"""Distant references and release decisions using the mission's feedback source."""
import numpy as np

from simulation.orbital_rendezvous.reference import FlightProfile
from simulation.capture26_mission import Capture26Mission


class Capture26Flight:
    def __init__(self, scenario, *, handoff_distance_m=110., config=None):
        self.config = config or Capture26Mission()
        self.profile = FlightProfile(scenario=scenario, handoff_distance_m=handoff_distance_m,
                                     hold_duration_s=5. if self.config.is_demo else 120.)
        self.dwell_start = None
        self.safe_hold = False
        self.hold_offset = None
        self.reason = 'following bounded distant-flight reference'
        self.last_ns = None

    def hold(self, reason):
        self.safe_hold = True
        self.reason = reason
        self.dwell_start = None

    def step(self, now_ns, state, *, healthy):
        if self.config.is_demo and self.last_ns is not None and now_ns-self.last_ns > 150_000_000:
            self.dwell_start = None
        self.last_ns = now_ns
        t = now_ns*1e-9
        target_r, target_v, target_a = self.profile.target(t)
        desired_r, desired_v, desired_a = self.profile.chaser(t)
        # The nominal profile supplies geometry; tracking follows the passive
        # target using the feedback source selected by the mission profile.
        p = state['p'][1]+desired_r-target_r
        v = state['v'][1]+desired_v-target_v
        if not healthy:
            self.hold('unplanned spacecraft contact' if self.config.is_demo else 'distant-flight navigation or cooperative gyro unavailable')
        if self.safe_hold:
            # Latch once in the inertial relative frame. Referencing the current
            # chaser state on every update cannot brake a closing vehicle.
            if self.hold_offset is None:
                self.hold_offset = (state['p'][0]-state['p'][1]).copy()
            p = state['p'][1]+self.hold_offset
            v = state['v'][1].copy()
        service_rate = float(np.linalg.norm(state['rates'][0]-state['b'][0]))
        rate_stable = service_rate <= self.config.service_angular_rate_limit_rad_s
        position_error = float(np.linalg.norm(p-state['p'][0]))
        velocity_error = float(np.linalg.norm(v-state['v'][0]))
        stable = (healthy and (self.config.is_demo or rate_stable) and not self.safe_hold
                  and t >= self.profile.start_approach_s+self.profile.approach_duration_s
                  and position_error <= 2.
                  and velocity_error <= (.01 if self.config.is_demo else .02))
        self.dwell_start = now_ns if stable and self.dwell_start is None else self.dwell_start if stable else None
        required_s = 5. if self.config.is_demo else 120.
        released = self.dwell_start is not None and now_ns-self.dwell_start >= round(required_s*1e9)
        return dict(position_goal=p, velocity_goal=v,
                    differential_acceleration=np.zeros(3) if self.safe_hold else desired_a-target_a,
                    phase='SafeHold' if self.safe_hold else 'FarReady' if released else self.profile.phase(t),
                    reason=self.reason, released=released,
                    service_angular_rate_rad_s=service_rate, service_angular_rate_stable=rate_stable,
                    acceptance_profile=self.config.acceptance_profile, feedback_source=self.config.feedback_source,
                    node='FarHandoff110m', position_error_m=position_error, velocity_error_m_s=velocity_error,
                    completed_nodes=['FarHandoff110m'] if released else [],
                    position_qualified=position_error <= 2., velocity_qualified=velocity_error <= .01,
                    at_node=t >= self.profile.start_approach_s+self.profile.approach_duration_s,
                    dwell_start_ns=self.dwell_start, dwell_remaining_s=max(0., required_s-(
                        0. if self.dwell_start is None else (now_ns-self.dwell_start)*1e-9)),
                    dwell_s=0. if self.dwell_start is None else (now_ns-self.dwell_start)*1e-9)
