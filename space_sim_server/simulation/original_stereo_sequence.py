"""Continuous original-camera trial, with an explicit GNSS bridge policy.

This module produces references, never body states or artificial observations.
The bridge changes the original 12 m/5.5 m visual release assumptions and is
therefore opt-in, recorded, and separate from the production V-bar policy.
"""
from dataclasses import dataclass

import numpy as np

from simulation.near_field.reference import RestLeg, StopReference


@dataclass(frozen=True)
class OriginalStereoLimits:
    position_m: float = .05
    speed_m_s: float = .01
    sigma_m: float = .04
    dwell_s: float = 5.
    bridge_speed_m_s: float = .02
    visual_speed_m_s: float = .02
    timeout_s: float = 300.
    pointing_timeout_s: float = 180.
    image_timeout_s: float = 60.
    visual_age_s: float = .35
    update_gap_s: float = .15

    def __post_init__(self):
        values = np.array(list(vars(self).values()), float)
        if not np.isfinite(values).all() or np.any(values <= 0):
            raise ValueError('positive finite sequence limits required')


class OriginalStereoSequence:
    """One navigation/physics history from arrival to tool-gap handoff.

    A 5.5 m GNSS-only dwell grants no visual readiness. Real image quality is
    acquired at the separate near checkpoint. A default instance waits at
    17.4 m for vision, never granting the diagnostic bridge implicitly.
    """
    def __init__(self, initial_com, station17_com, station5_com, near_com, platform_com,
                 *, limits=OriginalStereoLimits(), gnss_bridge_diagnostic=False,
                 tool_gap_m=.5, with_grasp=False):
        self.limits = limits
        self.points = [np.asarray(p, float).copy() for p in
                       (initial_com, station17_com, station5_com, near_com, platform_com)]
        if any(p.shape != (3,) or not np.isfinite(p).all() for p in self.points):
            raise ValueError('finite three-dimensional station references required')
        if not all(self.points[i][0] > self.points[i+1][0] for i in range(4)):
            raise ValueError('checkpoints must approach monotonically')
        if not np.isfinite(tool_gap_m) or tool_gap_m <= 0:
            raise ValueError('positive tool gap required')
        self.station = self.points[1]
        self.diagnostic = bool(gnss_bridge_diagnostic)
        self.with_grasp = bool(with_grasp)
        self.tool_gap_m = float(tool_gap_m)
        self.phase = 'NavigationAcquire'
        self.phase_start = 0.
        self.previous = self.since = self.pointing_started = None
        self.failure = self.stop = self.leg = None
        self.start = self.points[0].copy()
        self.end = self.start.copy()
        self.direction = np.zeros(3)
        self.events = [dict(time_s=0., phase=self.phase)]
        self.telemetry = {}
        self.image_release_s = None
        self.platform_entered_s = self.platform_completed_s = None

    @property
    def target_pointing_enabled(self):
        return self.phase not in {'NavigationAcquire', 'Station17Approach', 'Station17Hold', 'SafeHold', 'Grasp'}

    def _transition(self, phase, now):
        self.phase, self.phase_start, self.since = phase, float(now), None
        self.events.append(dict(time_s=float(now), phase=phase))

    def _qualified(self, value, now):
        if not value:
            self.since = None
        elif self.since is None:
            self.since = now
        return self.since is not None and now-self.since >= self.limits.dwell_s-1e-9

    def _move(self, phase, now, start, end, speed):
        self.start, self.end = np.array(start), np.array(end)
        distance = np.linalg.norm(self.end-self.start)
        self.direction = (self.end-self.start)/max(distance, 1e-15)
        self.leg = RestLeg(0., distance, speed)
        self._transition(phase, now)

    def _fail(self, reason, now, state):
        if self.failure is None:
            self.failure, self.stop = reason, StopReference(state)
            self._transition('SafeHold', now)

    def update(self, now, state, *, position_sigma_m, inputs_valid, service_ready,
               corridor_ready, prealigned=False, visual_ready=False, visual_stamp_s=None,
               estimated_tool_center_gripper_m=None, grasp_complete=False, grasp_failure=None):
        state = np.asarray(state, float)
        if state.shape != (6,) or not np.isfinite(state).all() or not np.isfinite(now):
            raise ValueError('finite state and time required')
        if self.previous is not None and (now <= self.previous or now-self.previous > self.limits.update_gap_s+1e-9):
            self._fail('control_update_gap', now, state)
        self.previous = now
        common = bool(inputs_valid and service_ready and corridor_ready
                      and np.isfinite(position_sigma_m) and 0 <= position_sigma_m < self.limits.sigma_m)
        if not common and self.phase not in {'NavigationAcquire', 'SafeHold', 'Grasp'}:
            self._fail('navigation_link_pointing_or_corridor', now, state)
        fresh = bool(visual_ready and visual_stamp_s is not None
                     and np.isfinite(visual_stamp_s) and 0 <= now-visual_stamp_s <= self.limits.visual_age_s)
        speed = np.linalg.norm(state[3:])

        def at(point):
            return np.linalg.norm(state[:3]-point) <= self.limits.position_m and speed <= self.limits.speed_m_s

        ref, vel, acc = self.end.copy(), np.zeros(3), np.zeros(3)
        if self.phase == 'NavigationAcquire':
            if self._qualified(common and at(self.points[0]), now):
                self._move('Station17Approach', now, self.points[0], self.points[1], self.limits.bridge_speed_m_s)
            elif now-self.phase_start > self.limits.timeout_s:
                self._fail('navigation_acquire_timeout', now, state)
        if self.phase in {'Station17Approach', 'GNSSBridge5p5Approach', 'GNSSBridgeNearApproach', 'VisualApproach'}:
            if self.phase != 'Station17Approach' and not prealigned:
                self._fail('alignment_lost_during_approach', now, state)
            elif self.phase == 'VisualApproach' and not fresh:
                self._fail('visual_lost_after_release', now, state)
            else:
                progress = self.leg.sample(now-self.phase_start)
                ref = self.start+self.direction*progress[0]
                vel, acc = self.direction*progress[1], self.direction*progress[2]
                if now-self.phase_start >= self.leg.duration:
                    next_phase = {'Station17Approach': 'Station17Hold',
                                  'GNSSBridge5p5Approach': 'GNSSBridge5p5Hold',
                                  'GNSSBridgeNearApproach': 'OriginalStereoAcquire',
                                  'VisualApproach': 'PlatformEntry'}[self.phase]
                    self._transition(next_phase, now)
        if self.phase == 'Station17Hold':
            ref = self.points[1].copy()
            if self._qualified(common and at(ref), now):
                self.pointing_started = now
                self._transition('TargetPointing', now)
            elif now-self.phase_start > self.limits.timeout_s:
                self._fail('station17_settle_timeout', now, state)
        if self.phase == 'TargetPointing':
            ref = self.points[1].copy()
            if self._qualified(common and at(ref) and prealigned, now):
                if self.diagnostic:
                    self._move('GNSSBridge5p5Approach', now, ref, self.points[2], self.limits.bridge_speed_m_s)
                else:
                    self._transition('OriginalPolicyVisualWait', now)
            elif now-self.phase_start > self.limits.pointing_timeout_s:
                self._fail('target_pointing_timeout', now, state)
        if self.phase == 'OriginalPolicyVisualWait':
            ref = self.points[1].copy()
            if now-self.phase_start > self.limits.image_timeout_s:
                self._fail('original_policy_far_visual_unavailable', now, state)
        if self.phase == 'GNSSBridge5p5Hold':
            ref = self.points[2].copy()
            if not prealigned:
                self._fail('alignment_lost_at_5p5', now, state)
            elif self._qualified(common and at(ref), now):
                self._move('GNSSBridgeNearApproach', now, ref, self.points[3], self.limits.bridge_speed_m_s)
            elif now-self.phase_start > self.limits.timeout_s:
                self._fail('station5p5_settle_timeout', now, state)
        if self.phase == 'OriginalStereoAcquire':
            ref = self.points[3].copy()
            if self._qualified(common and at(ref) and prealigned and fresh, now):
                self.image_release_s = now
                self._move('VisualApproach', now, ref, self.points[4], self.limits.visual_speed_m_s)
            elif now-self.phase_start > self.limits.image_timeout_s:
                self._fail('original_stereo_not_ready', now, state)
        if self.phase in {'PlatformEntry', 'PlatformHold'}:
            ref = self.points[4].copy()
            tool = np.asarray(estimated_tool_center_gripper_m, float)
            tool_ok = (tool.shape == (3,) and np.isfinite(tool).all()
                       and np.linalg.norm(tool-[0., 0., -self.tool_gap_m]) <= self.limits.position_m)
            if not fresh or not prealigned:
                self._fail('alignment_or_visual_lost_at_platform', now, state)
            elif self._qualified(common and at(ref) and tool_ok, now):
                if self.phase == 'PlatformEntry':
                    self.platform_entered_s = now
                    self._transition('PlatformHold', now)
                else:
                    self.platform_completed_s = now
                    self._transition('Grasp' if self.with_grasp else 'TerminalHold', now)
            elif now-self.phase_start > self.limits.timeout_s:
                self._fail('platform_settle_timeout', now, state)
        if self.phase == 'TerminalHold':
            ref = self.points[4].copy()
            if not fresh or not prealigned:
                self._fail('alignment_or_visual_lost_after_release', now, state)
        if self.phase == 'Grasp':
            ref = self.points[4].copy()
            if grasp_failure:
                self._fail('grasp:'+str(grasp_failure), now, state)
            elif grasp_complete:
                self._transition('Complete', now)
        if self.phase == 'Complete':
            ref = self.points[4].copy()
        if self.phase == 'SafeHold':
            ref, vel, acc = self.stop.sample(now-self.phase_start)
        self.telemetry = dict(phase=self.phase, failure_reason=self.failure,
            target_pointing_enabled=self.target_pointing_enabled, prealigned=bool(prealigned),
            visual_ready=fresh, gnss_bridge_diagnostic=self.diagnostic,
            production_visual_policy_unchanged=True, formal_visual_release=False,
            trial_original_image_release=self.image_release_s is not None,
            station_com_m=self.station.tolist(), reference_com_m=ref.tolist(),
            reference_velocity_m_s=vel.tolist(), estimated_speed_m_s=float(speed))
        return ref, vel, acc
