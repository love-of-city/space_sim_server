"""Navigation arrival -> stationary target prealignment -> visual handoff.

This coordinator supplies references and stage permissions, never physical
states. The caller owns the dynamics, prealignment feedback, visual estimator
and mission-specific collision/braking checks. Optical checkpoint conversion
is resolved into a fixed COM reference before entering this sequence.
"""
from dataclasses import dataclass

import numpy as np

from simulation.near_field.reference import RestLeg, StopReference


@dataclass(frozen=True)
class PointingHandoffConfig:
    arrival_tolerance_m: float = .05
    arrival_speed_m_s: float = .002
    position_sigma_m: float = .05
    dwell_s: float = 5.
    settle_timeout_s: float = 300.
    pointing_timeout_s: float = 180.
    visual_wait_timeout_s: float = 60.
    max_gap_s: float = .15
    visual_max_age_s: float = .35
    speed_m_s: float = .02
    visual_dwell_s: float = 0.

    def __post_init__(self):
        values=np.asarray([v for k,v in vars(self).items() if k!='visual_dwell_s'],float)
        if not np.isfinite(values).all() or np.any(values<=0):
            raise ValueError('positive finite handoff limits required')
        if not np.isfinite(self.visual_dwell_s) or self.visual_dwell_s<0:
            raise ValueError('nonnegative finite visual dwell required')


class PointingHandoff:
    """Fail closed when telemetry/alignment disappears or vision never arrives.

    `prealigned` must come from the continuous geometric/attitude handshake,
    not an elapsed-time flag. Vision is a distinct external measurement gate.
    The diagnostic continuation flag bypasses only the visual wait; it is
    explicitly recorded and must not be used as a formal visual release.
    """
    def __init__(self, initial_com, station_com, config=PointingHandoffConfig(), *,
                 end_com=None, gnss_diagnostic_continuation=False):
        self.config=config
        self.initial=np.asarray(initial_com,float).copy()
        self.station=np.asarray(station_com,float).copy()
        self.end=None if end_com is None else np.asarray(end_com,float).copy()
        for value in (self.initial,self.station,*(() if self.end is None else (self.end,))):
            if value.shape!=(3,) or not np.isfinite(value).all():
                raise ValueError('finite three-dimensional COM references required')
        if not np.allclose(self.initial[1:],self.station[1:]) or (self.end is not None and not np.allclose(self.end[1:],self.station[1:])):
            raise ValueError('this sequence supports one along-track leg')
        self.diagnostic=bool(gnss_diagnostic_continuation)
        self.phase='NavigationAcquire'
        self.phase_start=0.
        self.previous=None
        self.since=None
        self.start_time=None
        self.leg=None
        self.failure=None
        self.stop=None
        self.events=[]
        self.pointing_started=None
        self.telemetry={}

    @property
    def target_pointing_enabled(self):
        return self.phase in {'TargetPointing','VisualWait','GNSSDiagnosticContinue','VisualContinue','TerminalHold'}

    def _transition(self, phase, now):
        self.phase=phase;self.phase_start=now;self.since=None
        self.events.append(dict(time_s=float(now),phase=phase))

    def _qualified_for(self, qualified, now, duration=None):
        if not qualified:
            self.since=None
        elif self.since is None:
            self.since=now
        required=self.config.dwell_s if duration is None else duration
        return self.since is not None and now-self.since>=required-1e-9

    def _fail(self, reason, now, state):
        if self.failure is None:
            self.failure=reason;self.stop=StopReference(state)
            self._transition('SafeHold',now)

    def update(self, now, com_state, *, position_sigma_m, inputs_valid,
               service_ready, corridor_ready, prealigned=False,
               visual_ready=False, visual_stamp_s=None):
        state=np.asarray(com_state,float)
        if state.shape!=(6,) or not np.isfinite(state).all() or not np.isfinite(now):
            raise ValueError('finite same-epoch navigation state required')
        if self.start_time is None:
            self.start_time=now;self.phase_start=now
            self.events.append(dict(time_s=float(now),phase=self.phase))
        if self.previous is not None and (now<=self.previous or now-self.previous>self.config.max_gap_s+1e-9):
            self._fail('control_time_gap',now,state)
        self.previous=now
        nav_ok=bool(np.isfinite(position_sigma_m) and 0<=position_sigma_m<self.config.position_sigma_m)
        common=bool(inputs_valid and nav_ok and service_ready and corridor_ready)
        # Initial acquisition holds the declared starting station while the
        # estimator/pointing settle. It grants no approach or target-slew permit.
        # Once approach starts, any loss of a common condition is latched.
        if not common and self.phase!='NavigationAcquire':
            self._fail('navigation_link_pointing_or_corridor',now,state)
        speed=float(np.linalg.norm(state[3:]))
        at_station=bool(np.linalg.norm(state[:3]-self.station)<self.config.arrival_tolerance_m
                        and speed<self.config.arrival_speed_m_s)
        ref=self.initial.copy();velocity=np.zeros(3);acceleration=np.zeros(3)
        pointing_before=self.target_pointing_enabled
        visual_fresh=bool(visual_ready and visual_stamp_s is not None
                          and np.isfinite(visual_stamp_s) and 0<=now-visual_stamp_s<=self.config.visual_max_age_s)
        if self.phase=='NavigationAcquire':
            stable=np.linalg.norm(state[:3]-self.initial)<self.config.arrival_tolerance_m and speed<self.config.arrival_speed_m_s
            if self._qualified_for(common and stable,now):
                self.leg=RestLeg(self.initial[0],self.station[0],self.config.speed_m_s)
                self._transition('StationApproach',now)
            elif now-self.phase_start>self.config.settle_timeout_s:
                self._fail('navigation_acquire_timeout',now,state)
        if self.phase=='StationApproach':
            progress=self.leg.sample(now-self.phase_start)
            ref=self.station.copy();ref[0]=progress[0];velocity[0]=progress[1];acceleration[0]=progress[2]
            if now-self.phase_start>=self.leg.duration:
                self._transition('StationSettle',now)
        if self.phase=='StationSettle':
            ref=self.station.copy()
            if self._qualified_for(common and at_station,now):
                if self.pointing_started is None:
                    self.pointing_started=now
                self._transition('TargetPointing',now)
            elif now-self.phase_start>self.config.settle_timeout_s:
                self._fail('station_settle_timeout',now,state)
        if self.phase=='TargetPointing':
            ref=self.station.copy()
            # The feedback controller is enabled on the next tick. Never use a
            # prealignment result obtained while the stage was disabled.
            if self._qualified_for(pointing_before and prealigned and common and at_station,now):
                self._transition('VisualWait',now)
            elif now-self.pointing_started>self.config.pointing_timeout_s:
                self._fail('target_pointing_timeout',now,state)
        if self.phase=='VisualWait':
            ref=self.station.copy()
            if not prealigned or not at_station:
                self._transition('TargetPointing',now)
            elif self._qualified_for(visual_fresh,now,self.config.visual_dwell_s) or self.diagnostic:
                next_phase='GNSSDiagnosticContinue' if self.diagnostic else 'VisualContinue'
                if self.end is None:
                    self._transition('TerminalHold',now)
                else:
                    self.leg=RestLeg(self.station[0],self.end[0],self.config.speed_m_s)
                    self._transition(next_phase,now)
            elif now-self.phase_start>self.config.visual_wait_timeout_s:
                self._fail('visual_not_ready_after_pointing',now,state)
        if self.phase in {'VisualContinue','GNSSDiagnosticContinue'}:
            if not prealigned or (self.phase=='VisualContinue' and not visual_fresh):
                self._fail('alignment_or_visual_lost_after_release',now,state)
            else:
                progress=self.leg.sample(now-self.phase_start)
                ref=self.end.copy();ref[0]=progress[0];velocity[0]=progress[1];acceleration[0]=progress[2]
                if now-self.phase_start>=self.leg.duration:
                    self._transition('TerminalHold',now)
        if self.phase=='TerminalHold':
            ref=self.station.copy() if self.end is None else self.end.copy()
            if not prealigned or (not self.diagnostic and not visual_fresh):
                self._fail('alignment_or_visual_lost_after_release',now,state)
        if self.phase=='SafeHold':
            ref,velocity,acceleration=self.stop.sample(now-self.phase_start)
        self.telemetry=dict(phase=self.phase,failure_reason=self.failure,
            target_pointing_enabled=self.target_pointing_enabled,
            at_station=at_station,prealigned=bool(prealigned),visual_ready=visual_fresh,
            gnss_diagnostic_continuation=self.diagnostic,
            formal_visual_release=bool(self.phase in {'VisualContinue','TerminalHold'} and not self.diagnostic),
            reference_com_m=ref.tolist(),reference_velocity_m_s=velocity.tolist(),
            station_com_m=self.station.tolist(),estimated_speed_m_s=speed)
        return ref,velocity,acceleration
