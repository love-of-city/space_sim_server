"""Supervise pointing/images/grasp while VbarApproach owns all platform references.

Image observations update navigation; V-bar continues to consume the posterior
through platform entry and hold. No second translation reference generator is
installed, and no physics or sensor history is reset at visual acquisition.
"""
from copy import deepcopy

import numpy as np

from simulation.near_field.parameters import PARAMETERS as P
from simulation.near_field.vbar_approach import OriginalCameraVbarPolicy, VbarApproach
from simulation.original_stereo_sequence import OriginalStereoLimits


class VbarStereoSequence:
    def __init__(self, initial_com, station17_com, station5_com, near_com, platform_com,
                 *, gnss_bridge_diagnostic=False, tool_gap_m=.5, with_grasp=False,
                 epoch_s=0.):
        self.points = [np.asarray(p, float).copy() for p in
                       (initial_com, station17_com, station5_com, near_com, platform_com)]
        if (not np.isfinite(epoch_s) or epoch_s < 0 or not np.isfinite(tool_gap_m) or tool_gap_m <= 0
                or any(p.shape != (3,) or not np.isfinite(p).all() for p in self.points)
                or not all(self.points[i][0] > self.points[i+1][0] for i in range(4))
                or initial_com[0] <= P.vbar50_m or station17_com[0] >= P.vbar50_m):
            raise ValueError('ordered finite V-bar checkpoints and mission epoch required')
        self.limits = OriginalStereoLimits()
        self.tool_gap_m, self.with_grasp = float(tool_gap_m), bool(with_grasp)
        self.epoch = float(epoch_s)
        near_depth = P.visual_depth_m+self.points[3][0]-self.points[2][0]
        self.vbar = VbarApproach(original_camera_policy=OriginalCameraVbarPolicy(
            acquisition_com_m=tuple(self.points[3]), acquisition_depth_m=float(near_depth),
            gnss_bridge_diagnostic=gnss_bridge_diagnostic))
        self.owner = 'vbar'
        # Kept as a compatibility field for the initial trial's report schema.
        # This is a node-completion epoch, NOT a reference-owner handoff.
        self.handoff_s = None
        self.handoff_evidence = None
        self.previous = None
        self.telemetry = {}
        self.events = [dict(time_s=self.epoch, phase='VbarAcquire', reference_owner='vbar')]
        self._phase, self.phase_start = 'VbarAcquire', self.epoch

    @property
    def phase(self):
        return self._phase

    @property
    def failure(self):
        return self.vbar.failure_reason

    @property
    def target_pointing_enabled(self):
        return (self.vbar.gates['Optical17p4m'].completed_at_s is not None
                and self.phase not in {'SafeHold', 'Grasp', 'Complete'})

    @property
    def station(self):
        return self.points[1]

    def _completion(self, gate):
        value = gate.completed_at_s
        return None if value is None else self.epoch+value

    @property
    def image_release_s(self):
        return self._completion(self.vbar.gates['OriginalStereoAcquire'])

    @property
    def platform_entered_s(self):
        return self._completion(self.vbar.gates['PlatformStandoff'])

    @property
    def platform_completed_s(self):
        return self._completion(self.vbar.platform_hold_gate)

    def _transition(self, phase, now):
        if phase != self._phase:
            self._phase, self.phase_start = phase, float(now)
            self.events.append(dict(time_s=float(now), phase=phase, reference_owner='vbar'))

    def update(self, now, state, *, covariance, optical_depth_m, available_acceleration_m_s2,
               observation_age_s, position_sigma_m, inputs_valid, service_ready,
               corridor_ready, geometry_ready=True, prealigned=False, visual_ready=False,
               visual_stamp_s=None, estimated_tool_center_gripper_m=None,
               grasp_complete=False, grasp_failure=None):
        state, covariance = np.asarray(state, float), np.asarray(covariance, float)
        if (state.shape != (6,) or covariance.shape != (6, 6)
                or not np.isfinite(state).all() or not np.isfinite(covariance).all()
                or not np.isfinite(now) or now < self.epoch):
            raise ValueError('finite same-epoch relative state and covariance required')
        if (self.previous is not None
                and not 0 < now-self.previous <= self.limits.update_gap_s+1e-9):
            self.vbar.fail('control_update_gap', state)
        self.previous = now
        capture_active = self.with_grasp and self.platform_completed_s is not None
        if grasp_failure:
            self.vbar.fail('grasp:'+str(grasp_failure), state)
        common = bool(inputs_valid and service_ready and corridor_ready and geometry_ready
                      and np.isfinite(position_sigma_m) and 0 <= position_sigma_m < self.limits.sigma_m
                      and np.isfinite(observation_age_s) and 0 <= observation_age_s <= .35+1e-9)
        fresh = bool(visual_ready and visual_stamp_s is not None and np.isfinite(visual_stamp_s)
                     and -1e-9 <= now-visual_stamp_s <= self.limits.visual_age_s+1e-9)
        tool = np.asarray(estimated_tool_center_gripper_m, float)
        platform_ready = bool(tool.shape == (3,) and np.isfinite(tool).all()
                              and np.linalg.norm(tool-[0., 0., -self.tool_gap_m]) <= self.limits.position_m)

        def optical_s(depth):
            if depth == P.optical17_m:
                return float(self.station[0])
            return float(self.points[2][0]+depth-P.visual_depth_m)

        ref = self.vbar.update(now-self.epoch, state, covariance, common=common,
            sigma=position_sigma_m, visual_valid=fresh, visual_ready=fresh,
            optical_depth=optical_depth_m, optical_s=optical_s,
            hold=self.points[-1], offset_end_s=self.points[-1][0], geometry_ready=geometry_ready,
            available_acceleration=available_acceleration_m_s2, observation_age=observation_age_s,
            platform_ready=platform_ready, prealigned=prealigned, capture_active=capture_active)
        if self.handoff_s is None and self.vbar.gates['Optical17p4m'].completed_at_s is not None:
            evidence = dict(acquire=deepcopy(self.vbar.telemetry['acquire']),
                            **deepcopy(self.vbar.telemetry['nodes']))
            for gate in evidence.values():
                if gate['completed_at_s'] is not None:
                    gate['completed_at_s'] += self.epoch
            self.handoff_s, self.handoff_evidence = now, evidence
        segment = self.vbar.telemetry['subsegment']
        if self.failure:
            phase = 'SafeHold'
        elif self.platform_completed_s is not None:
            phase = 'Complete' if self.with_grasp and grasp_complete else 'Grasp' if self.with_grasp else 'TerminalHold'
        elif self.platform_entered_s is not None:
            phase = 'PlatformHold'
        elif self.image_release_s is not None:
            phase = 'PlatformEntry' if segment.endswith('Hold') else 'VisualApproach'
        elif self.vbar.gates['GNSS5p5m'].completed_at_s is not None:
            phase = ('OriginalStereoAcquire' if self.vbar.gates['OriginalStereoAcquire'].waiting_since_s is not None
                     else 'GNSSBridgeNearApproach')
        elif self.vbar.pointing_gate.completed_at_s is not None:
            phase = 'GNSSBridge5p5Hold' if segment.endswith('Hold') else 'GNSSBridge5p5Approach'
        elif self.handoff_s is not None:
            phase = 'TargetPointing'
        else:
            phase = segment
        self._transition(phase, now)
        self.telemetry = dict(self.vbar.telemetry, phase=self.phase, failure_reason=self.failure,
            reference_owner='vbar', reference_com_m=np.asarray(ref[0]).tolist(),
            reference_velocity_m_s=np.asarray(ref[1]).tolist(),
            vbar_station17_completed_s=self.handoff_s,
            vbar_nodes=deepcopy(self.vbar.telemetry.get('nodes', {})),
            formal_visual_release=False, production_visual_policy_unchanged=True,
            gnss_bridge_diagnostic=self.vbar.original_camera_policy.gnss_bridge_diagnostic,
            target_pointing_enabled=bool(self.target_pointing_enabled), prealigned=bool(prealigned),
            visual_ready=fresh, trial_original_image_release=self.image_release_s is not None,
            station_com_m=self.station.tolist(), estimated_speed_m_s=float(np.linalg.norm(state[3:])))
        return ref
