"""Near-only V-bar acquisition, mandatory nodes, path and stopping protection."""
from dataclasses import dataclass

import numpy as np

from .approach_plan import DwellGate
from .parameters import PARAMETERS as P
from .reference import RestLeg, StopReference, offset_path


@dataclass(frozen=True)
class OriginalCameraVbarPolicy:
    """Explicit original-camera route; does not alter the legacy web policy."""
    acquisition_com_m: tuple
    acquisition_depth_m: float = 1.5
    gnss_bridge_diagnostic: bool = False
    position_tolerance_m: float = .05
    speed_limit_m_s: float = .01
    navigation_sigma_m: float = .04
    pointing_timeout_s: float = 180.

    def __post_init__(self):
        point = np.asarray(self.acquisition_com_m, float)
        values = np.asarray([self.acquisition_depth_m, self.position_tolerance_m,
                             self.speed_limit_m_s, self.navigation_sigma_m, self.pointing_timeout_s])
        if (point.shape != (3,) or not np.isfinite(point).all() or not np.isfinite(values).all()
                or np.any(values <= 0) or not .5 < self.acquisition_depth_m < P.visual_depth_m
                or not isinstance(self.gnss_bridge_diagnostic, bool)):
            raise ValueError('invalid original-camera V-bar policy')


class VbarApproach:
    def __init__(self, *, node_position_tolerances_m=None, original_camera_policy=None):
        if original_camera_policy is not None and not isinstance(original_camera_policy, OriginalCameraVbarPolicy):
            raise ValueError('explicit original-camera V-bar policy required')
        self.original_camera_policy = original_camera_policy
        self.visual_index, self.offset_index = (3, 4) if original_camera_policy else (2, 3)
        defaults = [P.vbar50_tolerance_m, P.optical17_tolerance_m,
                    P.visual_tolerance_m, P.platform_tolerance_m]
        if original_camera_policy:
            defaults = [original_camera_policy.position_tolerance_m]*5
        self.position_tolerances = np.asarray(defaults if node_position_tolerances_m is None
                                              else node_position_tolerances_m, float).copy()
        if (self.position_tolerances.shape != (len(defaults),) or not np.isfinite(self.position_tolerances).all()
                or np.any(self.position_tolerances <= 0)):
            raise ValueError('positive finite tolerance required for each V-bar node')
        self.acquire = DwellGate(timeout_s=P.acquire_timeout_s)
        names = (('Vbar50m', 'Optical17p4m', 'GNSS5p5m', 'OriginalStereoAcquire', 'PlatformStandoff')
                 if original_camera_policy else
                 ('Vbar50m', 'Optical17p4m', 'VisualReady5p5m', 'PlatformStandoff'))
        self.gates = {name: DwellGate(timeout_s=P.visual_timeout_s if name in
                      ('VisualReady5p5m', 'OriginalStereoAcquire') else P.node_timeout_s) for name in names}
        self.node_ids = tuple(self.gates)
        self.index = 0
        self.initial = None
        self.leg = None
        self.leg_start = 0.
        self.targets = None
        self.failure_reason = None
        self.safe_reference = None
        self.safe_stop = None
        self.safe_stop_start = None
        self.safe_verified = False
        self.boundary_wait = None
        self.boundary_reference = None
        self.boundary_kind = None
        self.telemetry = {}
        self.offset_progress = 0.
        self.pointing_gate = DwellGate(timeout_s=180.)
        self.pointing_started_s = None
        self.platform_hold_gate = DwellGate(timeout_s=P.node_timeout_s)

    def fail(self, reason, state):
        if self.failure_reason is None:
            self.failure_reason = reason
            # Ballistic stopping setpoint lies in the direction of current
            # motion. It does not command a return to the fault-time position.
            self.safe_stop = StopReference(state)
            self.safe_reference = self.safe_stop.endpoint.copy()

    def update(self, now, state, covariance, *, common, sigma, visual_valid, visual_ready,
               optical_depth, optical_s, hold, offset_end_s, geometry_ready, available_acceleration,
               observation_age, platform_ready=False, prealigned=False, capture_active=False):
        if self.initial is None:
            self.initial = state.copy()
        speed = float(np.linalg.norm(state[3:]))
        node = self.node_ids[self.index]
        gate = self.gates[node]
        position = state[:3].copy()
        velocity, acceleration = np.zeros(3), np.zeros(3)
        reference = self.initial[:3].copy()
        subsegment = 'NearAcquire'
        conditions = {}
        policy = self.original_camera_policy
        speed_limit = policy.speed_limit_m_s if policy else P.speed_m_s
        sigma_limit = policy.navigation_sigma_m if policy else P.gnss_sigma_m
        pointing_pause = bool(policy and self.gates['Optical17p4m'].completed_at_s is not None
                              and self.pointing_gate.completed_at_s is None)
        if self.boundary_wait is not None:
            recovered = visual_valid if self.boundary_kind == P.vision_boundary_m else visual_ready
            if recovered and now-self.boundary_wait >= self.boundary_stop.duration:
                self.next_start = self.boundary_reference[0]
                self.leg = None
                self.boundary_wait, self.boundary_reference, self.boundary_kind = None, None, None
        if self.acquire.completed_at_s is None:
            # Freeze along-track at handoff; remove transverse offset without
            # discontinuous physical writes or a return to a fixed 110 m.
            if self.leg is None:
                distance = np.linalg.norm(self.initial[1:3])
                self.leg = RestLeg(0., distance, P.acquire_speed_m_s)
            progress = self.leg.sample(now)
            length = np.linalg.norm(self.initial[1:3])
            direction = self.initial[1:3]/max(length, 1e-12)
            reference[1:] = self.initial[1:3]-direction*progress[0]
            velocity[1:], acceleration[1:] = -direction*progress[1], -direction*progress[2]
            conditions = {'transverse': np.linalg.norm(state[1:3]) < P.acquire_transverse_m,
                          'speed': speed < speed_limit, 'sigma': sigma < sigma_limit,
                          'common': common, 'cooperative_geometry': geometry_ready}
            self.acquire.update(now, holding=now >= self.leg.duration, qualified=all(conditions.values()))
            if now >= P.acquire_timeout_s and self.acquire.completed_at_s is None:
                self.fail('entry_acquisition_timeout', state)
            if self.acquire.completed_at_s is not None:
                self.targets = [np.array([P.vbar50_m, 0., 0.]),
                                np.array([optical_s(P.optical17_m), 0., 0.]),
                                np.array([optical_s(P.visual_depth_m), 0., 0.]), hold.copy()]
                if policy:
                    self.targets.insert(3, np.array(policy.acquisition_com_m, float))
                    if not all(self.targets[i][0] > self.targets[i+1][0] for i in range(4)):
                        raise ValueError('original-camera V-bar nodes must approach monotonically')
                self.offset_end_s = offset_end_s
                self.leg = None
                self.next_start = reference[0]
        elif pointing_pause:
            reference = self.targets[1].copy()
            subsegment = 'TargetPointing'
            if self.pointing_started_s is None:
                self.pointing_started_s = now
            conditions = dict(position=np.linalg.norm(state[:3]-reference) < policy.position_tolerance_m,
                              speed=speed < speed_limit, sigma=sigma < sigma_limit, common=common,
                              cooperative_geometry=geometry_ready, target_aligned=prealigned,
                              bridge_authorized=policy.gnss_bridge_diagnostic or visual_ready)
            self.pointing_gate.update(now, holding=True, qualified=all(conditions.values()))
            if now-self.pointing_started_s >= policy.pointing_timeout_s and self.pointing_gate.completed_at_s is None:
                self.fail('target_pointing_or_bridge_authorization_timeout', state)
        else:
            target = self.targets[self.index]
            if self.leg is None:
                vmax = (P.cruise_speed_m_s if self.index < 2 else P.offset_speed_m_s
                        if self.index == self.offset_index else P.close_speed_m_s)
                self.leg = RestLeg(self.next_start, target[0], vmax,
                                   amax=P.offset_acceleration_m_s2 if self.index == self.offset_index else P.acceleration_m_s2)
                self.leg_start = now
            elapsed = now-self.leg_start
            progress = self.leg.sample(elapsed)
            reference = np.array([progress[0], 0., 0.])
            velocity[0], acceleration[0] = progress[1:]
            if self.index == self.offset_index:
                reference, velocity, acceleration, self.offset_progress = offset_path(
                    progress, self.targets[self.visual_index][0], self.offset_end_s,
                    self.targets[self.offset_index][1:])
            holding = elapsed >= self.leg.duration
            braking = elapsed >= self.leg.braking_start
            subsegment = f'{node}{"Hold" if holding else "Brake" if braking else "Approach"}'
            tolerance = self.position_tolerances[self.index]
            conditions = {'position': np.linalg.norm(state[:3]-target) < tolerance,
                          'speed': speed < speed_limit,
                          'sigma': sigma < (sigma_limit if policy else
                                          P.visual_sigma_m if self.index >= 2 else P.gnss_sigma_m),
                          'common': common, 'cooperative_geometry': geometry_ready,
                          'visual_ready': self.index < self.visual_index or visual_ready or capture_active,
                          'platform_geometry': self.index != self.offset_index or platform_ready}
            if policy:
                conditions['target_aligned'] = self.index < 2 or prealigned or capture_active
            gate.update(now, holding=holding, braking=braking, qualified=all(conditions.values()))
            reason = gate.failure(now)
            if reason:
                self.fail(f'{node}:{reason}', state)
            if gate.completed_at_s is not None and self.index < self.offset_index:
                self.next_start = target[0]
                self.index += 1
                self.leg = None

        if policy and self.gates['PlatformStandoff'].completed_at_s is not None:
            qualified = (common and geometry_ready and platform_ready and visual_ready and prealigned
                         and sigma < sigma_limit and speed < speed_limit
                         and np.linalg.norm(state[:3]-hold) < policy.position_tolerance_m)
            self.platform_hold_gate.update(now, holding=True, qualified=qualified)
            if self.platform_hold_gate.failure(now):
                self.fail('platform_hold_timeout', state)

        # Covariance and braking are always evaluated, including stationkeeping.
        sigma_perp = float(np.sqrt(max(0., np.linalg.eigvalsh(covariance[1:3, 1:3]).max())))
        width = min(P.corridor_max_m, max(P.corridor_min_m, P.corridor_slope*state[0]))
        corridor_margin = width-np.linalg.norm(state[1:3]-reference[1:])-3*sigma_perp
        braking_accel = min(P.braking_m_s2, available_acceleration)
        velocity_sigma = float(np.sqrt(max(0., covariance[3, 3])))
        vplus = max(0., -state[3]+3*velocity_sigma)
        tau = max(0., observation_age)+.05+P.execution_delay_s+.5*max(0., braking_accel)/P.jerk_m_s3
        stop_distance = (vplus*tau+vplus*vplus/(2*braking_accel)+3*np.sqrt(max(0., covariance[0, 0]))
                         + P.boundary_margin_m) if braking_accel > 0 else None
        boundary = P.vision_boundary_m if not visual_valid else P.ready_boundary_m if not visual_ready else None
        if policy and policy.gnss_bridge_diagnostic:
            # The reference already stops at the real-image acquisition node.
            # Keep a separate inner braking boundary while it has no vision.
            boundary = None if visual_ready or capture_active else policy.acquisition_depth_m-.5
        margin = None if boundary is None or stop_distance is None else optical_depth-boundary-stop_distance
        if self.acquire.completed_at_s is not None:
            if corridor_margin <= 0:
                self.fail('transverse_corridor', state)
            if braking_accel <= 0:
                self.fail('braking_authority_unverified', state)
            if self.gates[self.node_ids[self.visual_index]].completed_at_s is not None and not visual_ready and not capture_active:
                self.fail('visual_readiness_lost_after_release', state)
            if policy:
                if not common or not np.isfinite(sigma) or not 0 <= sigma < sigma_limit:
                    self.fail('navigation_health_lost', state)
                if self.pointing_gate.completed_at_s is not None and not prealigned and not capture_active:
                    self.fail('target_alignment_lost', state)
            if self.boundary_wait is not None or (boundary is not None and margin is not None and margin < 0):
                active_boundary = self.boundary_kind if self.boundary_kind is not None else boundary
                if optical_depth <= active_boundary+P.boundary_margin_m:
                    self.fail('optical_braking_margin', state)
                if self.boundary_wait is None:
                    self.boundary_wait = now
                    self.boundary_kind = boundary
                    self.boundary_stop = StopReference(state)
                    self.boundary_reference = self.boundary_stop.endpoint.copy()
                reference, velocity, acceleration = self.boundary_stop.sample(now-self.boundary_wait)
                subsegment = 'OpticalBoundaryHold'
                if now-self.boundary_wait >= P.visual_timeout_s:
                    self.fail('optical_boundary_timeout', state)
        if now >= P.duration_s:
            self.fail('near_mission_timeout', state)
        if self.failure_reason is not None:
            if self.safe_stop_start is None:
                self.safe_stop_start = now
            reference, velocity, acceleration = self.safe_stop.sample(now-self.safe_stop_start)
            subsegment = 'SafeHoldBrake' if speed >= .003 else 'SafeHold'
            self.safe_verified = bool(speed < .003 and sigma < P.gnss_sigma_m and common and braking_accel > 0)
        references = ['assembled COM along-track', 'optical depth on COM V-bar', 'optical depth on COM V-bar', 'stowed TCP axial gap']
        values = [P.vbar50_m, P.optical17_m, P.visual_depth_m, .5]
        if policy:
            references = ['assembled COM along-track', 'optical depth on COM V-bar',
                          'GNSS optical-depth station on COM V-bar',
                          'real-image acquisition on COM V-bar', 'prepared TCP axial gap']
            values = [P.vbar50_m, P.optical17_m, P.visual_depth_m, policy.acquisition_depth_m, .5]
        self.telemetry = {'algorithm': 'COM V-bar rotating-frame PD with gravity/Coriolis feedforward',
                          'frame': 'TNR', 'sign': 'target minus chaser', 'reference_point': 'assembled COM',
                          'subsegment': subsegment, 'state': state.tolist(), 'reference_m': reference.tolist(),
                          'reference_velocity_m_s': velocity.tolist(), 'reference_acceleration_m_s2': acceleration.tolist(),
                          'corridor_width_m': width, 'corridor_margin_m': corridor_margin,
                          'braking_acceleration_m_s2': braking_accel, 'stopping_distance_m': stop_distance,
                          'optical_boundary_m': boundary, 'braking_margin_m': margin,
                          'offset_progress': self.offset_progress, 'failure_reason': self.failure_reason,
                          'safe_hold_verified': self.safe_verified, 'conditions': {k: bool(v) for k, v in conditions.items()},
                          'node_id': node, 'node_error_m': (state[:3]-(self.targets[self.node_ids.index(node)]
                                              if self.targets is not None else reference)).tolist(),
                          'acquire': self.acquire.telemetry(now),
                          'nodes': {name: dict(g.telemetry(now), reference_point=references[i], target=values[i], unit='m',
                                    reference_com_m=self.targets[i].tolist() if self.targets is not None else None)
                                    for i, (name, g) in enumerate(self.gates.items())}}
        if policy:
            self.telemetry.update(reference_owner='vbar',
                original_camera_gnss_bridge=policy.gnss_bridge_diagnostic,
                target_pointing=self.pointing_gate.telemetry(now),
                platform_hold=self.platform_hold_gate.telemetry(now))
        return reference, velocity, acceleration
