"""Image measurement identity and revocable visual gate; no simulator imports."""
from __future__ import annotations

import hashlib
import json
import math

import numpy as np

POSE_ERROR_ORDER = ['position_x_m', 'position_y_m', 'position_z_m',
                    'rotation_x_rad', 'rotation_y_rad', 'rotation_z_rad']


def calibration_id(config):
    """Include extrinsics and layout, not only camera intrinsics."""
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def covariance(value, dimension):
    result = np.asarray(value, dtype=float)
    if (result.shape != (dimension, dimension) or not np.isfinite(result).all()
            or not np.allclose(result, result.T, atol=1e-10)
            or np.linalg.eigvalsh(result)[0] <= 0):
        raise ValueError('measurement covariance must be finite, symmetric and positive definite')
    return result


def principal_sigma(matrix):
    return math.sqrt(max(0., float(np.linalg.eigvalsh(matrix)[-1])))


def image_clock(result, now_ns, received_wall_time_ns=None):
    """Physical exposure age; optional wall clocks are diagnostics only."""
    exposure = int(result['sim_time_ns'])
    age = int(now_ns)-exposure
    if age < 0:
        raise ValueError('visual exposure is outside the physical clock')
    source = received = None
    timing = result.get('capture_timing')
    if (received_wall_time_ns is not None and isinstance(timing, dict)
            and all(isinstance(timing.get(eye), dict) and 'source_wall_time_ns' in timing[eye] for eye in ('left', 'right'))):
        try:
            received = int(received_wall_time_ns)
            clocks = [int(result['capture_timing'][eye]['source_wall_time_ns']) for eye in ('left', 'right')]
            source = min(clocks)
        except (TypeError, ValueError, OverflowError):
            source = received = None  # Optional diagnostics cannot reject a physical observation.
    return exposure, age, source, received


class VisualReadiness:
    """Revocable streak in physical time, independent of platform stability."""
    def __init__(self, mission_id, session_id, calibration, *, minimum=10):
        self.identity = (mission_id, session_id, calibration)
        self.minimum = minimum
        self.count = 0
        self.reason = 'no accepted image observation'
        self.last_exposure_ns = self.last_receive_ns = None
        self._last_seen_exposure_ns = self._last_seen_receive_ns = None
        self._last_seen_wall_source_ns = self._last_seen_wall_receive_ns = None
        self.last_wall_source_ns = self.last_wall_receive_ns = None
        self._seen = set()
        self.ready = False
        self._accepted_events = []
        self.revocations = []
        self.last_failures = []

    def revoke(self, reason, *, now_ns=None, failures=None, category='image_invalid'):
        failures = list(failures or [str(reason)])
        # Keep each actual reset, plus changes in failure conditions while empty.
        if self.count or failures != self.last_failures:
            self.revocations.append(dict(time_ns=now_ns, count_before=self.count,
                count_after=0, failures=failures, category=category))
        self.last_failures = failures
        self.count, self.ready, self.reason = 0, False, str(reason)
        self._accepted_events.clear()
        return False

    def reconcile(self, navigation_acceptance, now_ns=None):
        """A delayed GNSS correction may revoke a previously accepted pose."""
        if any(navigation_acceptance.get(event) is False for event in self._accepted_events):
            self.revoke('delayed navigation replay rejected a counted observation',
                        now_ns=now_ns, category='navigation_replay')
        return self.ready

    def tick(self, now_ns, received_wall_time_ns=None):
        failures = []
        if self.count and self.last_exposure_ns is not None:
            if now_ns - self.last_exposure_ns > 350_000_000:
                failures.append('accepted exposure is stale')
            if now_ns - self.last_receive_ns > 150_000_000:
                failures.append('effective visual updates interrupted')
        if failures:
            self.revoke(failures[0], now_ns=now_ns, failures=failures, category='simulation_time_interruption')
        return self.ready

    def check_uncertainty(self, position, attitude, now_ns):
        if self.count and (principal_sigma(covariance(position, 3)) >= .04 or principal_sigma(covariance(attitude, 3)) >= math.radians(.3)):
            self.revoke('relative navigation uncertainty exceeded', now_ns=now_ns, category='precision_exceeded')
        return self.ready

    def observe(self, result, now_ns, *, accepted, relative_position_covariance,
                relative_attitude_covariance, received_wall_time_ns=None, navigation_reason=None):
        failures = []
        interval_failures = []
        try:
            identity = tuple(result.get(key) for key in ('mission_id', 'session_id', 'calibration_id'))
            exposure, age, wall_source, wall_receive = image_clock(result, now_ns, received_wall_time_ns)
            frame = str(result['source_frame_id'])
            if identity != self.identity:
                failures.append('vision identity mismatch')
            if not frame or frame in self._seen or (self._last_seen_exposure_ns is not None and exposure <= self._last_seen_exposure_ns):
                failures.append('duplicate or out-of-order visual observation')
            if age > 350_000_000:
                failures.append('visual exposure age exceeded')
            if not accepted:
                failures.append('navigation rejected: ' + str(navigation_reason or result.get('reason', 'innovation')))
            if result.get('status') != 'ok' or result.get('ambiguous', True):
                failures.append('invalid or ambiguous stereo solution')
            if self._last_seen_exposure_ns is not None:
                if exposure-self._last_seen_exposure_ns > 150_000_000:
                    interval_failures.append('visual sampling interval exceeded')
                if now_ns-self._last_seen_receive_ns > 150_000_000:
                    interval_failures.append('effective visual receive interval exceeded')
            # Invalid solver events may intentionally omit a pose and quality.
            if result.get('status') == 'ok':
                counts = result['counts']
                if min(counts['used_markers_left'], counts['used_markers_right'], counts['common_markers']) < 2:
                    failures.append('insufficient complete known inlier markers')
                if any(not 0 <= float(result['reprojection_error_px'][eye]) <= 2.5
                       or not 0 <= float(result['max_corner_error_px'][eye]) <= 5 for eye in ('left', 'right')):
                    failures.append('image reprojection quality decreased')
                c = result['pose_covariance']
                if (c['frame'] != 'left_optical' or c['rotation_error'] != 'left_multiplicative'
                        or c.get('order') != POSE_ERROR_ORDER):
                    failures.append('unsupported pose covariance convention')
                measured = covariance(c['matrix'], 6)
                if math.sqrt(measured[2, 2]) >= .03 or principal_sigma(measured[3:, 3:]) >= math.radians(2):
                    failures.append('visual measurement uncertainty exceeded')
            p = covariance(relative_position_covariance, 3)
            a = covariance(relative_attitude_covariance, 3)
            if principal_sigma(p) >= .04 or principal_sigma(a) >= math.radians(.3):
                failures.append('relative navigation uncertainty exceeded')
            # A wrong-session or future frame cannot poison the active clock.
            if identity == self.identity and 0 <= exposure <= now_ns:
                if self._last_seen_exposure_ns is None or exposure > self._last_seen_exposure_ns:
                    self._last_seen_exposure_ns, self._last_seen_receive_ns = exposure, int(now_ns)
                    self._seen.add(frame)
                    if len(self._seen) > 1024:
                        self._seen = {frame}
            if failures:
                all_failures = failures+interval_failures
                category = 'precision_exceeded' if any('uncertainty' in f for f in failures) else 'image_invalid'
                if result.get('process_healthy') is False:
                    category = 'process_fault'
                return self.revoke(failures[0], now_ns=now_ns, failures=all_failures, category=category)
            if interval_failures:
                # This qualified observation begins a new streak after the gap.
                self.revoke(interval_failures[0], now_ns=now_ns, failures=interval_failures,
                            category='simulation_time_interruption')
            self.last_exposure_ns, self.last_receive_ns = exposure, int(now_ns)
            self.last_wall_source_ns, self.last_wall_receive_ns = wall_source, wall_receive
            event = 'visual/'+'/'.join(str(result[k]) for k in ('mission_id', 'session_id', 'source_frame_id', 'calibration_id'))
            self._accepted_events.append(event)
            self.count += 1
            self.last_failures = []
            self.ready = self.count >= self.minimum
            self.reason = 'ready' if self.ready else 'collecting distinct accepted exposures'
            return self.ready
        except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as exc:
            failures.append('malformed visual observation: '+str(exc))
            return self.revoke(failures[0], now_ns=now_ns, failures=failures)

    def status(self, now_ns, received_wall_time_ns=None):
        # Observing telemetry must never change admission or reset diagnostics.
        age = None if self.last_exposure_ns is None else (now_ns-self.last_exposure_ns)*1e-9
        wall_age = (None if received_wall_time_ns is None or self.last_wall_source_ns is None
                    else (received_wall_time_ns-self.last_wall_source_ns)*1e-9)
        return dict(ready=self.ready, count=self.count, reason=self.reason,
                    exposure_age_s=age, simulation_exposure_age_s=age,
                    wall_pipeline_age_s=wall_age, failures=list(self.last_failures),
                    revocation_count=len(self.revocations))
