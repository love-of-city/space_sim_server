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
    """Count both physical age and real pipeline age on the localhost channel.

    Replay/unit inputs can use physical time alone. Live callers supply the
    local receive clock and must have both source clocks from the UE request.
    This prevents slower-than-real-time physics from hiding an old image.
    """
    exposure = int(result['sim_time_ns'])
    age = int(now_ns)-exposure
    if age < 0:
        raise ValueError('visual exposure is outside the physical clock')
    source = received = None
    if received_wall_time_ns is not None:
        received = int(received_wall_time_ns)
        clocks = [int(result['capture_timing'][eye]['source_wall_time_ns']) for eye in ('left', 'right')]
        source = min(clocks)
        if source < 0 or received < max(clocks):
            raise ValueError('invalid localhost image clock')
        age = max(age, received-source)
    return exposure, age, source, received


class VisualReadiness:
    """Only navigation-accepted image poses count; live checks use both clocks.

    Tick this gate even when there are no results. An invalid result clears the
    streak immediately. Delivery time is the time the control loop consumes the
    result, so readback, transport, queues and solving all count towards age.
    """
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

    def revoke(self, reason):
        self.count, self.ready, self.reason = 0, False, str(reason)
        self._accepted_events.clear()
        return False

    def reconcile(self, navigation_acceptance):
        """A delayed GNSS correction may revoke a previously accepted pose."""
        if any(navigation_acceptance.get(event) is False for event in self._accepted_events):
            self.revoke('delayed navigation replay rejected a counted observation')
        return self.ready

    def tick(self, now_ns, received_wall_time_ns=None):
        if self.last_exposure_ns is not None:
            if now_ns - self.last_exposure_ns > 350_000_000:
                self.revoke('accepted exposure is stale')
            elif now_ns - self.last_receive_ns > 150_000_000:
                self.revoke('effective visual updates interrupted')
        if received_wall_time_ns is not None and self.last_wall_source_ns is not None:
            if received_wall_time_ns < self.last_wall_receive_ns:
                self.revoke('localhost image clock regressed')
            elif received_wall_time_ns-self.last_wall_source_ns > 350_000_000:
                self.revoke('accepted image pipeline age exceeded')
            elif received_wall_time_ns-self.last_wall_receive_ns > 150_000_000:
                self.revoke('effective wall-clock visual updates interrupted')
        return self.ready

    def observe(self, result, now_ns, *, accepted, relative_position_covariance,
                relative_attitude_covariance, received_wall_time_ns=None, navigation_reason=None):
        try:
            identity = tuple(result.get(key) for key in ('mission_id', 'session_id', 'calibration_id'))
            if identity != self.identity:
                return self.revoke('vision identity mismatch')
            exposure = int(result['sim_time_ns'])
            frame = str(result['source_frame_id'])
            if not 0 <= exposure <= now_ns:
                return self.revoke('visual exposure is outside the physical clock')
            if frame in self._seen or (self._last_seen_exposure_ns is not None and exposure <= self._last_seen_exposure_ns):
                return self.revoke('duplicate or out-of-order visual observation')
            self._seen.add(frame)
            # A bounded cache is sufficient because epochs enforce monotonicity.
            if len(self._seen) > 1024:
                self._seen = {frame}
            old_exposure, old_receive = self._last_seen_exposure_ns, self._last_seen_receive_ns
            self._last_seen_exposure_ns, self._last_seen_receive_ns = exposure, int(now_ns)
            if old_exposure is not None and (exposure-old_exposure > 150_000_000 or now_ns-old_receive > 150_000_000):
                self.revoke('visual update interval exceeded')
            if not 0 <= now_ns-exposure <= 350_000_000:
                return self.revoke('visual exposure age exceeded')
            if not accepted:
                return self.revoke('navigation rejected: ' + str(navigation_reason or result.get('reason', 'innovation')))
            _, age, wall_source, wall_receive = image_clock(result, now_ns, received_wall_time_ns)
            if age > 350_000_000:
                return self.revoke('visual image pipeline age exceeded')
            if wall_source is not None:
                previous_source, previous_receive = self._last_seen_wall_source_ns, self._last_seen_wall_receive_ns
                self._last_seen_wall_source_ns, self._last_seen_wall_receive_ns = wall_source, wall_receive
                if previous_source is not None and (wall_source <= previous_source or wall_receive < previous_receive):
                    return self.revoke('nonmonotonic localhost image clock')
                if previous_source is not None and (wall_source-previous_source > 150_000_000 or wall_receive-previous_receive > 150_000_000):
                    self.revoke('actual visual update interval exceeded')
            if result.get('status') != 'ok' or result.get('ambiguous', True):
                return self.revoke('invalid or ambiguous stereo solution')
            counts = result['counts']
            if min(counts['used_markers_left'], counts['used_markers_right'], counts['common_markers']) < 2:
                return self.revoke('insufficient complete known inlier markers')
            if any(not 0 <= float(result['reprojection_error_px'][eye]) <= 2.5
                   or not 0 <= float(result['max_corner_error_px'][eye]) <= 5 for eye in ('left', 'right')):
                return self.revoke('image reprojection quality decreased')
            c = result['pose_covariance']
            if (c['frame'] != 'left_optical' or c['rotation_error'] != 'left_multiplicative'
                    or c.get('order') != POSE_ERROR_ORDER):
                return self.revoke('unsupported pose covariance convention')
            measured = covariance(c['matrix'], 6)
            p = covariance(relative_position_covariance, 3)
            a = covariance(relative_attitude_covariance, 3)
            if math.sqrt(measured[2, 2]) >= .03 or principal_sigma(measured[3:, 3:]) >= math.radians(2):
                return self.revoke('visual measurement uncertainty exceeded')
            if principal_sigma(p) >= .04 or principal_sigma(a) >= math.radians(.3):
                return self.revoke('relative navigation uncertainty exceeded')
            self.last_exposure_ns, self.last_receive_ns = exposure, int(now_ns)
            self.last_wall_source_ns, self.last_wall_receive_ns = wall_source, wall_receive
            event = 'visual/'+'/'.join(str(result[k]) for k in ('mission_id', 'session_id', 'source_frame_id', 'calibration_id'))
            self._accepted_events = (self._accepted_events+[event])[-self.minimum:]
            self.count += 1
            self.ready = self.count >= self.minimum
            self.reason = 'ready' if self.ready else 'collecting distinct accepted exposures'
            return self.ready
        except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError):
            return self.revoke('malformed visual observation')

    def status(self, now_ns, received_wall_time_ns=None):
        self.tick(now_ns, received_wall_time_ns)
        age = None if self.last_exposure_ns is None else (now_ns-self.last_exposure_ns)*1e-9
        if age is not None and received_wall_time_ns is not None and self.last_wall_source_ns is not None:
            age = max(age, (received_wall_time_ns-self.last_wall_source_ns)*1e-9)
        return dict(ready=self.ready, count=self.count, reason=self.reason,
                    exposure_age_s=age)
