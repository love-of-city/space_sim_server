"""Simulation-time delivery and prediction of time-tagged paired GNSS packets.

Only the receiver samples truth. Guidance receives delivered measurements and a
prediction using its own recorded commands. No wall-clock sleep, future packet
or current truth is used to compensate communication latency.
"""
from collections import deque
import copy

import numpy as np

from simulation.differential_gnss import DifferentialGNSS, JointOrbitFilter
from simulation.orbital_rendezvous.reference import MU, gravity


def _propagate(state, covariance, dt, accelerations):
    state = state.copy()
    transition, noise = np.eye(12), np.zeros((12, 12))
    for i, control in enumerate(accelerations):
        r, v = state[i, :3].copy(), state[i, 3:].copy()
        acceleration = gravity(r) + control
        position = r + dt*v + .5*dt*dt*acceleration
        state[i] = np.r_[position, v + .5*dt*(acceleration + gravity(position) + control)]
        radius = np.linalg.norm(r)
        gradient = MU * (3*np.outer(r, r)/radius**5 - np.eye(3)/radius**3)
        block = np.eye(6)
        block[:3, 3:] = dt*np.eye(3)
        block[3:, :3] = dt*gradient
        block[:3, :3] += .5*dt*dt*gradient
        injection = np.vstack((.5*dt*dt*np.eye(3), dt*np.eye(3)))
        selection = slice(i*6, (i+1)*6)
        transition[selection, selection] = block
        noise[selection, selection] = injection @ injection.T * .003**2
    covariance = transition @ covariance @ transition.T + noise
    return state, covariance


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list, deque)):
        return [_json_value(item) for item in value]
    return value


class DelayedGNSSNavigation:
    """Keep measurement-epoch posteriors separate from current predictions.

    ``accelerations`` in step() is the command applied over the interval ending
    at ``now``. The receiver runs at its configured rate; delivery/prediction may
    run faster. In-flight samples and their reference geometry survive handoff.
    """
    def __init__(self, config, seed=7):
        self.receiver = DifferentialGNSS(config, seed)
        self.filter = JointOrbitFilter()
        self.pending = deque()
        self.controls = deque()
        self.now = None
        self.estimate = None
        self.covariance = None
        self.last_packet = None
        self.last_receive_time = None
        self.first_receive_time = None
        self.delivered_count = 0
        self.maximum_queue_depth = 0
        # Evaluation only: never read by filtering, prediction or guidance.
        self.measurement_audit = []

    def _project(self, epoch):
        state, covariance = self.filter.state.copy(), self.filter.covariance.copy()
        cursor = self.filter.last_stamp
        if epoch < cursor - 1e-9:
            raise ValueError("cannot predict GNSS navigation backwards")
        for start, end, acceleration in self.controls:
            if end <= cursor + 1e-10:
                continue
            if start > cursor + 1e-8:
                raise ValueError("GNSS prediction has a gap in command history")
            until = min(end, epoch)
            if until > cursor:
                state, covariance = _propagate(state, covariance, until-cursor, acceleration)
                cursor = until
            if cursor >= epoch - 1e-10:
                break
        if cursor < epoch - 1e-8:
            raise ValueError("GNSS prediction lacks command history")
        return state, covariance

    def step(self, truth, now, accelerations, *, origin_offsets=None, cooperative=None):
        now = float(now)
        acceleration = np.asarray(accelerations, dtype=float)
        if (not np.isfinite(now) or now < 0 or acceleration.shape != (2, 3)
                or not np.isfinite(acceleration).all()):
            raise ValueError("invalid GNSS delivery clock or commanded acceleration")
        if self.now is not None:
            if now <= self.now:
                raise ValueError("GNSS delivery clock must advance")
            self.controls.append((self.now, now, acceleration.copy()))
        self.now = now
        period = 1/self.receiver.config.sample_rate_hz
        last = self.receiver.last_time
        if last is None or now-last >= period-1e-9:
            packet = self.receiver.measure(truth, now)
            self.measurement_audit.append({"stamp": now, "due": packet["due"], "sequence": packet["sequence"],
                                           "truth": np.asarray(truth, dtype=float).copy(),
                                           "observations": packet["observations"].copy(), "received": np.nan})
            if origin_offsets is not None:
                offsets = np.asarray(origin_offsets, dtype=float)
                if offsets.shape != (2, 6) or not np.isfinite(offsets).all():
                    raise ValueError("invalid GNSS reference-point offsets")
                packet["origin_offsets"] = offsets.copy()
            if cooperative is not None:
                packet["cooperative"] = copy.deepcopy(cooperative)
            self.pending.append(packet)
            self.maximum_queue_depth = max(self.maximum_queue_depth, len(self.pending))
        while self.pending and self.pending[0]["due"] <= now+1e-9:
            packet = self.pending.popleft()
            if self.filter.state is None:
                self.filter.update(packet, np.zeros((2, 3)))
            else:
                state, covariance = self._project(packet["stamp"])
                self.filter.update_projected(packet, state, covariance)
            self.last_packet = packet
            self.last_receive_time = now
            if self.first_receive_time is None:
                self.first_receive_time = now
            self.delivered_count += 1
            self.measurement_audit[packet["sequence"]-1]["received"] = now
        if self.filter.state is not None:
            self.estimate, self.covariance = self._project(now)
            while self.controls and self.controls[0][1] <= self.filter.last_stamp+1e-10:
                self.controls.popleft()
        return None if self.estimate is None else self.estimate.copy()

    @property
    def relative_covariance(self):
        if self.covariance is None:
            return None
        difference = np.hstack((np.eye(6), -np.eye(6)))
        return difference @ self.covariance @ difference.T

    def telemetry(self, now=None):
        packet = self.last_packet
        now = self.now if now is None else now
        return {"gnss_sample_time_s": packet["stamp"] if packet else None,
                "gnss_receive_time_s": self.last_receive_time,
                "gnss_measurement_age_s": now-packet["stamp"] if packet else None,
                "gnss_delivered_sequence": packet["sequence"] if packet else None,
                "gnss_acquired_sequence": self.receiver.sequence,
                "gnss_pending_count": len(self.pending),
                "gnss_delivered_count": self.delivered_count,
                "gnss_requested_delay_s": self.receiver.config.delay_s}

    def snapshot(self):
        return {"schema": "spacesim-gnss-transport/1", "epoch_s": self.now,
                "reference": "assembled_center_of_mass",
                "pending": _json_value(self.pending),
                "first_receive_time_s": self.first_receive_time,
                "maximum_queue_depth": self.maximum_queue_depth,
                **self.telemetry()}
