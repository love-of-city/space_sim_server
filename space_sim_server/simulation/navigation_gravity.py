"""Estimated-state gravity queries, independent of the propagated spacecraft.

The flight controller and navigation filters share native BSK field parameters
and public ephemerides with truth dynamics, never spacecraft truth coordinates.
Query messages are separate: delayed filter prediction must not rewind the
physics adapter's source messages, sample clock or held forces.
"""
from collections import OrderedDict

import numpy as np

from simulation.orbital_rendezvous.reference import MU, gravity


class PointMassNavigationGravity:
    """Historical filter model, also usable without a native BSK installation."""

    def acceleration(self, position, time_s):
        return gravity(np.asarray(position, dtype=float))

    def gradient(self, position, time_s):
        r = np.asarray(position, dtype=float)
        radius = np.linalg.norm(r)
        return MU * (3 * np.outer(r, r) / radius**5 - np.eye(3) / radius**3)


class ReplayGravity:
    """Reuse a native local field linearization at the identical replay epoch.

    Reuse is bounded to 25 cm, with no time interpolation. At LEO the quadratic
    acceleration remainder is below 1e-11 m/s², independently checked against
    the actual native field at startup. A larger correction queries the field
    again. The cache contains estimates and public field parameters only.
    """
    def __init__(self, field):
        self.field = field
        self._gradients = OrderedDict()

    def acceleration(self, position, time_s):
        position = np.asarray(position, dtype=float)
        entry = self._entry(position, time_s)
        anchor, acceleration, gradient = entry
        if gradient is not None:
            return acceleration+gradient@(position-anchor)
        if np.array_equal(anchor, position):
            return acceleration.copy()
        return self.field.acceleration(position, time_s)

    def _entry(self, position, time_s):
        position = np.asarray(position, dtype=float)
        epoch = round(time_s*1e9)
        entries = self._gradients.get(epoch, [])
        for entry in entries:
            anchor = entry[0]
            if np.linalg.norm(position-anchor) <= .25:
                return entry
        entry = [position.copy(), self.field.acceleration(position, time_s), None]
        self._gradients[epoch] = (entries+[entry])[-4:]
        self._gradients.move_to_end(epoch)
        while len(self._gradients) > 512:
            self._gradients.popitem(last=False)
        return entry

    def gradient(self, position, time_s):
        entry = self._entry(position, time_s)
        if entry[2] is None:
            entry[2] = self.field.gradient(entry[0], time_s)
        return entry[2]


class BskNavigationGravity:
    """Earth-centered acceleration at an estimated J2000 position and epoch."""

    def __init__(self, environment, *, epoch_s=0.):
        from Basilisk.architecture import messaging
        from Basilisk.simulation import NBodyGravity

        self.gravity = NBodyGravity.NBodyGravity()
        if not np.isfinite(epoch_s) or epoch_s < 0:
            raise ValueError('invalid navigation gravity epoch offset')
        self.epoch_s = float(epoch_s)
        self.gravity.ModelTag = "estimatedStateGravity"
        self.sampler = environment.ephemeris_sampler
        self.sources = []
        self._sample_key = None
        self._sample_cache = OrderedDict()
        for name, source in environment.sources.items():
            body = source["body"]
            bridge = messaging.SpicePlanetStateMsg()
            self.gravity.addGravitySource(name, body.gravityModel, bool(body.isCentralBody))
            native = self.gravity.getGravitySource(name)
            native.stateInMsg.subscribeTo(bridge)
            self.sources.append({"body": body, "reader": source["input_message"].addSubscriber(),
                                 "message": bridge, "native": native})
        if sum(bool(s["body"].isCentralBody) for s in self.sources) != 1:
            raise ValueError("navigation requires exactly one central gravity source")
        self.gravity.Reset(0)

    def _sample(self, time_s):
        if not np.isfinite(time_s) or time_s < -1e-9:
            raise ValueError("invalid navigation gravity epoch")
        nanos = round((time_s-self.epoch_s) * 1e9)
        epochs = tuple(int(s["reader"].timeWritten()) for s in self.sources)
        key = nanos, epochs
        if key == self._sample_key:
            return
        cached = self._sample_cache.get(key)
        payloads = []
        for index, (source, epoch) in enumerate(zip(self.sources, epochs, strict=True)):
            reader = source["reader"]
            if not reader.isWritten():
                raise RuntimeError("navigation ephemeris has not been published")
            payload = cached[index] if cached is not None else self.sampler.sample(reader(), epoch, nanos,
                point_mass=type(source["body"].gravityModel).__name__ == "PointMassGravityModel")
            payloads.append(payload)
            source["message"].write(payload, max(0, nanos))
            if source["body"].isCentralBody:
                self.central_position = np.asarray(payload.PositionVector)
        self._sample_key = key
        self._sample_cache[key] = payloads
        self._sample_cache.move_to_end(key)
        while len(self._sample_cache) > 512:
            self._sample_cache.popitem(last=False)

    def acceleration(self, position, time_s):
        r = np.asarray(position, dtype=float)
        if r.shape != (3,) or not np.isfinite(r).all() or np.linalg.norm(r) < 1:
            raise ValueError("invalid estimated position for native gravity")
        self._sample(time_s)
        compute = self.gravity.computeAccelerationFromSource
        result = np.zeros(3)
        for source in self.sources:
            result += np.asarray(compute(source["native"], r + self.central_position)).reshape(3)
            if not source["body"].isCentralBody:
                result -= np.asarray(compute(source["native"], self.central_position)).reshape(3)
        return result

    def gradient(self, position, time_s):
        # Symmetric 1 m probes of the same native field. No duplicated J2 formula.
        r = np.asarray(position, dtype=float)
        basis = np.eye(3)
        return np.column_stack([(self.acceleration(r + delta, time_s)
                                 - self.acceleration(r - delta, time_s)) / 2 for delta in basis])
