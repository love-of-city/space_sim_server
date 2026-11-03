"""Estimated-state gravity queries, independent of the propagated spacecraft.

The flight controller and navigation filters share native BSK field parameters
and public ephemerides with truth dynamics, never spacecraft truth coordinates.
Query messages are separate: delayed filter prediction must not rewind the
physics adapter's source messages, sample clock or held forces.
"""
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


class BskNavigationGravity:
    """Earth-centered acceleration at an estimated J2000 position and epoch."""

    def __init__(self, environment):
        from Basilisk.architecture import messaging
        from Basilisk.simulation import NBodyGravity

        self.gravity = NBodyGravity.NBodyGravity()
        self.gravity.ModelTag = "estimatedStateGravity"
        self.sampler = environment.ephemeris_sampler
        self.sources = []
        self._sample_key = None
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
        nanos = round(time_s * 1e9)
        epochs = tuple(int(s["reader"].timeWritten()) for s in self.sources)
        key = nanos, epochs
        if key == self._sample_key:
            return
        for source, epoch in zip(self.sources, epochs, strict=True):
            reader = source["reader"]
            if not reader.isWritten():
                raise RuntimeError("navigation ephemeris has not been published")
            payload = self.sampler.sample(reader(), epoch, nanos,
                point_mass=type(source["body"].gravityModel).__name__ == "PointMassGravityModel")
            source["message"].write(payload, nanos)
            if source["body"].isCentralBody:
                self.central_position = np.asarray(payload.PositionVector)
        self._sample_key = key

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
