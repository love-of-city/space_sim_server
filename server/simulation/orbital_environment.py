"""Adapter for native Basilisk orbital components; no gravity formula lives here.

NBodyGravity evaluates shared GravBodyData.gravityModel objects at each COM and
at the reference point via its native computeAccelerationFromSource method.
Subtracting per source removes common acceleration (the central-body indirect
term cancels too), without costly per-body SWIG message copies. MuJoCo remains
the sole multibody owner; source ephemerides are native BSK messages.
"""
from __future__ import annotations

import os
from typing import Any

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import NBodyGravity, gravityEffector

from simulation.orbital_frames import interpolate_origin  # re-export for callers

ORBITAL_MODES = ("bsk", "linear_tidal")


def orbital_mode(value: str | None = None) -> str:
    mode = str(value if value is not None else os.environ.get("SPACE_SIM_ORBITAL_MODE", "bsk")).strip().lower()
    if mode not in ORBITAL_MODES:
        raise ValueError(f"orbital mode must be one of {ORBITAL_MODES}")
    return mode


class LinearEphemerisSampler:
    """Sample BSK messages near their epoch, without advancing an outer task.

    Translation uses the supplied velocity; orientation uses its derivative
    projected onto SO(3). This is a short-interval approximation, not a SPICE
    replacement. A different sampler can implement the same sample method.
    Missing orientation is accepted only for native point-mass gravity.
    """

    name = "position_velocity_extrapolation_rotation_projection"

    def __init__(self, max_offset_s: float = 1.0):
        if not np.isfinite(max_offset_s) or max_offset_s <= 0:
            raise ValueError("max ephemeris sampling offset must be positive and finite")
        self.max_offset_s = float(max_offset_s)

    def sample(self, original, epoch_ns: int, nanos: int, *, point_mass: bool):
        dt = (nanos - epoch_ns) * 1e-9
        if abs(dt) > self.max_offset_s:
            raise ValueError("ephemeris sample too far from message epoch; update its provider")
        payload = messaging.SpicePlanetStateMsgPayload()
        position = np.asarray(original.PositionVector, dtype=float)
        velocity = np.asarray(original.VelocityVector, dtype=float)
        rotation = np.asarray(original.J20002Pfix, dtype=float)
        derivative = np.asarray(original.J20002Pfix_dot, dtype=float)
        if not all(np.isfinite(a).all() for a in (position, velocity, rotation, derivative)):
            raise ValueError("non-finite orbital ephemeris")
        if point_mass:
            rotation = np.eye(3)
        else:
            if (not np.allclose(rotation @ rotation.T, np.eye(3), rtol=0, atol=1e-7)
                    or not np.isclose(np.linalg.det(rotation), 1., rtol=0, atol=1e-7)):
                raise ValueError("non-point-mass gravity requires a valid body-fixed orientation")
            if dt and np.any(derivative):
                u, _, vt = np.linalg.svd(rotation + dt*derivative)
                rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
        payload.PositionVector = (position + dt*velocity).tolist()
        payload.VelocityVector = original.VelocityVector
        payload.J20002Pfix = rotation.tolist()
        payload.J20002Pfix_dot = original.J20002Pfix_dot
        payload.J2000Current = original.J2000Current + dt
        payload.PlanetName = original.PlanetName
        payload.computeOrient = original.computeOrient
        return payload


class BskOrbitalEnvironment:
    """Own the BSK field component and its messages; extensible before first step.

    Register the SAME GravBodyData objects on the origin Spacecraft and here, so
    point-mass, spherical-harmonic or other BSK GravityModel configuration cannot
    silently diverge. Additional components call add_gravity_source with a BSK
    SpicePlanetStateMsg (all sources must use the same ephemeris frame/center).
    This adapter does not apply spacecraft gravity-gradient torque automatically.
    """

    def __init__(self, orbit_reference: Any, *, ephemeris_sampler=None):
        self.orbit_reference = orbit_reference
        self.ephemeris_sampler = ephemeris_sampler or LinearEphemerisSampler()
        self.gravity = NBodyGravity.NBodyGravity()
        self.gravity.ModelTag = "localBskNBodyGravity"
        self.sources: dict[str, dict[str, Any]] = {}
        self._body_ids: np.ndarray | None = None
        self._masses: np.ndarray | None = None
        self._forces: np.ndarray | None = None
        self._active = False
        self.evaluations = 0
        self.last_time_ns: int | None = None
        self.max_differential_acceleration = 0.0

    def add_gravity_source(self, name: str, body: Any, state_message: Any) -> None:
        if self._active:
            raise RuntimeError("gravity sources are fixed for a session; rebuild the session to change them")
        name = str(name).strip()
        if not name or name in self.sources:
            raise ValueError(f"empty or repeated gravity source: {name!r}")
        if any(int(source["body"].this) == int(body.this) for source in self.sources.values()):
            raise ValueError("the same GravBodyData cannot be registered twice")
        if body.isCentralBody and any(s["body"].isCentralBody for s in self.sources.values()):
            raise ValueError("only one central gravity source is allowed")
        if not np.isfinite(body.mu) or body.mu <= 0:
            raise ValueError("gravity source mu must be positive and finite")
        # Installed BSK's NBodyGravity accepts GravityModel, not GravBodyData.
        # Let BSK initialize mu/harmonic coefficients from its own body object.
        body.initBody(self.gravity.moduleID)
        body.planetBodyInMsg.subscribeTo(state_message)
        bridge = messaging.SpicePlanetStateMsg()
        self.gravity.addGravitySource(name, body.gravityModel, bool(body.isCentralBody))
        self.gravity.getGravitySource(name).stateInMsg.subscribeTo(bridge)
        self.sources[name] = {"body": body, "reader": state_message.addSubscriber(),
                              "input_message": state_message,  # own Python writer lifetime too
                              "message": bridge, "native": self.gravity.getGravitySource(name),
                              "payload": None, "sample_position": np.zeros(3), "time_ns": 0,
                              "signature": self._source_signature(body)}
        self.orbit_reference.gravField.setGravBodies(
            gravityEffector.GravBodyVector([s["body"] for s in self.sources.values()]))

    def reset(self) -> None:
        """Reset time bookkeeping; normal platform resets rebuild all components."""
        self.last_time_ns = None
        self.evaluations = 0
        self.max_differential_acceleration = 0.0
        for source in self.sources.values():
            source["payload"] = None
        if self._active:
            self.gravity.Reset(0)

    def begin_interval(self) -> None:
        """Snapshot ephemeris once; extrapolate its position/orientation per substep."""
        for source in self.sources.values():
            if self._source_signature(source["body"]) != source["signature"]:
                raise ValueError("gravity source configuration changed; rebuild the session")
            reader = source["reader"]
            if not reader.isWritten():
                raise RuntimeError("orbital ephemeris message has not been written")
            source["payload"] = reader()
            source["time_ns"] = int(reader.timeWritten())

    @staticmethod
    def _source_signature(body):
        # Model coefficients must likewise be configured before registration.
        return (float(body.mu), bool(body.isCentralBody), int(body.gravityModel.this))

    def _write_sources(self, nanos: int) -> None:
        for source in self.sources.values():
            original = source["payload"]
            if original is None:
                raise RuntimeError("begin_interval must precede orbital force evaluation")
            # BSK 2.11.1's spherical model reports dependsOnOrientation=False.
            # Only the concrete native point-mass model is safe to fast-path;
            # never use that flag to discard an arbitrary model's orientation.
            point_mass = type(source["body"].gravityModel).__name__ == "PointMassGravityModel"
            payload = self.ephemeris_sampler.sample(
                original, source["time_ns"], nanos, point_mass=point_mass)
            if (not np.isfinite(np.asarray(payload.PositionVector)).all()
                    or not np.isfinite(np.asarray(payload.J20002Pfix)).all()):
                raise ValueError("non-finite orbital ephemeris")
            source["sample_position"] = np.asarray(payload.PositionVector)
            source["message"].write(payload, nanos, self.gravity.moduleID)

    def _initialize_targets(self, masses: np.ndarray) -> None:
        if not self.sources or sum(bool(s["body"].isCentralBody) for s in self.sources.values()) != 1:
            raise ValueError("orbital environment requires exactly one central gravity source")
        if not np.isfinite(masses).all() or np.any(masses < 0):
            raise ValueError("body masses must be finite and nonnegative")
        self._masses = masses.copy()
        self._body_ids = np.flatnonzero(masses > 0)
        self._body_ids = self._body_ids[self._body_ids != 0]  # never force MuJoCo world
        self._forces = np.zeros((len(masses), 3))
        self.gravity.Reset(0)
        self._active = True

    def forces(self, positions_local: np.ndarray, masses: np.ndarray,
               origin_position: np.ndarray, nanos: int) -> np.ndarray:
        """Evaluate full BSK fields, returning only relative per-body forces [N]."""
        nanos = int(nanos)
        if nanos < 0:
            raise ValueError("orbital time must be nonnegative")
        if self.last_time_ns is not None and nanos < self.last_time_ns:
            raise ValueError("orbital time went backwards; reset the session")
        if (masses.ndim != 1 or positions_local.shape != (len(masses), 3) or np.shape(origin_position) != (3,)
                or not np.isfinite(positions_local).all() or not np.isfinite(origin_position).all()):
            raise ValueError("invalid COM/origin positions")
        if not self._active:
            self._initialize_targets(masses)
        elif not np.array_equal(masses, self._masses):
            raise ValueError("runtime mass changes need an explicit orbital mass update contract")
        self._write_sources(nanos)
        # BSK's native method expects positions relative to the SPICE origin.
        # MuJoCo/Spacecraft coordinates are central-body centered. The central
        # acceleration subtraction performed by computeAccelerationOnTarget is
        # common to body and O, so cancels exactly in this differential.
        central = next(s["sample_position"] for s in self.sources.values() if s["body"].isCentralBody)
        origin_spice = origin_position + central
        positions_spice = positions_local[self._body_ids] + origin_spice
        accelerations = np.zeros_like(positions_spice)
        compute = self.gravity.computeAccelerationFromSource
        for source in self.sources.values():
            native = source["native"]
            common = np.asarray(compute(native, origin_spice)).reshape(3)
            for row, position in enumerate(positions_spice):
                accelerations[row] += np.asarray(compute(native, position)).reshape(3) - common
        self._forces[self._body_ids] = masses[self._body_ids, None] * accelerations
        if len(accelerations):
            self.max_differential_acceleration = max(self.max_differential_acceleration,
                                                    float(np.linalg.norm(accelerations, axis=1).max()))
        if not np.isfinite(self._forces).all():
            raise FloatingPointError("BSK orbital field produced non-finite force")
        self.evaluations += 1
        self.last_time_ns = nanos
        return self._forces

    def telemetry(self) -> dict:
        return {"mode": "bsk", "component": "NBodyGravity",
                "evaluation_api": "computeAccelerationFromSource", "sources": [
            {"name": name, "model": type(s["body"].gravityModel).__name__,
             "central": bool(s["body"].isCentralBody), "mu_m3_s2": float(s["body"].mu)}
            for name, s in self.sources.items()], "evaluations": self.evaluations,
            "bodies": 0 if self._body_ids is None else len(self._body_ids),
            "max_differential_acceleration_m_s2": self.max_differential_acceleration,
            "ephemeris_sampling": getattr(self.ephemeris_sampler, "name", type(self.ephemeris_sampler).__name__),
            "origin_sampling": "cubic_hermite", "force_cadence": "each_mujoco_substep"}
