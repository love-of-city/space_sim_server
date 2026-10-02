"""Orbit and environment: SPICE ephemeris plus the orbital reference point O.

On the local backend Basilisk propagates only O (a point-mass `Spacecraft`); every
MJCF body lives in the local frame L carried by O. On the basilisk backend the
same gravity factory drives MJScene's own integration instead, so this component
only creates the ephemeris and reports what the caller must bind.
"""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot

EPHEMERIS_PRIORITY = 19_000


class OrbitComponent:
    """SPICE ephemeris, Earth/Sun gravity sources, and (local) the origin O."""

    name = "orbit"

    def __init__(self, *, epoch_utc: str, central_body: str = "Earth", frame: str = "J2000",
                 fixed_frames: tuple[str, ...] = ("IAU_EARTH", "IAU_SUN")) -> None:
        self.epoch_utc = epoch_utc
        self.central_body = central_body
        self.frame = frame
        self.fixed_frames = fixed_frames
        self.gravity_factory: Any = None
        self.earth: Any = None
        self.sun: Any = None
        self.ephemeris: Any = None
        self.spacecraft: Any = None
        self.origin: Any = None
        self.gravity: Any = None
        self.earth_model: Any = None
        self.sun_model: Any = None
        self.gravity_targets: list[str] = []

    def install(self, ctx: AssemblyContext) -> None:
        from Basilisk.simulation import NBodyGravity, pointMassGravityModel, spacecraft
        from Basilisk.utilities import simIncludeGravBody

        from simulation.teleop_grasp_unreal import _create_ephemeris_interface

        gravity_factory = simIncludeGravBody.gravBodyFactory()
        earth = gravity_factory.createEarth()
        sun = gravity_factory.createSun()
        earth.isCentralBody = True
        ephemeris = _create_ephemeris_interface(gravity_factory, self.epoch_utc)
        self.gravity_factory, self.earth, self.sun, self.ephemeris = (
            gravity_factory, earth, sun, ephemeris)
        ctx.ports.set_ephemeris(ephemeris)
        # The renderer draws the same Earth/Sun state messages gravity uses, so it
        # reads them from here rather than creating a second ephemeris.
        ctx.extra.setdefault("celestial", (earth, sun))

        if ctx.config.dynamics_backend == "local":
            # Basilisk propagates only O under the same SPICE Earth/Sun field; the
            # physics core carries every body relative to O and applies the tidal
            # term about it. SPICE and O run at the physics rate, before IK.
            ctx.add(ephemeris, Slot.ORBIT, every=1)
            orbit_reference = spacecraft.Spacecraft()
            orbit_reference.ModelTag = "localFrameOrigin"
            orbit_reference.hub.mHub = 1.0  # [kg] point carrier; mass does not enter its motion
            gravity_factory.addBodiesTo(orbit_reference)
            ctx.add(orbit_reference, Slot.ORBIT, every=1)
            ctx.ports.set_orbit_reference(orbit_reference)
            self.spacecraft = orbit_reference
            self.gravity_targets = [orbit_reference.ModelTag]
            from simulation.local_mujoco_stepper import SpacecraftOrbitOrigin
            self.origin = SpacecraftOrbitOrigin(orbit_reference, float(earth.mu))
            # The physics core integrates every body relative to O and applies the
            # tidal term about it; hand O over through the port so the core never
            # falls back to its static, gravity-free default origin.
            ctx.ports.set_orbit_origin(self.origin)
            ctx.keep_alive(self.origin)
            return

        # basilisk backend: MJScene integrates every body, so gravity is registered
        # on its dynamics task. The source strengths stay zero until the initial
        # state has been assigned; `enable_gravity` is called for that.
        ctx.scene.AddModelToDynamicsTask(ephemeris, EPHEMERIS_PRIORITY)
        gravity = NBodyGravity.NBodyGravity()
        gravity.ModelTag = "earthSunGravity"
        # Higher priorities run first: MJScene forward kinematics (10_000) must
        # publish this substep's states before gravity reads them.
        ctx.scene.AddModelToDynamicsTask(gravity, 9_500)
        earth_model = pointMassGravityModel.PointMassGravityModel()
        earth_model.muBody = 0.0
        gravity.addGravitySource("earth", earth_model, True)
        gravity.getGravitySource("earth").stateInMsg.subscribeTo(ephemeris.planetStateOutMsgs[0])
        sun_model = pointMassGravityModel.PointMassGravityModel()
        sun_model.muBody = 0.0
        gravity.addGravitySource("sun", sun_model, False)
        gravity.getGravitySource("sun").stateInMsg.subscribeTo(ephemeris.planetStateOutMsgs[1])
        self.gravity_targets = list(ctx.scene.getBodyNames())
        for name in self.gravity_targets:
            gravity.addGravityTarget(name, ctx.scene.getBody(name))
        self.gravity, self.earth_model, self.sun_model = gravity, earth_model, sun_model
        ctx.keep_alive(gravity, earth_model, sun_model)

    def enable_gravity(self) -> None:
        """Turn on the real field strengths (basilisk backend only)."""
        if self.earth_model is None:
            return
        self.earth_model.muBody = float(self.earth.mu)
        self.sun_model.muBody = float(self.sun.mu)
