"""Atmospheric drag: the first component added through the documented extension path.

This is phase 6 of the migration (docs/MUJOCO_CORE_ARCHITECTURE.md §8.2): a new
environment *force* that touches no core code. It is one new file plus one line in
the session's component list, and it exercises the parts of the design that only
matter once something new is added:

* the ENVIRONMENT slot, which nothing used before;
* a divided rate (`every=24`, 10 Hz) whose phase is aligned with state publication;
* a read of ``planet_state`` (SPICE) and of the body COM state;
* the additive body-wrench port, so another source (thrusters, SRP) can act on the
  same body at the same time without either knowing about the other.

Basilisk's own ``dragDynamicEffector`` cannot be used here: it attaches to a
``spacecraft.Spacecraft`` and joins *its* equations of motion, which do not exist on
this backend (see §7). The mechanical part lives in MJCF; the force law lives here.

Default: disabled. Nothing changes in a production session unless a scenario asks
for it, so the physics stays byte-identical by default.
"""
from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
from Basilisk.architecture import messaging, sysModel

from simulation.assembly import AssemblyContext, Slot

# Earth's spin rate about the J2000 z axis. Used to build the co-rotating
# atmosphere velocity; the true pole is offset by a few arcseconds, which is far
# below the uncertainty of an exponential density model.
EARTH_ROTATION_RAD_S = 7.2921150e-5
# 240 Hz / 24 = 10 Hz. Density changes slowly; the force is held zero-order in
# between, and `every` must divide the body/site publication stride.
DRAG_EVERY = 24


def drag_force_n(density_kg_m3: float, relative_velocity_m_s: Sequence[float],
                 area_m2: float, drag_coefficient: float) -> np.ndarray:
    """Free-molecule drag on a flat plate: ``F = -0.5 rho Cd A |v_rel| v_rel``.

    Opposite to the relative wind, so it always removes energy from the orbit.
    """
    velocity = np.asarray(relative_velocity_m_s, dtype=float)
    speed = float(np.linalg.norm(velocity))
    if speed <= 0.0:
        return np.zeros(3)
    return (-0.5 * float(density_kg_m3) * float(drag_coefficient) * float(area_m2)
            * speed * velocity)


def atmosphere_velocity_n(position_n: Sequence[float]) -> np.ndarray:
    """Velocity of the co-rotating atmosphere at an inertial position [m/s]."""
    return np.cross(np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]), np.asarray(position_n, dtype=float))


class PlateDrag(sysModel.SysModel):
    """Turn atmospheric density and body motion into a held external wrench."""

    def __init__(self, density: Any, state: Any, *, area_m2: float, drag_coefficient: float,
                 center_of_pressure_body_m: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        super().__init__()
        self.ModelTag = "plateDrag"
        if not math.isfinite(area_m2) or area_m2 <= 0.0:
            raise ValueError("drag area must be finite and positive")
        if not math.isfinite(drag_coefficient) or drag_coefficient <= 0.0:
            raise ValueError("drag coefficient must be finite and positive")
        self.area_m2 = float(area_m2)
        self.drag_coefficient = float(drag_coefficient)
        self.center_of_pressure_body_m = np.asarray(center_of_pressure_body_m, dtype=float)
        self._density = density.addSubscriber()
        self._state = state.addSubscriber()
        self.forceOutMsg = messaging.CmdForceInertialMsg()
        self.torqueOutMsg = messaging.CmdTorqueBodyMsg()
        self.last_force_n = np.zeros(3)
        self.last_torque_nm = np.zeros(3)

    def UpdateState(self, current_sim_nanos: int) -> None:
        from Basilisk.utilities import RigidBodyKinematics as rbk

        density = float(self._density().neutralDensity)
        state = self._state()
        relative = (np.asarray(state.v_BN_N, dtype=float)
                    - atmosphere_velocity_n(state.r_BN_N))
        force_n = drag_force_n(density, relative, self.area_m2, self.drag_coefficient)
        # The wrench port takes an inertial-axis force at the COM plus a body-axis
        # torque, so an off-COM center of pressure is expressed as r_cp x F.
        force_b = rbk.MRP2C(np.asarray(state.sigma_BN, dtype=float)) @ force_n
        torque_b = np.cross(self.center_of_pressure_body_m, force_b)
        self.last_force_n, self.last_torque_nm = force_n, torque_b
        self.forceOutMsg.write(messaging.CmdForceInertialMsgPayload(
            forceRequestInertial=force_n.tolist()), current_sim_nanos, self.moduleID)
        self.torqueOutMsg.write(messaging.CmdTorqueBodyMsgPayload(
            torqueRequestBody=torque_b.tolist()), current_sim_nanos, self.moduleID)


class AtmosphericDragComponent:
    """Install the exponential atmosphere and its drag law on one body."""

    name = "atmospheric_drag"

    def __init__(self, body: str = "cubesat_bus", *, area_m2: float = 0.06,
                 drag_coefficient: float = 2.2,
                 center_of_pressure_body_m: Sequence[float] = (0.0, 0.0, 0.0),
                 base_density_kg_m3: float | None = None,
                 scale_height_m: float | None = None) -> None:
        self.body = body
        self.area_m2 = float(area_m2)
        self.drag_coefficient = float(drag_coefficient)
        self.center_of_pressure_body_m = center_of_pressure_body_m
        self.base_density_kg_m3 = base_density_kg_m3
        self.scale_height_m = scale_height_m
        self.atmosphere: Any = None
        self.drag: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from Basilisk.simulation import exponentialAtmosphere

        if ctx.ports.stepper is None:
            raise RuntimeError(
                "atmospheric drag is a local-backend component: it applies an external "
                "body wrench, which the basilisk path expresses as an effector instead")
        if not ctx.ports.has_body(self.body):
            raise RuntimeError(f"drag body {self.body!r} is not in this scene")

        atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
        atmosphere.ModelTag = "exponentialAtmosphere"
        atmosphere.addSpacecraftToModel(ctx.ports.body_com_state(self.body))
        atmosphere.planetPosInMsg.subscribeTo(ctx.ports.planet_state("earth"))
        if self.base_density_kg_m3 is not None:
            atmosphere.baseDensity = float(self.base_density_kg_m3)
        if self.scale_height_m is not None:
            atmosphere.scaleHeight = float(self.scale_height_m)

        drag = PlateDrag(
            atmosphere.envOutMsgs[0], ctx.ports.body_com_state(self.body),
            area_m2=self.area_m2, drag_coefficient=self.drag_coefficient,
            center_of_pressure_body_m=self.center_of_pressure_body_m)
        ctx.ports.add_body_wrench(self.body, source=self.name,
                                  force_inertial=drag.forceOutMsg, torque_body=drag.torqueOutMsg)
        ctx.add(atmosphere, Slot.ENVIRONMENT, every=DRAG_EVERY)
        ctx.add(drag, Slot.ENVIRONMENT, every=DRAG_EVERY)
        ctx.keep_alive(atmosphere, drag)
        self.atmosphere, self.drag = atmosphere, drag
