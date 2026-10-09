"""Teleop IK: turn operator Cartesian commands into joint reference targets."""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot


class TeleopIkComponent:
    """Schedule the Cartesian IK controller at its own rate, below the clock."""

    name = "teleop_ik"

    def __init__(self, target: Any, *, ik_rate_hz: int) -> None:
        self.target = target
        self.ik_rate_hz = int(ik_rate_hz)
        self.controller: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from space_arm_platform.sampling import ik_step_stride

        from simulation.teleop_grasp_unreal import CartesianIkControlModel

        controller = CartesianIkControlModel(
            self.target, clock=ctx.clock, stride=ik_step_stride(self.ik_rate_hz))
        self.controller = controller
        # COMMAND segment, above the joint reference publisher: the publisher reads
        # the cache this controller just refreshed.
        ctx.add(controller, Slot.COMMAND, every=1)
        ctx.keep_alive(controller)
