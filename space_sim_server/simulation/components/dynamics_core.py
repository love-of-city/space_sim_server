"""Dynamics core: the fixed-step MuJoCo stepper that integrates every MJCF body.

This component is the only writer of the multibody state. It runs in the middle of
the physics task (Slot.PHYSICS): modules before it read t(k-1), modules after it
read t(k). On the basilisk backend the core is MJScene's own RKF45 integration,
which the scenario builder installs, so this component is a no-op there.
"""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot


class DynamicsCoreComponent:
    """Install and own the local MuJoCo physics core (local backend only)."""

    name = "dynamics_core"

    def __init__(self, model_path: Any, *, substeps: int | None = None) -> None:
        self.model_path = model_path
        self.substeps = substeps
        self.stepper: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        if ctx.config.dynamics_backend != "local":
            return
        stepper = ctx.simulation.local_stepper
        # Binding through the port also applies the propagated origin O if the
        # orbit component registered it first (and either order is valid).
        ctx.ports.set_stepper(stepper)
        ctx.config.publish_stride = stepper.publish_every
        # The scenario builder created the core (it needs the model before the
        # controller chain exists), so record it here rather than add it again.
        ctx.note_external(stepper, component=self.name, task="graspTask",
                          priority=int(Slot.PHYSICS), every=1, phase=0)
        ctx.keep_alive(stepper)
        self.stepper = stepper
