"""Physics clock: drive the physics task on a drift-free rational 240 Hz grid."""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot


class ClockComponent:
    """Install `RationalPhysicsClock` first in the physics task.

    Every other module's rate and phase is expressed against this clock's
    `step_index`, so it must exist before anything is placed.
    """

    name = "clock"

    def __init__(self, task_name: str = "graspTask") -> None:
        self.task_name = task_name
        self.clock: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from simulation.physics_clock import RationalPhysicsClock

        task = next((task for task in ctx.simulation.TaskList if task.Name == self.task_name), None)
        if task is None:
            raise RuntimeError(f"physics task {self.task_name!r} does not exist")
        clock = RationalPhysicsClock(task)
        self.clock = clock
        # The clock is placed first so AssemblyContext can hand it to every
        # later divider; it registers at the top of the CLOCK segment.
        ctx.clock = clock
        ctx.add(clock, Slot.CLOCK, every=1)
        ctx.keep_alive(clock)
