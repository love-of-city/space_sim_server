"""Attitude control: the reaction-wheel chain, scheduled on the physics grid.

The chain used to run in its own 100 Hz task, which is not a division of the 240 Hz
physics grid, so it read a body state whose age varied between 0 and 6.7 ms and
stamped its outputs with a time that did not correspond to the state it used. It now
runs on the physics task at a fixed division of the grid (120 Hz by default), phase
aligned with state publication, so the input age is exactly one physics step.

The wheel drives keep their own slot (ACTUATOR): MuJoCo holds the motor command for
the whole step on the local backend, so the torque and speed limits must see fresh
wheel speed each step, not a stale substep value.
"""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot

WHEEL_DRIVE_PRIORITY = 2_000
ATTITUDE_CHAIN_PRIORITY = 8_500


class AttitudeComponent:
    """Install the reaction-wheel control chain and its drives."""

    name = "attitude"

    def __init__(self, model_path: Any, *, enabled: bool | None = None) -> None:
        self.model_path = model_path
        self.enabled = enabled
        self.control: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from simulation.attitude_control import AttitudeControl

        # The scenario builder defers this (defer_attitude_control=True), so this is
        # the only place the chain is installed. Creating it in both places would
        # schedule the whole chain twice and leave one copy spinning without effect.
        if getattr(ctx.simulation, "attitude_control", None) is not None:
            raise RuntimeError(
                "the attitude control chain is already installed; the scenario builder "
                "must defer it (defer_attitude_control=True) when components assemble it")
        local = ctx.config.dynamics_backend == "local"
        drive_task = (ctx.config.physics_task, WHEEL_DRIVE_PRIORITY) if local else None
        physics_grid = None
        if local:
            from space_arm_platform.sampling import DYNAMICS_HZ
            from simulation.attitude_control import load_settings

            rate = load_settings(self.model_path.with_name("attitude_control.json"))["control_rate_hz"]
            every = int(round(DYNAMICS_HZ / rate))
            physics_grid = (ctx.config.physics_task, ATTITUDE_CHAIN_PRIORITY, every, ctx.clock)
        control = AttitudeControl(
            ctx.simulation, self._process(ctx), ctx.scene, self.model_path,
            enabled=self.enabled, drive_task=drive_task, physics_grid=physics_grid)
        self.control = control
        ctx.simulation.attitude_control = control
        ctx.extra["attitude_control"] = control
        # The chain installs itself straight into Basilisk tasks (it needs a second
        # task on the basilisk backend), so report each module it placed. Without
        # this the schedule dump would omit the whole chain, which is exactly how a
        # duplicated installation stayed invisible.
        for model, task, priority, every, phase in control.scheduled:
            ctx.note_external(model, component=self.name, task=task, priority=priority,
                              every=every, phase=phase)
        # On the local backend the wheel drives are a separate module the chain adds
        # itself; on the basilisk backend they live on the MJScene dynamics task.
        if local:
            ctx.note_external(control.drive_group, component=self.name,
                              task=ctx.config.physics_task, priority=WHEEL_DRIVE_PRIORITY,
                              every=1, phase=0)
        else:
            ctx.note_external(control.drive_group, component=self.name, task="MJScene",
                              priority=6_500, every=0, phase=0)
        ctx.keep_alive(control)

    @staticmethod
    def _process(ctx: AssemblyContext) -> Any:
        """The process that owns the physics task; AttitudeControl registers on it."""
        for process in ctx.simulation.procList:
            if process.Name == "graspProcess":
                return process
        return ctx.simulation.procList[0]


_ = Slot
