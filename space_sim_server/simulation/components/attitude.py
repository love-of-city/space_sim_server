"""Attitude control: the reaction-wheel chain, scheduled on the physics grid.

The chain runs at 50 Hz on the 400 Hz physics grid, phase-aligned with
body/site publication. The input age is exactly one physics step.

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

    def __init__(self, model_path: Any, *, enabled: bool | None = None,
                 estimated_navigation=None, initial_attitude_prior=None, reference_provider=None,
                 control_rate_hz=None) -> None:
        self.model_path = model_path
        self.enabled = enabled
        self.control: Any = None
        self.estimated_navigation = estimated_navigation
        self.initial_attitude_prior = initial_attitude_prior
        self.reference_provider = reference_provider
        self.control_rate_hz = control_rate_hz

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

            rate = self.control_rate_hz or load_settings(self.model_path.with_name("attitude_control.json"))["control_rate_hz"]
            every = int(round(DYNAMICS_HZ / rate))
            # The chain reads body states, which the core publishes every
            # `publish_stride` steps. It installs itself outside ctx.add, so apply
            # the same rule here. An injected NavAtt estimate has an independent
            # update rate and never reads the thinned physical body message.
            stride = max(1, int(ctx.config.publish_stride))
            if self.estimated_navigation is None and every > 1 and every % stride:
                from simulation.physics_ports import PortError
                raise PortError(
                    f"attitude control at {rate:g} Hz runs every {every} physics steps, not a multiple "
                    f"of the body/site publication stride {stride}; it would read stale states")
            physics_grid = (ctx.config.physics_task, ATTITUDE_CHAIN_PRIORITY, every, ctx.clock)
        control = AttitudeControl(
            ctx.simulation, self._process(ctx), ctx.scene, self.model_path,
            enabled=self.enabled, drive_task=drive_task, physics_grid=physics_grid,
            estimated_navigation=self.estimated_navigation,
            initial_attitude_prior=self.initial_attitude_prior, reference_provider=self.reference_provider,
            control_rate_hz=self.control_rate_hz)
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
