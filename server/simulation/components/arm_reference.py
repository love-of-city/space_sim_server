"""Scheduled joint references: what turns Cartesian teleop into joint targets.

The reference publisher is deliberately a separate component from the IK
controller: IK updates the cached reference at its own rate, and this publisher
pushes the held reference to the servos on the physics grid. On the local backend
the servos live inside the physics core, so this component also declares them.
"""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot


class JointReferenceComponent:
    """Publish held joint position/velocity references and declare the arm servos."""

    name = "joint_reference"

    def __init__(self, publisher: Any, *, joints: tuple[str, ...], actuators: tuple[str, ...],
                 kp: Any, kd: Any, limits: Any, ik_rate_hz: int,
                 wire_servos: bool = True) -> None:
        self.publisher = publisher
        self.joints = tuple(joints)
        self.actuators = tuple(actuators)
        self.kp, self.kd, self.limits = kp, kd, limits
        self.ik_rate_hz = int(ik_rate_hz)
        # The scenario builder creates the local physics core together with its
        # servos, because the core must exist before the controller chain. When it
        # has already wired these actuators, this component only schedules the
        # publisher; wiring them twice would be caught as a duplicate owner.
        self.wire_servos = bool(wire_servos)
        self.servo_specs: list[Any] = []

    def install(self, ctx: AssemblyContext) -> None:
        from space_arm_platform.sampling import ik_step_stride

        every = ik_step_stride(self.ik_rate_hz)
        if ctx.clock is not None:
            self.publisher.clock = ctx.clock
            self.publisher.stride = every
        # COMMAND segment, after IK: the servo references must be the ones IK just
        # computed for this step, not the previous step's.
        ctx.add(self.publisher, Slot.COMMAND, every=1)
        if ctx.config.dynamics_backend != "local" or not self.wire_servos:
            return
        for index, (actuator, joint) in enumerate(zip(self.actuators, self.joints, strict=True)):
            self.servo_specs.append(ctx.ports.drive_servo(
                actuator, self.publisher.positionOutMsgs[index], self.publisher.velocityOutMsgs[index],
                owner=self.name, joint=joint, kp=float(self.kp[index]), kd=float(self.kd[index]),
                limit=float(self.limits[index])))
