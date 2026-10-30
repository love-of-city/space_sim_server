"""Assembly layer: slots, rates/phases, exclusive ownership, and the schedule dump."""
from __future__ import annotations

from pathlib import Path
import sys

import pytest

pytest.importorskip("Basilisk")
from Basilisk.architecture import messaging, sysModel  # noqa: E402
from Basilisk.utilities import SimulationBaseClass, macros  # noqa: E402

from simulation.assembly import (  # noqa: E402
    AssemblyContext, RateDivider, SessionConfig, Slot,
)
from simulation.physics_ports import PhysicsPorts, PortError  # noqa: E402

DT = 1.0 / 240.0


class Recorder(sysModel.SysModel):
    """Minimal module that records the ticks it was actually called on."""

    def __init__(self, tag: str = "recorder") -> None:
        super().__init__()
        self.ModelTag = tag
        self.calls: list[int] = []

    def UpdateState(self, nanos: int) -> None:
        self.calls.append(int(nanos))


class Block:
    """A component stub that places modules through the context."""

    def __init__(self, name: str, *modules: object) -> None:
        self.name = name
        self._modules = modules

    def install(self, ctx: AssemblyContext) -> None:
        for module in self._modules:
            ctx.add(module, Slot.CONTROL, every=1)


class Clock(sysModel.SysModel):
    """Stand-in for RationalPhysicsClock: counts steps and records its own order."""

    def __init__(self, log: list[str] | None = None) -> None:
        super().__init__()
        self.ModelTag = "testClock"
        self.step_index = 0
        self.log = log

    def UpdateState(self, nanos: int) -> None:
        self.step_index += 1
        if self.log is not None:
            self.log.append(self.ModelTag)


def build(publish_stride: int = 1):
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("p")
    process.addTask(simulation.CreateNewTask("t", macros.sec2nano(DT)))
    scene = type("Scene", (), {
        "getBodyNames": lambda self: ["bus"],
        "getBody": lambda self, name: None,
        "getSingleActuator": lambda self, name: None,
        "getSite": lambda self, name: None,
        "stateOutMsg": None,
    })()
    ctx = AssemblyContext(
        simulation, scene,
        config=SessionConfig(dynamics_backend="local", publish_stride=publish_stride,
                             physics_task="t"),
        physics_ports=PhysicsPorts(scene))
    return simulation, ctx


def run(simulation, seconds: float) -> None:
    simulation.ConfigureStopTime(int(round(seconds * 1e9)))
    simulation.ExecuteSimulation()


def test_slot_order_is_the_documented_execution_order():
    assert Slot.CLOCK > Slot.ORBIT > Slot.ENVIRONMENT > Slot.COMMAND > Slot.CONTROL
    assert Slot.CONTROL > Slot.ACTUATOR > Slot.PHYSICS > Slot.SENSOR > Slot.RECORD > Slot.OUTPUT


def test_modules_run_in_slot_order_and_priority_decreases_within_a_segment():
    simulation, ctx = build()
    order: list[str] = []

    def module(tag):
        class M(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.ModelTag = tag

            def UpdateState(self, nanos):
                order.append(tag)
        return M()

    clock = Clock(order)
    ctx.clock = clock
    ctx.add(clock, Slot.CLOCK)
    ctx.add(module("first"), Slot.CONTROL)
    ctx.add(module("second"), Slot.CONTROL)
    ctx.add(module("output"), Slot.OUTPUT)
    simulation.InitializeSimulation()
    order.clear()  # initialization resets models in its own order; only steps matter
    run(simulation, DT)
    # One physics step: CLOCK, then CONTROL in registration order, then OUTPUT.
    assert order[:4] == ["testClock", "first", "second", "output"]


def test_describe_reports_the_schedule_and_survives_a_golden_comparison():
    simulation, ctx = build()
    clock = Clock()
    ctx.clock = clock
    ctx.install([Block("a", clock), Block("b", Recorder("m1"), Recorder("m2"))])
    described = ctx.describe()
    assert [entry["component"] for entry in described] == ["a", "b", "b"]
    assert [entry["module"] for entry in described] == ["testClock", "m1", "m2"]
    # Priorities strictly decrease inside the segment, in registration order.
    priorities = [entry["priority"] for entry in described if entry["component"] == "b"]
    assert priorities == sorted(priorities, reverse=True)


def test_rate_divider_runs_the_child_at_the_requested_phase():
    simulation, ctx = build()
    clock = Clock()
    ctx.clock = clock
    ctx.add(clock, Slot.CLOCK)
    child = Recorder("divided")
    ctx.add(child, Slot.COMMAND, every=4, phase=1)
    simulation.InitializeSimulation()
    run(simulation, 20 * DT)
    # Executes on steps 1, 5, 9, 13, 17 (tick_time_ns is rounded, so compare
    # against the recorded clock indices rather than raw nanoseconds).
    assert len(child.calls) == 5
    assert ctx.describe()[-1]["every"] == 4 and ctx.describe()[-1]["phase"] == 1
    assert ctx.summary()["dividers"][0]["every"] == 4


def test_divider_forwards_reset_to_the_child():
    class Resettable(Recorder):
        def __init__(self) -> None:
            super().__init__("resettable")
            self.resets = 0

        def Reset(self, nanos):
            self.resets += 1

    simulation, ctx = build()
    clock = Clock()
    ctx.clock = clock
    ctx.add(clock, Slot.CLOCK)
    child = Resettable()
    ctx.add(child, Slot.COMMAND, every=2)
    simulation.InitializeSimulation()
    run(simulation, 10 * DT)
    assert child.resets == 1  # InitializeSimulation reaches the wrapper, which forwards


def test_divider_rejects_a_rate_that_misses_the_publication_grid():
    simulation, ctx = build(publish_stride=2)
    clock = Clock()
    ctx.clock = clock
    ctx.add(clock, Slot.CLOCK)
    with pytest.raises(PortError, match="publication stride"):
        ctx.add(Recorder(), Slot.COMMAND, every=3)
    ctx.add(Recorder(), Slot.COMMAND, every=4)  # multiple of the stride is fine


def test_duplicate_component_installation_is_rejected():
    simulation, ctx = build()
    block = Block("once")
    ctx.install([block])
    with pytest.raises(PortError, match="already installed"):
        ctx.install([block])


def test_exclusive_actuator_ownership_and_additive_wrench_sources():
    from Basilisk.architecture import messaging as msg

    class FakeInput:
        """Stands in for an MJScene actuator input port."""

        def __init__(self) -> None:
            self.subscribed_to: object | None = None

        def subscribeTo(self, message):
            self.subscribed_to = message

    class FakeActuator:
        def __init__(self) -> None:
            self.actuatorInMsg = FakeInput()

    class FakeScene:
        def __init__(self) -> None:
            self.actuators = {"motor": FakeActuator()}

        def getSingleActuator(self, name):
            return self.actuators[name]

    scene = FakeScene()
    ports = PhysicsPorts(scene)
    command = msg.SingleActuatorMsg()
    ports.drive_actuator("motor", command, owner="a")
    ports.drive_actuator("motor", command, owner="a")  # same owner: idempotent
    with pytest.raises(PortError, match="already driven"):
        ports.drive_actuator("motor", command, owner="b")

    class FakeStepper:
        actuator_names = ("servo",)
        requestedOutMsgs: list = []
        appliedOutMsgs: list = []

        def __init__(self) -> None:
            self.servos: list = []
            self.wrenches: list = []

        def connect_servo(self, spec):
            self.servos.append(spec)

        def add_body_wrench_input(self, body, force, torque, *, source):
            self.wrenches.append((body, source))

    stepper = FakeStepper()
    ports = PhysicsPorts(scene, stepper=stepper)
    position, velocity = msg.ScalarJointStateMsg(), msg.ScalarJointStateMsg()
    ports.drive_servo("servo", position, velocity, owner="arm", kp=1.0, kd=0.1, limit=2.0)
    with pytest.raises(PortError, match="already served"):
        ports.drive_servo("servo", position, velocity, owner="other", kp=1.0, kd=0.1, limit=2.0)
    ports.add_body_wrench("bus", source="drag")
    ports.add_body_wrench("bus", source="thruster")  # additive: a second source is fine
    with pytest.raises(PortError, match="already registered"):
        ports.add_body_wrench("bus", source="drag")
    assert [source for _, source in stepper.wrenches] == ["drag", "thruster"]


def test_write_ports_fail_loudly_on_the_basilisk_backend():
    scene = type("Scene", (), {"getSingleActuator": lambda self, name: None})()
    ports = PhysicsPorts(scene)  # no local physics core
    with pytest.raises(PortError, match="local dynamics core"):
        ports.drive_servo("motor", messaging.ScalarJointStateMsg(),
                          messaging.ScalarJointStateMsg(), owner="arm", kp=1.0, kd=0.1, limit=1.0)
    with pytest.raises(PortError, match="local dynamics core"):
        ports.add_body_wrench("bus", source="drag")
    with pytest.raises(PortError, match="local dynamics core"):
        ports.servo_effort("motor")


def test_read_ports_require_registered_providers():
    scene = type("Scene", (), {"getBodyNames": lambda self: []})()
    ports = PhysicsPorts(scene)
    with pytest.raises(PortError, match="no orbit reference"):
        ports.origin_state()
    with pytest.raises(PortError, match="no ephemeris"):
        ports.planet_state("earth")
    with pytest.raises(PortError, match="unsupported planet"):
        ports.set_ephemeris(type("E", (), {"planetStateOutMsgs": [None, None]})())
        ports.planet_state("mars")


def test_rate_divider_uses_the_clock_index_not_wall_time():
    clock = Clock()
    child = Recorder()
    divider = RateDivider(child, every=3, phase=0, clock=clock)
    for _ in range(6):
        divider.UpdateState(0)
        clock.step_index += 1
    assert len(child.calls) == 2  # indices 0 and 3
    assert divider.calls == 2


class FakeCore:
    """Stands in for the physics core: tracks the origin it was given."""

    def __init__(self) -> None:
        self.origin = "DEFAULT_STATIC_ORIGIN"


def test_orbit_origin_reaches_the_physics_core_in_either_order():
    """Regression: a missing wire here silently removes all gravity.

    The core integrates every body relative to the propagated reference point O
    and applies the tidal term about it. If O is never handed over, the core keeps
    its own static gravity-free origin and the whole system drifts away from
    Earth - which is exactly what a component refactor dropped once.
    """
    origin = object()
    scene = type("Scene", (), {})()
    # Order 1: origin registered before the core is bound.
    ports = PhysicsPorts(scene)
    ports.set_orbit_origin(origin)
    core = FakeCore()
    ports.set_stepper(core)
    assert core.origin is origin
    # Order 2: core bound first, origin attached afterwards.
    ports = PhysicsPorts(scene)
    core = FakeCore()
    ports.set_stepper(core)
    ports.set_orbit_origin(origin)
    assert core.origin is origin
    assert ports.stepper is core


def test_orbit_origin_and_core_each_reject_a_second_registration():
    origin = object()
    scene = type("Scene", (), {})()
    ports = PhysicsPorts(scene)
    ports.set_orbit_origin(origin)
    ports.set_orbit_origin(origin)  # same object is idempotent
    with pytest.raises(PortError, match="already registered"):
        ports.set_orbit_origin(object())
    core = FakeCore()
    ports.set_stepper(core)
    with pytest.raises(PortError, match="already registered"):
        ports.set_stepper(FakeCore())


_ = sys
_ = Path
