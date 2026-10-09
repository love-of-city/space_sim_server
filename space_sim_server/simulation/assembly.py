"""Assembly: turn a component list into one deterministic Basilisk task graph.

Why this exists
---------------
Session assembly used to live inline in the teleop entry point: roughly 250 lines
that created modules, wired messages, chose task priorities by hand, and kept a
`keep_alive` tuple so Python objects were not garbage collected. Adding a module
meant editing that block, and the only description of the resulting execution
order was the code itself.

This module makes the order data:

* `Slot` names a segment of the physics task; `AssemblyContext.add` allocates the
  actual Basilisk priority inside it, so components never invent magic numbers.
* `every`/`phase` express a rate as an integer division of the 240 Hz grid and
  pin its execution phase to the state-publication phase (see §3.4/§6.3 of
  docs/MUJOCO_CORE_ARCHITECTURE.md).
* `describe()` returns the final schedule, which is logged at startup and used as
  a golden test, so a reordering is visible in review instead of inferred.

`Component` is deliberately small: create modules, wire ports, add them. Nothing
else. Components hold no state across sessions because every reset rebuilds the
whole graph.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Iterable, Protocol, runtime_checkable

from Basilisk.architecture import sysModel

from simulation.physics_ports import PhysicsPorts, PortError


class Slot(IntEnum):
    """Highest graspTask priority of each segment (higher runs earlier).

    The numeric values are the documented segments of
    docs/MUJOCO_CORE_ARCHITECTURE.md §4.2, chosen so the existing live order is
    reproduced exactly.
    """

    CLOCK = 20_000
    ORBIT = 19_999        # ephemeris, orbital reference point O
    ENVIRONMENT = 16_999  # atmosphere, magnetic field, drag, ...
    COMMAND = 11_999      # IK, joint references, scripted trajectory
    CONTROL = 8_999       # guidance, navigation, control laws
    ACTUATOR = 2_999      # thruster/magnetorquer models, wheel drives
    PHYSICS = 1_000       # only the dynamics core
    SENSOR = 999          # star tracker, IMU, ...
    RECORD = 0            # recorders
    OUTPUT = -10_000      # render bridge, observation snapshot


# Segments that run before the dynamics core read the previous step's body states;
# segments after it read the current step's. Used to default the divider phase.
_BEFORE_CORE = (Slot.CLOCK, Slot.ORBIT, Slot.ENVIRONMENT, Slot.COMMAND, Slot.CONTROL, Slot.ACTUATOR)


@dataclass(frozen=True)
class ScheduledModule:
    """One module as placed in a task."""

    component: str
    model: Any
    slot: Slot
    priority: int
    every: int
    phase: int
    task: str = "graspTask"
    def to_dict(self) -> dict[str, Any]:
        return {
            "component": self.component,
            "task": self.task,
            "priority": self.priority,
            "module": getattr(self.model, "ModelTag", type(self.model).__name__),
            "every": self.every,
            "phase": self.phase,
            # every=0 means "scheduled by its own task period", not by this grid.
            "rate_hz": (None if self.every == 0 else 240.0 / self.every),
        }


class RateDivider(sysModel.SysModel):
    """Run one child module every N physics steps, at a fixed phase.

    The child is *not* registered in any task: it receives SelfInit/Reset/UpdateState
    only when this divider calls them. That keeps the child's own scheduling out of
    Basilisk's hands while preserving the lifecycle `InitializeSimulation` would
    otherwise provide.
    """

    def __init__(self, child: Any, *, every: int, phase: int, clock: Any) -> None:
        super().__init__()
        self.ModelTag = f"{getattr(child, 'ModelTag', type(child).__name__)}Divider{every}"
        self.child = child
        self.every = int(every)
        self.phase = int(phase)
        self.clock = clock
        self.calls = 0

    def _due(self) -> bool:
        index = getattr(self.clock, "step_index", None)
        if index is None:
            return True
        return index % self.every == self.phase % self.every

    def SelfInit(self) -> None:  # pragma: no cover - exercised only by Basilisk
        self.child.SelfInit()

    def Reset(self, current_sim_nanos: int) -> None:
        self.child.Reset(current_sim_nanos)

    def UpdateState(self, current_sim_nanos: int) -> None:
        if not self._due():
            return
        self.calls += 1
        self.child.UpdateState(current_sim_nanos)


@runtime_checkable
class Component(Protocol):
    """A named group of modules plus their wiring and schedule."""

    name: str

    def install(self, ctx: "AssemblyContext") -> None: ...


@dataclass
class SessionConfig:
    """Everything the graph shape depends on, resolved before assembly."""

    model_path: Any = None
    dynamics_backend: str = "local"
    local_substeps: int | None = None
    publish_stride: int = 2
    clock_rate_hz: int = 240
    ik_rate_hz: int = 120
    render_rate_hz: int = 30
    physics_task: str = "graspTask"
    options: dict[str, Any] = field(default_factory=dict)


class AssemblyContext:
    """Collect modules from components and attach them in a deterministic order."""

    def __init__(self, simulation: Any, scene: Any, *, key_alive: bool = True,
                 config: SessionConfig | None = None, physics_ports: PhysicsPorts | None = None) -> None:
        self.simulation = simulation
        self.scene = scene
        self.config = config or SessionConfig()
        self.ports = physics_ports if physics_ports is not None else PhysicsPorts(scene)
        self.clock: Any = None
        self._scheduled: list[ScheduledModule] = []
        self._next_priority: dict[tuple[str, Slot], int] = {}
        self._kept: list[Any] = []
        self._dividers: list[RateDivider] = []
        self.installed: list[str] = []
        self._current: str | None = None
        # Session services that are not physics state (IK target, control client,
        # resolved CLI options, ...). Components read them here; nothing in this
        # dict is a message or a physics quantity, so it is not a second state
        # authority: physical data moves only through `ports`.
        self.extra: dict[str, Any] = {}

    # --- component lifecycle -----------------------------------------------------
    def install(self, components: Iterable[Component]) -> "AssemblyContext":
        for component in components:
            name = str(getattr(component, "name", "")).strip()
            if not name:
                raise PortError(f"{type(component).__name__} has no component name")
            if name in self.installed:
                raise PortError(f"component {name!r} is already installed")
            self._current = name
            try:
                component.install(self)
            finally:
                self._current = None
            self.installed.append(name)
        return self

    # --- placement ---------------------------------------------------------------
    def add(self, model: Any, slot: Slot, *, every: int = 1, phase: int | None = None,
            component: str | None = None, task: str | None = None) -> Any:
        """Place one module in a segment, at a rate that divides the physics grid."""
        slot = Slot(slot)
        task = task or self.config.physics_task
        every = int(every)
        if every < 1:
            raise PortError("every must be a positive number of physics steps")
        stride = max(1, int(self.config.publish_stride))
        reads_kinematics = slot in _BEFORE_CORE or slot in (Slot.SENSOR, Slot.RECORD, Slot.OUTPUT)
        if reads_kinematics and every > 1 and every % stride:
            raise PortError(
                f"every={every} is not a multiple of the body/site publication stride {stride}; "
                "this module would always read one publication behind")
        if phase is None:
            # Before the core: run just after a publication so t(k-1) is fresh.
            phase = 1 % every if slot in _BEFORE_CORE else 0
        phase = int(phase) % every
        key = (task, slot)
        priority = self._next_priority.get(key, int(slot))
        self._next_priority[key] = priority - 1
        entry = ScheduledModule(self._current or component or "?", model, slot, priority, every, phase, task)
        self._scheduled.append(entry)
        if every == 1:
            self.simulation.AddModelToTask(task, model, priority)
        else:
            divider = RateDivider(model, every=every, phase=phase, clock=self.clock)
            self._dividers.append(divider)
            self.simulation.AddModelToTask(task, divider, priority)
        self.keep_alive(model)
        return model

    def keep_alive(self, *objects: Any) -> None:
        """Retain references so Python does not collect the native modules."""
        self._kept.extend(objects)

    def note_external(self, model: Any, *, component: str, task: str, priority: int,
                      every: int = 1, phase: int = 0) -> None:
        """Record a module that another layer already attached to a task.

        Some components (attitude control) register their own modules on their own
        task while they are built, because that construction needs MJCF hardware
        details. This records them in the schedule so `describe()` is a complete
        picture of what runs, and so a golden test can detect a reordering.
        """
        self._scheduled.append(ScheduledModule(
            component=component, model=model, slot=Slot.CLOCK, priority=int(priority),
            every=int(every), phase=int(phase), task=str(task)))

    # --- description -------------------------------------------------------------
    def describe(self) -> list[dict[str, Any]]:
        """The final schedule, in execution order (Basilisk runs high priority first)."""
        return [entry.to_dict() for entry in sorted(
            self._scheduled, key=lambda item: (item.task, -item.priority))]

    def summary(self) -> dict[str, Any]:
        """One-line JSON payload logged at startup and asserted by tests."""
        return {
            "type": "module_schedule",
            "dynamics_backend": self.config.dynamics_backend,
            "publish_stride": self.config.publish_stride,
            "components": list(self.installed),
            "modules": self.describe(),
            "dividers": [{"module": divider.ModelTag, "every": divider.every,
                          "phase": divider.phase} for divider in self._dividers],
        }
