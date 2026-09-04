"""Composable simulation architecture for Basilisk and MJScene.

The orchestration layer deliberately keeps the physics backend and Basilisk
modules behind small, Python-level ports. A scene backend owns the authoritative
state integration; Basilisk modules consume a snapshot and return actuator
outputs. This makes adding a new Basilisk module a registration/configuration
change instead of a rewrite of the main simulation loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Any, Iterable, Mapping, Protocol, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
SCENE_STATE_PROTOCOL = "space-sim-state/1"


def _vector3(values: Sequence[float], name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result  # type: ignore[return-value]


def _quaternion(values: Sequence[float], name: str) -> Quaternion:
    if len(values) != 4:
        raise ValueError(f"{name} must contain exactly 4 values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} must have a non-zero norm")
    return tuple(value / norm for value in result)  # type: ignore[return-value]


@dataclass(frozen=True)
class RigidBodyState:
    """Kinematic state of one rigid body in the agreed simulation frame."""

    body_id: str
    position_m: Vector3
    velocity_m_s: Vector3
    attitude_wxyz: Quaternion = (1.0, 0.0, 0.0, 0.0)
    angular_velocity_rad_s: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not self.body_id.strip():
            raise ValueError("body_id must not be empty")
        object.__setattr__(self, "position_m", _vector3(self.position_m, "position_m"))
        object.__setattr__(self, "velocity_m_s", _vector3(self.velocity_m_s, "velocity_m_s"))
        object.__setattr__(self, "attitude_wxyz", _quaternion(self.attitude_wxyz, "attitude_wxyz"))
        object.__setattr__(
            self,
            "angular_velocity_rad_s",
            _vector3(self.angular_velocity_rad_s, "angular_velocity_rad_s"),
        )


@dataclass(frozen=True)
class RigidBodyProperties:
    """Mass properties required by Basilisk and the physics backend."""

    body_id: str
    mass_kg: float
    center_of_mass_m: Vector3 = (0.0, 0.0, 0.0)
    inertia_kg_m2: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        if not self.body_id.strip():
            raise ValueError("body_id must not be empty")
        if not math.isfinite(float(self.mass_kg)) or self.mass_kg <= 0.0:
            raise ValueError("mass_kg must be finite and positive")
        object.__setattr__(self, "center_of_mass_m", _vector3(self.center_of_mass_m, "center_of_mass_m"))
        if len(self.inertia_kg_m2) != 9:
            raise ValueError("inertia_kg_m2 must contain 9 values")
        inertia = tuple(float(value) for value in self.inertia_kg_m2)
        if not all(math.isfinite(value) for value in inertia):
            raise ValueError("inertia_kg_m2 must contain only finite values")
        object.__setattr__(self, "inertia_kg_m2", inertia)


@dataclass(frozen=True)
class JointState:
    """State of one scalar or revolute joint."""

    joint_id: str
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0

    def __post_init__(self) -> None:
        if not self.joint_id.strip():
            raise ValueError("joint_id must not be empty")
        for name in ("position_rad", "velocity_rad_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class ReactionWheelState:
    wheel_id: str
    speed_rad_s: float = 0.0
    torque_limit_nm: float | None = None

    def __post_init__(self) -> None:
        if not self.wheel_id.strip():
            raise ValueError("wheel_id must not be empty")
        if not math.isfinite(float(self.speed_rad_s)):
            raise ValueError("speed_rad_s must be finite")
        if self.torque_limit_nm is not None and (
            not math.isfinite(float(self.torque_limit_nm)) or self.torque_limit_nm <= 0.0
        ):
            raise ValueError("torque_limit_nm must be finite and positive when provided")


@dataclass(frozen=True)
class ThrusterState:
    thruster_id: str
    enabled: bool = False
    thrust_n: float = 0.0

    def __post_init__(self) -> None:
        if not self.thruster_id.strip():
            raise ValueError("thruster_id must not be empty")
        if not math.isfinite(float(self.thrust_n)) or self.thrust_n < 0.0:
            raise ValueError("thrust_n must be finite and non-negative")


@dataclass(frozen=True)
class EnvironmentState:
    """SPICE/ephemeris output consumed by Basilisk modules."""

    sim_time_s: float
    bodies: Mapping[str, RigidBodyState] = field(default_factory=dict)
    epoch_utc: str = ""
    reference_center: str = "Earth"
    reference_frame: str = "J2000"

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.sim_time_s)) or self.sim_time_s < 0.0:
            raise ValueError("sim_time_s must be finite and non-negative")
        object.__setattr__(self, "bodies", dict(self.bodies))


@dataclass(frozen=True)
class SceneState:
    """Complete state snapshot exchanged at one simulation boundary."""

    sim_time_s: float
    bodies: Mapping[str, RigidBodyState] = field(default_factory=dict)
    body_properties: Mapping[str, RigidBodyProperties] = field(default_factory=dict)
    joints: Mapping[str, JointState] = field(default_factory=dict)
    reaction_wheels: Mapping[str, ReactionWheelState] = field(default_factory=dict)
    thrusters: Mapping[str, ThrusterState] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.sim_time_s)) or self.sim_time_s < 0.0:
            raise ValueError("sim_time_s must be finite and non-negative")
        object.__setattr__(self, "bodies", dict(self.bodies))
        object.__setattr__(self, "body_properties", dict(self.body_properties))
        object.__setattr__(self, "joints", dict(self.joints))
        object.__setattr__(self, "reaction_wheels", dict(self.reaction_wheels))
        object.__setattr__(self, "thrusters", dict(self.thrusters))
        object.__setattr__(self, "metadata", dict(self.metadata))

    def at_time(self, sim_time_s: float) -> SceneState:
        """Return a snapshot with a new simulation timestamp."""

        return replace(self, sim_time_s=float(sim_time_s))

    def to_message(self) -> dict[str, Any]:
        """Serialize the state boundary shared by modules and adapters."""

        return {
            "protocol": SCENE_STATE_PROTOCOL,
            "type": "state",
            "sim_time_s": float(self.sim_time_s),
            "bodies": [
                {
                    "body_id": body.body_id,
                    "position_m": list(body.position_m),
                    "velocity_m_s": list(body.velocity_m_s),
                    "attitude_wxyz": list(body.attitude_wxyz),
                    "angular_velocity_rad_s": list(body.angular_velocity_rad_s),
                }
                for body in sorted(self.bodies.values(), key=lambda item: item.body_id)
            ],
            "body_properties": [
                {
                    "body_id": properties.body_id,
                    "mass_kg": float(properties.mass_kg),
                    "center_of_mass_m": list(properties.center_of_mass_m),
                    "inertia_kg_m2": list(properties.inertia_kg_m2),
                }
                for properties in sorted(
                    self.body_properties.values(), key=lambda item: item.body_id
                )
            ],
            "joints": [
                {
                    "joint_id": joint.joint_id,
                    "position_rad": float(joint.position_rad),
                    "velocity_rad_s": float(joint.velocity_rad_s),
                }
                for joint in sorted(self.joints.values(), key=lambda item: item.joint_id)
            ],
            "reaction_wheels": [
                {
                    "wheel_id": wheel.wheel_id,
                    "speed_rad_s": float(wheel.speed_rad_s),
                    "torque_limit_nm": wheel.torque_limit_nm,
                }
                for wheel in sorted(
                    self.reaction_wheels.values(), key=lambda item: item.wheel_id
                )
            ],
            "thrusters": [
                {
                    "thruster_id": thruster.thruster_id,
                    "enabled": bool(thruster.enabled),
                    "thrust_n": float(thruster.thrust_n),
                }
                for thruster in sorted(self.thrusters.values(), key=lambda item: item.thruster_id)
            ],
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class WrenchCommand:
    force_n: Vector3 = (0.0, 0.0, 0.0)
    torque_nm: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "force_n", _vector3(self.force_n, "force_n"))
        object.__setattr__(self, "torque_nm", _vector3(self.torque_nm, "torque_nm"))

    def plus(self, other: WrenchCommand) -> WrenchCommand:
        return WrenchCommand(
            tuple(a + b for a, b in zip(self.force_n, other.force_n, strict=True)),
            tuple(a + b for a, b in zip(self.torque_nm, other.torque_nm, strict=True)),
        )


@dataclass(frozen=True)
class JointActuatorCommand:
    """Optional position, velocity, and torque commands for one joint."""

    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None

    def __post_init__(self) -> None:
        for name in ("position_rad", "velocity_rad_s", "torque_nm"):
            value = getattr(self, name)
            if value is not None and not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite when provided")


@dataclass
class ControlOutput:
    """Outputs produced by one or more Basilisk modules."""

    external_wrenches: dict[str, WrenchCommand] = field(default_factory=dict)
    joint_commands: dict[str, JointActuatorCommand] = field(default_factory=dict)
    reaction_wheel_torques_nm: dict[str, float] = field(default_factory=dict)
    thruster_thrust_n: dict[str, float] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def combine(cls, outputs: Iterable[ControlOutput]) -> ControlOutput:
        """Combine module outputs deterministically.

        Wrenches and actuator efforts are additive because multiple Basilisk
        modules may contribute environmental or control terms. Joint command
        targets are exclusive; registering two competing position/velocity
        owners is almost certainly a configuration error and fails loudly.
        """

        combined = cls()
        for output in outputs:
            for body_id, wrench in output.external_wrenches.items():
                combined.external_wrenches[body_id] = combined.external_wrenches.get(
                    body_id, WrenchCommand()
                ).plus(wrench)
            for joint_id, command in output.joint_commands.items():
                previous = combined.joint_commands.get(joint_id)
                if previous is not None and previous != command:
                    raise ValueError(f"multiple Basilisk modules command joint {joint_id!r}")
                combined.joint_commands[joint_id] = command
            for wheel_id, torque in output.reaction_wheel_torques_nm.items():
                combined.reaction_wheel_torques_nm[wheel_id] = (
                    combined.reaction_wheel_torques_nm.get(wheel_id, 0.0) + float(torque)
                )
            for thruster_id, thrust in output.thruster_thrust_n.items():
                combined.thruster_thrust_n[thruster_id] = (
                    combined.thruster_thrust_n.get(thruster_id, 0.0) + float(thrust)
                )
            combined.metadata.update(output.metadata)
        return combined


@dataclass(frozen=True)
class SimulationStep:
    """Result of one complete environment/control/physics cycle."""

    previous_state: SceneState
    environment: EnvironmentState
    control: ControlOutput
    state: SceneState


class SceneBackend(Protocol):
    """Port implemented by MJScene or a test double."""

    def initialize(self, state: SceneState) -> None: ...

    def apply_control(self, control: ControlOutput) -> None: ...

    def step(self, dt_s: float) -> None: ...

    def read_state(self) -> SceneState: ...


class StatePublisher(Protocol):
    """Port for Basilisk messages, render bridges, recorders, or telemetry."""

    def publish(self, state: SceneState) -> None: ...


class EphemerisProvider(Protocol):
    """Port implemented by a SPICE-backed environment updater."""

    def update(self, sim_time_s: float) -> EnvironmentState: ...


class BasiliskModule(Protocol):
    """Minimal lifecycle contract for an extensible state-driven module."""

    name: str

    def initialize(self, state: SceneState) -> None: ...

    def update(
        self, state: SceneState, environment: EnvironmentState, dt_s: float
    ) -> ControlOutput: ...


@dataclass(frozen=True)
class BasiliskTaskBinding:
    """One native Basilisk SysModel attached to a simulation task."""

    name: str
    model: Any
    task_name: str
    priority: int = 0


class BasiliskModuleRegistry:
    """Register and attach native Basilisk modules in a deterministic order.

    Existing scenario builders can continue to create their native dynamics
    graph. New environment, navigation, guidance, control, actuator, or
    telemetry modules should be registered here rather than scattered across
    the scenario entry point.
    """

    def __init__(self) -> None:
        self._bindings: list[BasiliskTaskBinding] = []

    @property
    def bindings(self) -> tuple[BasiliskTaskBinding, ...]:
        return tuple(self._bindings)

    def register(self, name: str, model: Any, *, task_name: str, priority: int = 0) -> Any:
        name = str(name).strip()
        task_name = str(task_name).strip()
        if not name or not task_name:
            raise ValueError("Basilisk module name and task_name must not be empty")
        if any(binding.name == name for binding in self._bindings):
            raise ValueError(f"Basilisk module {name!r} is already registered")
        self._bindings.append(BasiliskTaskBinding(name, model, task_name, int(priority)))
        return model

    def attach(self, simulation: Any) -> None:
        """Attach all registered models to their Basilisk tasks."""

        for binding in self._bindings:
            simulation.AddModelToTask(binding.task_name, binding.model, binding.priority)

    def keep_alive(self) -> tuple[Any, ...]:
        """Return registered models so scenario owners can retain references."""

        return tuple(binding.model for binding in self._bindings)


class StaticEphemeris:
    """Simple provider useful for tests and scenes without celestial dynamics."""

    def __init__(self, *, epoch_utc: str = "", bodies: Mapping[str, RigidBodyState] | None = None) -> None:
        self.epoch_utc = epoch_utc
        self.bodies = dict(bodies or {})

    def update(self, sim_time_s: float) -> EnvironmentState:
        return EnvironmentState(sim_time_s=sim_time_s, bodies=self.bodies, epoch_utc=self.epoch_utc)


class SimulationOrchestrator:
    """Run the canonical state-message → Basilisk → MJScene loop."""

    def __init__(
        self,
        scene: SceneBackend,
        *,
        ephemeris: EphemerisProvider | None = None,
        publishers: Iterable[StatePublisher] = (),
        modules: Iterable[BasiliskModule] = (),
    ) -> None:
        self.scene = scene
        self.ephemeris = ephemeris or StaticEphemeris()
        self.publishers = list(publishers)
        self.modules: list[BasiliskModule] = []
        self._initialized = False
        for module in modules:
            self.add_module(module)

    def add_module(self, module: BasiliskModule) -> None:
        name = str(module.name).strip()
        if not name:
            raise ValueError("Basilisk module name must not be empty")
        if any(existing.name == name for existing in self.modules):
            raise ValueError(f"Basilisk module {name!r} is already registered")
        self.modules.append(module)
        if self._initialized:
            module.initialize(self.scene.read_state())

    def initialize(self, state: SceneState) -> None:
        self.scene.initialize(state)
        for module in self.modules:
            module.initialize(state)
        self._publish(state)
        self._initialized = True

    def step(self, dt_s: float) -> SimulationStep:
        if not self._initialized:
            raise RuntimeError("SimulationOrchestrator.initialize must be called first")
        dt_s = float(dt_s)
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")

        previous_state = self.scene.read_state()
        self._publish(previous_state)
        environment = self.ephemeris.update(previous_state.sim_time_s)
        outputs = [module.update(previous_state, environment, dt_s) for module in self.modules]
        control = ControlOutput.combine(outputs)
        self.scene.apply_control(control)
        self.scene.step(dt_s)
        state = self.scene.read_state()
        self._publish(state)
        return SimulationStep(previous_state, environment, control, state)

    def _publish(self, state: SceneState) -> None:
        for publisher in self.publishers:
            publisher.publish(state)
