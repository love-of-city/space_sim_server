"""Physically-typed message ports for scene components.

Why this exists
---------------
Components used to reach into the MJScene (and, on the local backend, into the
MuJoCo stepper) directly: `scene.getBody(b).getOrigin().stateOutMsg`,
`scene.getSingleActuator(a).actuatorInMsg.subscribeTo(...)`, and so on. That made
every new component depend on both the physics backend and the exact MJScene API,
and it made two classes of mistake invisible until runtime: two owners writing the
same actuator command, and a component reading a state published on a different
schedule than it assumed.

`PhysicsPorts` is the single place that knows those APIs. Everything a component
needs is a named port:

* read ports have any number of readers and are cheap to hand out;
* write ports are owned: `drive_actuator` and `drive_servo` fail loudly if a
  second owner claims the same actuator, while `add_body_wrench` is additive and
  keyed by source name (drag, thrusters, ... all act on the same body).

Which port is fresh at which point in a step is documented in
docs/MUJOCO_CORE_ARCHITECTURE.md §3.4: joint/scene messages are written every
physics step, body/site messages only every `stride` steps.
"""
from __future__ import annotations

from typing import Any


class PortError(RuntimeError):
    """Raised when a component wires a port that is missing, or claims a taken one."""


class PhysicsPorts:
    """Named access to the physics state and actuator inputs of one session.

    ``scene`` is the authoritative MJScene (state container and publisher).
    ``stepper`` is the local MuJoCo core when that backend is active, else None.
    ``ephemeris`` and ``orbit_reference`` exist only where the component that owns
    them installed them; the accessors raise until then.
    """

    def __init__(self, scene: Any, *, stepper: Any = None) -> None:
        self.scene = scene
        self.stepper = stepper
        self._ephemeris: Any = None
        self._orbit_reference: Any = None
        self._orbit_origin: Any = None
        self._gravity_factory: Any = None
        self._orbital_environment: Any = None
        self._actuator_owners: dict[str, str] = {}
        self._servo_owners: dict[str, str] = {}
        self._wrench_sources: dict[str, set[str]] = {}
        self._body_names: tuple[str, ...] | None = None
        self._joint_owner: dict[str, str] = {}

    # --- provider registration (used by the components that own these) -----------
    def set_ephemeris(self, ephemeris: Any) -> None:
        if self._ephemeris is not None:
            raise PortError("an ephemeris is already registered")
        self._ephemeris = ephemeris

    def set_orbit_reference(self, spacecraft: Any) -> None:
        if self._orbit_reference is not None:
            raise PortError("an orbit reference is already registered")
        self._orbit_reference = spacecraft

    def set_orbit_origin(self, origin: Any) -> None:
        """Attach the propagated reference point O to the physics core.

        The core integrates every body relative to O and applies differential
        gravity about it, so without this it would fall back to its own static,
        gravity-free origin and the whole system would drift in free space.
        """
        if self._orbit_origin is not None and self._orbit_origin is not origin:
            raise PortError("an orbit origin is already registered")
        self._orbit_origin = origin
        if self.stepper is not None:
            self.stepper.origin = origin

    def set_stepper(self, stepper: Any) -> None:
        """Bind the physics core, applying any origin that was registered first."""
        if self.stepper is not None and self.stepper is not stepper:
            raise PortError("a physics core is already registered")
        self.stepper = stepper
        if self._orbit_origin is not None:
            stepper.origin = self._orbit_origin

    def set_gravity_factory(self, factory: Any) -> None:
        if self._gravity_factory is not None:
            raise PortError("a gravity factory is already registered")
        self._gravity_factory = factory

    def orbital_providers(self) -> tuple[Any, Any]:
        if self._orbit_reference is None or self._gravity_factory is None:
            raise PortError("install the local orbit component before orbital gravity")
        return self._orbit_reference, self._gravity_factory

    def set_orbital_environment(self, environment: Any) -> None:
        if self._orbital_environment is not None:
            raise PortError("an orbital environment is already registered")
        if self.stepper is None:
            raise PortError("install the local physics core before orbital gravity")
        self.stepper.set_orbital_environment(environment)
        self._orbital_environment = environment

    def orbital_environment(self) -> Any:
        """Extension components register native BSK sources before initialization."""
        if self._orbital_environment is None:
            raise PortError("the BSK orbital environment is not installed")
        return self._orbital_environment

    # --- read ports --------------------------------------------------------------
    @property
    def body_names(self) -> tuple[str, ...]:
        if self._body_names is None:
            self._body_names = tuple(str(name) for name in self.scene.getBodyNames())
        return self._body_names

    def has_body(self, body: str) -> bool:
        return body in self.body_names

    def body_state(self, body: str) -> Any:
        """Body frame origin in N (``SCStatesMsg``); published every `stride` steps."""
        return self._body(body).getOrigin().stateOutMsg

    def body_com_state(self, body: str) -> Any:
        """Body center of mass in N; the input gravity/atmosphere models expect."""
        return self._body(body).getCenterOfMass().stateOutMsg

    def site_state(self, site: str) -> Any:
        """Site frame in N; same publication schedule as body states."""
        return self.scene.getSite(site).stateOutMsg

    def joint_state(self, joint: str) -> Any:
        """Scalar joint position (rad or m), written every physics step."""
        return self._joint(joint).stateOutMsg

    def joint_rate(self, joint: str) -> Any:
        """Scalar joint rate, written every physics step."""
        return self._joint(joint).stateDotOutMsg

    def scene_state(self) -> Any:
        """Bulk ``MJSceneStateMsg`` (qpos/qvel), written every physics step."""
        return self.scene.stateOutMsg

    def servo_effort(self, actuator: str) -> tuple[Any, Any]:
        """(requested, applied) servo effort as ``SingleActuatorMsg``.

        Only available on the local backend, where the MuJoCo servos replace the
        PID/limiter chain and publish the same two messages it did.
        """
        if self.stepper is None:
            raise PortError("servo effort is published only by the local dynamics core")
        names = list(self.stepper.actuator_names)
        if actuator not in names:
            raise PortError(f"unknown actuator {actuator!r}")
        index = names.index(actuator)
        try:
            return self.stepper.requestedOutMsgs[index], self.stepper.appliedOutMsgs[index]
        except IndexError:
            raise PortError(f"actuator {actuator!r} is not served by the local dynamics core") from None

    def origin_state(self) -> Any:
        """Orbital reference point O propagated by Basilisk (local backend)."""
        if self._orbit_reference is None:
            raise PortError("no orbit reference is registered for this session")
        return self._orbit_reference.scStateOutMsg

    def planet_state(self, name: str) -> Any:
        """SPICE planet state message for gravity/renderer use."""
        if self._ephemeris is None:
            raise PortError("no ephemeris is registered for this session")
        index = {"earth": 0, "sun": 1}.get(name.casefold())
        if index is None:
            raise PortError(f"unsupported planet {name!r}; expected 'earth' or 'sun'")
        return self._ephemeris.planetStateOutMsgs[index]

    # --- write ports -------------------------------------------------------------
    def drive_actuator(self, actuator: str, command: Any, *, owner: str) -> None:
        """Command one non-servo MJCF actuator (motor, thruster, ...). Exclusive.

        Two owners on one actuator is a wiring bug: the scene would silently apply
        whichever module ran last. Claiming is per actuator, not per message.
        """
        existing = self._actuator_owners.get(actuator)
        if existing is not None and existing != owner:
            raise PortError(
                f"actuator {actuator!r} is already driven by {existing!r}; {owner!r} cannot own it too")
        if actuator in self._servo_owners:
            raise PortError(
                f"actuator {actuator!r} is served by {self._servo_owners[actuator]!r} and cannot "
                f"also be driven as a command actuator")
        if not hasattr(command, "addSubscriber"):
            raise PortError(
                f"command for {actuator!r} must be a Basilisk output message "
                "(the actuator input is what gets subscribed)")
        target = self.scene.getSingleActuator(actuator)
        if target is None:
            raise PortError(f"actuator {actuator!r} is not in this scene")
        if not hasattr(target.actuatorInMsg, "subscribeTo"):
            raise PortError(f"actuator {actuator!r} has no subscribable input message")
        target.actuatorInMsg.subscribeTo(command)
        self._actuator_owners[actuator] = owner

    def drive_servo(self, actuator: str, position: Any, velocity: Any, *, owner: str,
                    joint: str | None = None, kp: float | None = None, kd: float | None = None,
                    limit: float | None = None) -> Any:
        """Serve one MJCF actuator as an implicit MuJoCo position servo. Exclusive.

        On the local backend this connects a `ServoSpec` to the physics core; the
        effort is then published where the PID/limiter chain published it. Returns
        the created spec so the caller can keep a reference.
        """
        from simulation.local_mujoco_stepper import ServoSpec

        if self.stepper is None:
            raise PortError("servos are provided by the local dynamics core")
        if actuator in self._actuator_owners:
            raise PortError(
                f"actuator {actuator!r} is driven by {self._actuator_owners[actuator]!r} and cannot "
                f"also be served")
        existing = self._servo_owners.get(actuator)
        if existing is not None and existing != owner:
            raise PortError(f"servo {actuator!r} is already served by {existing!r}")
        joint = joint or actuator
        for value, name in ((kp, "kp"), (kd, "kd"), (limit, "limit")):
            if value is None:
                raise PortError(f"servo {actuator!r} needs a finite {name}")
        spec = ServoSpec(actuator, joint, float(kp), float(kd), float(limit), position, velocity)
        self.stepper.connect_servo(spec)
        self._servo_owners[actuator] = owner
        return spec

    def add_body_wrench(self, body: str, *, source: str, force_inertial: Any = None,
                        torque_body: Any = None) -> None:
        """Register one additive external wrench source on one body.

        Additive because several physical sources act on one body at once; keyed
        by `source` so a duplicated registration is caught instead of doubling a
        force. Local backend only: the basilisk path integrates with MJScene, whose
        effector interface is `AddModelToDynamicsTask`.
        """
        if self.stepper is None:
            raise PortError("external body wrenches are applied by the local dynamics core")
        owners = self._wrench_sources.setdefault(body, set())
        if source in owners:
            raise PortError(f"wrench source {source!r} is already registered on body {body!r}")
        self.stepper.add_body_wrench_input(body, force_inertial, torque_body, source=source)
        owners.add(source)

    def close(self) -> None:
        """Release the physics core's native handle (called on session teardown)."""
        if self.stepper is not None:
            self.stepper.close()

    # --- internals ---------------------------------------------------------------
    def _body(self, body: str) -> Any:
        if not self.has_body(body):
            raise PortError(f"body {body!r} is not in this scene")
        return self.scene.getBody(body)

    def _joint(self, joint: str) -> Any:
        owner = self._joint_owner.get(joint)
        if owner is None:
            for name in self.body_names:
                try:
                    self.scene.getBody(name).getScalarJoint(joint)
                except Exception:  # noqa: BLE001 - SWIG raises for a missing joint
                    continue
                self._joint_owner[joint] = owner = name
                break
        if owner is None:
            raise PortError(f"joint {joint!r} is not a scalar joint of any body")
        return self.scene.getBody(owner).getScalarJoint(joint)
