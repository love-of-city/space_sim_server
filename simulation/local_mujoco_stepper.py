"""Local-frame fixed-step MuJoCo dynamics backend ("local") for SARM scenes.

Why: under Basilisk's adaptive RKF45 every gripper/plug contact forces the
integrator into repeated step rejection (about 148 MuJoCo evaluations per 240 Hz
step instead of 11). Contact and the stiff joint servos are integrated here by
MuJoCo's own fixed-step ``implicitfast`` instead, which is stable at 240 Hz.
Second reason: MuJoCo collision detection is only reliable near the origin.
With every body at orbital radius (~6.9e6 m, as in MJScene's inertial frame)
the same gripper/plug contact flickers on and off; it is exact up to ~3e5 m.

Division of labour (one integrator per state):

* Basilisk propagates only the orbital reference point O (a point-mass
  ``Spacecraft`` under the Earth/Sun field and SPICE ephemeris).
* This module integrates every MJCF body in the local frame L (origin O, axes
  parallel to N, zero gravity) with an independent mjModel compiled from the
  same MJCF. Native BSK gravity components supply differential body forces;
  the old analytic Earth tide remains an explicit regression mode.
* The MJScene stays in the task as the authoritative *state container and
  message publisher*: after each step the inertial state ``O + x_L`` is written
  to its bulk qpos/qvel states and published through its normal body, site and
  joint messages, so every existing consumer (render bridge, IK, attitude
  control, observations) is unchanged. Its own integrator never runs.

Actuator interface (generic, by MJCF actuator name):

* Servo actuators receive a joint position/velocity reference message pair and
  are expressed as MuJoCo position servos, so the damping term is implicit:
  ``force = clip(kp*(q_ref - q) + kd*(qd_ref - qd), +-limit)``. The requested
  and applied efforts are published as ``SingleActuatorMsg``.
* Every other actuator reads its MJScene ``actuatorInMsg`` each step (e.g. the
  reaction-wheel drives), held for the step (zero-order hold).
* Environment/effector forces (thrusters, magnetorquers, drag, higher-order
  gravity, ...) connect through ``add_body_wrench_input``: a standard Basilisk
  ``CmdForceInertialMsg`` (inertial axes, at the body COM) and/or
  ``CmdTorqueBodyMsg`` (body axes) per MJCF body, also held for the step.
  Non-gravitational net forces move the COM away from O; the periodic rebase
  keeps local coordinates small without changing any inertial state.
"""
from __future__ import annotations

import ctypes
from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any, Callable

import numpy as np
from Basilisk.architecture import messaging, sysModel

from simulation.native_acceleration import load_native_library
from simulation.orbital_frames import OrbitalForceProvider, interpolate_origin

BACKEND_BASILISK = "basilisk"
BACKEND_LOCAL = "local"
DYNAMICS_BACKENDS = (BACKEND_BASILISK, BACKEND_LOCAL)
# Default backend. "local" integrates with fixed-step MuJoCo implicitfast, which
# is the only one that reaches real time in contact (RKF45 rejects steps there);
# "basilisk" (MJScene + adaptive RKF45) stays available as the accuracy
# reference and for contact-free comparisons.
DEFAULT_BACKEND = BACKEND_LOCAL
# One 240 Hz implicitfast step: faster than real time in every measured contact
# case. More substeps trade speed for accuracy (errors are first order: the wrist
# roll joint J6, the stiffest, is ~2.5e-3 rad max off the RKF45 reference at 1,
# ~1.4e-3 at 2 and ~7e-4 at 4; see docs/LOCAL_DYNAMICS_BACKEND.md).
DEFAULT_SUBSTEPS = 1
MAX_SUBSTEPS = 16
# Publication after every physics step k:
# * always: MJScene bulk qpos/qvel, joint state/rate messages, the scene state
#   message and servo efforts. Joint messages read the bulk states directly
#   (MJScalarJoint::writeJointStateMessage), so this is cheap (~0.05 ms) and IK,
#   reference protection and wheel drives see exactly what MJScene would publish.
# * when k % stride == 0, and on every render/observation tick
#   (k % SYNC_STEPS == 0, the 30 Hz grid on 240 Hz dynamics): body/site messages
#   (MJScene.postIntegration = forward kinematics of its publish-only copy,
#   ~0.1 ms in isolation, ~0.6 ms in the live process). Their consumers are the
#   30 Hz render bridge and observation (always exact) and the 100 Hz attitude
#   navigation, which may read a site state up to stride-1 steps old; the
#   message's timeWritten() says so.
# Physics (MuJoCo) is stepped on every 240 Hz tick regardless of the stride.
DEFAULT_PUBLISH_STRIDE = 2
ALLOWED_PUBLISH_STRIDES = (1, 2, 4, 8)
SYNC_STEPS = 8  # space_arm_platform.sampling: DYNAMICS_HZ // RENDER_HZ
# Move O onto the system COM before local coordinates grow large enough for
# the linearized tidal term or float resolution to matter.
DEFAULT_REBASE_DISTANCE_M = 50.0
# Measured: MuJoCo collision detection is exact up to ~3e5 m from the origin,
# degrades near 1e6 m and fails at orbital radius (run/diagnostics/local_stepper/
# orbital_offset_contact.py), so a once-per-second check has a wide margin.
REBASE_CHECK_STEPS = 240
_OBJ_BODY = 1
_OBJ_ACTUATOR = 19
_OBJ_JOINT = 3
_JOINT_FREE = 0
_DOUBLE_P = ctypes.POINTER(ctypes.c_double)
_INT64_P = ctypes.POINTER(ctypes.c_int64)


def dynamics_backend(value: str | None = None) -> str:
    """Resolve the backend from an explicit value or SPACE_SIM_DYNAMICS_BACKEND."""
    selected = (value or os.environ.get("SPACE_SIM_DYNAMICS_BACKEND") or DEFAULT_BACKEND).strip().lower()
    if selected not in DYNAMICS_BACKENDS:
        raise ValueError(f"dynamics backend must be one of {DYNAMICS_BACKENDS}, got {selected!r}")
    return selected


def local_substeps(value: int | None = None) -> int:
    """Fixed MuJoCo substeps per physics step (SPACE_SIM_LOCAL_SUBSTEPS)."""
    raw = value if value is not None else os.environ.get("SPACE_SIM_LOCAL_SUBSTEPS", DEFAULT_SUBSTEPS)
    count = int(raw)
    if not 1 <= count <= MAX_SUBSTEPS:
        raise ValueError(f"local substeps must be in [1, {MAX_SUBSTEPS}]")
    return count


def publish_stride(value: int | None = None) -> int:
    """Body/site kinematics messages every Nth physics step (SPACE_SIM_LOCAL_PUBLISH_STRIDE)."""
    raw = value if value is not None else os.environ.get("SPACE_SIM_LOCAL_PUBLISH_STRIDE", DEFAULT_PUBLISH_STRIDE)
    count = int(raw)
    if count not in ALLOWED_PUBLISH_STRIDES:
        raise ValueError(f"publish stride must be one of {ALLOWED_PUBLISH_STRIDES}")
    return count


def _library():
    lib = load_native_library("local_mujoco_stepper")
    if getattr(lib, "_lms_bound", False):
        return lib
    lib.lms_abi.restype = ctypes.c_int
    lib.lms_header_version.restype = ctypes.c_int
    lib.lms_create.restype = ctypes.c_void_p
    lib.lms_create.argtypes = [ctypes.c_wchar_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    lib.lms_destroy.argtypes = [ctypes.c_void_p]
    lib.lms_dims.argtypes = [ctypes.c_void_p, _INT64_P]
    lib.lms_name2id.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
    lib.lms_id2name.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
    lib.lms_joint.argtypes = [ctypes.c_void_p, ctypes.c_int, _INT64_P]
    lib.lms_free_joints.argtypes = [ctypes.c_void_p, _INT64_P, ctypes.c_int]
    lib.lms_set_servo.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.lms_set_state.argtypes = [ctypes.c_void_p, _DOUBLE_P, _DOUBLE_P]
    lib.lms_get_state.argtypes = [ctypes.c_void_p, _DOUBLE_P, _DOUBLE_P]
    lib.lms_step.argtypes = [ctypes.c_void_p, _DOUBLE_P, ctypes.c_double, ctypes.c_int, _DOUBLE_P, ctypes.c_double,
                             _DOUBLE_P, _DOUBLE_P, _DOUBLE_P, _DOUBLE_P, _DOUBLE_P]
    lib.lms_body_coms.argtypes = [ctypes.c_void_p, _DOUBLE_P, _DOUBLE_P]
    lib.lms_set_orbital_forces.argtypes = [ctypes.c_void_p, _DOUBLE_P]
    lib.lms_momentum.argtypes = [ctypes.c_void_p, _DOUBLE_P]
    lib.lms_set_body_wrench.argtypes = [ctypes.c_void_p, ctypes.c_int, _DOUBLE_P, _DOUBLE_P]
    if lib.lms_abi() != 1 or lib.lms_header_version() != 3007000:
        raise RuntimeError("local MuJoCo stepper ABI/header mismatch; rebuild it")
    lib._lms_bound = True
    return lib


def _basilisk_mujoco_runtime() -> Path:
    """The mujoco.dll Basilisk loaded; the stepper must share that runtime."""
    import Basilisk
    runtime = Path(Basilisk.__file__).resolve().parent / "mujoco.dll"
    if not runtime.is_file():
        raise FileNotFoundError(f"Basilisk MuJoCo runtime not found: {runtime}")
    return runtime


def _pointer(array: np.ndarray):
    return array.ctypes.data_as(_DOUBLE_P)


@dataclass(frozen=True)
class ServoSpec:
    """One PD joint servo evaluated implicitly inside MuJoCo."""
    actuator: str
    joint: str
    kp: float
    kd: float
    limit: float
    position_reference: Any  # ScalarJointStateMsg
    velocity_reference: Any  # ScalarJointStateMsg


class _WrenchChannel:
    """One named source of external wrench for a single MJCF body.

    Several sources may target the same body (drag, thrusters, magnetorquers);
    the stepper sums them before the step, so no source has to know the others.
    """

    def __init__(self, body_id: int, source: str, force_reader: Any, torque_reader: Any) -> None:
        self.body_id = body_id
        self.source = source
        self.force_reader = force_reader
        self.torque_reader = torque_reader
        self.force = np.zeros(3)
        self.torque = np.zeros(3)


class OrbitOrigin:
    """Inertial position/velocity of the local-frame origin O.

    ``reader(now_s)`` returns (r_N [m], v_N [m/s]) at the current tick;
    ``shift(dr, dv)`` moves O without changing any inertial body state (used
    for rebasing). The default is a gravity-free point in uniform motion,
    initially at rest at the inertial origin (the native scripted scenario).
    """
    def __init__(self, reader: Callable[[float], tuple[np.ndarray, np.ndarray]] | None = None,
                 shift: Callable[[np.ndarray, np.ndarray], None] | None = None, mu: float = 0.0) -> None:
        self._reference_r = np.zeros(3)
        self._reference_v = np.zeros(3)
        self._reference_s = 0.0
        self.reader = reader
        self._shift = shift
        self.mu = float(mu)

    def state(self, now_s: float) -> tuple[np.ndarray, np.ndarray]:
        if self.reader is None:
            return self._reference_r + self._reference_v * (now_s - self._reference_s), self._reference_v.copy()
        r, v = self.reader(now_s)
        return np.asarray(r, dtype=float), np.asarray(v, dtype=float)

    def shift(self, dr: np.ndarray, dv: np.ndarray, now_s: float) -> None:
        if self._shift is None:
            r, v = self.state(now_s)
            self._reference_r, self._reference_v, self._reference_s = r + dr, v + dv, now_s
        else:
            self._shift(np.asarray(dr, dtype=float), np.asarray(dv, dtype=float))


class SpacecraftOrbitOrigin(OrbitOrigin):
    """O propagated by a Basilisk point-mass Spacecraft (gravity via factory)."""
    def __init__(self, spacecraft: Any, mu: float) -> None:
        self.spacecraft = spacecraft
        super().__init__(self._read, self._shift_states, mu)

    def _states(self):
        manager = self.spacecraft.dynManager
        return (manager.getStateObject(self.spacecraft.hub.nameOfHubPosition),
                manager.getStateObject(self.spacecraft.hub.nameOfHubVelocity))

    def _read(self, _now_s: float | None = None):
        # The hub states are the integrator's own values at this tick (the
        # Spacecraft already advanced to it); its output message may lag by
        # one call during initialization, so it is not used here.
        position, velocity = self._states()
        return (np.asarray(position.getState(), dtype=float).reshape(3),
                np.asarray(velocity.getState(), dtype=float).reshape(3))

    def _shift_states(self, dr, dv):
        position, velocity = self._states()
        r, v = self._read()
        position.setState((r + dr).reshape(3, 1))
        velocity.setState((v + dv).reshape(3, 1))

    def set_state(self, r: np.ndarray, v: np.ndarray) -> None:
        position, velocity = self._states()
        position.setState(np.asarray(r, dtype=float).reshape(3, 1))
        velocity.setState(np.asarray(v, dtype=float).reshape(3, 1))


class LocalMujocoStepper(sysModel.SysModel):
    """Advance all MJCF bodies with fixed-step implicitfast MuJoCo in frame L."""

    def __init__(self, scene: Any, model_path: Path, servos: list[ServoSpec], *,
                 origin: OrbitOrigin | None = None, substeps: int | None = None,
                 publish_every: int | None = None,
                 rebase_distance_m: float = DEFAULT_REBASE_DISTANCE_M) -> None:
        super().__init__()
        self.ModelTag = "localMujocoStepper"
        self.scene = scene
        self.origin = origin or OrbitOrigin()
        self.orbital_environment: OrbitalForceProvider | None = None
        self._previous_origin = None
        self.substeps = local_substeps(substeps)
        self.publish_every = publish_stride(publish_every)
        self.rebase_distance_m = float(rebase_distance_m)
        self._lib = _library()
        error = ctypes.create_string_buffer(1024)
        handle = self._lib.lms_create(str(_basilisk_mujoco_runtime()), str(Path(model_path).resolve()).encode(),
                                      error, len(error))
        if not handle:
            raise RuntimeError(f"local MuJoCo stepper failed to load {model_path}: {error.value.decode(errors='replace')}")
        self._handle = ctypes.c_void_p(handle)
        dims = np.zeros(6, dtype=np.int64)
        self._check(self._lib.lms_dims(self._handle, dims.ctypes.data_as(_INT64_P)), "dims")
        self.nq, self.nv, self.nu, self.nbody, self.njnt, nfree = (int(v) for v in dims)
        free = np.zeros(2 * max(nfree, 1), dtype=np.int64)
        count = self._lib.lms_free_joints(self._handle, free.ctypes.data_as(_INT64_P), max(nfree, 1))
        self._check(min(count, 0), "free joints")
        self.free_qpos = free[0:2 * count:2].astype(int)
        self.free_dof = free[1:2 * count:2].astype(int)
        # MJScene registers its bulk states in its own Reset, which runs after
        # this higher-priority model's Reset; bind them on the first update.
        self._qpos_state = self._qvel_state = None

        names = [self._name(_OBJ_ACTUATOR, i) for i in range(self.nu)]
        self.actuator_names = tuple(names)
        self._scene_actuators = {name: scene.getSingleActuator(name) for name in names}
        self._servo_specs: list[ServoSpec] = []
        # Servo effort is published where the Basilisk PID/limiter published it.
        self.requestedOutMsgs: list[Any] = []
        self.appliedOutMsgs: list[Any] = []
        self._requested_writers: list[Any] = []
        self._applied_writers: list[Any] = []
        self._payload = messaging.SingleActuatorMsgPayload()
        self._wrench_channels: list[_WrenchChannel] = []
        # Actuator partition, refreshed whenever the servo set grows.
        self.servo_index = np.zeros(0, dtype=int)
        self.servo_kp = np.zeros(0)
        self.servo_kd = np.zeros(0)
        self.servo_limit = np.zeros(0)
        self.servo_qpos = np.zeros(0, dtype=int)
        self.servo_dof = np.zeros(0, dtype=int)
        self.command_index = np.zeros(0, dtype=int)
        self._position_refs: list[Any] = []
        self._velocity_refs: list[Any] = []
        self._command_readers: list[Any] = []

        self.ctrl = np.zeros(self.nu)
        self.qpos = np.zeros(self.nq)
        self.qvel = np.zeros(self.nv)
        self.qacc = np.zeros(self.nv)
        self.actuator_force = np.zeros(self.nu)
        self.stats = np.zeros(6)
        self._origin_buffer = np.zeros(3)
        self._body_coms = np.zeros((self.nbody, 3))
        self._body_masses = np.zeros(self.nbody)
        self._pointers = tuple(_pointer(a) for a in (self.ctrl, self._origin_buffer, self.qpos, self.qvel,
                                                     self.qacc, self.actuator_force, self.stats))
        self._last_nanos: int | None = None
        self._published_q: np.ndarray | None = None
        self._published_v: np.ndarray | None = None
        self.step_count = 0
        self.kinematics_publish_count = 0
        self.external_writes = 0
        self.rebase_count = 0
        self.max_contacts = 0
        self.max_constraint_rows = 0
        self.max_solver_iterations = 0
        self.servos_finalized = False
        # MJScene publishes but never integrates while this backend owns time.
        scene.isDynamicsSynced = True
        for spec in servos:
            self.connect_servo(spec)
        self._refresh_actuators()

    def set_orbital_environment(self, environment: OrbitalForceProvider) -> None:
        if self.orbital_environment is not None or self._last_nanos is not None:
            raise ValueError("orbital environment must be installed exactly once before stepping")
        self.orbital_environment = environment

    def _step_with_orbital_environment(self, now, dt, r_o, v_o) -> int:
        environment = self.orbital_environment
        environment.begin_interval()
        r0, v0 = self._previous_origin
        maxima = np.zeros(3)
        status = 0
        for substep in range(self.substeps):
            nanos = self._last_nanos + ((now - self._last_nanos) * substep // self.substeps)
            end_nanos = self._last_nanos + ((now - self._last_nanos) * (substep + 1) // self.substeps)
            origin = interpolate_origin(r0, v0, r_o, v_o, dt,
                                        (nanos - self._last_nanos) / (now - self._last_nanos))
            self._check(self._lib.lms_body_coms(self._handle, _pointer(self._body_coms),
                                               _pointer(self._body_masses)), "fresh COM kinematics")
            forces = np.ascontiguousarray(
                environment.forces(self._body_coms, self._body_masses, origin, nanos), dtype=np.float64)
            if forces.shape != (self.nbody, 3) or not np.isfinite(forces).all():
                raise ValueError("orbital provider must return finite (nbody, 3) COM forces")
            self._check(self._lib.lms_set_orbital_forces(self._handle, _pointer(forces)), "BSK orbital force")
            # mu=0 disables the old analytic tide: never apply gravity twice.
            status = self._lib.lms_step(self._handle, self._pointers[0], (end_nanos - nanos) * 1e-9, 1,
                                        self._pointers[1], 0.0, *self._pointers[2:])
            maxima = np.maximum(maxima, self.stats[:3])
            if status < 0:
                self.stats[5] = substep + 1
                return status
        self.stats[:3] = maxima
        self.stats[5] = self.substeps
        return status

    # --- actuators ------------------------------------------------------------------
    def _refresh_actuators(self) -> None:
        """Recompute servo/command index arrays from the current servo set."""
        names = list(self.actuator_names)
        if self._servo_specs:
            self.servo_index = np.array(
                [names.index(spec.actuator) for spec in self._servo_specs], dtype=int)
            self.servo_kp = np.array([spec.kp for spec in self._servo_specs], dtype=float)
            self.servo_kd = np.array([spec.kd for spec in self._servo_specs], dtype=float)
            self.servo_limit = np.array([spec.limit for spec in self._servo_specs], dtype=float)
        else:
            self.servo_index = np.zeros(0, dtype=int)
            self.servo_kp = np.zeros(0)
            self.servo_kd = np.zeros(0)
            self.servo_limit = np.zeros(0)
        for index, spec in zip(self.servo_index, self._servo_specs, strict=True):
            self._check(self._lib.lms_set_servo(self._handle, int(index), spec.kp, spec.kd, spec.limit),
                        spec.actuator)
        address = np.zeros(3, dtype=np.int64)
        qpos_addresses, dof_addresses = [], []
        for spec in self._servo_specs:
            joint = self._lib.lms_name2id(self._handle, _OBJ_JOINT, spec.joint.encode())
            self._check(joint, f"servo joint {spec.joint}")
            self._check(self._lib.lms_joint(self._handle, joint, address.ctypes.data_as(_INT64_P)), spec.joint)
            qpos_addresses.append(int(address[0]))
            dof_addresses.append(int(address[1]))
        self.servo_qpos = np.array(qpos_addresses, dtype=int)
        self.servo_dof = np.array(dof_addresses, dtype=int)
        self.command_index = np.array(
            [i for i in range(self.nu) if i not in set(self.servo_index.tolist())], dtype=int)
        self._command_readers = [self._scene_actuators[self.actuator_names[i]].actuatorInMsg
                                 for i in self.command_index]

    def connect_servo(self, spec: ServoSpec) -> None:
        """Declare one MJCF actuator as a MuJoCo position servo (before the first step).

        Accepts a fully built ``ServoSpec``; the position/velocity reference
        messages may be published by any module, including one installed later.
        """
        if self.servos_finalized:
            raise RuntimeError("servos cannot be added after the first physics step")
        if spec.actuator in {existing.actuator for existing in self._servo_specs}:
            raise ValueError(f"actuator {spec.actuator!r} is already served")
        if spec.actuator not in self.actuator_names:
            raise ValueError(f"servo actuator is missing from the model: {spec.actuator!r}")
        if spec.position_reference is None or spec.velocity_reference is None:
            raise ValueError(f"servo {spec.actuator!r} needs position and velocity references")
        self._servo_specs.append(spec)
        self.requestedOutMsgs.append(messaging.SingleActuatorMsg())
        self.appliedOutMsgs.append(messaging.SingleActuatorMsg())
        self._requested_writers.append(self.requestedOutMsgs[-1].addAuthor())
        self._applied_writers.append(self.appliedOutMsgs[-1].addAuthor())
        self._scene_actuators[spec.actuator].actuatorInMsg.subscribeTo(self.appliedOutMsgs[-1])
        self._position_refs.append(spec.position_reference.addSubscriber())
        self._velocity_refs.append(spec.velocity_reference.addSubscriber())
        self._refresh_actuators()

    def _finalize_actuators(self) -> None:
        """Verify the actuator wiring once, before the first step."""
        if self.servos_finalized:
            return
        overlapping = set(self.servo_index.tolist()) & set(self.command_index.tolist())
        if overlapping:
            raise RuntimeError(f"actuators are both served and commanded: {sorted(overlapping)}")
        self._refresh_actuators()
        self.servos_finalized = True

    def add_body_wrench_input(self, body: str, force_inertial: Any = None, torque_body: Any = None,
                              *, source: str = "external") -> _WrenchChannel:
        """Register one held external wrench source on one MJCF body.

        ``force_inertial``: ``CmdForceInertialMsg`` (``forceRequestInertial`` [N],
        inertial/local axes, at the body COM). ``torque_body``:
        ``CmdTorqueBodyMsg`` (``torqueRequestBody`` [N*m], body axes).

        Several sources may target the same body (drag, thrusters, magnetorquers,
        ...): the stepper sums them before every step, so no source needs to know
        the others. ``source`` names the writer and must be unique per body, so a
        mis-wired component fails at assembly instead of silently doubling a force.
        """
        if force_inertial is None and torque_body is None:
            raise ValueError("a body wrench input needs a force and/or a torque message")
        body_id = self._lib.lms_name2id(self._handle, _OBJ_BODY, body.encode())
        if body_id < 1:
            raise ValueError(f"unknown or world body for wrench input: {body!r}")
        if source in {channel.source for channel in self._wrench_channels}:
            raise ValueError(f"wrench source {source!r} is already registered")
        channel = _WrenchChannel(
            body_id, source,
            force_inertial.addSubscriber() if force_inertial is not None else None,
            torque_body.addSubscriber() if torque_body is not None else None)
        self._wrench_channels.append(channel)
        return channel

    def _apply_wrench_inputs(self) -> None:
        """Sum every registered source per body and hand the result to MuJoCo."""
        if not self._wrench_channels:
            return
        force = np.zeros(3)
        torque = np.zeros(3)
        for channel in self._wrench_channels:
            channel.force[:] = (channel.force_reader().forceRequestInertial
                                if channel.force_reader is not None else 0.0)
            channel.torque[:] = (channel.torque_reader().torqueRequestBody
                                 if channel.torque_reader is not None else 0.0)
        for body_id in sorted({channel.body_id for channel in self._wrench_channels}):
            force[:] = 0.0
            torque[:] = 0.0
            for channel in self._wrench_channels:
                if channel.body_id == body_id:
                    force += channel.force
                    torque += channel.torque
            self._check(self._lib.lms_set_body_wrench(self._handle, body_id,
                                                      _pointer(force), _pointer(torque)),
                        "body wrench")

    # --- construction helpers -------------------------------------------------------
    def _check(self, status: int, what: str) -> None:
        if status < 0:
            raise RuntimeError(f"local MuJoCo stepper error {status} ({what})")

    def _name(self, obj_type: int, index: int) -> str:
        buffer = ctypes.create_string_buffer(256)
        self._check(self._lib.lms_id2name(self._handle, obj_type, index, buffer, len(buffer)), "id2name")
        return buffer.value.decode()

    def _bind_scene(self) -> None:
        """Bind MJScene's (re-)registered bulk states; they are the published authority."""
        if self.scene.getActState() is not None:
            raise NotImplementedError("actuator activation states are not supported by the local backend")
        manager = self.scene.dynManager
        self._qpos_state = manager.getStateObject("mujocoQpos")
        self._qvel_state = manager.getStateObject("mujocoQvel")
        self._verify_layout()

    def _verify_layout(self) -> None:
        """The independent model must address qpos/qvel exactly like MJScene."""
        scene_q = np.asarray(self.scene.assembleFullQpos(), dtype=float).reshape(-1)
        scene_v = np.asarray(self.scene.assembleFullQvel(), dtype=float).reshape(-1)
        if scene_q.size != self.nq or scene_v.size != self.nv:
            raise RuntimeError(f"model layout mismatch: MJScene nq/nv {scene_q.size}/{scene_v.size}, "
                               f"local {self.nq}/{self.nv}")
        address = np.zeros(3, dtype=np.int64)
        for joint in range(self.njnt):
            name = self._name(_OBJ_JOINT, joint)
            self._check(self._lib.lms_joint(self._handle, joint, address.ctypes.data_as(_INT64_P)), name)
            if int(address[2]) == _JOINT_FREE:
                continue  # Free joints are checked through their bodies below.
            owner = self._joint_owner(name)
            scene_joint = self.scene.getBody(owner).getScalarJoint(name)
            if (scene_joint.getQposAdr(), scene_joint.getQvelAdr()) != (int(address[0]), int(address[1])):
                raise RuntimeError(f"joint {name} address differs between MJScene and the local model")
        scene_free = sorted(
            (self.scene.getBody(b).getFreeJoint().getQposAdr(), self.scene.getBody(b).getFreeJoint().getQvelAdr())
            for b in self.scene.getBodyNames() if self.scene.getBody(b).isFree())
        if scene_free != sorted(zip(self.free_qpos.tolist(), self.free_dof.tolist())):
            raise RuntimeError("free-joint layout differs between MJScene and the local model")
        self._verify_mass_properties()

    def _verify_mass_properties(self) -> None:
        """The publish-only copy must describe the same masses as this model.

        The local model integrates every body of the compiled MJCF. MJScene's
        publish-only copy drops the collision-only rigid flexcomps, so it can hold
        fewer bodies. Those dropped bodies carry no inertial mass in the current
        models, but nothing enforced that: a flexcomp with mass would silently bias
        every published COM and attitude state. Check both directions instead of
        trusting it: shared bodies must agree in mass, and a body present only in
        the local model must be massless.
        """
        values = np.zeros(4)
        scene_names = set(self.scene.getBodyNames())
        for index in range(self.nbody):
            name = self._name(_OBJ_BODY, index)
            if index == 0 or name == "world":
                continue  # MuJoCo's world body carries no mass and has no state.
            self._check(self._lib.lms_body_mass(self._handle, index, values.ctypes.data_as(_DOUBLE_P)),
                        f"body mass {name}")
            mass = float(values[0])
            if name in scene_names:
                scene_mass = float(self.scene.getBody(name).getMass())
                if not math.isclose(mass, scene_mass, rel_tol=1e-9, abs_tol=1e-9):
                    raise RuntimeError(
                        f"body {name!r} mass differs: MJScene {scene_mass}, local {mass}")
            elif mass > 0.0:
                raise RuntimeError(
                    f"body {name!r} is not published by MJScene but carries {mass} kg; "
                    "the publish-only copy would report wrong mass properties")

    def _joint_owner(self, joint_name: str) -> str:
        for body in self.scene.getBodyNames():
            try:
                self.scene.getBody(body).getScalarJoint(joint_name)
            except Exception:  # noqa: BLE001 - SWIG reports a missing joint as an exception
                continue
            return body
        raise RuntimeError(f"joint {joint_name} is not a scalar joint of any MJScene body")

    # --- frame conversion -------------------------------------------------------------
    def _to_local(self, q: np.ndarray, v: np.ndarray, r_o: np.ndarray, v_o: np.ndarray):
        q, v = q.copy(), v.copy()
        for qa, da in zip(self.free_qpos, self.free_dof):
            q[qa:qa + 3] -= r_o
            v[da:da + 3] -= v_o
        return q, v

    def _to_inertial(self, q: np.ndarray, v: np.ndarray, r_o: np.ndarray, v_o: np.ndarray):
        q, v = q.copy(), v.copy()
        for qa, da in zip(self.free_qpos, self.free_dof):
            q[qa:qa + 3] += r_o
            v[da:da + 3] += v_o
        return q, v

    def _scene_state(self) -> tuple[np.ndarray, np.ndarray]:
        return (np.asarray(self._qpos_state.getState(), dtype=float).reshape(-1),
                np.asarray(self._qvel_state.getState(), dtype=float).reshape(-1))

    def _adopt_scene_state(self, r_o: np.ndarray, v_o: np.ndarray) -> None:
        q, v = self._scene_state()
        q, v = self._to_local(q, v, r_o, v_o)
        self._check(self._lib.lms_set_state(self._handle, _pointer(q), _pointer(v)), "set_state")
        self.qpos[:], self.qvel[:] = q, v

    def _publish(self, current_sim_nanos: int, r_o: np.ndarray, v_o: np.ndarray, kinematics: bool = True) -> None:
        q, v = self._to_inertial(self.qpos, self.qvel, r_o, v_o)
        self._qpos_state.setState(q.reshape(-1, 1))
        self._qvel_state.setState(v.reshape(-1, 1))
        self._published_q, self._published_v = q, v
        if kinematics:
            # Copies the bulk states into MJScene's mjData and writes its
            # forward-kinematics (body/site) messages.
            self.scene.postIntegration(current_sim_nanos)
            self.kinematics_publish_count += 1
        # MJScene.UpdateState never integrates in this mode: it writes the
        # joint and scene state messages from the bulk states. The scene is
        # not in the physics task, so nothing else re-labels its messages.
        self.scene.UpdateState(current_sim_nanos)

    def _external_write(self) -> bool:
        q, v = self._scene_state()
        return not (np.array_equal(q, self._published_q) and np.array_equal(v, self._published_v))

    # --- Basilisk interface -------------------------------------------------------
    def Reset(self, current_sim_nanos: int) -> None:
        self._last_nanos = None
        self._previous_origin = None
        self._published_q = self._published_v = None
        if self.orbital_environment is not None:
            self.orbital_environment.reset()

    def UpdateState(self, current_sim_nanos: int) -> None:
        now = int(current_sim_nanos)
        now_s = now * 1.0e-9
        r_o, v_o = self.origin.state(now_s)
        if self._last_nanos is None:
            self._bind_scene()
            # The first step is the last moment a component may declare a servo.
            self._finalize_actuators()
        if self._last_nanos is None or self._external_write():
            # First tick, or initialization/reset code assigned MJScene states
            # (setPosition/setVelocity): adopt them as the new local state.
            if self._last_nanos is not None:
                self.external_writes += 1
            self._adopt_scene_state(r_o, v_o)
            self._previous_origin = (r_o.copy(), v_o.copy())
            if self._last_nanos is None:
                self._last_nanos = now
                self._publish(now, r_o, v_o)
                return
        dt = (now - self._last_nanos) * 1.0e-9
        if dt <= 0.0:
            self._publish(now, r_o, v_o)
            return
        for slot, index in enumerate(self.servo_index):
            q_ref = float(self._position_refs[slot]().state)
            qd_ref = float(self._velocity_refs[slot]().state)
            self.ctrl[index] = q_ref + self.servo_kd[slot] / self.servo_kp[slot] * qd_ref
        for reader, index in zip(self._command_readers, self.command_index):
            self.ctrl[index] = float(reader().input)
        if self._wrench_channels:
            self._apply_wrench_inputs()
        self._origin_buffer[:] = r_o
        ctrl, origin, qpos, qvel, qacc, force, stats = self._pointers
        if self.orbital_environment is None:
            status = self._lib.lms_step(self._handle, ctrl, dt, self.substeps, origin, self.origin.mu,
                                       qpos, qvel, qacc, force, stats)
        else:
            status = self._step_with_orbital_environment(now, dt, r_o, v_o)
        if status < 0:
            raise FloatingPointError(
                f"local MuJoCo dynamics diverged at t={now * 1e-9:.6f}s (status {status}, "
                f"substep {int(self.stats[5])}/{self.substeps}, contacts {int(self.stats[3])})")
        self.step_count += 1
        self.max_contacts = max(self.max_contacts, int(self.stats[0]))
        self.max_constraint_rows = max(self.max_constraint_rows, int(self.stats[1]))
        self.max_solver_iterations = max(self.max_solver_iterations, int(self.stats[2]))
        self._last_nanos = now
        if self.step_count % REBASE_CHECK_STEPS == 0 and self._rebase_if_needed(now_s):
            r_o, v_o = self.origin.state(now_s)
        self._previous_origin = (r_o.copy(), v_o.copy())
        self._publish(now, r_o, v_o, kinematics=(self.step_count % self.publish_every == 0
                                                 or self.step_count % SYNC_STEPS == 0))
        self._publish_effort(now)

    def _publish_effort(self, current_sim_nanos: int) -> None:
        for slot, index in enumerate(self.servo_index):
            # ctrl already holds q_ref + kd/kp*qd_ref, so this equals the PID
            # output kp*(q_ref - q) + kd*(qd_ref - qd) before the limiter.
            requested = (self.servo_kp[slot] * (self.ctrl[index] - self.qpos[self.servo_qpos[slot]])
                         - self.servo_kd[slot] * self.qvel[self.servo_dof[slot]])
            self._payload.input = float(requested)
            self._requested_writers[slot](self._payload, self.moduleID, current_sim_nanos)
            self._payload.input = float(self.actuator_force[index])
            self._applied_writers[slot](self._payload, self.moduleID, current_sim_nanos)

    def _rebase_if_needed(self, now_s: float) -> bool:
        """Move O onto the system COM once it drifted too far; inertial states are unchanged."""
        momentum = self.momentum()
        com = momentum["com_m"]
        if float(np.linalg.norm(com)) <= self.rebase_distance_m:
            return False
        dr, dv = com, momentum["com_velocity_m_s"]
        q, v = self._to_local(self.qpos, self.qvel, dr, dv)
        self._check(self._lib.lms_set_state(self._handle, _pointer(q), _pointer(v)), "rebase")
        self.qpos[:], self.qvel[:] = q, v
        self.origin.shift(dr, dv, now_s)
        self.rebase_count += 1
        return True

    # --- diagnostics ----------------------------------------------------------------------
    def momentum(self) -> dict[str, np.ndarray]:
        out = np.zeros(12)
        self._check(self._lib.lms_momentum(self._handle, _pointer(out)), "momentum")
        return {"linear_kg_m_s": out[0:3], "angular_about_com_kg_m2_s": out[3:6],
                "com_m": out[6:9], "com_velocity_m_s": out[9:12]}

    def telemetry(self) -> dict[str, Any]:
        return {"dynamics_backend": BACKEND_LOCAL, "integrator": "mujoco_implicitfast_fixed_step",
                "substeps": self.substeps, "steps": self.step_count, "kinematics_publish_stride": self.publish_every,
                "kinematics_publishes": self.kinematics_publish_count, "max_contacts": self.max_contacts,
                "max_constraint_rows": self.max_constraint_rows,
                "max_solver_iterations": self.max_solver_iterations,
                "external_state_writes": self.external_writes, "rebases": self.rebase_count,
                "tidal_mu_m3_s2": self.origin.mu if self.orbital_environment is None else 0.0,
                "orbital_environment": (self.orbital_environment.telemetry() if self.orbital_environment
                                        else {"mode": "linear_tidal"})}

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.lms_destroy(self._handle)
            self._handle = None

    def __del__(self) -> None:  # pragma: no cover - interpreter shutdown ordering
        try:
            self.close()
        except Exception:  # noqa: BLE001
            pass


def servo_specs(actuators, joints, kp, kd, limits, reference) -> list[ServoSpec]:
    """Servo specs for joint motors driven by one reference publisher."""
    if not (len(actuators) == len(joints) == len(kp) == len(kd) == len(limits)):
        raise ValueError("servo actuator and gain arrays must have equal length")
    for value in (*kp, *kd, *limits):
        if not math.isfinite(float(value)):
            raise ValueError("servo gains and limits must be finite")
    return [ServoSpec(name, joint, float(p), float(d), float(limit),
                      reference.positionOutMsgs[i], reference.velocityOutMsgs[i])
            for i, (name, joint, p, d, limit) in enumerate(zip(actuators, joints, kp, kd, limits, strict=True))]
