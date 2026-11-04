"""Message-connected connector retention; the dynamics core owns all motion."""
from __future__ import annotations

from dataclasses import asdict
import json
import numpy as np
from Basilisk.architecture import messaging, sysModel

from simulation.assembly import Slot
from simulation.plug_retention import (
    PlugFriction, _cross3, load_retention_parameters, model_retention_sockets, paired_com_wrenches,
)
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from simulation.native_plug_retention import NativePlugFriction


class RetentionKinematics:
    """One immutable scene sample shared by the retention models on each tick.

    All retention models run before the dynamics core. Read the same published
    message once, and reuse each body's pose (especially the shared base).
    This cache belongs to one component/session and is cleared on Reset.
    """

    def __init__(self, state):
        self.reader = state.addSubscriber()
        self.reset()

    def reset(self):
        self.nanos = None
        self.poses = {}

    def sample(self, nanos):
        if self.nanos != nanos:
            state = self.reader()
            self.q = np.asarray(state.qpos, dtype=float).reshape(-1)
            self.v = np.asarray(state.qvel, dtype=float).reshape(-1)
            self.poses.clear()
            self.nanos = nanos
        return self.q, self.v

    def pose(self, layout, nanos):
        self.sample(nanos)
        qa, va, com = layout
        key = (qa, va)
        if key not in self.poses:
            rotation = quaternion_wxyz_to_matrix(self.q[qa+3:qa+7])
            position = self.q[qa:qa+3]
            self.poses[key] = (position, rotation, self.v[va:va+3],
                               rotation @ self.v[va+3:va+6], position + rotation @ com)
        return self.poses[key]


class PlugRetentionModel(sysModel.SysModel):
    def __init__(self, state, base_layout, plug_layout, detent, friction, *, name,
                 kinematics=None, claims=None, plug=None, owner=None):
        super().__init__()
        self.ModelTag = f"plugRetention_{name}"
        self.kinematics = kinematics if kinematics is not None else RetentionKinematics(state)
        self.reader = self.kinematics.reader
        self.base_layout, self.plug_layout, self.detent = base_layout, plug_layout, detent
        self.friction = friction
        self.outputs = [messaging.CmdForceInertialMsg(), messaging.CmdTorqueBodyMsg(),
                        messaging.CmdForceInertialMsg(), messaging.CmdTorqueBodyMsg()]
        self._writers = [message.addAuthor() for message in self.outputs]
        self._payloads = [messaging.CmdForceInertialMsgPayload(), messaging.CmdTorqueBodyMsgPayload(),
                          messaging.CmdForceInertialMsgPayload(), messaging.CmdTorqueBodyMsgPayload()]
        self.last_force_n = np.zeros(3)
        self.last_sample_ns = None
        self._configured_socket = None
        self.claims, self.plug, self.owner = claims, plug, owner

    def Reset(self, nanos):
        self.kinematics.reset()
        if self.claims is not None and self.claims.get(self.plug) == self.owner:
            self.claims.pop(self.plug)
        self.detent.reset()
        self.friction.configure()
        self._configured_socket = None
        self.last_force_n = np.zeros(3)
        self.last_sample_ns = None
        self._write([np.zeros(3)] * 4, nanos)

    def _write(self, values, nanos):
        for index, (writer, payload, value) in enumerate(zip(self._writers, self._payloads, values, strict=True)):
            if index % 2 == 0:
                payload.forceRequestInertial = value.tolist()
            else:
                payload.torqueRequestBody = value.tolist()
            writer(payload, self.moduleID, nanos)

    def UpdateState(self, nanos):
        # Scene initial conditions may be set AFTER InitializeSimulation.
        # Wait for the core to adopt/publish them; Reset's MJScene message can
        # still contain the XML pose and would consume the one initial capture.
        if not self.friction.stepper.state_initialized or not self.reader.isWritten():
            return
        if self.claims is not None and self.claims.get(self.plug) not in (None, self.owner):
            self.friction.configure()
            self._configured_socket = None
            self.last_force_n = np.zeros(3)
            self.last_sample_ns = int(nanos)
            self._write([np.zeros(3)] * 4, nanos)
            return
        if isinstance(self.detent, NativePlugFriction):
            q, v = self.kinematics.sample(nanos)
            values = self.detent.paired_wrenches(q, v, self.base_layout, self.plug_layout)
            force = values[0]
        else:
            bp, br, bv, bw, bc = self.kinematics.pose(self.base_layout, nanos)
            pp, pr, pv, pw, pc = self.kinematics.pose(self.plug_layout, nanos)
            offset = pp-bp
            force, torque = self.detent.wrench(
                br.T @ offset, br.T @ pr, br.T @ (pv-bv-_cross3(bw, offset)), br.T @ (pw-bw))
            force, torque = br @ force, br @ torque
            values = paired_com_wrenches(force, torque, pp, pc, bc, pr, br)
        socket = self.detent.active
        if self.claims is not None:
            if socket is not None:
                self.claims[self.plug] = self.owner
            elif self.claims.get(self.plug) == self.owner:
                self.claims.pop(self.plug)
        # Socket frames are fixed in their owning body. The native core moves
        # them each substep, so unchanged enabled sockets need no rewrite.
        # Keep disabled calls to clear free-state force telemetry.
        if socket is None or socket is not self._configured_socket:
            self.friction.configure(enabled=socket is not None,
                                    axis=(0., 0., 1.) if socket is None else socket.rotation_body[:, 2],
                                    seat=(0., 0., 0.) if socket is None else socket.position_body_m)
            self._configured_socket = socket
        self.last_force_n, self.last_sample_ns = force, int(nanos)
        self._write(values, nanos)

    def telemetry(self):
        friction = self.friction.telemetry()
        status = self.detent.status
        if self.detent.active is not None and friction["friction_mode"] == "static" and status != "retained":
            status = "held"
        return dict(status=status,
                    socket=None if self.detent.active is None else self.detent.active.name,
                    axial_displacement_m=self.detent.displacement_m,
                    force_inertial_n=(self.last_force_n + friction["force_inertial_n"]).tolist(),
                    axial_friction_force_n=friction["axial_force_n"],
                    axial_velocity_m_s=friction["axial_velocity_m_s"],
                    friction_mode=friction["friction_mode"],
                    sample_time_ns=None if self.last_sample_ns is None else str(self.last_sample_ns))


class PlugRetentionComponent:
    name = "plug_retention"

    def __init__(self, *, model_path, parameters=None, sockets=None, base="cubesat_bus", guidance_backend="native"):
        self.model_path, self.parameters, self.sockets, self.base = model_path, parameters, sockets, base
        if guidance_backend not in ("native", "python"):
            raise ValueError("guidance_backend must be native or python")
        self.guidance_backend = guidance_backend
        self.models = {}

    def install(self, ctx):
        if self.parameters is None:
            raise ValueError("plug retention parameters must be supplied explicitly")
        if ctx.ports.stepper is None:
            raise RuntimeError("plug retention requires the local dynamics backend")
        sockets = self.sockets if self.sockets is not None else retention_sockets(self.model_path)
        if not sockets:
            return
        self.moving_owners = any(s.owner_body not in (None, self.base) for seats in sockets.values() for s in seats)
        claims, groups = {}, []
        self.model_bodies = {}
        kinematics = RetentionKinematics(ctx.ports.scene_state())
        guidance = NativePlugFriction if self.guidance_backend == "native" else PlugFriction
        for body, all_seats in sockets.items():
            owners = list(dict.fromkeys(s.owner_body or self.base for s in all_seats))
            for owner in owners:
                seats = tuple(s for s in all_seats if (s.owner_body or self.base) == owner)
                key = body if len(owners) == 1 else body + '@' + owner
                groups.append((body, owner, seats, key))
        for body, owner, seats, key in groups:
            base_layout = ctx.ports.free_body_layout(owner)
            friction = ctx.ports.axial_friction(body, owner, static_n=self.parameters.static_friction_n,
                                                kinetic_n=self.parameters.sliding_friction_n,
                                                travel_m=self.parameters.friction_travel_m)
            model = PlugRetentionModel(ctx.ports.scene_state(), base_layout,
                                       ctx.ports.free_body_layout(body), guidance(seats, self.parameters),
                                       friction, name=key, kinematics=kinematics,
                                       claims=claims, plug=body, owner=owner)
            ctx.ports.add_body_wrench(body, source=f"{self.name}_{key}",
                                     force_inertial=model.outputs[0], torque_body=model.outputs[1])
            ctx.ports.add_body_wrench(owner, source=f"{self.name}_{key}_reaction",
                                     force_inertial=model.outputs[2], torque_body=model.outputs[3])
            ctx.add(model, Slot.ACTUATOR)
            ctx.keep_alive(model)
            self.models[key] = model
            self.model_bodies[key] = body
        ctx.extra[self.name] = self

    def telemetry(self):
        result = {}
        for key, model in self.models.items():
            body = self.model_bodies[key]
            if body not in result or model.detent.active is not None:
                result[body] = dict(model.telemetry())
                if self.moving_owners:
                    result[body]["owner_body"] = model.owner
        return result

    def configuration(self):
        return dict(enabled=True, version="plug-retention/3-moving-owner" if self.moving_owners else "plug-retention/2-dry-friction", guidance_backend=self.guidance_backend,
                    parameters=asdict(self.parameters),
                    sockets={body: [dict(name=s.name, initial=s.initial, keyed=s.keyed,
                                         **({"owner_body": model.owner} if self.moving_owners else {}), position_body_m=s.position_body_m.tolist(),
                                         rotation_body=s.rotation_body.tolist(),
                                         plug_rotation_body=s.plug_rotation_body.tolist())
                                    for s in model.detent.sockets]
                             for body, model in self.models.items()})


def install_configured_plug_retention(ctx):
    parameters = load_retention_parameters()
    if parameters is None:
        return None
    sockets = retention_sockets(ctx.config.model_path)
    if not sockets:
        return None
    component = PlugRetentionComponent(model_path=ctx.config.model_path, parameters=parameters, sockets=sockets)
    ctx.install([component])
    print(json.dumps(dict(type="plug_retention_configuration", model=str(ctx.config.model_path),
                         **component.configuration()), sort_keys=True), flush=True)
    return component


def retention_sockets(model_path):
    """Resolve moving ownership only for models with target-owned seats."""
    import xml.etree.ElementTree as ET
    from simulation.scene_library.frames import site_attachment
    root = ET.parse(model_path).getroot()
    names = [site.get("name", "") for site in root.iter("site")]
    moving = any(site_attachment(root, name)[0] != "cubesat_bus" for name in names
                 if name.endswith(("_pickup", "_insert_40p1", "_insert_40p5")))
    return model_retention_sockets(model_path, owned=moving)
