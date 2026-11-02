"""Read task-specific state from the shared native simulation without changing it."""

import numpy as np
from dataclasses import dataclass, field
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from simulation.scripted_tasks.drawer_lift import DrawerSample
from simulation.scripted_tasks.plug_insert import PlugSample
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES
from simulation.scripted_tasks.threaded_tasks import THREADED_TASKS
from simulation.scripted_tasks.floating_capture import FLOATING_TASKS


class DrawerState:
    def __init__(self, task, scene, stepper):
        self.task, self.stepper = task, stepper
        self.body_names = (
            {i: stepper._name(1, i) for i in range(stepper.nbody)}
            if hasattr(task, "version")
            else {}
        )
        self.drawer = scene.getBody("drawer_base").getScalarJoint("drawer_base_slide")
        self.buckle = scene.getBody("drawer_buckle").getScalarJoint(
            "drawer_buckle_joint"
        )

    def sample(self, time_s, q, v, wrench):
        contacts = None
        if self.body_names:
            contacts = [0, 0]
            for _, _, first, second, distance in self.stepper.contact_snapshot():
                names = {
                    self.body_names.get(int(first)),
                    self.body_names.get(int(second)),
                }
                if distance <= 0.0001 and names.intersection(
                    {"drawer_base", "drawer_buckle"}
                ):
                    for index in (0, 1):
                        if f"ee_finger{index + 1}" in names:
                            contacts[index] += 1
        return DrawerSample(
            time_s,
            q,
            v,
            float(self.stepper.qpos[self.drawer.getQposAdr()]),
            float(self.stepper.qvel[self.drawer.getQvelAdr()]),
            float(self.stepper.qpos[self.buckle.getQposAdr()]),
            np.r_[wrench["force_n"], wrench["torque_nm"]],
            wrench["valid"],
            tuple(contacts) if contacts is not None else None,
            native_qpos=self.stepper.qpos.copy(),
        )

    def metrics(self, sample):
        return dict(
            **(self.task.metrics(sample) if hasattr(self.task, "metrics") else {}),
            drawer_position_m=sample.drawer_m,
            drawer_m=sample.drawer_m,
            final_drawer_velocity_m_s=sample.drawer_velocity_m_s,
            drawer_velocity_m_s=sample.drawer_velocity_m_s,
            buckle_rad=sample.buckle_rad,
            finger_drawer_contacts=sample.finger_drawer_contacts,
            drawer_displacement_m=sample.drawer_m - self.task._initial_drawer,
        )


class PlugState:
    def __init__(self, task, scene, stepper):
        self.task, self.stepper = task, stepper
        self.base = scene.getBody("cubesat_bus").getFreeJoint()
        self.plug = scene.getBody(task.config.plug_body).getFreeJoint()
        self.body_names = {i: stepper._name(1, i) for i in range(stepper.nbody)}

    def sample(self, time_s, q, v, wrench):
        qa, va = self.base.getQposAdr(), self.base.getQvelAdr()
        pa, pv = self.plug.getQposAdr(), self.plug.getQvelAdr()
        state, rate = self.stepper.qpos, self.stepper.qvel
        rb = quaternion_wxyz_to_matrix(state[qa + 3 : qa + 7])
        rp = quaternion_wxyz_to_matrix(state[pa + 3 : pa + 7])
        position = rb.T @ (state[pa : pa + 3] - state[qa : qa + 3])
        rotation = rb.T @ rp
        speed = rb.T @ (rate[pv : pv + 3] - rate[va : va + 3]) - np.cross(
            rate[va + 3 : va + 6], position
        )
        omega = rotation @ rate[pv + 3 : pv + 6] - rate[va + 3 : va + 6]
        contacts = None
        if getattr(self, "body_names", None):
            contacts = [0, 0]
            for _, _, first, second, distance in self.stepper.contact_snapshot():
                names = {
                    self.body_names.get(int(first)),
                    self.body_names.get(int(second)),
                }
                if distance <= 0.0001 and self.task.config.plug_body in names:
                    for index in (0, 1):
                        if f"ee_finger{index + 1}" in names:
                            contacts[index] += 1
        return PlugSample(
            time_s,
            q,
            v,
            position,
            rotation,
            speed,
            omega,
            np.r_[wrench["force_n"], wrench["torque_nm"]],
            wrench["valid"],
            tuple(contacts) if contacts is not None else None,
            native_qpos=state.copy(),
        )

    def metrics(self, sample):
        return self.task.metrics(sample)


def _body_rotation(state):
    """SCStates uses MRP for N→body; return the active body→N matrix."""
    sigma = np.asarray(state.sigma_BN, dtype=float)
    squared = float(sigma @ sigma)
    return quaternion_wxyz_to_matrix(
        np.r_[(1.0 - squared) / (1.0 + squared), 2.0 * sigma / (1.0 + squared)]
    )


class ThreadedState:
    """Read free and articulated part poses from official body messages.

    The captive fastener is a child of the hinged rod, so interpreting its
    scalar qpos as a free-joint quaternion would give a fictitious pose. Body
    origins publish the complete articulated transform; scalar positions and
    rates are independently read at the joints' actual native addresses.
    """

    scalar_names = ("locking_rod_hinge", "fastener_slide", "fastener_spin")
    contact_pairs = {
        "fastener_hook_contacts": frozenset(("fastener", "hook")),
        "locking_rod_clip_contacts": frozenset(("locking_rod", "limit_clip")),
        "fastener_clip_contacts": frozenset(("fastener", "limit_clip")),
    }

    def __init__(self, task, scene, stepper):
        self.task, self.stepper = task, stepper
        self.base = scene.getBody("cubesat_bus").getOrigin().stateOutMsg
        self.part = scene.getBody(task.config.part_body).getOrigin().stateOutMsg
        self.body_names = {i: stepper._name(1, i) for i in range(stepper.nbody)}
        self.scalar_joints = {}
        if task.name == "locking_rod_cycle":
            for name in self.scalar_names:
                for body_name in scene.getBodyNames():
                    try:
                        joint = scene.getBody(body_name).getScalarJoint(name)
                    except Exception:  # SWIG raises when this body does not own it.
                        continue
                    self.scalar_joints[name] = joint
                    break
                else:
                    raise ValueError(f"Threaded task requires scalar joint {name}")

    def sample(self, time_s, q, v, wrench):
        from simulation.scripted_tasks.threaded_motion import ThreadedSample

        base, part = self.base.read(), self.part.read()
        rb, rp = _body_rotation(base), _body_rotation(part)
        position = rb.T @ (np.asarray(part.r_BN_N) - np.asarray(base.r_BN_N))
        rotation = rb.T @ rp
        base_omega = np.asarray(base.omega_BN_B)
        speed = rb.T @ (np.asarray(part.v_BN_N) - np.asarray(base.v_BN_N))
        speed -= np.cross(base_omega, position)
        omega = rotation @ np.asarray(part.omega_BN_B) - base_omega
        scalars = {}
        for name, joint in self.scalar_joints.items():
            scalars[name] = float(self.stepper.qpos[joint.getQposAdr()])
            scalars[name + "_velocity"] = float(self.stepper.qvel[joint.getQvelAdr()])
        contacts = [0, 0]
        if self.task.name == "locking_rod_cycle":
            scalars.update({name: 0.0 for name in self.contact_pairs})
        force_snapshot = (self.task.name == "locking_rod_cycle"
                          and hasattr(self.stepper, "contact_force_snapshot"))
        snapshot = (self.stepper.contact_force_snapshot() if force_snapshot
                    else self.stepper.contact_snapshot())
        if force_snapshot:
            scalars["fastener_hook_normal_force_n"] = 0.0
            scalars["locking_rod_clip_normal_force_n"] = 0.0
        for contact in snapshot:
            _, _, first, second, distance = contact[:5]
            if distance > 0.0001:
                continue
            names = frozenset((self.body_names.get(int(first)), self.body_names.get(int(second))))
            if self.task.config.part_body in names:
                for index in (0, 1):
                    if f"ee_finger{index + 1}" in names:
                        contacts[index] += 1
            for key, pair in self.contact_pairs.items():
                if self.task.name == "locking_rod_cycle" and pair == names:
                    scalars[key] += 1.0
                    if force_snapshot:
                        force_key = key.replace("_contacts", "_normal_force_n")
                        if force_key in scalars:
                            scalars[force_key] += max(0.0, float(contact[5]))
        latch_contacts = [0, 0]
        # Contact positions and this pose share frame L and the same native
        # substep. Orbital body messages include a large origin offset and may
        # be published on a slower grid; mixing them made contact flicker.
        part_position_local, part_rotation_local = self.stepper.body_geometry_pose(self.task.config.part_body)
        for contact in self.stepper.contact_geometry_snapshot():
            if contact[4] > .0001:
                continue
            names = frozenset((self.body_names.get(int(contact[2])), self.body_names.get(int(contact[3]))))
            if self.task.config.part_body not in names:
                continue
            local = part_rotation_local.T@(contact[5:8]-part_position_local)
            if self.task.is_latch_contact(self.task.config.part_body, local):
                for index in (0, 1):
                    if f"ee_finger{index+1}" in names:
                        latch_contacts[index] += 1
        return ThreadedSample(
            time_s, q, v, position, rotation, speed, omega,
            np.r_[wrench["force_n"], wrench["torque_nm"]], wrench["valid"],
            tuple(contacts), scalars, self.stepper.qpos.copy(),
            tuple(latch_contacts),
        )

    def metrics(self, sample):
        return self.task.metrics(sample)


@dataclass(frozen=True)
class FloatingSample(PlugSample):
    unexpected_contacts: list = field(default_factory=list)
    finger_normal_force_n: tuple = (0., 0.)


class FloatingState(PlugState):
    def sample(self, time_s, q, v, wrench):
        sample = super().sample(time_s, q, v, wrench)
        unexpected = []
        normal_force = [0., 0.]
        for contact in self.stepper.contact_force_snapshot():
            _, _, first, second, distance = contact[:5]
            names = {self.body_names[int(first)], self.body_names[int(second)]}
            if self.task.asset.body in names and distance <= .0001:
                for index in (0, 1):
                    if f"ee_finger{index+1}" in names:
                        normal_force[index] += max(0., float(contact[5]))
            if distance >= -.0005:
                continue
            a, b = self.body_names[int(first)], self.body_names[int(second)]
            arm = lambda name: name.startswith(("ee", "wrist_ft")) or name in {f"link{i}" for i in range(1, 6)}
            if a.startswith("ee") and b.startswith("ee"):
                continue
            if ((a in {"ee_finger1", "ee_finger2"} and b == self.task.asset.body)
                    or (b in {"ee_finger1", "ee_finger2"} and a == self.task.asset.body)):
                continue
            if arm(a) or arm(b) or self.task.asset.body in (a, b):
                unexpected.append(dict(body1=a, body2=b, penetration_m=float(-distance)))
        return FloatingSample(**vars(sample), unexpected_contacts=unexpected,
                              finger_normal_force_n=tuple(normal_force))


STATE_READERS = {
    "drawer_cycle": DrawerState,
    "drawer_lift": DrawerState,
    "plug_insert": PlugState,
}

STATE_READERS.update({name: PlugState for name in PLUG_PROFILES})
STATE_READERS.update({name: ThreadedState for name in THREADED_TASKS})
STATE_READERS.update({name: FloatingState for name in FLOATING_TASKS})
