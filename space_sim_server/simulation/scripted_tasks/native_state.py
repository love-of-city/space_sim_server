"""Read task-specific state from the shared native simulation without changing it."""

import numpy as np
import xml.etree.ElementTree as ET
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from simulation.scripted_tasks.drawer_lift import DrawerSample
from simulation.scripted_tasks.plug_insert import PlugSample
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES


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
        self.moving_mount = bool(getattr(task, 'model_path', None) and
            ET.parse(task.model_path).find(
                './worldbody/body[@name="capture_target"]//body[@name="drawer_base"]') is not None)

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
        mount = None
        if self.moving_mount:
            base, drawer = [self.stepper.body_detail(n) for n in ('cubesat_bus', 'drawer_base')]
            rb = quaternion_wxyz_to_matrix(base[3:7])
            rd = rb.T @ quaternion_wxyz_to_matrix(drawer[3:7])
            axis_local = self.task.drawer_rotation.T @ self.task.axis
            origin = rb.T @ (drawer[:3]-base[:3]) - rd @ axis_local * self.stepper.qpos[self.drawer.getQposAdr()]
            mount = origin, rd
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
            self.stepper.qpos.copy(),
            mount,
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
        from simulation.scene_library.frames import site_attachment
        root = ET.parse(task.model_path).getroot()
        self.sites = {}
        for key, name in (('hole', task.config.target_site), ('pickup', task.profile.prefix + '_pickup')):
            owner, position, rotation = site_attachment(root, name)
            self.sites[key] = (scene.getBody(owner).getFreeJoint(), position, rotation)

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
        sites = {}
        for key, (joint, local_p, local_r) in self.sites.items():
            address = joint.getQposAdr()
            owner_r = quaternion_wxyz_to_matrix(state[address+3:address+7])
            sites[key] = (rb.T @ (state[address:address+3] + owner_r @ local_p - state[qa:qa+3]),
                          rb.T @ owner_r @ local_r)
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
            self.stepper.qpos.copy(),
            sites,
        )

    def metrics(self, sample):
        return self.task.metrics(sample)


STATE_READERS = {
    "drawer_cycle": DrawerState,
    "drawer_lift": DrawerState,
    "plug_insert": PlugState,
}

STATE_READERS.update({name: PlugState for name in PLUG_PROFILES})

from simulation.scripted_tasks.locking_rod import LockingRodState
STATE_READERS['locking_rod_cycle'] = LockingRodState
