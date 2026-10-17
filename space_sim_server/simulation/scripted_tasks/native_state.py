"""Read task-specific state from the shared native simulation without changing it."""

import numpy as np
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from simulation.scripted_tasks.drawer_lift import DrawerSample
from simulation.scripted_tasks.plug_insert import PlugSample


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
        )

    def metrics(self, sample):
        return self.task.metrics(sample)


STATE_READERS = {
    "drawer_cycle": DrawerState,
    "drawer_lift": DrawerState,
    "plug_insert": PlugState,
}
