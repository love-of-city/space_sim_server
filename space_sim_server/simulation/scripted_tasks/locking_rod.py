"""Arm actuator references for a captive rod; never write mechanism state."""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from simulation.scene_library.manifest import digest
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from .drawer_cycle import DrawerCycleTask
from .motion_timing import time_joint_path


@dataclass(frozen=True)
class LockingRodConfig:
    force_limit_n: float = 40.
    torque_limit_nm: float = 4.
    arm_speed_rad_s: float = .08
    grip_preload_rad: float = .04


@dataclass(frozen=True)
class LockingRodSample:
    time_s: float
    joints: np.ndarray
    joint_velocities: np.ndarray
    rod_angle_rad: float
    rod_velocity_rad_s: float
    fastener_slide_m: float
    fastener_spin_rad: float
    fastener_spin_velocity_rad_s: float
    hook_normal_force_n: float
    fastener_position_body_m: np.ndarray
    contacts: tuple
    wrench: np.ndarray
    wrench_valid: bool
    native_qpos: np.ndarray


class LockingRodState:
    def __init__(self, task, scene, stepper):
        self.task, self.stepper = task, stepper
        self.body_names = {i: stepper._name(1, i) for i in range(stepper.nbody)}
        self.joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in
                       [('locking_rod', 'locking_rod_hinge'), ('fastener', 'fastener_slide'),
                        ('fastener', 'fastener_spin')]]

    def sample(self, time_s, q, v, wrench):
        snapshot, detail = self.stepper.contact_snapshot(), self.stepper.contact_detail()
        if len(snapshot) != len(detail) or not np.array_equal(snapshot[:, :2], detail[:, :2]):
            raise ValueError('Native contact geometry and force records disagree')
        contacts, normal = [0, 0], 0.
        for row, force in zip(snapshot, detail):
            names = {self.body_names[int(row[2])], self.body_names[int(row[3])]}
            if row[4] <= .0001 and 'fastener' in names:
                for i in (0, 1):
                    contacts[i] += int(f'ee_finger{i+1}' in names)
            if names == {'fastener', 'hook'}:
                normal += max(0., force[6])
        base, nut = self.stepper.body_detail('cubesat_bus'), self.stepper.body_detail('fastener')
        position = quaternion_wxyz_to_matrix(base[3:7]).T @ (nut[:3]-base[:3])
        positions = [float(self.stepper.qpos[j.getQposAdr()]) for j in self.joints]
        rates = [float(self.stepper.qvel[j.getQvelAdr()]) for j in self.joints]
        return LockingRodSample(time_s, q, v, positions[0], rates[0], positions[1], positions[2],
            rates[2], normal, position, tuple(contacts), np.r_[wrench['force_n'], wrench['torque_nm']],
            bool(wrench['valid']), self.stepper.qpos.copy())

    def metrics(self, sample):
        return self.task.metrics(sample)


class LockingRodTask:
    name = 'locking_rod_cycle'
    version = 'locking-rod-cycle/1-candidate'
    stages = ('settle', 'approach', 'grip', 'fold', 'restore', 'tighten', 'release', 'retract', 'verify')

    def __init__(self, model, path):
        self.model_path = Path(model)
        self.path_file = Path(path)
        self.path_sha256 = digest(self.path_file)
        document = json.loads(self.path_file.read_text(encoding='utf-8-sig'))
        if document.get('schema') != 'space-arm-locking-rod-path/1' or document.get('model_sha256') != digest(model):
            raise ValueError('Rod path must match this exact model')
        geometry = DrawerCycleTask(model)
        self.chain, self.lower, self.upper = geometry.chain, geometry.lower, geometry.upper
        self.config = LockingRodConfig()
        self.paths = {}
        for stage in self.stages[1:-1]:
            rows = [s['joint_reference'] for s in document['samples'] if s['stage'] == stage]
            a = np.asarray(rows, dtype=float)
            if a.ndim != 2 or a.shape[1] != 8 or len(a) < 2 or not np.isfinite(a).all():
                raise ValueError('Incomplete rod actuator path')
            if np.any(a[:, :6] < self.lower) or np.any(a[:, :6] > self.upper):
                raise ValueError('Rod path exceeds arm joint range')
            if np.any(a[:, 6] < 0) or np.any(a[:, 6] > 1) or not np.allclose(a[:, 7], -a[:, 6]):
                raise ValueError('Invalid gripper path')
            if stage in ('grip', 'fold', 'restore', 'tighten'):
                a[:, 6] = np.maximum(0., a[:, 6] - self.config.grip_preload_rad)
                a[:, 7] = -a[:, 6]
            self.paths[stage] = a
        self.initial = np.asarray(document['initial_joint_position_rad'])
        self.stage, self.result, self.reason = 'pending', None, ''
        self.reference, self.velocity = np.zeros(8), np.zeros(8)
        self.events = []
        self._last_time = None
        self._stable = self._motion_s = self._overload = 0.
        self.max_force_n = self.max_torque_nm = 0.

    def _enter(self, stage, sample):
        self.stage, self._entered = stage, sample.time_s
        self._stable = self._motion_s = 0.
        self._timing = None
        if stage in self.paths:
            path = np.vstack([self.reference, self.paths[stage]])
            self._timing = time_joint_path(path, .15 if stage in ('approach', 'retract') else 0.,
                .5 if stage in ('fold', 'restore') else 2.2 if stage == 'tighten' else 0.,
                joint_speed=self.config.arm_speed_rad_s, linear_speed=.015, angular_speed=.1,
                linear_acceleration=.05, angular_acceleration=.2, joint_acceleration=.3)
        self._duration = self._timing.duration if self._timing else 0.
        self.events.append(dict(stage=stage, sim_time_s=sample.time_s, planned_motion_seconds=self._duration))

    def fail(self, reason, sample):
        self.result, self.reason = 'failed', reason
        self.reference[:], self.velocity[:] = sample.joints, 0.
        self.events.append(dict(stage='failed', during=self.stage, reason=reason, sim_time_s=sample.time_s))

    @staticmethod
    def locked(s):
        return (abs(s.rod_angle_rad) < .025 and .004 < s.fastener_slide_m < .007
                and s.hook_normal_force_n > 1. and abs(s.fastener_spin_velocity_rad_s) < .01)

    def step(self, s):
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if self._last_time is None:
            self.reference[:] = s.joints
            self._last_time = s.time_s
            if np.max(np.abs(s.joints-self.initial)) > .01 or abs(s.rod_angle_rad) > .02 or abs(s.fastener_slide_m) > .0001:
                self.fail('initial_rod_or_arm_state_mismatch', s)
            else:
                self._enter('settle', s)
            return self.reference.copy(), self.velocity.copy()
        dt, self._last_time = s.time_s-self._last_time, s.time_s
        if not 0 < dt <= .010001:
            raise ValueError('Rod controller requires monotonic >=100Hz observations')
        if not np.isfinite(np.r_[s.joints, s.joint_velocities, s.wrench,
                                 s.rod_angle_rad, s.fastener_slide_m, s.hook_normal_force_n]).all():
            self.fail('invalid_measurement', s)
            return self.reference.copy(), self.velocity.copy()
        force, torque = np.linalg.norm(s.wrench[:3]), np.linalg.norm(s.wrench[3:])
        self.max_force_n, self.max_torque_nm = max(self.max_force_n, force), max(self.max_torque_nm, torque)
        self._overload = self._overload+dt if force > 40. or torque > 4. else 0.
        if self._overload >= .1:
            self.fail('wrist_load_limit', s)
        elif not s.wrench_valid and s.time_s > .2:
            self.fail('wrist_sensor_unavailable', s)
        elif s.time_s-self._entered > max(20., 2.5*self._duration+10.):
            self.fail('stage_timeout', s)
        if self.result is not None:
            return self.reference.copy(), self.velocity.copy()
        old = self.reference.copy()
        if self._timing is not None and np.max(np.abs(self.reference[:6]-s.joints[:6])) < .06:
            self._motion_s = min(self._duration, self._motion_s+dt)
            self.reference[:] = self._timing.joint_state(self._motion_s)[0]
        self.velocity[:] = (self.reference-old)/dt
        stable = np.max(np.abs(s.joint_velocities[:6])) < .02
        reached = self._motion_s >= self._duration and stable
        bilateral = min(s.contacts) > 0
        if self.stage == 'grip':
            reached &= bilateral and s.joints[6] > .05
        elif self.stage == 'fold':
            reached &= bilateral and s.rod_angle_rad < -.45 and abs(s.rod_velocity_rad_s) < .01
        elif self.stage == 'restore':
            reached &= bilateral and abs(s.rod_angle_rad) < .02 and abs(s.rod_velocity_rad_s) < .01
        elif self.stage == 'tighten':
            reached &= bilateral and self.locked(s)
        elif self.stage in ('release', 'retract', 'verify'):
            reached &= self.locked(s) and max(s.contacts) == 0 and s.joints[6] > .95
            if self.stage != 'release':
                p, _ = self.chain.forward(s.joints[:6])
                reached &= np.linalg.norm(p-s.fastener_position_body_m) > .12
        else:
            reached &= np.max(np.abs(self.reference[:6]-s.joints[:6])) < .02
        self._stable = self._stable+dt if reached else 0.
        if self._stable >= (2. if self.stage == 'verify' else .3):
            index = self.stages.index(self.stage)
            if index == len(self.stages)-1:
                self.result = 'success'
                self.events.append(dict(stage='success', sim_time_s=s.time_s))
            else:
                self._enter(self.stages[index+1], s)
        return self.reference.copy(), self.velocity.copy()

    def metrics(self, s):
        return dict(task_name=self.name, task_version=self.version, rod_angle_rad=s.rod_angle_rad,
            rod_velocity_rad_s=s.rod_velocity_rad_s, fastener_slide_m=s.fastener_slide_m,
            fastener_spin_rad=s.fastener_spin_rad, fastener_spin_velocity_rad_s=s.fastener_spin_velocity_rad_s,
            hook_normal_force_n=s.hook_normal_force_n, finger_fastener_contacts=list(s.contacts),
            fastener_position_body_m=s.fastener_position_body_m.tolist(), path_sha256=self.path_sha256)
