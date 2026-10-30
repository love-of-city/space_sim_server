"""Task 5: intercept a freely moving team asset using measured simulation state.

The controller writes actuator references only. A successful capture requires
bilateral finger contact and a stable object-to-hand transform, never a weld.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics, rotation_matrix_to_vector,
)


@dataclass(frozen=True)
class FloatingAsset:
    body: str
    label: str
    instruction_name: str
    grip_point: tuple[float, float, float]
    closing_axis: tuple[float, float, float]
    approach_axis: tuple[float, float, float]


FLOATING_TASKS = {
    "floating_bolt_capture": FloatingAsset(
        "bolt", "漂浮航空螺栓", "floating aviation bolt", (0., 0., -.005),
        (0., 1., 0.), (0., 0., 1.)),
    "floating_plug_capture": FloatingAsset(
        "cylindrical_plug", "漂浮圆柱插头", "floating cylindrical plug", (0., 0., -.006),
        (0., 1., 0.), (0., 0., 1.)),
    "floating_panel_capture": FloatingAsset(
        "satellite_outer_panel", "漂浮卫星外侧板", "detached satellite panel",
        (-.36, .0046, 0.), (0., 1., 0.), (1., 0., 0.)),
}
FLOATING_BASELINE_DEG = (90., -60., 60., 0., -45., 0.)


@dataclass(frozen=True)
class FloatingCaptureConfig:
    task: str = "floating_bolt_capture"
    arm_speed_rad_s: float = .35
    linear_speed_m_s: float = .04
    gripper_speed_rad_s: float = .25
    hold_s: float = 3.
    force_limit_n: float = 80.
    torque_limit_nm: float = 8.
    timeout_s: float = 150.

    def __post_init__(self):
        if self.task not in FLOATING_TASKS:
            raise ValueError("Unknown floating capture task")
        for key, value in vars(self).items():
            if key != "task" and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{key} must be finite and positive")
        if self.arm_speed_rad_s > .35 or self.linear_speed_m_s > .04:
            raise ValueError("Floating capture exceeds configured motion limits")

    @property
    def plug_body(self):
        # The shared free-body reader uses this field for plugs and fragments.
        return FLOATING_TASKS[self.task].body


class FloatingCaptureTask:
    version = "floating-capture/1"
    stages = ("open", "intercept", "grip", "hold")

    def __init__(self, model_path, config=None):
        self.model_path = Path(model_path)
        self.config = config or FloatingCaptureConfig()
        self.name = self.config.task
        self.asset = FLOATING_TASKS[self.name]
        self.chain = SerialChainKinematics.from_mjcf(
            self.model_path, base_body="cubesat_bus",
            joint_names=tuple(f"joint{i}" for i in range(1, 7)), tool_site="sarm_ee")
        root = ET.parse(model_path).getroot()
        body = root.find(f'./worldbody/body[@name="{self.asset.body}"]')
        if body is None or (body.find("freejoint") is None and body.find('joint[@type="free"]') is None):
            raise ValueError("Floating target must be an independent free body")
        self.lower, self.upper = np.array([
            np.fromstring(root.find(f'.//joint[@name="joint{i}"]').get("range"), sep=" ")
            for i in range(1, 7)]).T
        self.stage, self.result, self.reason = "pending", None, ""
        self.reference, self.velocity = np.zeros(8), np.zeros(8)
        self.events = []
        self.max_force_n = self.max_torque_nm = 0.
        self._last_time = None
        self._entered = self._stable = self._overload = 0.
        self._held_local = self._hold_joints = None
        self._held_rotation = None
        self._hold_opening = 0.
        self._contact_loss = 0.

    def grasp_pose(self, sample):
        rotation = sample.plug_rotation_body
        point = sample.plug_position_body_m + rotation @ np.asarray(self.asset.grip_point)
        x = rotation @ self.asset.closing_axis
        z = rotation @ self.asset.approach_axis
        return point, np.column_stack((x, np.cross(z, x), z))

    def metrics(self, sample):
        point, desired = self.grasp_pose(sample)
        p, r = self.chain.forward(sample.joints[:6])
        local = r.T @ (point-p)
        j = self.chain.jacobian(sample.joints[:6])
        target_velocity = sample.plug_velocity_body_m_s + np.cross(
            sample.plug_angular_velocity_body_rad_s,
            sample.plug_rotation_body @ np.asarray(self.asset.grip_point))
        relative = target_velocity - j[:3] @ sample.joint_velocities[:6]
        return dict(
            task_version=self.version, target_body=self.asset.body,
            feedback_source="authoritative_simulation_state",
            target_position_body_m=sample.plug_position_body_m.tolist(),
            target_velocity_body_m_s=sample.plug_velocity_body_m_s.tolist(),
            target_angular_velocity_body_rad_s=sample.plug_angular_velocity_body_rad_s.tolist(),
            grasp_error_m=float(np.linalg.norm(point-p)),
            orientation_error_rad=float(np.linalg.norm(rotation_matrix_to_vector(desired@r.T))),
            relative_speed_m_s=float(np.linalg.norm(relative)),
            finger_target_contacts=list(sample.finger_plug_contacts or (0, 0)),
            finger_normal_force_n=list(sample.finger_normal_force_n),
            held_seconds=self._stable if self.stage == "hold" else 0.,
            slip_m=0. if self._held_local is None else float(np.linalg.norm(local-self._held_local)),
            angular_slip_rad=0. if self._held_rotation is None else float(np.linalg.norm(
                rotation_matrix_to_vector((r.T@sample.plug_rotation_body)@self._held_rotation.T))),
            grasp_point_tool_m=local.tolist(),
            unexpected_contacts=getattr(sample, "unexpected_contacts", []),
        )

    def _enter(self, stage, sample):
        self.stage, self._entered, self._stable = stage, sample.time_s, 0.
        self.events.append(dict(stage=stage, sim_time_s=float(sample.time_s)))

    def fail(self, reason, sample):
        if self.result is None:
            self.result, self.reason = "failed", reason
            self.events.append(dict(stage="failed", sim_time_s=float(sample.time_s), reason=reason))
            if np.isfinite(sample.joints).all():
                self.reference[:] = sample.joints
            self.velocity[:] = 0.

    def _track(self, sample, dt):
        point, desired = self.grasp_pose(sample)
        p, r = self.chain.forward(sample.joints[:6])
        error = point-p
        speed = sample.plug_velocity_body_m_s + np.cross(
            sample.plug_angular_velocity_body_rad_s,
            sample.plug_rotation_body @ np.asarray(self.asset.grip_point))
        translation = speed + 2.5*error
        translation *= min(1., self.config.linear_speed_m_s/max(np.linalg.norm(translation), 1e-12))
        angular = sample.plug_angular_velocity_body_rad_s + 2.*rotation_matrix_to_vector(desired@r.T)
        angular *= min(1., .3/max(np.linalg.norm(angular), 1e-12))
        if dt <= 0:
            return
        dq = self.chain.inverse_velocity_ik_pose(
            np.clip(sample.joints[:6], self.lower, self.upper), np.r_[translation, angular],
            joint_velocity_limits=np.full(6, self.config.arm_speed_rad_s),
            joint_position_min=self.lower, joint_position_max=self.upper, dt=dt,
        ).joint_velocity_rad_s
        self.reference[:6] = np.clip(
            np.clip(self.reference[:6]+dt*dq, sample.joints[:6]-.04, sample.joints[:6]+.04),
            self.lower, self.upper)

    def step(self, sample):
        if not math.isfinite(sample.time_s) or (self._last_time is not None and sample.time_s < self._last_time):
            raise ValueError("Task time must be finite and monotonic")
        dt = 0. if self._last_time is None else sample.time_s-self._last_time
        self._last_time = sample.time_s
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        contacts = np.asarray(sample.finger_plug_contacts)
        forces = np.asarray(sample.finger_normal_force_n)
        if (contacts.shape != (2,) or forces.shape != (2,)
                or not np.isfinite(contacts).all() or not np.isfinite(forces).all()
                or np.any(contacts < 0) or np.any(contacts != np.floor(contacts))
                or np.any(forces < 0)):
            self.fail("invalid_contact_observation", sample)
            return self.reference.copy(), np.zeros(8)
        old = self.reference.copy()
        force, torque = np.linalg.norm(sample.wrench[:3]), np.linalg.norm(sample.wrench[3:])
        self.max_force_n, self.max_torque_nm = max(self.max_force_n, float(force)), max(self.max_torque_nm, float(torque))
        self._overload = self._overload+dt if force > self.config.force_limit_n or torque > self.config.torque_limit_nm else 0.
        if not np.isfinite(np.r_[sample.joints, sample.joint_velocities, sample.wrench,
                                 sample.plug_position_body_m, sample.plug_velocity_body_m_s,
                                 sample.plug_rotation_body.ravel(), sample.plug_angular_velocity_body_rad_s,
                                 sample.finger_normal_force_n]).all():
            self.fail("nonfinite_state", sample)
        elif not sample.wrench_valid and sample.time_s > .2:
            self.fail("wrist_sensor_unavailable", sample)
        elif self._overload >= .1:
            self.fail("wrist_load_limit", sample)
        elif getattr(sample, "unexpected_contacts", []):
            self.fail("unexpected_collision", sample)
        elif sample.time_s > self.config.timeout_s:
            self.fail("interception_timeout", sample)
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if self.stage == "pending":
            self.reference[:] = sample.joints
            old = self.reference.copy()
            self._enter("open", sample)
        if self.stage == "intercept" or (self.stage == "grip" and self._hold_joints is None):
            self._track(sample, dt)
        elif self.stage in ("grip", "hold") and self._hold_joints is not None:
            self.reference[:6] = self._hold_joints
        opening = (1. if self.stage in ("open", "intercept") else
                   self._hold_opening if self._hold_joints is not None else 0.)
        jaw_speed = min(.12, self.config.gripper_speed_rad_s) if self.stage == "grip" else self.config.gripper_speed_rad_s
        self.reference[6] += np.clip(opening-self.reference[6], -dt*jaw_speed, dt*jaw_speed)
        self.reference[7] = -self.reference[6]
        m = self.metrics(sample)
        bilateral = all(count > 0 for count in m["finger_target_contacts"]) and min(m["finger_normal_force_n"]) >= .02
        if self.stage == "open" and sample.joints[6] >= .95:
            self._enter("intercept", sample)
        elif self.stage == "intercept":
            ready = m["grasp_error_m"] < .0015 and m["orientation_error_rad"] < .025 and m["relative_speed_m_s"] < .006
            self._stable = self._stable+dt if ready else 0.
            if self._stable >= .15:
                self._enter("grip", sample)
        elif self.stage == "grip":
            if bilateral and m["grasp_error_m"] < .008 and self._hold_joints is None:
                # Stop following contact-induced object jitter. Hold through
                # the original position servos with a small jaw preload.
                self._hold_joints = self.reference[:6].copy()
                self._hold_opening = max(0., sample.joints[6]-.002)
                self.reference[6:] = [self._hold_opening, -self._hold_opening]
            ready = bilateral and m["grasp_error_m"] < .008 and m["relative_speed_m_s"] < .01
            self._stable = self._stable+dt if ready else 0.
            if self._stable >= .3:
                self._held_local = np.asarray(m["grasp_point_tool_m"])
                self._held_rotation = self.chain.forward(sample.joints[:6])[1].T@sample.plug_rotation_body
                self._enter("hold", sample)
            elif sample.time_s-self._entered > 20.:
                self.fail("bilateral_contact_not_established", sample)
        elif self.stage == "hold":
            ready = bilateral and m["slip_m"] < .003 and m["angular_slip_rad"] < .05 and m["relative_speed_m_s"] < .005
            self._stable = self._stable+dt if ready else 0.
            self._contact_loss = 0. if bilateral else self._contact_loss+dt
            if self._contact_loss > .5:
                self.fail("target_lost", sample)
            elif self._stable >= self.config.hold_s:
                self.result, self.reason = "success", "bilateral_contact_and_stable_hold"
                self.events.append(dict(stage="success", sim_time_s=float(sample.time_s)))
        self.velocity[:] = (self.reference-old)/dt if dt > 0 else 0.
        return self.reference.copy(), self.velocity.copy()


def create_floating_task(name, model):
    return FloatingCaptureTask(model, FloatingCaptureConfig(task=name))
