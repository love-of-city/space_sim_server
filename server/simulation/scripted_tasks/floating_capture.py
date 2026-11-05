"""Task 5: intercept a freely moving team asset using measured simulation state.

The controller writes actuator references only. A successful capture requires
bilateral finger contact and a stable object-to-hand transform, never a weld.
"""
from .floating_assets import FLOATING_TASKS
from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from space_arm_platform.sampling import SCRIPTED_TASK_HZ
from simulation.serial_chain_kinematics import (
    SerialChainKinematics, rotation_matrix_to_vector,
)


FLOATING_BASELINE_DEG = (90., -60., 60., 0., -45., 0.)


@dataclass(frozen=True)
class FloatingCaptureConfig:
    task: str = "floating_bolt_capture"
    arm_speed_rad_s: float = .35
    linear_speed_m_s: float = .04
    angular_speed_rad_s: float = .3
    gripper_speed_rad_s: float = .25
    hold_s: float = 2.
    settle_s: float = 1.
    retract_position_body_m: tuple = (.30, .35, .36)
    retract_speed_m_s: float = .025
    force_limit_n: float = 80.
    torque_limit_nm: float = 8.
    timeout_s: float = 150.

    def __post_init__(self):
        if self.task not in FLOATING_TASKS:
            raise ValueError("Unknown floating capture task")
        for key, value in vars(self).items():
            if key not in ("task", "retract_position_body_m") and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{key} must be finite and positive")

        position = np.asarray(self.retract_position_body_m, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("Retraction position must contain three finite coordinates")
        if self.hold_s < 2. or self.settle_s < 1.:
            raise ValueError("Capture requires at least 2 s before retraction and 1 s at destination")

    def snapshot(self):
        return {**vars(self), "retract_position_body_m": list(self.retract_position_body_m)}

    @property
    def plug_body(self):
        # The shared free-body reader uses this field for plugs and fragments.
        return FLOATING_TASKS[self.task].body


class FloatingCaptureTask:
    version = "floating-capture/8"
    stages = ("open", "pregrasp", "intercept", "grip", "hold", "retract", "settle")

    def __init__(self, model_path, config=None):
        self.model_path = Path(model_path)
        self.config = config or FloatingCaptureConfig()
        self.name = self.config.task
        self.asset = FLOATING_TASKS[self.name]
        from .floating_grasps import grasp_candidates
        self.grasp = grasp_candidates(self.name)[0]
        self.object_instances = []
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
        self._jaw_contact_seen = False
        self._contact_loss = 0.
        self._hold_completed = 0.
        self._retract_start = self._retract_rotation = None

    def grasp_pose(self, sample):
        rotation = sample.plug_rotation_body
        point = sample.plug_position_body_m + rotation @ np.asarray(self.grasp.grip_point)
        x = rotation @ self.grasp.closing_axis
        z = rotation @ self.grasp.approach_axis
        return point, np.column_stack((x, np.cross(z, x), z))

    def metrics(self, sample):
        point, desired = self.grasp_pose(sample)
        p, r = self.chain.forward(sample.joints[:6])
        local = r.T @ (point-p)
        j = self.chain.jacobian(sample.joints[:6])
        target_velocity = sample.plug_velocity_body_m_s + np.cross(
            sample.plug_angular_velocity_body_rad_s,
            sample.plug_rotation_body @ np.asarray(self.grasp.grip_point))
        relative = target_velocity - j[:3] @ sample.joint_velocities[:6]
        return dict(
            task_version=self.version, target_body=self.asset.body, grasp_candidate_id=self.grasp.id,
            feedback_source="authoritative_simulation_state",
            target_position_body_m=sample.plug_position_body_m.tolist(),
            target_velocity_body_m_s=sample.plug_velocity_body_m_s.tolist(),
            target_angular_velocity_body_rad_s=sample.plug_angular_velocity_body_rad_s.tolist(),
            grasp_error_m=float(np.linalg.norm(point-p)),
            orientation_error_rad=float(np.linalg.norm(rotation_matrix_to_vector(desired@r.T))),
            relative_speed_m_s=float(np.linalg.norm(relative)),
            finger_target_contacts=list(sample.finger_plug_contacts or (0, 0)),
            finger_normal_force_n=list(sample.finger_normal_force_n),
            held_seconds=self._stable if self.stage == "hold" else self._hold_completed,
            settled_seconds=self._stable if self.stage == "settle" else 0.,
            measured_tool_position_body_m=p.tolist(), measured_tool_rotation_body=r.tolist(),
            measured_tool_linear_speed_m_s=float(np.linalg.norm(j[:3]@sample.joint_velocities[:6])),
            measured_tool_angular_speed_rad_s=float(np.linalg.norm(j[3:]@sample.joint_velocities[:6])),
            retraction_target_position_body_m=list(self.config.retract_position_body_m),
            retraction_start_position_body_m=None if self._retract_start is None else self._retract_start.tolist(),
            retraction_target_rotation_body=None if self._retract_rotation is None else self._retract_rotation.tolist(),
            retraction_error_m=float(np.linalg.norm(np.asarray(self.config.retract_position_body_m)-p)),
            retraction_orientation_error_rad=0. if self._retract_rotation is None else float(np.linalg.norm(
                rotation_matrix_to_vector(self._retract_rotation@r.T))),
            slip_m=0. if self._held_local is None else float(np.linalg.norm(local-self._held_local)),
            angular_slip_rad=0. if self._held_rotation is None else float(np.linalg.norm(
                rotation_matrix_to_vector((r.T@sample.plug_rotation_body)@self._held_rotation.T))),
            grasp_point_tool_m=local.tolist(),
            unexpected_contacts=getattr(sample, "unexpected_contacts", []),
            target_instance_id=self.asset.body,
            floating_objects=getattr(sample, "floating_objects", []),
            non_target_contacts=getattr(sample, "non_target_contacts", []),
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
        speed = sample.plug_velocity_body_m_s + np.cross(
            sample.plug_angular_velocity_body_rad_s,
            sample.plug_rotation_body @ np.asarray(self.grasp.grip_point))
        if self.stage == "pregrasp":
            offset = -self.grasp.pregrasp_m*desired[:, 2]
            point = point+offset
            speed = speed+np.cross(sample.plug_angular_velocity_body_rad_s, offset)
        self._drive_pose(sample, dt, point, desired, self.config.linear_speed_m_s,
                         speed, sample.plug_angular_velocity_body_rad_s)

    def _drive_pose(self, sample, dt, point, desired, linear_limit, speed=None, spin=None,
                    motion_scale=1.):
        p, r = self.chain.forward(sample.joints[:6])
        translation = (np.zeros(3) if speed is None else speed) + 2.5*(point-p)
        translation *= min(1., linear_limit/max(np.linalg.norm(translation), 1e-12))
        angular = (np.zeros(3) if spin is None else spin) + 2.*rotation_matrix_to_vector(desired@r.T)
        angular *= min(1., self.config.angular_speed_rad_s/max(np.linalg.norm(angular), 1e-12))
        translation *= motion_scale
        angular *= motion_scale
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

    def _drive_gripper(self, sample, dt, metrics):
        """Bound servo compression and maintain a light bilateral preload."""
        measured = float(sample.joints[6])
        forces = np.asarray(sample.finger_normal_force_n)
        contact = any(sample.finger_plug_contacts)
        if self.stage == "grip" and contact:
            self._jaw_contact_seen = True
        if self.stage in ("open", "pregrasp", "intercept"):
            goal, speed = 1., self.config.gripper_speed_rad_s
        elif self._hold_joints is not None:
            # Position servos remain the only actuators. Weak contact can be
            # preloaded slowly; excessive one-sided force must open the jaw.
            rate = (.02 if max(forces) > 12. else
                    float(np.clip(.02*(min(forces)-5.), -.02, .02)))
            goal = self.reference[6] + dt*rate
            speed = .02
        elif self._jaw_contact_seen:
            # Do not keep squeezing a target pushed off the grasp frame.
            # The Cartesian tracker continues to align before closing again.
            goal = (measured+.002 if max(forces) > 12. else
                    measured if metrics["grasp_error_m"] > .0015 else 0.)
            speed = .02
        else:
            goal, speed = 0., min(.06, self.config.gripper_speed_rad_s)
        command = self.reference[6] + np.clip(goal-self.reference[6], -dt*speed, dt*speed)
        if self._jaw_contact_seen or self._hold_joints is not None:
            # The original 0.002 rad preload is now a per-step error bound,
            # preventing a blocked jaw from accumulating a full-close target.
            command = np.clip(command, measured-.002, measured+.002)
        self.reference[6] = np.clip(command, 0., 1.)
        self.reference[7] = -self.reference[6]
        if self._hold_joints is not None:
            self._hold_opening = float(self.reference[6])

    def step(self, sample):
        if not math.isfinite(sample.time_s) or (self._last_time is not None and sample.time_s < self._last_time):
            raise ValueError("Task time must be finite and monotonic")
        dt = 0. if self._last_time is None else sample.time_s-self._last_time
        invalid_timing = (self._last_time is not None
                          and (dt <= 0. or dt > 1./SCRIPTED_TASK_HZ + 1e-9))
        self._last_time = sample.time_s
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if invalid_timing:
            # Wall-clock backpressure pauses simulation time. Missing physics
            # observations must never count toward an uninterrupted grasp.
            self.fail("invalid_observation_timing", sample)
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
        elif getattr(sample, "non_target_contacts", []):
            self.fail("non_target_contact", sample)
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
        if self.stage in ("pregrasp", "intercept") or (self.stage == "grip" and self._hold_joints is None):
            self._track(sample, dt)
        elif self.stage in ("retract", "settle"):
            # Start transport from rest without a Cartesian velocity step.
            # Smoothstep reaches the unchanged speed envelope after one second.
            phase = min(1., max(0., sample.time_s-self._entered)) if self.stage == "retract" else 1.
            ramp = phase*phase*(3.-2.*phase)
            self._drive_pose(sample, dt, np.asarray(self.config.retract_position_body_m),
                             self._retract_rotation, self.config.retract_speed_m_s,
                             motion_scale=ramp)
        elif self.stage in ("grip", "hold") and self._hold_joints is not None:
            self.reference[:6] = self._hold_joints
        m = self.metrics(sample)
        self._drive_gripper(sample, dt, m)
        bilateral = all(count > 0 for count in m["finger_target_contacts"]) and min(m["finger_normal_force_n"]) >= .02
        if self.stage == "open" and sample.joints[6] >= .95:
            self._enter("pregrasp", sample)
        elif self.stage == "pregrasp":
            point, desired = self.grasp_pose(sample)
            prepoint = point-self.grasp.pregrasp_m*desired[:, 2]
            tcp, _ = self.chain.forward(sample.joints[:6])
            if np.linalg.norm(prepoint-tcp) < .002 and m["orientation_error_rad"] < .025:
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
                self._hold_completed = self._stable
                self._retract_start, self._retract_rotation = self.chain.forward(sample.joints[:6])
                self._retract_start, self._retract_rotation = self._retract_start.copy(), self._retract_rotation.copy()
                if np.linalg.norm(self._retract_start-np.asarray(self.config.retract_position_body_m)) < .05:
                    self.fail("retraction_distance_too_short", sample)
                else:
                    self._enter("retract", sample)
        elif self.stage in ("retract", "settle"):
            retained = bilateral and m["slip_m"] < .003 and m["angular_slip_rad"] < .05 and m["relative_speed_m_s"] < .005
            if not retained:
                self.fail("grasp_lost_during_retraction", sample)
            else:
                arrived = (m["retraction_error_m"] < .003 and m["retraction_orientation_error_rad"] < .025
                           and m["measured_tool_linear_speed_m_s"] < .003
                           and m["measured_tool_angular_speed_rad_s"] < .02)
                if self.stage == "retract" and arrived and m["retraction_error_m"] < .001:
                    # Keep the same Cartesian servo through arrival; resetting
                    # its reference to measured joints would introduce a step.
                    self._enter("settle", sample)
                elif self.stage == "settle":
                    if not arrived:
                        self.fail("retraction_destination_lost", sample)
                    else:
                        self._stable += dt
                    if self.result is None and self._stable >= self.config.settle_s:
                        if np.linalg.norm(np.asarray(m["measured_tool_position_body_m"])-self._retract_start) < .05:
                            self.fail("retraction_distance_too_short", sample)
                        else:
                            self.result, self.reason = "success", "retained_object_retracted_and_settled"
                            self.events.append(dict(stage="success", sim_time_s=float(sample.time_s)))
        self.velocity[:] = (self.reference-old)/dt if dt > 0 else 0.
        return self.reference.copy(), self.velocity.copy()


def floating_controller_config(name, scene):
    options = scene.get("floating_controller", {})
    allowed = {"linear_speed_m_s", "angular_speed_rad_s", "arm_speed_rad_s"}
    if set(options) - allowed:
        raise ValueError("floating_controller accepts only tracking linear, angular and joint speeds")
    retraction = scene.get("floating_retraction", {})
    if set(retraction) - {"position_body_m", "speed_m_s"}:
        raise ValueError("floating_retraction accepts only position_body_m and speed_m_s")
    mapped = {"retract_"+key: value for key, value in retraction.items()}
    return FloatingCaptureConfig(task=name, **options, **mapped)


def create_floating_task(name, model, scene=None):
    task = FloatingCaptureTask(model, floating_controller_config(name, scene or {}))
    if scene is not None:
        from .floating_grasps import selected_grasp
        task.grasp = selected_grasp(name, scene)
        task.object_instances = scene.get("collection_config", {}).get("floating_objects", {}).get("instances", [])
    return task
