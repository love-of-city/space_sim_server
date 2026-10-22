"""Tracked free-body pick/transfer/insert tasks; actuator references only."""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
from simulation.serial_chain_kinematics import (
    SerialChainKinematics,
    quaternion_wxyz_to_matrix,
    rotation_matrix_to_vector,
    axis_angle_to_matrix,
)
from simulation.scripted_tasks.motion import solve_pose
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES


def fixed_site_pose(root, name, base="cubesat_bus"):
    parents = {child: parent for parent in root.iter() for child in parent}
    site = root.find(f'.//site[@name="{name}"]')
    if site is None:
        raise ValueError(f"Missing target site {name}")
    position = np.fromstring(site.get("pos", "0 0 0"), sep=" ")
    rotation = quaternion_wxyz_to_matrix(
        np.fromstring(site.get("quat", "1 0 0 0"), sep=" ")
    )
    body = parents[site]
    while body.get("name") != base:
        if (
            body.tag != "body"
            or body.find("joint") is not None
            or body.find("freejoint") is not None
        ):
            raise ValueError("Target must be fixed relative to the spacecraft")
        rb = quaternion_wxyz_to_matrix(
            np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
        )
        position = np.fromstring(body.get("pos", "0 0 0"), sep=" ") + rb @ position
        rotation = rb @ rotation
        body = parents[body]
    return position, rotation


@dataclass(frozen=True)
class PlugInsertConfig:
    target_site: str | None = None
    plug_body: str = "cylindrical_plug"
    grasp_offset_m: tuple[float, float, float] = (0.0, 0.0, -0.003)
    transport_height_m: float = 0.245
    arm_speed_rad_s: float = 0.08
    linear_speed_m_s: float = 0.025
    retract_speed_m_s: float = 0.01
    insert_speed_m_s: float = 0.002
    gripper_speed_rad_s: float = 0.2
    release_speed_rad_s: float = 0.05
    release_open_rad: float = 0.65
    force_limit_n: float = 40.0
    torque_limit_nm: float = 4.0
    shaft_tip_local_m: float | None = None
    flange_local_m: float = 0.025
    radial_tolerance_m: float = 0.0004
    seat_tolerance_m: float = 0.001
    hold_s: float = 2.0
    grasp_relative_speed_m_s: float = 0.015
    tracking_speed_rad_s: float = 0.20

    def __post_init__(self):
        profile = next(
            (p for p in PLUG_PROFILES.values() if p.body == self.plug_body), None
        )
        if profile is None:
            raise ValueError("Unknown plug body")
        if self.target_site is None:
            object.__setattr__(self, "target_site", profile.target_site)
        if self.shaft_tip_local_m is None:
            object.__setattr__(self, "shaft_tip_local_m", profile.tip_m)
        if self.target_site not in (
            profile.prefix + "_insert_40p1",
            profile.target_site,
        ):
            raise ValueError(
                "Target must be this plug's forward assembly hole, not the pickup location"
            )
        for name, value in vars(self).items():
            if isinstance(value, (float, int)) and (
                not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if self.arm_speed_rad_s > 0.0873 or self.insert_speed_m_s > 0.005:
            raise ValueError("Configured motion exceeds the validated planning limits")
        if (
            np.asarray(self.grasp_offset_m).shape != (3,)
            or not np.isfinite(self.grasp_offset_m).all()
        ):
            raise ValueError("Invalid grasp offset")


@dataclass(frozen=True)
class PlugSample:
    time_s: float
    joints: np.ndarray
    joint_velocities: np.ndarray
    plug_position_body_m: np.ndarray
    plug_rotation_body: np.ndarray
    plug_velocity_body_m_s: np.ndarray
    plug_angular_velocity_body_rad_s: np.ndarray
    wrench: np.ndarray
    wrench_valid: bool
    finger_plug_contacts: tuple[int, int] | None = None


class PlugInsertTask:
    name = "plug_insert"
    version = "plug-insert/2"
    stages = (
        "settle",
        "raise",
        "above_pickup",
        "align",
        "open",
        "descend",
        "grip",
        "lift",
        "transfer",
        "above_hole",
        "align_hole",
        "insert",
        "seat",
        "release",
        "retract",
        "verify",
    )

    def __init__(self, model_path: Path, config=None):
        self.config = config or PlugInsertConfig()
        self.profile = next(
            p for p in PLUG_PROFILES.values() if p.body == self.config.plug_body
        )
        self.name = self.profile.task
        root = ET.parse(model_path).getroot()
        self.chain = SerialChainKinematics.from_mjcf(
            model_path,
            base_body="cubesat_bus",
            joint_names=tuple(f"joint{i}" for i in range(1, 7)),
            tool_site="sarm_ee",
        )
        self.hole, hole_r = fixed_site_pose(root, self.config.target_site)
        self.pickup, _ = fixed_site_pose(root, self.profile.prefix + "_pickup")
        if np.linalg.norm(self.hole[:2] - self.pickup[:2]) < 0.05 or not np.allclose(
            hole_r, np.eye(3)
        ):
            raise ValueError(
                "The assembly target must be a separate upward-facing hole"
            )
        self.lower, self.upper = np.array(
            [
                np.fromstring(
                    root.find(f'.//joint[@name="joint{i}"]').get("range"), sep=" "
                )
                for i in range(1, 7)
            ]
        ).T
        self.stage, self.result, self.reason = "pending", None, ""
        self.events = []
        self.reference, self.velocity = np.zeros(8), np.zeros(8)
        self._last_time = None
        self._stable = self._overload = self._motion_s = 0.0
        self.max_force_n = self.max_torque_nm = 0.0
        self._path = None
        self._grasp_transform = None
        self._insert_start = None
        self._initial_position = None
        self._completed_at = None
        self._ever_lifted = False
        self._closed_reference = 0.0
        self._bias_position = np.zeros(3)
        self._approach_rotation = None
        self._pickup_latched = False

    def metrics(self, sample):
        axis = sample.plug_rotation_body[:, 2]
        tip = sample.plug_position_body_m + axis * self.config.shaft_tip_local_m
        flange = sample.plug_position_body_m + axis * self.config.flange_local_m
        plane = (
            sample.plug_position_body_m
            + axis * ((self.hole[2] - sample.plug_position_body_m[2]) / axis[2])
            if abs(axis[2]) > 0.1
            else np.full(3, 1e3)
        )
        return dict(
            plug_position_body_m=sample.plug_position_body_m.tolist(),
            plug_rotation_body=sample.plug_rotation_body.tolist(),
            plug_velocity_body_m_s=sample.plug_velocity_body_m_s.tolist(),
            plug_angular_velocity_body_rad_s=sample.plug_angular_velocity_body_rad_s.tolist(),
            task_name=self.name,
            task_version=self.version,
            plug_body=self.config.plug_body,
            finger_plug_contacts=sample.finger_plug_contacts,
            orientation_error_rad=float(
                np.linalg.norm(
                    rotation_matrix_to_vector(
                        np.diag([1.0, -1.0, -1.0]) @ sample.plug_rotation_body.T
                    )
                )
            ),
            target_site=self.config.target_site,
            target_position_body_m=self.hole.tolist(),
            radial_error_m=float(np.linalg.norm(plane[:2] - self.hole[:2])),
            axis_error_rad=float(np.arccos(np.clip(-axis[2], -1, 1))),
            insertion_depth_m=float(self.hole[2] - tip[2]),
            seat_gap_m=float(flange[2] - self.hole[2]),
            ever_lifted=bool(self._ever_lifted),
        )

    def fail(self, reason, sample):
        self.result, self.reason = "failed", reason
        self.reference[:], self.velocity[:] = sample.joints, 0.0
        self.events.append(
            dict(
                stage="failed",
                reason=reason,
                sim_time_s=sample.time_s,
                **self.metrics(sample),
            )
        )
        self._completed_at = sample.time_s

    def _plan(self, goal, rotation):
        p, r = self.chain.forward(self.reference[:6])
        rv = rotation_matrix_to_vector(rotation @ r.T)
        angle = np.linalg.norm(rv)
        path = [self.reference[:6].copy()]
        for a in np.linspace(0, 1, 81)[1:]:
            ra = axis_angle_to_matrix(rv / angle, a * angle) @ r if angle > 1e-12 else r
            q = solve_pose(
                self.chain, p + a * (goal - p), ra, path[-1], self.lower, self.upper
            )
            if np.max(np.abs(q - path[-1])) > 0.3:
                raise ValueError("IK branch discontinuity")
            path.append(q)
        self._path = np.array(path)
        slope = np.max(np.abs(np.diff(self._path, axis=0))) * 80
        self._duration = max(
            1.0,
            1.875 * slope / self.config.arm_speed_rad_s,
            1.875
            * np.linalg.norm(goal - p)
            / (
                self.config.retract_speed_m_s
                if self.stage == "retract"
                else self.config.linear_speed_m_s
            ),
            1.875 * angle / 0.2,
        )
        self._goal, self._rotation = goal, rotation

    def _tool_for_plug(self, sample, desired_position, desired_rotation):
        p, r = self.chain.forward(sample.joints[:6])
        relative_r = r.T @ sample.plug_rotation_body
        relative_p = r.T @ (sample.plug_position_body_m - p)
        target_r = desired_rotation @ relative_r.T
        return desired_position - target_r @ relative_p, target_r

    def _enter(self, stage, sample):
        self.stage = stage
        if self._approach_rotation is not None:
            self.grasp, self.grasp_rotation = self._tracked_grasp(sample)
        self._pickup_anchor = (self.grasp.copy(), self.grasp_rotation.copy())
        self._pickup_hold = self.reference[:6].copy()
        self._pickup_latched = False
        self._entered = sample.time_s
        self._stable = 0.0
        self._motion_s = 0.0
        self._path = None
        self._duration = 0.0
        self.events.append(
            dict(stage=stage, sim_time_s=sample.time_s, **self.metrics(sample))
        )
        p, r = self.chain.forward(self.reference[:6])
        goal = None
        self._goal = None
        rotation = r
        if stage == "raise":
            goal = p.copy()
            goal[2] = max(p[2], self.config.transport_height_m)
        elif stage == "above_pickup":
            goal = self.grasp.copy()
            goal[2] = self.config.transport_height_m
        elif stage == "align":
            goal = p.copy()
            rotation = self.grasp_rotation
        elif stage == "descend":
            goal = self.grasp.copy()
            rotation = self.grasp_rotation
        elif stage == "lift":
            cp, cr = self.chain.forward(sample.joints[:6])
            self._grasp_transform = (
                cr.T @ (sample.plug_position_body_m - cp),
                cr.T @ sample.plug_rotation_body,
            )
            goal = p.copy()
            goal[2] = self.config.transport_height_m
        elif stage == "transfer":
            desired = np.array([*self.hole[:2], self.config.transport_height_m])
            goal, rotation = self._tool_for_plug(sample, desired, self.plug_rotation)
        elif stage == "above_hole":
            desired = np.array(
                [*self.hole[:2], self.hole[2] + self.config.shaft_tip_local_m + 0.012]
            )
            goal, rotation = self._tool_for_plug(sample, desired, self.plug_rotation)
        elif stage in ("align_hole", "insert", "seat", "release"):
            self._plug_goal = np.array(
                [*self.hole[:2], self.hole[2] + self.config.shaft_tip_local_m + 0.012]
            )
            if stage in ("insert", "seat", "release"):
                self._plug_goal[2] = sample.plug_position_body_m[2]
            self._seat_z = self.hole[2] + self.config.flange_local_m
            if stage == "release":
                self._plug_goal[2] = self._seat_z
        elif stage == "retract":
            # Withdraw between the open fingers, along their tangential direction.
            # Vertical arm recoil otherwise pulls the free spacecraft away from the plug.
            direction = r[:, 1].copy()
            direction[2] = 0.0
            direction /= np.linalg.norm(direction)
            if direction[0] < 0:
                direction = -direction
            goal = p + 0.12 * direction
        if stage == "lift" and self.profile.feedback_extraction:
            # Recenter and untwist the keyed shaft in its pickup guide before
            # raising it. A fixed wrist trajectory can jam a tilted free part.
            self._plug_goal = np.array(
                [*self.pickup[:2], sample.plug_position_body_m[2]]
            )
            goal = None
        if goal is not None:
            self._plan(goal, rotation)
        self._nominal_goal = None if self._goal is None else self._goal.copy()
        self._nominal_rotation = (
            self._rotation.copy() if hasattr(self, "_rotation") else r.copy()
        )
        self._timeout = max(30.0, 2.5 * self._duration + 15.0)
        if stage in ("insert", "verify") or (
            stage == "lift" and self.profile.feedback_extraction
        ):
            self._timeout = 90.0

    def _next(self, sample):
        self._enter(self.stages[self.stages.index(self.stage) + 1], sample)

    def _tracked_grasp(self, sample):
        # These heads can be grasped from above while their shafts turn. Keep a
        # stable jaw heading during approach; align keyed shafts after lifting.
        return (
            sample.plug_position_body_m
            + sample.plug_rotation_body @ np.asarray(self.config.grasp_offset_m),
            self._approach_rotation.copy(),
        )

    def _follow_pickup(self, sample, dt, previous):
        """Follow measured part drift without writing the physical state."""
        if self.stage not in ("above_pickup", "align", "open", "descend", "grip"):
            return
        if (
            self.stage == "grip"
            and sample.finger_plug_contacts
            and all(sample.finger_plug_contacts)
        ):
            self._pickup_latched = True
        if self._pickup_latched:
            self.reference[:6] = previous[:6]
            return
        nominal = (
            self.reference[:6].copy() if self._path is not None else self._pickup_hold
        )
        p, r = self.chain.forward(nominal)
        origin, rotation = self._pickup_anchor
        grasp, grasp_rotation = self._tracked_grasp(sample)
        offset = grasp - origin
        delta = (
            grasp_rotation @ rotation.T if self.stage != "above_pickup" else np.eye(3)
        )
        target = p + offset
        desired = delta @ r
        q = solve_pose(self.chain, target, desired, nominal, self.lower, self.upper)
        delta_q = q - previous[:6]
        max_step = self.config.tracking_speed_rad_s * dt
        delta_q *= min(1.0, max_step / max(float(np.max(np.abs(delta_q))), 1e-12))
        self.reference[:6] = previous[:6] + delta_q
        if self._nominal_goal is not None:
            self._goal = self._nominal_goal + offset
            self._rotation = delta @ self._nominal_rotation

    def _grasp_relative_speed(self, sample):
        grasp_offset = sample.plug_rotation_body @ np.asarray(
            self.config.grasp_offset_m
        )
        target_velocity = sample.plug_velocity_body_m_s + np.cross(
            sample.plug_angular_velocity_body_rad_s, grasp_offset
        )
        tool_velocity = (
            self.chain.jacobian(sample.joints[:6])[:3] @ sample.joint_velocities[:6]
        )
        return float(np.linalg.norm(tool_velocity - target_velocity))

    def _part_feedback(self, sample, dt):
        p, r = self.chain.forward(sample.joints[:6])
        target, rotation = self._tool_for_plug(
            sample, self._plug_goal, self.plug_rotation
        )
        error = np.r_[target - p, 0.2 * rotation_matrix_to_vector(rotation @ r.T)]
        jac = self.chain.jacobian(sample.joints[:6])
        jac[3:] *= 0.2
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(6), 3.0 * error)
        dq *= min(1.0, self.config.arm_speed_rad_s / max(np.max(np.abs(dq)), 1e-12))
        self.reference[:6] = np.clip(
            self.reference[:6] + dt * dq, self.lower, self.upper
        )

    def step(self, sample: PlugSample):
        if self._last_time is not None and sample.time_s < self._last_time:
            raise ValueError("Task time must be monotonic")
        dt = 0.0 if self._last_time is None else sample.time_s - self._last_time
        self._last_time = sample.time_s
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if self.stage == "pending":
            self.reference[:] = sample.joints
            self._initial_position = sample.plug_position_body_m.copy()
            self.plug_rotation = np.diag([1.0, -1.0, -1.0])
            self.grasp = (
                sample.plug_position_body_m
                + sample.plug_rotation_body @ np.asarray(self.config.grasp_offset_m)
            )
            # Closing axis across the head; approach along spacecraft -Z.
            _, current_r = self.chain.forward(sample.joints[:6])
            closing = np.array([1.0, 0.0, 0.0])
            if closing @ current_r[:, 0] < 0:
                closing = -closing
            approach = np.array([0.0, 0.0, -1.0])
            self.grasp_rotation = np.column_stack(
                [closing, np.cross(approach, closing), approach]
            )
            self._approach_rotation = self.grasp_rotation.copy()
            if np.linalg.norm(sample.plug_position_body_m[:2] - self.pickup[:2]) > 0.05:
                self.fail("plug_not_at_pickup", sample)
            else:
                self._enter("settle", sample)
            return self.reference.copy(), self.velocity.copy()
        old = self.reference.copy()
        force = np.linalg.norm(sample.wrench[:3])
        torque = np.linalg.norm(sample.wrench[3:])
        self.max_force_n = max(self.max_force_n, float(force))
        self.max_torque_nm = max(self.max_torque_nm, float(torque))
        self._overload = (
            self._overload + dt
            if force > self.config.force_limit_n or torque > self.config.torque_limit_nm
            else 0.0
        )
        if self._overload >= 0.15:
            self.fail("wrist_load_limit", sample)
        elif not sample.wrench_valid and sample.time_s > 0.2:
            self.fail("wrist_sensor_unavailable", sample)
        elif sample.time_s - self._entered > self._timeout:
            self.fail("stage_timeout", sample)
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if self._path is not None:
            if np.max(np.abs(self.reference[:6] - sample.joints[:6])) < 0.06:
                self._motion_s = min(self._duration, self._motion_s + dt)
            a = self._motion_s / self._duration
            u = (10 * a**3 - 15 * a**4 + 6 * a**5) * 80
            i = min(int(u), 79)
            self.reference[:6] = self._path[i] + (u - i) * (
                self._path[i + 1] - self._path[i]
            )
        try:
            self._follow_pickup(sample, dt, old)
        except ValueError as exc:
            self.fail(f"tracking_ik_failed: {exc}", sample)
            return self.reference.copy(), np.zeros(8)
        if self.stage in ("open", "grip", "release"):
            target = (
                self.config.release_open_rad
                if self.stage == "release"
                else (1.0 if self.stage == "open" else self._closed_reference)
            )
            speed = (
                self.config.release_speed_rad_s
                if self.stage == "release"
                else self.config.gripper_speed_rad_s
            )
            self.reference[6] += np.clip(
                target - self.reference[6],
                -speed * dt,
                speed * dt,
            )
            self.reference[7] = -self.reference[6]
        metrics = self.metrics(sample)
        if self.stage == "lift" and self.profile.feedback_extraction:
            if (
                np.linalg.norm(sample.plug_position_body_m[:2] - self.pickup[:2])
                < 0.0015
                and metrics["orientation_error_rad"] < 0.02
            ):
                self._plug_goal[2] = min(
                    self.config.transport_height_m, self._plug_goal[2] + 0.012 * dt
                )
            self._part_feedback(sample, dt)
        if self.stage in ("align_hole", "insert", "seat"):
            if self.stage == "insert":
                self._plug_goal[2] = max(
                    self._seat_z, self._plug_goal[2] - self.config.insert_speed_m_s * dt
                )
            if self.stage == "seat":
                self._plug_goal[2] = self._seat_z
            self._part_feedback(sample, dt)
        if self.stage == "release" and sample.joints[6] < 0.40:
            self._part_feedback(sample, dt)
        if self.stage in ("transfer", "above_hole", "align_hole", "insert", "seat"):
            p, r = self.chain.forward(sample.joints[:6])
            expected = p + r @ self._grasp_transform[0]
            if np.linalg.norm(expected - sample.plug_position_body_m) > 0.015:
                self.fail("plug_slipped_from_gripper", sample)
        elapsed = sample.time_s - self._entered
        p, r = self.chain.forward(sample.joints[:6])
        if self.stage == "settle":
            ready = elapsed >= 1.0
        elif self._path is not None:
            ready = (
                self._motion_s >= self._duration
                and np.linalg.norm(p - self._goal) < 0.004
                and np.linalg.norm(rotation_matrix_to_vector(self._rotation @ r.T))
                < 0.035
            )
            if self.stage == "descend":
                ready = (
                    ready
                    and self._grasp_relative_speed(sample)
                    < self.config.grasp_relative_speed_m_s
                )
            if self.stage == "lift" and ready:
                self._ever_lifted = (
                    sample.plug_position_body_m[2] - self._initial_position[2] > 0.12
                )
                if not self._ever_lifted:
                    self.fail("plug_not_lifted", sample)
                    ready = False
        elif self.stage == "lift" and self.profile.feedback_extraction:
            ready = (
                self._plug_goal[2] >= self.config.transport_height_m
                and np.linalg.norm(sample.plug_position_body_m - self._plug_goal)
                < 0.004
            )
            if ready:
                self._ever_lifted = bool(
                    sample.plug_position_body_m[2] - self._initial_position[2] > 0.12
                )
                ready = self._ever_lifted
        elif self.stage == "open":
            ready = (
                self.reference[6] >= 0.999 and sample.joints[6] > 0.95 and elapsed > 5.0
            )
        elif self.stage == "release":
            ready = (
                self.reference[6] >= self.config.release_open_rad - 1e-6
                and sample.joints[6] > self.config.release_open_rad - 0.02
            )
            self._stable = self._stable + dt if ready else 0.0
            ready = self._stable >= 1.0
        elif self.stage == "grip":
            ready = (
                elapsed > 6.0 and self.reference[6] < 0.001 and sample.joints[6] > 0.025
            )
            if sample.finger_plug_contacts is not None:
                ready = ready and all(sample.finger_plug_contacts)
            if elapsed > 8.0 and sample.joints[6] < 0.025:
                self.fail("gripper_closed_without_object", sample)
        elif self.stage == "align_hole":
            ready = (
                np.linalg.norm(sample.plug_position_body_m - self._plug_goal) < 0.00015
                and metrics["axis_error_rad"] < 0.002
                and (not self.profile.keyed or metrics["orientation_error_rad"] < 0.004)
            )
            self._stable = self._stable + dt if ready else 0.0
            ready = self._stable >= 0.5
        elif self.stage in ("insert", "seat", "verify"):
            ready = (
                metrics["radial_error_m"] < self.config.radial_tolerance_m
                and metrics["axis_error_rad"] < 0.01
                and abs(metrics["seat_gap_m"]) < self.config.seat_tolerance_m
                and metrics["insertion_depth_m"]
                > self.config.shaft_tip_local_m - self.config.flange_local_m - 0.002
                and (not self.profile.keyed or metrics["orientation_error_rad"] < 0.01)
            )
            # For the keyed guide, the held part can have tiny contact oscillations
            # despite continuous geometric seating. Release only after seating;
            # final acceptance still requires the original low measured velocities.
            if self.stage == "verify" or self.profile.prefix != "guide":
                ready = ready and np.linalg.norm(sample.plug_velocity_body_m_s) < 0.002
                ready = (
                    ready
                    and np.linalg.norm(sample.plug_angular_velocity_body_rad_s) < 0.03
                )
            if self.stage == "insert":
                ready = ready and self._plug_goal[2] <= self._seat_z + 1e-6
            if self.stage == "verify":
                ready = (
                    ready
                    and self._ever_lifted
                    and sample.joints[6] > self.config.release_open_rad - 0.03
                    and np.linalg.norm(p[:2] - self.hole[:2]) > 0.09
                )
            self._stable = self._stable + dt if ready else 0.0
            ready = self._stable >= (
                self.config.hold_s if self.stage == "verify" else 1.0
            )
        else:
            ready = False
        if self.result is None and ready:
            if self.stage == "verify":
                self.result = "success"
                self._completed_at = sample.time_s
                self.events.append(
                    dict(
                        stage="success",
                        sim_time_s=sample.time_s,
                        **self.metrics(sample),
                    )
                )
            else:
                try:
                    self._next(sample)
                except ValueError as exc:
                    self.fail(f"planning_failed: {exc}", sample)
        self.velocity = (
            (self.reference - old) / dt
            if dt > 0 and self.result != "failed"
            else np.zeros(8)
        )
        return self.reference.copy(), self.velocity.copy()
