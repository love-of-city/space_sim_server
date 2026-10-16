"""Closed-loop drawer lifting for the delivered SARM 2026-10-12 model.

Only actuator references are produced. The policy never writes physical state,
changes contact geometry, disables the latch, or attaches the drawer to the hand.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics,
    axis_angle_to_matrix,
    rotation_matrix_to_vector,
    quaternion_wxyz_to_matrix,
)


@dataclass(frozen=True)
class DrawerLiftConfig:
    lift_m: float = 0.08
    clearance_m: float = 0.205
    # Calibrated against the delivered handle; expressed in drawer_base coordinates.
    grip_offset_m: tuple[float, float, float] = (0.0, 0.0, -0.005)
    arm_speed_rad_s: float = 0.08
    linear_speed_m_s: float = 0.025
    gripper_speed_rad_s: float = 0.2
    force_limit_n: float = 80.0
    torque_limit_nm: float = 8.0
    position_tolerance_m: float = 0.003
    loaded_position_tolerance_m: float = 0.005
    hold_s: float = 2.0

    def __post_init__(self):
        for name in (
            "lift_m",
            "clearance_m",
            "arm_speed_rad_s",
            "linear_speed_m_s",
            "gripper_speed_rad_s",
            "force_limit_n",
            "torque_limit_nm",
            "position_tolerance_m",
            "loaded_position_tolerance_m",
            "hold_s",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.lift_m > 0.12 or self.arm_speed_rad_s > 0.0873:
            raise ValueError("lift must be <= 0.12 m and arm speed <= 0.0873 rad/s")
        if (
            np.asarray(self.grip_offset_m).shape != (3,)
            or not np.isfinite(self.grip_offset_m).all()
        ):
            raise ValueError("grip_offset_m must contain three finite values")


@dataclass(frozen=True)
class DrawerSample:
    time_s: float
    joints: np.ndarray
    joint_velocities: np.ndarray
    drawer_m: float
    drawer_velocity_m_s: float
    buckle_rad: float
    wrench: np.ndarray
    wrench_valid: bool
    finger_drawer_contacts: tuple[int, int] | None = None


# Re-export retained for existing drawer-policy users.
from simulation.scripted_tasks.motion import solve_pose


class DrawerLiftTask:
    """Measured-state gates around a bounded, smooth Cartesian waypoint plan."""

    stages = (
        "settle",
        "raise",
        "above_drawer",
        "align",
        "open",
        "descend",
        "grip",
        "lift",
        "hold",
    )

    def __init__(self, model_path: Path, config: DrawerLiftConfig | None = None):
        self.config = config or DrawerLiftConfig()
        self.chain = SerialChainKinematics.from_mjcf(
            model_path,
            base_body="cubesat_bus",
            joint_names=tuple(f"joint{i}" for i in range(1, 7)),
            tool_site="sarm_ee",
        )
        root = ET.parse(model_path).getroot()
        bus = root.find('.//body[@name="cubesat_bus"]')
        drawer = bus.find('body[@name="drawer_base"]') if bus is not None else None
        if drawer is None or root.find('.//body[@name="drawer_buckle"]') is None:
            raise ValueError("Task requires the SARM drawer and its original buckle")
        slide = drawer.find('joint[@name="drawer_base_slide"]')
        if slide is None or slide.get("type") != "slide":
            raise ValueError("Task requires drawer_base_slide")
        drawer_rotation = quaternion_wxyz_to_matrix(
            np.fromstring(drawer.get("quat", "1 0 0 0"), sep=" ")
        )
        if not np.allclose(drawer_rotation, np.diag([-1.0, 1.0, -1.0])):
            raise ValueError(
                "Unsupported drawer orientation; recalibrate the grasp pose"
            )
        self.drawer_rotation = drawer_rotation
        self.axis = self.drawer_rotation @ np.fromstring(slide.get("axis"), sep=" ")
        if not np.allclose(self.axis, [0.0, 0.0, 1.0]):
            raise ValueError("Task requires the delivered upward slide axis")
        self.drawer_origin = np.fromstring(drawer.get("pos"), sep=" ")
        self.grip_origin = self.drawer_origin + self.drawer_rotation @ np.asarray(
            self.config.grip_offset_m
        )
        limits = [
            np.fromstring(
                root.find(f'.//joint[@name="joint{i}"]').get("range"), sep=" "
            )
            for i in range(1, 7)
        ]
        self.lower, self.upper = np.asarray(limits).T
        self.stage = "pending"
        self.result = None
        self.reason = ""
        self.events = []
        self.reference = np.zeros(8)
        self.velocity = np.zeros(8)
        self._last_time = None
        self._stable = 0.0
        self._overload = 0.0
        self._path = None
        self._motion_s = 0.0
        self._initial_drawer = 0.0
        self.max_force_n = 0.0
        self.max_torque_nm = 0.0

    def _enter(self, name, sample):
        self.stage = name
        self._entered = sample.time_s
        self._stable = 0.0
        self._motion_s = 0.0
        self._path = None
        self._duration = 0.0
        self.events.append({"stage": name, "sim_time_s": sample.time_s})
        # Preserve the held command across stages; measured servo deflection is
        # not a new joint target (that would create a jump at grasp -> lift).
        p, r = self.chain.forward(self.reference[:6])
        goal, rotation = self._stage_goal(name, p, r)
        self._goal, self._rotation = goal, rotation
        if goal is not None:
            rv = rotation_matrix_to_vector(rotation @ r.T)
            angle = np.linalg.norm(rv)
            seed = self.reference[:6].copy()
            path = [seed]
            for alpha in np.linspace(0.0, 1.0, 81)[1:]:
                rot = (
                    axis_angle_to_matrix(rv / angle, alpha * angle) @ r
                    if angle > 1e-12
                    else r
                )
                seed = solve_pose(
                    self.chain,
                    p + alpha * (goal - p),
                    rot,
                    seed,
                    self.lower,
                    self.upper,
                )
                if np.max(np.abs(seed - path[-1])) > 0.3:
                    raise ValueError(
                        "IK changed branch; refusing a discontinuous joint reference"
                    )
                path.append(seed)
            self._path = np.asarray(path)
            # Quintic time scaling has maximum derivative 1.875. Bound every joint.
            slope = np.max(np.abs(np.diff(self._path, axis=0))) * (len(path) - 1)
            self._duration = max(
                1.0,
                1.875 * slope / self.config.arm_speed_rad_s,
                1.875 * np.linalg.norm(goal - p) / self.config.linear_speed_m_s,
                1.875 * angle / 0.2,
            )
        self._timeout = max(20.0, 2.5 * self._duration + 10.0)

    def _stage_goal(self, name, p, r):
        goal = None
        rotation = r
        if name == "raise":
            goal = p.copy()
            goal[2] = max(p[2], self.hover[2])
        elif name == "above_drawer":
            goal = self.hover
        elif name == "align":
            goal, rotation = self.hover, self.grasp_rotation
        elif name == "descend":
            goal, rotation = self.grasp, self.grasp_rotation
        elif name == "lift":
            goal, rotation = (
                self.grasp + self.axis * self.config.lift_m,
                self.grasp_rotation,
            )
        return goal, rotation

    def _gripper_target(self):
        return {"open": 1.0, "grip": 0.0}.get(self.stage)

    def _loaded_stage(self):
        return self.stage == "lift"

    def _stable_time(self):
        return (
            self.config.hold_s
            if self.stage == "hold"
            else (1.0 if self.stage in ("settle", "grip") else 0.4)
        )

    def fail(self, reason, sample):
        self.result = "failed"
        self.reason = reason
        self.reference[:] = sample.joints
        self.velocity.fill(0.0)
        self.events.append(
            {
                "stage": "failed",
                "during": self.stage,
                "reason": reason,
                "sim_time_s": sample.time_s,
            }
        )

    def step(self, sample: DrawerSample):
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if (
            np.shape(sample.joints) != (8,)
            or np.shape(sample.joint_velocities) != (8,)
            or np.shape(sample.wrench) != (6,)
        ):
            raise ValueError(
                "Task requires eight joint positions/velocities and a six-axis wrench"
            )
        if (
            not math.isfinite(sample.time_s)
            or not np.isfinite(sample.joints).all()
            or not np.isfinite(sample.joint_velocities).all()
            or not np.isfinite(
                [sample.drawer_m, sample.drawer_velocity_m_s, sample.buckle_rad]
            ).all()
            or (sample.wrench_valid and not np.isfinite(sample.wrench).all())
        ):
            raise ValueError("Non-finite task observation")
        if self._last_time is None:
            self._last_time = sample.time_s
            self.reference[:] = sample.joints
            self._initial_drawer = sample.drawer_m
            if abs(sample.drawer_m) > 0.005:
                self.fail("drawer_not_initially_closed", sample)
                return self.reference.copy(), self.velocity.copy()
            self.grasp = self.grip_origin + self.axis * sample.drawer_m
            self.hover = self.grasp + self.axis * self.config.clearance_m
            _, r = self.chain.forward(sample.joints[:6])
            closing = self.drawer_rotation[:, 1].copy()
            if np.dot(closing, r[:, 0]) < 0:
                closing *= -1
            down = -self.axis
            self.grasp_rotation = np.column_stack(
                [closing, np.cross(down, closing), down]
            )
            self._enter("settle", sample)
            return self.reference.copy(), self.velocity.copy()
        dt = sample.time_s - self._last_time
        if dt < -1e-9 or dt > 0.1:
            raise ValueError("Task requires monotonic simulation samples at >=10 Hz")
        if dt <= 0:
            return self.reference.copy(), self.velocity.copy()
        self._last_time = sample.time_s
        old = self.reference.copy()
        force = float(np.linalg.norm(sample.wrench[:3])) if sample.wrench_valid else 0.0
        torque = (
            float(np.linalg.norm(sample.wrench[3:])) if sample.wrench_valid else 0.0
        )
        self.max_force_n = max(self.max_force_n, force)
        self.max_torque_nm = max(self.max_torque_nm, torque)
        self._overload = (
            self._overload + dt
            if (
                force > self.config.force_limit_n
                or torque > self.config.torque_limit_nm
            )
            else 0.0
        )
        if self._overload >= 0.1:
            self.fail("wrist_load_limit", sample)
        elif sample.time_s - self._entered > self._timeout:
            self.fail("stage_timeout", sample)
        elif not sample.wrench_valid and sample.time_s > 0.2:
            self.fail("wrist_sensor_unavailable", sample)
        if self.result is not None:
            return self.reference.copy(), self.velocity.copy()
        if self._path is not None:
            # Do not keep winding the reference forward when the arm is blocked.
            if np.max(np.abs(self.reference[:6] - sample.joints[:6])) < 0.06:
                self._motion_s = min(self._duration, self._motion_s + dt)
            a = self._motion_s / self._duration
            u = (10 * a**3 - 15 * a**4 + 6 * a**5) * (len(self._path) - 1)
            i = min(int(u), len(self._path) - 2)
            self.reference[:6] = self._path[i] + (u - i) * (
                self._path[i + 1] - self._path[i]
            )
        target = self._gripper_target()
        if target is not None:
            self.reference[6] += np.clip(
                target - self.reference[6],
                -self.config.gripper_speed_rad_s * dt,
                self.config.gripper_speed_rad_s * dt,
            )
            self.reference[7] = -self.reference[6]
        self.velocity[:] = (self.reference - old) / dt
        p, r = self.chain.forward(sample.joints[:6])
        reached = np.max(np.abs(sample.joint_velocities[:6])) < 0.015
        if self._goal is not None:
            reached = reached and self._motion_s >= self._duration
            tolerance = (
                self.config.loaded_position_tolerance_m
                if self._loaded_stage()
                else self.config.position_tolerance_m
            )
            reached = reached and np.linalg.norm(p - self._goal) < tolerance
            reached = (
                reached
                and np.linalg.norm(rotation_matrix_to_vector(self._rotation @ r.T))
                < 0.025
            )
        reached = self._stage_reached(sample, reached, p)
        self._stable = self._stable + dt if reached else 0.0
        hold = self._stable_time()
        if self._stable >= hold and self.result is None:
            index = self.stages.index(self.stage)
            if index == len(self.stages) - 1:
                self.result = "success"
                self.velocity.fill(0.0)
                self.events.append(
                    {
                        "stage": "success",
                        "sim_time_s": sample.time_s,
                        "drawer_displacement_m": sample.drawer_m - self._initial_drawer,
                    }
                )
            else:
                try:
                    self._enter(self.stages[index + 1], sample)
                except ValueError as exc:
                    self.fail(f"planning_failed: {exc}", sample)
        return self.reference.copy(), self.velocity.copy()

    def _stage_reached(self, sample, reached, p):
        if self.stage == "open":
            reached = reached and sample.joints[6] > 0.95
        elif self.stage == "grip":
            reached = reached and self.reference[6] < 1e-6 and sample.joints[6] > 0.025
        elif self.stage == "hold":
            reached = (
                reached
                and abs(sample.drawer_m - self._initial_drawer - self.config.lift_m)
                <= 0.005
            )
            reached = reached and sample.joints[6] > 0.025
            reached = reached and abs(sample.drawer_velocity_m_s) < 0.003
            reached = (
                reached
                and np.linalg.norm(p - (self.grip_origin + self.axis * sample.drawer_m))
                < self.config.loaded_position_tolerance_m
            )
            if sample.drawer_m - self._initial_drawer < 0.5 * self.config.lift_m:
                self.fail("drawer_slipped_or_not_lifted", sample)
        return reached
