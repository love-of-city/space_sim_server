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
from simulation.scripted_tasks.motion_timing import time_joint_path
from simulation.scripted_tasks.approach_path import ApproachPosePath
from simulation.scripted_tasks.carry_path import CarryPosePath
from simulation.scripted_tasks.pose_tracking import PoseTargetFilter
from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
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
    grasp_offset_m: tuple[float, float, float] | None = None
    transport_height_m: float = 0.245
    arm_speed_rad_s: float | None = None
    linear_speed_m_s: float | None = None
    retract_speed_m_s: float | None = None
    insert_speed_m_s: float | None = None
    gripper_speed_rad_s: float = 0.2
    release_speed_rad_s: float | None = None
    release_open_rad: float = 0.65
    force_limit_n: float = 40.0
    torque_limit_nm: float = 4.0
    overload_stop_enabled: bool = False
    shaft_tip_local_m: float | None = None
    flange_local_m: float = 0.025
    radial_tolerance_m: float = 0.0004
    seat_tolerance_m: float = 0.001
    hold_s: float = 2.0
    grasp_relative_speed_m_s: float = 0.015
    tracking_speed_rad_s: float | None = None
    pregrasp_distance_m: float = 0.045
    grasp_approach_speed_m_s: float | None = None
    extraction_speed_m_s: float = 0.025
    carry_speed_m_s: float = 0.05
    carry_acceleration_m_s2: float = 0.15
    carry_jerk_m_s3: float = 0.8

    def __post_init__(self):
        profile = next(
            (p for p in PLUG_PROFILES.values() if p.body == self.plug_body), None
        )
        if profile is None:
            raise ValueError("Unknown plug body")
        if self.grasp_offset_m is None:
            object.__setattr__(self, "grasp_offset_m", profile.grasp_offset_m)
        drawer = DrawerCycleConfig()
        coordinated = profile.grasp_closing_axis is not None
        for name, legacy, aligned in (
            ("insert_speed_m_s", 0.002, 0.025),
            ("release_speed_rad_s", 0.05, 0.2),
            ("arm_speed_rad_s", 0.08, drawer.arm_speed_rad_s),
            ("linear_speed_m_s", 0.025, drawer.linear_speed_m_s),
            ("retract_speed_m_s", 0.01, drawer.linear_speed_m_s),
            ("tracking_speed_rad_s", 0.20, drawer.arm_speed_rad_s),
            ("grasp_approach_speed_m_s", 0.008, drawer.approach_speed_m_s),
        ):
            if getattr(self, name) is None:
                object.__setattr__(self, name, aligned if coordinated else legacy)
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
        if not isinstance(self.overload_stop_enabled, bool):
            raise ValueError("overload_stop_enabled must be boolean")
        for name, value in vars(self).items():
            if name == "overload_stop_enabled":
                continue
            if isinstance(value, (float, int)) and (
                isinstance(value, bool) or not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (self.arm_speed_rad_s > (drawer.max_arm_speed_rad_s if coordinated else 0.0873)
                or (coordinated and self.linear_speed_m_s > drawer.linear_speed_m_s)
                or self.insert_speed_m_s > (0.025 if coordinated else 0.005)):
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
    native_qpos: np.ndarray | None = None


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
    legacy_latch_version = "plug-insert/3-latch-grasp"
    legacy_latch_stages = (
        "settle", "raise", "above_pickup", "approach", "descend", "grip",
        "extract", "lift", "transfer", "above_hole", "align_hole",
        "insert", "seat", "release", "retract", "verify",
    )

    coordinated_version = "plug-insert/4-coordinated-approach"
    coordinated_stages = (
        "settle", "approach", "descend", "grip", "extract", "lift", "transfer",
        "above_hole", "align_hole", "insert", "seat", "release", "retract", "verify",
    )

    smooth_tracking_version = "plug-insert/5-smooth-tracking"
    gentle_carry_version = "plug-insert/6-gentle-carry"
    continuous_carry_version = "plug-insert/7-continuous-carry"
    continuous_contact_version = "plug-insert/8-continuous-contact"
    overlapped_release_version = "plug-insert/9-overlapped-release"
    shared_motion_version = "plug-insert/10-shared-continuous-motion"
    latch_version = "plug-insert/11-all-latch-aligned"
    gentle_release_version = "plug-insert/12-round-plug-release"
    cylindrical_clearance_version = "plug-insert/12-cylindrical-clearance"
    integrated_cylindrical_version = "plug-insert/13-cylindrical-clearance-gentle-release"
    continuous_carry_stages = tuple(stage for stage in coordinated_stages if stage != "descend")
    latch_stages = tuple(stage for stage in continuous_carry_stages if stage not in ("align_hole", "seat"))

    @classmethod
    def task_version(cls, task_name):
        from .plug_profiles import plug_profile
        profile = plug_profile(task_name)
        if profile.prefix == "cylindrical":
            return cls.integrated_cylindrical_version
        return cls.gentle_release_version if profile.grasp_closing_axis else cls.version

    @classmethod
    def recorded_stages(cls, version):
        if version in (cls.integrated_cylindrical_version, cls.cylindrical_clearance_version, cls.gentle_release_version, cls.latch_version, cls.shared_motion_version, cls.overlapped_release_version, cls.continuous_contact_version):
            return cls.latch_stages
        if version in (cls.continuous_carry_version, cls.gentle_carry_version, cls.smooth_tracking_version):
            return cls.continuous_carry_stages
        if version == cls.coordinated_version:
            return cls.coordinated_stages
        if version == cls.legacy_latch_version:
            return cls.legacy_latch_stages
        if version in (None, cls.version):
            return cls.stages
        raise ValueError(f"Unsupported plug task version: {version}")

    def __init__(self, model_path: Path, config=None):
        self.model_path = Path(model_path)
        self.config = config or PlugInsertConfig()
        self.profile = next(
            p for p in PLUG_PROFILES.values() if p.body == self.config.plug_body
        )
        self.name = self.profile.task
        self.coordinated_grasp = self.profile.grasp_closing_axis is not None
        if self.coordinated_grasp:
            self.version = self.task_version(self.name)
            self.stages = self.latch_stages
        root = ET.parse(model_path).getroot()
        self.chain = SerialChainKinematics.from_mjcf(
            model_path,
            base_body="cubesat_bus",
            joint_names=tuple(f"joint{i}" for i in range(1, 7)),
            tool_site="sarm_ee",
        )
        self.hole, hole_r = fixed_site_pose(root, self.config.target_site)
        self.pickup, self.pickup_rotation = fixed_site_pose(root, self.profile.prefix + "_pickup")
        self.hole_rotation = hole_r
        if np.linalg.norm(self.hole[:2] - self.pickup[:2]) < 0.05 or not np.allclose(
            hole_r[:, 2], [0., 0., 1.]
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
        self._timing = None
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
        self._grasp_sign = 1.0
        self._contact_recovery = False
        self._recovery_stable = 0.0
        self._contact_recoveries = 0
        self._pickup_filter = None
        self._gripper_timing = None
        self._carry_profile = None
        self._axial_speed = 0.
        self._axial_acceleration = 0.
        self._axial_measured_velocity = 0.
        self.retract_plan = None

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
            overload_stop_enabled=self.config.overload_stop_enabled,
            plug_body=self.config.plug_body,
            finger_plug_contacts=sample.finger_plug_contacts,
            grasp_heading_error_rad=self._grasp_heading_error(sample),
            contact_recovery_active=self._contact_recovery,
            contact_recoveries=self._contact_recoveries,
            orientation_error_rad=float(
                np.linalg.norm(
                    rotation_matrix_to_vector(
                        self.hole_rotation @ np.diag([1.0, -1.0, -1.0]) @ sample.plug_rotation_body.T
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
            grasp_hold_opening_rad=float(self._closed_reference),
            retract_plan=self.retract_plan,
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
        approach_profile = None
        if self.coordinated_grasp and self.stage == "approach":
            approach_profile = ApproachPosePath(
                p, r, goal, rotation, -rotation[:, 2], self._opening_at_entry,
                self.config.pregrasp_distance_m,
            )
        self._approach_profile = approach_profile
        path = [self.reference[:6].copy()]
        for a in np.linspace(0, 1, 81)[1:]:
            ra = axis_angle_to_matrix(rv / angle, a * angle) @ r if angle > 1e-12 else r
            pa = p + a * (goal - p)
            if approach_profile is not None:
                pa, ra = approach_profile.pose(a)
            q = solve_pose(self.chain, pa, ra, path[-1], self.lower, self.upper)
            if np.max(np.abs(q - path[-1])) > 0.3:
                raise ValueError("IK branch discontinuity")
            path.append(q)
        self._path = np.array(path)
        slope = np.max(np.abs(np.diff(self._path, axis=0))) * 80
        speed = self.config.linear_speed_m_s
        carrying = self.coordinated_grasp and self.stage in ("lift", "transfer", "above_hole")
        if carrying:
            speed = min(speed, self.config.carry_speed_m_s)
        elif self.stage == "retract":
            speed = self.config.retract_speed_m_s
        elif self.coordinated_grasp and self.stage == "descend":
            speed = min(speed, self.config.grasp_approach_speed_m_s)
        elif self.stage == "extract":
            speed = min(speed, self.config.extraction_speed_m_s)
        self._duration = max(
            1.0,
            1.875 * slope / self.config.arm_speed_rad_s,
            1.875
            * np.linalg.norm(goal - p)
            / speed,
            1.875 * angle / 0.2,
        )
        if self.coordinated_grasp and self.stage != "release":
            cfg = DrawerCycleConfig()
            distance = float(np.linalg.norm(goal - p))
            slow_distance = 0.
            if approach_profile is not None:
                linear, angular, opening = approach_profile.derivative_bounds()
                distance, angle = linear[0], angular[0]
                slow_distance = cfg.slow_distance_m * distance / approach_profile.height
            # Carrying a newly extracted connector needs a gentler envelope
            # than empty-arm motion, through lift, transfer AND hole approach.
            acceleration = cfg.linear_acceleration_m_s2
            jerk = cfg.linear_jerk_m_s3
            if carrying:
                acceleration = min(acceleration, self.config.carry_acceleration_m_s2)
                jerk = min(jerk, self.config.carry_jerk_m_s3)
            self._timing = time_joint_path(
                self._path, float(distance), float(angle),
                joint_speed=self.config.arm_speed_rad_s,
                linear_speed=speed, angular_speed=cfg.angular_speed_rad_s,
                linear_acceleration=acceleration,
                angular_acceleration=cfg.angular_acceleration_rad_s2,
                joint_acceleration=cfg.joint_acceleration_rad_s2,
                linear_jerk=jerk,
                angular_jerk=cfg.angular_jerk_rad_s3,
                joint_jerk=cfg.joint_jerk_rad_s3,
                slow_distance=slow_distance,
                slow_speed=self.config.grasp_approach_speed_m_s,
            )
            if approach_profile is not None:
                for bounds, limits in (
                    (linear, (speed, cfg.linear_acceleration_m_s2, cfg.linear_jerk_m_s3)),
                    (angular, (cfg.angular_speed_rad_s, cfg.angular_acceleration_rad_s2, cfg.angular_jerk_rad_s3)),
                    (opening, (self.config.gripper_speed_rad_s, cfg.gripper_acceleration_rad_s2, cfg.gripper_jerk_rad_s3)),
                ):
                    self._timing = self._timing.limit_derivatives(bounds, *limits)
            self._timing.path.validate_limits(self.lower, self.upper)
            self._duration = self._timing.duration
        self._goal, self._rotation = goal, rotation

    def _plan_carry(self, sample):
        # Freeze the measured grasp transform once for the entire carried path.
        p, r = self.chain.forward(self.reference[:6])
        offset, relative_r = self._grasp_transform
        goal = np.r_[self.hole[:2], self.hole[2] + self.profile.tip_m + .012]
        profile = CarryPosePath(p + r @ offset, r @ relative_r, goal,
                                self.plug_rotation, self.config.transport_height_m)
        self._carry_profile = profile
        waypoints = [self.reference[:6].copy()]
        for u in np.linspace(0., 1., 241)[1:]:
            position, rotation = profile.pose(u)
            tool_r = rotation @ relative_r.T
            q = solve_pose(self.chain, position-tool_r @ offset, tool_r,
                           waypoints[-1], self.lower, self.upper)
            if np.max(np.abs(q-waypoints[-1])) > .3:
                raise ValueError("Carry IK branch discontinuity")
            waypoints.append(q)
        self._path = np.asarray(waypoints)
        cfg = DrawerCycleConfig()
        self._timing = time_joint_path(
            self._path, profile.length, 1.875*profile.angle,
            joint_speed=self.config.arm_speed_rad_s,
            linear_speed=self.config.carry_speed_m_s,
            angular_speed=cfg.angular_speed_rad_s,
            linear_acceleration=self.config.carry_acceleration_m_s2,
            angular_acceleration=cfg.angular_acceleration_rad_s2,
            joint_acceleration=cfg.joint_acceleration_rad_s2,
            linear_jerk=self.config.carry_jerk_m_s3,
            angular_jerk=cfg.angular_jerk_rad_s3,
            joint_jerk=cfg.joint_jerk_rad_s3,
            slow_distance=.025, slow_speed=self.config.insert_speed_m_s,
            start_speed_m_s=float(np.clip((self.chain.jacobian(self.reference[:6])[:3] @ self.velocity[:6])[2], 0., self.config.carry_speed_m_s)),
            end_speed_m_s=self.config.insert_speed_m_s,
        )
        self._timing.path.validate_limits(self.lower, self.upper)
        self._duration = self._timing.duration
        self._rotation = self.plug_rotation @ relative_r.T
        self._goal = goal-self._rotation @ offset

    def _tool_for_plug(self, sample, desired_position, desired_rotation):
        p, r = self.chain.forward(sample.joints[:6])
        relative_r = r.T @ sample.plug_rotation_body
        relative_p = r.T @ (sample.plug_position_body_m - p)
        target_r = desired_rotation @ relative_r.T
        return desired_position - target_r @ relative_p, target_r

    def _enter(self, stage, sample):
        # These are recording labels on one trajectory, not motion endpoints.
        if (self.coordinated_grasp and self._carry_profile is not None
                and (self.stage, stage) in (("lift", "transfer"), ("transfer", "above_hole"))):
            self.stage = stage
            self.events.append(dict(stage=stage, sim_time_s=sample.time_s, **self.metrics(sample)))
            return
        incoming_stage = self.stage
        incoming_velocity = self.chain.jacobian(self.reference[:6])[:3] @ self.velocity[:6]
        self._carry_profile = None
        self.stage = stage
        if self._approach_rotation is not None:
            grasp, rotation = self._tracked_grasp(sample)
            if self.coordinated_grasp:
                if self._pickup_filter is None:
                    self._pickup_filter = PoseTargetFilter(grasp, rotation)
                grasp, rotation = self._pickup_filter.update(grasp, rotation, 0.)
            self.grasp, self.grasp_rotation = grasp, rotation
        self._pickup_anchor = (self.grasp.copy(), self.grasp_rotation.copy())
        self._pickup_hold = self.reference[:6].copy()
        self._pickup_latched = False
        self._entered = sample.time_s
        self._opening_at_entry = float(self.reference[6])
        self._stable = 0.0
        self._motion_s = 0.0
        self._timing = None
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
            if self.coordinated_grasp:
                rotation = self.grasp_rotation
        elif stage == "approach":
            rotation = self.grasp_rotation
            goal = self.grasp.copy()
        elif stage == "align":
            goal = p.copy()
            rotation = self.grasp_rotation
        elif stage == "descend":
            goal = self.grasp.copy()
            rotation = self.grasp_rotation
        elif stage in ("extract", "lift"):
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
                if self.coordinated_grasp:
                    # Unwind the insertion servo preload smoothly. Holding the
                    # old reference would drive the wrist down as fingers open.
                    goal, rotation = self.chain.forward(sample.joints[:6])
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
        if self.coordinated_grasp and stage in ("extract", "insert"):
            self._axial_speed = float(max(0., -incoming_velocity[2])) if stage == "insert" else 0.
            self._axial_acceleration = 0.
            self._axial_measured_velocity = float((self.chain.jacobian(sample.joints[:6])[:3] @ sample.joint_velocities[:6])[2])
            if stage == "insert" and incoming_stage == "above_hole":
                # Keep the reference pose/velocity at handoff, including servo lag.
                cp, cr = self.chain.forward(sample.joints[:6])
                self._plug_goal = sample.plug_position_body_m + (p-cp)
                self._plug_goal[:2] = self.hole[:2]
        if stage == "extract":
            # Stay centered in the pickup frame until the pins are clear.
            self._extract_z = self.pickup[2] + self.profile.tip_m + 0.012
            self._plug_goal = np.array([*self.pickup[:2], sample.plug_position_body_m[2]])
            self._extract_rotation = self.pickup_rotation @ np.diag([1., -1., -1.])
            goal = None
        if self.coordinated_grasp and stage == "lift":
            self._plan_carry(sample)
        elif stage == "retract" and self.profile.prefix == "cylindrical" and sample.native_qpos is not None:
            from .plug_clearance import plan_cylindrical_retract
            plan_cylindrical_retract(self, sample)
        elif goal is not None:
            self._plan(goal, rotation)
        continue_opening = self.coordinated_grasp and (incoming_stage, stage) in (
            ("release", "retract"), ("retract", "verify"))
        if not continue_opening:
            self._gripper_timing = None
        if self.coordinated_grasp and stage == "release":
            self._release_opening_s = 0.
            self._release_unloaded = False
            self._release_grip_reference = float(self.reference[6])
            self._release_start_opening = float(np.clip(
                max(sample.joints[6], -sample.joints[7]),
                self.reference[6], self.config.release_open_rad))
            self._release_unloaded = (
                np.max(np.abs(self.reference[:6] - sample.joints[:6])) < .002
                and abs(self._release_start_opening - self.reference[6]) < 1e-6)
        if self.coordinated_grasp and stage in ("grip", "release"):
            if stage == "grip" and self.profile.prefix == "cylindrical" and sample.native_qpos is not None:
                from .plug_clearance import cylindrical_hold_opening
                self._closed_reference = cylindrical_hold_opening(self, sample)
            cfg = DrawerCycleConfig()
            target = self.config.release_open_rad if stage == "release" else self._closed_reference
            speed = self.config.release_speed_rad_s if stage == "release" else self.config.gripper_speed_rad_s
            self._gripper_reverse = stage == "release"
            waypoints = [[target], [self._release_start_opening]] if self._gripper_reverse else [[self.reference[6]], [target]]
            round_release = self._gripper_reverse and not self.profile.keyed
            self._gripper_timing = time_joint_path(
                waypoints, 0., 0.,
                joint_speed=speed, linear_speed=cfg.linear_speed_m_s,
                angular_speed=cfg.angular_speed_rad_s,
                linear_acceleration=cfg.linear_acceleration_m_s2,
                angular_acceleration=cfg.angular_acceleration_rad_s2,
                joint_acceleration=cfg.gripper_acceleration_rad_s2,
                joint_jerk=cfg.gripper_jerk_rad_s3,
                # Reverse-time release: gently clear the finger/head contacts,
                # then open faster in free space. No added dwell.
                slow_fraction=(min(1., .10 / max(target - self._release_start_opening, 1e-9))
                               if round_release else (.25 if self._gripper_reverse else 0.)),
                slow_joint_speed=min(speed, .005 if round_release else .05) if self._gripper_reverse else None,
            )
        self._nominal_goal = None if self._goal is None else self._goal.copy()
        self._nominal_rotation = (
            self._rotation.copy() if hasattr(self, "_rotation") else r.copy()
        )
        self._timeout = max(30.0, 2.5 * self._duration + 15.0)
        if stage in ("extract", "insert", "verify") or (
            stage == "lift" and self.profile.feedback_extraction
        ):
            self._timeout = 90.0

    def _next(self, sample):
        self._enter(self.stages[self.stages.index(self.stage) + 1], sample)

    def planned_path_reference(self, elapsed):
        """Nominal timed arm/opening path, also used by collision preflight.

        Runtime may pause the arm clock for tracking lag and add measured object
        drift correction; preflight checks the nominal static scene only.
        """
        if self._timing is not None:
            progress = self._timing.at(elapsed)
            opening = self._opening_at_entry
            if self.stage == "approach":
                opening = self._approach_profile.opening(progress)
            return np.r_[self._timing.path.at(progress), opening, -opening]
        a = np.clip(elapsed / self._duration, 0.0, 1.0)
        u = (10 * a**3 - 15 * a**4 + 6 * a**5) * 80
        i = min(int(u), 79)
        arm = self._path[i] + (u - i) * (self._path[i + 1] - self._path[i])
        opening = self._opening_at_entry
        if self.coordinated_grasp and self.stage in ("raise", "above_pickup", "approach", "descend"):
            opening = min(1.0, opening + self.config.gripper_speed_rad_s * elapsed)
        return np.r_[arm, opening, -opening]

    def _tracked_grasp(self, sample):
        rotation = self._approach_rotation.copy()
        if self.coordinated_grasp:
            # The delivered meshes all place their paired head tabs on Y.
            # Track the measured part, not the original socket heading.
            # Follow the measured body frame, including randomized yaw and drift.
            closing = self._grasp_sign * sample.plug_rotation_body @ np.asarray(
                self.profile.grasp_closing_axis
            )
            approach = sample.plug_rotation_body[:, 2]
            rotation = np.column_stack([closing, np.cross(approach, closing), approach])
        return (
            sample.plug_position_body_m
            + sample.plug_rotation_body @ np.asarray(self.config.grasp_offset_m),
            rotation,
        )

    def _grasp_heading_error(self, sample):
        if not self.coordinated_grasp:
            return None
        _, tool_r = self.chain.forward(sample.joints[:6])
        axis = sample.plug_rotation_body @ np.asarray(self.profile.grasp_closing_axis)
        return float(np.arccos(np.clip(abs(tool_r[:, 0] @ axis), 0.0, 1.0)))

    def _follow_pickup(self, sample, dt, previous):
        """Follow measured part drift without writing the physical state."""
        if self.stage not in ("above_pickup", "align", "open", "approach", "descend", "grip"):
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
        if self.coordinated_grasp:
            grasp, grasp_rotation = self._pickup_filter.update(grasp, grasp_rotation, dt)
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

    def _advance_axial_speed(self, target, dt):
        # Velocity target tracking; acceleration is continuous and
        # bounded by jerk, rather than switching axial motion on/off each tick.
        acceleration = np.clip(12.*(target-self._axial_speed), -.15, .15)
        self._axial_acceleration += np.clip(acceleration-self._axial_acceleration, -.8*dt, .8*dt)
        self._axial_speed = max(0., self._axial_speed + dt*self._axial_acceleration)
        return self._axial_speed

    def _part_feedback(self, sample, dt, desired_rotation=None, axial_velocity=0.):
        p, r = self.chain.forward(sample.joints[:6])
        target, rotation = self._tool_for_plug(
            sample, self._plug_goal,
            self.plug_rotation if desired_rotation is None else desired_rotation,
        )
        error = np.r_[target - p, 0.2 * rotation_matrix_to_vector(rotation @ r.T)]
        moving_contact = self.coordinated_grasp and self.stage == "extract"
        jac = self.chain.jacobian(self.reference[:6] if moving_contact else sample.joints[:6])
        jac[3:] *= 0.2
        # Velocity feedforward makes contact speed independent of a large
        # positional lead; feedback still recenters the actual grasped part.
        twist = 3.0 * error
        twist[2] += axial_velocity
        if self.coordinated_grasp and self.stage == "extract":
            measured = float((self.chain.jacobian(sample.joints[:6])[:3] @ sample.joint_velocities[:6])[2])
            self._axial_measured_velocity += (1.-math.exp(-dt/(.005 if self.stage == "extract" else .01)))*(measured-self._axial_measured_velocity)
            # Damp spring preload release using measured motion, not a force abort.
            twist[2] += (8. if self.stage == "extract" else 4.)*(axial_velocity-self._axial_measured_velocity)
            twist[:2] = 6.*error[:2]
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(6), twist)
        # Axial contact motion uses the arm envelope; passive seating retains its cap.
        feedback_speed = self.config.arm_speed_rad_s
        if self.coordinated_grasp and self.stage not in ("extract", "insert"):
            feedback_speed = min(0.08, feedback_speed)
        dq *= min(1.0, feedback_speed / max(np.max(np.abs(dq)), 1e-12))
        candidate = self.reference[:6] + dt*dq
        if self.coordinated_grasp and self.stage in ("extract", "insert"):
            # Position-servo anti-windup: a stuck part must not accumulate an
            # ever-growing command offset and release it as a spring kick.
            candidate = np.clip(candidate, sample.joints[:6]-.06, sample.joints[:6]+.06)
        self.reference[:6] = np.clip(candidate, self.lower, self.upper)

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
            self.plug_rotation = self.hole_rotation @ np.diag([1.0, -1.0, -1.0])
            self.grasp = (
                sample.plug_position_body_m
                + sample.plug_rotation_body @ np.asarray(self.config.grasp_offset_m)
            )
            # Closing axis across the head; approach along spacecraft -Z.
            _, current_r = self.chain.forward(sample.joints[:6])
            closing = (
                sample.plug_rotation_body @ np.asarray(self.profile.grasp_closing_axis)
                if self.coordinated_grasp else self.pickup_rotation[:, 0].copy()
            )
            if closing @ current_r[:, 0] < 0:
                closing = -closing
                self._grasp_sign = -1.0
            approach = (
                sample.plug_rotation_body[:, 2]
                if self.coordinated_grasp else np.array([0.0, 0.0, -1.0])
            )
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
        if self.config.overload_stop_enabled and self._overload >= 0.15:
            self.fail("wrist_load_limit", sample)
        elif not sample.wrench_valid and sample.time_s > 0.2:
            self.fail("wrist_sensor_unavailable", sample)
        elif sample.time_s - self._entered > self._timeout:
            self.fail("stage_timeout", sample)
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if self._path is not None:
            unloading = self.coordinated_grasp and self.stage == "release"
            # Preload can exceed the normal tracking-error gate. Unloading moves
            # the reference back toward the measured wrist, so it must not wait
            # for the loaded robot to reach that old (penetrating) reference.
            if unloading or np.max(np.abs(self.reference[:6] - sample.joints[:6])) < 0.06:
                self._motion_s = min(self._duration, self._motion_s + dt)
            self.reference[:6] = self.planned_path_reference(self._motion_s)[:6]
        try:
            self._follow_pickup(sample, dt, old)
        except ValueError as exc:
            self.fail(f"tracking_ik_failed: {exc}", sample)
            return self.reference.copy(), np.zeros(8)
        opening_on_approach = self.coordinated_grasp and self.stage in (
            "raise", "above_pickup", "approach", "descend"
        )
        if self.coordinated_grasp and self.stage == "approach":
            self.reference[6:] = self.planned_path_reference(self._motion_s)[6:]
        elif self._gripper_timing is not None:
            opening_time = sample.time_s-self._entered
            if self.coordinated_grasp and self.stage in ("release", "retract", "verify"):
                # Unload finger and arm preload together, without a command
                # jump from the blocked closed target to the measured opening.
                wrist_speed = np.linalg.norm(self.chain.jacobian(sample.joints[:6])[:3] @ sample.joint_velocities[:6])
                self._release_unloaded = self._release_unloaded or (
                    self._motion_s >= self._duration and wrist_speed < .005)
                if self._release_unloaded:
                    self._release_opening_s += dt
                opening_time = self._release_opening_s
            if self._gripper_reverse:
                opening_time = self._gripper_timing.duration-opening_time
            opening = self._gripper_timing.joint_state(opening_time)[0][0]
            if self.coordinated_grasp and self.stage == "release" and not self._release_unloaded:
                u = min(1., self._motion_s / max(self._duration, 1e-9))
                blend = u**3 * (10. - 15.*u + 6.*u*u)
                opening = self._release_grip_reference + blend * (
                    self._release_start_opening - self._release_grip_reference)
            self.reference[6:] = [opening, -opening]
        elif self.stage in ("open", "grip", "release") or opening_on_approach:
            target = (
                self.config.release_open_rad
                if self.stage == "release"
                else (1.0 if self.stage == "open" or opening_on_approach else self._closed_reference)
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
        if self.stage == "extract":
            alignment = np.linalg.norm(rotation_matrix_to_vector(
                self._extract_rotation @ sample.plug_rotation_body.T
            ))
            radial = np.linalg.norm(sample.plug_position_body_m[:2] - self.pickup[:2])
            # A binary 0.5 mm gate flickered under contact compliance and kept
            # otherwise centered pulls nearly stationary. Fade only for a real
            # offset, and continue recentering while pulling.
            fraction = max((radial-.00075)/.00125, (alignment-.015)/.035)
            fraction = float(np.clip(fraction, 0., 1.))
            blend = 1.-(3*fraction**2-2*fraction**3)
            speed = self._advance_axial_speed(self.config.extraction_speed_m_s*blend, dt)
            if blend > 0.:
                self._plug_goal[2] = min(
                    self._plug_goal[2] + speed*dt,
                    sample.plug_position_body_m[2] + .002,
                )
            self._part_feedback(sample, dt, self._extract_rotation,
                                axial_velocity=speed*blend)
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
                aligned = (
                    metrics["radial_error_m"] < self.config.radial_tolerance_m
                    and metrics["orientation_error_rad"] < 0.004
                )
                if self.coordinated_grasp:
                    if (self.config.overload_stop_enabled and not self._contact_recovery and (
                        torque > 0.65 * self.config.torque_limit_nm
                        or force > 0.8 * self.config.force_limit_n
                    )):
                        # A short axial backoff unloads a tilted/keyed contact;
                        # correct XY/yaw before trying again, never force through it.
                        self._contact_recovery = True
                        self._contact_recoveries += 1
                        self._recovery_stable = 0.0
                        self._plug_goal[2] = min(
                            self.hole[2] + self.profile.tip_m + 0.012,
                            sample.plug_position_body_m[2] + 0.001,
                        )
                    if self._contact_recovery:
                        unloaded = (
                            torque < 0.5 * self.config.torque_limit_nm
                            and force < 0.65 * self.config.force_limit_n
                            and aligned
                            and abs(sample.plug_position_body_m[2] - self._plug_goal[2]) < 0.0003
                        )
                        self._recovery_stable = self._recovery_stable + dt if unloaded else 0.0
                        if self._recovery_stable >= 0.3:
                            self._contact_recovery = False
                    elif not aligned:
                        self._plug_goal[2] = max(self._plug_goal[2], sample.plug_position_body_m[2])
                insertion_blend = 1. if aligned else 0.
                if not self.coordinated_grasp or (aligned and not self._contact_recovery):
                    speed = self.config.insert_speed_m_s
                    if self.coordinated_grasp:
                        # Decelerate only for actual seating, not at the air/entry boundary.
                        remaining = max(0., self._plug_goal[2]-self._seat_z)
                        speed = self._advance_axial_speed(min(speed, 8.*remaining), dt)
                    next_z = self._plug_goal[2] - speed * dt
                    if self.coordinated_grasp:
                        # Bound penetration demand during contact, rather than
                        # continuing to advance the reference through a jam.
                        next_z = max(next_z, sample.plug_position_body_m[2] - 0.002)
                    self._plug_goal[2] = max(self._seat_z, next_z)
            if self.stage == "seat":
                self._plug_goal[2] = self._seat_z
            axial_velocity = 0.
            if self.coordinated_grasp and self.stage == "insert":
                if insertion_blend == 0. or self._contact_recovery:
                    self._advance_axial_speed(0., dt)
                if aligned and not self._contact_recovery:
                    axial_velocity = -self._axial_speed
            self._part_feedback(sample, dt, axial_velocity=axial_velocity)
        if self.stage == "release" and not self.coordinated_grasp and sample.joints[6] < 0.40:
            self._part_feedback(sample, dt)
        if self.stage in ("transfer", "above_hole", "align_hole", "insert", "seat") or (
            self.coordinated_grasp and self.stage in ("extract", "lift")
        ):
            p, r = self.chain.forward(sample.joints[:6])
            expected = p + r @ self._grasp_transform[0]
            if np.linalg.norm(expected - sample.plug_position_body_m) > 0.015:
                self.fail("plug_slipped_from_gripper", sample)
        elapsed = sample.time_s - self._entered
        p, r = self.chain.forward(sample.joints[:6])
        if self.stage == "settle":
            ready = elapsed >= 1.0
        elif self._carry_profile is not None and self.stage in ("lift", "transfer"):
            # Phase markers do not reset time, re-solve IK, or wait for settling.
            index = 0 if self.stage == "lift" else 1
            ready = self._timing.at(self._motion_s) >= self._carry_profile.boundaries[index]
            if ready and self.stage == "lift":
                self._ever_lifted = bool(sample.plug_position_body_m[2] - self._initial_position[2] > .12)
                if not self._ever_lifted:
                    self.fail("plug_not_lifted", sample)
                    ready = False
        elif self._path is not None and self.stage != "release":
            ready = (
                self._motion_s >= self._duration
                and np.linalg.norm(p - self._goal) < 0.004
                and np.linalg.norm(rotation_matrix_to_vector(self._rotation @ r.T))
                < 0.035
            )
            if self.stage == "descend" or (self.coordinated_grasp and self.stage == "approach"):
                ready = (
                    ready
                    and self._grasp_relative_speed(sample)
                    < self.config.grasp_relative_speed_m_s
                )
                if self.coordinated_grasp:
                    ready = ready and sample.joints[6] > 0.95
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
        elif self.stage == "extract":
            ready = (
                sample.plug_position_body_m[2] >= self._extract_z - 0.002
                and np.linalg.norm(sample.plug_position_body_m[:2] - self.pickup[:2]) < 0.001
                and np.linalg.norm(rotation_matrix_to_vector(
                    self._extract_rotation @ sample.plug_rotation_body.T)) < 0.01
            )
        elif self.stage == "open":
            ready = (
                self.reference[6] >= 0.999 and sample.joints[6] > 0.95 and elapsed > 5.0
            )
        elif self.stage == "release":
            if self.coordinated_grasp and sample.finger_plug_contacts is not None:
                # Clear both fingers before withdrawing, then finish opening
                # concurrently. Do not wait for the full free-space aperture.
                clear_opening = min(self.config.release_open_rad - .02,
                                    self._release_start_opening + .08)
                if not self.profile.keyed:
                    # The round shaft has no torsional restraint. A partial
                    # opening can clear the fingers now but hit the head again
                    # during lateral withdrawal, leaving a freely spinning part.
                    clear_opening = self.config.release_open_rad - .002
                ready = (self._release_unloaded
                         and not any(sample.finger_plug_contacts)
                         and min(sample.joints[6], -sample.joints[7]) >= clear_opening)
            else:
                ready = (
                    self.reference[6] >= self.config.release_open_rad - 1e-6
                    and sample.joints[6] > self.config.release_open_rad - 0.02
                    and (self._path is None or self._motion_s >= self._duration)
                )
            self._stable = self._stable + dt if ready else 0.0
            ready = self._stable >= (0.1 if self.coordinated_grasp else 1.0)
        elif self.stage == "grip":
            ready = (
                elapsed > 6.0 and self.reference[6] < 0.001 and sample.joints[6] > 0.025
            )
            if sample.finger_plug_contacts is not None:
                ready = ready and all(sample.finger_plug_contacts)
            if self.coordinated_grasp:
                # Bilateral contact alone can be a grasp on the wrong head faces.
                ready = (
                    self.reference[6] <= self._closed_reference + .001
                    and sample.joints[6] > 0.025
                    and sample.finger_plug_contacts is not None
                    and all(sample.finger_plug_contacts)
                    and metrics["grasp_heading_error_rad"] < 0.05
                    and np.linalg.norm(p - self.grasp) < 0.004
                    and self._grasp_relative_speed(sample) < self.config.grasp_relative_speed_m_s
                )
                self._stable = self._stable + dt if ready else 0.0
                ready = self._stable >= 0.15
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
            if self.stage == "insert" and not self.coordinated_grasp:
                ready = ready and self._plug_goal[2] <= self._seat_z + 1e-6
            # Aviation seating is accepted from actual geometry and velocity,
            # not a virtual target that can lag during contact recentering.
            if self.stage == "verify":
                ready = (
                    ready
                    and self._ever_lifted
                    and sample.joints[6] > self.config.release_open_rad - 0.03
                    and np.linalg.norm(p[:2] - self.hole[:2]) > 0.09
                )
            self._stable = self._stable + dt if ready else 0.0
            ready = self._stable >= (
                (min(self.config.hold_s, .5) if self.coordinated_grasp else self.config.hold_s) if self.stage == "verify" else (.15 if self.coordinated_grasp else 1.0)
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
