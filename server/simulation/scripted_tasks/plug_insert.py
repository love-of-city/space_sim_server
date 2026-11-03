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
from simulation.scripted_tasks import plug_motion_config as _motion_config
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES
from simulation.scripted_tasks.plug_contact_feedback import PlugContactFeedback


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
    version = _motion_config.CURRENT_TASK_VERSION
    config_schema = _motion_config.CONFIG_SCHEMA
    stages = (
        "settle", "approach", "grip", "extract", "lift", "transfer",
        "above_hole", "insert", "release", "retract", "verify",
    )

    @classmethod
    def task_version(cls, task_name):
        from .plug_profiles import plug_profile
        plug_profile(task_name)  # Reject unknown task identities.
        return cls.version

    def __init__(self, model_path: Path, config=None):
        self.model_path = Path(model_path)
        self.config = config or _motion_config.PlugInsertConfig()
        self.profile = next(
            p for p in PLUG_PROFILES.values() if p.body == self.config.plug_body
        )
        self.name = self.profile.task
        self.configuration_snapshot = _motion_config.config_snapshot(self.config, self.name)
        root = ET.parse(model_path).getroot()
        self.chain = SerialChainKinematics.from_mjcf(
            model_path,
            base_body="cubesat_bus",
            joint_names=tuple(f"joint{i}" for i in range(1, 7)),
            tool_site="sarm_ee",
        )
        self.hole, hole_r = fixed_site_pose(root, self.config.target_site)
        self.pickup, self.pickup_rotation = fixed_site_pose(root, self.profile.prefix + "_pickup")
        self._extract_z = self.pickup[2] + self.profile.tip_m + 0.012
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
        self._axial_target_speed_m_s = 0.
        self._contact_feedback = PlugContactFeedback(
            extraction_lateral_gain_s_inv=self.config.extraction_lateral_gain_s_inv,
            extraction_velocity_gain=self.config.extraction_velocity_gain)
        self._axial_acceleration = 0.
        self._insert_alignment_paused = False
        self._insert_alignment_pause_s = 0.
        self._insert_alignment_hold_s = 0.
        self._insert_alignment_creep_s = 0.
        self._insert_alignment_creep_wait_s = 0.
        self._insert_alignment_transitions = 0
        self._insert_alignment_creep_active = False
        self._insert_alignment_creep_count = 0
        self._insert_speed_scale = 1.
        self.retract_plan = None
        self._release_clearance_probe = None
        self._release_clearance_opening = None
        self.release_clearance_checks = []
        self._release_hold_joints = None
        self.environment_planning = False
        self.motion_plans = []
        self._searched_carry = False

    def configure_contact_feedback(self, joint_stiffness_nm_rad, retention_parameters):
        """Bind preload to the actual runtime servo and retention configuration."""
        if self.stage != "pending":
            raise RuntimeError("Contact feedback must be configured before the task starts")
        self._contact_feedback = PlugContactFeedback(
            joint_stiffness_nm_rad=joint_stiffness_nm_rad,
            friction_force_n=(retention_parameters.sliding_friction_n
                              if retention_parameters is not None else 0.),
            friction_travel_m=(retention_parameters.friction_travel_m
                               if retention_parameters is not None else 0.),
            extraction_lateral_gain_s_inv=self.config.extraction_lateral_gain_s_inv,
            extraction_velocity_gain=self.config.extraction_velocity_gain,
        )
        return self._contact_feedback.configuration()

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
            insertion_alignment_paused=self._insert_alignment_paused,
            insertion_alignment_pause_s=self._insert_alignment_pause_s,
            insertion_alignment_hold_s=self._insert_alignment_hold_s,
            insertion_alignment_creep_s=self._insert_alignment_creep_s,
            insertion_alignment_transitions=self._insert_alignment_transitions,
            insertion_alignment_creep_active=self._insert_alignment_creep_active,
            insertion_alignment_creep_count=self._insert_alignment_creep_count,
            insertion_speed_scale=self._insert_speed_scale,
            axial_target_speed_m_s=(self._axial_target_speed_m_s
                                   if self.stage in ("extract", "insert") else 0.),
            contact_preload_force_n=(self._contact_feedback.preload_force_n
                                     if self.stage in ("extract", "insert") else 0.),
            contact_remaining_m=(
                max(0., self._extract_z - .002 - sample.plug_position_body_m[2])
                if self.stage == "extract" else
                max(0., float(flange[2] - self.hole[2]) - (
                    min(self.config.seat_tolerance_m, .0001)
                    if self.profile.prefix == "cylindrical" else self.config.seat_tolerance_m))
                if self.stage == "insert" else 0.
            ),
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
            release_clearance_checks=list(self.release_clearance_checks),
        )

    def fail(self, reason, sample):
        from .plug_clearance import close_cylindrical_release_probe
        close_cylindrical_release_probe(self)
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
        if self.stage == 'approach':
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
        carrying = self.stage in ('lift', 'transfer', 'above_hole')
        if carrying:
            speed = min(speed, self.config.carry_speed_m_s)
        elif self.stage == "retract":
            speed = self.config.retract_speed_m_s
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
        if self.stage != 'release':
            cfg = _motion_config.MOTION_LIMITS
            distance = float(np.linalg.norm(goal - p))
            slow_distance = 0.
            if approach_profile is not None:
                linear, angular, opening = approach_profile.derivative_bounds()
                distance, angle = linear[0], angular[0]
                slow_distance = cfg.slow_distance_m
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
        cfg = _motion_config.MOTION_LIMITS
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
        if stage not in self.stages:
            raise ValueError(f"Unsupported current plug stage: {stage}")
        # These are recording labels on one trajectory, not motion endpoints.
        if (self._carry_profile is not None and (self.stage, stage) in (('lift', 'transfer'), ('transfer', 'above_hole'))):
            self.stage = stage
            self.events.append(dict(stage=stage, sim_time_s=sample.time_s, **self.metrics(sample)))
            return
        incoming_stage = self.stage
        if (incoming_stage,stage) != ("release","retract"):
            from .plug_clearance import close_cylindrical_release_probe
            close_cylindrical_release_probe(self)
        if stage == "release":
            self._release_clearance_opening = None
            self.release_clearance_checks = []
        searched_release_complete = (
            incoming_stage == "retract" and stage == "verify"
            and self._timing is not None
            and self._timing.path.coefficients.shape[-1] == 7
        )
        incoming_velocity = self.chain.jacobian(self.reference[:6])[:3] @ self.velocity[:6]
        self._carry_profile = None
        self._searched_carry = False
        self.stage = stage
        if self._approach_rotation is not None:
            grasp, rotation = self._tracked_grasp(sample)
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
        if stage == "approach":
            rotation = self.grasp_rotation
            goal = self.grasp.copy()
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
        elif stage in ('insert', 'release'):
            self._plug_goal = np.array(
                [*self.hole[:2], self.hole[2] + self.config.shaft_tip_local_m + 0.012]
            )
            if stage in ('insert', 'release'):
                self._plug_goal[2] = sample.plug_position_body_m[2]
            self._seat_z = self.hole[2] + self.config.flange_local_m
            if stage == "release":
                self._plug_goal[2] = self._seat_z
                # Unwind the insertion servo preload smoothly. Holding the
                # old reference would drive the wrist down as fingers open.
                goal, rotation = self.chain.forward(sample.joints[:6])
                self._release_hold_joints = sample.joints[:6].copy()
        elif stage == "retract":
            # Withdraw between the open fingers, along their tangential direction.
            # Vertical arm recoil otherwise pulls the free spacecraft away from the plug.
            direction = r[:, 1].copy()
            direction[2] = 0.0
            direction /= np.linalg.norm(direction)
            if direction[0] < 0:
                direction = -direction
            goal = p + 0.12 * direction
        if stage in ('extract', 'insert'):
            self._contact_feedback.reset(self.reference[:6])
            self._axial_speed = float(max(0., -incoming_velocity[2])) if stage == "insert" else 0.
            self._axial_acceleration = 0.
            if stage == "insert" and incoming_stage == "above_hole":
                # Keep the reference pose/velocity at handoff, including servo lag.
                cp, cr = self.chain.forward(sample.joints[:6])
                self._plug_goal = sample.plug_position_body_m + (p-cp)
                self._plug_goal[:2] = self.hole[:2]
        if stage == "extract":
            # Stay centered in the pickup frame until the pins are clear.
            self._plug_goal = np.array([*self.pickup[:2], sample.plug_position_body_m[2]])
            self._extract_rotation = self.pickup_rotation @ np.diag([1., -1., -1.])
            goal = None
        error = None
        retract_candidates = None
        try:
            if stage == 'lift':
                self._plan_carry(sample)
            elif stage == "retract" and self.profile.prefix == "cylindrical" and sample.native_qpos is not None:
                from .plug_clearance import plan_cylindrical_retract, cylindrical_retract_candidates
                retract_candidates = cylindrical_retract_candidates(self)
                # A blocked diagonal still needs a rising destination. Do not
                # fall back to the obsolete horizontal goal beside the plugs.
                goal,rotation = retract_candidates[0]['position'],retract_candidates[0]['rotation']
                plan_cylindrical_retract(self, sample, retract_candidates)
            elif goal is not None:
                self._plan(goal, rotation)
        except ValueError as exc:
            if not self.environment_planning or stage not in ("approach","lift","retract"):
                raise
            error = str(exc)
        if self.environment_planning and stage in ("approach","lift","retract"):
            from .environment_motion import plan
            if stage == "lift":
                offset, relative_r = self._grasp_transform
                rotation = self.plug_rotation @ relative_r.T
                goal = np.r_[self.hole[:2],self.hole[2]+self.profile.tip_m+.012]-rotation@offset
            elif error is None and self._goal is not None:
                goal,rotation=self._goal,self._rotation
            self._goal,self._rotation=goal,rotation
            report=plan(self,sample,goal,rotation,-rotation[:,2] if stage != "retract" else rotation[:,2],
                        held=stage=="lift",error=error,
                        opening=self.config.release_open_rad if stage=="retract" else 1.,
                        path_reference=self.path_reference,
                        fallback_positions=(None if retract_candidates is None else
                                            [target['position'] for target in retract_candidates]))
            self._searched_carry=stage=="lift" and report['route']=='searched'
            if retract_candidates is not None and report['route']=='searched':
                target = retract_candidates[report['goal_candidate_index']]
                self.retract_plan = dict(
                    height_m=target['height_m'],direction_body=target['direction_body'],
                    route='searched',attempts=(self.retract_plan or {}).get('attempts',[]),
                    goal_candidate_index=report['goal_candidate_index'])
        if self.environment_planning and sample.native_qpos is not None and stage in ("extract", "insert"):
            from .operation_clearance import check_operation
            check_operation(self, sample)
        continue_opening = (incoming_stage, stage) in (('release', 'retract'), ('retract', 'verify'))
        if not continue_opening or searched_release_complete:
            # The searched arm/jaw path completed the opening on its own clock.
            # Its predecessor's release timer was not advanced during that
            # path; replaying it in verify would abruptly close the fingers.
            self._gripper_timing = None
        if stage == 'release':
            self._release_opening_s = 0.
            self._release_unloaded = False
            self._release_grip_reference = float(self.reference[6])
            self._release_start_opening = float(np.clip(
                max(sample.joints[6], -sample.joints[7]),
                self.reference[6], self.config.release_open_rad))
            self._release_unloaded = (
                np.max(np.abs(self.reference[:6] - sample.joints[:6])) < .002
                and abs(self._release_start_opening - self.reference[6]) < 1e-6)
        if stage in ('grip', 'release'):
            if stage == "grip" and self.profile.prefix == "cylindrical" and sample.native_qpos is not None:
                from .plug_clearance import cylindrical_hold_opening
                self._closed_reference = cylindrical_hold_opening(self, sample)
            cfg = _motion_config.MOTION_LIMITS
            target = self.config.release_open_rad if stage == "release" else self._closed_reference
            speed = self.config.release_speed_rad_s if stage == "release" else self.config.gripper_speed_rad_s
            self._gripper_reverse = stage == "release"
            waypoints = [[target], [self._release_start_opening]] if self._gripper_reverse else [[self.reference[6]], [target]]
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
                slow_fraction=.25 if self._gripper_reverse else 0.,
                slow_joint_speed=min(speed, .05) if self._gripper_reverse else None,
            )
        self._nominal_goal = None if self._goal is None else self._goal.copy()
        self._nominal_rotation = (
            self._rotation.copy() if hasattr(self, "_rotation") else r.copy()
        )
        self._timeout = max(30.0, 2.5 * self._duration + 15.0)
        if stage in ('extract', 'insert', 'verify'):
            self._timeout = 90.0

    def _next(self, sample):
        self._enter(self.stages[self.stages.index(self.stage) + 1], sample)

    def path_reference(self, progress):
        if self._timing.path.coefficients.shape[-1] == 7:
            q=self._timing.path.at(progress)
            return np.r_[q[:6],q[6],-q[6]]
        opening=(self._approach_profile.opening(progress) if self.stage=="approach" else self._opening_at_entry)
        if self.stage == "retract" and self._gripper_timing is not None:
            elapsed = self._release_opening_s + self._timing.seconds_at(progress)
            if self._gripper_reverse:
                elapsed = self._gripper_timing.duration - elapsed
            opening = self._gripper_timing.joint_state(elapsed)[0][0]
        return np.r_[self._timing.path.at(progress),opening,-opening]

    def planned_path_reference(self, elapsed):
        """Nominal timed arm/opening path, also used by collision preflight.

        Runtime may pause the arm clock for tracking lag. Coordinated approach
        uses this checked path without drift correction; contact feedback in
        later stages still uses measured state. Preflight uses a static scene.
        """
        if self._timing is not None:
            progress = self._timing.at(elapsed)
            return self.path_reference(progress)
        a = np.clip(elapsed / self._duration, 0.0, 1.0)
        u = (10 * a**3 - 15 * a**4 + 6 * a**5) * 80
        i = min(int(u), 79)
        arm = self._path[i] + (u - i) * (self._path[i + 1] - self._path[i])
        opening = self._opening_at_entry
        if self.stage in ('approach',):
            opening = min(1.0, opening + self.config.gripper_speed_rad_s * elapsed)
        return np.r_[arm, opening, -opening]

    def _tracked_grasp(self, sample):
        rotation = self._approach_rotation.copy()
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
        _, tool_r = self.chain.forward(sample.joints[:6])
        axis = sample.plug_rotation_body @ np.asarray(self.profile.grasp_closing_axis)
        return float(np.arccos(np.clip(abs(tool_r[:, 0] @ axis), 0.0, 1.0)))

    def _follow_pickup(self, sample, dt, previous):
        """Follow measured part drift without writing the physical state."""
        if self.stage not in ('approach', 'grip'):
            return
        if self.stage == 'approach' and self._timing is not None:
            # The checked arm/finger trajectory owns the shared approach clock.
            # Adding a second tracker here bends the straight line (or detour)
            # after collision checking and can abruptly cap its velocity.
            # Correct residual part drift during the physical grip phase.
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
        grasp, grasp_rotation = self._pickup_filter.update(grasp, grasp_rotation, dt)
        offset = grasp - origin
        delta = (
            grasp_rotation @ rotation.T
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

    def _insertion_orientation_error(self, metrics):
        # A round connector has no key: axial spin cannot obstruct its bore.
        return metrics["orientation_error_rad" if self.profile.keyed else "axis_error_rad"]

    def _contact_orientation_error(self, actual, desired):
        if self.profile.keyed:
            return float(np.linalg.norm(rotation_matrix_to_vector(desired @ actual.T)))
        return float(np.arccos(np.clip(desired[:, 2] @ actual[:, 2], -1., 1.)))

    def _contact_rotation(self, actual, desired):
        """Align a round shaft's axis without chasing its unconstrained spin."""
        if self.profile.keyed:
            return desired
        axis = np.cross(actual[:, 2], desired[:, 2])
        sine = float(np.linalg.norm(axis))
        cosine = float(np.clip(actual[:, 2] @ desired[:, 2], -1., 1.))
        if sine < 1e-10:
            return actual if cosine > 0. else desired
        return axis_angle_to_matrix(axis / sine, math.atan2(sine, cosine)) @ actual

    def _insertion_alignment_scale(self, metrics, dt, force_n=0., torque_nm=0.):
        # Retain the physical alignment envelope. Taper before its boundary and
        # resume inside a separate band, rather than alternating every tick.
        # After a short pause, a mild error can use a load-limited creep speed;
        # this keeps demonstrations continuous without pushing through a hard
        # alignment error or a loaded contact.
        ratio = max(metrics["radial_error_m"] / self.config.radial_tolerance_m,
                    self._insertion_orientation_error(metrics) /
                    self.config.insert_orientation_tolerance_rad)
        was_creeping = self._insert_alignment_creep_active
        self._insert_alignment_creep_active = False
        paused = self._insert_alignment_paused
        if ratio >= 1.:
            self._insert_alignment_paused = True
        elif ratio <= self.config.insert_resume_ratio:
            self._insert_alignment_paused = False
        self._insert_alignment_transitions += int(paused != self._insert_alignment_paused)
        if self._insert_alignment_paused:
            if not paused:
                # Keep the aggregate pause metric, but start the creep delay
                # anew for each distinct pause window.
                self._insert_alignment_creep_wait_s = 0.
            self._insert_alignment_pause_s += dt
            self._insert_alignment_creep_wait_s += dt
            creep_safe = (
                self._insert_alignment_creep_wait_s >= self.config.insert_creep_delay_s
                and ratio <= self.config.insert_creep_ratio
                and math.isfinite(force_n) and math.isfinite(torque_nm)
                and force_n <= self.config.force_limit_n * self.config.insert_creep_force_fraction
                and torque_nm <= self.config.torque_limit_nm * self.config.insert_creep_torque_fraction
            )
            if creep_safe:
                self._insert_alignment_creep_active = True
                if not was_creeping:
                    self._insert_alignment_creep_count += 1
                self._insert_alignment_creep_s += dt
                self._insert_speed_scale = self.config.insert_creep_speed_scale
            else:
                self._insert_alignment_hold_s += dt
                self._insert_speed_scale = 0.
        else:
            self._insert_alignment_creep_wait_s = 0.
            u = float(np.clip((ratio-self.config.insert_slowdown_ratio) /
                              (1.-self.config.insert_slowdown_ratio), 0., 1.))
            self._insert_speed_scale = 1.-u*u*(3.-2.*u)
        return self._insert_speed_scale

    def _advance_axial_speed(self, target, dt):
        # Velocity target tracking; acceleration is continuous and
        # bounded by jerk, rather than switching axial motion on/off each tick.
        acceleration = np.clip(12.*(target-self._axial_speed), -.15, .15)
        self._axial_acceleration += np.clip(acceleration-self._axial_acceleration, -.8*dt, .8*dt)
        self._axial_speed = max(0., self._axial_speed + dt*self._axial_acceleration)
        return self._axial_speed

    def _part_feedback(self, sample, dt, desired_rotation=None, axial_velocity=0.):
        self._axial_target_speed_m_s = float(axial_velocity)
        desired_rotation = self.plug_rotation if desired_rotation is None else desired_rotation
        target, rotation = self._tool_for_plug(
            sample, self._plug_goal,
            self._contact_rotation(sample.plug_rotation_body, desired_rotation),
        )
        if self.stage in ('extract', 'insert'):
            self.reference[:6] = self._contact_feedback.step(
                self.chain, sample.joints[:6], target, rotation,
                axial_velocity_m_s=axial_velocity,
                seat_gap_m=(sample.plug_position_body_m[2]-self._seat_z
                            if self.stage == "insert" else 0.),
                dt=dt, max_speed_rad_s=self.config.arm_speed_rad_s,
                lower=self.lower, upper=self.upper,
                motion=self.stage,
                extraction_travel_m=(sample.plug_position_body_m[2]-self._initial_position[2]
                                     if self.stage == "extract" else 0.),
                measured_joint_velocities=sample.joint_velocities[:6],
                preload_allowed=not self._contact_recovery,
            )
            return
    def _hold_release_pose(self, sample, dt):
        """Hold the measured release-entry pose throughout finger opening.

        Recomputing a grasp transform from a slipping part follows the slip and
        can sweep the wrist into a neighbor. The release path already saved the
        measured wrist as its goal. Measured twist damps the unloading motion;
        back-calculation toward that same joint pose prevents a blocked wrist
        from accumulating a reference offset that springs free with the fingers.
        Neither term switches off at the last contact or resets the reference.
        """
        p, r = self.chain.forward(sample.joints[:6])
        error = np.r_[self._goal-p, .2*rotation_matrix_to_vector(self._rotation@r.T)]
        jac = self.chain.jacobian(sample.joints[:6])
        jac[3:] *= .2
        measured_twist = jac @ sample.joint_velocities[:6]
        dq = jac.T @ np.linalg.solve(jac@jac.T+1e-6*np.eye(6), 3.*error-measured_twist)
        # The release-entry IK branch is the unloaded equilibrium reference.
        # Use the pose feedback's existing 3/s rate for reference tracking as
        # well: a stationary blocked wrist produces a finite correction.
        dq -= 3.*(self.reference[:6]-self._release_hold_joints)
        speed = min(.08, self.config.arm_speed_rad_s)
        dq *= min(1., speed/max(float(np.max(np.abs(dq))),1e-12))
        candidate = np.clip(self.reference[:6]+dt*dq,
                            sample.joints[:6]-.06, sample.joints[:6]+.06)
        self.reference[:6] = np.clip(candidate,self.lower,self.upper)

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
            )
            if closing @ current_r[:, 0] < 0:
                closing = -closing
                self._grasp_sign = -1.0
            approach = (
                sample.plug_rotation_body[:, 2]
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
        if self.config.overload_stop_enabled and self._overload >= _motion_config.TASK_RULES.overload_confirm_s:
            self.fail("wrist_load_limit", sample)
        elif not sample.wrench_valid and sample.time_s > 0.2:
            self.fail("wrist_sensor_unavailable", sample)
        elif sample.time_s - self._entered > self._timeout:
            self.fail("stage_timeout", sample)
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        unloading = self.stage == 'release'
        seating_release = unloading and self.profile.prefix == "cylindrical"
        if self._path is not None and not (seating_release and self._release_unloaded):
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
        opening_on_approach = self.stage in ('approach',)
        if self._timing is not None and self._timing.path.coefficients.shape[-1] == 7:
            self.reference[6:] = self.planned_path_reference(self._motion_s)[6:]
        elif self.stage == 'approach':
            self.reference[6:] = self.planned_path_reference(self._motion_s)[6:]
        elif self._gripper_timing is not None:
            opening_time = sample.time_s-self._entered
            if self.stage in ('release', 'retract', 'verify'):
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
            if self.stage == 'release' and (not self._release_unloaded):
                u = min(1., self._motion_s / max(self._duration, 1e-9))
                blend = u**3 * (10. - 15.*u + 6.*u*u)
                opening = self._release_grip_reference + blend * (
                    self._release_start_opening - self._release_grip_reference)
            self.reference[6:] = [opening, -opening]
        elif self.stage in ('grip', 'release') or opening_on_approach:
            target = (
                self.config.release_open_rad if self.stage == 'release' else 1.0 if opening_on_approach else self._closed_reference
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
        # Other stages use measured poses/contacts directly; constructing the
        # full serialized diagnostics there does not affect their controller.
        metrics = (self.metrics(sample) if self.stage in ('lift', 'grip', 'insert', 'verify') else None)
        if self.stage == "extract":
            alignment = self._contact_orientation_error(
                sample.plug_rotation_body, self._extract_rotation)
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
        if self.stage in ('insert',):
            if self.stage == "insert":
                aligned = (
                    metrics["radial_error_m"] < self.config.radial_tolerance_m
                    and self._insertion_orientation_error(metrics)
                    < self.config.insert_orientation_tolerance_rad
                )
                insertion_blend = self._insertion_alignment_scale(
                    metrics, dt, force, torque
                )
                advance = insertion_blend > 0. and (
                    aligned or self._insert_alignment_creep_active
                )
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
                elif not advance:
                    self._plug_goal[2] = max(self._plug_goal[2], sample.plug_position_body_m[2])
                if advance and (not self._contact_recovery):
                    speed = self.config.insert_speed_m_s
                    # Decelerate only for actual seating, not at the air/entry boundary.
                    remaining = max(0., self._plug_goal[2]-self._seat_z)
                    speed = self._advance_axial_speed(min(speed*insertion_blend, 8.*remaining), dt)
                    next_z = self._plug_goal[2] - speed * dt
                    # Bound penetration demand during contact, rather than
                    # continuing to advance the reference through a jam.
                    next_z = max(next_z, sample.plug_position_body_m[2] - 0.002)
                    self._plug_goal[2] = max(self._seat_z, next_z)
            axial_velocity = 0.
            if self.stage == 'insert':
                if insertion_blend == 0. or self._contact_recovery:
                    self._advance_axial_speed(0., dt)
                if advance and not self._contact_recovery:
                    axial_velocity = -self._axial_speed
            self._part_feedback(sample, dt, axial_velocity=axial_velocity)
        if seating_release and self._release_unloaded:
            # A four-bar finger can remain hooked while its partner clears.
            # Hold the release-entry wrist pose as the jaws open, so measured
            # servo deflection cannot drag the hook sideways along the head.
            # Continue through clearance opening: dropping feedback when the
            # last finger clears leaves its accumulated servo preload behind.
            self._hold_release_pose(sample, dt)
        if self.stage in ('transfer', 'above_hole', 'insert') or self.stage in ('extract', 'lift'):
            p, r = self.chain.forward(sample.joints[:6])
            expected = p + r @ self._grasp_transform[0]
            if np.linalg.norm(expected - sample.plug_position_body_m) > 0.015:
                self.fail("plug_slipped_from_gripper", sample)
        if self._searched_carry and sample.plug_position_body_m[2]-self._initial_position[2]>.12:
            self._ever_lifted=True
        elapsed = sample.time_s - self._entered
        p, r = self.chain.forward(sample.joints[:6])
        if self.stage == "settle":
            ready = elapsed >= _motion_config.TASK_RULES.initial_settle_s
        elif self._carry_profile is not None and self.stage in ("lift", "transfer"):
            # Phase markers do not reset time, re-solve IK, or wait for settling.
            index = 0 if self.stage == "lift" else 1
            progress=self._timing.at(self._motion_s)
            ready = progress >= (1. if self._searched_carry else self._carry_profile.boundaries[index])
            if ready and self.stage == "lift":
                self._ever_lifted = self._ever_lifted or bool(sample.plug_position_body_m[2] - self._initial_position[2] > .12)
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
            if self.stage == 'approach':
                ready = (
                    ready
                    and self._grasp_relative_speed(sample)
                    < self.config.grasp_relative_speed_m_s
                )
                ready = ready and sample.joints[6] > 0.95
            if self.stage == "lift" and ready:
                self._ever_lifted = (
                    sample.plug_position_body_m[2] - self._initial_position[2] > 0.12
                )
                if not self._ever_lifted:
                    self.fail("plug_not_lifted", sample)
                    ready = False
        elif self.stage == "extract":
            ready = (
                sample.plug_position_body_m[2] >= self._extract_z - 0.002
                and np.linalg.norm(sample.plug_position_body_m[:2] - self.pickup[:2]) < 0.001
                and self._contact_orientation_error(
                    sample.plug_rotation_body, self._extract_rotation) < 0.01
            )
        elif self.stage == "release":
            if sample.finger_plug_contacts is not None:
                # Clear both fingers before withdrawing, then finish opening
                # concurrently. Do not wait for the full free-space aperture.
                # Native cylindrical geometry chooses the needed aperture below;
                # query it once bilateral clearance is confirmed instead of
                # waiting for an unrelated fixed jaw-angle increment first.
                native_clearance = (
                    self.profile.prefix == "cylindrical"
                    and sample.native_qpos is not None
                )
                clear_opening = min(self.config.release_open_rad - .02,
                                    self._release_start_opening + .08)
                ready = (self._release_unloaded
                         and not any(sample.finger_plug_contacts)
                         and (native_clearance or min(sample.joints[6], -sample.joints[7]) >= clear_opening))
            else:
                ready = (
                    self.reference[6] >= self.config.release_open_rad - 1e-6
                    and sample.joints[6] > self.config.release_open_rad - 0.02
                    and (self._path is None or self._motion_s >= self._duration)
                )
            self._stable = self._stable + dt if ready else 0.0
            ready = self._stable >= _motion_config.TASK_RULES.release_confirm_s
            if ready and self.profile.prefix == "cylindrical" and sample.native_qpos is not None:
                from .plug_clearance import cylindrical_release_clearance
                try:
                    ready = cylindrical_release_clearance(self, sample)
                except ValueError as exc:
                    self.fail(f"release_clearance_failed: {exc}", sample)
                    ready = False
        elif self.stage == "grip":
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
            ready = self._stable >= _motion_config.TASK_RULES.grasp_confirm_s
            if elapsed > 8.0 and sample.joints[6] < 0.025:
                self.fail("gripper_closed_without_object", sample)
        elif self.stage in ('insert', 'verify'):
            ready = (
                metrics["radial_error_m"] < self.config.radial_tolerance_m
                and metrics["axis_error_rad"] < _motion_config.TASK_RULES.final_alignment_tolerance_rad
                and abs(metrics["seat_gap_m"]) < self.config.seat_tolerance_m
                and metrics["insertion_depth_m"]
                > self.config.shaft_tip_local_m - self.config.flange_local_m - 0.002
                and (not self.profile.keyed or metrics["orientation_error_rad"] < _motion_config.TASK_RULES.final_alignment_tolerance_rad)
            )
            # Before release, retain the existing held-part motion gates.
            # Once released and withdrawn, confirm continued geometric
            # seating; residual motion is recorded but does not delay success.
            geometric_completion = self.stage == "verify"
            if not geometric_completion and self.profile.require_seated_velocity:
                ready = ready and np.linalg.norm(sample.plug_velocity_body_m_s) < 0.002
                angular_velocity = sample.plug_angular_velocity_body_rad_s
                if not self.profile.keyed:
                    axis = sample.plug_rotation_body[:, 2]
                    angular_velocity = angular_velocity - axis * (angular_velocity @ axis)
                ready = (
                    ready
                    and np.linalg.norm(angular_velocity) < 0.03
                )
            if self.stage == "insert" and self.profile.release_seat_tolerance_m is not None:
                # The cylindrical flange must seat before releasing the fingers;
                # the final geometric acceptance remains the common tolerance.
                ready = ready and abs(metrics["seat_gap_m"]) < min(
                    self.config.seat_tolerance_m, self.profile.release_seat_tolerance_m)
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
            confirm_s = self.config.completion_confirm_s if geometric_completion else _motion_config.TASK_RULES.seated_confirm_s
            ready = self._stable >= confirm_s
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
