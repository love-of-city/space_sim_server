"""Smooth actuator-only motion shared by physical threaded manipulation tasks.

Nothing in this module changes a body pose, joint state, constraint or force.
All feedback uses the raw native sample; smoothing applies only to commands.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics, axis_angle_to_matrix, quaternion_wxyz_to_matrix,
    rotation_matrix_to_vector,
)
from .approach_path import ApproachPosePath
from .drawer_cycle import DrawerCycleConfig
from .motion import solve_pose
from .motion_timing import time_joint_path


@dataclass(frozen=True)
class ThreadedMotionConfig:
    part_body: str = "bolt"
    arm_speed_rad_s: float = .35
    linear_speed_m_s: float = .05
    angular_speed_rad_s: float = .20
    gripper_speed_rad_s: float = .20
    release_open_rad: float = .75
    transport_height_m: float = .245
    pregrasp_distance_m: float = .045
    grasp_approach_speed_m_s: float = .025
    rotation_speed_rad_s: float = .20
    rotation_acceleration_rad_s2: float = .40
    rotation_jerk_rad_s3: float = 1.6
    grip_confirm_s: float = .15
    terminal_confirm_s: float = .20
    verify_hold_s: float = 1.0
    initial_settle_s: float = .25
    stage_timeout_s: float = 120.
    overload_stop_enabled: bool = False

    def __post_init__(self):
        if not self.part_body or not isinstance(self.part_body, str):
            raise ValueError("part_body must be a named physical body")
        if not isinstance(self.overload_stop_enabled, bool) or self.overload_stop_enabled:
            raise ValueError("Threaded demonstrations do not use a wrist overload abort")
        for key, value in vars(self).items():
            if key == "head_offset_m":
                if not math.isfinite(value) or abs(value) > .025:
                    raise ValueError("Invalid signed latch height")
                continue
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{key} must be finite and positive")
        if self.arm_speed_rad_s > .35 or self.linear_speed_m_s > .1:
            raise ValueError("Motion exceeds the shared arm/translation envelope")
        if not 0 < self.release_open_rad <= 1:
            raise ValueError("Invalid release opening")


@dataclass(frozen=True)
class ThreadedSample:
    time_s: float
    joints: np.ndarray
    joint_velocities: np.ndarray
    part_position_body_m: np.ndarray
    part_rotation_body: np.ndarray
    part_velocity_body_m_s: np.ndarray
    part_angular_velocity_body_rad_s: np.ndarray
    wrench: np.ndarray
    wrench_valid: bool
    finger_part_contacts: tuple[int, int] | None = None
    scalar_joints: dict[str, float] = field(default_factory=dict)
    native_qpos: np.ndarray | None = None
    finger_latch_contacts: tuple[int, int] | None = None


def model_body_pose(root, name, base="cubesat_bus"):
    """Body pose at the MJCF reference configuration, relative to the bus."""
    parents = {child: parent for parent in root.iter() for child in parent}
    def absolute(body_name):
        body = root.find(f'.//body[@name="{body_name}"]')
        if body is None:
            raise ValueError(f"Missing body {body_name}")
        p, r = np.zeros(3), np.eye(3)
        while body.tag == "body":
            rb = quaternion_wxyz_to_matrix(np.fromstring(body.get("quat", "1 0 0 0"), sep=" "))
            p = np.fromstring(body.get("pos", "0 0 0"), sep=" ") + rb @ p
            r = rb @ r
            body = parents[body]
        return p, r
    p,r = absolute(name)
    bp,br = absolute(base)
    return br.T@(p-bp),br.T@r


class ThreadedTaskBase:
    def __init__(self, model_path: Path, config: ThreadedMotionConfig):
        self.model_path, self.config = Path(model_path), config
        self.root = ET.parse(model_path).getroot()
        self.chain = SerialChainKinematics.from_mjcf(
            model_path, base_body="cubesat_bus",
            joint_names=tuple(f"joint{i}" for i in range(1, 7)), tool_site="sarm_ee",
        )
        self.lower, self.upper = np.array([
            np.fromstring(self.root.find(f'.//joint[@name="joint{i}"]').get("range"), sep=" ")
            for i in range(1, 7)
        ]).T
        self.stage, self.result, self.reason = "pending", None, ""
        self.reference, self.velocity = np.zeros(8), np.zeros(8)
        self.events = []
        self.max_force_n = self.max_torque_nm = 0.
        self._last_time = None
        self._entered = self._stable = 0.
        self._timeout = config.stage_timeout_s
        self._timing, self._path = None, None
        self._duration = self._motion_s = 0.
        self._gripper_timing = None
        self._opening_elapsed = 0.
        self._grasp_sign = None
        self._grasp_transform = None
        self._command_rate, self._command_acceleration = np.zeros(6), np.zeros(6)
        self._reference_stage = None
        self._published_acceleration = np.zeros(6)
        self._quality = dict(samples=0, max_speed_rad_s=0., max_acceleration_rad_s2=0.,
                             max_jerk_rad_s3=0., max_stage_velocity_jump_rad_s=0.,
                             rotation_samples=0, rotation_latch_samples=0,
                             rotation_max_wrist_axis_error_rad=0.)

    def metrics(self, sample):
        return dict(
            task_name=self.name, task_version=self.version,
            part_body=self.config.part_body,
            part_position_body_m=sample.part_position_body_m.tolist(),
            part_rotation_body=sample.part_rotation_body.tolist(),
            part_velocity_body_m_s=sample.part_velocity_body_m_s.tolist(),
            part_angular_velocity_body_rad_s=sample.part_angular_velocity_body_rad_s.tolist(),
            finger_part_contacts=sample.finger_part_contacts,
            finger_latch_contacts=sample.finger_latch_contacts,
            latch_engaged=self._latch_engaged(sample),
            wrist_axis_error_rad=self._wrist_axis_error(sample),
            scalar_joints=dict(sample.scalar_joints), overload_stop_enabled=False,
            reference_motion_quality=dict(self._quality),
        )

    @staticmethod
    def is_latch_contact(part_body, local_position):
        """The original paired CAD head tabs, excluding hex flats and neck pads."""
        x, y, z = np.asarray(local_position)
        flat = .0194855716 if part_body == "bolt" else .0216506351
        return bool(abs(x) <= .0095 and .0045 <= z <= .0108
                    and flat-.006 <= abs(y) <= flat-.001)

    def _wrist_axis_error(self, sample):
        _, rotation = self.chain.forward(sample.joints[:6])
        return float(np.arccos(np.clip(rotation[:, 2]@sample.part_rotation_body[:, 2], -1., 1.)))

    def _latch_engaged(self, sample):
        if sample.finger_latch_contacts is None or not all(sample.finger_latch_contacts):
            return False
        position, rotation = self.chain.forward(sample.joints[:6])
        goal = sample.part_position_body_m+sample.part_rotation_body@np.asarray(self._latch_offset())
        heading = abs(rotation[:, 0]@sample.part_rotation_body[:, 1])
        return bool(np.linalg.norm(position-goal)<.002
                    and heading>math.cos(.05) and self._wrist_axis_error(sample)<.035)

    def _latch_offset(self):
        return getattr(self.config, "grasp_offset_m", (0., 0., getattr(self.config, "head_offset_m", -.005)))

    def fail(self, reason, sample):
        if self.result is not None:
            return
        self.result, self.reason = "failed", reason
        self.reference[:], self.velocity[:] = sample.joints, 0.
        self.events.append(dict(stage="failed", reason=reason, sim_time_s=sample.time_s, **self.metrics(sample)))

    def _succeed(self, sample):
        self.result, self.reason = "success", ""
        self.velocity[:] = 0.
        self.events.append(dict(stage="success", sim_time_s=sample.time_s, **self.metrics(sample)))

    def _set_stage(self, stage, sample, timeout=None):
        self.stage, self._entered, self._stable = stage, sample.time_s, 0.
        self._timing, self._path = None, None
        self._duration = self._motion_s = 0.
        self._timeout = timeout or self.config.stage_timeout_s
        self.events.append(dict(stage=stage, sim_time_s=sample.time_s, **self.metrics(sample)))

    def _plan_pose(self, position, rotation, opening=None, speed=None,
                   approach=False, path_fn=None, distance=None, angle=None, approach_clearance=None):
        p, r = self.chain.forward(self.reference[:6])
        rotation, position = np.asarray(rotation), np.asarray(position)
        rv = rotation_matrix_to_vector(rotation @ r.T)
        magnitude = float(np.linalg.norm(rv))
        start_opening = float(self.reference[6])
        end_opening = start_opening if opening is None else float(opening)
        approach_path = None
        if approach:
            approach_path = ApproachPosePath(p, r, position, rotation, -rotation[:, 2],
                                             start_opening, self.config.pregrasp_distance_m
                                             if approach_clearance is None else approach_clearance)
            end_opening = 1.
        waypoints = [self.reference[:6].copy()]
        for u in np.linspace(0., 1., 121)[1:]:
            pa = p + u*(position-p)
            ra = axis_angle_to_matrix(rv/magnitude, u*magnitude) @ r if magnitude > 1e-12 else r
            if approach_path is not None:
                pa, ra = approach_path.pose(u)
            elif path_fn is not None:
                pa, ra = path_fn(float(u))
            q = solve_pose(self.chain, pa, ra, waypoints[-1], self.lower, self.upper)
            if np.max(np.abs(q-waypoints[-1])) > .30:
                raise ValueError("Threaded task IK branch discontinuity")
            waypoints.append(q)
        self._path = np.asarray(waypoints)
        cfg = DrawerCycleConfig()
        length = float(np.linalg.norm(position-p)) if distance is None else float(distance)
        turn = magnitude if angle is None else float(angle)
        if approach_path is not None:
            linear, angular, grip = approach_path.derivative_bounds()
            length, turn = linear[0], angular[0]
        self._timing = time_joint_path(
            self._path, length, turn,
            joint_speed=self.config.arm_speed_rad_s,
            linear_speed=speed or self.config.linear_speed_m_s,
            angular_speed=self.config.angular_speed_rad_s,
            linear_acceleration=.15, angular_acceleration=.4,
            joint_acceleration=cfg.joint_acceleration_rad_s2,
            linear_jerk=.8, angular_jerk=1.6, joint_jerk=cfg.joint_jerk_rad_s3,
            slow_distance=.02 if approach else 0.,
            slow_speed=self.config.grasp_approach_speed_m_s,
        )
        if approach_path is not None:
            for bounds, limits in (
                (linear, (self.config.linear_speed_m_s,.15,.8)),
                (angular, (self.config.angular_speed_rad_s,.4,1.6)),
                (grip, (self.config.gripper_speed_rad_s,.4,1.6)),
            ):
                self._timing = self._timing.limit_derivatives(bounds,*limits)
        elif abs(end_opening-start_opening) > 1e-8:
            delta = abs(end_opening-start_opening)
            self._timing = self._timing.limit_derivatives(
                (1.875*delta, 5.78*delta, 60.*delta), self.config.gripper_speed_rad_s,.4,1.6)
        self._timing.path.validate_limits(self.lower,self.upper)
        self._duration, self._motion_s = self._timing.duration, 0.
        self._path_opening = (start_opening,end_opening,approach_path)

    def planned_path_reference(self, elapsed):
        progress = self._timing.at(elapsed)
        start, end, approach = self._path_opening
        u = float(progress)
        opening = approach.opening(u) if approach is not None else start+(end-start)*u**3*(10.-15.*u+6.*u*u)
        return np.r_[self._timing.path.at(u),opening,-opening]

    def _path_step(self, sample, dt):
        if self._timing is None:
            return True
        if np.max(np.abs(self.reference[:6]-sample.joints[:6])) < .08:
            self._motion_s = min(self._duration,self._motion_s+dt)
        self.reference[:] = self.planned_path_reference(self._motion_s)
        p,r = self.chain.forward(sample.joints[:6])
        goal,rr = self.chain.forward(self._path[-1])
        return bool(self._motion_s >= self._duration and np.linalg.norm(p-goal)<.004
                    and np.linalg.norm(rotation_matrix_to_vector(rr@r.T))<.035)

    def _start_opening(self, target, sample=None, speed=None, unload=False):
        cfg = DrawerCycleConfig()
        start = float(self.reference[6])
        waypoints = [[start],[float(target)]]
        if unload and sample is not None and target > start:
            measured = float(np.clip(max(sample.joints[6],-sample.joints[7]),start,target))
            if measured > start+1e-5:
                waypoints = [[start],[measured],[float(target)]]
        self._gripper_timing = time_joint_path(
            waypoints,0.,0.,joint_speed=speed or self.config.gripper_speed_rad_s,
            linear_speed=self.config.linear_speed_m_s,angular_speed=self.config.angular_speed_rad_s,
            linear_acceleration=.15,angular_acceleration=.4,
            joint_acceleration=cfg.gripper_acceleration_rad_s2,joint_jerk=cfg.gripper_jerk_rad_s3)
        self._opening_elapsed=0.

    def _advance_opening(self, dt):
        if self._gripper_timing is None:
            return True
        self._opening_elapsed=min(self._gripper_timing.duration,self._opening_elapsed+dt)
        opening=float(self._gripper_timing.joint_state(self._opening_elapsed)[0][0])
        self.reference[6:]=[opening,-opening]
        return self._opening_elapsed >= self._gripper_timing.duration

    def _grasp_pose(self,sample,closing_local=(0.,1.,0.),approach_local=(0.,0.,1.),offset_local=(0.,0.,.010)):
        closing=sample.part_rotation_body@np.asarray(closing_local)
        approach=sample.part_rotation_body@np.asarray(approach_local)
        closing=closing-approach*(approach@closing)
        closing/=np.linalg.norm(closing)
        if self._grasp_sign is None:
            dot=float(closing@self.chain.forward(sample.joints[:6])[1][:,0])
            # Orthogonal initial headings otherwise pick opposite IK branches
            # between exact preflight and a noisy real initial measurement.
            if abs(dot)<1e-4:
                dot=float(closing[np.argmax(np.abs(closing))])
            self._grasp_sign=1. if dot>=0 else -1.
        closing*=self._grasp_sign
        return (sample.part_position_body_m+sample.part_rotation_body@np.asarray(offset_local),
                np.column_stack([closing,np.cross(approach,closing),approach]))

    def _capture_grasp(self,sample):
        p,r=self.chain.forward(sample.joints[:6])
        self._grasp_transform=(r.T@(sample.part_position_body_m-p),r.T@sample.part_rotation_body)

    def _track_part(self,position,rotation,sample,dt,linear_velocity=None,angular_velocity=None,
                    *, measured_feedback=False):
        p,r=self.chain.forward(sample.joints[:6])
        if self._grasp_transform is None:
            relp=r.T@(sample.part_position_body_m-p)
            relr=r.T@sample.part_rotation_body
        else:
            # A measured bilateral grasp defines a single rigid reference.
            # Re-estimating it every frame would silently absorb slip and send
            # the wrist chasing ever larger offsets after contact loss.
            relp,relr=self._grasp_transform
        toolr=np.asarray(rotation)@relr.T
        goal=np.asarray(position)-toolr@relp
        velocity=np.zeros(3) if linear_velocity is None else np.asarray(linear_velocity).copy()
        if angular_velocity is not None:
            velocity-=np.cross(angular_velocity,toolr@relp)
        self._follow_pose(goal,toolr,sample,dt,velocity,angular_velocity,
                          measured_feedback=measured_feedback)

    def _follow_pose(self,position,rotation,sample,dt,linear_velocity=None,angular_velocity=None,
                     *, measured_feedback=False):
        if dt<=0:
            return
        # One finite-response reference generator owns the command. Open-jaw
        # tracking and captive mechanism motion use command FK. A free screw
        # helix explicitly uses measured FK to reject sustained servo/contact
        # deflection, so spinning above a thread mouth cannot count as advance.
        # Both modes retain the same acceleration, jerk and reference-lead
        # limits; neither mode overwrites a running timed path.
        feedback_joints=sample.joints[:6] if measured_feedback else self.reference[:6]
        p,r=self.chain.forward(feedback_joints)
        error=np.r_[np.asarray(position)-p,.2*rotation_matrix_to_vector(np.asarray(rotation)@r.T)]
        twist=4.*error
        if linear_velocity is not None: twist[:3]+=linear_velocity
        if angular_velocity is not None: twist[3:]+=.2*np.asarray(angular_velocity)
        jac=self.chain.jacobian(feedback_joints).copy()
        jac[3:]*=.2
        dq=jac.T@np.linalg.solve(jac@jac.T+1e-6*np.eye(6),twist)
        dq*=min(1.,self.config.arm_speed_rad_s/max(np.max(np.abs(dq)),1e-12))
        if measured_feedback:
            # Brake toward the allowed lead before the final hard position
            # envelope. Clipping the integrated position at a loaded screw
            # mouth imposed an abrupt velocity change despite the jerk limit.
            # Include actual joint velocity so a moving envelope does not
            # unnecessarily brake normal tracking.
            lead=sample.joints[:6]-self.reference[:6]
            dq=np.clip(dq,sample.joint_velocities[:6]+4.*(lead-.035),
                          sample.joint_velocities[:6]+4.*(lead+.035))
        # A finite response time is essential with a jerk-limited integrator.
        # Dividing the velocity error by dt requested bang-bang acceleration
        # at 200 Hz and sustained a ~0.03 rad/s limit cycle even for a fixed
        # pose. The native servo cannot settle against that moving reference.
        acceleration=np.clip(8.*(dq-self._command_rate),-.7,.7)
        self._command_acceleration+=np.clip(acceleration-self._command_acceleration,-8.*dt,8.*dt)
        self._command_rate+=self._command_acceleration*dt
        self._command_rate=np.clip(self._command_rate,-self.config.arm_speed_rad_s,self.config.arm_speed_rad_s)
        candidate=self.reference[:6]+self._command_rate*dt
        self.reference[:6]=np.clip(np.clip(candidate,sample.joints[:6]-.06,sample.joints[:6]+.06),self.lower,self.upper)

    def _step_start(self,sample):
        if self._last_time is not None and sample.time_s<self._last_time:
            raise ValueError("Task time must be monotonic")
        dt=0. if self._last_time is None else sample.time_s-self._last_time
        self._last_time=sample.time_s
        if self.stage=="pending": self.reference[:]=sample.joints
        old=self.reference.copy()
        measured = np.concatenate((
            np.asarray(sample.joints).ravel(), np.asarray(sample.joint_velocities).ravel(),
            np.asarray(sample.part_position_body_m).ravel(), np.asarray(sample.part_rotation_body).ravel(),
            np.asarray(sample.part_velocity_body_m_s).ravel(),
            np.asarray(sample.part_angular_velocity_body_rad_s).ravel(),
            np.asarray(sample.wrench).ravel(), np.asarray(tuple(sample.scalar_joints.values()), dtype=float),
        ))
        if not np.isfinite(measured).all():
            self.fail("nonfinite_native_sample",sample)
        self.max_force_n=max(self.max_force_n,float(np.linalg.norm(sample.wrench[:3])))
        self.max_torque_nm=max(self.max_torque_nm,float(np.linalg.norm(sample.wrench[3:])))
        if self.result is None and self.stage!="pending" and sample.time_s-self._entered>self._timeout:
            self.fail("stage_timeout",sample)
        if not sample.wrench_valid and sample.time_s>.2:
            self.fail("wrist_sensor_unavailable",sample)
        return dt,old

    def _step_finish(self,sample,old,dt):
        previous_rate = self.velocity[:6].copy()
        previous_acceleration = self._published_acceleration.copy()
        self.velocity[:]=(self.reference-old)/dt if dt>0 and self.result is None else 0.
        # Paths, holds and feedback must share the state of the reference that
        # was actually published. Retaining a previous feedback stroke here
        # restarted the wrist at that old speed after a fully stopped rewind.
        self._command_rate[:] = self.velocity[:6]
        if dt > 0:
            self._command_acceleration[:] = (self._command_rate-previous_rate)/dt
            if self.result is None:
                quality = self._quality
                quality["samples"] += 1
                for key, value in (("max_speed_rad_s", self._command_rate),
                                   ("max_acceleration_rad_s2", self._command_acceleration),
                                   ("max_jerk_rad_s3", (self._command_acceleration-previous_acceleration)/dt)):
                    quality[key] = max(quality[key], float(np.max(np.abs(value))))
                if self._reference_stage is not None and self._reference_stage != self.stage:
                    quality["max_stage_velocity_jump_rad_s"] = max(
                        quality["max_stage_velocity_jump_rad_s"], float(np.max(np.abs(self._command_rate-previous_rate))))
                if self.stage in ("unscrew", "screw_in", "loosen", "tighten"):
                    quality["rotation_samples"] += 1
                    quality["rotation_latch_samples"] += int(self._latch_engaged(sample))
                    quality["rotation_max_wrist_axis_error_rad"] = max(
                        quality["rotation_max_wrist_axis_error_rad"], self._wrist_axis_error(sample))
        self._reference_stage = self.stage
        self._published_acceleration[:] = self._command_acceleration
        return self.reference.copy(),self.velocity.copy()

    def _motion_stopped(self, sample, tolerance=.002):
        return bool(np.max(np.abs(self._command_rate)) < tolerance
                    and np.max(np.abs(sample.joint_velocities[:6])) < .012)

    def preflight_paths(self,initial_q):
        raise NotImplementedError("Threaded tasks must provide geometry-aware preflight paths")
