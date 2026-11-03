"""Tracked free-body pick/transfer/insert tasks; actuator references only."""

from __future__ import annotations
from dataclasses import dataclass, replace
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
from simulation.scripted_tasks.plug_profiles import SCENE_PLUG_PROFILES as PLUG_PROFILES


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
        preheld_lock = root.find('./equality/weld[@name="preheld_capture_lock"]')
        if (body.get('name') == 'capture_target' and preheld_lock is not None
                and preheld_lock.get('body1') == 'gripper'
                and preheld_lock.get('body2') == 'capture_target'
                and preheld_lock.get('active', 'true') == 'true'):
            # Initial planning geometry only. The target remains a free body;
            # measured full-state collision checks still govern every motion.
            rb = quaternion_wxyz_to_matrix(np.fromstring(body.get('quat', '1 0 0 0'), sep=' '))
            position = np.fromstring(body.get('pos', '0 0 0'), sep=' ') + rb @ position
            rotation = rb @ rotation
            base_body = root.find(f'./worldbody/body[@name="{base}"]')
            if base_body is None or parents[body].tag != 'worldbody':
                raise ValueError('Preheld initial geometry requires independent world bodies')
            base_rotation = quaternion_wxyz_to_matrix(np.fromstring(base_body.get('quat', '1 0 0 0'), sep=' '))
            base_position = np.fromstring(base_body.get('pos', '0 0 0'), sep=' ')
            return base_rotation.T @ (position - base_position), base_rotation.T @ rotation
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
    transport_clearance_m: float | None = None
    part_velocity_damping: float | None = None
    transfer_offset_xy_m: tuple[float, float] = (0., 0.)
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
    seat_clearance_m: float | None = None
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
        if self.seat_clearance_m is None:
            object.__setattr__(self,'seat_clearance_m',profile.seat_clearance_m)
        if self.target_site not in (
            profile.prefix + "_insert_40p1",
            profile.target_site,
            "service_" + profile.prefix + "_insert_40p1",
            "service_" + profile.target_site,
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
        if self.seat_clearance_m>=self.seat_tolerance_m:
            raise ValueError('Seating command clearance must remain inside the unchanged seating tolerance')
        if self.part_velocity_damping is not None and self.part_velocity_damping > 5.:
            raise ValueError('Part velocity damping must be in (0,5] when enabled')
        offset=np.asarray(self.transfer_offset_xy_m)
        if offset.shape != (2,) or not np.isfinite(offset).all() or np.any(np.abs(offset)>.25):
            raise ValueError('Transfer waypoint offset requires two finite values within +/-0.25m')
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
    site_poses: dict | None = None


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
        self.model_path = Path(model_path)
        self.config = config or PlugInsertConfig()
        if (self.config.part_velocity_damping is not None or any(self.config.transfer_offset_xy_m)
                or self.config.release_speed_rad_s != .05):
            self.version = 'plug-insert/3-scene-variants'
        if self.config.part_velocity_damping is not None:
            self.version = 'plug-insert/4-axial-damping'
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
        from simulation.scene_library.frames import site_attachment
        self.hole_owner = site_attachment(root, self.config.target_site)[0]
        self.pickup, self.pickup_rotation = fixed_site_pose(root, self.profile.prefix + "_pickup")
        self.pickup_owner = site_attachment(root, self.profile.prefix + "_pickup")[0]
        if self.config.transport_clearance_m is not None and (
            np.allclose(hole_r[:, 2], [0., 0., 1.])
            and np.allclose(self.pickup_rotation[:, 2], [0., 0., 1.])
        ):
            # Explicit opt-in for raised installation surfaces; legacy height
            # remains exactly unchanged for existing baseline configurations.
            self.config = replace(self.config, transport_height_m=float(
                max(self.hole[2], self.pickup[2]) + self.config.transport_clearance_m))
        self.hole_rotation = hole_r
        if np.linalg.norm(self.hole - self.pickup) < 0.05:
            raise ValueError("The assembly target must be a separate hole")
        if self.config.transport_clearance_m is None and not (
            np.allclose(hole_r[:, 2], [0., 0., 1.])
            and np.allclose(self.pickup_rotation[:, 2], [0., 0., 1.])
        ):
            raise ValueError("Non-horizontal task faces require explicit transport_clearance_m")
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
        self._seat_recovery = False
        self._seat_unloaded = True
        self._seat_axial_enabled = False

    def _axis_coordinate(self, position, *, pickup=False):
        rotation = self.pickup_rotation if pickup else self.hole_rotation
        return float(np.asarray(position) @ rotation[:, 2])

    def _axis_goal(self, coordinate, *, pickup=False):
        """Point on the measured socket axis; coordinate is along its normal."""
        origin = self.pickup if pickup else self.hole
        normal = (self.pickup_rotation if pickup else self.hole_rotation)[:, 2]
        return origin + normal * (coordinate - float(origin @ normal))

    def _transport_coordinate(self, *, pickup=False):
        if self.config.transport_clearance_m is None:
            # Legacy horizontal layouts retain their absolute transport plane.
            return self.config.transport_height_m
        origin = self.pickup if pickup else self.hole
        normal = (self.pickup_rotation if pickup else self.hole_rotation)[:, 2]
        height = float(origin @ normal)
        if self.pickup_rotation[:, 2] @ self.hole_rotation[:, 2] > 1 - 1e-8:
            height = max(float(self.pickup @ normal), float(self.hole @ normal))
        return height + self.config.transport_clearance_m

    def _clear_of_pickup(self, sample):
        start = getattr(self, '_initial_pickup_clearance',
                        self._axis_coordinate(self._initial_position-self.pickup, pickup=True))
        return self._axis_coordinate(sample.plug_position_body_m-self.pickup, pickup=True)-start > .12

    def _withdrawal_tangent(self, rotation):
        normal = self.hole_rotation[:, 2]
        direction = rotation[:, 1] - normal * float(rotation[:, 1] @ normal)
        if np.linalg.norm(direction) < 1e-8:
            direction = self.hole_rotation[:, 0].copy()
        direction /= np.linalg.norm(direction)
        if direction @ self.hole_rotation[:, 0] < 0:
            direction = -direction
        return direction

    def metrics(self, sample):
        axis = sample.plug_rotation_body[:, 2]
        tip = sample.plug_position_body_m + axis * self.config.shaft_tip_local_m
        flange = sample.plug_position_body_m + axis * self.config.flange_local_m
        normal = self.hole_rotation[:, 2]
        axial = float(axis @ normal)
        plane = (
            sample.plug_position_body_m
            + axis * (((self.hole - sample.plug_position_body_m) @ normal) / axial)
            if abs(axial) > 0.1
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
                        self.hole_rotation @ np.diag([1.0, -1.0, -1.0]) @ sample.plug_rotation_body.T
                    )
                )
            ),
            target_site=self.config.target_site,
            target_position_body_m=self.hole.tolist(),
            radial_error_m=float(np.linalg.norm(plane - self.hole)),
            axis_error_rad=float(np.arccos(np.clip(-axial, -1, 1))),
            insertion_depth_m=float((self.hole - tip) @ normal),
            seat_gap_m=float((flange - self.hole) @ normal),
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
            normal = self.pickup_rotation[:, 2]
            goal = p + normal * max(0., self._transport_coordinate(pickup=True)-float(p @ normal))
        elif stage == "above_pickup":
            normal = self.pickup_rotation[:, 2]
            goal = self.grasp + normal * (self._transport_coordinate(pickup=True)-float(self.grasp @ normal))
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
            normal = self.pickup_rotation[:, 2]
            goal = p + normal * (self._transport_coordinate(pickup=True)-float(p @ normal))
        elif stage == "transfer":
            desired = self._axis_goal(self._transport_coordinate())
            desired += self.hole_rotation[:, :2] @ np.asarray(self.config.transfer_offset_xy_m)
            goal, rotation = self._tool_for_plug(sample, desired, self.plug_rotation)
        elif stage == "above_hole":
            desired = self.hole + self.hole_rotation[:, 2] * (self.config.shaft_tip_local_m + .012)
            goal, rotation = self._tool_for_plug(sample, desired, self.plug_rotation)
        elif stage in ("align_hole", "insert", "seat", "release"):
            self._plug_goal = self.hole + self.hole_rotation[:, 2] * (self.config.shaft_tip_local_m + .012)
            if stage in ("insert", "seat", "release"):
                self._plug_goal = self._axis_goal(self._axis_coordinate(sample.plug_position_body_m))
            self._seat_z = self._axis_coordinate(self.hole) + self.config.flange_local_m + self.config.seat_clearance_m
            if stage=='seat':
                self._seat_recovery=abs(self._axis_coordinate(sample.plug_position_body_m)-self._seat_z)>.00015
                self._seat_unloaded=not self._seat_recovery
                self._seat_axial_enabled=False
            if stage == "release":
                self._plug_goal = self._axis_goal(self._seat_z)
            if stage=='release' or (stage=='seat' and self._seat_recovery):
                self._begin_reference_unload(sample)
        elif stage == "retract":
            # Withdraw between the open fingers, along their tangential direction.
            # Vertical arm recoil otherwise pulls the free spacecraft away from the plug.
            direction = self._withdrawal_tangent(r)
            goal = p + 0.12 * direction
        if stage == "lift" and self.profile.feedback_extraction:
            # Recenter and untwist the keyed shaft in its pickup guide before
            # raising it. A fixed wrist trajectory can jam a tilted free part.
            self._plug_goal = self._axis_goal(self._axis_coordinate(sample.plug_position_body_m, pickup=True), pickup=True)
            goal = None
        if goal is not None:
            self._plan(goal, rotation)
        self._nominal_goal = None if self._goal is None else self._goal.copy()
        self._nominal_rotation = (
            self._rotation.copy() if hasattr(self, "_rotation") else r.copy()
        )
        self._timeout = max(30.0, 2.5 * self._duration + 15.0)
        if stage in ("insert", "verify"):
            self._timeout = 90.0
        elif stage=='lift' and self.profile.feedback_extraction:
            distance=max(0.,self._transport_coordinate(pickup=True)-self._axis_coordinate(sample.plug_position_body_m, pickup=True))
            self._timeout=max(90.,3*distance/.006+30.)
        if stage=='seat' and self._seat_recovery:
            # A held recipient deflects while the unchanged 20 N retention
            # threshold is overcome. Allow bounded low-speed recovery to finish;
            # keep all pose, release, velocity and wrist-load gates unchanged.
            self._timeout=max(self._timeout,90. if self.hole_owner != 'cubesat_bus' else 60.)
        if stage == 'release':
            opening_time = max(0., self.config.release_open_rad - self.reference[6]) / self.config.release_speed_rad_s
            self._timeout = max(self._timeout, self._release_unload_duration + opening_time + 15.)

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

    def _advance_extraction_goal(self,sample,dt):
        normal = self.pickup_rotation[:, 2]
        desired_rotation = self.pickup_rotation @ np.diag([1., -1., -1.])
        offset = sample.plug_position_body_m - self.pickup
        lateral = offset - normal * float(offset @ normal)
        axis_error = np.arccos(np.clip(-sample.plug_rotation_body[:, 2] @ normal, -1., 1.))
        orientation_error = np.linalg.norm(rotation_matrix_to_vector(desired_rotation @ sample.plug_rotation_body.T))
        aligned=(np.linalg.norm(lateral)<.00025 and axis_error<.002
                 and (not self.profile.keyed or orientation_error<.003))
        lightly_loaded=(np.linalg.norm(sample.wrench[:3])<.75*self.config.force_limit_n
                        and np.linalg.norm(sample.wrench[3:])<.75*self.config.torque_limit_nm)
        lead=.001 if aligned and lightly_loaded else 0.
        coordinate=min(self._transport_coordinate(pickup=True),
                       self._axis_coordinate(self._plug_goal, pickup=True)+.006*dt,
                       self._axis_coordinate(sample.plug_position_body_m, pickup=True)+lead)
        self._plug_goal = self._axis_goal(coordinate, pickup=True)

    def _advance_insertion_goal(self, sample, dt):
        next_z = max(
            self._seat_z, self._axis_coordinate(self._plug_goal) - self.config.insert_speed_m_s * dt
        )
        metrics = self.metrics(sample)
        aligned = (
            metrics["radial_error_m"] < self.config.radial_tolerance_m / 2
            and metrics["axis_error_rad"] < 0.002
            and (not self.profile.keyed or metrics['orientation_error_rad']<.004)
        )
        lightly_loaded = (
            np.linalg.norm(sample.wrench[:3]) < 0.75 * self.config.force_limit_n
            and np.linalg.norm(sample.wrench[3:]) < 0.75 * self.config.torque_limit_nm
        )
        # Contact stalls the part while the outer position integrator can
        # keep pushing. Unqueue descent until alignment/load recovers.
        lead = 0.001 if aligned and lightly_loaded else 0.0
        next_z = max(next_z, self._axis_coordinate(sample.plug_position_body_m) - lead)
        self._plug_goal = self._axis_goal(next_z)

    def _begin_reference_unload(self,sample):
        self._release_from=self.reference[:6].copy()
        self._release_hold=sample.joints[:6].copy()
        self._release_unload_duration=max(1.,1.875*np.max(np.abs(self._release_from-self._release_hold))/self.config.arm_speed_rad_s)

    def _unload_before_release(self, sample):
        elapsed = sample.time_s - self._entered
        progress = np.clip(
            elapsed / self._release_unload_duration, 0.0, 1.0
        )
        blend = progress**3 * (10.0 - 15.0 * progress + 6.0 * progress**2)
        # The insertion reference contains load-supporting servo deflection.
        # Remove it smoothly while still gripping, before its load disappears.
        self.reference[:6] = self._release_from + blend * (
            self._release_hold - self._release_from
        )
        return elapsed <= self._release_unload_duration

    def _part_feedback(self, sample, dt):
        p, r = self.chain.forward(sample.joints[:6])
        target, rotation = self._tool_for_plug(
            sample, self._plug_goal,
            self.pickup_rotation @ np.diag([1., -1., -1.]) if self.stage == 'lift' else self.plug_rotation
        )
        error = np.r_[target - p, 0.2 * rotation_matrix_to_vector(rotation @ r.T)]
        jac = self.chain.jacobian(sample.joints[:6])
        jac[3:] *= 0.2
        correction = 3.0 * error
        if self.config.part_velocity_damping is not None and self.stage in ('insert', 'seat'):
            # Brake only the insertion axis. Direct angular-velocity feedback
            # excited contact chatter in the rejected six-axis prototype.
            # Filter the measured axial speed; retain the original orientation
            # controller and all raw observations for independent scoring.
            alpha = dt / (.05 + dt)
            previous = getattr(self, '_filtered_axial_speed', 0.)
            normal = self.hole_rotation[:, 2]
            self._filtered_axial_speed = previous + alpha * (float(sample.plug_velocity_body_m_s @ normal) - previous)
            correction[:3] -= normal * self.config.part_velocity_damping * self._filtered_axial_speed
        else:
            self._filtered_axial_speed = 0.
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(6), correction)
        dq *= min(1.0, self.config.arm_speed_rad_s / max(np.max(np.abs(dq)), 1e-12))
        self.reference[:6] = np.clip(
            self.reference[:6] + dt * dq, self.lower, self.upper
        )

    def step(self, sample: PlugSample):
        if sample.site_poses:
            hole, hole_r = sample.site_poses['hole']
            if getattr(self, '_plug_goal', None) is not None:
                old_p, old_r = ((self.pickup, self.pickup_rotation) if self.stage == 'lift'
                                else (self.hole, self.hole_rotation))
                new_p, new_r = sample.site_poses['pickup'] if self.stage == 'lift' else (hole, hole_r)
                self._plug_goal = new_p + new_r @ old_r.T @ (self._plug_goal-old_p)
            self.hole, self.hole_rotation = hole.copy(), hole_r.copy()
            self.pickup, self.pickup_rotation = sample.site_poses['pickup']
            if getattr(self, '_seat_z', None) is not None:
                self._seat_z = self._axis_coordinate(self.hole) + self.config.flange_local_m + self.config.seat_clearance_m
            self.plug_rotation = hole_r @ np.diag([1., -1., -1.])
        if self._last_time is not None and sample.time_s < self._last_time:
            raise ValueError("Task time must be monotonic")
        dt = 0.0 if self._last_time is None else sample.time_s - self._last_time
        self._last_time = sample.time_s
        if self.result is not None:
            return self.reference.copy(), np.zeros(8)
        if self.stage == "pending":
            self.reference[:] = sample.joints
            self._initial_position = sample.plug_position_body_m.copy()
            self._initial_pickup_clearance = self._axis_coordinate(self._initial_position-self.pickup, pickup=True)
            self.plug_rotation = self.hole_rotation @ np.diag([1.0, -1.0, -1.0])
            self.grasp = (
                sample.plug_position_body_m
                + sample.plug_rotation_body @ np.asarray(self.config.grasp_offset_m)
            )
            # Closing axis across the head; approach along the donor normal.
            _, current_r = self.chain.forward(sample.joints[:6])
            closing = self.pickup_rotation[:, 0].copy()
            if closing @ current_r[:, 0] < 0:
                closing = -closing
            approach = -self.pickup_rotation[:, 2]
            self.grasp_rotation = np.column_stack(
                [closing, np.cross(approach, closing), approach]
            )
            self._approach_rotation = self.grasp_rotation.copy()
            displacement = sample.plug_position_body_m-self.pickup
            lateral = displacement - self.pickup_rotation[:, 2] * float(displacement @ self.pickup_rotation[:, 2])
            if np.linalg.norm(lateral) > 0.05:
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
            if getattr(self, "environment_planning", False) and self._timing is not None:
                self.reference[:] = self.path_reference(self._timing.at(self._motion_s))
            else:
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
        unloading = (
            self.stage == "release"
            and self._unload_before_release(sample)
        )
        if self.stage=='seat' and self._seat_recovery and not self._seat_unloaded:
            unloading=self._unload_before_release(sample)
            self._seat_unloaded=not unloading
        if self.stage in ("open", "grip", "release") and not unloading:
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
            self._advance_extraction_goal(sample,dt)
            self._part_feedback(sample, dt)
        if self.stage in ("align_hole", "insert", "seat"):
            self._plug_goal = self._axis_goal(self._axis_coordinate(self._plug_goal))
            if self.stage == "insert":
                self._advance_insertion_goal(sample, dt)
            if self.stage == "seat":
                self._plug_goal = self._axis_goal(self._seat_z)
                if self._seat_recovery:
                    if self._seat_axial_enabled:
                        self._seat_axial_enabled=(metrics['radial_error_m']<.0002 and metrics['axis_error_rad']<.0015
                            and (not self.profile.keyed or metrics['orientation_error_rad']<.003))
                    else:
                        self._seat_axial_enabled=(metrics['radial_error_m']<.00015 and metrics['axis_error_rad']<.001
                            and (not self.profile.keyed or metrics['orientation_error_rad']<.002))
                    lightly_loaded=force<.75*self.config.force_limit_n and torque<.75*self.config.torque_limit_nm
                    measured = self._axis_coordinate(sample.plug_position_body_m)
                    coordinate = (min(self._seat_z, measured+.0005)
                                  if self._seat_axial_enabled and lightly_loaded else measured)
                    self._plug_goal = self._axis_goal(coordinate)
            if not unloading:self._part_feedback(sample, dt)
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
                self._ever_lifted = self._clear_of_pickup(sample)
                if not self._ever_lifted:
                    self.fail("plug_not_lifted", sample)
                    ready = False
        elif self.stage == "lift" and self.profile.feedback_extraction:
            ready = (
                self._axis_coordinate(self._plug_goal, pickup=True) >= self._transport_coordinate(pickup=True)-1e-12
                and np.linalg.norm(sample.plug_position_body_m - self._plug_goal)
                < 0.004
            )
            if ready:
                self._ever_lifted = self._clear_of_pickup(sample)
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
                elapsed > 6.0 and self.reference[6] <= self._closed_reference+0.001 and sample.joints[6] > 0.025
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
            if self.stage=='seat':
                # The broad task goal region alone does not prove that the
                # measured part reached the collision-checked release pose.
                ready=ready and not unloading and abs(self._axis_coordinate(sample.plug_position_body_m)-self._seat_z)<.00015
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
                ready = ready and self._axis_coordinate(self._plug_goal) <= self._seat_z + 1e-6
            if self.stage == "verify":
                ready = (
                    ready
                    and self._ever_lifted
                    and sample.joints[6] > self.config.release_open_rad - 0.03
                    and np.linalg.norm(self.hole_rotation[:, :2].T @ (p-self.hole)) > 0.09
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
