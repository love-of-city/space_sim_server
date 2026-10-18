"""Read-only success predicates for policy-driven physical tasks.

Registry entries consume authoritative samples, never schedule task controllers,
compute actuator commands, or infer progress from scripted phases.
"""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix

RENDER_PERIOD_NS = 20_000_000


class DrawerCycleSuccess:
    """Conservative physical witnesses for the calibrated drawer cycle.

    The position, closure, latch and dwell tolerances come from DrawerCycleConfig.
    The measured arm speed/orientation, drawer speed, and grip thresholds match
    DrawerLiftTask/DrawerCycleTask. Planner completion and script phases play no role.
    """

    def __init__(self, model_path: Path, config: DrawerCycleConfig | None = None):
        self.config = config or DrawerCycleConfig()
        root = ET.parse(model_path).getroot()
        bus = root.find('.//body[@name="cubesat_bus"]')
        drawer = bus.find('body[@name="drawer_base"]') if bus is not None else None
        buckle = drawer.find('.//body[@name="drawer_buckle"]') if drawer is not None else None
        slide = drawer.find('joint[@name="drawer_base_slide"]') if drawer is not None else None
        if drawer is None or buckle is None or slide is None or slide.get("type") != "slide":
            raise ValueError("success monitor requires the calibrated drawer, buckle and slide")
        if any(name in drawer.attrib for name in ("euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("success monitor requires the calibrated drawer quaternion")
        self.rotation = quaternion_wxyz_to_matrix(np.fromstring(drawer.get("quat", "1 0 0 0"), sep=" "))
        self.axis = np.fromstring(slide.get("axis", ""), sep=" ")
        origin = np.fromstring(drawer.get("pos", ""), sep=" ")
        if self.axis.shape != (3,) or origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError("invalid drawer geometry")
        self.axis = self.rotation @ self.axis
        if not np.allclose(self.rotation, np.diag([-1., 1., -1.])) or not np.allclose(self.axis, [0., 0., 1.]):
            raise ValueError("drawer geometry differs from the calibrated upward slide")
        self.grip_origin = origin + self.rotation @ np.asarray(self.config.grip_offset_m)
        self.reset(None)

    @staticmethod
    def _read(sample):
        scalar = np.asarray([sample["drawer_position_m"], sample["drawer_velocity_m_s"],
                             sample["buckle_rad"], sample["gripper_drive_position_rad"]], dtype=float)
        p = np.asarray(sample["end_effector_position_body_m"], dtype=float)
        rotation = np.asarray(sample["end_effector_rotation_body"], dtype=float)
        velocity = np.asarray(sample["arm_joint_velocity_rad_s"], dtype=float)
        contacts = sample["finger_drawer_contacts"]
        if (scalar.shape != (4,) or p.shape != (3,) or rotation.shape != (3, 3) or velocity.shape != (6,)
                or not all(np.isfinite(v).all() for v in (scalar, p, rotation, velocity))
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)
                or not isinstance(contacts, (tuple, list)) or len(contacts) != 2
                or any(type(v) is not int or v < 0 for v in contacts)):
            raise ValueError("incomplete or non-finite physical task sample")
        return (*scalar, p, rotation, velocity, contacts)

    def reset(self, initial: dict | None) -> None:
        self.active = initial is not None
        self.error = None
        self.last_time_ns = None
        self.initial_drawer = self.initial_buckle = None
        self.hover = self.grasp_rotation = None
        self.grasp_verified = self.open_verified = self.seated_verified = self.released = False
        self.peak = 0.
        self._dwell = {name: 0 for name in ("grasp", "lift", "seat", "final")}
        self._previous = {name: False for name in self._dwell}
        self._closed = self._success = False
        self._hover_error = None
        self._jaw_alignment_error = None
        if initial is None:
            return
        try:
            drawer, _, buckle, _, _, _, _, _ = self._read(initial)
            stamp = initial["sim_time_ns"]
            if type(stamp) is not int or stamp < 0 or abs(drawer) > .005:
                raise ValueError("success monitor requires an initially closed drawer and valid simulation time")
            self.last_time_ns = stamp
            self.initial_drawer, self.initial_buckle = float(drawer), float(buckle)
            self.hover = self.grip_origin + self.axis * (drawer + self.config.clearance_m)
            # The delivered parallel jaws form an unsigned closing axis. An
            # initial pose almost perpendicular to it cannot choose a physical
            # "correct finger" by the sign of a floating-point dot product.
            closing = self.rotation[:, 1].copy()
            down = -self.axis
            self.grasp_rotation = np.column_stack([closing, np.cross(down, closing), down])
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.error = str(error)

    def _stable(self, name, condition, dt_ns, seconds):
        self._dwell[name] = self._dwell[name] + dt_ns if condition and self._previous[name] else 0
        self._previous[name] = bool(condition)
        return self._dwell[name] >= round(seconds * 1e9)

    def update(self, sample: dict) -> dict:
        if not self.active or self.error:
            return self.status()
        try:
            stamp = sample["sim_time_ns"]
            if type(stamp) is not int:
                raise ValueError("invalid physical task timestamp")
            dt_ns = stamp - self.last_time_ns
            if dt_ns == 0:
                return self.status()  # Duplicate snapshots never add dwell or new evidence.
            if dt_ns != RENDER_PERIOD_NS:
                raise ValueError("physical task samples must be consecutive 50 Hz render frames")
            drawer, speed, buckle, opening, p, rotation, velocity, contacts = self._read(sample)
            self.last_time_ns = stamp
            displacement = float(drawer - self.initial_drawer)
            self.peak = max(self.peak, displacement)
            seated = abs(displacement) <= self.config.close_tolerance_m and abs(speed) < .003
            self._closed = seated and abs(buckle - self.initial_buckle) <= self.config.buckle_tolerance_rad
            at_handle = np.linalg.norm(p - (self.grip_origin + self.axis * drawer)) < self.config.loaded_position_tolerance_m
            still = np.max(np.abs(velocity)) < .015
            # Swapping the two opposing jaws is a half-turn about tool-down:
            # both transverse axes reverse, but down remains directed. Test the
            # full principal angle against these two frames, not abs(trace) or
            # abs(tool-down), so sideways/upward tools and 90-degree yaw fail.
            jaw_swapped = self.grasp_rotation * np.array([-1., -1., 1.])
            cosine = max((np.trace(reference @ rotation.T) - 1.) / 2.
                         for reference in (self.grasp_rotation, jaw_swapped))
            self._jaw_alignment_error = float(np.arccos(np.clip(cosine, -1., 1.)))
            aligned = self._jaw_alignment_error < .025
            grasp = bool(all(contacts) and .025 < opening < .95 and at_handle and still and aligned)
            self.grasp_verified |= self._stable("grasp", grasp, dt_ns, 1.)
            # The loaded tolerance is 5 mm in the delivered config, matching the
            # existing lift witness. Extra opening is not mistaken for failure.
            lifted = (self.grasp_verified and grasp and abs(speed) < .003
                      and displacement >= self.config.lift_m - self.config.loaded_position_tolerance_m)
            self.open_verified |= self._stable("lift", lifted, dt_ns, self.config.hold_s)
            self.seated_verified |= self._stable("seat", self.open_verified and grasp and seated,
                                                 dt_ns, self.config.verify_hold_s)
            if self.seated_verified and seated and opening > .95 and still:
                self.released = True
            self._hover_error = float(np.linalg.norm(p - self.hover))
            final = (self.open_verified and self.seated_verified and self.released and self._closed
                     and opening > .95 and self._hover_error < self.config.position_tolerance_m
                     and still and aligned)
            self._success = self._stable("final", final, dt_ns, self.config.verify_hold_s)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.error = str(error)
            self._success = False
        return self.status()

    def status(self) -> dict:
        available = self.active and self.error is None
        return dict(task_name="drawer_cycle", task_version="drawer-policy-success/2",
                    success_available=bool(available), success=bool(available and self._success),
                    success_unavailable_reason=self.error or (None if self.active else "no active drawer policy session"),
                    grasp_verified=bool(self.grasp_verified), open_verified=bool(self.open_verified),
                    seated_before_release_verified=bool(self.seated_verified), release_verified=bool(self.released),
                    peak_drawer_displacement_m=self.peak, drawer_closed=bool(self._closed),
                    verification_stable_s=self._dwell["final"] * 1e-9,
                    expected_hover_position_body_m=self.hover.tolist() if self.hover is not None else None,
                    hover_position_error_m=self._hover_error,
                    jaw_alignment_error_rad=self._jaw_alignment_error,
                    lift_threshold_m=self.config.lift_m - self.config.loaded_position_tolerance_m)


TASK_SUCCESS_MONITORS = {"drawer_cycle": DrawerCycleSuccess}


class PolicyTaskSuccessProvider:
    """Select a read-only monitor; unsupported geometry/tasks never claim success."""

    def __init__(self, model_path: Path | None):
        self.monitor = None
        self.task_name = None
        self.error = "model geometry unavailable"
        if model_path is not None:
            try:
                self.monitor = TASK_SUCCESS_MONITORS["drawer_cycle"](model_path)
                self.error = None
            except (ValueError, KeyError, OSError, ET.ParseError) as error:
                self.error = str(error)

    def reset(self, task_name: str | None, initial: dict | None) -> None:
        self.task_name = task_name
        if self.monitor is not None:
            self.monitor.reset(initial if task_name == "drawer_cycle" else None)

    def update(self, sample: dict) -> dict:
        if self.task_name == "drawer_cycle" and self.monitor is not None:
            return self.monitor.update(sample)
        return dict(task_name=self.task_name or "", success_available=False, success=False,
                    success_unavailable_reason=self.error if self.task_name else "no physical task selected")
