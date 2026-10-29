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
from simulation.evaluation.task_specs import DrawerBenchmarkCriteria

RENDER_PERIOD_NS = 20_000_000
EVALUATION_PERIOD_NS = 5_000_000
_CRITERIA_FIELDS = (
    "lift_m", "loaded_position_tolerance_m", "close_tolerance_m",
    "buckle_tolerance_rad", "position_tolerance_m", "clearance_m",
    "hold_s", "verify_hold_s",
)


class DrawerCycleSuccess:
    """Physical task witnesses with explicit benchmark and legacy modes.

    Benchmark requirements are versioned independently of controller settings.
    Legacy calibrated mode retains expert-specific pose diagnostics as gates.
    Neither mode reads planner completion or script progress flags.
    """

    def __init__(self, model_path: Path, config: DrawerCycleConfig | None = None,
                 *, sample_period_ns=RENDER_PERIOD_NS, mode="calibrated", retreat_distance_m=.09):
        self.config = DrawerBenchmarkCriteria() if mode == "benchmark" else (config or DrawerCycleConfig())
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
        self._configure_period(sample_period_ns)
        self._configure_mode(mode, retreat_distance_m)
        self.reset(None)

    def _configure_mode(self, mode, retreat_distance_m):
        if mode not in ("calibrated", "benchmark"):
            raise ValueError("unsupported drawer criteria mode")
        if (type(retreat_distance_m) not in (int, float) or not np.isfinite(retreat_distance_m)
                or retreat_distance_m <= 0):
            raise ValueError("drawer retreat distance must be finite and positive")
        self.mode, self.retreat_distance_m = mode, float(retreat_distance_m)
        if mode == "benchmark" and self.retreat_distance_m != DrawerBenchmarkCriteria().retreat_distance_m:
            raise ValueError("retreat distance differs from the versioned benchmark task")

    def _configure_period(self, period):
        if type(period) is not int or period not in (RENDER_PERIOD_NS, EVALUATION_PERIOD_NS):
            raise ValueError("drawer monitor requires an explicit 50 Hz or 200 Hz period")
        self.sample_period_ns = period

    @classmethod
    def from_geometry(cls, grip_origin, rotation, axis, config, *, sample_period_ns,
                      mode="calibrated", retreat_distance_m=.09):
        """Snapshot immutable model calibration; never consult controller progress."""
        if mode == "benchmark":config = DrawerBenchmarkCriteria()
        spec = dict(schema="drawer-physical-criteria/1", sample_period_ns=sample_period_ns,
                    mode=mode, retreat_distance_m=retreat_distance_m,
                    grip_origin=np.asarray(grip_origin).tolist(), rotation=np.asarray(rotation).tolist(),
                    axis=np.asarray(axis).tolist(),
                    config={name: float(getattr(config, name)) for name in _CRITERIA_FIELDS})
        return cls.from_spec(spec)

    @classmethod
    def from_spec(cls, spec):
        from types import SimpleNamespace
        try:
            if not isinstance(spec, dict) or spec.get("schema") != "drawer-physical-criteria/1":
                raise ValueError("drawer scoring requires a frozen drawer-physical-criteria/1 specification")
            config = spec["config"]
            if (not isinstance(config, dict) or set(config) != set(_CRITERIA_FIELDS)
                    or any(type(v) not in (int, float) or not np.isfinite(v) or v <= 0 for v in config.values())
                    or config["lift_m"] <= config["loaded_position_tolerance_m"]):
                raise ValueError("invalid drawer criteria thresholds")
            monitor = cls.__new__(cls)
            monitor.config = SimpleNamespace(**config)
            monitor.grip_origin = np.array(spec["grip_origin"], dtype=float)
            monitor.rotation = np.array(spec["rotation"], dtype=float)
            monitor.axis = np.array(spec["axis"], dtype=float)
            if (monitor.grip_origin.shape != (3,) or not np.isfinite(monitor.grip_origin).all()
                    or monitor.rotation.shape != (3, 3) or monitor.axis.shape != (3,)
                    or not np.allclose(monitor.rotation, np.diag([-1., 1., -1.]))
                    or not np.allclose(monitor.axis, [0., 0., 1.])):
                raise ValueError("drawer criteria require the calibrated upward slide geometry")
            monitor._configure_period(spec["sample_period_ns"])
            monitor._configure_mode(spec["mode"], spec["retreat_distance_m"])
            if monitor.mode == "benchmark":
                expected = DrawerBenchmarkCriteria()
                if any(config[name] != getattr(expected,name) for name in _CRITERIA_FIELDS):
                    raise ValueError("thresholds differ from the versioned benchmark task")
            monitor.reset(None)
            return monitor
        except (KeyError, TypeError, OverflowError) as error:
            raise ValueError("incomplete frozen drawer criteria specification") from error

    def specification(self):
        spec = dict(schema="drawer-physical-criteria/1", sample_period_ns=self.sample_period_ns,
                    mode=self.mode, retreat_distance_m=self.retreat_distance_m,
                    grip_origin=self.grip_origin.tolist(), rotation=self.rotation.tolist(), axis=self.axis.tolist(),
                    config={name: float(getattr(self.config, name)) for name in _CRITERIA_FIELDS})
        if self.mode == "benchmark":
            spec.update(task_id=DrawerBenchmarkCriteria().task_id, instruction=DrawerBenchmarkCriteria().instruction)
        return spec

    @staticmethod
    def _read(sample, *, optional_diagnostics=False):
        scalar = np.asarray([sample["drawer_position_m"], sample["drawer_velocity_m_s"],
                             sample["buckle_rad"], sample["gripper_drive_position_rad"]], dtype=float)
        p = np.asarray(sample["end_effector_position_body_m"], dtype=float)
        rotation = (None if optional_diagnostics and sample.get("end_effector_rotation_body") is None
                    else np.asarray(sample["end_effector_rotation_body"], dtype=float))
        velocity = (None if optional_diagnostics and sample.get("arm_joint_velocity_rad_s") is None
                    else np.asarray(sample["arm_joint_velocity_rad_s"], dtype=float))
        contacts = sample["finger_drawer_contacts"]
        if (scalar.shape != (4,) or p.shape != (3,)
                or not all(np.isfinite(v).all() for v in (scalar, p))
                or (rotation is not None and (rotation.shape != (3,3) or not np.isfinite(rotation).all()
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)))
                or (velocity is not None and (velocity.shape != (6,) or not np.isfinite(velocity).all()))
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
        self._peak_dwell = dict(self._dwell)
        self._closed = self._success = False
        self._hover_error = None
        self._jaw_alignment_error = None
        self._retreat_distance = None
        if initial is None:
            return
        try:
            drawer, _, buckle, _, _, _, _, _ = self._read(initial, optional_diagnostics=self.mode=="benchmark")
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
        self._peak_dwell[name] = max(self._peak_dwell[name], self._dwell[name])
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
            if dt_ns != self.sample_period_ns:
                raise ValueError(f"physical task samples must be consecutive {1e9/self.sample_period_ns:g} Hz frames")
            drawer, speed, buckle, opening, p, rotation, velocity, contacts = self._read(sample, optional_diagnostics=self.mode=="benchmark")
            self.last_time_ns = stamp
            displacement = float(drawer - self.initial_drawer)
            self.peak = max(self.peak, displacement)
            seated = abs(displacement) <= self.config.close_tolerance_m and abs(speed) < .003
            self._closed = seated and abs(buckle - self.initial_buckle) <= self.config.buckle_tolerance_rad
            at_handle = np.linalg.norm(p - (self.grip_origin + self.axis * drawer)) < self.config.loaded_position_tolerance_m
            still = velocity is not None and np.max(np.abs(velocity)) < .015
            # Swapping the two opposing jaws is a half-turn about tool-down:
            # both transverse axes reverse, but down remains directed. Test the
            # full principal angle against these two frames, not abs(trace) or
            # abs(tool-down), so sideways/upward tools and 90-degree yaw fail.
            jaw_swapped = self.grasp_rotation * np.array([-1., -1., 1.])
            self._jaw_alignment_error = None
            if rotation is not None:
                cosine = max((np.trace(reference @ rotation.T) - 1.) / 2.
                             for reference in (self.grasp_rotation, jaw_swapped))
                self._jaw_alignment_error = float(np.arccos(np.clip(cosine, -1., 1.)))
            aligned = self._jaw_alignment_error is not None and self._jaw_alignment_error < .025
            grasp = bool(all(contacts) and .025 < opening < .95)
            if self.mode == "calibrated":
                grasp = bool(grasp and at_handle and still and aligned)
            grasp_stable = self._stable("grasp", grasp, dt_ns, 1.)
            self.grasp_verified |= grasp_stable if self.mode == "calibrated" else grasp
            # The loaded tolerance is 5 mm in the delivered config, matching the
            # existing lift witness. Extra opening is not mistaken for failure.
            lifted = (self.grasp_verified and grasp and abs(speed) < .003
                      and displacement >= self.config.lift_m - self.config.loaded_position_tolerance_m)
            self.open_verified |= self._stable("lift", lifted, dt_ns, self.config.hold_s)
            seat_condition = bool(self.open_verified and seated and
                                  (grasp if self.mode == "calibrated" else opening < .95 and any(contacts)))
            seat_stable = self._stable("seat", seat_condition, dt_ns, self.config.verify_hold_s)
            self.seated_verified |= seat_stable if self.mode == "calibrated" else seat_condition
            no_contact = not any(contacts)
            if self.seated_verified and seated and opening > .95 and no_contact and (still or self.mode == "benchmark"):
                self.released = True
            self._hover_error = float(np.linalg.norm(p - self.hover))
            self._retreat_distance = float(np.linalg.norm(p - (self.grip_origin + self.axis * drawer)))
            final = (self.open_verified and self.seated_verified and self.released and self._closed
                     and opening > .95 and no_contact)
            if self.mode == "calibrated":
                final = final and self._hover_error < self.config.position_tolerance_m and still and aligned
            else:
                final = final and self._retreat_distance >= self.retreat_distance_m
            self._success = self._stable("final", final, dt_ns, self.config.verify_hold_s)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.error = str(error)
            self._success = False
        return self.status()

    def status(self) -> dict:
        available = self.active and self.error is None
        return dict(task_name="drawer_cycle", task_version="drawer-benchmark-success/1" if self.mode=="benchmark" else "drawer-policy-success/3",
                    criteria_mode=self.mode,
                    success_available=bool(available), success=bool(available and self._success),
                    success_unavailable_reason=self.error or (None if self.active else "no active drawer policy session"),
                    grasp_verified=bool(self.grasp_verified), open_verified=bool(self.open_verified),
                    seated_before_release_verified=bool(self.seated_verified), release_verified=bool(self.released),
                    peak_drawer_displacement_m=self.peak, drawer_closed=bool(self._closed),
                    verification_stable_s=self._dwell["final"] * 1e-9,
                    max_grasp_hold_s=self._peak_dwell["grasp"] * 1e-9,
                    max_open_hold_s=self._peak_dwell["lift"] * 1e-9,
                    max_seat_hold_s=self._peak_dwell["seat"] * 1e-9,
                    expected_hover_position_body_m=self.hover.tolist() if self.hover is not None else None,
                    hover_position_error_m=self._hover_error,
                    retreat_distance_m=self._retreat_distance,
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
                self.monitor = TASK_SUCCESS_MONITORS["drawer_cycle"](model_path, mode="benchmark")
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
