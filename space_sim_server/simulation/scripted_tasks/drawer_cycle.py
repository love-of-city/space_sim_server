"""Physical drawer opening/closing cycle; no state edits or grasp attachment."""

from dataclasses import dataclass
import math
from typing import ClassVar
import numpy as np
from .drawer_lift import DrawerLiftConfig, DrawerLiftTask
from .motion_timing import time_joint_path


@dataclass(frozen=True)
class DrawerCycleConfig(DrawerLiftConfig):
    # Simulation-task override; not a new hardware velocity rating.
    max_arm_speed_rad_s: ClassVar[float] = 0.35
    arm_speed_rad_s: float = 0.35
    linear_speed_m_s: float = 0.1
    approach_speed_m_s: float = 0.025
    seating_speed_m_s: float = 0.012
    slow_distance_m: float = 0.02
    angular_speed_rad_s: float = 0.2
    linear_acceleration_m_s2: float = 0.2
    angular_acceleration_rad_s2: float = 0.4
    joint_acceleration_rad_s2: float = 0.7
    linear_jerk_m_s3: float = 0.8
    angular_jerk_rad_s3: float = 1.6
    joint_jerk_rad_s3: float = 8.0
    grip_speed_rad_s: float = 0.2
    grip_contact_speed_rad_s: float = 0.2
    grip_contact_range_rad: float = 0.2
    gripper_acceleration_rad_s2: float = 0.4
    gripper_jerk_rad_s3: float = 1.6
    close_tolerance_m: float = 0.002
    buckle_tolerance_rad: float = 0.025
    return_speed_m_s: float = 0.1
    seating_overtravel_m: float = 0.001
    verify_hold_s: float = 2.0

    def __post_init__(self):
        super().__post_init__()
        for name in (
            "close_tolerance_m",
            "buckle_tolerance_rad",
            "return_speed_m_s",
            "approach_speed_m_s",
            "seating_speed_m_s",
            "slow_distance_m",
            "angular_speed_rad_s",
            "linear_acceleration_m_s2",
            "angular_acceleration_rad_s2",
            "joint_acceleration_rad_s2",
            "linear_jerk_m_s3",
            "angular_jerk_rad_s3",
            "joint_jerk_rad_s3",
            "grip_speed_rad_s",
            "grip_contact_speed_rad_s",
            "grip_contact_range_rad",
            "gripper_acceleration_rad_s2",
            "gripper_jerk_rad_s3",
            "seating_overtravel_m",
            "verify_hold_s",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")

        if self.linear_speed_m_s > 0.1 or self.return_speed_m_s > 0.1:
            raise ValueError("Drawer translation speed must be <= 0.1 m/s")
        if self.seating_overtravel_m >= self.close_tolerance_m:
            raise ValueError(
                "seating overtravel must be below the closed-position tolerance"
            )


class DrawerCycleTask(DrawerLiftTask):
    name = "drawer_cycle"
    version = "drawer-cycle/3"
    stages = DrawerLiftTask.stages + ("return", "seat", "release", "retract", "verify")

    def __init__(self, model_path, config=None):
        super().__init__(model_path, config or DrawerCycleConfig())
        self.open_verified = False
        self.closed_verified = False
        self.peak_drawer_displacement_m = 0.0
        self.initial_buckle_rad = None
        self._gripper_timing = None
        self._gripper_motion_s = 0.0

    def _stage_goal(self, name, p, r):
        if name == "seat":
            # Small bounded preload overcomes servo deflection at the latch.
            # This moves only the actuator reference, never the drawer state.
            return (
                self.grasp - self.axis * self.config.seating_overtravel_m,
                self.grasp_rotation,
            )
        if name == "return":
            return self.grasp.copy(), self.grasp_rotation
        if name == "retract":
            return self.hover.copy(), self.grasp_rotation
        return super()._stage_goal(name, p, r)

    def _enter(self, name, sample):
        if name == "settle":
            self.initial_buckle_rad = sample.buckle_rad
        if name == "return":
            self.open_verified = True
        if name == "release":
            self.closed_verified = True
        super()._enter(name, sample)
        self._gripper_timing = None
        self._gripper_motion_s = 0.0
        target = self._gripper_target()
        if target is not None and abs(target - self.reference[6]) > 1e-12:
            cfg = self.config
            speed = cfg.gripper_speed_rad_s
            closing = target < self.reference[6]
            if closing:
                speed = min(speed, cfg.grip_speed_rad_s)
            self._gripper_timing = time_joint_path(
                [[self.reference[6]], [target]],
                0.0,
                0.0,
                joint_speed=speed,
                linear_speed=cfg.linear_speed_m_s,
                angular_speed=cfg.angular_speed_rad_s,
                linear_acceleration=cfg.linear_acceleration_m_s2,
                angular_acceleration=cfg.angular_acceleration_rad_s2,
                joint_acceleration=cfg.gripper_acceleration_rad_s2,
                joint_jerk=cfg.gripper_jerk_rad_s3,
                slow_fraction=min(
                    1.0, cfg.grip_contact_range_rad / abs(target - self.reference[6])
                )
                if closing
                else 0.0,
                slow_joint_speed=cfg.grip_contact_speed_rad_s if closing else None,
            )
            self._timeout = max(self._timeout, 2.5 * self._gripper_timing.duration + 10)
            self.events[-1]["planned_gripper_seconds"] = self._gripper_timing.duration
        self.events[-1].update(self.metrics(sample))

    def _time_path(self, name, distance, angle):
        cfg = self.config
        speed = cfg.linear_speed_m_s
        slow_speed = None
        if name == "descend":
            slow_speed = cfg.approach_speed_m_s
        elif name == "return":
            speed = min(speed, cfg.return_speed_m_s)
            slow_speed = cfg.seating_speed_m_s
        elif name == "seat":
            speed = min(speed, cfg.seating_speed_m_s)
        timing = time_joint_path(
            self._path,
            distance,
            angle,
            joint_speed=cfg.arm_speed_rad_s,
            linear_speed=speed,
            angular_speed=cfg.angular_speed_rad_s,
            linear_acceleration=cfg.linear_acceleration_m_s2,
            angular_acceleration=cfg.angular_acceleration_rad_s2,
            joint_acceleration=cfg.joint_acceleration_rad_s2,
            linear_jerk=cfg.linear_jerk_m_s3,
            angular_jerk=cfg.angular_jerk_rad_s3,
            joint_jerk=cfg.joint_jerk_rad_s3,
            slow_distance=cfg.slow_distance_m if slow_speed is not None else 0.0,
            slow_speed=slow_speed,
        )
        timing.path.validate_limits(self.lower, self.upper)
        return timing

    def _step_gripper(self, target, dt):
        if self._gripper_timing is None:
            self.reference[6] = target
        else:
            self._gripper_motion_s = min(
                self._gripper_motion_s + dt, self._gripper_timing.duration
            )
            self.reference[6] = self._gripper_timing.joint_state(
                self._gripper_motion_s
            )[0][0]
        self.reference[7] = -self.reference[6]

    def _gripper_target(self):
        if self.stage in ("release", "retract", "verify"):
            return 1.0
        return super()._gripper_target()

    def _loaded_stage(self):
        return self.stage in ("lift", "return", "seat")

    def _stable_time(self):
        if self.stage in ("seat", "verify"):
            return self.config.verify_hold_s
        return super()._stable_time()

    def _seated(self, sample):
        return (
            abs(sample.drawer_m - self._initial_drawer) <= self.config.close_tolerance_m
            and abs(sample.drawer_velocity_m_s) < 0.003
        )

    def _closed(self, sample):
        return (
            self._seated(sample)
            and abs(sample.buckle_rad - self.initial_buckle_rad)
            <= self.config.buckle_tolerance_rad
        )

    def _stage_reached(self, sample, reached, p):
        self.peak_drawer_displacement_m = max(
            self.peak_drawer_displacement_m, sample.drawer_m - self._initial_drawer
        )
        reached = super()._stage_reached(sample, reached, p)
        if (
            self.stage in ("open", "grip", "release")
            and self._gripper_timing is not None
        ):
            reached = (
                reached and self._gripper_motion_s >= self._gripper_timing.duration
            )
        if self.stage == "grip":
            reached = (
                reached
                and sample.finger_drawer_contacts is not None
                and all(sample.finger_drawer_contacts)
            )
        if self.stage in ("return", "seat"):
            reached = (
                reached
                and self.open_verified
                and self._seated(sample)
                and sample.joints[6] > 0.025
            )
            reached = (
                reached
                and np.linalg.norm(p - (self.grip_origin + self.axis * sample.drawer_m))
                < self.config.loaded_position_tolerance_m
            )
        elif self.stage == "release":
            reached = reached and self._seated(sample) and sample.joints[6] > 0.95
        elif self.stage in ("retract", "verify"):
            reached = (
                reached
                and self.open_verified
                and self.closed_verified
                and self._closed(sample)
                and sample.joints[6] > 0.95
            )
            if self.stage == "verify":
                reached = (
                    reached
                    and np.linalg.norm(p - self.hover)
                    < self.config.position_tolerance_m
                )
        return reached

    def metrics(self, sample):
        return dict(
            task_version=self.version,
            open_verified=self.open_verified,
            seated_before_release_verified=self.closed_verified,
            peak_drawer_displacement_m=self.peak_drawer_displacement_m,
            drawer_closed=self._closed(sample)
            if self.initial_buckle_rad is not None
            else False,
            gripper_opening_normalized=float(np.clip(sample.joints[6], 0, 1)),
        )
