"""Physical drawer opening/closing cycle; no state edits or grasp attachment."""

from dataclasses import dataclass
import math
from typing import ClassVar
import numpy as np
from .drawer_lift import DrawerLiftConfig, DrawerLiftTask
from .motion_timing import time_joint_path
from .approach_path import ApproachPosePath


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
    initial_settle_s: float = 0.25
    transit_confirm_s: float = 0.1
    terminal_confirm_s: float = 0.2
    approach_clearance_m: float = 0.06
    grip_confirm_s: float = 0.15
    seat_confirm_s: float = 0.2
    release_clearance_opening: float = 0.6

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
            "initial_settle_s",
            "transit_confirm_s",
            "terminal_confirm_s",
            "approach_clearance_m",
            "grip_confirm_s",
            "seat_confirm_s",
            "release_clearance_opening",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")

        if not self.slow_distance_m < self.approach_clearance_m < self.clearance_m:
            raise ValueError("Approach clearance must lie between slow distance and retreat clearance")
        if not 0.5 <= self.release_clearance_opening <= 0.95:
            raise ValueError("Release clearance opening must be between 0.5 and 0.95")
        if self.linear_speed_m_s > 0.1 or self.return_speed_m_s > 0.1:
            raise ValueError("Drawer translation speed must be <= 0.1 m/s")
        if self.seating_overtravel_m >= self.close_tolerance_m:
            raise ValueError(
                "seating overtravel must be below the closed-position tolerance"
            )


class DrawerCycleTask(DrawerLiftTask):
    name = "drawer_cycle"
    version = "drawer-cycle/5"
    legacy_stages = DrawerLiftTask.stages + ("return", "seat", "release", "retract", "verify")
    v4_stages = tuple(stage for stage in legacy_stages if stage != "align")
    stages = ("settle", "approach", "grip", "lift", "return", "release", "retract", "verify")
    approach_stages = ("approach",)

    @classmethod
    def recorded_stages(cls, version):
        if version == cls.version:
            return cls.stages
        if version == "drawer-cycle/4":
            return cls.v4_stages
        if version in (None, "drawer-cycle/1", "drawer-cycle/2", "drawer-cycle/3"):
            return cls.legacy_stages
        raise ValueError(f"Unsupported drawer task version: {version}")

    def __init__(self, model_path, config=None):
        super().__init__(model_path, config or DrawerCycleConfig())
        self.open_verified = False
        self.closed_verified = False
        self.peak_drawer_displacement_m = 0.0
        self.initial_buckle_rad = None
        self._released_closed_since = None
        self._approach_profile = None
        self._gripper_timing = None
        self._gripper_motion_s = 0.0

    def _stage_goal(self, name, p, r):
        if name == "approach":
            return self.grasp.copy(), self.grasp_rotation
        if name == "raise" and p[2] >= self.hover[2] - 1e-9:
            # Already at safe height: there is no trajectory to time.
            return None, r
        if name == "above_drawer":
            # Translate and turn together only after reaching the safe height.
            return self.hover.copy(), self.grasp_rotation
        if name in ("return", "seat"):
            # Small bounded preload overcomes servo deflection at the latch.
            # This moves only the actuator reference, never the drawer state.
            return (
                self.grasp - self.axis * self.config.seating_overtravel_m,
                self.grasp_rotation,
            )
        if name == "retract":
            return self.hover.copy(), self.grasp_rotation
        return super()._stage_goal(name, p, r)

    def _enter(self, name, sample):
        continuing_open = self.stage == "release" and name == "retract"
        previous_gripper = self._gripper_timing, self._gripper_motion_s
        if name == "settle":
            self.initial_buckle_rad = sample.buckle_rad
        if name == "return":
            self.open_verified = True
        if name == "release":
            self.closed_verified = True
        super()._enter(name, sample)
        if name == "raise" and self._path is None:
            # Skip only the empty trajectory, not the measured pose check.
            self._goal, self._rotation = self.chain.forward(self.reference[:6])
        self._gripper_timing = None
        self._gripper_motion_s = 0.0
        target = self._gripper_target()
        if continuing_open:
            self._gripper_timing, self._gripper_motion_s = previous_gripper
        elif name == "approach":
            self.events[-1]["coordinated_opening"] = True
        elif target is not None and abs(target - self.reference[6]) > 1e-12:
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

    def _plan_path(self, name, p, r, goal, rotation):
        if name != "approach":
            return super()._plan_path(name, p, r, goal, rotation)
        self._approach_profile = ApproachPosePath(
            p, r, goal, rotation, self.axis, self.reference[6], self.config.approach_clearance_m
        )
        linear, angular, _ = self._approach_profile.derivative_bounds()
        return self._solve_path(self._approach_profile.pose, count=161), linear[0], angular[0]

    def approach_reference(self, progress):
        """Same arm and finger configuration used by execution and collision preflight."""
        opening = self._approach_profile.opening(progress)
        return np.r_[self._timing.path.at(progress), opening, -opening]

    def _time_path(self, name, distance, angle):
        cfg = self.config
        speed = cfg.linear_speed_m_s
        slow_speed = None
        if name in ("approach", "descend"):
            slow_speed = cfg.approach_speed_m_s
        elif name == "return":
            speed = min(speed, cfg.return_speed_m_s)
            slow_speed = cfg.seating_speed_m_s
        elif name == "seat":
            speed = min(speed, cfg.seating_speed_m_s)
        slow_distance = cfg.slow_distance_m if slow_speed is not None else 0.0
        if name == "approach":
            # Progress has an exact axial distance scale even on the curved section.
            slow_distance *= distance / self._approach_profile.height
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
            slow_distance=slow_distance,
            slow_speed=slow_speed,
        )
        if name == "approach":
            linear, angular, opening = self._approach_profile.derivative_bounds()
            for bounds, limits in (
                (linear, (cfg.linear_speed_m_s, cfg.linear_acceleration_m_s2, cfg.linear_jerk_m_s3)),
                (angular, (cfg.angular_speed_rad_s, cfg.angular_acceleration_rad_s2, cfg.angular_jerk_rad_s3)),
                (opening, (cfg.gripper_speed_rad_s, cfg.gripper_acceleration_rad_s2, cfg.gripper_jerk_rad_s3)),
            ):
                timing = timing.limit_derivatives(bounds, *limits)
        timing.path.validate_limits(self.lower, self.upper)
        return timing

    def _step_gripper(self, target, dt):
        if self.stage == "approach":
            self.reference[6] = self._approach_profile.opening(self._timing.at(self._motion_s))
        elif self._gripper_timing is None:
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
        if self.stage in ("approach", "release", "retract", "verify"):
            return 1.0
        return super()._gripper_target()

    def _loaded_stage(self):
        return self.stage in ("lift", "return", "seat")

    def _stable_time(self):
        if self.stage == "settle":
            return self.config.initial_settle_s
        if self.stage == "grip":
            return self.config.grip_confirm_s
        if self.stage == "return":
            return self.config.seat_confirm_s
        if self.stage in ("approach", "raise", "above_drawer", "descend", "lift", "release", "retract"):
            return self.config.transit_confirm_s
        if self.stage == "seat":
            return self.config.verify_hold_s
        if self.stage == "verify":
            return self.config.terminal_confirm_s
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
        # Observe the released drawer during retreat too. Any renewed motion,
        # latch opening or finger contact restarts the full verification window.
        released_closed = (
            self.stage in ("release", "retract", "verify")
            and self._closed(sample)
            and sample.joints[6] >= self.config.release_clearance_opening
            and sample.finger_drawer_contacts == (0, 0)
        )
        if not released_closed:
            self._released_closed_since = None
        elif self._released_closed_since is None:
            self._released_closed_since = sample.time_s
        if self.stage == "settle":
            reached = reached and sample.wrench_valid
        self.peak_drawer_displacement_m = max(
            self.peak_drawer_displacement_m, sample.drawer_m - self._initial_drawer
        )
        reached = super()._stage_reached(sample, reached, p)
        if (
            self.stage in ("open", "grip")
            and self._gripper_timing is not None
        ):
            reached = (
                reached and self._gripper_motion_s >= self._gripper_timing.duration
            )
        if self.stage == "approach":
            reached = reached and sample.joints[6] > 0.95
        if self.stage == "lift":
            reached = (
                reached
                and abs(sample.drawer_m - self._initial_drawer - self.config.lift_m) <= 0.005
                and abs(sample.drawer_velocity_m_s) < 0.003
                and sample.joints[6] > 0.025
                and sample.finger_drawer_contacts is not None
                and all(sample.finger_drawer_contacts)
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
            reached = (reached and self._closed(sample)
                       and sample.joints[6] >= self.config.release_clearance_opening
                       and sample.finger_drawer_contacts == (0, 0))
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
                    and self._released_closed_since is not None
                    and sample.time_s - self._released_closed_since >= self.config.verify_hold_s
                    and np.linalg.norm(p - self.hover)
                    < self.config.position_tolerance_m
                )
        return reached

    def metrics(self, sample):
        return dict(
            task_version=self.version,
            released_closed_seconds=(
                0.0 if self._released_closed_since is None
                else max(0.0, sample.time_s - self._released_closed_since)
            ),
            open_verified=self.open_verified,
            seated_before_release_verified=self.closed_verified,
            peak_drawer_displacement_m=self.peak_drawer_displacement_m,
            drawer_closed=self._closed(sample)
            if self.initial_buckle_rad is not None
            else False,
            gripper_opening_normalized=float(np.clip(sample.joints[6], 0, 1)),
        )
