"""Physical drawer opening/closing cycle; no state edits or grasp attachment."""

from dataclasses import dataclass
import math
import numpy as np
from .drawer_lift import DrawerLiftConfig, DrawerLiftTask


@dataclass(frozen=True)
class DrawerCycleConfig(DrawerLiftConfig):
    close_tolerance_m: float = 0.002
    buckle_tolerance_rad: float = 0.025
    return_speed_m_s: float = 0.012
    seating_overtravel_m: float = 0.001
    verify_hold_s: float = 2.0

    def __post_init__(self):
        super().__post_init__()
        for name in (
            "close_tolerance_m",
            "buckle_tolerance_rad",
            "return_speed_m_s",
            "seating_overtravel_m",
            "verify_hold_s",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")

        if self.seating_overtravel_m >= self.close_tolerance_m:
            raise ValueError(
                "seating overtravel must be below the closed-position tolerance"
            )


class DrawerCycleTask(DrawerLiftTask):
    name = "drawer_cycle"
    version = "drawer-cycle/1"
    stages = DrawerLiftTask.stages + ("return", "seat", "release", "retract", "verify")

    def __init__(self, model_path, config=None):
        super().__init__(model_path, config or DrawerCycleConfig())
        self.open_verified = False
        self.closed_verified = False
        self.peak_drawer_displacement_m = 0.0
        self.initial_buckle_rad = None

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
        if name == "return":
            self._duration = max(
                self._duration,
                1.875 * self.config.lift_m / self.config.return_speed_m_s,
            )
            self._timeout = max(30.0, 2.5 * self._duration + 10.0)
        self.events[-1].update(self.metrics(sample))

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
