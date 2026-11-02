"""Independent physical drawer opening, release and retreat."""

from dataclasses import dataclass
import numpy as np

from .drawer_cycle import DrawerCycleConfig, DrawerCycleTask


@dataclass(frozen=True)
class DrawerOpenConfig(DrawerCycleConfig):
    """Keep the established drawer timing and collision-planning configuration."""
    verify_hold_s: float = 2.0


class DrawerOpenTask(DrawerCycleTask):
    name = "drawer_open"
    version = "drawer-open/1"
    stages = ("settle", "approach", "grip", "lift", "release", "retract", "verify")
    required_operation_stages = ("lift", "retract")

    def __init__(self, model_path, config=None):
        self._released_open_since = None
        super().__init__(model_path, config or DrawerOpenConfig())

    @classmethod
    def recorded_stages(cls, version):
        if version != cls.version:
            raise ValueError(f"Unsupported drawer task version: {version}")
        return cls.stages

    def _record_verified_transition(self, name, sample):
        if name == "settle":
            self.initial_buckle_rad = sample.buckle_rad
        if name == "release":
            # The lift stage has already checked actual travel, stationary
            # motion and bilateral finger contact before this transition.
            self.open_verified = True

    def _opened(self, sample):
        return (
            abs(sample.drawer_m - self._initial_drawer - self.config.lift_m) <= 0.005
            and abs(sample.drawer_velocity_m_s) < 0.003
        )

    def _stage_reached(self, sample, reached, p):
        self.peak_drawer_displacement_m = max(
            self.peak_drawer_displacement_m, sample.drawer_m - self._initial_drawer
        )
        if self.stage not in ("release", "retract", "verify"):
            self._released_open_since = None
            return super()._stage_reached(sample, reached, p)
        released_open = (
            self._opened(sample)
            and sample.joints[6] >= self.config.release_clearance_opening
            and sample.finger_drawer_contacts == (0, 0)
        )
        if not released_open:
            self._released_open_since = None
        elif self._released_open_since is None:
            self._released_open_since = sample.time_s
        reached = reached and self.open_verified and released_open
        if self.stage in ("retract", "verify"):
            reached = reached and sample.joints[6] > 0.95
        if self.stage == "verify":
            reached = (
                reached
                and self._released_open_since is not None
                and sample.time_s - self._released_open_since >= self.config.verify_hold_s
                and np.linalg.norm(p - self.hover) < self.config.position_tolerance_m
            )
        return reached

    def metrics(self, sample):
        return dict(
            task_version=self.version,
            drawer_initial_m=self._initial_drawer,
            drawer_target_m=self._initial_drawer + self.config.lift_m,
            drawer_open=self._opened(sample),
            open_verified=self.open_verified,
            peak_drawer_displacement_m=self.peak_drawer_displacement_m,
            released_open_seconds=(
                0.0 if self._released_open_since is None
                else max(0.0, sample.time_s - self._released_open_since)
            ),
            gripper_opening_normalized=float(np.clip(sample.joints[6], 0, 1)),
        )
