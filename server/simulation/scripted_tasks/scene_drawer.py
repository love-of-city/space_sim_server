"""Drawer scene expert with benchmark dwell; collection timings stay independent."""
from dataclasses import dataclass
from .drawer_cycle import DrawerCycleConfig, DrawerCycleTask


@dataclass(frozen=True)
class SceneDrawerCycleConfig(DrawerCycleConfig):
    verify_hold_s: float = 2.0


class SceneDrawerCycleTask(DrawerCycleTask):
    version = "drawer-cycle/6-straight-approach"

    def __init__(self, model_path, config=None):
        super().__init__(model_path, config or SceneDrawerCycleConfig())

    def _stable_time(self):
        if self.stage == "settle":
            return self.config.initial_settle_s
        if self.stage == "grip":
            return max(self.config.grip_confirm_s,1.)+.05
        if self.stage == "return":
            return max(self.config.seat_confirm_s,self.config.verify_hold_s)+.05
        if self.stage == 'lift':
            return self.config.hold_s+.05
        if self.stage in ("approach", "raise", "above_drawer", "descend", "release", "retract"):
            return self.config.transit_confirm_s
        if self.stage == "seat":
            return self.config.verify_hold_s
        if self.stage == "verify":
            return self.config.terminal_confirm_s
        return super()._stable_time()

    def _closed(self, sample):
        return (
            self._seated(sample)
            and abs(sample.buckle_rad - self.initial_buckle_rad)
            <= self.config.buckle_tolerance_rad
        )
