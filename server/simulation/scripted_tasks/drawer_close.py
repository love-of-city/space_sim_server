"""Independent drawer closing from an episode-initialized open state."""

from dataclasses import dataclass

from .drawer_cycle import DrawerCycleConfig, DrawerCycleTask


@dataclass(frozen=True)
class DrawerCloseConfig(DrawerCycleConfig):
    """lift_m is the expected initial opening, not an extra runtime lift."""
    verify_hold_s: float = 2.0


class DrawerCloseTask(DrawerCycleTask):
    name = "drawer_close"
    version = "drawer-close/1"
    stages = ("settle", "approach", "grip", "return", "release", "retract", "verify")
    required_operation_stages = ("return", "retract")

    def __init__(self, model_path, config=None):
        super().__init__(model_path, config or DrawerCloseConfig())
        # The recorder samples metrics before the first controller step. Use the
        # compiled model's initial coordinate until the first measured sample.
        self._initial_drawer = self.drawer_initial_reference_m

    @classmethod
    def recorded_stages(cls, version):
        if version != cls.version:
            raise ValueError(f"Unsupported drawer task version: {version}")
        return cls.stages

    def _initial_state_error(self, sample):
        if abs(sample.drawer_m - self.config.lift_m) > 0.005:
            return "drawer_not_initially_open"
        return None

    def _record_verified_transition(self, name, sample):
        super()._record_verified_transition(name, sample)
        if name == "settle":
            self.open_verified = True

    def _stage_goal(self, name, p, r):
        if name in ("return", "seat"):
            return (
                self.grip_origin - self.axis * self.config.seating_overtravel_m,
                self.grasp_rotation,
            )
        if name == "retract":
            # Reuse the original cycle's retreat height relative to the closed
            # drawer, rather than adding the initial eight-centimetre opening.
            return self.grip_origin + self.axis * self.config.clearance_m, self.grasp_rotation
        return super()._stage_goal(name, p, r)

    def _enter(self, name, sample):
        if name == "settle":
            self.hover = self.grip_origin + self.axis * self.config.clearance_m
        super()._enter(name, sample)

    def _seated(self, sample):
        return (
            abs(sample.drawer_m) <= self.config.close_tolerance_m
            and abs(sample.drawer_velocity_m_s) < 0.003
        )

    def _closed(self, sample):
        # v1 closes to absolute zero, not the opened episode reference.
        return (self._seated(sample)
                and abs(sample.buckle_rad - self.initial_buckle_rad)
                <= self.config.buckle_tolerance_rad)

    def metrics(self, sample):
        return dict(
            **super().metrics(sample),
            drawer_initial_m=self._initial_drawer,
            drawer_target_m=0.0,
        )
