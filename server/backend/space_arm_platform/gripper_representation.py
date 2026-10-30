"""Training representations, separate from raw simulator joint coordinates."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GripperOpeningProfile:
    closed: float
    opened: float
    coordinate: str
    unit: str

    def normalize(self, position: float) -> float:
        if not math.isfinite(position):
            raise ValueError("non-finite gripper position")
        # Contact/soft constraints can slightly exceed the commanded working range.
        return min(1.0, max(0.0, (position - self.closed) / (self.opened - self.closed)))

    def metadata(self) -> dict:
        return dict(closed=self.closed, opened=self.opened, coordinate=self.coordinate,
                    unit=self.unit, normalized_range=[0.0, 1.0],
                    convention="0=closed, 1=maximum commanded opening",
                    clipping="clip to [0, 1]; retain unclipped raw joint telemetry",
                    meaning="normalized drive travel, not linear fingertip distance or grip force")


# Matches the current SARM teleop limits and validated drawer/plug task endpoints.
# Other grippers must declare their own calibrated range before opting into 7D.
GRIPPER_OPENING_PROFILES = {
    "four_bar_tendon": GripperOpeningProfile(0.0, 1.0, "ee_link11_joint", "rad"),
}


def opening_profile(gripper_type: str) -> GripperOpeningProfile | None:
    return GRIPPER_OPENING_PROFILES.get(gripper_type)
