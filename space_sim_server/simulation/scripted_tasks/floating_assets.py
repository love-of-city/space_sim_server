"""Object identities and grasp frames shared by floating capture tasks."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FloatingAsset:
    body: str
    label: str
    instruction_name: str
    grip_point: tuple[float, float, float]
    closing_axis: tuple[float, float, float]
    approach_axis: tuple[float, float, float]


    def __post_init__(self):
        for value in (self.grip_point, self.closing_axis, self.approach_axis):
            if len(value) != 3 or not all(math.isfinite(v) for v in value):
                raise ValueError("Floating asset vectors must have three finite values")
        for axis in (self.closing_axis, self.approach_axis):
            if not math.isclose(sum(v*v for v in axis), 1., abs_tol=1e-9):
                raise ValueError("Floating asset axes must be unit vectors")
        if abs(sum(a*b for a, b in zip(self.closing_axis, self.approach_axis))) > 1e-9:
            raise ValueError("Floating asset axes must be orthogonal")

    def snapshot(self):
        return dict(body=self.body, grip_point=list(self.grip_point),
                    closing_axis=list(self.closing_axis), approach_axis=list(self.approach_axis))


def validate_asset_snapshot(release, asset):
    if release.get("schema") != "floating-release/2" or release.get("asset_snapshot") != asset.snapshot():
        raise ValueError("Floating release asset snapshot does not match current description")


FLOATING_TASKS = {
    "floating_aviation_plug_capture": FloatingAsset(
        "aviation_plug", "漂浮航空插头", "floating aviation plug", (0., 0., -.003),
        (0., 1., 0.), (0., 0., 1.)),
    "floating_bolt_capture": FloatingAsset(
        "bolt", "漂浮航空螺栓", "floating aviation bolt", (0., 0., -.005),
        (0., 1., 0.), (0., 0., 1.)),
    "floating_plug_capture": FloatingAsset(
        "cylindrical_plug", "漂浮圆柱插头", "floating cylindrical plug", (0., 0., -.006),
        (0., 1., 0.), (0., 0., 1.)),
    "floating_panel_capture": FloatingAsset(
        "satellite_outer_panel", "漂浮卫星外侧板", "detached satellite panel",
        (-.36, .0046, 0.), (0., 1., 0.), (1., 0., 0.)),
}
