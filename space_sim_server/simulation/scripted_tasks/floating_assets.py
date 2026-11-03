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
    # A source body means a new root-only copy, with the original assembly intact.
    source_body: str | None = None

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
        result = dict(body=self.body, grip_point=list(self.grip_point),
                      closing_axis=list(self.closing_axis), approach_axis=list(self.approach_axis))
        if self.source_body is not None:
            result["source_body"] = self.source_body
        return result


def validate_asset_snapshot(release, asset):
    if release.get("schema") != "floating-release/2" or release.get("asset_snapshot") != asset.snapshot():
        raise ValueError("Floating release asset snapshot does not match current description")


FLOATING_TASKS = {
    "floating_task_box_capture": FloatingAsset(
        "floating_task_box_base", "漂浮任务箱底座", "floating task box base", (-.003, .275, -.0125),
        (0., 0., 1.), (1., 0., 0.), source_body="task_box_base"),
    "floating_hook_capture": FloatingAsset(
        "floating_hook", "漂浮挂钩", "floating hook", (.398, .500, .055),
        (0., 1., 0.), (-1., 0., 0.), source_body="hook"),
    "floating_limit_clip_capture": FloatingAsset(
        "floating_limit_clip", "漂浮限位夹", "floating limit clip", (.256, .500, .043),
        (0., 1., 0.), (0., 0., -1.), source_body="limit_clip"),
    "floating_drawer_buckle_capture": FloatingAsset(
        "floating_drawer_buckle", "漂浮抽屉扣", "floating drawer buckle", (-.117, .008, 0.),
        (0., 0., 1.), (1., 0., 0.), source_body="drawer_buckle"),
    "floating_fastener_capture": FloatingAsset(
        "floating_fastener", "漂浮紧固件", "floating fastener", (0., 0., -.005),
        (0., 1., 0.), (0., 0., 1.), source_body="fastener"),
    "floating_locking_rod_capture": FloatingAsset(
        "floating_locking_rod", "漂浮锁杆", "floating locking rod", (0., .065, -.015),
        (1., 0., 0.), (0., 0., 1.), source_body="locking_rod"),
    "floating_drawer_capture": FloatingAsset(
        "floating_drawer_base", "漂浮抽屉本体", "floating drawer body", (0., 0., -.005),
        (0., 1., 0.), (0., 0., 1.), source_body="drawer_base"),
    "floating_board1_capture": FloatingAsset(
        "floating_asm_board1", "漂浮安装板1", "floating mounting board 1", (.317, .250, .0025),
        (0., 0., 1.), (1., 0., 0.), source_body="asm_board1"),
    "floating_board2_capture": FloatingAsset(
        "floating_asm_board2", "漂浮安装板2", "floating mounting board 2", (.0955, .210, -.005),
        (0., 0., 1.), (1., 0., 0.), source_body="asm_board2"),
    "floating_board3_capture": FloatingAsset(
        "floating_asm_board3", "漂浮安装板3", "floating mounting board 3", (.207, .040, .0025),
        (0., 0., 1.), (1., 0., 0.), source_body="asm_board3"),
    "floating_inner_panel_capture": FloatingAsset(
        "floating_satellite_inner_panel", "漂浮卫星内侧板", "floating satellite inner panel",
        (.382, -.07275, .0470787), (0., 1., 0.), (-1., 0., 0.), source_body="satellite_inner_panel"),
    "floating_guide_plug_capture": FloatingAsset(
        "guide_plug", "漂浮导向插头", "floating guide plug", (0., 0., -.003),
        (0., 1., 0.), (0., 0., 1.)),
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
