"""Independent plug tasks and geometry measured from the delivered MJCF."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlugProfile:
    task: str
    label: str
    body: str
    prefix: str
    tip_m: float = 0.095
    flange_m: float = 0.025
    keyed: bool = False
    # Object-local closing axis through the two modeled head latch features.
    grasp_closing_axis: tuple[float, float, float] = (0.0, 1.0, 0.0)
    grasp_offset_m: tuple[float, float, float] = (0., 0., -.003)
    insert_orientation_tolerance_rad: float = .004
    extraction_lateral_gain_s_inv: float = 3.
    extraction_velocity_gain: float = 8.
    release_seat_tolerance_m: float | None = None
    require_seated_velocity: bool = True

    @property
    def target_site(self):
        return self.prefix + "_insert_40p5"

    @property
    def instruction(self):
        name = {
            "aviation": "aviation connector",
            "guide": "guide plug",
            "cylindrical": "cylindrical plug",
        }[self.prefix]
        return f"Pick up the {name}, align it with the corresponding socket, and insert it fully."


PLUG_PROFILES = {
    "aviation_plug_insert": PlugProfile(
        "aviation_plug_insert",
        "航空插头",
        "aviation_plug",
        "aviation",
        0.080,
        extraction_lateral_gain_s_inv=4.,
        keyed=True,
        grasp_closing_axis=(0.0, 1.0, 0.0),
    ),
    "guide_plug_insert": PlugProfile(
        "guide_plug_insert",
        "导向插头",
        "guide_plug",
        "guide",
        require_seated_velocity=False,
        keyed=True,
        grasp_closing_axis=(0.0, 1.0, 0.0),
    ),
    "cylindrical_plug_insert": PlugProfile(
        "cylindrical_plug_insert",
        "圆柱插头",
        "cylindrical_plug",
        "cylindrical",
        grasp_closing_axis=(0.0, 1.0, 0.0),
        grasp_offset_m=(0., 0., -.006),
        insert_orientation_tolerance_rad=.008,
        extraction_velocity_gain=6.,
        release_seat_tolerance_m=.0001,
    ),
}


def plug_profile(task):
    return PLUG_PROFILES["cylindrical_plug_insert" if task == "plug_insert" else task]


def is_plug_task(task):
    return task == "plug_insert" or task in PLUG_PROFILES
