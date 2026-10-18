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
    feedback_extraction: bool = False

    @property
    def target_site(self):
        return self.prefix + "_insert_40p5"

    @property
    def instruction(self):
        return f"跟踪并抓取{self.label}，搬到前方对应装配孔，对准后插入，松爪撤离并确认保持就位。"


PLUG_PROFILES = {
    "aviation_plug_insert": PlugProfile(
        "aviation_plug_insert",
        "航空插头",
        "aviation_plug",
        "aviation",
        0.080,
        keyed=True,
    ),
    "guide_plug_insert": PlugProfile(
        "guide_plug_insert",
        "导向插头",
        "guide_plug",
        "guide",
        keyed=True,
        feedback_extraction=True,
    ),
    "cylindrical_plug_insert": PlugProfile(
        "cylindrical_plug_insert",
        "圆柱插头",
        "cylindrical_plug",
        "cylindrical",
        feedback_extraction=True,
    ),
}


def plug_profile(task):
    return PLUG_PROFILES["cylindrical_plug_insert" if task == "plug_insert" else task]


def is_plug_task(task):
    return task == "plug_insert" or task in PLUG_PROFILES
