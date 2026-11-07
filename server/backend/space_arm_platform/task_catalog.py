"""Pure task identities shared by collection, instructions, APIs and tools.

No controller or simulation runtime is imported here. Collection task order is
part of seed derivation; preserve existing positions when extending the catalog.
The registered plug alias retains its cylindrical meaning. Its diagnostic
warmup belongs to the runner, rather than identity normalization.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

TaskName = Literal[
    "drawer_cycle", "aviation_plug_insert", "guide_plug_insert",
    "cylindrical_plug_insert", "locking_rod_cycle", "aviation_bolt_insert",
    "drawer_open", "drawer_close", "locking_rod_fold", "locking_rod_restore",
    "capture_target_grasp",
]
TASK_CHOICES = tuple(TaskName.__args__)
TASK_ALIASES = {"plug_insert": "cylindrical_plug_insert"}
DRAWER_TASKS = ("drawer_cycle", "drawer_open", "drawer_close")
PLUG_TASKS = ("aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert")
LOCKING_ROD_TASKS = ("locking_rod_cycle", "locking_rod_fold", "locking_rod_restore")
THREADED_TASKS = ("aviation_bolt_insert", *LOCKING_ROD_TASKS)
SCRIPTED_TASK_CHOICES = (*DRAWER_TASKS, *PLUG_TASKS, *THREADED_TASKS, *TASK_ALIASES, "capture_target_grasp")
RUNNER_TASK_CHOICES = ("drawer_lift", *SCRIPTED_TASK_CHOICES)
# Product support is independent of the repository's full supported set and of
# the tasks enabled in an individual batch. This declared product subset is
# not evidence of physical or UE certification for a new build.
COLLECTOR_TASK_CHOICES = (
    "drawer_cycle", "aviation_plug_insert", "guide_plug_insert",
    "cylindrical_plug_insert", "aviation_bolt_insert", "locking_rod_cycle",
)
# Preserve the instruction bank's established iteration order.
INSTRUCTION_TASKS = (*TASK_CHOICES[:4], "aviation_bolt_insert", "locking_rod_cycle", *TASK_CHOICES[6:])
COLOR_TASKS = frozenset((*PLUG_TASKS, "aviation_bolt_insert"))


@dataclass(frozen=True)
class TaskMetadata:
    name: str
    label: str
    short_label: str
    family: str
    instruction: str
    description: str
    target_site: str | None = None
    default_duration_s: float = 360.0
    template: str = "sarm-task-box-20261012"


_TASKS = (
    TaskMetadata("capture_target_grasp", "近距四爪抓捕目标星", "四爪抓捕", "capture",
                 "Capture the visible target interface securely with all four jaws, then bring the target spacecraft to the operating-arm working pose and hold it steady.",
                 "双星初始相对静止；对准、接近、闭爪、稳定夹持，再以真实接触搬至操作位置。",
                 default_duration_s=1200.0, template="sarm-dual-capture26"),
    TaskMetadata("drawer_cycle", "抽屉取出、放回与撤离", "抽屉往返", "drawer",
                 "Open the drawer approximately 8 cm, then close it fully and release it.",
                 "接近 → 抓取 → 抽出 → 放回 → 松爪 → 撤离。"),
    TaskMetadata("aviation_plug_insert", "航空插头拿取与插入", "航空插头", "plug",
                 "Pick up the aviation plug, align it with the corresponding socket, and insert it fully.",
                 "独立任务：跟踪并抓取航空插头 → 搬到前方对应插孔 → 对准插入 → 松爪撤离 → 验收。", "aviation_assembly_hole"),
    TaskMetadata("guide_plug_insert", "导向插头拿取与插入", "导向插头", "plug",
                 "Pick up the guide plug, align it with the corresponding socket, and insert it fully.",
                 "独立任务：跟踪并抓取导向插头 → 搬到前方对应插孔 → 对准插入 → 松爪撤离 → 验收。", "guide_assembly_hole"),
    TaskMetadata("cylindrical_plug_insert", "圆柱插头拿取与插入", "圆柱插头", "plug",
                 "Pick up the cylindrical plug, align it with the corresponding socket, and insert it fully.",
                 "独立任务：跟踪并抓取圆柱插头 → 搬到前方对应插孔 → 对准插入 → 松爪撤离 → 验收。", "cylindrical_assembly_hole"),
    TaskMetadata("locking_rod_cycle", "杆状机构放倒与复位", "锁杆往返", "threaded",
                 "Loosen the locking rod's fastener, fold the rod into its clip, then raise it and tighten the fastener.",
                 "旋松防脱紧固件 → 绕底部铰链放倒至卡扣 → 松爪再抓取 → 扶起复位 → 旋紧 → 撤离验收。", default_duration_s=900.0),
    TaskMetadata("aviation_bolt_insert", "航空螺丝旋出与装配", "航空螺丝", "threaded",
                 "Unscrew the aviation bolt from its holder, then screw it fully into the corresponding hole.",
                 "抓取初始孔内螺丝 → 旋出取下 → 搬运对孔 → 旋入拧紧 → 松爪撤离 → 验收。", "bolt_assembly_hole", 900.0),
    TaskMetadata("drawer_open", "抽屉拉起", "抽屉拉起", "drawer",
                 "Open the drawer approximately 8 cm, then release it and withdraw the gripper.",
                 "关闭初态 → 接近并抓取 → 解锁拉起约 8 cm → 松爪撤离，保持打开。"),
    TaskMetadata("drawer_close", "抽屉放回", "抽屉放回", "drawer",
                 "Close the open drawer fully, then release it and withdraw the gripper.",
                 "已打开约 8 cm 的初态 → 抓取 → 放回并锁定 → 松爪撤离。"),
    TaskMetadata("locking_rod_fold", "锁杆装置放倒", "锁杆放倒", "threaded",
                 "Loosen the locking rod's fastener, then fold the rod into its clip and release it.",
                 "竖起锁紧初态 → 旋松紧固件 → 放倒入卡扣 → 松爪撤离。", default_duration_s=900.0),
    TaskMetadata("locking_rod_restore", "锁杆装置复位", "锁杆复位", "threaded",
                 "Raise the locking rod from its clip, tighten its fastener, then release it.",
                 "放倒入卡扣初态 → 抓取扶起 → 旋紧紧固件 → 松爪撤离。", default_duration_s=900.0),
)
TASK_CATALOG = {task.name: task for task in _TASKS}
TASK_LABELS = {name: task.label for name, task in TASK_CATALOG.items()}


def normalize_task_name(name: str) -> str:
    """Resolve registered aliases without guessing unknown identities."""
    return TASK_ALIASES.get(name, name)


def task_metadata(name: str) -> TaskMetadata:
    return TASK_CATALOG[normalize_task_name(name)]


def task_default_duration_s(name: str) -> float:
    task = TASK_CATALOG.get(normalize_task_name(name))
    return task.default_duration_s if task else 360.0
