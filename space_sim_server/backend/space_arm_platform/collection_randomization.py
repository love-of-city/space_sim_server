"""Validated collection options shared by API, CLI and simulator.

Ranges are engineering guardrails, not calibrated hardware noise models.
"""

from typing import Literal
from pydantic import BaseModel, ConfigDict, Field, StrictBool


class Option(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    enabled: StrictBool = False


class JointNoise(Option):
    position_std_deg: float = Field(default=0.02, ge=0, le=1)
    velocity_std_deg_s: float = Field(default=0.1, ge=0, le=5)


class JointBias(Option):
    max_abs_deg: float = Field(default=0.1, ge=0, le=2)


class ActionNoise(Option):
    std_deg: float = Field(default=0.03, ge=0, le=0.5)
    correlation_s: float = Field(default=0.2, ge=0.05, le=5)


class ObservationDelay(Option):
    delay_ms: int = Field(default=20, ge=0, le=200, multiple_of=5)


class WrenchNoise(Option):
    force_std_n: float = Field(default=0.1, ge=0, le=5)
    torque_std_nm: float = Field(default=0.005, ge=0, le=0.5)


class ExternalDisturbance(Option):
    force_std_n: float = Field(default=0.1, ge=0, le=5)
    torque_std_nm: float = Field(default=0.005, ge=0, le=0.5)
    interval_s: float = Field(default=1, ge=0.1, le=10)
    # Keep the target explicit and validate it against supported model bodies.
    body: Literal["ee_base", "cubesat_bus", "capture_target"] = "ee_base"


class Lighting(Option):
    variation_pct: float = Field(default=20, ge=0, le=80)


class Dynamics(Option):
    mass_variation_pct: float = Field(default=5, ge=0, le=20)
    friction_variation_pct: float = Field(default=5, ge=0, le=30)


class CollectionRandomization(BaseModel):
    model_config = ConfigDict(extra="forbid")
    joint_observation_noise: JointNoise = Field(default_factory=JointNoise)
    joint_bias: JointBias = Field(default_factory=JointBias)
    action_noise: ActionNoise = Field(default_factory=ActionNoise)
    observation_delay: ObservationDelay = Field(default_factory=ObservationDelay)
    wrist_wrench_noise: WrenchNoise = Field(default_factory=WrenchNoise)
    external_disturbance: ExternalDisturbance = Field(
        default_factory=ExternalDisturbance
    )
    lighting: Lighting = Field(default_factory=Lighting)
    dynamics: Dynamics = Field(default_factory=Dynamics)

    def flags(self):
        return {key: value.enabled for key, value in self}


# A single catalog drives UI labels, input constraints, units and defaults.
LABELS = {
    "joint_observation_noise": (
        "关节观测噪声",
        "对六个关节的录制角度、角速度添加高斯测量噪声；不改变真实物理状态。",
    ),
    "joint_bias": ("关节测量偏置", "每条记录抽取六个固定角度偏置，整条保持不变。"),
    "action_noise": (
        "运动指令扰动",
        "在六轴目标角度上叠加有时间相关性的随机偏差，实际作用于机械臂。",
    ),
    "observation_delay": (
        "关节观测延迟",
        "延迟录制中的关节与推导末端观测，保留测量时刻；RGB 和六维力维持原采样时钟。",
    ),
    "wrist_wrench_noise": (
        "六维力传感器噪声",
        "在传感器坐标系中添加高斯噪声，50 Hz 与 200 Hz 共用同一读数，并保留真实值。",
    ),
    "external_disturbance": (
        "外部扰动力",
        "向选定刚体质心施加平滑变化的随机力和力矩；安全保护使用真实载荷。",
    ),
    "lighting": ("光照随机化", "每条记录随机改变太阳光强，整条保持同一倍率。"),
    "dynamics": (
        "动力学参数随机化",
        "每条记录随机缩放全模型质量及惯量、接触摩擦；几何保持不变。",
    ),
}
PARAMETERS = {
    "position_std_deg": ("角度标准差（°）", 0.01),
    "velocity_std_deg_s": ("角速度标准差（°/s）", 0.05),
    "max_abs_deg": ("固定偏置范围（±°）", 0.05),
    "std_deg": ("目标角度噪声尺度（°）", 0.01),
    "correlation_s": ("相关时间（秒）", 0.05),
    "delay_ms": ("延迟（毫秒，5 的倍数）", 5),
    "force_std_n": ("力噪声尺度（N）", 0.01),
    "torque_std_nm": ("力矩噪声尺度（N·m）", 0.001),
    "interval_s": ("外力变化间隔（秒）", 0.1),
    "body": ("受力刚体", None),
    "variation_pct": ("光强变化范围（±%）", 1),
    "mass_variation_pct": ("质量及惯量变化范围（±%）", 1),
    "friction_variation_pct": ("摩擦变化范围（±%）", 1),
}


def randomization_catalog():
    defaults = CollectionRandomization()
    result = []
    for key, option in defaults:
        fields = []
        for name, schema in option.model_json_schema()["properties"].items():
            if name == "enabled":
                continue
            label, step = PARAMETERS[name]
            fields.append(dict(name=name, label=label, step=step, **schema))
        label, description = LABELS[key]
        result.append(
            dict(
                key=key,
                label=label,
                description=description,
                default=option.model_dump(),
                fields=fields,
            )
        )
    return result
