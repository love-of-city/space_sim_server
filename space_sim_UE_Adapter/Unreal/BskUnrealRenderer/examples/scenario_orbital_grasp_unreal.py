"""500 km 地球轨道上的卫星交会、机械臂抓取与 UE 可视化示例。

本文件是完整任务的编排层，适合按以下顺序阅读：

1. :data:`RW_DEFINITIONS` 和任务常量定义硬件参数、时间线与控制限幅；
2. :func:`build_orbital_mjcf` 在原抓取模型上增加机载相机、三个反作用轮和
   一对相反方向的平移推进器，始终不修改外部原始 XML；
3. :func:`run` 复用原生 SO-101 抓取场景的机械臂 PID 与接触模型，再加入
   地球引力、轨道初值、Basilisk 姿态 FSW 和相对接近控制器；
4. Basilisk/MJScene 以 500 Hz 统一推进全部权威动力学，渲染桥仅以约
   30 Hz 读取并发送状态，UE 不参与动力学计算；
5. 任务结束后计算轨道、交会、姿态、反作用轮和抓取接触指标。

任务时间线（单位：仿真秒）：

* 0～2 s：共轨飞行并保持初始 0.75 m 接近偏置；
* 2～20 s：沿对接相机视轴执行平滑闭环接近；
* 20～22 s：目标附近定点保持；
* 22～26.5 s：机械臂从预抓取姿态伸向目标，平移控制继续保持相对位置；
* 26.5～28 s：夹爪闭合；
* 29～31 s：机械臂携带目标回撤约 3 cm；
* 31 s 以后：保持最终抓取状态并完成验收。

坐标和单位约定：动力学全部使用 SI 单位；MJCF 四元数为 wxyz；
``DOCKING_AXIS_BODY`` 在卫星机体系中表达。UE 坐标映射和米到厘米转换由
``bsk_render_adapter`` 完成，不应在本文件内重复转换。
"""

from __future__ import annotations
# 逐行说明：启用延迟解析类型注解，避免运行时立即求值前向类型引用。

import argparse
# 逐行说明：导入 `argparse` 模块，供后续相关计算或系统调用使用。
import json
# 逐行说明：导入 `json` 模块，供后续相关计算或系统调用使用。
import math
# 逐行说明：导入 `math` 模块，供后续相关计算或系统调用使用。
import os
# 逐行说明：导入 `os` 模块，供后续相关计算或系统调用使用。
from pathlib import Path
# 逐行说明：从 `pathlib` 导入 `Path`，供后续场景构建直接使用。
import subprocess
# 逐行说明：导入 `subprocess` 模块，供后续相关计算或系统调用使用。
import sys
# 逐行说明：导入 `sys` 模块，供后续相关计算或系统调用使用。
import tempfile
# 逐行说明：导入 `tempfile` 模块，供后续相关计算或系统调用使用。
import time
# 逐行说明：导入 `time` 模块，供后续相关计算或系统调用使用。
import xml.etree.ElementTree as ET
# 逐行说明：导入 `xml.etree.ElementTree as ET` 模块，供后续相关计算或系统调用使用。

import numpy as np
# 逐行说明：导入 `numpy as np` 模块，供后续相关计算或系统调用使用。


# ---------------------------------------------------------------------------
# 1. 航天器、控制器与任务时间线常量
# ---------------------------------------------------------------------------

# 三个反作用轮与 MuJoCo 刚体/关节/执行器的稳定名称映射。
# axis_body 是轮轴在卫星机体系中的方向；三个正交轮分别控制滚转、俯仰和偏航。
# position_body_m 只定义轮子在卫星上的安装位置，不改变其控制轴。
RW_DEFINITIONS = (
# 逐行说明：定义三个反作用轮的刚体、铰链、执行器、安装轴和显示属性。
    {
    # 逐行说明：开始构造当前列表、元组或字典表达式。
        "body_name": "orbit_rw_x",
        # 逐行说明：设置权威 MuJoCo 反作用轮刚体名称。
        "joint_name": "orbit_rw_x_spin",
        # 逐行说明：设置反作用轮连续旋转铰链名称。
        "actuator_name": "orbit_rw_x_motor",
        # 逐行说明：设置向反作用轮铰链施加力矩的电机名称。
        "position_body_m": (0.0, 0.132, 0.0),
        # 逐行说明：设置设备在卫星机体系中的安装位置，单位为 m。
        "axis_body": (1.0, 0.0, 0.0),
        # 逐行说明：设置设备轴线在卫星机体系中的单位方向。
        "color_rgba": (0.18, 0.45, 1.0, 1.0),
        # 逐行说明：设置 UE 可视元素的线性 RGBA 颜色。
        "label": "RW-X",
        # 逐行说明：设置任务 UI 中显示的设备名称。
    },
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    {
    # 逐行说明：开始构造当前列表、元组或字典表达式。
        "body_name": "orbit_rw_y",
        # 逐行说明：设置权威 MuJoCo 反作用轮刚体名称。
        "joint_name": "orbit_rw_y_spin",
        # 逐行说明：设置反作用轮连续旋转铰链名称。
        "actuator_name": "orbit_rw_y_motor",
        # 逐行说明：设置向反作用轮铰链施加力矩的电机名称。
        "position_body_m": (-0.132, 0.0, 0.0),
        # 逐行说明：设置设备在卫星机体系中的安装位置，单位为 m。
        "axis_body": (0.0, 1.0, 0.0),
        # 逐行说明：设置设备轴线在卫星机体系中的单位方向。
        "color_rgba": (0.15, 0.9, 0.38, 1.0),
        # 逐行说明：设置 UE 可视元素的线性 RGBA 颜色。
        "label": "RW-Y",
        # 逐行说明：设置任务 UI 中显示的设备名称。
    },
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    {
    # 逐行说明：开始构造当前列表、元组或字典表达式。
        "body_name": "orbit_rw_z",
        # 逐行说明：设置权威 MuJoCo 反作用轮刚体名称。
        "joint_name": "orbit_rw_z_spin",
        # 逐行说明：设置反作用轮连续旋转铰链名称。
        "actuator_name": "orbit_rw_z_motor",
        # 逐行说明：设置向反作用轮铰链施加力矩的电机名称。
        "position_body_m": (0.0, 0.0, -0.188),
        # 逐行说明：设置设备在卫星机体系中的安装位置，单位为 m。
        "axis_body": (0.0, 0.0, 1.0),
        # 逐行说明：设置设备轴线在卫星机体系中的单位方向。
        "color_rgba": (1.0, 0.22, 0.15, 1.0),
        # 逐行说明：设置 UE 可视元素的线性 RGBA 颜色。
        "label": "RW-Z",
        # 逐行说明：设置任务 UI 中显示的设备名称。
    },
    # 逐行说明：结束当前多行容器、函数调用或表达式。
)
# 逐行说明：结束当前多行容器、函数调用或表达式。

# 单轮质量、主轴惯量、横向惯量、最大控制力矩和最大轮速。
# 轮速上限从 6000 rpm 换算为 rad/s，供任务结束后的饱和检查使用。
RW_MASS_KG = 0.08
# 逐行说明：设置单个反作用轮的质量，单位为 kg。
RW_SPIN_INERTIA = 2.5e-5
# 逐行说明：设置反作用轮绕自转轴的主惯量，单位为 kg·m²。
RW_TRANSVERSE_INERTIA = 1.65e-5
# 逐行说明：设置反作用轮垂直于自转轴的横向惯量。
RW_TORQUE_LIMIT_NM = 0.003
# 逐行说明：设置单轮允许的最大正负控制力矩。
RW_SPEED_LIMIT_RAD_S = 6000.0 * 2.0 * math.pi / 60.0
# 逐行说明：把 6000 rpm 换算为 rad/s，作为轮速验收上限。

# 统一动力学步长 2 ms，即 500 Hz。机械臂 PID、接触、姿态 FSW、反作用轮、
# 推进器和轨道传播都在同一任务时钟上执行，避免多速率状态先后关系难以理解。
MISSION_TIME_STEP_S = 0.002
# 逐行说明：设置统一动力学步长为 2 ms，也就是 500 Hz。
ORBIT_ALTITUDE_M = 500_000.0
# 逐行说明：设置卫星圆轨道高度为 500 km。
SPICE_EPOCH = "2025 NOVEMBER 15 12:00:00.000"
# 逐行说明：设置任务绝对历元；Sun 和 Earth 的星历状态都由 Basilisk SPICE 在该历元起算。
SUN_REFERENCE_DISTANCE_M = 149_597_870_693.0
# 逐行说明：采用一历元天文单位作为太阳渲染照度的参考距离。

# 目标在正式接近开始前额外位于抓取构型前方 0.75 m。
RENDEZVOUS_INITIAL_OFFSET_M = 0.75
# 逐行说明：设置正式接近开始前额外的 0.75 m 轴向间距。
DOCKING_CAMERA_POSITION_BODY_M = np.array([0.06, -0.08, 0.19])
# 逐行说明：记录对接相机在卫星机体系中的安装位置。
# 从机载对接相机指向已验证最终抓取点的单位向量。
# 平移接近方向、相机视轴和机械臂工作方向有意保持共线，便于导航与抓取。
DOCKING_AXIS_BODY = np.array([0.79852285, 0.19236641, 0.57040023])
# 逐行说明：定义相机视轴、接近方向和抓取方向共用的机体系单位向量。

# 交会控制时间：2～20 s 接近，20～22 s 定点保持；机械臂开始工作后继续
# 保持到 26.5 s，之后关闭平移推进器，避免夹爪接触时控制器相互对抗。
RENDEZVOUS_START_S = 2.0
# 逐行说明：设置闭环接近开始时刻。
RENDEZVOUS_STOP_S = 20.0
# 逐行说明：设置 0.75 m 接近轨迹完成时刻。
STATION_KEEP_STOP_S = 22.0
# 逐行说明：设置定点保持结束及机械臂任务开始时刻。
GRASP_TIME_OFFSET_S = STATION_KEEP_STOP_S
# 逐行说明：把机械臂局部任务时间原点设在定点保持结束时刻。
RENDEZVOUS_CONTROL_STOP_S = GRASP_TIME_OFFSET_S + 4.5
# 逐行说明：设置平移控制器停止输出推进器命令的时刻。
GRASP_SEQUENCE_DURATION_S = 12.0
# 逐行说明：设置机械臂伸出、闭合和回撤序列的总时长。
MINIMUM_MISSION_DURATION_S = GRASP_TIME_OFFSET_S + GRASP_SEQUENCE_DURATION_S
# 逐行说明：计算能够完整执行抓取序列的最短任务时长。

# 接近控制器参数。0.60 N 是双向合成后的绝对推力上限；控制器使用估计质量
# 将参考相对加速度换算为前馈力，并叠加位置/速度 PD 反馈。
RENDEZVOUS_MAX_THRUST_N = 0.60
# 逐行说明：设置接近或制动方向的最大推力为 0.60 N。
RENDEZVOUS_ESTIMATED_MASS_KG = 24.984
# 逐行说明：设置平移控制器用于加速度前馈的航天器估计质量。
RENDEZVOUS_POSITION_GAIN_N_PER_M = 6.0
# 逐行说明：设置相对位置误差到推力的比例反馈增益。
RENDEZVOUS_VELOCITY_GAIN_N_PER_MPS = 20.0
# 逐行说明：设置相对速度误差到推力的微分反馈增益。
TIGHT_GRIP_JOINT_RAD = -0.16
# 逐行说明：设置夹爪稳定夹紧目标时的关节角。
# 机械臂伸出构型：这是朝逆运动学解移动的保守构型，使夹爪参考点靠近把手，
# 同时保留原工具方向，避免一步到位造成过大的关节力矩或接触冲击。
EXTENDED_ALIGNED_ARM = np.array(
# 逐行说明：定义机械臂伸向目标并与把手对齐的六关节构型。
    [0.0, 0.10916789, -0.19346612, 0.08429824, 0.0, 0.4]
    # 逐行说明：开始构造当前列表、元组或字典表达式。
)
# 逐行说明：结束当前多行容器、函数调用或表达式。
# 抓住目标后的回撤构型：沿对接轴拉回约 3 cm，工具方向只改变约 1°。
RETRACTED_GRASP_ARM = np.array(
# 逐行说明：定义夹紧目标后向卫星方向回撤的六关节构型。
    [0.01690432, -0.11100908, 0.19918142, -0.08815929, -0.00059566, -0.16]
    # 逐行说明：开始构造当前列表、元组或字典表达式。
)
# 逐行说明：结束当前多行容器、函数调用或表达式。


def _quintic_rendezvous_reference(seconds: float) -> tuple[float, float, float]:
# 逐行说明：开始定义 `_quintic_rendezvous_reference` 函数，具体职责由紧随其后的中文文档字符串说明。
    """返回沿对接轴的累计接近距离、速度和加速度参考。

    五次平滑函数 ``10s³-15s⁴+6s⁵`` 在区间两端的位置连续，并且速度、
    加速度都为零，因此推进器不会在 2 s 或 20 s 产生阶跃命令。

    返回值中的距离从 0 增加到 0.75 m；在相对状态
    ``目标位置 - 卫星位置`` 中，这意味着期望间距逐渐减少 0.75 m。
    """

    if seconds <= RENDEZVOUS_START_S:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        return 0.0, 0.0, 0.0
        # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
    if seconds >= RENDEZVOUS_STOP_S:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        return RENDEZVOUS_INITIAL_OFFSET_M, 0.0, 0.0
        # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
    duration = RENDEZVOUS_STOP_S - RENDEZVOUS_START_S
    # 逐行说明：接收本次任务需要推进的仿真时长。
    phase = (seconds - RENDEZVOUS_START_S) / duration
    # 逐行说明：计算当前任务阶段名称。
    # blend、blend_rate、blend_acceleration 分别是归一化位移及其一、二阶导数。
    blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `blend` 供后续步骤使用。
    blend_rate = (30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4) / duration
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `blend_rate` 供后续步骤使用。
    blend_acceleration = (
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `blend_acceleration` 供后续步骤使用。
        60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3
        # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
    ) / duration**2
    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
    return (
    # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
        RENDEZVOUS_INITIAL_OFFSET_M * blend,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        RENDEZVOUS_INITIAL_OFFSET_M * blend_rate,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        RENDEZVOUS_INITIAL_OFFSET_M * blend_acceleration,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。


def build_orbital_mjcf(source: Path, output: Path, mesh_directory: Path) -> Path:
# 逐行说明：开始定义 `build_orbital_mjcf` 函数，具体职责由紧随其后的中文文档字符串说明。
    """从原抓取 XML 生成在轨任务专用 MJCF，且绝不修改外部原文件。

    原模型已经包含 CubeSat、SO-101、目标和接触几何。本函数只在内存中的
    XML 树上补充：网格绝对路径、机载对接相机、三个反作用轮刚体/铰链、
    三个轮电机以及一对相反方向的平移执行器，最后写入 ``Saved/Generated``。

    这种派生方式让原始模型仍可被原生 MuJoCo Demo 使用，也便于逐项比较新增
    航天语义前后的动力学差异。
    """

    # 解析源 XML，并把 meshdir 指向外部 SO-101 STL 所在目录。使用绝对路径
    # 可避免生成文件换到 Saved/Generated 后相对路径失效。
    root = ET.parse(source).getroot()
    # 逐行说明：保存解析后的 MJCF XML 根节点。
    compiler = root.find("./compiler")
    # 逐行说明：保存 MJCF compiler 配置节点。
    if compiler is None:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        compiler = ET.Element("compiler")
        # 逐行说明：保存 MJCF compiler 配置节点。
        root.insert(0, compiler)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
    compiler.set("meshdir", mesh_directory.resolve().as_posix())
    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。

    # 所有机载设备都挂到 cubesat_bus 下，因此会自然继承卫星的世界位姿。
    bus = root.find(".//body[@name='cubesat_bus']")
    # 逐行说明：取得并保存 CubeSat 服务卫星刚体句柄。
    if bus is None:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        raise ValueError("cubesat_bus was not found in the source MJCF")
        # 逐行说明：检测到无法继续的输入或运行状态，立即抛出明确异常。
    # 这是物理意义上的机载导航相机，不是 UE 的观众自由相机。
    # 把安装位姿写入 MJCF，可让 MuJoCo、Vizard、UE 等渲染端共享同一相机定义，
    # 并保证相机始终刚性跟随卫星本体。
    ET.SubElement(
    # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
        bus,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        "camera",
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        {
        # 逐行说明：开始构造当前列表、元组或字典表达式。
            "name": "cubesat_docking_camera",
            # 逐行说明：设置当前 MJCF 元素的稳定名称。
            "pos": "0.06 -0.08 0.19",
            # 逐行说明：设置结构中的 `pos` 字段，供下游模块按稳定键名读取。
            # MuJoCo 相机约定：-Z 为前方、+Y 为上方。该四元数已经过视轴验证，
            # 使相机朝向机械臂最终抓取区域。
            "quat": "0.38279638 0.65706406 -0.59453085 -0.26127919",
            # 逐行说明：设置当前元素采用 wxyz 顺序的安装四元数。
            "fovy": "60",
            # 逐行说明：设置结构中的 `fovy` 字段，供下游模块按稳定键名读取。
            "resolution": "1280 720",
            # 逐行说明：设置结构中的 `resolution` 字段，供下游模块按稳定键名读取。
        },
        # 逐行说明：结束当前多行容器、函数调用或表达式。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    # 每个轮子是卫星子刚体，并通过无限位 hinge 关节连续旋转。
    # 最后一个四元数用于把默认圆柱轴转到所需的 X/Y/Z 轮轴方向。
    wheel_xml = (
    # 逐行说明：定义生成三个正交反作用轮所需的 XML 参数。
        ("orbit_rw_x", "orbit_rw_x_spin", "0 0.132 0", "1 0 0", "0.7071067812 0 0.7071067812 0"),
        # 逐行说明：开始构造当前列表、元组或字典表达式。
        ("orbit_rw_y", "orbit_rw_y_spin", "-0.132 0 0", "0 1 0", "0.7071067812 -0.7071067812 0 0"),
        # 逐行说明：开始构造当前列表、元组或字典表达式。
        ("orbit_rw_z", "orbit_rw_z_spin", "0 0 -0.188", "0 0 1", "1 0 0 0"),
        # 逐行说明：开始构造当前列表、元组或字典表达式。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    for name, joint_name, position, axis, quaternion in wheel_xml:
    # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
        body = ET.SubElement(bus, "body", {"name": name, "pos": position})
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `body` 供后续步骤使用。
        ET.SubElement(
        # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
            body,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            "joint",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            {
            # 逐行说明：开始构造当前列表、元组或字典表达式。
                "name": joint_name,
                # 逐行说明：设置当前 MJCF 元素的稳定名称。
                "type": "hinge",
                # 逐行说明：设置当前 MJCF 元素或字段的类型。
                "axis": axis,
                # 逐行说明：设置铰链允许旋转的轴向。
                "limited": "false",
                # 逐行说明：声明反作用轮铰链不受角度范围限制，可连续旋转。
                "damping": "0.000002",
                # 逐行说明：设置铰链轴承的等效粘性阻尼。
            },
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # MuJoCo 的 diaginertia 依次为 Ixx、Iyy、Izz；主轴方向使用较大的
        # RW_SPIN_INERTIA，其余两个方向使用横向惯量。
        inertia = (
        # 逐行说明：按当前轮轴方向排列 Ixx、Iyy、Izz 三个主惯量。
            f"{RW_SPIN_INERTIA} {RW_TRANSVERSE_INERTIA} {RW_TRANSVERSE_INERTIA}"
            # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            if axis == "1 0 0"
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            else f"{RW_TRANSVERSE_INERTIA} {RW_SPIN_INERTIA} {RW_TRANSVERSE_INERTIA}"
            # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            if axis == "0 1 0"
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            else f"{RW_TRANSVERSE_INERTIA} {RW_TRANSVERSE_INERTIA} {RW_SPIN_INERTIA}"
            # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        ET.SubElement(
        # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
            body,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            "inertial",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            {"pos": "0 0 0", "mass": str(RW_MASS_KG), "diaginertia": inertia},
            # 逐行说明：开始构造当前列表、元组或字典表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 权威反作用轮是参与 MJScene 动力学的刚体。这个 geom 只向 MuJoCo 提供
        # 尺寸，alpha=0 使其不直接显示；UE 中可见的彩色轮由通用反作用轮
        # VisualElement 根据真实轮速和力矩进行动画，避免同一轮子显示两遍。
        ET.SubElement(
        # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
            body,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            "geom",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            {
            # 逐行说明：开始构造当前列表、元组或字典表达式。
                "type": "cylinder",
                # 逐行说明：设置当前 MJCF 元素或字段的类型。
                "size": "0.025 0.012",
                # 逐行说明：设置当前几何体尺寸。
                "quat": quaternion,
                # 逐行说明：设置当前元素采用 wxyz 顺序的安装四元数。
                "mass": "0",
                # 逐行说明：设置当前刚体质量。
                "contype": "0",
                # 逐行说明：设置碰撞类型位掩码；零表示不主动参与碰撞匹配。
                "conaffinity": "0",
                # 逐行说明：设置可碰撞对象位掩码；零表示该辅助几何不碰撞。
                "group": "2",
                # 逐行说明：设置 MuJoCo 几何显示/筛选分组。
                "rgba": "0.2 0.5 1 0",
                # 逐行说明：设置 MJCF 几何颜色；alpha=0 时仅保留动力学尺寸而不显示。
            },
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。

    # 给三个铰链增加直接力矩电机。电机输入稍后由 Basilisk 姿态 FSW 计算，
    # 再经过 ±RW_TORQUE_LIMIT_NM 限幅后接入。
    actuators = root.find("./actuator")
    # 逐行说明：保存 MJCF actuator 容器节点。
    if actuators is None:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        actuators = ET.SubElement(root, "actuator")
        # 逐行说明：保存 MJCF actuator 容器节点。
    for definition in RW_DEFINITIONS:
    # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
        ET.SubElement(
        # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
            actuators,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            "motor",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            {
            # 逐行说明：开始构造当前列表、元组或字典表达式。
                "name": definition["actuator_name"],
                # 逐行说明：设置当前 MJCF 元素的稳定名称。
                "joint": definition["joint_name"],
                # 逐行说明：指定当前电机实际驱动的关节。
            },
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
    # 两个 general actuator 是方向相反的理想单向推进器。
    # gear 的前 3 项是机体系力方向，后 3 项为直接施加的力矩（这里均为 0）。
    # approach 沿 +DOCKING_AXIS_BODY 推进卫星，braking 沿反方向减速/纠偏。
    ET.SubElement(
    # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
        actuators,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        "general",
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        {
        # 逐行说明：开始构造当前列表、元组或字典表达式。
            "name": "docking_approach_thruster",
            # 逐行说明：设置当前 MJCF 元素的稳定名称。
            "site": "cubesat_origin",
            # 逐行说明：指定推进器作用的机载 site。
            "gear": "0.79852285 0.19236641 0.57040023 0 0 0",
            # 逐行说明：设置执行器的三维施力方向和三维直接力矩。
        },
        # 逐行说明：结束当前多行容器、函数调用或表达式。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    ET.SubElement(
    # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
        actuators,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        "general",
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        {
        # 逐行说明：开始构造当前列表、元组或字典表达式。
            "name": "docking_braking_thruster",
            # 逐行说明：设置当前 MJCF 元素的稳定名称。
            "site": "cubesat_origin",
            # 逐行说明：指定推进器作用的机载 site。
            "gear": "-0.79852285 -0.19236641 -0.57040023 0 0 0",
            # 逐行说明：设置执行器的三维施力方向和三维直接力矩。
        },
        # 逐行说明：结束当前多行容器、函数调用或表达式。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    # 只写派生输出，原 source 文件保持只读不变。
    output.parent.mkdir(parents=True, exist_ok=True)
    # 逐行说明：确保输出目录存在，并允许递归创建父目录。
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    # 逐行说明：写出当前 Basilisk 消息或把生成结果保存到磁盘。
    return output
    # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。


def _specific_orbit(position: np.ndarray, velocity: np.ndarray, mu: float) -> tuple[float, float]:
# 逐行说明：开始定义 `_specific_orbit` 函数，具体职责由紧随其后的中文文档字符串说明。
    """由惯性位置/速度计算轨道半径和二体轨道半长轴。

    比机械能 ``epsilon=v²/2-mu/r``，椭圆轨道满足 ``a=-mu/(2*epsilon)``。
    该函数只用于任务结束后的数值验收，不参与飞行控制。
    """
    radius = float(np.linalg.norm(position))
    # 逐行说明：计算并保存当前惯性位置向量的模，也就是轨道半径。
    energy = 0.5 * float(np.dot(velocity, velocity)) - mu / radius
    # 逐行说明：计算二体轨道单位质量机械能。
    return radius, -mu / (2.0 * energy)
    # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。


def _contact_replay_metrics(
# 逐行说明：开始定义 `_contact_replay_metrics` 函数，具体职责由紧随其后的中文文档字符串说明。
    mjcf_path: Path,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    state_recorder,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    start_seconds: float,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    stop_seconds: float,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
) -> dict[str, float]:
# 逐行说明：开始该复合语句的缩进代码块。
    """在独立 Python/MuJoCo 子进程中重建接触并计算抓取指标。

    当前 Windows 环境中的 Basilisk 与 pip MuJoCo 会加载不兼容的 MuJoCo DLL，
    因而不能安全地在同一进程中同时导入。主进程先保存每个采样时刻的 qpos、
    qvel 和时间戳；子进程再用同一个 MJCF 恢复状态，重新调用接触查询并输出
    JSON 指标。这里是离线验收，不会把重放结果反馈到权威仿真。
    """

    replay_script = Path(__file__).with_name("orbital_grasp_contact_replay.py")
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `replay_script` 供后续步骤使用。
    with tempfile.TemporaryDirectory(prefix="bsk-orbital-contact-") as directory:
    # 逐行说明：进入上下文管理器，并在代码块结束时自动释放临时资源。
        # 临时 NPZ 只在分析期间存在，退出上下文后自动清理。
        state_path = Path(directory) / "states.npz"
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `state_path` 供后续步骤使用。
        np.savez_compressed(
        # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
            state_path,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            qpos=np.asarray(state_recorder.qpos, dtype=float),
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `qpos` 供后续步骤使用。
            qvel=np.asarray(state_recorder.qvel, dtype=float),
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `qvel` 供后续步骤使用。
            times_ns=np.asarray(state_recorder.times(), dtype=np.uint64),
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `times_ns` 供后续步骤使用。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        replay_environment = os.environ.copy()
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `replay_environment` 供后续步骤使用。
        replay_environment.pop("MUJOCO_GL", None)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        # 移除 MUJOCO_GL，接触重建不需要图形上下文，也避免 EGL 配置干扰。
        result = subprocess.run(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `result` 供后续步骤使用。
            [
            # 逐行说明：开始构造当前列表、元组或字典表达式。
                sys.executable,
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                str(replay_script),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                "--mjcf",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                str(mjcf_path),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                "--states",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                str(state_path),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                "--start",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                str(start_seconds),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                "--stop",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                str(stop_seconds),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            ],
            # 逐行说明：结束当前多行容器、函数调用或表达式。
            check=False,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `check` 供后续步骤使用。
            capture_output=True,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `capture_output` 供后续步骤使用。
            text=True,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `text` 供后续步骤使用。
            env=replay_environment,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `env` 供后续步骤使用。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        if result.returncode != 0:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            raise RuntimeError(
            # 逐行说明：检测到无法继续的输入或运行状态，立即抛出明确异常。
                "Python MuJoCo contact replay failed: "
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                + (result.stderr.strip() or result.stdout.strip())
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
    return json.loads(result.stdout)
    # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。


def run(
# 逐行说明：开始定义 `run` 函数，具体职责由紧随其后的中文文档字符串说明。
    model_root: Path,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    catalog: Path,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    host: str,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    port: int,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    duration: float,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    simulation_rate: float,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    generated_mjcf: Path,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    metrics_path: Path,
    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    recording_path: Path | None = None,
    # 逐行说明：可选 `.bskrec` 输出路径；由通用渲染桥记录完整场景、事件和状态帧。
    recording_only: bool = False,
    # 逐行说明：为真时不连接 UE，而是尽快完成动力学计算并只写录制文件。
) -> dict[str, float | int | bool]:
# 逐行说明：开始该复合语句的缩进代码块。
    """构建、运行并验收一次完整的在轨交会抓取任务。

    参数中的 ``duration`` 是仿真时长，``simulation_rate`` 是仿真时间相对
    墙钟时间的倍率；倍率只影响外层节奏控制，不会改变 500 Hz 数值步长。
    返回字典既包含连续数值指标，也包含可直接用于自动测试的布尔判据。
    """

    # 先验证输入，避免 UE 已启动后才因明显参数错误退出。
    if duration <= 0.0 or simulation_rate <= 0.0:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        raise ValueError("duration and simulation_rate must be positive")
        # 逐行说明：检测到无法继续的输入或运行状态，立即抛出明确异常。
    if recording_only and recording_path is None:
    # 逐行说明：仅录制模式必须有明确文件路径，防止运行完成后没有任何输出。
        raise ValueError("recording_path is required in recording-only mode")
        # 逐行说明：检测到缺失录制目标后立即报告配置错误。
    if duration < MINIMUM_MISSION_DURATION_S:
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        raise ValueError(f"duration must be at least {MINIMUM_MISSION_DURATION_S} seconds")
        # 逐行说明：检测到无法继续的输入或运行状态，立即抛出明确异常。
    if not catalog.is_file():
    # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
        raise FileNotFoundError(f"asset catalog is missing: {catalog}")
        # 逐行说明：检测到无法继续的输入或运行状态，立即抛出明确异常。

    # 延迟导入原生抓取模块很重要：该辅助函数会建立适合 Windows 的
    # Basilisk/MuJoCo DLL 加载顺序，避免两个不同 MuJoCo 构建发生冲突。
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    # 逐行说明：从 `scenario_spacecraft_arm_grasp_unreal` 导入 `load_native_grasp_module`，供后续场景构建直接使用。

    # 复用原场景已经验证过的机械臂 PID、接触几何、关节名称和记录器；
    # 本示例只在其上叠加航天轨道与姿态/交会控制，不复制第二套抓取算法。
    native = load_native_grasp_module(model_root)
    # 逐行说明：加载并保存原生机械臂抓取场景模块。
    augmented_path = build_orbital_mjcf(native.MODEL_PATH, generated_mjcf, native.MESH_DIR)
    # 逐行说明：保存加入相机、轮子和推进器后的派生 MJCF 路径。
    original_model_path = native.MODEL_PATH
    # 逐行说明：备份原生场景模型路径，任务构建后需要恢复。
    original_time_step = native.TIME_STEP
    # 逐行说明：备份原生场景时间步长，任务构建后需要恢复。
    original_reference_descriptor = native.JointTrajectoryPublisher.__dict__["reference"]
    # 逐行说明：备份原机械臂轨迹类方法描述符。
    extended_closed = EXTENDED_ALIGNED_ARM.copy()
    # 逐行说明：由伸出构型复制得到闭合夹爪构型。
    extended_closed[-1] = TIGHT_GRIP_JOINT_RAD
    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
    extended_withdraw = RETRACTED_GRASP_ARM.copy()
    # 逐行说明：复制抓取后回撤构型，避免后续原地修改常量。

    def mission_joint_reference(seconds: float) -> tuple[np.ndarray, np.ndarray]:
    # 逐行说明：开始定义 `mission_joint_reference` 函数，具体职责由紧随其后的中文文档字符串说明。
        """给六个关节生成分段平滑的位置/速度参考。

        ``local_seconds`` 以 22 s 机械臂任务起点为零。在此之前始终保持
        PREGRASP，因此卫星接近过程中机械臂不会成为自由摆动机构。
        每次构型切换均调用原场景的五次轨迹段，保证端点速度为零。
        """
        local_seconds = seconds - GRASP_TIME_OFFSET_S
        # 逐行说明：把全局仿真时间换算为从 22 s 开始的机械臂局部时间。
        # 22～23 s：保持预抓取姿态，给交会控制与机械臂任务留出交接时间。
        if local_seconds < 1.0:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            return native.PREGRASP.copy(), np.zeros(6)
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
        # 23～26 s：伸向目标把手。
        if local_seconds < 4.0:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            return native.JointTrajectoryPublisher._segment(
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
                local_seconds, 1.0, 4.0, native.PREGRASP, EXTENDED_ALIGNED_ARM
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 26～26.5 s：保持伸出构型，等待相对运动收敛。
        if local_seconds < 4.5:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            return EXTENDED_ALIGNED_ARM.copy(), np.zeros(6)
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
        # 26.5～28 s：只把最后一个夹爪关节从张开位置闭合到 -0.16 rad。
        if local_seconds < 6.0:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            return native.JointTrajectoryPublisher._segment(
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
                local_seconds, 4.5, 6.0, EXTENDED_ALIGNED_ARM, extended_closed
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 28～29 s：保持夹紧，让双侧接触稳定建立。
        if local_seconds < 7.0:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            return extended_closed.copy(), np.zeros(6)
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
        # 29～31 s：在保持夹爪闭合的同时把目标拉回约 3 cm。
        if local_seconds < 9.0:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            return native.JointTrajectoryPublisher._segment(
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
                local_seconds, 7.0, 9.0, extended_closed, extended_withdraw
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        return extended_withdraw.copy(), np.zeros(6)
        # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。

    # 临时把原场景的模型路径、步长和轨迹函数替换为在轨版本。
    # 权威 MJScene 动力学、机械臂 PID、姿态 FSW、反作用轮和交会控制器统一
    # 运行在 500 Hz；渲染只是对状态进行约 30 Hz 的只读降采样。
    native.MODEL_PATH = augmented_path
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `MODEL_PATH` 供后续步骤使用。
    native.TIME_STEP = MISSION_TIME_STEP_S
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `TIME_STEP` 供后续步骤使用。
    native.JointTrajectoryPublisher.reference = classmethod(
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `reference` 供后续步骤使用。
        lambda cls, seconds: mission_joint_reference(seconds)
        # 逐行说明：定义一个简短匿名函数，用于把当前对象或参数延迟传给回调。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。

    from Basilisk.architecture import messaging, sysModel
    # 逐行说明：从 `Basilisk.architecture` 导入 `messaging, sysModel`，供后续场景构建直接使用。
    from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback, rwMotorTorque
    # 逐行说明：从 `Basilisk.fswAlgorithms` 导入 `attTrackingError, inertial3D, mrpFeedback, rwMotorTorque`，供后续场景构建直接使用。
    from Basilisk.simulation import (
    # 逐行说明：从 `Basilisk.simulation` 导入 `(`，供后续场景构建直接使用。
        arrayMotorTorqueToSingleActuators,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        saturationSingleActuator,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        scalarJointStatesToRWSpeed,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        simpleNav,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    from Basilisk.utilities import macros, orbitalMotion, simIncludeGravBody
    # 逐行说明：从 `Basilisk.utilities` 导入 `macros, orbitalMotion, simIncludeGravBody`，供后续场景构建直接使用。
    from bsk_render_adapter import BasiliskRenderBridge, RecordingOnlyPublisher, SceneSettings, VisualElement
    # 逐行说明：导入通用渲染桥、离线录制发布器和 UE 场景描述类型。

    class RendezvousThrustController(sysModel.SysModel):
    # 逐行说明：定义 `RendezvousThrustController` 类，把该控制功能封装成可加入 Basilisk 任务的模块。
        """沿对接轴控制相对距离的一维闭环推进器控制器。

        输入为卫星和目标的惯性系位置/速度，输出为两个非负单向推力命令。
        控制器先产生一个有正负号的合成力：正值交给接近推进器，负值的绝对值
        交给制动推进器，因此两个理想推进器不会同时输出正推力。
        """

        def __init__(self) -> None:
        # 逐行说明：开始定义 `__init__` 函数，具体职责由紧随其后的中文文档字符串说明。
            super().__init__()
            # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
            self.ModelTag = "orbitalClosedLoopRendezvous"
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
            self.busStateInMsg = messaging.SCStatesMsgReader()
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `busStateInMsg` 供后续步骤使用。
            self.targetStateInMsg = messaging.SCStatesMsgReader()
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `targetStateInMsg` 供后续步骤使用。
            self.approachOutMsg = messaging.SingleActuatorMsg()
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `approachOutMsg` 供后续步骤使用。
            self.brakingOutMsg = messaging.SingleActuatorMsg()
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `brakingOutMsg` 供后续步骤使用。

        def Reset(self, CurrentSimNanos: int) -> None:
        # 逐行说明：开始定义 `Reset` 函数，具体职责由紧随其后的中文文档字符串说明。
            self.UpdateState(CurrentSimNanos)
            # 逐行说明：调用当前 Basilisk 模块实例的方法，更新本步消息或内部状态。

        def UpdateState(self, CurrentSimNanos: int) -> None:
        # 逐行说明：开始定义 `UpdateState` 函数，具体职责由紧随其后的中文文档字符串说明。
            seconds = CurrentSimNanos * macros.NANO2SEC
            # 逐行说明：把 Basilisk 纳秒时间转换为秒。
            # 到 26.5 s 后显式归零，避免夹爪接触阶段的平移控制与接触约束对抗。
            signed_force = 0.0
            # 逐行说明：保存可正可负的轴向合成推力；符号决定使用哪一侧推进器。
            if seconds < RENDEZVOUS_CONTROL_STOP_S:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                bus_state = self.busStateInMsg()
                # 逐行说明：读取服务卫星当前权威惯性状态消息。
                target_state = self.targetStateInMsg()
                # 逐行说明：读取目标当前权威惯性状态消息。
                travelled, reference_speed, reference_acceleration = (
                # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
                    _quintic_rendezvous_reference(seconds)
                    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
                )
                # 逐行说明：结束当前多行容器、函数调用或表达式。
                # native.TARGET_POS 是最终抓取构型下目标相对卫星的位置。
                # 初始再加 0.75 m；travelled 从 0 增至 0.75 m，因此期望间距
                # 在 2～20 s 逐渐收敛到最终抓取位置。
                desired_relative_axis = (
                # 逐行说明：计算目标相对卫星在对接轴上的期望距离。
                    float(np.dot(native.TARGET_POS, DOCKING_AXIS_BODY))
                    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
                    + RENDEZVOUS_INITIAL_OFFSET_M
                    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                    - travelled
                    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                )
                # 逐行说明：结束当前多行容器、函数调用或表达式。
                # 相对状态统一定义为“目标减卫星”，然后投影到机体系对接轴。
                # 当前姿态 FSW 把机体系稳定在惯性参考附近，所以该轴可以直接
                # 用于这个短时近距离演示；更通用任务应逐帧把机体系轴旋转到惯性系。
                relative_position = np.asarray(target_state.r_BN_N) - np.asarray(bus_state.r_BN_N)
                # 逐行说明：计算目标位置减卫星位置的惯性系相对位置。
                relative_velocity = np.asarray(target_state.v_BN_N) - np.asarray(bus_state.v_BN_N)
                # 逐行说明：计算目标速度减卫星速度的惯性系相对速度。
                relative_axis = float(np.dot(relative_position, DOCKING_AXIS_BODY))
                # 逐行说明：把实际相对位置投影到对接轴。
                relative_speed_axis = float(np.dot(relative_velocity, DOCKING_AXIS_BODY))
                # 逐行说明：把实际相对速度投影到对接轴。
                # travelled 增大意味着目标-卫星间距减小，所以相对距离的速度和
                # 加速度参考要取负号。
                reference_speed_axis = -reference_speed
                # 逐行说明：把累计接近速度改写成相对距离速度参考。
                reference_acceleration_axis = -reference_acceleration
                # 逐行说明：把累计接近加速度改写成相对距离加速度参考。
                position_error = desired_relative_axis - relative_axis
                # 逐行说明：计算期望轴向距离减实际轴向距离。
                velocity_error = reference_speed_axis - relative_speed_axis
                # 逐行说明：计算期望轴向相对速度减实际相对速度。
                # 正推力让卫星沿相机视轴前进，会使“目标减卫星”的相对距离减小，
                # 因此从相对加速度映射到卫星推力时需要整体负号。
                # 第一项是加速度前馈，后两项分别修正位置和速度误差。
                signed_force = (
                # 逐行说明：保存可正可负的轴向合成推力；符号决定使用哪一侧推进器。
                    -RENDEZVOUS_ESTIMATED_MASS_KG * reference_acceleration_axis
                    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                    -RENDEZVOUS_POSITION_GAIN_N_PER_M * position_error
                    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                    -RENDEZVOUS_VELOCITY_GAIN_N_PER_MPS * velocity_error
                    # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                )
                # 逐行说明：结束当前多行容器、函数调用或表达式。
                # 限制到 ±0.60 N，保证近距离机动温和且可由当前模型承受。
                signed_force = float(np.clip(
                # 逐行说明：保存可正可负的轴向合成推力；符号决定使用哪一侧推进器。
                    signed_force,
                    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                    -RENDEZVOUS_MAX_THRUST_N,
                    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                    RENDEZVOUS_MAX_THRUST_N,
                    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                ))
                # 逐行说明：结束当前多行容器、函数调用或表达式。
            # 把双向合成命令拆成两个只能输出非负推力的对置执行器。
            approach = max(0.0, signed_force)
            # 逐行说明：提取合成推力的正半轴作为接近推进器命令。
            braking = max(0.0, -signed_force)
            # 逐行说明：提取合成推力的负半轴绝对值作为制动推进器命令。
            self.approachOutMsg.write(
            # 逐行说明：写出当前 Basilisk 消息或把生成结果保存到磁盘。
                messaging.SingleActuatorMsgPayload(input=approach), CurrentSimNanos, self.moduleID
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
            self.brakingOutMsg.write(
            # 逐行说明：写出当前 Basilisk 消息或把生成结果保存到磁盘。
                messaging.SingleActuatorMsgPayload(input=braking), CurrentSimNanos, self.moduleID
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。

    # 调用原场景工厂创建机械臂 PID、扭矩限幅器、MJScene 和状态记录器。
    # try/finally 确保临时修改的模块级配置始终恢复，不污染同一进程的后续任务。
    try:
    # 逐行说明：进入需要保证资源恢复或释放的受保护代码段。
        simulation, scene, native_dynamics, native_recorders = native._build_simulation()
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        # 在轨迹发布器实例上固定偏移后的时间表，然后再恢复外部类定义。
        native_dynamics[0].reference = mission_joint_reference
        # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
    finally:
    # 逐行说明：无论前面是否发生异常，都执行随后缩进的恢复或清理操作。
        native.MODEL_PATH = original_model_path
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `MODEL_PATH` 供后续步骤使用。
        native.TIME_STEP = original_time_step
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `TIME_STEP` 供后续步骤使用。
        native.JointTrajectoryPublisher.reference = original_reference_descriptor
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `reference` 供后续步骤使用。
    # 开启额外运动方程调用，使 Basilisk 重力模型贡献能够进入 MJScene 方程。
    scene.extraEoMCall = True
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `extraEoMCall` 供后续步骤使用。
    bus = scene.getBody("cubesat_bus")
    # 逐行说明：取得并保存 CubeSat 服务卫星刚体句柄。
    target = scene.getBody("capture_target")
    # 逐行说明：取得并保存待抓取目标刚体句柄。

    # 建立地球中心引力。Earth 是整个 MJScene 的中心天体；卫星、机械臂和目标
    # 都处于同一自由落体环境，而不是在 UE 中伪造轨道动画。
    gravity_factory = simIncludeGravBody.gravBodyFactory()
    # 逐行说明：创建 Basilisk 引力天体工厂。
    earth = gravity_factory.createEarth()
    # 逐行说明：创建并保存地球引力天体。
    earth.isCentralBody = True
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `isCentralBody` 供后续步骤使用。
    sun = gravity_factory.createSun()
    # 逐行说明：创建太阳天体；它的状态随后由同一个 Basilisk SPICE 模块提供。
    ephemeris = gravity_factory.createSpiceInterface(time=SPICE_EPOCH, epochInMsg=True)
    # 逐行说明：加载 Basilisk 官方 SPICE 星历核并建立 Earth、Sun 输出消息。
    ephemeris.zeroBase = "Earth"
    # 逐行说明：把星历零基准设为地心，使现有 500 km 地球轨道初值保持原有定义。
    scene.AddModelToDynamicsTask(ephemeris, 75)
    # 逐行说明：在 MJScene 每次动力学求值时先更新星历，再计算多天体引力。
    gravity_model = gravity_factory.addBodiesTo(scene)
    # 逐行说明：把地球引力模型连接到整个 MJScene。

    # 取得三个轮子的权威 MuJoCo 铰链和电机句柄。之后所有轮速都直接来自这些
    # joint state，不在 UE 内按渲染帧积分。
    wheel_joints = []
    # 逐行说明：建立三个权威反作用轮铰链句柄列表。
    wheel_actuators = []
    # 逐行说明：建立三个权威反作用轮电机句柄列表。
    for definition in RW_DEFINITIONS:
    # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
        wheel_body = scene.getBody(definition["body_name"])
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `wheel_body` 供后续步骤使用。
        wheel_joints.append(wheel_body.getScalarJoint(definition["joint_name"]))
        # 逐行说明：把当前构造结果追加到对应列表，供后续统一连接。
        wheel_actuators.append(scene.getSingleActuator(definition["actuator_name"]))
        # 逐行说明：把当前构造结果追加到对应列表，供后续统一连接。

    # 姿态 FSW 消息链：
    # bus COM state -> SimpleNav -> 姿态跟踪误差 -> MRP反馈控制力矩
    # -> 三轮力矩分配 -> 单轮限幅 -> MJScene 轮电机。
    navigation = simpleNav.SimpleNav()
    # 逐行说明：创建读取卫星质心状态的简化导航模块。
    navigation.ModelTag = "orbitalGraspSimpleNav"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    navigation.scStateInMsg.subscribeTo(bus.getCenterOfMass().stateOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    simulation.AddModelToTask("graspTask", navigation, 900)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # 惯性参考 MRP=[0,0,0]，即保持初始惯性指向，使对接轴和相机视线稳定。
    attitude_reference = inertial3D.inertial3D()
    # 逐行说明：创建期望惯性姿态参考模块。
    attitude_reference.ModelTag = "orbitalGraspInertialReference"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    attitude_reference.sigma_R0N = [0.0, 0.0, 0.0]
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `sigma_R0N` 供后续步骤使用。
    simulation.AddModelToTask("graspTask", attitude_reference, 800)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    attitude_error = attTrackingError.attTrackingError()
    # 逐行说明：创建实际姿态相对期望姿态的误差计算模块。
    attitude_error.ModelTag = "orbitalGraspAttitudeError"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    attitude_error.attNavInMsg.subscribeTo(navigation.attOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    attitude_error.attRefInMsg.subscribeTo(attitude_reference.attRefOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    simulation.AddModelToTask("graspTask", attitude_error, 700)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # 向标准 Basilisk FSW 声明三个正交轮的安装矩阵、主轴惯量和力矩上限。
    # GsMatrix_B 的三列分别是 X/Y/Z 轮轴，所以当前映射是单位阵。
    wheel_configuration = messaging.RWArrayConfigMsgPayload()
    # 逐行说明：创建标准 Basilisk 三轮阵列配置消息载荷。
    wheel_configuration.numRW = 3
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `numRW` 供后续步骤使用。
    wheel_configuration.GsMatrix_B = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `GsMatrix_B` 供后续步骤使用。
    wheel_configuration.JsList = [RW_SPIN_INERTIA] * 3
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `JsList` 供后续步骤使用。
    wheel_configuration.uMax = [RW_TORQUE_LIMIT_NM] * 3
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `uMax` 供后续步骤使用。
    wheel_configuration_message = messaging.RWArrayConfigMsg().write(wheel_configuration)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `wheel_configuration_message` 供后续步骤使用。

    # 把三个 MJScene 标量铰链角速度转换为 Basilisk 标准 RW 轮速消息，
    # 供含轮速耦合项的姿态控制器使用。
    wheel_speeds = scalarJointStatesToRWSpeed.ScalarJointStatesToRWSpeed()
    # 逐行说明：创建 MJ 标量关节速度到标准反作用轮速度消息的转换器。
    wheel_speeds.ModelTag = "mjsceneWheelSpeeds"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    wheel_speeds.setNumJoints(3)
    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
    for index, joint in enumerate(wheel_joints):
    # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
        wheel_speeds.jointStateInMsgs[index].subscribeTo(joint.stateDotOutMsg)
        # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    simulation.AddModelToTask("graspTask", wheel_speeds, 600)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # 标准 MRP 姿态反馈。Ki=-1 且 integralLimit=0 表示本 Demo 不启用积分项，
    # 主要由 K（姿态误差）和 P（角速度误差）完成稳定。
    attitude_controller = mrpFeedback.mrpFeedback()
    # 逐行说明：创建标准 MRP 姿态反馈控制器。
    attitude_controller.ModelTag = "orbitalGraspMrpFeedback"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    attitude_controller.K = 0.08
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `K` 供后续步骤使用。
    attitude_controller.Ki = -1.0
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `Ki` 供后续步骤使用。
    attitude_controller.P = 0.35
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `P` 供后续步骤使用。
    attitude_controller.integralLimit = 0.0
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `integralLimit` 供后续步骤使用。
    attitude_controller.guidInMsg.subscribeTo(attitude_error.attGuidOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    attitude_controller.rwParamsInMsg.subscribeTo(wheel_configuration_message)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    attitude_controller.rwSpeedsInMsg.subscribeTo(wheel_speeds.rwSpeedOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    # 控制器使用的卫星等效惯量矩阵，按行展开为 3×3。
    vehicle_configuration = messaging.VehicleConfigMsgPayload(
    # 逐行说明：创建姿态控制器使用的航天器等效惯量配置。
        ISCPntB_B=[0.36, 0.0, 0.0, 0.0, 0.36, 0.0, 0.0, 0.0, 0.23]
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `ISCPntB_B` 供后续步骤使用。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    vehicle_configuration_message = messaging.VehicleConfigMsg().write(vehicle_configuration)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `vehicle_configuration_message` 供后续步骤使用。
    attitude_controller.vehConfigInMsg.subscribeTo(vehicle_configuration_message)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    simulation.AddModelToTask("graspTask", attitude_controller, 500)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # 将三轴本体控制力矩映射为三个反作用轮的电机力矩命令。
    torque_mapping = rwMotorTorque.rwMotorTorque()
    # 逐行说明：创建本体三轴控制力矩到各反作用轮力矩的分配器。
    torque_mapping.ModelTag = "orbitalGraspWheelTorqueMapping"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    torque_mapping.rwParamsInMsg.subscribeTo(wheel_configuration_message)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    torque_mapping.vehControlInMsg.subscribeTo(attitude_controller.cmdTorqueOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    torque_mapping.controlAxes_B = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `controlAxes_B` 供后续步骤使用。
    simulation.AddModelToTask("graspTask", torque_mapping, 400)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # rwMotorTorque 输出一个轮阵列消息；MJScene 每个电机需要独立消息，故拆分。
    torque_splitter = arrayMotorTorqueToSingleActuators.ArrayMotorTorqueToSingleActuators()
    # 逐行说明：把轮阵列力矩消息拆分成三个独立执行器消息。
    torque_splitter.ModelTag = "orbitalGraspWheelTorqueSplitter"
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
    torque_splitter.setNumActuators(3)
    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
    torque_splitter.torqueInMsg.subscribeTo(torque_mapping.rwMotorTorqueOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    simulation.AddModelToTask("graspTask", torque_splitter, 300)
    # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # 每个轮子最终都经过 ±0.003 N·m 的硬限幅，再连接到 MJScene 执行器。
    wheel_limiters = []
    # 逐行说明：保存三个反作用轮的硬力矩限幅器。
    for index, actuator in enumerate(wheel_actuators):
    # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
        limiter = saturationSingleActuator.SaturationSingleActuator()
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `limiter` 供后续步骤使用。
        limiter.ModelTag = f"orbitalGraspWheel{index + 1}Limiter"
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `ModelTag` 供后续步骤使用。
        limiter.setMinInput(-RW_TORQUE_LIMIT_NM)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        limiter.setMaxInput(RW_TORQUE_LIMIT_NM)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        limiter.actuatorInMsg.subscribeTo(torque_splitter.actuatorOutMsgs[index])
        # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
        simulation.AddModelToTask("graspTask", limiter, 200 - index)
        # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。
        actuator.actuatorInMsg.subscribeTo(limiter.actuatorOutMsg)
        # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
        wheel_limiters.append(limiter)
        # 逐行说明：把当前构造结果追加到对应列表，供后续统一连接。

    # 平移控制器读取卫星和目标原点的权威状态，并驱动 XML 中的一对推进器。
    maneuver_command = RendezvousThrustController()
    # 逐行说明：创建并保存相对接近推进器控制器。
    maneuver_command.busStateInMsg.subscribeTo(bus.getOrigin().stateOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    maneuver_command.targetStateInMsg.subscribeTo(target.getOrigin().stateOutMsg)
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
    scene.AddModelToDynamicsTask(maneuver_command, 6500)
    # 逐行说明：把控制/发布模块加入 MJScene 动力学任务。
    scene.getSingleActuator("docking_approach_thruster").actuatorInMsg.subscribeTo(
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
        maneuver_command.approachOutMsg
        # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    scene.getSingleActuator("docking_braking_thruster").actuatorInMsg.subscribeTo(
    # 逐行说明：把本模块输入消息订阅到上游输出，建立 Basilisk 数据依赖。
        maneuver_command.brakingOutMsg
        # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。

    # 验收记录器只需 30 Hz。它们不会改变 500 Hz 动力学，只是按较低频率读取
    # 卫星/目标状态、姿态误差、轮速与推进器命令，降低内存占用。
    sample_time = macros.sec2nano(1.0 / 30.0)
    # 逐行说明：把 30 Hz 采样周期转换为 Basilisk 纳秒。
    bus_recorder = bus.getOrigin().stateOutMsg.recorder(sample_time)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `bus_recorder` 供后续步骤使用。
    target_recorder = target.getOrigin().stateOutMsg.recorder(sample_time)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `target_recorder` 供后续步骤使用。
    attitude_recorder = attitude_error.attGuidOutMsg.recorder(sample_time)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `attitude_recorder` 供后续步骤使用。
    wheel_recorders = [joint.stateDotOutMsg.recorder(sample_time) for joint in wheel_joints]
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `wheel_recorders` 供后续步骤使用。
    approach_recorder = maneuver_command.approachOutMsg.recorder(sample_time)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `approach_recorder` 供后续步骤使用。
    braking_recorder = maneuver_command.brakingOutMsg.recorder(sample_time)
    # 逐行说明：计算本行右侧表达式，并把结果保存到 `braking_recorder` 供后续步骤使用。
    for recorder in [
    # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
        bus_recorder,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        target_recorder,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        attitude_recorder,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        approach_recorder,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        braking_recorder,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        *wheel_recorders,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
    ]:
    # 逐行说明：开始该复合语句的缩进代码块。
        simulation.AddModelToTask("graspTask", recorder, -100)
        # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

    # -----------------------------------------------------------------------
    # UE 渲染桥：只读取权威状态，不向动力学回写位姿
    # -----------------------------------------------------------------------
    # 以 cubesat_bus 为浮动原点，既保留惯性绝对位置，又避免 UE 在 500 km
    # 轨道坐标上直接渲染局部厘米级抓取动作造成精度损失。
    bridge = BasiliskRenderBridge(
    # 逐行说明：创建非阻塞 BSK 到 UE 通用渲染桥。
        host=host,
        # 逐行说明：接收 UE TCP 监听地址。
        port=port,
        # 逐行说明：接收 UE 状态协议监听端口。
        origin_object="orbital_grasp/cubesat_bus",
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `origin_object` 供后续步骤使用。
        frame_period_ns=sample_time,
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `frame_period_ns` 供后续步骤使用。
        publisher=RecordingOnlyPublisher() if recording_only else None,
        # 逐行说明：离线录制时禁用 TCP；实时模式继续使用默认非阻塞网络发布器。
        recording_path=recording_path,
        # 逐行说明：把所有通用协议消息同时写入指定 `.bskrec` 文件。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    try:
    # 逐行说明：进入需要保证资源恢复或释放的受保护代码段。
        body_ids = bridge.add_mj_scene(
        # 逐行说明：保存 MJScene 刚体名称到稳定渲染对象 ID 的映射。
            scene,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            namespace="orbital_grasp",
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `namespace` 供后续步骤使用。
            # 解析与权威 MJScene 完全相同的派生 MJCF，让相机、父子刚体、几何
            # 和材质都从同一个源自动发现。轮子辅助 geom 的 alpha 为 0，UE 只
            # 显示下面注册的遥测反作用轮，避免重复模型。
            source_path=augmented_path,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `source_path` 供后续步骤使用。
            mesh_asset_catalog=catalog,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `mesh_asset_catalog` 供后续步骤使用。
            semantic_label="orbital_service_vehicle",
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `semantic_label` 供后续步骤使用。
            camera_picture_in_picture=True,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `camera_picture_in_picture` 供后续步骤使用。
            camera_capture_rate_hz=15.0,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `camera_capture_rate_hz` 供后续步骤使用。
            camera_pip_resolution=(480, 270),
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `camera_pip_resolution` 供后续步骤使用。
            camera_picture_in_picture_start_slot=1,
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `camera_picture_in_picture_start_slot` 供后续步骤使用。
            camera_display_names={
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `camera_display_names` 供后续步骤使用。
                "cubesat_docking_camera": "CubeSat Docking Camera",
                # 逐行说明：设置结构中的 `cubesat_docking_camera` 字段，供下游模块按稳定键名读取。
                "so101_wrist_cam": "SO-101 Wrist Camera",
                # 逐行说明：设置结构中的 `so101_wrist_cam` 字段，供下游模块按稳定键名读取。
            },
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # Earth 和 Sun 的位置、速度、固连姿态都来自同一个 Basilisk SPICE
        # 星历。显式 light role 取代通过名字猜测太阳物理语义的旧方式。
        bridge.add_celestial_bodies(
        # 逐行说明：向通用渲染桥注册地球和太阳的动态星历消息。
            [earth, sun],
            # 逐行说明：保持天体顺序与 Basilisk 重力工厂/SPICE 输出顺序一致。
            visual_overrides={
            # 逐行说明：为太阳声明渲染角色和主方向光参数，而不是让 UE 按名称硬编码。
                "sun": {
                # 逐行说明：开始构造太阳的通用天体可视覆盖配置。
                    "visual_role": "star",
                    # 逐行说明：把该天体声明为恒星类型。
                    "luminous": True,
                    # 逐行说明：让太阳模型本身使用发光材质语义。
                    "drives_directional_light": True,
                    # 逐行说明：指定该天体的星历位置驱动 UE 主平行光方向。
                    "light_color_rgb": (1.0, 0.98, 0.92),
                    # 逐行说明：设置接近太阳光谱观感的暖白色线性 RGB。
                    "light_illuminance_lux_at_reference_distance": 8.0,
                    # 逐行说明：保留当前曝光下的参考照度，距离变化按平方反比缩放。
                    "light_reference_distance_m": SUN_REFERENCE_DISTANCE_M,
                    # 逐行说明：设置参考照度对应的一天文单位距离。
                },
                # 逐行说明：结束太阳可视覆盖配置。
            },
            # 逐行说明：结束通用天体可视覆盖映射。
        )
        # 逐行说明：完成 Earth/Sun 星历到 UE 天体协议的注册。
        # 为三个权威 MJ 铰链增加渲染语义：尺寸、标签、力矩命令和轮速上限。
        # UE 动画读取真实 joint angle/velocity，而不是按画面帧率自行旋转。
        wheel_visual_definitions = []
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `wheel_visual_definitions` 供后续步骤使用。
        for index, definition in enumerate(RW_DEFINITIONS):
        # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
            wheel_visual_definitions.append(
            # 逐行说明：把当前构造结果追加到对应列表，供后续统一连接。
                {
                # 逐行说明：开始构造当前列表、元组或字典表达式。
                    **definition,
                    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                    "command_message": wheel_limiters[index].actuatorOutMsg,
                    # 逐行说明：设置结构中的 `command_message` 字段，供下游模块按稳定键名读取。
                    "omega_max_rad_s": RW_SPEED_LIMIT_RAD_S,
                    # 逐行说明：设置结构中的 `omega_max_rad_s` 字段，供下游模块按稳定键名读取。
                    "torque_max_Nm": RW_TORQUE_LIMIT_NM,
                    # 逐行说明：设置结构中的 `torque_max_Nm` 字段，供下游模块按稳定键名读取。
                    "diameter_m": 0.05,
                    # 逐行说明：设置结构中的 `diameter_m` 字段，供下游模块按稳定键名读取。
                    "thickness_m": 0.024,
                    # 逐行说明：设置结构中的 `thickness_m` 字段，供下游模块按稳定键名读取。
                }
                # 逐行说明：结束当前多行容器、函数调用或表达式。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        bridge.add_mj_reaction_wheels(
        # 逐行说明：把权威 MJ 铰链状态注册为通用反作用轮遥测与动画。
            scene,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            wheel_visual_definitions,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            parent_id=body_ids["cubesat_bus"],
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `parent_id` 供后续步骤使用。
            prefix="orbital_grasp/reaction_wheel",
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `prefix` 供后续步骤使用。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。

        def thrust_visual_state(message):
        # 逐行说明：开始定义 `thrust_visual_state` 函数，具体职责由紧随其后的中文文档字符串说明。
            """把 Basilisk 单执行器消息转换为通用推进器可视状态。"""
            force = float(message.read().input)
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `force` 供后续步骤使用。
            return {
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
                "visible": force > 1.0e-9,
                # 逐行说明：根据实际推力判断 UE 是否显示羽流。
                "value": force / RENDEZVOUS_MAX_THRUST_N,
                # 逐行说明：把当前推力归一化为通用可视元素强度。
                "channels": {
                # 逐行说明：开始定义带单位的推进器动态遥测通道。
                    "throttle": force / RENDEZVOUS_MAX_THRUST_N,
                    # 逐行说明：输出 0～1 的归一化节流率。
                    "thrust_N": force,
                    # 逐行说明：输出当前实际命令推力，单位为 N。
                    "enabled": force > 1.0e-9,
                    # 逐行说明：输出当前推进器是否正在工作的布尔状态。
                },
                # 逐行说明：结束当前多行容器、函数调用或表达式。
            }
            # 逐行说明：结束当前多行容器、函数调用或表达式。

        # 羽流位置与真实执行器作用方向相反：推进器沿 +d 产生推力时，尾焰朝 -d。
        # 这些 VisualElement 只控制显示，真实力仍由 MJScene general actuator 施加。
        for visual_id, label, position, normal, command_message in (
        # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
            (
            # 逐行说明：开始构造当前列表、元组或字典表达式。
                "orbital_grasp/thruster/approach",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                "Closed-loop approach engine",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                tuple(-0.15 * DOCKING_AXIS_BODY),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                tuple(-DOCKING_AXIS_BODY),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                maneuver_command.approachOutMsg,
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            ),
            # 逐行说明：结束当前多行容器、函数调用或表达式。
            (
            # 逐行说明：开始构造当前列表、元组或字典表达式。
                "orbital_grasp/thruster/braking",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                "Closed-loop braking engine",
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                tuple(0.15 * DOCKING_AXIS_BODY),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                tuple(DOCKING_AXIS_BODY),
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                maneuver_command.brakingOutMsg,
                # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            ),
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        ):
        # 逐行说明：开始该复合语句的缩进代码块。
            bridge.add_thrusters(VisualElement(
            # 逐行说明：向 manifest 注册一个推进器羽流和动态遥测元素。
                visual_id=visual_id,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `visual_id` 供后续步骤使用。
                kind="thruster",
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `kind` 供后续步骤使用。
                parent_id=body_ids["cubesat_bus"],
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `parent_id` 供后续步骤使用。
                position_body_m=position,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `position_body_m` 供后续步骤使用。
                normal_body=normal,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `normal_body` 供后续步骤使用。
                field_of_view_rad=(math.radians(22.0),),
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `field_of_view_rad` 供后续步骤使用。
                size_m=0.03,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `size_m` 供后续步骤使用。
                range_m=0.34,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `range_m` 供后续步骤使用。
                color_rgba=(0.25, 0.62, 1.0, 0.72),
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `color_rgba` 供后续步骤使用。
                label=label,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `label` 供后续步骤使用。
                channel_schema={
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `channel_schema` 供后续步骤使用。
                    "throttle": {"type": "number", "unit": "1", "minimum": 0.0, "maximum": 1.0},
                    # 逐行说明：输出 0～1 的归一化节流率。
                    "thrust_N": {"type": "number", "unit": "N"},
                    # 逐行说明：输出当前实际命令推力，单位为 N。
                    "enabled": {"type": "boolean"},
                    # 逐行说明：输出当前推进器是否正在工作的布尔状态。
                },
                # 逐行说明：结束当前多行容器、函数调用或表达式。
                state_provider=lambda message=command_message: thrust_visual_state(message),
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `state_provider` 供后续步骤使用。
            ))
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 局部抓取视图默认聚焦夹爪。67 ms 相当于约两帧缓存，用于位置线性插值
        # 和姿态 SLERP；这只改善显示平滑度，不影响权威仿真状态。
        bridge.set_scene_settings(
        # 逐行说明：配置浮动原点、默认相机目标、插值和轨迹显示策略。
            SceneSettings(
            # 逐行说明：开始一个跨多行书写的调用或数据结构，后续各行继续提供内容。
                origin_object_id=body_ids["cubesat_bus"],
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `origin_object_id` 供后续步骤使用。
                default_camera_target=body_ids["so101_gripper"],
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `default_camera_target` 供后续步骤使用。
                default_camera_distance_m=1.35,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `default_camera_distance_m` 供后续步骤使用。
                # 近距离相机正好位于密切轨道附近，完整轨道线会穿过前景，因此
                # 局部任务视图默认关闭轨道线；行星尺度相机仍可单独打开。
                orbit_lines=False,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `orbit_lines` 供后续步骤使用。
                trajectory_history=True,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `trajectory_history` 供后续步骤使用。
                interpolation_delay_ms=67.0,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `interpolation_delay_ms` 供后续步骤使用。
                max_extrapolation_ms=67.0,
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `max_extrapolation_ms` 供后续步骤使用。
                fill_light_intensity_lux=0.0,
                # 逐行说明：关闭非物理反向补光，使背阳面由星历太阳方向真实决定。
            )
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # -------------------------------------------------------------------
        # Vizard 风格任务事件与 UE -> BSK 白名单命令
        # -------------------------------------------------------------------
        mission_control = {"paused": False}
        # 逐行说明：保存可由 UE 白名单命令改变的任务暂停状态。

        def mission_phase(seconds: float) -> str:
        # 逐行说明：开始定义 `mission_phase` 函数，具体职责由紧随其后的中文文档字符串说明。
            """把连续仿真时间归类为任务 UI 使用的离散阶段。"""
            if seconds < RENDEZVOUS_START_S:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                return "initial_orbit"
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
            if seconds < RENDEZVOUS_STOP_S:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                return "rendezvous"
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
            if seconds < STATION_KEEP_STOP_S:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                return "station_keep"
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
            if seconds < GRASP_TIME_OFFSET_S + 4.5:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                return "arm_approach"
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
            if seconds < GRASP_TIME_OFFSET_S + 7.0:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                return "capture"
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
            if seconds < MINIMUM_MISSION_DURATION_S:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                return "retraction"
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
            return "complete"
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。

        def set_paused(value: bool):
        # 逐行说明：开始定义 `set_paused` 函数，具体职责由紧随其后的中文文档字符串说明。
            """生成暂停/继续命令处理器；处理器只在仿真线程中修改状态。"""
            def handler(payload, sim_time_ns):
            # 逐行说明：开始定义 `handler` 函数，具体职责由紧随其后的中文文档字符串说明。
                mission_control["paused"] = value
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                return {
                # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
                    "paused": value,
                    # 逐行说明：设置结构中的 `paused` 字段，供下游模块按稳定键名读取。
                    "phase": mission_phase(sim_time_ns * macros.NANO2SEC),
                    # 逐行说明：设置结构中的 `phase` 字段，供下游模块按稳定键名读取。
                }
                # 逐行说明：结束当前多行容器、函数调用或表达式。
            return handler
            # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。

        # 只有显式注册的命令才能执行；未知命令会返回明确错误，不会静默忽略。
        bridge.register_command_handler(
        # 逐行说明：注册一个只允许在仿真线程执行的 UE 白名单命令。
            "mission.pause",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            set_paused(True),
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            label="Pause simulation",
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `label` 供后续步骤使用。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        bridge.register_command_handler(
        # 逐行说明：注册一个只允许在仿真线程执行的 UE 白名单命令。
            "mission.resume",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            set_paused(False),
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            label="Resume simulation",
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `label` 供后续步骤使用。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        bridge.register_command_handler(
        # 逐行说明：注册一个只允许在仿真线程执行的 UE 白名单命令。
            "mission.status",
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            lambda payload, sim_time_ns: {
            # 逐行说明：定义一个简短匿名函数，用于把当前对象或参数延迟传给回调。
                "paused": mission_control["paused"],
                # 逐行说明：设置结构中的 `paused` 字段，供下游模块按稳定键名读取。
                "phase": mission_phase(sim_time_ns * macros.NANO2SEC),
                # 逐行说明：设置结构中的 `phase` 字段，供下游模块按稳定键名读取。
            },
            # 逐行说明：结束当前多行容器、函数调用或表达式。
            label="Query mission status",
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `label` 供后续步骤使用。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 较低模型优先级让 bridge 在各动力学/控制模块更新后读取同一步最新状态。
        simulation.AddModelToTask("graspTask", bridge, -10_000)
        # 逐行说明：把模块加入 graspTask，并用最后一个参数设置执行优先级。

        # -------------------------------------------------------------------
        # 初始化 500 km 圆轨道、相对状态、机械臂和轮速
        # -------------------------------------------------------------------
        simulation.InitializeSimulation()
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        orbit = orbitalMotion.ClassicElements()
        # 逐行说明：创建并保存经典轨道根数结构。
        # 六个经典轨道根数：a 为地球半径+500 km，e=0 表示圆轨道，i=0
        # 表示赤道轨道；真近点角 pi 只选择轨道上的初始位置，不改变轨道形状。
        orbit.a = earth.radEquator + ORBIT_ALTITUDE_M
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `a` 供后续步骤使用。
        orbit.e = 0.0
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `e` 供后续步骤使用。
        orbit.i = 0.0
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `i` 供后续步骤使用。
        orbit.Omega = 0.0
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `Omega` 供后续步骤使用。
        orbit.omega = 0.0
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `omega` 供后续步骤使用。
        orbit.f = math.pi
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `f` 供后续步骤使用。
        initial_position, initial_velocity = orbitalMotion.elem2rv(earth.mu, orbit)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        # orbitalMotion.elem2rv 把轨道根数转换为 J2000 惯性系位置和速度。
        bus.setPosition(initial_position)
        # 逐行说明：设置对应刚体或关节的权威初始位置。
        bus.setVelocity(initial_velocity)
        # 逐行说明：设置对应刚体或关节的权威初始速度。
        bus.setAttitude([0.0, 0.0, 0.0])
        # 逐行说明：设置刚体的初始 MRP 姿态。
        bus.setAttitudeRate([0.0, 0.0, 0.0])
        # 逐行说明：设置刚体的初始机体系角速度。
        # 目标与服务卫星初始速度相同，因此二者先近似共轨；目标位置是在最终
        # 抓取相对位姿 native.TARGET_POS 上再沿对接轴增加 0.75 m。随后服务
        # 卫星在相对状态反馈下接近、制动并定点保持，目标没有被脚本强制移动。
        rendezvous_lead = DOCKING_AXIS_BODY * RENDEZVOUS_INITIAL_OFFSET_M
        # 逐行说明：计算目标沿对接轴额外领先卫星的 0.75 m 向量。
        target.setPosition(initial_position + native.TARGET_POS + rendezvous_lead)
        # 逐行说明：设置对应刚体或关节的权威初始位置。
        target.setVelocity(initial_velocity)
        # 逐行说明：设置对应刚体或关节的权威初始速度。
        target.setAttitude(native._quaternion_to_mrp(native.TARGET_QUAT))
        # 逐行说明：设置刚体的初始 MRP 姿态。
        target.setAttitudeRate([0.0, 0.0, 0.0])
        # 逐行说明：设置刚体的初始机体系角速度。
        # 关节初值必须与轨迹发布器的 PREGRASP 参考一致且速度为零，否则 PID
        # 会在第一个仿真步产生较大瞬态力矩，看起来像机械臂突然甩动。
        for (body_name, joint_name), position in zip(native.JOINTS, native.PREGRASP, strict=True):
        # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
            joint = scene.getBody(body_name).getScalarJoint(joint_name)
            # 逐行说明：计算本行右侧表达式，并把结果保存到 `joint` 供后续步骤使用。
            joint.setPosition(float(position))
            # 逐行说明：设置对应刚体或关节的权威初始位置。
            joint.setVelocity(0.0)
            # 逐行说明：设置对应刚体或关节的权威初始速度。
        # 给反作用轮设置小的非零初始轮速，验证控制器能读取和调节真实轮状态。
        for joint, rpm in zip(wheel_joints, (20.0, -15.0, 10.0), strict=True):
        # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
            joint.setVelocity(rpm * 2.0 * math.pi / 60.0)
            # 逐行说明：设置对应刚体或关节的权威初始速度。

        # -------------------------------------------------------------------
        # 分块推进与墙钟节奏控制
        # -------------------------------------------------------------------
        # 每次让 Basilisk 从当前时刻推进到下一 1/30 s 边界。内部仍执行约
        # 16～17 个 2 ms 步；这样既能实时发送，又不会让 UE 帧率反压动力学。
        wall_start = time.monotonic()
        # 逐行说明：记录墙钟起点，用于按 simulation_rate 控制实时节奏。
        frame_count = int(math.ceil(duration * 30.0))
        # 逐行说明：计算整个任务需要执行的 30 Hz 外层推进次数。
        announced_phases: set[str] = set()
        # 逐行说明：记录已经发送到 UE 的阶段，保证每阶段只发布一次。
        for frame in range(1, frame_count + 1):
        # 逐行说明：遍历该序列中的元素，并对每个元素执行随后缩进的处理。
            sim_seconds = min(frame / 30.0, duration)
            # 逐行说明：计算当前外层帧对应的目标仿真时间。
            if mission_control["paused"]:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                # 暂停时不推进仿真时间，但仍以约 30 Hz 处理 UE 命令，否则从
                # UI 发出的 mission.resume 永远没有机会被仿真线程读取。
                pause_started = time.monotonic()
                # 逐行说明：计算本行右侧表达式，并把结果保存到 `pause_started` 供后续步骤使用。
                while mission_control["paused"]:
                # 逐行说明：只要该条件仍为真，就重复执行随后缩进的循环体。
                    bridge.process_commands(macros.sec2nano(max(0.0, (frame - 1) / 30.0)))
                    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
                    time.sleep(1.0 / 30.0)
                    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
                wall_start += time.monotonic() - pause_started
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
            # simulation_rate=1 表示 1 仿真秒对应 1 墙钟秒；更大值只减少等待，
            # 不改变模型步长、控制增益或状态时间戳。
            if not recording_only:
            # 逐行说明：实时发送时遵守墙钟倍率；仅录制时尽快完成计算。
                deadline = wall_start + sim_seconds / simulation_rate
                # 逐行说明：计算当前仿真时间按指定倍率对应的墙钟截止时刻。
                time.sleep(max(0.0, deadline - time.monotonic()))
                # 逐行说明：等待到当前实时帧截止时刻，但不改变权威数值步长。
            simulation.ConfigureStopTime(macros.sec2nano(sim_seconds))
            # 逐行说明：把 Basilisk 下一次停止时间推进到当前 30 Hz 边界。
            simulation.ExecuteSimulation()
            # 逐行说明：执行内部 2 ms 步，直到到达刚设置的停止时间。
            # 每个阶段只发布一次事件，防止 30 Hz 循环把事件队列刷满。
            phase = mission_phase(sim_seconds)
            # 逐行说明：计算当前任务阶段名称。
            if phase not in announced_phases:
            # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
                announced_phases.add(phase)
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
                bridge.publish_event(
                # 逐行说明：通过可靠事件队列向 UE 发布一次任务阶段变化。
                    "mission_phase",
                    # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
                    {
                    # 逐行说明：开始构造当前列表、元组或字典表达式。
                        "severity": "info",
                        # 逐行说明：设置结构中的 `severity` 字段，供下游模块按稳定键名读取。
                        "message": f"Mission phase: {phase}",
                        # 逐行说明：设置结构中的 `message` 字段，供下游模块按稳定键名读取。
                        "phase": phase,
                        # 逐行说明：设置结构中的 `phase` 字段，供下游模块按稳定键名读取。
                        "sim_time_ns": str(macros.sec2nano(sim_seconds)),
                        # 逐行说明：设置结构中的 `sim_time_ns` 字段，供下游模块按稳定键名读取。
                    },
                    # 逐行说明：结束当前多行容器、函数调用或表达式。
                )
                # 逐行说明：结束当前多行容器、函数调用或表达式。
        wall_elapsed_seconds = time.monotonic() - wall_start
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `wall_elapsed_seconds` 供后续步骤使用。

        # -------------------------------------------------------------------
        # 数值验收：这些计算不参与控制，只判断任务是否真正完成
        # -------------------------------------------------------------------
        positions = np.asarray(bus_recorder.r_BN_N, dtype=float).reshape(-1, 3)
        # 逐行说明：整理卫星记录器中的惯性位置序列。
        velocities = np.asarray(bus_recorder.v_BN_N, dtype=float).reshape(-1, 3)
        # 逐行说明：整理卫星记录器中的惯性速度序列。
        target_positions = np.asarray(target_recorder.r_BN_N, dtype=float).reshape(-1, 3)
        # 逐行说明：整理目标记录器中的惯性位置序列。
        target_velocities = np.asarray(target_recorder.v_BN_N, dtype=float).reshape(-1, 3)
        # 逐行说明：整理目标记录器中的惯性速度序列。
        if len(positions) < 2:
        # 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
            raise RuntimeError("orbital mission did not produce enough state samples")
            # 逐行说明：检测到无法继续的输入或运行状态，立即抛出明确异常。
        initial_radius, initial_semimajor = _specific_orbit(positions[0], velocities[0], earth.mu)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        final_radius, final_semimajor = _specific_orbit(positions[-1], velocities[-1], earth.mu)
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        # sigma_BR 是本体相对参考姿态的 MRP 误差，取三维向量范数便于设阈值。
        attitude_errors = np.linalg.norm(
        # 逐行说明：计算每个样本的 MRP 姿态误差范数。
            np.asarray(attitude_recorder.sigma_BR, dtype=float).reshape(-1, 3), axis=1
            # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        wheel_speeds_data = np.column_stack(
        # 逐行说明：把三个轮速记录器合并为按时间排列的矩阵。
            [np.asarray(recorder.state, dtype=float).reshape(-1) for recorder in wheel_recorders]
            # 逐行说明：开始构造当前列表、元组或字典表达式。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        approach_samples = np.asarray(approach_recorder.input, dtype=float).reshape(-1)
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `approach_samples` 供后续步骤使用。
        braking_samples = np.asarray(braking_recorder.input, dtype=float).reshape(-1)
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `braking_samples` 供后续步骤使用。
        relative_positions = target_positions - positions
        # 逐行说明：计算完整记录中的目标减卫星相对位置。
        sample_seconds = np.asarray(bus_recorder.times(), dtype=float) * macros.NANO2SEC
        # 逐行说明：把记录器纳秒时间戳转换为秒。
        # 22 s 是平移交会向机械臂任务交接的时刻：检查位置、相对速度和姿态。
        rendezvous_index = min(
        # 逐行说明：定位 22 s 交会到机械臂任务的交接样本。
            len(sample_seconds) - 1,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            int(np.searchsorted(sample_seconds, GRASP_TIME_OFFSET_S)),
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        rendezvous_error = relative_positions[rendezvous_index] - native.TARGET_POS
        # 逐行说明：计算交接时实际相对位置相对最终抓取位置的误差。
        handoff_attitude_error = float(attitude_errors[rendezvous_index])
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `handoff_attitude_error` 供后续步骤使用。
        maximum_pregrasp_attitude_error = float(np.max(attitude_errors[: rendezvous_index + 1]))
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `maximum_pregrasp_attitude_error` 供后续步骤使用。
        rendezvous_relative_speed = float(np.linalg.norm(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `rendezvous_relative_speed` 供后续步骤使用。
            target_velocities[rendezvous_index] - velocities[rendezvous_index]
            # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
        ))
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 26.5 s 关闭平移控制：再次检查夹爪闭合前相对状态是否仍然稳定。
        closure_index = min(
        # 逐行说明：定位 26.5 s 平移控制关闭时的样本。
            len(sample_seconds) - 1,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            int(np.searchsorted(sample_seconds, RENDEZVOUS_CONTROL_STOP_S)),
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        closure_position_error = relative_positions[closure_index] - native.TARGET_POS
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `closure_position_error` 供后续步骤使用。
        closure_relative_speed = float(np.linalg.norm(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `closure_relative_speed` 供后续步骤使用。
            target_velocities[closure_index] - velocities[closure_index]
            # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
        ))
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        initial_relative_position = target_positions[0] - positions[0]
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `initial_relative_position` 供后续步骤使用。
        approach_translation = float(np.linalg.norm(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `approach_translation` 供后续步骤使用。
            relative_positions[rendezvous_index] - initial_relative_position
            # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
        ))
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 比较 29 s 和 31 s 的相对位置，确认目标随夹爪产生了明显回撤。
        withdrawal_start = min(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `withdrawal_start` 供后续步骤使用。
            len(sample_seconds) - 1,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            int(np.searchsorted(sample_seconds, GRASP_TIME_OFFSET_S + 7.0)),
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        withdrawal_stop = min(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `withdrawal_stop` 供后续步骤使用。
            len(sample_seconds) - 1,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            int(np.searchsorted(sample_seconds, GRASP_TIME_OFFSET_S + 9.0)),
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        grasp_withdrawal = float(
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `grasp_withdrawal` 供后续步骤使用。
            np.linalg.norm(relative_positions[withdrawal_stop] - relative_positions[withdrawal_start])
            # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 独立重放 29 s 以后状态，检查双侧接触、夹爪-目标漂移和最终相对速度。
        contact_metrics = _contact_replay_metrics(
        # 逐行说明：保存独立 MuJoCo 接触重建得到的抓取质量指标。
            augmented_path,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            native_recorders[0],
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            GRASP_TIME_OFFSET_S + 7.0,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            duration,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 指标分为五类：运行节奏、轨道安全、交会精度、姿态/RW 状态、抓取质量。
        metrics: dict[str, float | int | bool] = {
        # 逐行说明：汇总轨道、交会、姿态、轮速、推力和抓取验收结果。
            "duration_s": duration,
            # 逐行说明：设置结构中的 `duration_s` 字段，供下游模块按稳定键名读取。
            "wall_elapsed_s": wall_elapsed_seconds,
            # 逐行说明：设置结构中的 `wall_elapsed_s` 字段，供下游模块按稳定键名读取。
            "achieved_realtime_factor": duration / wall_elapsed_seconds,
            # 逐行说明：设置结构中的 `achieved_realtime_factor` 字段，供下游模块按稳定键名读取。
            "dynamics_rate_hz": 1.0 / MISSION_TIME_STEP_S,
            # 逐行说明：设置结构中的 `dynamics_rate_hz` 字段，供下游模块按稳定键名读取。
            "initial_altitude_m": initial_radius - earth.radEquator,
            # 逐行说明：设置结构中的 `initial_altitude_m` 字段，供下游模块按稳定键名读取。
            "final_altitude_m": final_radius - earth.radEquator,
            # 逐行说明：设置结构中的 `final_altitude_m` 字段，供下游模块按稳定键名读取。
            "initial_semimajor_axis_m": initial_semimajor,
            # 逐行说明：设置结构中的 `initial_semimajor_axis_m` 字段，供下游模块按稳定键名读取。
            "final_semimajor_axis_m": final_semimajor,
            # 逐行说明：设置结构中的 `final_semimajor_axis_m` 字段，供下游模块按稳定键名读取。
            "semimajor_axis_change_m": final_semimajor - initial_semimajor,
            # 逐行说明：设置结构中的 `semimajor_axis_change_m` 字段，供下游模块按稳定键名读取。
            "final_bus_target_distance_m": float(np.linalg.norm(target_positions[-1] - positions[-1])),
            # 逐行说明：设置结构中的 `final_bus_target_distance_m` 字段，供下游模块按稳定键名读取。
            "initial_rendezvous_offset_m": RENDEZVOUS_INITIAL_OFFSET_M,
            # 逐行说明：设置结构中的 `initial_rendezvous_offset_m` 字段，供下游模块按稳定键名读取。
            "approach_translation_m": approach_translation,
            # 逐行说明：设置结构中的 `approach_translation_m` 字段，供下游模块按稳定键名读取。
            "rendezvous_position_error_m": float(np.linalg.norm(rendezvous_error)),
            # 逐行说明：设置结构中的 `rendezvous_position_error_m` 字段，供下游模块按稳定键名读取。
            "rendezvous_relative_speed_m_s": rendezvous_relative_speed,
            # 逐行说明：设置结构中的 `rendezvous_relative_speed_m_s` 字段，供下游模块按稳定键名读取。
            "closure_position_error_m": float(np.linalg.norm(closure_position_error)),
            # 逐行说明：设置结构中的 `closure_position_error_m` 字段，供下游模块按稳定键名读取。
            "closure_relative_speed_m_s": closure_relative_speed,
            # 逐行说明：设置结构中的 `closure_relative_speed_m_s` 字段，供下游模块按稳定键名读取。
            "rendezvous_error_x_m": float(rendezvous_error[0]),
            # 逐行说明：设置结构中的 `rendezvous_error_x_m` 字段，供下游模块按稳定键名读取。
            "rendezvous_error_y_m": float(rendezvous_error[1]),
            # 逐行说明：设置结构中的 `rendezvous_error_y_m` 字段，供下游模块按稳定键名读取。
            "rendezvous_error_z_m": float(rendezvous_error[2]),
            # 逐行说明：设置结构中的 `rendezvous_error_z_m` 字段，供下游模块按稳定键名读取。
            "grasp_withdrawal_m": grasp_withdrawal,
            # 逐行说明：设置结构中的 `grasp_withdrawal_m` 字段，供下游模块按稳定键名读取。
            **contact_metrics,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            # 同时满足回撤距离、抓取点漂移、线速度和角速度阈值，才认为目标确实
            # 被稳定抓住并随机械臂移动，而不是仅发生瞬时碰撞。
            "grasp_motion_detected": bool(
            # 逐行说明：设置结构中的 `grasp_motion_detected` 字段，供下游模块按稳定键名读取。
                grasp_withdrawal >= 0.025
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                and contact_metrics["withdrawal_grasp_site_drift_m"] <= 0.003
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                and contact_metrics["final_gripper_target_relative_speed_m_s"] <= 0.005
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                and contact_metrics["final_gripper_target_relative_rate_rad_s"] <= 0.08
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            ),
            # 逐行说明：结束当前多行容器、函数调用或表达式。
            "maximum_attitude_error_mrp": float(np.max(attitude_errors)),
            # 逐行说明：设置结构中的 `maximum_attitude_error_mrp` 字段，供下游模块按稳定键名读取。
            "maximum_pregrasp_attitude_error_mrp": maximum_pregrasp_attitude_error,
            # 逐行说明：设置结构中的 `maximum_pregrasp_attitude_error_mrp` 字段，供下游模块按稳定键名读取。
            "handoff_attitude_error_mrp": handoff_attitude_error,
            # 逐行说明：设置结构中的 `handoff_attitude_error_mrp` 字段，供下游模块按稳定键名读取。
            "final_attitude_error_mrp": float(attitude_errors[-1]),
            # 逐行说明：设置结构中的 `final_attitude_error_mrp` 字段，供下游模块按稳定键名读取。
            "maximum_wheel_speed_rad_s": float(np.max(np.abs(wheel_speeds_data))),
            # 逐行说明：设置结构中的 `maximum_wheel_speed_rad_s` 字段，供下游模块按稳定键名读取。
            "wheel_speed_within_limit": bool(
            # 逐行说明：设置结构中的 `wheel_speed_within_limit` 字段，供下游模块按稳定键名读取。
                np.max(np.abs(wheel_speeds_data)) <= RW_SPEED_LIMIT_RAD_S
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
            ),
            # 逐行说明：结束当前多行容器、函数调用或表达式。
            "approach_thrust_sample_count": int(np.count_nonzero(approach_samples > 1.0e-9)),
            # 逐行说明：设置结构中的 `approach_thrust_sample_count` 字段，供下游模块按稳定键名读取。
            "braking_thrust_sample_count": int(np.count_nonzero(braking_samples > 1.0e-9)),
            # 逐行说明：设置结构中的 `braking_thrust_sample_count` 字段，供下游模块按稳定键名读取。
            "orbit_safe": bool(np.min(np.linalg.norm(positions, axis=1)) - earth.radEquator > 450_000.0),
            # 逐行说明：设置结构中的 `orbit_safe` 字段，供下游模块按稳定键名读取。
            # 交接条件：位置误差 <1 cm、相对速度 <5 mm/s、MRP误差 <0.01。
            "rendezvous_handoff_stable": bool(
            # 逐行说明：设置结构中的 `rendezvous_handoff_stable` 字段，供下游模块按稳定键名读取。
                np.linalg.norm(rendezvous_error) < 0.01
                # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
                and rendezvous_relative_speed < 0.005
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
                and handoff_attitude_error < 0.01
                # 逐行说明：执行本行表达式，完成当前代码块中的这一项具体操作。
            ),
            # 逐行说明：结束当前多行容器、函数调用或表达式。
        }
        # 逐行说明：结束当前多行容器、函数调用或表达式。
        # 保存 JSON 便于自动测试、绘图或与后续参数调优结果比较。
        metrics_path.parent.mkdir(parents=True, exist_ok=True)
        # 逐行说明：确保输出目录存在，并允许递归创建父目录。
        metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
        return metrics
        # 逐行说明：结束当前函数，并把本行给出的结果返回给调用者。
    finally:
    # 逐行说明：无论前面是否发生异常，都执行随后缩进的恢复或清理操作。
        # 无论正常结束还是抛出异常，都停止发送线程并释放 TCP 端口。
        bridge.close()
        # 逐行说明：关闭非阻塞发送线程和 TCP 资源。
        gravity_factory.unloadSpiceKernels()
        # 逐行说明：任务结束后卸载本场景加载的 Basilisk SPICE 星历核。
        # Basilisk/SWIG 对象由 Python 引用关系维持生命周期。显式收集到元组中
        # 可防止某些控制器/消息包装器在 run() 返回前被垃圾回收。
        _ = (
        # 逐行说明：计算本行右侧表达式，并把结果保存到 `_` 供后续步骤使用。
            native_dynamics,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            native_recorders,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            gravity_model,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            gravity_factory,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            ephemeris,
            # 逐行说明：保持 SPICE 星历模块存活到任务结束。
            sun,
            # 逐行说明：保持太阳重力天体和星历消息连接存活到任务结束。
            navigation,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            attitude_reference,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            attitude_error,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            wheel_speeds,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            attitude_controller,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            torque_mapping,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            torque_splitter,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            wheel_limiters,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
            maneuver_command,
            # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        )
        # 逐行说明：结束当前多行容器、函数调用或表达式。


def main() -> None:
# 逐行说明：开始定义 `main` 函数，具体职责由紧随其后的中文文档字符串说明。
    """解析命令行参数并运行任务；PowerShell 启动脚本最终调用这里。"""
    parser = argparse.ArgumentParser(description=__doc__)
    # 逐行说明：创建命令行参数解析器。
    parser.add_argument("--model-root", type=Path, required=True)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--catalog", type=Path, required=True)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--host", default="127.0.0.1")
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--port", type=int, default=5558)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--duration", type=float, default=34.0)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--simulation-rate", type=float, default=1.0)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--generated-mjcf", type=Path, required=True)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--metrics", type=Path, required=True)
    # 逐行说明：声明一个可由 PowerShell 启动脚本传入的命令行参数。
    parser.add_argument("--recording", type=Path)
    # 逐行说明：可选通用 `.bskrec` 输出路径；实时模式下也可以同步记录。
    parser.add_argument("--record-only", action="store_true")
    # 逐行说明：只运行 BSK/MJScene 并生成录制，不尝试连接正在运行的 UE。
    args = parser.parse_args()
    # 逐行说明：解析并保存 PowerShell 启动脚本传入的命令行参数。
    if args.record_only and args.recording is None:
    # 逐行说明：检查仅录制模式是否指定了有效输出路径。
        parser.error("--recording is required with --record-only")
        # 逐行说明：向命令行用户报告缺失参数并停止运行。
    metrics = run(
    # 逐行说明：汇总轨道、交会、姿态、轮速、推力和抓取验收结果。
        args.model_root.resolve(),
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.catalog.resolve(),
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.host,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.port,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.duration,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.simulation_rate,
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.generated_mjcf.resolve(),
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.metrics.resolve(),
        # 逐行说明：向当前多行参数列表或数据结构提供这一项，并继续读取下一项。
        args.recording.resolve() if args.recording is not None else None,
        # 逐行说明：将可选录制路径规范化为绝对路径后传入通用渲染桥。
        args.record_only,
        # 逐行说明：将是否禁用实时 TCP 的选择传给任务运行函数。
    )
    # 逐行说明：结束当前多行容器、函数调用或表达式。
    print("Orbital grasp mission completed")
    # 逐行说明：把任务完成状态或验收指标输出到终端，便于人工检查。
    print(json.dumps(metrics, indent=2))
    # 逐行说明：把任务完成状态或验收指标输出到终端，便于人工检查。
    if args.recording is not None:
    # 逐行说明：录制启用时明确输出最终文件路径，便于随后交给 UE 回放。
        print(f"Orbital grasp recording written to {args.recording.resolve()}")
        # 逐行说明：打印通用 `.bskrec` 文件的绝对路径。


if __name__ == "__main__":
# 逐行说明：判断该条件是否成立；成立时执行随后缩进的分支。
    main()
    # 逐行说明：执行本行函数或方法调用，把结果用于当前场景构建步骤。
