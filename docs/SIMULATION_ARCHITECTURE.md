# 仿真核心架构与扩展约定

> 更新：2026-09-08。本文聚焦 BSK/MJScene 核心；整个前端、后端、UE、视频、采集与部署关系见[总体架构文档](SYSTEM_ARCHITECTURE.md)。
>
> 本文区分当前 SARM 实际调用路径与通用架构抽象。8/6 关节契约不一致已修复；新姿态链详见[反作用轮与姿态保持](ATTITUDE_CONTROL.md)，其余限制见[当前缺口](SYSTEM_ARCHITECTURE.md#gaps)。

## 当前 SARM 实际执行路径

```text
scripts/run_simulation.ps1
  → simulation/teleop_grasp_unreal.py
  → 适配器 load_native_grasp_module()
  → model/SARM/platform/scenarios/scenario_sarm_grasp.py
      → SimulationBaseClass + graspProcess + graspTask（2 ms）
      → MJScene.fromFile(sarm_platform.xml)
      → 目标消息 + 8 组 PID / 限幅器 / 执行器连接
      → 三个轮体/hinge motor + AttitudeControl（独立 100 Hz 姿态任务）
  → 服务端接入 SPICE、Earth/Sun NBodyGravity、独立 IK task、渲染桥
  → 初始化自由体轨道与目标/关节状态
  → ConfigureStopTime() / ExecuteSimulation() 分段推进
  → 原生状态经渲染桥发往 UE，观测另发往后端
```

BSK 负责调度、环境模型、IK/PID、姿态反馈/轮矩分配和指令处理；MJScene 是当前多刚体积分、约束、接触与力耦合的唯一权威。UE 不参与物理积分。

太阳光照倍率通过实例的 `environment.lighting.sunlight_intensity_scale` → `SceneSettings` → UE 传递，默认 1、范围 0～10。它只改变渲染，不输入重力/姿态模块，详见[太阳光照初始化](SUNLIGHT_CONFIGURATION.md)。

## 当前任务与消息顺序

| 层级 | 模块 | 优先级/频率 |
| --- | --- | --- |
| MJScene 动力学子任务 | SPICE | `20000` |
| MJScene 动力学子任务 | 正向运动学 | `10000` |
| MJScene 动力学子任务 | `NBodyGravity` | `9500` |
| MJScene 动力学子任务 | 轨迹/目标消息发布 | `9000` |
| MJScene 动力学子任务 | 第 i 个 PID / 限幅器 | `8000-i` / `7000-i` |
| MJScene 动力学子任务 | 反作用轮限矩/轮速保护驱动 | `6500-i` |
| 独立 `sarmAttitudeTask` | 真值导航、初始惯性参考、MRP PD、轮矩分配/转换 | 默认 100 Hz，process 中 task 优先级 `-10` |
| 独立 `teleopIkTask` | 遥操作 IK | 默认 100 Hz，process 中 task 优先级 `100` |
| 外层 `graspTask` | `BasiliskRenderBridge` | 优先级 `-10000`，桥内按名义 30 Hz 节流 |

同一任务内较高优先级先执行，不同层级的数字不可直接比较。动力学子任务可在积分子步多次执行；IK 只更新缓存目标，轨迹发布器读取缓存，不重复求解。

## 通用架构抽象（不等于已全部接入默认场景）

[simulation/architecture.py](../simulation/architecture.py) 定义的扩展流程是：

```text
SceneBackend.read_state()
  → SceneState / 状态发布
  → EphemerisProvider.update(sim_time_s)
  → BasiliskModule.update(state, environment, dt)
  → ControlOutput.combine()
  → SceneBackend.apply_control() / step()
  → 新的 SceneState / 状态发布
```

主要边界：

- `SceneState`：刚体、质量属性、关节及反作用轮/推进器等状态抽象。
- `EnvironmentState`：环境状态。
- `ControlOutput`：力、力矩及执行器输出；力/力矩可组合，竞争的关节目标不能无声覆盖。
- `SceneBackend`、`EphemerisProvider`、`BasiliskModule`、`StatePublisher`：接口约定。
- `SimulationOrchestrator`：实现上述通用闭环，有测试，但当前 SARM 入口没有实例化它，也没有通过其 `SceneBackend.step()` 推进。
- `BasiliskModuleRegistry`：原生 `SysModel` 的 task/priority 注册器；当前实际用于 `teleop_ik`、`render_state_publisher`。

姿态任务由原生场景中的 `AttitudeControl` 直接安装，未经过 registry。当前 SPICE、重力和原生关节/轮驱动控制链仍直接通过 `scene.AddModelToDynamicsTask(...)` 接入。不能再将它们描述为均已迁入 registry。`space-sim-state/1` 是通用状态序列化接口，不是现有 `space-arm-control/1` 观测或 UE `bsk-render/2` 的替代协议。

## 新增模块约定

1. 明确输入状态/质量属性的来源、单位、坐标系和采样时刻。
2. 明确输出是环境力/力矩、执行器命令、导航结果还是仅遥测，避免重复积分和重复计力。
3. 普通原生 `SysModel` 明确目标 task、频率、priority；动力学子步模块接入 MJScene 对应任务。
4. 对同一 actuator 或目标消息明确唯一写入者及多控制源仲裁规则。
5. 若使用通用编排器，先实现和验证实际后端适配，不能仅注册 Python 接口就宣称原生系统已迁移。
6. 增加消息连接、模块顺序、数值行为、限幅、失联与端到端状态契约测试。

增加模块不一定需要修改 UE；只有新增可视化对象、设备通道或图像产品时才扩展适配器/UE。渲染表现不能代替对应物理器件已建模的证明。

## 关键代码与专题

- [实际遥操作入口](../simulation/teleop_grasp_unreal.py)
- [原生模型与 PID 构建](../model/SARM/platform/scenarios/scenario_sarm_grasp.py)
- [当前运行 MJCF](../model/SARM/platform/sarm_platform.xml)
- [总体架构：配置与器件归属](SYSTEM_ARCHITECTURE.md#configuration)
- [总体架构：时间与坐标](SYSTEM_ARCHITECTURE.md#frames)
- [动力学诊断](SARM_CONTROL_DIAGNOSIS.md)
- [星历与 UE 对齐](CELESTIAL_ALIGNMENT.md)
