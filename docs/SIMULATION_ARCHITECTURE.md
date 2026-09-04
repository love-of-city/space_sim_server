# 仿真流程架构

重构分支 `refactor/simulation-architecture-20260904` 将仿真流程拆成四个边界：

```text
场景初始化
    ↓
SceneState / 状态消息
    ↓
SPICE / EphemerisProvider 更新环境
    ↓
BasiliskModuleRegistry 中的 Basilisk 模块
    ↓
ControlOutput（力、力矩、关节、反作用轮、推进器）
    ↓
MJScene（唯一的动力学积分、约束、接触和多刚体耦合权威）
    ↓
新的 SceneState / 状态发布
```

## 代码边界

- `simulation/architecture.py`
  - `SceneState`：完整场景状态快照；
  - `EnvironmentState`：SPICE/星历环境状态；
  - `ControlOutput`：Basilisk 到 MJScene 的统一执行器输出；
  - `SimulationOrchestrator`：标准状态闭环；
  - `BasiliskModuleRegistry`：将原生 Basilisk `SysModel` 按任务和优先级注册到仿真图。
- `simulation/teleop_grasp_unreal.py`
  - 保留现有场景构建和通信兼容性；
  - 已将 SPICE、遥操作 IK 和 UE 状态发布器统一通过 `BasiliskModuleRegistry` 挂载；
  - 后续 Basilisk 模块应通过 registry 接入，而不是在入口函数中散落调用 `AddModelToTask`。

## 新增 Basilisk 模块的约定

1. 明确模块读取的 `SceneState` / Basilisk 消息；
2. 明确模块输出的力、力矩或执行器命令；
3. 为原生 `SysModel` 指定目标 task 和 priority；
4. 使用 `module_registry.register(...)` 注册；
5. 为状态输入、输出映射和模块顺序增加测试。

UE 端不参与动力学积分。UE 只接收 `bsk-render/2` 状态帧，负责渲染、相机、特效和任务 UI，因此增加 Basilisk 模块不会要求 UE 修改，除非新增模块需要展示新的可视化对象或通道。
