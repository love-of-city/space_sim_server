# 机械臂控制链路冗余与不必要部分审查报告

> 审查日期：2026-09-16\
> 代码基线：`c343ca9b5f92cfa1897cccf1c08cec2fdf716cb7`（`fix:remove-teleop-tracking-error-gating`）\
> 审查范围：浏览器输入、后端安全处理、后端到仿真传输、实时 IK、关节控制器、观测回传、Episode 录制、场景状态查询及协议兼容字段。\
> 本报告只提出精简建议，不修改控制逻辑。

## 1. 结论摘要

机械臂控制算法的核心链路基本都必要：

```text
输入归一化
→ deadman / 数值校验 / 速度限幅
→ 最新动作缓存
→ 笛卡尔速度 IK
→ 关节位置与速度参考
→ 关节 PD
→ 力矩限幅
→ MJScene 动力学积分
→ 状态观测
```

真正不必要的部分主要不是 IK 或动力学步骤，而是以下几类：

1. **控制热路径上的同步文件 I/O**：动作记录和观测记录会直接占用异步事件循环。
2. **每条动作重复读取场景状态文件**：当前最多约 30 次/秒读取并解析 `run/scene_runtime.json`。
3. **只写不读的死状态**：`SimulationHub._latest_action` 没有生产消费者。
4. **恒定或未消费的诊断字段**：`tracking_scale` 恒为 `1.0`，部分奇异值诊断只计算不发布。
5. **旧协议和旧名称**：`gripper_velocity_rad_s`、`damped_least_squares_ik`、`so101CartesianIkController` 与实际实现不一致。
6. **可合并的数值计算和初始化**：重复 SVD、重复关节初始化，以及空闲状态下 30 Hz 的完整动作和 ACK 消息。

这些问题不会改变当前控制算法的正确性，但会降低实时余量、增加数据量，并让接口语义变得模糊。

## 2. 判定标准

本报告将候选问题分为三类：

| 分类 | 定义 | 处理原则 |
| --- | --- | --- |
| A. 可直接删除 | 无生产消费者，删除后不改变控制、安全或数据语义 | 优先清理 |
| B. 可移出热路径或合并 | 功能必要，但不应阻塞控制消息或重复计算 | 保留功能，调整位置 |
| C. 有条件删除 | 当前控制不需要，但可能服务于诊断、数据或旧客户端 | 先决定协议和数据需求 |

以下内容不应仅因“没有直接参与 IK”就被删除：

- 三层失联保护；
- 前后端和仿真端重复校验；
- 多页面控制权仲裁；
- Episode 录制和渲染状态发布；
- `reset_generation`；
- IK 后的关节位置限幅。

这些步骤分别覆盖不同的失效域或产品目标，属于纵深防御或有条件必要功能。

## 3. 审查结果汇总

| ID | 级别 | 问题 | 建议 |
| --- | --- | --- | --- |
| R-01 | P0 | 动作记录先于仿真下发，且同步写 `actions.jsonl` | 先发布到仿真，再异步记录 |
| R-02 | P0 | 观测记录在事件循环内同步写 `steps.jsonl` | 使用有界队列和独立写线程 |
| R-03 | P1 | 每条动作读取 `scene_runtime.json` | 改为内存状态缓存，低频校验进程与文件 |
| R-04 | P1 | `SimulationHub._latest_action` 只写不读 | 删除该状态 |
| R-05 | P1 | `tracking_scale` 恒为 `1.0` 但仍发布 | 删除或改成明确的“门控已关闭”状态 |
| R-06 | P1 | `velocity_scale`、最小奇异值、条件数只计算不发布 | 正式发布用于诊断，或删除返回字段 |
| R-07 | P1 | `gripper_velocity_rad_s` 是错误单位的兼容别名 | 协议 v2 删除，保留短期适配器 |
| R-08 | P1 | 空闲时仍以约 30 Hz 发送零动作和 ACK | 运动时保持高频，空闲时事件驱动加低频心跳 |
| R-09 | P2 | 受限 IK 在 SVD 后再次调用 `matrix_rank` | 直接复用奇异值计算 Rank |
| R-10 | P2 | IK capability 和 ModelTag 使用旧 SO-101/阻尼 IK 名称 | 重命名为当前真实语义 |
| R-11 | P2 | 旧 `inverse_velocity()` 只被测试和基准工具使用 | 明确标记 legacy 或迁到基准模块 |
| R-12 | P2 | 关节初始状态被初始化两次 | 合并为一次带场景参数的初始化 |
| R-13 | P2 | 每帧正运动学主要用于跟踪误差诊断 | 若保留诊断则必要；若删除诊断可一并精简 |

## 4. P0：控制热路径上的非控制 I/O

### R-01 动作记录阻塞仿真下发

**证据**

- `backend/space_arm_platform/app.py:712-713` 先调用 `recorder.record_action(action)`，再调用 `hub.publish_action(action)`。
- `backend/space_arm_platform/recorder.py:118-125` 在调用线程内同步打开并追加 `actions.jsonl`。
- `backend/space_arm_platform/recorder.py:253-254` 每条记录都执行一次文件打开、写入和关闭。

**问题**

录制是否开启会影响控制消息送达仿真的时间。磁盘短暂阻塞、杀毒扫描、文件系统抖动或日志目录所在磁盘繁忙时，控制延迟会直接增大。动作记录属于数据功能，不应位于机械臂控制的关键路径。

**建议**

调整顺序：

```text
安全处理 action
→ 先 publish_action 到仿真
→ 将 action 放入有界录制队列
→ 独立写线程批量写入 actions.jsonl
```

队列需要定义背压策略：

- 优先保持控制不阻塞；
- 队列满时记录明确的 `recording_dropped` 或 `episode_degraded` 状态；
- Episode 停止时执行有上限的 flush；
- 不允许为了等待磁盘而阻塞 30 Hz 控制链。

**修改风险**

低到中。主要风险是动作记录顺序、Episode 完整性和停止时 flush 语义，需要增加录制顺序测试。

### R-02 观测记录阻塞异步事件循环

**证据**

- `backend/space_arm_platform/app.py:192-195` 将同步 `EpisodeRecorder.record_observation()` 注册为 Hub 回调。
- `backend/space_arm_platform/simulation_hub.py:189-190` 在仿真连接处理协程中直接 `await` 该回调。
- `backend/space_arm_platform/recorder.py:127-151` 在回调中同步写入 `steps.jsonl`。
- `backend/space_arm_platform/recorder.py:253-254` 每个观测都重新打开文件。

**问题**

`record_observation()` 本身没有真正的异步切换，因此执行期间的磁盘 I/O 会阻塞 FastAPI 事件循环。录制开启时，动作最多约 30 次/秒、观测最多约 30 次/秒，合计可能形成约 60 次/秒的同步文件打开、写入和关闭。

这会影响：

- WebSocket 动作接收；
- observation 广播；
- `/api/state` 和其他 HTTP 请求；
- 场景状态查询；
- Episode 停止时的状态同步。

**建议**

使用单一有界录制队列，或按 `actions.jsonl`、`steps.jsonl` 分成两个队列：

```text
SimulationHub
→ 非阻塞 enqueue(observation)
→ recorder writer thread
→ 批量 append / fsync 策略
```

重要约束：

- 观测写入顺序必须按 `render_frame_id` 保持；
- 相机配对仍需按 `render_frame_id` 和 `sim_time_ns` 完成；
- 队列拥塞时宁可标记 Episode 不完整，也不要阻塞控制通信；
- `stop()` 只等待有界 flush，不无限等待。

**修改风险**

中。需要补充并发、顺序、停止 flush 和队列溢出测试。

## 5. P1：重复状态、错误语义和无效流量

### R-03 每条动作重复读取场景状态文件

**证据**

- `backend/space_arm_platform/app.py:651-652` 的 `user_controls_current_scene()` 每次调用 `scenes.status()`。
- `backend/space_arm_platform/app.py:703` 对每条动作执行该检查。
- `backend/space_arm_platform/scene_runtime.py:351-356` 每次 `status()` 都读取并解析 `run/scene_runtime.json`。

**问题**

当前前端以约 30 Hz 发送动作，因此控制期间最多会每秒读取并解析约 30 次同一个状态文件。场景阶段变化远低于该频率，该 I/O 对控制权限判断没有等比例收益。

**建议**

将场景阶段和实例信息缓存在 `SceneRuntimeManager` 内：

- 启动、停止、重置、进程退出时更新缓存；
- 每条动作只检查内存中的 `phase == running` 和所有权；
- 可以低频或仅在 API 查询时重新读取运行时文件；
- 子进程退出的主动检测应保留，但不必与每条动作绑定。

**修改风险**

低。需要确保子进程崩溃后控制权限能及时失效。

### R-04 `SimulationHub._latest_action` 是只写不读的死状态

**证据**

- `backend/space_arm_platform/simulation_hub.py:27` 定义字段。
- 同文件 `89`、`121`、`129`、`157`、`198` 只进行赋值或清空。
- 全仓库生产代码没有读取该字段。
- `tests/test_scene_reset.py:203` 仅断言重置后该字段为 `None`。

**问题**

它容易让人误以为 Hub 同时维护了两套“最近动作”状态。实际控制、API 和录制使用的是：

- `SafetyController.last_action`；
- observations 中的 `applied_action_sequence`；
- `actions.jsonl` / `steps.jsonl`。

**建议**

删除 `_latest_action` 及全部赋值，同时删除或改写对应内部测试。若确实需要对外显示“最后成功发送的动作”，应增加一个命名明确且真正被 `/api/state` 使用的机制，而不是保留不可见字段。

**修改风险**

低。需要搜索第三方或未纳入仓库的外部消费者。

### R-05 `tracking_scale` 已失去控制作用

**证据**

- `simulation/teleop_grasp_unreal.py:432`、`497` 初始化。
- `simulation/teleop_grasp_unreal.py:555` 每帧固定写为 `1.0`。
- `simulation/teleop_grasp_unreal.py:1091` 继续发布该值。
- `docs/SARM_CONTROL_DIAGNOSIS.md:108-112` 已说明跟踪误差门控被删除。

**问题**

字段名称仍在暗示“跟踪误差会影响控制速度”，但当前实现中它没有任何控制作用，恒为 `1.0`。这会造成两个误导：

1. 阅读代码时误以为仍有门控；
2. 外部分析系统可能把该值当作有效控制状态。

**建议**

二选一：

- 删除 `tracking_scale`；
- 或改为明确的诊断字段，例如 `tracking_gate_enabled = false`，并说明跟踪误差仅用于遥测。

如果删除，应同步修改文档、观察契约和测试。位置与姿态跟踪误差本身仍有诊断价值，不必随该字段一起删除。

**修改风险**

低到中。主要是协议和外部数据兼容性。

### R-06 奇异值诊断只计算不发布

**证据**

- `simulation/serial_chain_kinematics.py:33-35` 定义 `velocity_scale`、`minimum_singular_value`、`condition_number`。
- `simulation/serial_chain_kinematics.py:205-212` 已计算 SVD、最小奇异值和条件数。
- `simulation/teleop_grasp_unreal.py:575-577` 将结果复制到 Target 状态。
- 这些字段没有出现在当前 observation 消费路径，前端也没有使用。

**问题**

它们不是完成运动控制所必需的结果，但又是判断“接近奇异位形”最直接的诊断量。当前状态是“计算了但没人看”，属于半完成接口。

**建议**

优先级从高到低：

1. 正式发布最小奇异值、条件数和 velocity scale；
2. 或者删除 Target 中的复制和 `IkResult` 中无消费者的字段；
3. 不建议保留当前“计算但不输出”的中间状态。

如果目标是机械臂调试，推荐方案 1，因为这些字段比仅发布 `jacobian_rank` 更能解释速度为何被整体缩放。

### R-07 `gripper_velocity_rad_s` 是错误单位的兼容别名

**证据**

- `backend/space_arm_platform/models.py:64-65` 同时定义 `gripper_velocity_rad_s` 和 `gripper_velocity_m_s`。
- `backend/space_arm_platform/safety.py:145-146` 将同一个米每秒值同时写入两个字段。
- `simulation/teleop_grasp_unreal.py:388-389` 接受任一字段。
- `simulation/teleop_grasp_unreal.py:591` 优先读取 `gripper_velocity_m_s`。
- `contracts/space-arm-control-v1.schema.json:84`、`134` 继续将旧字段列为协议内容。

**问题**

SARM 两个手指是滑动关节，单位是米和米每秒，不是弧度。继续传递 `rad_s` 会让日志和数据消费者得到错误单位解释。

**建议**

在新的协议版本中：

- 只保留 `gripper_velocity_m_s`；
- 服务端可在一个过渡期接受旧字段并转换；
- 仿真端不再把旧字段当作并列有效协议；
- 测试明确验证旧客户端迁移行为。

**修改风险**

中。会增加协议版本维护工作，应避免在同一提交中同时改控制算法。

### R-08 空闲状态下的 30 Hz 零动作和逐条 ACK

**证据**

- `frontend/app.js:202` 每 33 ms 调用一次 `sendAction()`。
- `frontend/app.js:864` 即使没有输入也会执行发送路径。
- `frontend/app.js:748` 发送完整 `operator_action`。
- `backend/space_arm_platform/app.py:716-719` 每条动作都返回 `action_ack`。
- `frontend/app.js:448-450` 仅使用 ACK 更新序列号和延迟时间。

**问题**

运动期间约 30 Hz 的动作流是合理的。但空闲时持续发送零动作，会：

- 产生大量无变化动作记录；
- 产生逐条 ACK；
- 增加 WebSocket、TCP、JSON 和日志开销；
- 让“有效控制输入”和“连接心跳”混在同一条业务消息中。

不能简单删除心跳，因为 deadman 和失联保护依赖新鲜消息。问题在于空闲时不需要 30 Hz。

**建议**

改为事件驱动加低频心跳：

- 按键、摇杆、deadman 或速度档位变化：立即发送；
- 持续运动：保持约 30 Hz；
- 空闲：发送低频零动作心跳，例如 5–10 Hz；
- ACK 只在状态变化、限幅、拒绝或低频健康检查时发送；
- 如果数据集需要固定频率，可以在录制线程中重采样，而不是让网络层承担全部空闲记录。

**修改风险**

中。需要验证从空闲切换到运动的首帧延迟，以及后续 WebSocket 重连和 deadman 行为。

## 6. P2：重复计算、旧入口和初始化合并

### R-09 受限 IK 重复执行 SVD

**证据**

- `simulation/serial_chain_kinematics.py:205` 调用 `np.linalg.svd(jacobian, compute_uv=False)`。
- `simulation/serial_chain_kinematics.py:212` 又调用 `np.linalg.matrix_rank(jacobian, tol=1.0e-5)`。
- 对二维矩阵，`matrix_rank` 内部还会再次执行 SVD。

**问题**

同一 6×6 雅可比在每次 IK 更新中做两次奇异值分解。默认 IK 频率为 100 Hz 时，这最多意味着每秒约 100 次额外的小矩阵 SVD。绝对耗时不大，但这是完全可避免的重复计算，而且位于实时控制任务中。

**建议**

直接使用已计算的奇异值：

```python
rank = int(np.count_nonzero(singular_values > 1.0e-5))
```

需要增加测试，确保结果与当前固定容差的 `matrix_rank` 一致。

**修改风险**

低。数值阈值必须与现有一致。

### R-10 能力名和 ModelTag 使用旧语义

**证据**

- `simulation/teleop_grasp_unreal.py:336` 声明 `damped_least_squares_ik`。
- `simulation/teleop_grasp_unreal.py:629` 使用 `so101CartesianIkController`。
- 当前实时实现使用 `inverse_velocity_bounded()`，并采用方向保持的受限最小二乘求解。

**问题**

当前 capability 容易让外部系统误以为使用的是固定阻尼 IK；ModelTag 仍保留旧 SO-101 名称。它们不影响运行，但会影响协议判断、日志检索和代码阅读。

**建议**

- capability 改为 `bounded_velocity_ik` 或更精确的名称；
- ModelTag 改为 `sarmCartesianIkController`；
- 若兼容旧客户端，保留旧 capability 的只读映射，但不要把它描述为新算法能力。

**修改风险**

低到中。需要检查是否有外部客户端根据 capability 名称分支。

### R-11 旧 `inverse_velocity()` 只用于测试和基准

**证据**

- `simulation/serial_chain_kinematics.py:137` 定义旧实现。
- 生产入口在 `simulation/teleop_grasp_unreal.py:563` 使用 `inverse_velocity_bounded()`。
- 旧实现只在 `tests/test_serial_chain_kinematics.py:47` 和 `tools/benchmark_runtime_layers.py:112,282` 中使用。

**问题**

同一模块中并列存在两个“可用”的逆速度 IK 方法，容易让新开发者误选旧实现。它作为基准参照仍有价值。

**建议**

- 重命名为 `inverse_velocity_unbounded_legacy()`；
- 或迁移到基准/测试辅助模块；
- 保留与 `inverse_velocity_bounded()` 的数值对照测试。

**修改风险**

低。

### R-12 关节初始状态被写入两次

**证据**

- `simulation/teleop_grasp_unreal.py:958` 调用原生 `_initialize_state()`。
- `model/SARM/platform/scenarios/scenario_sarm_grasp.py:250-262` 在该函数中写入 `PREGRASP` 关节状态。
- `simulation/teleop_grasp_unreal.py:969-970` 随后调用 `_apply_orbital_initial_state()`。
- `simulation/teleop_grasp_unreal.py:727-730` 再次写入 `initial_joints`。

**问题**

第二次写入是必要的，因为场景随机化后的 `initial_joints` 必须覆盖原生 PREGRASP。但第一次写入不应作为最终初始化路径保留，否则两条真值来源并存。

**建议**

将关节初态统一为一次初始化：

```text
_build_simulation()
→ initialize(simulation, scene, initial_joints, orbit)
```

不要让 `_initialize_state()` 和 `_apply_orbital_initial_state()` 分别拥有关节初态。该修改应先补充场景随机化和普通启动的回归测试。

**修改风险**

中。自由体、轨道和关节初始化耦合较多，不适合作为第一个清理提交。

### R-13 每帧正运动学主要用于跟踪误差诊断

**证据**

- `simulation/teleop_grasp_unreal.py:549` 在每次 IK 更新中用实际关节计算末端位姿。
- `simulation/teleop_grasp_unreal.py:583-585` 计算位置和姿态跟踪误差。
- `simulation/teleop_grasp_unreal.py:1091-1096` 将误差发布到 observation。

**问题**

该正运动学计算不是 IK 求解本身所需，只服务于跟踪误差遥测。如果团队确定不再需要这些误差，可以一并删除；如果保留诊断，这部分就是必要的。

**建议**

不要把 R-13 作为第一轮清理目标。先决定 `tracking_scale`、位置误差和姿态误差是否是正式诊断协议的一部分。若保留，应把字段正式写入 observation 模型，而不是依赖额外字段透传。

## 7. 不建议删除的“重复”

| 项目 | 为什么保留 |
| --- | --- |
| 前端、后端、仿真端数值校验 | 各自服务于 UI、信任边界和进程间协议边界 |
| 前端 deadman、后端 watchdog、仿真 stale 检查 | 三者覆盖页面停止、后端超时和进程失联三种不同故障 |
| `SimulationControlClient` 最新动作缓存 | 防止 500 Hz 物理线程积压过期命令 |
| `reset_generation` | 防止重置前旧命令在新场景中生效 |
| IK 后的关节位置限幅 | 防御数值误差和未来 IK 实现变更 |
| 关节 PD 和力矩限幅 | 物理执行的最后控制层，不能由 IK 替代 |
| observation 和 Episode 录制 | 训练数据平台的核心产品能力 |
| UE 渲染状态桥 | 只负责显示，不参与动力学控制 |
| `action_ack` 的失败语义 | 建议降频，但不应取消失败和限幅通知 |
| capability 协商 | 支持仿真与后端版本不一致和重置能力检查 |

## 8. 建议的目标链路

```text
浏览器输入变化
  → 立即发送动作
  → 空闲时低频心跳

后端 WebSocket
  → 权限和数值校验
  → SafetyController
  → SimulationHub.publish_action
  → 动作入有界录制队列

仿真进程
  → 最新动作缓存
  → 受限速度 IK
  → 复用一次 SVD 结果
  → 发布必要诊断
  → PD + 限幅 + MJScene

观测回传
  → SimulationHub 广播
  → 观测入有界录制队列
  → 独立线程按帧序写盘
```

控制路径中不再出现：

- 同步打开 JSONL；
- 每条动作读取运行时状态文件；
- 恒定无意义的控制字段；
- 可复用的重复矩阵分解；
- 空闲时不必要的 30 Hz 业务消息。

## 9. 建议实施顺序

### 第一批：P0，实时性与死状态

1. 为 `EpisodeRecorder` 增加有界队列和独立写线程。
2. 将 `record_action()` 移到 `publish_action()` 之后。
3. 将 observation 录制改为非阻塞入队。
4. 删除 `SimulationHub._latest_action`。
5. 为录制队列增加溢出、顺序、flush 和 Episode 完整性测试。

### 第二批：P1，状态与协议清理

1. 将 `SceneRuntimeManager.status()` 改为内存缓存。
2. 决定 `tracking_scale` 和跟踪误差字段的正式状态。
3. 发布或删除奇异值诊断字段。
4. 设计 `space-arm-control/2`，移除 `gripper_velocity_rad_s`。
5. 降低空闲动作和 ACK 频率，但保留运动高频流和失联保护。

### 第三批：P2，数值与命名整理

1. 复用奇异值计算 Rank。
2. 重命名 capability 和 ModelTag。
3. 标记或迁移旧 `inverse_velocity()`。
4. 合并重复关节初始化。

## 10. 验证矩阵

| 修改 | 必须验证 |
| --- | --- |
| 录制异步化 | 动作顺序、观测顺序、相机配对、停止 flush、队列溢出 |
| 场景状态缓存 | 启动、停止、崩溃、重置、多页面所有权变化 |
| 删除 Hub 动作状态 | 重置、重连、API 状态、所有现有测试 |
| 删除 `tracking_scale` | observation 契约兼容、前端和生产分析工具 |
| 协议删除旧夹爪字段 | 旧客户端拒绝或适配、测试夹具、JSON Schema |
| 空闲心跳优化 | 首帧延迟、deadman 超时、页面隐藏、WebSocket 重连 |
| SVD 复用 | 与现有 `matrix_rank` 在正常工作区和奇异位形上一致 |
| 初始化合并 | 默认 PREGRASP、场景随机化、重置和轨道初态 |
| IK 名称变更 | capability 协商、日志查询和外部客户端兼容 |

## 11. 验收标准

清理完成后应满足：

- 控制消息下发路径中不存在同步磁盘打开和写入；
- 每条动作不再读取 `scene_runtime.json`；
- `SimulationHub` 不再维护未使用的动作副本；
- 协议不再暴露错误单位的夹爪角速度字段；
- 不再重复执行同一雅可比的 SVD；
- 空闲时控制流量和 Action 记录显著减少；
- 运动时仍维持约 30 Hz 输入更新和小于 250 ms 的失联停止语义；
- Episode 在录制队列拥塞时明确标记为不完整，而不是阻塞机械臂控制；
- 所有现有安全、重置、IK、录制和前端遥测测试通过。

## 12. 证据检索命令

```powershell
git rev-parse HEAD
rg -n 'self\._latest_action|_latest_action' backend/space_arm_platform/simulation_hub.py tests/test_scene_reset.py
rg -n 'tracking_scale|velocity_scale|minimum_singular_value|condition_number' simulation
rg -n 'matrix_rank|singular_values' simulation/serial_chain_kinematics.py
rg -n 'record_action\(action\)|publish_action\(action\)' backend/space_arm_platform/app.py
rg -n 'def _append_jsonl|path\.open' backend/space_arm_platform/recorder.py
rg -n 'state_path|user_controls_current_scene|scenes\.status' backend/space_arm_platform/scene_runtime.py backend/space_arm_platform/app.py
rg -n 'gripper_velocity_rad_s|damped_least_squares_ik|so101CartesianIkController' backend simulation contracts tests
rg -n 'setInterval\(sendAction|action_ack' frontend/app.js backend/space_arm_platform/app.py
```

## 13. 最终判断

当前控制链中真正需要立即处理的是 **R-01 至 R-04**：录制 I/O、场景文件读取和死状态。它们不影响 IK 数学正确性，但会降低控制链的实时性和可维护性。

`gripper_velocity_rad_s`、`tracking_scale`、旧 IK 名称和未发布诊断字段适合放在协议和代码整理阶段处理。

三层输入保护、三层校验、关节 PD、力矩限幅、重置代际和观测回传不应被视为冗余删除对象。它们是当前平台安全性和数据可信度的组成部分。
