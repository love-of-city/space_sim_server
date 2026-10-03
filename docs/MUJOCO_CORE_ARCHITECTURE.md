# MuJoCo 物理核心架构：组成与迁移方案

> 2026-10-02 · 设计文档。本文给出目标架构、现状对照和迁移步骤。每个部件都标明 **已实现**（代码中已存在）或 **待实现**（仅为设计，尚无代码），请勿把"待实现"的内容当作现有能力。
> 迁移状态：阶段 0–6 已完成，阶段 2/3 的构建接入已落地（UE 端到端实测待跑），阶段 7 的文档已更新、两个旧文件的删除待确认。§6 描述的组件层现已实现，见 [simulation/assembly.py](../simulation/assembly.py)、[simulation/physics_ports.py](../simulation/physics_ports.py) 和 [simulation/components/](../simulation/components/)。
> 现行默认路径（local / MuJoCo 固定步长）见 [SIMULATION_ARCHITECTURE.md](SIMULATION_ARCHITECTURE.md)；local 后端的配置项和实测数据见 [LOCAL_DYNAMICS_BACKEND.md](LOCAL_DYNAMICS_BACKEND.md)。

## 1. 为什么改，改成什么

现在的默认路径（`--dynamics-backend basilisk`）由 Basilisk 的 MJScene 用自适应 RKF45 积分整个多体系统。接触力刚硬、不光滑，接触建立和断开时误差估计突增，积分器反复拒步重算：抓取工况下每个 240 Hz 物理步要调用 MuJoCo 约 148 次，实时因子（RTF，仿真时间 ÷ 墙钟时间）只有 0.044–0.068。放宽容差实测只能提速 2.6–6 倍，还要牺牲精度，仍远低于实时（[GRIPPER_CONTACT_STALL_ROOT_CAUSE.md](GRIPPER_CONTACT_STALL_ROOT_CAUSE.md)）：这是自适应显式积分器在刚性接触下的固有局限。

新架构的目标：

1. **MuJoCo 全权负责多体动力学**：刚体、关节、接触、约束、执行器，用 MuJoCo 自己的固定步长 `implicitfast` 积分，每个物理步算一次。
2. **Basilisk 只提供组件**：轨道参考点与星历、环境模型、导航制导控制（GNC）、敏感器和执行器模型。
3. **扩展有固定路径**：加一个 Basilisk 组件只需新增文件、在组件清单里加一行，不改核心代码。
4. **结构好理解**：每个状态只有一个积分者，每个执行器只有一个写入者，执行顺序能直接查到。

不在范围内：UE 渲染协议 `bsk-render/2` 和后端观测协议 `space-arm-control/1` 保持不变；不承诺逐帧硬实时；IK 和姿态控制算法本身不重写。

目前进度：物理核心已实现（local 后端，需用 `--dynamics-backend local` 或 `SPACE_SIM_DYNAMICS_BACKEND=local` 显式开启，默认仍是 basilisk），接触工况 RTF 已到 1.0 以上；组件框架（端口表、装配、槽位）尚未实现。

## 2. 总体结构

### 2.1 三条规则

1. **一个状态只有一个积分者。** 多体状态只由 MuJoCo 积分，轨道参考点 O 只由 Basilisk 积分，没有第二份被积分的副本。
2. **物理和控制数据只经 Basilisk 消息传递。** 消息是组件之间唯一的数据通道：组件不靠调用其他组件的方法来交换物理量或指令，也不在运行中改物理状态。汇总遥测时可以直接调用组件的 `telemetry()`。
3. **力可叠加，指令独占。** 同一刚体可以有多个外力来源（阻力、光压、推力器），由物理核心求和；每个执行器命令、每个伺服参考只能有一个写入者，出现第二个时在装配阶段报错。

### 2.2 分层

| 层 | 职责 | 现有组成 | 状态 |
|---|---|---|---|
| 物理核心 | 积分全部多体动力学 | `LocalMujocoStepper` + 原生库 `local_mujoco_stepper` | 已实现，非默认 |
| 状态发布层 | 把物理状态写成标准 Basilisk 消息 | MJScene（只发布、不积分） | 已实现 |
| 轨道与环境层 | 星历、轨道参考点 O、环境模型 | SPICE 接口、质点航天器 `localFrameOrigin` | 星历和 O 已实现；环境模型待接入 |
| 器件层 | 敏感器：真值 → 测量；执行器模型：指令 → 物理输入 | 真值导航 `simpleNav`、轮驱动 `WheelDrive` | 部分实现 |
| 控制层（GNC） | IK 与关节参考、姿态导航制导控制、将来的自主策略 | `CartesianIkControlModel`、`HeldJointReferencePublisher`、`AttitudeControl` | 已实现 |
| 外部接口层 | UE 渲染、后端观测、遥操作指令、记录 | `BasiliskRenderBridge`、`AuthoritativeObservationModel`、`SimulationControlClient` | 已实现 |
| 装配层 | 按组件清单创建模块、接线、排序 | 现写在 `teleop_grasp_unreal._run_session` 里 | 待实现（§6） |

### 2.3 状态归属

| 状态 | 唯一写入者 | 主要读者 |
|---|---|---|
| 全部刚体和关节的位置、速度 | 物理核心 | 状态发布层 |
| 轨道参考点 O 的惯性位置、速度 | 质点航天器 `localFrameOrigin`；重基时由物理核心平移 | 物理核心 |
| 刚体、site、关节状态消息 | 状态发布层，由物理核心触发：关节消息每步写，刚体和 site 消息每 `stride` 步写（§5.3） | 控制层、器件层、外部接口层 |
| 伺服请求/实际出力消息 | 物理核心 | IK 的参考调节与诊断 |
| 关节参考（位置、速度） | `HeldJointReferencePublisher`；场景脚本模式下为 `JointTrajectoryPublisher` | 物理核心 |
| 轮电机命令 | 轮驱动 `WheelDrive` | 物理核心 |
| 刚体外力 | 环境模型、执行器模型（目前尚无接入者） | 物理核心 |
| 行星状态 | SPICE 接口 | O 的轨道、渲染桥、环境模型 |
| 初始和复位状态 | 初始化代码，经 MJScene 的 setter 写入；物理核心下一步采纳 | — |

## 3. 坐标系、单位与时间

### 3.1 坐标系

| 记号 | 定义 | 出现在哪里 |
|---|---|---|
| N | J2000 惯性系，原点在地心（SPICE `referenceBase="J2000"`，`zeroBase="Earth"`） | **所有对外消息** |
| O | 轨道参考点：Basilisk 质点航天器，在地球和太阳引力下推进 | 物理核心内部 |
| L | 原点为 O、各轴与 N 平行的平移系；MuJoCo 的世界系，重力为零 | 物理核心内部 |

MuJoCo 只在 L 中计算，因为它的碰撞检测在轨道半径（约 6.9e6 m）的坐标下不可靠，接触会时有时无；离原点 3e5 m 以内结果精确。发布时换回 N：自由刚体的 `r_N = r_O + r_L`、`v_N = v_O + v_L`，其余坐标（姿态、铰链和滑动关节）不变。组件看到的永远是 N，不需要知道 L。

O 本身做自由运动，所以 L 中只剩地球潮汐项 `F_i = m_i · μ/r³ · (3·r̂r̂ᵀ − I) · x_i`：r、r̂ 是地心到 O 的距离和方向，`x_i` 是刚体质心在 L 中的位置。它由物理核心作为外力施加。太阳的潮汐项约为地球项的 3e-8，忽略。系统质心离 O 超过 50 m 时（每秒检查一次），把 O 平移到质心（重基），惯性状态不变。

### 3.2 单位与表示

- SI 单位。转动关节为 rad，滑动关节（手指）为 m。
- 刚体与 site 状态 `SCStatesMsgPayload`：`r_BN_N` [m]、`v_BN_N` [m/s]、`sigma_BN`（MRP）、`omega_BN_B` [rad/s，体轴]。
- 关节 `ScalarJointStateMsgPayload.state`：位置消息为 rad 或 m，速度消息为 rad/s 或 m/s。
- 执行器 `SingleActuatorMsgPayload.input`：N·m（转动）或 N（滑动、推力）。
- 外力 `CmdForceInertialMsgPayload.forceRequestInertial` [N]：N 轴分量，作用在刚体质心。外力矩 `CmdTorqueBodyMsgPayload.torqueRequestBody` [N·m]：刚体体轴分量。力不过质心时，偏心产生的力矩由发出方算进力矩里。
- MJCF 中四元数顺序为 wxyz；Basilisk 消息里姿态用 MRP。

### 3.3 时间网格

- 仿真时间是 Basilisk 的整数纳秒。物理任务 `graspTask` 第 k 步的时刻是 `tick_time_ns(k, 240)`（[sampling.py](../backend/space_arm_platform/sampling.py)）。遥操作入口安装的 `RationalPhysicsClock` 每步重设下一步的周期，长时间运行也不漂移。单独调用 `_build_simulation` 时（场景脚本、单元测试）没有这个时钟，`graspTask` 按模块常量 `TIME_STEP` 的固定周期运行。
- 物理核心和 MJScene 写出的每条消息，`timeWritten()` 都等于它所携带状态的时刻。姿态任务的输出例外（§4.3）。
- 其他频率都应是 240 Hz 的整数分频，并在物理步上执行：IK 和关节参考默认 120 Hz（每 2 步），渲染和观测 30 Hz（每 8 步）。姿态控制的 100 Hz 是现存例外（§4.3）。

### 3.4 采样与保持

物理核心在第 k 步把系统从 t(k−1) 积分到 t(k)，因此：

- 在物理核心**之前**执行的模块，读到的关节消息是 t(k−1) 的。它们写出的命令、参考和外力在 [t(k−1), t(k)] 内保持不变（零阶保持）。
- 在物理核心**之后**执行的模块，读到的关节消息是 t(k) 的。
- 刚体和 site 消息只在发布步更新（默认偶数步，§5.3），读到的是最近一个发布步的状态：在物理核心之前读，是 t(k−1) 或 t(k−2)；在物理核心之后读，发布步上是 t(k)，其余步是 t(k−1)。分频模块的执行相位要与发布步对齐，规则见 §6.3。
- 低频模块的输出保持到它下一次执行。

## 4. 执行顺序

### 4.1 一个物理步内发生什么

```text
第 k 步（t(k-1) → t(k)），graspTask，240 Hz
 1. 时钟                  校验时刻，设置下一步周期
 2. 星历、轨道参考点 O     推进到 t(k)
 3. 环境模型（预留）       读上一步的状态，算环境量和外力
 4. IK、关节参考           读 t(k-1) 的关节状态（默认每 2 步执行一次）
 5. 控制（预留）           GNC 算期望力矩等
 6. 执行器模型             轮驱动：期望力矩 → 限矩、限速后的电机命令
 7. 物理核心               保持 3-6 的输入，MuJoCo 从 t(k-1) 积分到 t(k)，发布 t(k) 状态
 8. 敏感器（预留）、记录器  读本步的状态
 9. 渲染桥、观测快照       每 8 步发一帧 t(k) 状态
```

"上一步""本步"的状态对刚体和 site 消息只在发布步成立，见 §3.4。

### 4.2 graspTask 的优先级

Basilisk 同一任务内优先级数值大的先执行。"当前模块"一列是 local 后端的现有值；"段"是给新组件预留的范围，作为约定由 `Slot` 实现（待实现，§6.3）。

| 段 | 优先级 | 当前模块（优先级） | 频率 |
|---|---|---|---|
| 时钟 | 20000 | `RationalPhysicsClock` | 240 Hz |
| 星历与轨道 | 19999 ~ 17000 | SPICE（19000）、原点 O（18000） | 240 Hz |
| 环境 | 16999 ~ 12000 | 预留 | — |
| 指令与参考 | 11999 ~ 9000 | IK（10000）、关节参考（9999）；场景脚本模式下为脚本轨迹（9000） | 120 / 120 Hz；脚本轨迹每步 |
| 控制（GNC） | 8999 ~ 3000 | 预留；姿态控制目前在独立任务（§4.3） | — |
| 执行器模型 | 2999 ~ 1001 | 轮驱动组（2000） | 240 Hz |
| **物理核心** | **1000** | `LocalMujocoStepper`（含状态发布） | 240 Hz |
| 敏感器 | 999 ~ 1 | 预留 | — |
| 记录 | 0 ~ −9999 | 状态与出力记录器（−1，仅记录模式） | 每步 |
| 外部接口 | ≤ −10000 | 渲染桥（−10000）、观测快照（−10001） | 30 Hz |

MJScene 不在 `graspTask` 中：它放在只在 t=0 执行一次的任务 `localScenePublication` 里，之后由物理核心每步调用它发布消息。

### 4.3 姿态控制任务（现存例外）

姿态控制在独立任务 `sarmAttitudeTask` 中以 100 Hz 运行，任务优先级 −10（与 `graspTask` 同时到期时后执行）。任务内顺序：初始参考锁存（1000）→ 真值导航 `simpleNav`（900）→ `inertial3D`（800）→ `attTrackingError`（700）→ 轮速转换（600）→ `mrpFeedback`（500）→ `rwMotorTorque`（400）→ 数组转单通道（300）。轮驱动（限矩、限速保护）不在这个任务里，而是在 `graspTask` 中每步执行。

100 Hz 不是 240 Hz 的整数分频，两者每 50 ms 才对齐一次，其余时刻姿态任务夹在两个物理步之间执行。导航读的本体质心消息默认每 2 步发布一次，读到的状态最旧约 6.7 ms；轮速取自每步发布的关节消息，最旧约 3.3 ms。这对控制律影响很小，但不符合 §3.3 的规则，而且导航输出的时间戳是姿态任务的时刻，不是所用状态的时刻。另外 `load_settings()` 仍按旧的 500 Hz（2 ms 步长）校验姿态频率。迁移计划第 4 阶段处理（§10）。

### 4.4 主循环

`teleop_grasp_unreal.py` 的外层循环每次用 `ConfigureStopTime` + `ExecuteSimulation` 推进一个 30 Hz 渲染帧（8 个物理步），在帧之间处理复位请求、流控和网络。帧内所有模块由 Basilisk 调度器按上表执行，Python 层不逐步编排。复位时整个会话（模块图）销毁重建。

## 5. 物理核心（已实现）

### 5.1 组成

- [simulation/local_mujoco_stepper.py](../simulation/local_mujoco_stepper.py)：`LocalMujocoStepper`（Basilisk `SysModel`）、`OrbitOrigin` / `SpacecraftOrbitOrigin`、`ServoSpec`、后端选择函数。
- [native/local_mujoco_stepper.cpp](../native/local_mujoco_stepper.cpp)：C 接口原生库，绑定 Basilisk 已加载的 `mujoco.dll`；进程中不导入 Python 的 `mujoco` 包。构建：`python tools/build_native_acceleration.py --component local_mujoco_stepper --mujoco-include <include 根目录>`，该目录下须有 `mujoco/mujoco.h`，且版本必须是 MuJoCo 3.7.0（与 Basilisk 内置的一致）。源码改动后，旧 DLL 会被哈希清单拒绝加载。
- 独立的 `mjModel`：从同一份 MJCF 编译，积分器 `implicitfast`，重力为零，关闭自动复位。

### 5.2 输入

| 输入 | 消息 | 处理 |
|---|---|---|
| 伺服参考 | 每个伺服一对 `ScalarJointStateMsg`（位置、速度） | 在 MuJoCo 内实现为位置伺服：`力 = clip(kp·(q_ref − q) + kd·(q̇_ref − q̇), ±limit)`，阻尼项隐式积分 |
| 执行器命令 | MJScene 中该执行器的 `actuatorInMsg`（`SingleActuatorMsg`） | 所有非伺服的 MJCF 执行器（如轮电机）每步读一次，步内保持 |
| 刚体外力 | `CmdForceInertialMsg` 和/或 `CmdTorqueBodyMsg` | `add_body_wrench_input(body, ...)`；目前每个刚体只允许一个来源（§6.4 改为可叠加） |
| 原点 O | `OrbitOrigin.state(t)` | L 与 N 的换算、潮汐项 |

哪些执行器是伺服以及 kp、kd、限幅，目前写在 Python 中：`scenario_sarm_grasp.py` 的 `KP` / `KD` / `TORQUE_LIMITS`，遥操作入口再覆盖机械臂六轴的值。

### 5.3 输出

每个物理步：

- **总是写**：MJScene 的整体 qpos/qvel 状态、全部关节位置和速度消息、场景状态消息 `MJSceneStateMsg`、伺服的请求/实际出力（`requestedOutMsgs` / `appliedOutMsgs`）。
- **每 `stride` 步（默认 2）以及每个渲染步（每 8 步）写**：刚体和 site 状态消息（MJScene 的 `postIntegration` 做一次正运动学）。

`stride` 只影响发布，不影响积分结果（有测试覆盖）。

### 5.4 MJScene 的角色

local 后端下 MJScene 不积分（`isDynamicsSynced = True`），只负责发布。它是从同一 MJCF 构建的只运动学副本：关闭接触和约束，去掉只用于碰撞的刚性 flexcomp。qpos/qvel 布局、几何和 site 不变，布局在启动时校验；按设计质量分布也不变，但这一点目前没有启动校验。初始化和复位代码经 MJScene 的 setter（`setPosition`、`setVelocity` 等）写入的状态，物理核心在下一步检测到并采纳。保留 MJScene 的理由见决策 D2（§11）。

### 5.5 限制

- **执行器必须在 MJCF 里声明。** 物理核心从 MJCF 文件编译自己的模型。它的执行器都必须在 MJScene 中存在，否则启动报错；反过来不校验：经 `MJScene.addForceActuator` 等 API 添加的执行器会被静默忽略（阶段 2 补上反向校验）。
- 不支持执行器激活状态（`act`），遇到时直接报错。
- 经 `MJScene.AddModelToDynamicsTask` 挂上的模块，其 `UpdateState` 在 local 后端下永远不会被调用，因为 MJScene 不积分。
- **动量守恒只有一阶精度。** 机械臂运动时，线动量误差在子步 1 / 2 / 4 下为 1.9e-4 / 9.3e-5 / 4.7e-5 kg·m/s，质心角动量误差为 9.0e-5 / 4.5e-5 / 2.2e-5 kg·m²/s；运动停止后不再累积。场景脚本为 RK4 设定的门槛是线动量 5e-5、角动量 1e-5：子步 4 时线动量达标，角动量仍超标；按一阶收敛估算，角动量达标约需 10 个子步。
- **实时是平均意义上的。** 在共享机器上仍有 5–30% 的 30 Hz 帧超过 33 ms。

## 6. 组件与装配

本节描述的机制已经实现（阶段 4），代码在 [simulation/assembly.py](../simulation/assembly.py)、[simulation/physics_ports.py](../simulation/physics_ports.py) 和 [simulation/components/](../simulation/components/)。§6.4 列出的核心改动也已完成，只保留"按需"的加速度输出一项。

### 6.1 组件

组件是一组 Basilisk 模块，加上它们的消息接线和所在的槽位、频率。例如"姿态控制"组件包含导航、制导、控制、分配和轮驱动几类模块。每个组件只实现一个方法：

```python
class Component(Protocol):
    name: str

    def install(self, ctx: AssemblyContext) -> None:
        """创建模块，经 ctx.ports 接线，经 ctx.add 放进槽位。"""
```

组件不保存跨会话状态：每次复位都会重建整个会话，`install` 会被再调用一次。

### 6.2 物理端口表 `PhysicsPorts`

组件不直接访问 MJScene 或物理核心对象，所有物理消息都从端口表获取。端口表按 MJCF 名称索引，负责检查写入规则。

```python
class PhysicsPorts:
    # 读端口：可以有任意多个读者
    def body_state(self, body: str) -> SCStatesMsg: ...         # 刚体坐标原点
    def body_com_state(self, body: str) -> SCStatesMsg: ...     # 刚体质心
    def site_state(self, site: str) -> SCStatesMsg: ...
    def joint_state(self, joint: str) -> ScalarJointStateMsg: ...    # 位置
    def joint_rate(self, joint: str) -> ScalarJointStateMsg: ...     # 速度
    def scene_state(self) -> MJSceneStateMsg: ...
    def servo_effort(self, actuator: str) -> tuple[SingleActuatorMsg, SingleActuatorMsg]: ...  # (请求, 实际)
    def origin_state(self) -> SCStatesMsg: ...                  # 轨道参考点 O
    def planet_state(self, name: str) -> SpicePlanetStateMsg: ...

    # 写端口
    def drive_actuator(self, actuator: str, command: SingleActuatorMsg, *, owner: str) -> None: ...
    def drive_servo(self, actuator: str, position: ScalarJointStateMsg,
                    velocity: ScalarJointStateMsg, *, owner: str) -> None: ...
    def add_body_wrench(self, body: str, *, source: str,
                        force_inertial: CmdForceInertialMsg | None = None,
                        torque_body: CmdTorqueBodyMsg | None = None) -> None: ...
```

| 端口 | 对应的现有写法 | 规则 |
|---|---|---|
| `body_state` | `scene.getBody(b).getOrigin().stateOutMsg` | 只读；每 `stride` 步更新 |
| `body_com_state` | `scene.getBody(b).getCenterOfMass().stateOutMsg` | 只读；每 `stride` 步更新 |
| `site_state` | `scene.getSite(s).stateOutMsg` | 只读；每 `stride` 步更新 |
| `joint_state` / `joint_rate` | `scene.getBody(所属刚体).getScalarJoint(j).stateOutMsg` / `.stateDotOutMsg` | 只读；每步更新 |
| `scene_state` | `scene.stateOutMsg` | 只读；每步更新 |
| `servo_effort` | `stepper.requestedOutMsgs[i]` / `stepper.appliedOutMsgs[i]` | 只读；每步更新 |
| `origin_state` | `orbit_reference.scStateOutMsg` | 只读；初始化时可能滞后一次调用，重基后到下一步才更新（物理核心因此直接读质点航天器的状态对象）。环境模型应优先读刚体状态 |
| `planet_state` | `ephemeris.planetStateOutMsgs[i]` | 只读 |
| `drive_actuator` | `scene.getSingleActuator(a).actuatorInMsg.subscribeTo(msg)` | **独占**；不能用于伺服执行器 |
| `drive_servo` | 构造 `ServoSpec` 传给物理核心 | **独占** |
| `add_body_wrench` | `stepper.add_body_wrench_input(...)` | **可叠加**；按 `source` 区分，物理核心求和 |

basilisk 后端下，端口表只保证读端口和 `drive_actuator` 可用。

### 6.3 装配上下文与槽位

```python
class Slot(IntEnum):
    """graspTask 内各段的最高优先级；数值大者先执行（见 §4.2）。"""
    CLOCK = 20_000
    ORBIT = 19_999        # 星历、轨道参考点 O
    ENVIRONMENT = 16_999
    COMMAND = 11_999      # IK、关节参考、脚本轨迹
    CONTROL = 8_999       # GNC
    ACTUATOR = 2_999      # 执行器模型
    PHYSICS = 1_000       # 只放物理核心
    SENSOR = 999
    RECORD = 0
    OUTPUT = -10_000      # 渲染桥、观测快照


class AssemblyContext:
    simulation: SimBaseClass
    ports: PhysicsPorts
    clock: RationalPhysicsClock
    config: SessionConfig     # 后端、子步、发布间隔、伺服配置、模型路径……

    def add(self, model, slot: Slot, *, every: int = 1, phase: int | None = None) -> None:
        """把模块放进槽位，在 step_index % every == phase 的物理步执行（频率 = 240 Hz / every）。"""

    def keep_alive(self, *objects) -> None: ...

    def describe(self) -> list[dict]:
        """返回最终执行顺序：任务、优先级、模块、频率、所属组件。"""
```

要点：

- 同一段内按注册顺序分配优先级，组件不需要自己挑数字。
- `every > 1` 时由一个分频包装模块代为调用子模块的 `SelfInit`、`Reset` 和 `UpdateState`。Basilisk 的 C++ 模块经 SWIG 暴露了这些方法；子模块没有直接挂在任务上，`InitializeSimulation` 不会初始化它们，所以包装模块必须转发这三个调用。现有的 IK 和关节参考发布器是在各自的 `UpdateState` 里按 `clock.step_index % stride` 自行跳过：判据相同，机制不同，阶段 2 统一。
- **执行相位与发布步对齐。** 刚体和 site 消息只在 `step_index % stride == 0` 的步末发布（§5.3）。`phase` 默认由槽位决定：物理核心之前的模块取 1，即在发布步的下一步执行，读到 t(k−1) 的新鲜状态；物理核心之后的模块取 0，即在发布步上执行，读到 t(k)。为此 `every` 必须是 `stride` 的整数倍，否则装配报错；或者把发布间隔设为 1，代价是正运动学发布从每 2 步一次变成每步一次（每次约 0.6 ms）。只读关节消息的模块不受相位影响。阶段 2 迁移现有模块时显式保留它们现在的相位（IK 和关节参考为 0），保证行为不变。
- `describe()` 的结果在启动日志中以一条 JSON 输出（`"type": "module_schedule"`），同时作为 golden 测试：执行顺序随时可查，改动可审。
- 会话按组件清单装配：遥操作入口、基准工具和测试共用同一个装配函数（基准工具目前靠替换入口里的函数来复用遥操作的装配）。

### 6.4 物理核心需要的改动

1. 刚体外力可叠加：同一刚体允许多个来源，物理核心每步求和。
2. 伺服参考在构造之后连接（供 `drive_servo` 使用）；首次 `Reset` 时检查每个伺服都已连接。
3. 启动时双向校验执行器集合：现在已经要求物理核心模型的执行器在 MJScene 中都存在，还要再检查 MJScene 里没有物理核心不认识的执行器。同时逐刚体比较两份模型的质量和质心（§5.4）。
4. （按需）加速度输出。按仓库里留存的 Basilisk 源码副本（[run/diagnostics/rkf45-source/MJSite.cpp](../run/diagnostics/rkf45-source/MJSite.cpp)），MJSite 只填写 `r_BN_N`、`v_BN_N`、`sigma_BN`、`omega_BN_B`；`nonConservativeAccelpntB_B`、`omegaDot_BN_B`、`TotalAccumDV_BN_B` 保持为零，`imuSensor` 直接接上会读到零（副本与安装版本是否一致待确认）。要接 IMU，需由物理核心补发加速度。

### 6.5 接入规则

1. **机械要素进 MJCF**：质量、惯量、自由度、几何、作用点（site）、执行器声明。Python 和 JSON 中不重复这些量。器件的控制参数放在 MJCF 旁的 JSON 中，参照 `attitude_control.json`。
2. 组件只读端口表中的消息，只写自己声明过的输入端口。
3. 运行期间不写物理状态。状态只能在初始化和复位时经 MJScene 的 setter 写入。
4. 不使用 `MJScene.AddModelToDynamicsTask`，也不使用 MJScene 的执行器添加 API。
5. 频率用 `every` 表示为 240 Hz 的整数分频，读刚体或 site 消息的模块还要求 `every` 是发布间隔的整数倍（§6.3）；不新建周期与 240 Hz 不整除的任务。
6. 每个组件附测试（§8.7）和基准结果（§8.8）。

## 7. 哪些 Basilisk 组件可以直接用

| 类别 | 例子 | 能否直接用 | 接法 |
|---|---|---|---|
| 飞行软件算法（`fswAlgorithms`） | `inertial3D`、`attTrackingError`、`mrpFeedback`、`rwMotorTorque`、`thrForceMapping`、`thrFiringSchmitt` | 能 | 纯消息进出；放 CONTROL 段 |
| 环境模型 | `exponentialAtmosphere`、`magneticFieldWMM`、`eclipse` | 能 | 读 `body_com_state` 和 `planet_state`（都在 N 中）；放 ENVIRONMENT 段 |
| 敏感器 | `simpleNav`（已在用）、`starTracker`、`magnetometer`、`coarseSunSensor` | 大多能 | 读刚体或 site 状态；放 SENSOR 段。`imuSensor` 见 §6.4 第 4 条 |
| 航天器动力学效应器 | `thrusterDynamicEffector`、`reactionWheelStateEffector`、`extForceTorque`、`dragDynamicEffector`、`facetSRPDynamicEffector`、`MtbEffector` | **不能** | 这些模块挂在 Basilisk 的 `spacecraft.Spacecraft` 上，参与它的运动方程，接不到 MuJoCo。做法：机械部分进 MJCF（例如反作用轮 = 铰链刚体 + 电机），力学规律写成执行器模型（§8.3）或刚体外力（§8.2） |
| MJScene 专用模块 | `MJJointPIDController`、`MJStochasticAtmDensity` 等 | 挂在 MJScene 动力学任务上的不会执行 | 按需改写后放进 `graspTask` |

现有的反作用轮就是"效应器改写"的样例：轮子是 MJCF 里的铰链刚体和电机，Basilisk 只提供控制链（`mrpFeedback` → `rwMotorTorque`），轮驱动负责限矩和限速。

## 8. 扩展指南

### 8.1 固定步骤

1. 确定类别：环境、执行器、敏感器、控制器或外部接口；对应的槽位见 §4.2。
2. 需要的机械要素写进 MJCF：site、执行器、刚体。
3. 新建 `simulation/components/<名称>.py`，实现 `install(ctx)`。
4. 在会话的组件清单中加一行。
5. 按 §8.7 写测试，按 §8.8 跑基准。

下面示例中的代码是示意：组件框架尚未实现，Basilisk 模块的参数以其文档为准。

### 8.2 环境力：大气阻力

```python
class AtmosphericDrag:
    name = "atmospheric_drag"

    def install(self, ctx):
        bus = ctx.ports.body_com_state("cubesat_bus")
        atmosphere = exponentialAtmosphere.ExponentialAtmosphere()     # Basilisk 环境模型
        atmosphere.addSpacecraftToModel(bus)
        atmosphere.planetPosInMsg.subscribeTo(ctx.ports.planet_state("earth"))
        drag = PlateDrag(area_m2=0.06, drag_coefficient=2.2)            # 自写 SysModel：ρ、v → F_N、τ_B
        drag.atmosphereInMsg.subscribeTo(atmosphere.envOutMsgs[0])
        drag.stateInMsg.subscribeTo(bus)
        ctx.ports.add_body_wrench("cubesat_bus", source=self.name,
                                  force_inertial=drag.forceOutMsg, torque_body=drag.torqueOutMsg)
        ctx.add(atmosphere, Slot.ENVIRONMENT, every=24)   # 10 Hz
        ctx.add(drag, Slot.ENVIRONMENT, every=24)
```

- Basilisk 的 `dragDynamicEffector` 不能直接用（§7），所以阻力本身写成一个小 `SysModel`。速度要用相对大气的速度；压心不在质心时，把 `r_cp × F` 算进体轴力矩。
- 外力在两次执行之间保持，10 Hz 对阻力足够。

### 8.3 执行器：推力器

在 MJCF 中声明推力点和执行器。MuJoCo 在 site 处沿其 z 轴施力，反作用力矩自动计入：

```xml
<site name="thr1_site" pos="0.10 0 -0.05" zaxis="0 0 1"/>   <!-- 写在 cubesat_bus 刚体内 -->
<actuator>
  <general name="thr1" site="thr1_site" gear="0 0 1 0 0 0"
           ctrllimited="true" ctrlrange="0 1.0"/>         <!-- 推力 0~1 N -->
</actuator>
```

- 物理核心把它当作普通执行器命令，每步读取，核心不用改。接入时先用单元测试确认 MJScene 为这个 site 传动的执行器生成了 `actuatorInMsg`。
- 推力器模型（自写 `SysModel`）把 Basilisk `thrFiringSchmitt` 给出的开机时间换算成每个物理步的推力（开关、最小冲量、上升沿），经 `ctx.ports.drive_actuator("thr1", ..., owner=self.name)` 独占写入；放在 ACTUATOR 段，每步执行。
- `thrForceMapping`、`thrFiringSchmitt` 放在 CONTROL 段，按需分频。
- 限制：MuJoCo 中质量固定，推进剂消耗不建模。
- 变体：磁力矩器的力矩 `m × B` 随磁场方向变化，更适合写成刚体外力矩（`add_body_wrench(torque_body=...)`），B 取自 `magneticFieldWMM`。

### 8.4 敏感器：星敏感器

- `starTracker.StarTracker()` 的 `scStateInMsg` 订阅 `ctx.ports.body_state("cubesat_bus")`，或在 MJCF 中为安装位置加一个 site 后订阅 `site_state`；放 SENSOR 段，例如 `every=24`（10 Hz）。输出 `sensorOutMsg` 供 GNC 使用。
- 敏感器在物理核心之后执行，读到 t(k) 的状态；GNC 在下一步读取，延迟一个物理步。

### 8.5 控制器：替换姿态控制律

- 只替换 CONTROL 段的模块，例如用 `mrpSteering` + `rateServoFullNonlinear` 代替 `mrpFeedback`。下游仍是 `rwMotorTorque` → 轮驱动。
- 控制器不直接写物理输入。轮电机命令由轮驱动（执行器模型）独占；控制器只给出期望力矩，限矩、限速保护不会被绕过。

### 8.6 新模型或新场景

- MJCF 需满足 §6.5 第 1、4 条；伺服和轮子的配置随模型一起放。
- 组件清单按场景给出。注意加载器、关节名、目标 site 和观测契约目前仍与 SARM 绑定（[SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) §16.2），新模型还需要处理这些。

### 8.7 测试清单

每个新组件至少覆盖：

1. **接线**：端口接上；同一执行器出现第二个写入者时装配报错。
2. **时序**：在 `describe()` 中的位置和频率正确；输出消息的 `timeWritten()` 符合预期。
3. **数值**：与解析解或参考实现对比；限幅、饱和、非有限值处理。
4. **物理一致性**：外力冲量等于系统动量变化；没有重复计力。例如地球引力已经体现在 O 的轨道和潮汐项里，不能再作为外力加一次。
5. **性能**：见 §8.8。

小模型单元测试可参照 [tests/test_local_mujoco_stepper.py](../tests/test_local_mujoco_stepper.py)（小 MJCF：本体、单连杆、轮子、球）。该测试需要先构建原生库，因此不在 CI 中运行（`scripts/ci-exclusions.json`）。

### 8.8 性能预算

实时要求每个 240 Hz 物理步平均不超过 4.17 ms。量产插头模型的六轴扫动中，仅 `mj_step` 就约 1.1–1.8 ms：用原生姿态后端时 RTF 1.08–1.16，余量只有 8–16%；用 Python 姿态后端时已低于实时（0.88–0.93）。因此：

- 新组件能低频就低频（`every`）。
- 以 240 Hz 运行的 Python 模块要测量单步开销，热点改用原生实现。
- 每个新组件合入时附基准结果：`python tools/profile_simulation_runtime.py --dynamics-backend local --posture-backend native ...`，与合入前对比。

## 9. 对外接口（保持不变）

- **UE 渲染**：渲染桥 `BasiliskRenderBridge.add_mj_scene(scene, ...)` 在装配时从 MJScene 读取刚体列表和几何信息（传入 MJCF 路径时，层级和网格元数据从 MJCF 文件解析），运行时读每个刚体的 `getOrigin().stateOutMsg`，按 30 Hz 发送 `bsk-render/2`。渲染桥在 UE 适配器仓库中，依赖 MJScene 的 API。
- **后端观测**：观测快照在每个渲染帧的同一仿真时刻读取关节消息和姿态遥测，发送 `space-arm-control/1`。
- **遥操作指令**：`SimulationControlClient` → `CartesianTeleopTarget` → IK。

## 10. 迁移计划

| 阶段 | 内容 | 验收标准 | 状态 |
|---|---|---|---|
| 0 物理核心 | local 后端 | 接触工况 RTF ≥ 1.0 | 已完成（非默认） |
| 1 验收现状 | 按后端区分动量门槛；补测绕原点角动量与停止后漂移；跑回归测试 | 门槛生效并有回归测试；已知的既有失败单独记录 | 已完成（回归测试见 §12） |
| 2 部署接入 | 把两个原生组件的构建并入部署引导；MuJoCo 3.7.0 头文件进仓库；跑一次 UE 完整链路 | 干净检出后能自动构建；渲染、观测、采集、复位全部正常 | 构建已接入；UE 完整链路待跑 |
| 3 切换默认后端 | 默认改为 local，同时默认原生姿态 | 全量测试；UE 端到端；基准矩阵 | 默认值与构建已落地；UE 端到端待跑 |
| 4 组件框架 | 实现端口表、装配上下文、槽位、`describe()` 和 §6.4 的核心改动；把现有模块改写为组件 | 行为不变（判据见下） | 已完成 |
| 5 规范多速率 | 姿态控制移入 `graspTask`，以 120 Hz（`every=2`）在 CONTROL 段执行，相位按 §6.3 对齐；修正 500 Hz 校验 | 姿态保持误差和轮速不劣于 100 Hz 基线；时序测试 | 已完成 |
| 6 扩展验证 | 按 §8 接入第一个新组件：大气阻力，可选、默认关闭 | 只新增文件、组件清单一行和 MJCF 改动；测试和基准齐全 | 已完成（`simulation/components/drag.py`） |
| 7 清理 | 删除 `architecture.py` 中未使用的编排器抽象及其测试；更新 SIMULATION / SYSTEM_ARCHITECTURE；basilisk 路径降为参考工具 | 文档与代码一致 | 已完成：`simulation/architecture.py` 与 `tests/test_architecture.py` 已删除（2026-10-02） |

阶段顺序说明：先切默认（阶段 3）再做组件框架（阶段 4），这样框架只需支持一条主路径，用户也能更早用上提速；阶段 4 的"行为不变"判据本身就能保护默认路径。

**阶段 2 的文件改动：**

- 新增 `simulation/physics_ports.py`、`simulation/assembly.py`，以及 `simulation/components/`：`orbit.py`（星历 + 原点 O）、`arm_teleop.py`（IK + 关节参考 + 伺服接线）、`attitude.py`（包装 `AttitudeControl`）、`render.py`、`observation.py`。脚本轨迹仍由场景构建器 `_build_simulation` 处理，没有单独的组件。
- 修改 `simulation/local_mujoco_stepper.py`（§6.4）；`scenario_sarm_grasp.py` 的 `_build_local_simulation` 只构建物理核心和发布层；`teleop_grasp_unreal.py` 的 `_run_session` 改为"组件清单 + 装配"；`tools/profile_simulation_runtime.py` 直接调用装配函数。
- 旧的 `BasiliskModuleRegistry` 已随 `simulation/architecture.py` 删除，其职责由 `AssemblyContext` 承担。

**阶段 2 的"行为不变"判据：** 迁移现有模块时保留它们现在的优先级和执行相位。改造前后 `describe()` 给出的模块顺序一致；同一脚本轨迹的 A/B 运行中，qpos 轨迹和消息时间戳逐位一致（至少差 ≤ 1e-12）；全量测试通过；RTF 无明显变化。阶段 2 要改入口文件，影响面最大：先在新文件里完成框架和组件，再一次性切换入口。

**阶段 4 的说明：** 120 Hz 时导航在发布步的下一步执行，读到的本体状态固定是上一步的（约 4.2 ms），比现在的 0–6.7 ms 更确定。80 Hz 的周期 12.5 ms 恰好是整数纳秒，并与 240 Hz 网格每 3 步对齐，但 3 不是默认发布间隔 2 的倍数，需要把发布间隔改为 1，不推荐。改频率会改变控制回路的离散化，需要重新确认增益。

## 11. 设计决策

**D1 主循环由 Basilisk 调度器驱动，不用 Python 编排器。**
仓库里曾有一套 Python 编排器（`simulation/architecture.py`，已于 2026-10-02 删除），包含 `SimulationOrchestrator` 以及 `SceneBackend`、`BasiliskModule` 等协议：每步在 Python 中读取全部状态，依次调用各模块，合并控制输出再推进。本架构不用它做主循环。原因：Basilisk 组件是靠消息连接的 C++ 模块，调度本身是原生的，没有 Python 开销；编排器要求每个模块包一层 Python 适配，并在每步构造完整的 `SceneState`，会给每个物理步再增加可观的 Python 开销。而 `graspTask` 里已有 7 个 Python 模块每步执行，量产模型的余量只有 8–16%。两套并行机制也会让接入者困惑。它的两条规则（力可叠加、指令独占）由端口表继承。这些抽象当时只有自身的测试在用，确认无生产引用后连同测试一起删除。

**D2 MJScene 保留为状态发布层。**
理由：它的发布逻辑是原生 C++，输出标准 Basilisk 消息；渲染桥（UE 适配器仓库）和姿态控制直接依赖它的 API；开销可接受（正运动学发布每次约 0.6 ms，默认每 2 步一次）。代价：内存中多一份只运动学模型，布局必须一致（启动校验），执行器只能在 MJCF 中声明。若将来发布成为瓶颈，或需要摆脱 MJScene API，再考虑由物理核心的原生库直接写 Basilisk 消息；那需要 Basilisk 消息头文件，并同步修改渲染桥。

**D3 轨道参考点由 Basilisk 推进。**
MuJoCo 在轨道坐标下碰撞检测不可靠；Basilisk 的引力和星历模型已经验证过。

**D4 力可叠加，指令独占。**
多个环境和执行器来源同时作用在一个刚体上是常态，应当求和；两个控制源争夺同一个执行器几乎总是配置错误，应在装配时失败，而不是运行时互相覆盖。

**D5 basilisk（RKF45）路径冻结。**
保留 `--dynamics-backend basilisk` 作为无接触工况的精度对照和回退手段；新组件只保证在 local 下可用。注意它不适合当接触精度的基准：它在轨道坐标下做碰撞检测，接触本身就会闪断。

## 12. 已知限制与未决问题

| 问题 | 影响 | 计划 |
|---|---|---|
| 动量门槛按后端区分 | 已解决：门槛是回归界限，不是物理要求 | 见 §5.5 与 §12 末的实测表 |
| 逐帧实时 | 5–30% 的帧超过 33 ms（量产插头模型） | 性能预算；持续基准 |
| 姿态控制 100 Hz | 违反整数分频规则；`load_settings()` 仍按 500 Hz 校验 | 阶段 5 |
| 伺服增益写在 Python | 配置分散 | 移到 MJCF 旁的 JSON（并入阶段 4） |
| 执行器集合只单向校验 | 经 API 添加的执行器会被静默忽略 | 阶段 4 |
| 只运动学副本的质量分布未校验 | 若被去掉的 flexcomp 带质量，发布的质心状态会有偏差 | 阶段 4 增加启动校验 |
| IMU 加速度字段 | 按源码副本这些字段为零，`imuSensor` 会读到零 | 需要时由物理核心补发 |
| 推进剂质量不变 | 推力器长时间工作后质量特性有偏差 | 暂不支持 |
| 太阳潮汐项忽略 | 约为地球项的 3e-8 | 可接受 |

**动量门槛的实测依据**（脚本抓取，`sarm_platform.xml`，无接触，10 s；判据为 `MOMENTUM_THRESHOLDS`）：

| 后端 | 子步 | 线动量 [kg·m/s] | 质心角动量 [kg·m²/s] | 绕原点角动量 [kg·m²/s] | 停止后线漂移 | RTF |
|---|---|---|---|---|---|---|
| basilisk RKF45 | — | 1e-9 | 0 | 0 | 0 | 1.42 |
| local | 1 | 1.87e-4 | 8.98e-5 | 2.83e-5 | 1.1e-6 | 5.21 |
| local | 2 | 9.32e-5 | 4.49e-5 | 1.42e-5 | — | 5.11 |
| local | 4 | 4.65e-5 | 2.24e-5 | 7.07e-6 | — | 4.13 |

误差随子步一阶收敛（每翻倍减半）。local 的门槛取实测值的约 2.5 倍，作为回归界限；"停止后漂移"另设门槛，专门抓累积发散（真实累积在 0.5 s 尾部会到 1e-4 量级，实测残差只有 1.1e-6）。

**阶段 1 发现的本轮之前就存在的失败**（不是本次改动引入，需要另行决定是否修）：`tests/test_orbital_initial_state.py` 有 6 项失败，原因是该测试构造的 `native` 是 `SimpleNamespace`，未带 `MODEL_PATH`，而 `teleop_grasp_unreal._apply_orbital_initial_state` 在 HEAD 版本里就会调用 `initialize_free_plugs(scene, native.MODEL_PATH, ...)`。修复应在测试侧补上这个属性，或让该函数容忍缺失。

## 13. 代码索引

| 文件 | 内容 |
|---|---|
| [simulation/local_mujoco_stepper.py](../simulation/local_mujoco_stepper.py) | 物理核心、轨道参考点、伺服配置 |
| [native/local_mujoco_stepper.cpp](../native/local_mujoco_stepper.cpp) | 原生 MuJoCo 步进 |
| [scenario_sarm_grasp.py](../model/SARM/platform/scenarios/scenario_sarm_grasp.py) | 场景构建：`_build_simulation`、`_build_local_simulation`、`_load_scene` |
| [simulation/teleop_grasp_unreal.py](../simulation/teleop_grasp_unreal.py) | 遥操作入口：会话装配 `_run_session`、主循环 |
| [simulation/attitude_control.py](../simulation/attitude_control.py) | 姿态控制链与轮驱动 |
| [simulation/physics_clock.py](../simulation/physics_clock.py) | 240 Hz 有理时间网格 |
| [simulation/joint_reference_publisher.py](../simulation/joint_reference_publisher.py) | 关节参考发布 |
| [simulation/observation_capture.py](../simulation/observation_capture.py) | 观测快照 |
| [simulation/physics_ports.py](../simulation/physics_ports.py) | 物理端口表 `PhysicsPorts` |
| [simulation/assembly.py](../simulation/assembly.py) | 槽位、装配上下文、分频包装、执行顺序导出 |
| [simulation/components/](../simulation/components/) | 各组件 |
| [tools/profile_simulation_runtime.py](../tools/profile_simulation_runtime.py) | 只算物理的 RTF 基准 |
| [tests/test_local_mujoco_stepper.py](../tests/test_local_mujoco_stepper.py) | 物理核心单元测试 |
| [LOCAL_DYNAMICS_BACKEND.md](LOCAL_DYNAMICS_BACKEND.md) | local 后端配置项与实测数据 |
