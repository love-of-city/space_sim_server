# MuJoCo 固定步长物理核心与组件化装配

## 背景、范围与跨仓库关系

`RKF45` 自适应积分在接触工况下反复试错：每次步长拒绝都要重算一遍 MuJoCo 方程，抓取/插拔场景实测约 148 次评估对应 1 个 240 Hz 物理步，把仿真拖到实时以下。本次把多体动力学整体交给 MuJoCo 固定步长 `implicitfast`（local 后端），Basilisk 只保留轨道参考点 O 的推进与消息型组件。

不更改的内容：MJCF 质量/惯量/关节/接触参数、控制律与增益、相机标定、RGB 编码、UE 协议（`bsk-render/2`、`bsk-capture/1`）、`space-arm-control/1` 观测布局。

跨仓库：本次改动只依赖既有 adapter 协议，无需同步修改 `space_sim_UE_Adapter`；但运行部署必须同时更新服务端并重启，不能只替换 Python 文件。

## 改动组织

- **物理核心** `simulation/local_mujoco_stepper.py` + `native/local_mujoco_stepper.cpp`：`mj_step` 以 `implicitfast` 积分全部刚体；关节 PD 改为 MuJoCo 位置伺服；场景只发布不积分。新增按来源叠加的刚体外力通道、构造后接入伺服、以及逐刚体质量一致性校验（`lms_body_mass`）。
- **端口表** `simulation/physics_ports.py`：把 MJScene 与 stepper 的 API 收进一处。读端口任意多读者；`drive_actuator`/`drive_servo` 独占，第二个占有者直接报错；`add_body_wrench` 按来源叠加。参考点 O 经 `set_orbit_origin`/`set_stepper` 交给核心，两种装配顺序都成立。
- **装配层** `simulation/assembly.py`：`Slot` 槽位、`AssemblyContext` 装配与优先级分配、`RateDivider` 分频（转发子模块生命周期）、`describe()` 导出执行顺序。
- **组件层** `simulation/components/`：`clock`、`orbit`、`dynamics_core`、`teleop_ik`、`arm_reference`、`attitude`、`render`、`observation`、`drag`。`_run_session` 改为"组件清单 + 装配"。
- **多速率规范**：姿态控制链从独立 100 Hz 任务移入物理任务 120 Hz 分频（`every=2`，相位与状态发布对齐），导航读到固定一步前的状态；校验从已废弃的 500 Hz 改为 240 Hz 网格。
- **原生加速**：`native/`、`third_party/mujoco`（3.7.0 头文件）、`scripts/native_acceleration_runtime.ps1` 在平台启动时构建缺失/过期的 DLL，清单同时记录源码与二进制哈希；加载失败不回退、不伪装成原生运行。
- **动量验收按后端区分**：basilisk 保持严格门槛（线动量 5e-5、角动量 1e-5），local 用实测一步误差约 2.5 倍，并新增停止后不累积检查。
- **删除旧编排器**：`simulation/architecture.py` 及其测试从未被默认路径使用，随本次一并移除。
- **扩展验证**：`simulation/components/drag.py` 作为文档 §8 的第一个新组件（大气阻力，可选、默认关闭），验证只新增文件、组件清单一行即可接入。

## 验证证据与界限

- 组件层、物理核心、姿态、动量、阻力、原生加速、时钟、星历回归：本地原生环境 268 通过、5 跳过。
- 纯物理基准 `tools/profile_simulation_runtime.py`：同一量产插头模型，接触工况 local 后端 RTF ≥ 1.0。
- 端到端（含真实装配路径）：local 后端 RTF 1.06，basilisk 后端 RTF 0.33，两后端均可运行。
- 轨道正确性：`tidal_mu_m3_s2` = 3.986e14，参考点 O 半径零漂移、速度与圆轨道理论值一致。
- 装配顺序 `describe()` 导出 17 个模块，姿态链与轮驱动各一份。

界限：底部基础 CI 不含 UE/GPU 与真实渲染；逐帧硬实时仍不保证（共享机器上仍有 5–30% 的 30 Hz 帧超过 33 ms）；量化模型的实时结论仍需真实生产场景测量。动量门槛的放宽只针对 local 后端，基线为实测值，未掩盖回归。

## 风险与后续

- 默认后端与默认姿态后端均已切换；basilisk 路径保留为精度参考，仍可用 `--dynamics-backend basilisk` 选择。
- 姿态链与物理核心目前仍由场景构建器创建后交给组件接管，尚未完全由组件自建，属于后续收尾。
- 不提交日志、录制数据、账号库、机器绝对路径、构建产物或本地环境；`run/`、`logs/` 已由 `.gitignore` 排除。
