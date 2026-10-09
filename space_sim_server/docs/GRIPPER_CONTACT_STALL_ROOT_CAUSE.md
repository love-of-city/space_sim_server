# 夹爪接触活动插头时卡顿的根因（2026-09-29）

## 结论

卡顿**不是 MuJoCo 碰撞算不过来**。实测单次 `MJScene::equationsOfMotion`
（即一次完整 MuJoCo 前向动力学：`mj_fwdPosition` + `mj_fwdVelocity` + `mj_fwdActuation`
+ `mj_fwdAcceleration` + `mj_fwdConstraint`）的耗时在接触前后几乎相同：

- 无接触 0.61 ms，有接触 0.56 ms（同一进程配置下重复 20 次的中位数）。

真正爆炸的是**每个调度步里 MuJoCo 被调用的次数**。`simulation/native_integration.py`
把 MJScene 交给 `svIntegratorRKF45` 自适应积分器；接触力是非光滑的，RKF45 用 4/5 阶两个解的差
估计截断误差，一旦超容差就拒绝该步并把内部步长至少缩小到 0.1 倍后重试（
`svIntegratorAdaptiveRungeKutta::integrate()` 的 while 循环），每个子步仍要 6 次完整
`equationsOfMotion`。所以 1/240 s 的一个调度步内：

| 情形 | 每个 240 Hz 调度步内的 MuJoCo 评估次数（估算） |
| --- | --- |
| 夹爪远离物体（无接触） | 约 11 次 |
| 夹爪压住插头抓取头 flex（28 个接触点） | 约 148 次 |

评估次数相差约 13 倍，单次评估耗时不变，于是 wall time 也相差约 13 倍。这就是
“一靠近/一接触就卡”的全部来源。

MuJoCo 官方例子里接触很多也不卡，是因为它们直接调 `mj_step` 做**固定步长半隐式积分**，
没有误差控制器；再多接触也只是一步一次求解，不会把积分次数成倍放大。本工程用 RKF45
是为了压住小惯量手腕在 240 Hz 固定 RK4 下的数值发散（见 `native_integration.py` 注释），
代价就是把接触的非光滑性转成了积分步长崩溃。

## 实测数据

全部为原生 Basilisk/MJScene 离线基准（`tools/profile_simulation_runtime.py`，
`--initial-state operating --motion hold`，动力学 240 Hz，1 仿真秒，排除 UE/网络/写盘）。
RTF = 仿真秒 / `ExecuteSimulation` 墙钟秒。接触模型由
`run/diagnostics/gripper-contact/generate.py` 生成：把导向插头放到该场景操作姿态的夹爪中心。

| 配置 | RTF | 单帧 p50 (ms) | 每 240 Hz 步 ms | 每步评估次数（估算） |
| --- | --- | --- | --- | --- |
| 无接触（插头后退 50 mm） | 0.568 | 55 | 6.9 | 约 11 |
| **接触，现役容差 1e-4** | **0.046** | **661** | **82.7** | **约 148** |
| 接触，绝对容差 1e-3 | 0.114 | 260 | 32.5 | 约 42–57 |
| 接触，绝对容差 1e-2 | 0.277 | 116 | 14.5 | 约 25 |
| 接触，关闭插头 flex 接触（保留刚性圆柱代理） | 0.143 | 228 | 28.5 | 约 48 |
| 接触，插头 flex `solref` 0.01 → 0.03 | 0.071 | 513 | 64.1 | 约 79 |

补充观察：

- 接触时约束行数 `nefc` 从 8 涨到 80，`ncon` 24–30；无接触时逐次评估耗时不变。
- 提高物理频率没有收益：同一接触模型在 240/480/960 Hz 下，每仿真秒墙钟时间为
  1.53 / 3.28 / 6.04 s，即成本与频率成正比。不要用“提高物理频率”来解决。
- 单独的原生探针（无轨道重力/IK/渲染）能复现单次评估成本，但复现不出 148 次/步的
  崩溃；必须用完整 teleop 基准链路才能重现现场卡顿。这说明触发条件是完整链路下的
  接触瞬态，而不是单纯的碰撞几何开销。

## 机理链

1. `MJScene` 把整机状态注册成两个 bulk state：`mujocoQpos`(40) 与 `mujocoQvel`(36)。
2. 模型里存在自由体（`cubesat_free`、`capture_target_free`、两个插头 freejoint）时，
   `MJScene::initializeDynamics()` 会把这两个状态的**相对容差置 0**，只保留全局绝对容差
   （现役默认 1e-4）。轨道位置量级 6.9e6 m 无法用相对容差，所以这个设计是必要的。
3. 手指压住 144 面 `flexcomp` 抓取头时同时存在约 28 个穿透量 -0.3…-5.9 mm 的点接触，
   接触力在 RK 各级之间不可导；4/5 阶两个解的差很快超过 1e-4。
4. `maxRelError > 1` → 拒绝该步 → `newTimeStep = max(0.9·dt·maxRelError^-1/5, 0.1·dt)`。
   该实现只有 0.1 倍的**缩小系数**下限，没有最小步长/最大迭代次数保护，于是接触越硬、
   细分越多，而每个 4.17 ms 调度步必须走完为止。
5. 每个子步 6 次完整 MuJoCo 评估，包含 flex 三角面窄相与椭圆锥约束求解 → 直接放大 wall time。

## 可选优化（按实测收益）

| 方案 | 实测收益 | 代价/风险 |
| --- | --- | --- |
| 放宽绝对容差到 1e-2（`SPACE_SIM_RKF45_ABSOLUTE_TOLERANCE=1e-2`，零代码改动） | 6.0× | qpos/qvel 是整机一个状态，臂关节容差同时放宽；抓取精度需回归 |
| 放宽绝对容差到 1e-3 | 2.6× | 同上，但幅度小得多；建议作为首选折中 |
| 关闭插头抓取头 flex 接触，只留刚性圆柱代理 | 3.1× | 抓取头凹槽/倒扣形状丢失，抓取行为要重新验收；属于模型改动 |
| 插头 flex `solref` 0.01 → 0.03 | 1.5× | 接触更软、穿透更深；不能用来掩盖机械配合问题 |
| 提高物理频率 | **无收益** | 成本随频率线性上升 |
| 接触阶段换固定步长/隐式积分器，或把插头接触从机械臂积分状态中分离 | 未测 | 需要改 `native_integration`/MJScene（上游），收益最大但工作量最大 |

## 复现

```powershell
# 1) 生成“插头位于操作姿态夹爪中心”的诊断模型（需要独立 Python MuJoCo 环境）
& <offline-mujoco-python> `
    run\diagnostics\gripper-contact\generate.py --gap 0.0

# 2) 原生基准：接触 vs 无接触，加 --eom-cost-probe 得到“每步评估次数”的分母
& <server-python> tools\profile_simulation_runtime.py `
    --adapter-root <path-to-space_sim_UE_Adapter> `
    --scene-instance run\diagnostics\gripper-contact\contact_scene.json `
    --model-path run\diagnostics\gripper-contact\contact_guide.xml `
    --output run\diagnostics\gripper-contact\native-deep-eomprobe.json `
    --duration 1.0 --motion hold --initial-state operating `
    --rkf-relative-tol 1e-4 --rkf-absolute-tol 1e-4 --eom-cost-probe

# 3) 容差扫描（同一接触模型）
    ... --rkf-absolute-tol 1e-3
    ... --rkf-absolute-tol 1e-2
```

`<offline-mujoco-python>` 与 `<server-python>` 按 [PYTHON_RUNTIME.md](PYTHON_RUNTIME.md) 选择；`<path-to-space_sim_UE_Adapter>` 用本机适配器仓库路径。

## 诊断工具与产物

- `run/diagnostics/gripper-contact/generate.py`：修正为按**运行姿态**
  （`randomization.arm_joint_position_rad`）放置插头。旧版用 home 关键帧，插头离运行夹爪
  约 0.3 m，导致此前所有原生基准 `max_contacts = 0`、测不到接触。
- `run/diagnostics/gripper-contact/native_contact_cost.py`：原生逐步成本探针，
  变体 `deep`/`light`/`noflex`/`clear`，支持 `--step-hz 240/480/960`。
- `tools/profile_simulation_runtime.py`：新增只读的 `--eom-cost-probe`
  与 `--rkf-state-abs-tol`（仅诊断；生产开关仍是环境变量
  `SPACE_SIM_RKF45_RELATIVE_TOLERANCE` / `SPACE_SIM_RKF45_ABSOLUTE_TOLERANCE`）。

## 尚未完成

- 抓不住物体的力/摩擦原因未在本文调查范围（用户已确认不是主要问题）。
- 关闭 flex 接触或放宽容差后的抓取成功率、插拔精度尚未回归验收。
