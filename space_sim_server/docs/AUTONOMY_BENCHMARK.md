# 卫星自动驾驶 Benchmark

最终交付范围和验收合同见 [最终目标](AUTONOMY_BENCHMARK_GOAL.md)。本页说明当前可用入口，以及后续实现必须遵循的数据边界。

当前提供 122 条来源需求，以及覆盖 11 条来源的 12 个可执行配置：S01 消旋、S02 姿态机动、S03 大角度机动、S09 有限时长轨道机动、S11 轨道抬升/降低（两个配置）、S14 单星轨道状态估计、D01 合作目标相对导航、D12 持续伴飞、D29 V-bar 接近、D30 R-bar 接近及 D32 合作目标保持点驻留。它们都有原生执行、内置及外部方法接口、独立评分、冻结种子批评测、离线复算，以及原平台内的登录、批次管理和结果入口。各配置的实际验收范围与结果见下文；其他任务族、真实图像和接触任务、评测数据转训练集仍在实施。目录中的 122 条需求不代表 122 个可执行环境。

## 任务目录

来源为用户最终版、止于抓取的清单，共 S26、D62、M24、T10。优先级分布为 P0 44、P1 53、P2 22、P3 3，保留来源研究优先级，不以实现成本重写优先级。

- 来源命名空间：`teacher-capture-scope-122-v1`。
- 目录格式：`spacesim-autonomy-catalog/1`。
- 数据文件：[`config/benchmarks/autonomy-catalog-v1.json`](../config/benchmarks/autonomy-catalog-v1.json)。
- 稳定任务 ID 使用语义名称，例如 `ssb.approach.waypoint-dwell` 对应本来源中的 D32。
- S、D 表中的视觉和合作标签来自原清单；M、T 表中原文未逐项列出的目标合作分类使用显式工程标签，并标记 `classification_origin`。每个可执行实例还需声明所有参与者的角色和合作程度。

前一版清单的同名编号意义不同，例如旧 D26 是翻滚捕获，当前 D26 是未知几何位姿估计。不得在没有来源命名空间的情况下合并两版结果。V0/V2 表示可建立两个不同观测版本，不表示某次任务同时需要两者。

在服务端目录，使用已有 Python 环境：

```text
python -m tools.autonomy_benchmark validate
python -m tools.autonomy_benchmark list --priority P0 --group D --vision V2
python -m tools.autonomy_benchmark list --family approach --format markdown
python -m tools.autonomy_benchmark schema
```

`validate` 返回完整目录的数量分布；筛选只作用于 `list` 的输出，不会改变目录覆盖范围。未知字段、错误来源版本、缺项、重复来源/稳定 ID、越界编号、非法标签、遗漏范围排除项都会使校验失败。`tasks` 单独列举实际注册的执行配置；`run D26` 等来源编号不会被当作可执行配置接受。

目录只描述需求，不能携带 `success`、成功率、已验收等运行结论。任务实例配置、执行后端注册和运行证据分别建模；后续界面也必须区分这三类事实。

`python tools/autonomy_benchmark.py matrix` 自动生成全部 122 条需求与注册配置的映射；加 `--format markdown` 输出可审阅表格。矩阵保留尚未注册的需求，逐配置标明视觉、合作程度、主体数、后端和任务指纹。`registered` 仅代表已注册执行映射，不等于原生验收通过；当前为 11 条来源、12 个配置，余下 111 条仍保留在矩阵中。同一矩阵随 `/api/benchmarks/catalog` 返回，界面分别显示配置数和对应来源数。

## 原生保持点任务

配置文件位于 [`config/benchmarks/tasks/d32-cooperative-gnss-v1.json`](../config/benchmarks/tasks/d32-cooperative-gnss-v1.json)。目标星在 500 km、51.6° 轨道，追踪星从约 160 m 沿轨后方出发，到达 `TNR=(-150,0,0) m` 并连续驻留 20 s。位置误差须不超过 0.75 m，旋转坐标系相对速度不超过 0.05 m/s，整个回合的距离须在 100–1000 m 内、相对速度不超过 2 m/s，最长 240 s。初态采用有界均匀扰动，完整扰动范围通过初始安全校验。

TNR 的轴序为沿轨切向、轨道法向、径向；不是 RTN。位置和速度以整星装配体 COM 为参考，物理状态在 J2000 中传播。相对速度扣除了目标轨道坐标系转动项。

- 动力学复用团队 SARM 与地面验证目标星的 113 个网格，机械臂及太阳翼处于既有远距模型的收拢构型，飞轮保留物理转子并显式输入零力矩。姿态被动演化，没有真值姿态保持控制器。
- `LocalMujocoStepper` 每 20 ms 推进 8 个内部步，即 400 Hz 积分；每个内部步使用 BSK 引力。配置包含地球 J2、太阳引力、阻力、太阳光压和固定种子的随机扰动。
- 方法每 0.5 s 接收两星独立带噪 GNSS 解及既有轨道滤波器的输出；目标估计通过合作链路提供。此配置无视觉、无测量延迟，不代表非合作导航、原始 GNSS 射频处理或真实图像任务。
- 动作为装配体 COM 上的理想 J2000 推力，模长上限 25 N。适配器根据物理姿态将其转换为实际原生本体系电机输入；这是声明的理想惯性推力模型，尚无实体推进器布局、推进剂或电能模型。结果只计算原生推力积分及其除以恒定质量得到的 `thrust_delta_v_m_s`。
- 方法只得到冻结的任务定义、质量和导航观测。物理真值单独写入评分日志，评分器不导入控制器，也不接受阶段名或 `success` 字段。当前 Python 方法协议面向受信研究代码，不构成对恶意代码的安全沙箱。

在已有 Basilisk 环境的云端服务目录执行。先确保 Git LFS 模型可用，并构建与源码一致的原生模块：

```text
python tools/assets/build_native_acceleration.py --component local_mujoco_stepper
python tools/autonomy_benchmark.py tasks
python tools/autonomy_benchmark.py spec d32-cooperative-gnss-v1
python tools/autonomy_benchmark.py schema --kind waypoint
python tools/autonomy_benchmark.py run d32-cooperative-gnss-v1 --seed 201 --output ../.tmp/waypoint-201
python tools/autonomy_benchmark.py rescore ../.tmp/waypoint-201
python tools/autonomy_benchmark.py batch d32-cooperative-gnss-v1 --split test --output ../.tmp/waypoint-test
python tools/autonomy_benchmark.py batch d32-cooperative-gnss-v1 --split test --output ../.tmp/waypoint-test --resume
```

训练种子为 1–3，验证种子为 101–103，冻结测试种子为 201–210。运行目录必须新建，单次运行不能覆盖已有结果。批次把任务、方法、源码哈希及完整种子表写入计划，每个回合使用独立进程，墙钟上限 900 s。`--resume` 只启动未尝试的样本；已有失败、取消或进程中断样本保留原结论，不自动重试。任务或源码变化需要新批次，进程仍存活时拒绝恢复。

基线 `gnss-pd-v1` 采用点质量前馈和 GNSS 估计上的 PD 控制；前馈近似与 J2 环境的差异保留在测试中。`--method no-thrust-v1` 是不推力负对照，`--method unsafe-approach-v1` 是持续朝目标推进的违规负对照。方法实现见 `simulation/benchmark_methods.py`，`reset/act/close` 输入输出协议位于 `benchmark_contract.py`。方法按任务类型注册，`tasks` 同时列出每个配置可用的方法；省略 `--method` 时使用该任务的传统基线，跨类型方法被拒绝。

每个回合保存冻结任务、源码/运行身份、观测、请求动作、原生物理证据、模型和运行时指纹、评分及文件哈希。`rescore` 核对文件哈希后从原始物理状态重新计算；不需要启动 Basilisk，也不会把取消或基础设施失败改成成功。哈希用于检查归档完整性，不是可信签名。

独立评分要求从 t=0 起每 20 ms 一条证据；缺帧、重复/倒退时间、非有限数和初始化后改写状态均导致无效证据。驻留必须连续满足位置与速度要求，硬安全违规否决成功。安全检查频率为 50 Hz COM 距离和相对速度检查，此场景不宣称进行接触验收。汇总主分母保留所有计划样本，分别列出超时、安全违规、方法失败、无效证据、基础设施失败和取消，并给出完成批次的 Wilson 95% 区间。

基础测试与原生验收分别执行：

```text
python -m pytest tests/test_benchmark_catalog.py tests/test_benchmark_execution.py tests/test_benchmark_attitude.py tests/test_benchmark_external.py tests/test_benchmark_platform.py -q
python -m pytest tests/test_benchmark_native.py -q
```

第二条在云端实际加载原生模型，检查推力和飞轮确实改变运动、飞轮加速保护与制动、复位重建物理与随机流、旧回合动作被拒绝及关闭后不能步进。普通 CI 按专用验收清单排除此测试，不能将排除或跳过称为原生通过。

### 当前验证结果

2026-11-06，云端实际运行；实现对应 `227cc66` 的代码，任务指纹为 `2a2c41936574f5f9676906f75a311fd445f7bd35fbb44f0b0aa4ab52e42f9432`。结果范围仅限上述 D32 合作、无视觉、零延迟配置。

| 验证 | 样本与结果 |
| --- | --- |
| 目录、协议、评分、重算、批次记账 | 52 项通过，包含复制模型对象绕过输入校验的回归 |
| 真实原生动作、复位与随机流 | 1 项原生测试通过，包含两次重建与不同动作的物理响应 |
| PD 基线，测试种子 201–210 | 10/10 成功；Wilson 95% 区间 72.25%–100%；完成时间 100.18–112.02 s |
| 同批次控制指标 | 最终位置误差 0.302–0.350 m；所有回合最小距离不低于 150.264 m；推力积分/质量为 0.302–0.453 m/s |
| 不推力负对照，种子 201 | 240 s 超时，最终位置误差 5.34 m，推力积分为零 |
| 持续接近负对照，种子 201 | 13.5 s 因相对速度超出 2 m/s 被否决；此时距离仍为 143.62 m |
| 独立进程中断回归，训练种子 1–3 | 主动中断种子 1 的自有进程，记录 1 次基础设施失败；后续 2 次成功；分母仍为 3 |
| 离线重算与批次恢复 | 基线及负对照重算一致；完整测试批次与中断回归恢复后，已有回合的文件哈希及修改时间均未变化，没有重新启动样本 |

测试批次逐项核对了原始模型、113 个网格、原生构建与 Basilisk MuJoCo 运行时指纹，均相同，所有完整回合的初始化后状态写入计数均为零。测试批次汇总 SHA-256 为 `a58c5ec1a4b2978d31af98f6ea289f2abcc4fda0479ea4e98847363a57548aab`。原始运行数据留在本次云端临时验证目录，源码仓库只保留实现、配置及本摘要。

## 原生单星姿态任务

配置 [`s02-wheel-attitude-v1.json`](../config/benchmarks/tasks/s02-wheel-attitude-v1.json) 对应 S02。场景只含一颗团队 SARM，保留原模型的几何、质量、收拢构型和三轴实体飞轮；移除独立目标星、目标接触对和理想平移执行器。当前模型生成复用既有远距模型构建器，因此准备阶段仍需上述两套 LFS 资产；最终物理场景只有 SARM 的 9 个网格文件，没有目标星。

初态相对给定姿态转过 25°–45°，转轴均匀取向，三轴初始角速度各在 ±0.001 rad/s 内。目标要求姿态误差不超过 0.5°、本体角速度模长不超过 0.001 rad/s，连续保持 10 s；最长 120 s。硬限制为本体角速度 0.15 rad/s、飞轮速度 628.319 rad/s、每轴电机力矩 0.2 N·m。

- 动力学仍为 400 Hz 原生积分、50 Hz 物理证据，地球 J2、太阳引力、阻力、太阳光压和有种子的扰动均保留。初始轨道状态对应整星 COM，初始化本体原点速度时扣除角速度与 COM 杆臂项。
- 方法以 50 Hz 接收带噪绝对姿态遥测、项目既有陀螺模型和理想飞轮编码器。四元数为本体到 J2000 的 `w,x,y,z`，角速度在本体系。绝对姿态噪声标准差为 0.0001 rad；陀螺包含白噪声、初始偏置、随机游走和量程判定。该配置是 V0 模拟传感器，不是星敏感器图像或视觉导航测试。
- 动作直接驱动原生飞轮关节电机，沿用团队硬件限矩和 98% 转速保护，超过保护阈值仍允许制动。没有直接给本体施加控制力矩，也没有隐藏的真值姿态控制器。
- `attitude-wheel-pd-v1` 复用团队姿态反馈增益，使用公开的装配体惯量和带噪观测计算飞轮力矩；`no-wheel-torque-v1` 与 `unsafe-spin-v1` 分别为零力矩和持续满力矩负对照。
- 评分器独立计算四元数夹角（`q` 与 `-q` 等价）和连续稳定时长，从原生电机回读计算各轴绝对力矩积分。硬安全违规、缺帧和初始化后改写状态均不能获得成功；没有电气损耗模型，不输出虚构能耗。

```text
python tools/autonomy_benchmark.py spec s02-wheel-attitude-v1
python tools/autonomy_benchmark.py schema --kind attitude
python tools/autonomy_benchmark.py schema --kind attitude-observation
python tools/autonomy_benchmark.py schema --kind wheel-action
python tools/autonomy_benchmark.py run s02-wheel-attitude-v1 --seed 201 --output ../.tmp/attitude-201
python tools/autonomy_benchmark.py batch s02-wheel-attitude-v1 --split test --output ../.tmp/attitude-test
python tools/autonomy_benchmark.py rescore ../.tmp/attitude-201
```

训练、验证和测试种子划分与 D32 相同，任务指纹为 `abd19ea93eb5122b14c3fb81f3ad4812cb8c1e8f0e3d775ef0a17bffe3467fa2`。2026-11-06 云端实测：

| 验证 | 结果 |
| --- | --- |
| 两类任务的目录、协议、评分、复算及模型派生检查 | 73 项通过；另有 2 项真实原生生命周期测试通过 |
| 姿态 PD，测试种子 201–210 | 10/10 成功；Wilson 95% 区间 72.25%–100%；完成时间 35.38–38.70 s |
| 姿态与安全指标 | 最终姿态误差 0.000711–0.000888 rad；全部回合峰值本体角速度不超过 0.09913 rad/s，飞轮速度不超过 64.982 rad/s |
| 零力矩，种子 201 | 120 s 超时；最终姿态误差 0.82838 rad，电机力矩积分为零 |
| 持续满力矩，种子 201 | 3.26 s 因本体角速度超出 0.15 rad/s 被否决 |
| 离线重算及恢复 | 十个基线和两个负对照的评分重算一致；批次恢复后 100 个回合文件的哈希和修改时间均未变，退出码保留 |
| 任务分派回归 | 重新运行 D32 测试种子 201，完整物理日志 SHA-256 与原批次该样本完全相同 |

姿态测试批次的原始模型、9 个网格及原生/BSK 运行时指纹一致，初始化后状态写入均为零。汇总 SHA-256 为 `70440b0f6f317de81df387a904087472953baaa8ed5c9e6ba37b805cec2b6967`。该结果只验收此 S02 配置，未覆盖消旋、飞轮卸载、执行器故障或视觉姿态估计。

### 入轨初始消旋与姿态稳定

[`s01-wheel-detumble-v1.json`](../config/benchmarks/tasks/s01-wheel-detumble-v1.json) 对应 S01。初始本体角速度从各轴 ±0.04 rad/s 的均匀立方体中抽样，只接受模长不小于 0.04 rad/s 的样本，因此每个初态都需要真实消旋。该分布是有模长下限的立方体分布，不是球面均匀分布。初始姿态误差为 25°–45°，目标为完成消旋、转至指定惯性姿态，并连续保持 10 s；总时限 120 s。

原生硬件、400 Hz 积分、带噪姿态/陀螺、目标容差与安全限制沿用 S02。默认方法使用限参考角速度的 `attitude-rate-limited-pd-v1`。任务加载器要求显式非零自旋分布，独立评分器还会检查首帧的真实角速度、姿态及飞轮初态；不符合初态范围的日志不能通过最终驻留获得成功。S02/S03 的任务指纹和随机初始化路径保持不变。

```text
python tools/autonomy_benchmark.py run s01-wheel-detumble-v1 --seed 1 --output ../.tmp/detumble-train-1
python tools/autonomy_benchmark.py batch s01-wheel-detumble-v1 --split test --output ../.tmp/detumble-test
python tools/autonomy_benchmark.py run s01-wheel-detumble-v1 --method no-wheel-torque-v1 --seed 201 --output ../.tmp/detumble-zero
python tools/autonomy_benchmark.py run s01-wheel-detumble-v1 --method unsafe-spin-v1 --seed 201 --output ../.tmp/detumble-unsafe
```

2026-11-06 云端冻结测试种子 201–210：10/10 成功，完成时间 34.84–40.40 s。实际初始角速度模长为 0.040159–0.053908 rad/s，最终姿态误差为 0.000712–0.000892 rad；整个评测的峰值本体角速度为 0.075024 rad/s，峰值飞轮速度为 77.869 rad/s。零力矩在 120 s 超时，结束角速度仍为 0.044938 rad/s；持续满力矩在 2.32 s 超出 0.15 rad/s 安全限制。

十个测试回合、训练种子 1、两个负对照及 S02 回归共 14 个回合独立复算一致。冻结源码下恢复批次后，100 个回合文件的哈希和修改时间未变；模型/资产/运行时指纹一致，初始化后状态写入为零。S02 种子 201 的完整物理日志仍为 `a44d6f2f45e643fdc1e0438e06b6b86f4102d4fbf38901730bc4662cfb5b652b`。加入首帧评分校验后，14 个归档回合再次复算一致；这次评分代码升级要求新建批次，不能混入此前冻结的运行计划。

S01 任务指纹为 `a65d7a880c26cfc279766b55a96f0e4b91ff318db2459116c5465979b9955f56`；云端 `.tmp/s01-test-rate-limited-v1/summary.json` SHA-256 为 `b696c0602248f101cd57dc6c63a705b62969416f47ff4a57003bb72b8b2c3fd7`。该配置只覆盖原飞轮能够处理的中等初始自旋，不代表磁力矩器消旋、飞轮卸载或执行器故障恢复。

### 大角度限时机动

[`s03-large-angle-wheel-v1.json`](../config/benchmarks/tasks/s03-large-angle-wheel-v1.json) 对应 S03，初始姿态误差为 120°–170°，要求在 75 s 内完成转向和最后 10 s 连续稳定保持。其传感器、原生飞轮、积分精度、最终姿态/角速度容差及硬安全限制与 S02 相同。75 s 是本配置明确的工程评测预算，不声称是最短时间最优值。

默认基线为 `attitude-rate-limited-pd-v1`：将姿态误差转换为目标角速度，限制该参考的模长，再用本体角速度反馈驱动实际飞轮；本配置的目标角速度上限为 0.075 rad/s。仍保留原 `attitude-wheel-pd-v1` 用于方法比较；S02 默认基线及其数值计算保持不变。加载器要求 S03 与大角度语义 ID 一致、所有初态至少转过 90°，且时间预算不超过 90 s，不能把原小角度配置更名为 S03。

```text
python tools/autonomy_benchmark.py spec s03-large-angle-wheel-v1
python tools/autonomy_benchmark.py run s03-large-angle-wheel-v1 --seed 1 --output ../.tmp/large-angle-train-1
python tools/autonomy_benchmark.py batch s03-large-angle-wheel-v1 --split test --output ../.tmp/large-angle-test
python tools/autonomy_benchmark.py run s03-large-angle-wheel-v1 --method attitude-wheel-pd-v1 --seed 201 --output ../.tmp/large-angle-original-pd
```

2026-11-06 云端冻结测试种子 201–210：限角速度基线 10/10 成功，Wilson 95% 区间 72.25%–100%，完成时间 59.06–67.96 s。最终姿态误差为 0.000719–0.000939 rad，全部回合峰值本体角速度不超过 0.075051 rad/s、飞轮速度不超过 69.268 rad/s。原 PD 在种子 201 的 2.96 s 触发 0.15 rad/s 安全限制，零力矩方法在 75 s 超时；两项失败均保留原结论。

所有正负结果独立重算一致；批次恢复后 100 个回合文件的哈希及修改时间未变，资产/原生运行时指纹一致，初始化后状态写入为零。S02 种子 201 的回归物理日志与原批次完全相同。任务指纹为 `56f10e02643e92c83b32ac8ae63a397682c3c1ea6968afe159e0c68a182ea65f`，测试汇总 SHA-256 为 `66291bba847cd8a88932bbd58647b9bd001ac3b3cfda49859e23e5bf198439db`。加入 S03 后，90 项相关自动检查及 Ruff 在云端通过。

## 原生单星轨道机动

轨道任务采用 `spacesim-autonomy-orbit/1`，只含原团队 SARM 收拢构型的单颗航天器、9 个原始网格和被动飞轮，在组合体质心增加限幅理想平移执行器。该执行器是仿真抽象，不代表已建模的具体推进器、燃料流量或姿轨联合控制。400 Hz 原生积分与 50 Hz 私有物理证据沿用原后端，初始化之后不写入位置或速度。

| 配置 | 目标及判据 |
| --- | --- |
| [`s09-finite-burn-v1`](../config/benchmarks/tasks/s09-finite-burn-v1.json) | 在 5–45 s 时间窗执行固定 J2000 矢量 Δv=(0,-2,0) m/s；实际推力积分矢量误差 ≤0.01 m/s，随后连续停推保持 5 s；总时限 60 s、累计 Δv 上限 3 m/s |
| [`s11-orbit-raise-v1`](../config/benchmarks/tasks/s11-orbit-raise-v1.json) | 从约 500 km 的圆轨道到达 501 km 圆轨道；半长轴误差 ≤10 m、偏心率 ≤2×10⁻⁶、轨道面变化 ≤10⁻⁴ rad；停推后连续满足 20 s |
| [`s11-orbit-lower-v1`](../config/benchmarks/tasks/s11-orbit-lower-v1.json) | 同一初始分布和判据，目标改为 499 km 圆轨道 |

S11 两项均限时 420 s、累计 Δv 上限 40 m/s。通用硬约束为高度 450–550 km、偏心率 ≤0.01、推力模长 ≤25 N；停推判定阈值为 0.05 N。轨道根数从实际质心状态独立计算，采用固定地球 μ=3.986004418×10¹⁴ m³/s² 的瞬时二体密切根数，地球参考半径为 6378136.6 m。瞬时到达目标高度但速度不匹配、仍在椭圆轨道上，或依靠持续推力维持位置，均不能代替上述成功条件。

初始高度均匀扰动 ±20 m，真近点角均匀扰动 ±2°，倾角 51.6°。本组配置使用地球点质量引力、太阳引力、阻力、太阳光压及 2×10⁻⁶ m/s² 随机加速度标准差。它不声称覆盖 J2 长期轨道维持、高空转移或燃料最优转移。

方法每 0.5 s 只接收本星带噪 GNSS 及轨道滤波输出，位置/速度噪声标准差分别为 0.2 m、0.002 m/s，没有目标星遥测、姿态真值或评分反馈。`gnss-orbit-maneuver-v1` 对 S09 使用标称质量和理想执行器的定时有限推力，对 S11 从初次 GNSS 估计规划平滑升降轨迹并闭环跟踪，估计进入目标轨道后停推。规划参考不赋给物理状态。评分使用原生电机力回读和真实姿态，按 50 Hz 积分推力；该积分是采样近似，输出累计 Δv 与惯性系矢量 Δv，不输出未建模的燃料或电量。

```powershell
python tools/autonomy_benchmark.py run s09-finite-burn-v1 --seed 1 --output run/benchmark/s09-train-1
python tools/autonomy_benchmark.py batch s11-orbit-raise-v1 --split test --output run/benchmark/s11-raise-test
python tools/autonomy_benchmark.py batch s11-orbit-lower-v1 --split test --output run/benchmark/s11-lower-test
python tools/autonomy_benchmark.py run s09-finite-burn-v1 --method-spec examples/autonomy_methods/orbit-pd.json --seed 201 --output run/benchmark/s09-external-201
```

`no-orbit-thrust-v1` 与 `unsafe-orbit-burn-v1` 分别提供零推力和不遵守时间窗/资源约束的持续推进负对照。`schema --kind orbit` 和 `schema --kind orbit-observation` 导出任务与观测格式。

2026-11-06，云端提交 `ec6bf79` 对应实现的冻结测试种子 201–210：

| 配置 | 成功/计划 | 完成时刻 | 终态误差 | 累计 Δv |
| --- | --- | --- | --- | --- |
| S09 有限时长机动 | 10/10 | 27.02 s | 矢量 Δv 误差 1.33×10⁻⁷–7.39×10⁻⁷ m/s | 2.0 m/s |
| S11 抬升 | 10/10 | 300.02 s | 半长轴 0.191–1.540 m；偏心率 5.52×10⁻⁷–6.49×10⁻⁷ | 13.188–13.662 m/s |
| S11 降低 | 10/10 | 300.02 s | 半长轴 0.052–1.529 m；偏心率 4.66×10⁻⁷–5.98×10⁻⁷ | 13.224–13.699 m/s |

每项 10 个样本的 Wilson 95% 区间均为 72.25%–100%，不外推至未运行的轨道或扰动。三个零推力负对照均到时限失败；S09 持续推进在首个决策周期因时间窗违规被拒绝，S11 两项持续推进均在 258.5 s 因偏心率越界被拒绝。S09/S11 抬升的独立进程方法在种子 201 上与内置方法的完整物理日志 SHA-256 相同。

30 个测试、6 个负对照、2 个外部方法及 3 个训练样本共 41 个回合均离线复算一致。三组批次在冻结源码下恢复，300 个回合文件的哈希和修改时间均不变；模型、网格、原生/BSK 运行时身份一致，初始化后状态写入均为零。该恢复检查在后续界面/导出代码更新之前完成；源代码改变后原计划会按设计拒绝继续执行，但仍可复算已有证据。

云端证据位于工作树 `.tmp/<配置 ID>-test/`，综合审计为 `.tmp/benchmark-orbit-acceptance.json`。三个测试汇总 SHA-256：S09 `1e8289585ac19951192ba4941c139b8d209bd02c614098d6fa64421ceaf218a5`；S11 抬升 `c6e291e56e07621649cec4f2c9ca6fdc65ea2082e18e7d986530fada78b3f25d`；S11 降低 `ab9a05487e7849d1b3f73373150d7d62a6f2345b91b49c57d126fdd9cf7e3bd9`。

综合审计 SHA-256 为 `2c88f57191cc4166be2a300edc8f4fd851866677fe8e06ce03a726ce1258ac50`。轨道实现通过 4 项真实原生生命周期检查；后续导出/映射更新通过 129 项相关后端检查、261 项前端检查及生产构建。

## 原生导航估计任务

[`s14-gnss-state-estimation-v1`](../config/benchmarks/tasks/s14-gnss-state-estimation-v1.json) 和 [`d01-cooperative-gnss-estimation-v1`](../config/benchmarks/tasks/d01-cooperative-gnss-estimation-v1.json) 使用 `spacesim-autonomy-navigation-evaluation/1`。前者为单颗 SARM 的绝对轨道状态估计，后者为 SARM 与合作目标的相对位置/速度估计；它们复用上述真实单星、双星物理场景。

这两项是导航技能评测：一个估计器、零颗由方法操纵的航天器，方法只提交估计值。原生平移和飞轮电机命令均保持为零；平移电机回读超过 10⁻⁸ N 即否决成功。航天器在 BSK 引力和既有环境扰动下自由传播，物理采样仍为 50 Hz、内部积分为 400 Hz。导航方法不会得到姿态真值、物理对象、实际初态或环境滤波器的后验。

输入为每 0.5 s 一次、未经环境滤波的 GNSS 解：各轴位置白噪声标准差 1 m、速度标准差 0.02 m/s，零投递延迟。D01 还通过声明的合作链路接收目标的独立 GNSS 解。这是 V0 模拟 GNSS 解，不是伪距/载波信号处理、视觉感知或非合作导航。

输出均以质心为参考点，使用 SI 单位：S14 为地心 J2000 的 `[r,v]`；D01 为 J2000 表达的 `[r_chaser-r_target, v_chaser-v_target]`，其中速度是惯性矢量差，不是旋转 TNR 坐标系的速度。动作必须匹配当前 episode、观测序号和采样时刻。数值传输保护边界为各位置分量 ±10¹² m、各速度分量 ±10⁷ m/s；这不是允许的轨道范围，超出边界作为方法契约错误处理。

整个回合长 120 s，必须对 t=0、0.5、…、119.5 s 的 240 次观测全部给出估计，最后 t=120 s 的观测只作终端快照。不存在预热排除期，也不能提前用一帧准确估计获得成功。RMSE 是三维误差模长的均方根，包含首帧与全部有效估计：

| 配置 | 位置 RMSE 上限 | 速度 RMSE 上限 | 最大位置误差 | 最大速度误差 |
| --- | --- | --- | --- | --- |
| S14 | 0.8 m | 0.02 m/s | 5 m | 0.15 m/s |
| D01 | 1.2 m | 0.03 m/s | 8 m | 0.2 m/s |

初始状态和物理安全检查独立于方法输出。首帧不符合声明分布记为 `invalid_initialization`；完整窗口的精度不达标记为 `task_failure`；重复、缺失、错序或与物理时刻不一致的证据不能通过。私有 `physics.jsonl` 同时保存原生物理帧与带 `record_type=navigation_estimate` 的方法输出，离线评分只用同一时刻的物理真值计算误差，不使用方法的成功标记。日志中的估计记录不写回仿真状态。

默认 `gnss-orbit-estimator-v1` 在方法侧运行已有六状态轨道滤波器，从第一份带噪 GNSS 解初始化，使用点质量预测模型；D01 的两颗航天器分别滤波后作差。`gnss-measurement-only-v1` 是不滤波的测量对照，`zero-navigation-estimate-v1` 是全零估计负对照。物理环境使用点质量地球、太阳引力、阻力、光压和固定种子扰动。本组不声称覆盖 GNSS 失效、延迟、J2 长时导航或分布式协同估计。

```powershell
python tools/autonomy_benchmark.py run s14-gnss-state-estimation-v1 --seed 1 --output run/benchmark/s14-train-1
python tools/autonomy_benchmark.py batch d01-cooperative-gnss-estimation-v1 --split test --output run/benchmark/d01-test
python tools/autonomy_benchmark.py run s14-gnss-state-estimation-v1 --method-spec examples/autonomy_methods/navigation-estimator.json --seed 201 --output run/benchmark/s14-external-201
python tools/autonomy_benchmark.py schema --kind navigation-task
python tools/autonomy_benchmark.py schema --kind gnss-measurements
python tools/autonomy_benchmark.py schema --kind navigation-estimate
```

训练种子 1 的原生执行已通过：S14 位置/速度 RMSE 为 0.3793 m / 0.00828 m/s，D01 为 0.3069 m / 0.01127 m/s。随后在源码 `dd80e6b` 冻结种子 201–210，未依据测试结果修改阈值或方法：

| 配置 | 成功/计划 | 位置 RMSE 范围 | 速度 RMSE 范围 |
| --- | --- | --- | --- |
| S14 绝对导航 | 10/10 | 0.2065–0.3293 m | 0.006850–0.009421 m/s |
| D01 合作相对导航 | 10/10 | 0.3580–0.5243 m | 0.007919–0.013642 m/s |

每回合均完成 120 s、6001 个物理帧与 240 个估计，实际推力积分和初始化后状态写入均为零。两组 10/10 的 Wilson 95% 区间各为 72.25%–100%；这只描述此冻结配置和方法的十个样本。

种子 201 的直接测量对照在 S14、D01 分别得到 1.7551 m、2.3724 m 的位置 RMSE，均因位置和速度 RMSE 超限记为 `task_failure`。两项零估计对照同样未达标，保留全部 240 次估计。独立进程估计器在两个任务的种子 201 上与内置方法具有完全相同的私有物理/估计日志。20 个测试、4 个对照、2 个独立进程、2 个训练及 S09/D32 两个回归回合，共 30 个回合离线复算全部一致；冻结源码下恢复两个批次，200 个原始文件的哈希与修改时间均未改变。每组原始模型、网格和原生/BSK 指纹一致。相关检查 155 项通过，包含六项真实原生生命周期检查。

- S14 任务指纹：`e9827b71e3544a1772418f8d304d81a66a1afe25b38f90e66d3b5df32d77b0c7`；批次汇总：`0c476f55cd456f5a169c92562009ab31f4151969aada398866defe1fd917be7b`。
- D01 任务指纹：`f16c23f5e9f5ffcaa4a657894c2433342ba45038345dac2471fa60a0d050160f`；批次汇总：`334d8595cbf4f2a048730e556adcfeee55a0cf9211621b6b15e646f7a7e25b66`。
- 云端证据：`.tmp/benchmark-navigation-acceptance.json`，SHA-256 `41e885a9f6a02c147f245d446e301c4bc1c7d182eeb28c618a4151ebd730011c`。原始回合位于 `.tmp/<configuration>-test` 及同名前缀的训练/对照目录。

网页验收发现深层 Windows 结果目录会使模型初始化失败，该失败保留在 `.tmp/benchmark-navigation-platform-v2`。修复后，评测输出统一采用 Windows 扩展路径，原生加载器通过 VFS 读取同一份完整 MJCF；网格仍使用原始资产的绝对路径。内存加载显式拒绝 include 和相对资产路径，其他场景仍可使用原文件加载接口。此处理对应 [Windows 路径限制](https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation)，不修改系统注册表或物理参数。需按上文命令重建原生模块。

修复后的 293 字符模型路径完成 S14 种子 201，完整物理/估计日志与修复前一致；导航生命周期检查还覆盖超过 260 字符且包含空格的目录。路径、执行、SDK、API 和原生生命周期共 79 项检查通过，另有 20 项原生动力学检查通过，其中对比了文件/内存加载的完整运动状态和电机力。S02、S09、D32 各自重新完整运行种子 201，物理日志均与先前冻结结果逐字节一致且复算一致，记录在 `.tmp/benchmark-memory-model-regressions.json`。261 项前端检查、生产构建及 Ruff 通过。原生源码 SHA-256 为 `fa6867f4cb83ada98ec68f684304e9b064cc971f902f50f2227c5c012d26a6f5`，本次 Windows DLL 为 `70946e4ffc2af678bac0291dad61fdf016df76e2b5c44790554b1456282100ae`。

实际 Edge 在同等目录深度运行 S14、D01 独立进程方法，以及 S14 零估计对照。三个结果的 JSON、CSV、私有日志下载和网页复算均通过；成功与失败日志都与对应 CLI 记录逐字节一致，失败成功率显示为 0/1，并列出未达标指标。页面使用“评测方法”和“误差指标”，区分全程 RMSE 与终态误差，无浏览器错误。证据在 `.tmp/benchmark-navigation-platform-v3`；报告 SHA-256 为 `c7533394815cce693c74dc82c4cbdd7c7ded0605b28026751f70a276bb5d08d2`。新源码不能续跑旧源码的冻结计划，应保留旧批次并创建新批次。

## 持续伴飞与轴向接近

`spacesim-autonomy-relative-flight/1` 复用真实双星场景和 GNSS 观测，但增加各来源特有的物理判据。两星参与、只控制追踪星、合作目标 C、V0；地球 J2、太阳引力、阻力、光压、带种子扰动、400 Hz 积分、50 Hz 私有记录与 2 Hz 控制沿用 D32。方法获得过滤后的两星 GNSS 状态，不获得物理真值。初态均由独立评分器核验，不能将追踪星放在终点直接取得接近成功。

| 配置 | 初始相对位置 TNR | 任务终点/持续条件 | 全程约束 |
| --- | --- | --- | --- |
| `d12-continuous-gnss-hold-v1` | (-150,0,0) m，各轴 ±0.1 m | 在同一位置连续伴飞全部 300 s | 每个物理采样的位置误差 ≤0.75 m、相对速度 ≤0.05 m/s，无进入宽限期 |
| `d29-vbar-gnss-approach-v1` | (-250,0,0) m，各轴 ±2 m | 到达 (-125,0,0) m 后驻留 10 s | 沿负 T 轴一侧接近，横向偏差 ≤8 m，实际前进至少 100 m |
| `d30-rbar-gnss-approach-v1` | (0,0,-250) m，各轴 ±2 m | 到达 (0,0,-125) m 后驻留 10 s | 沿负 R 轴一侧接近，横向偏差 ≤8 m，实际前进至少 100 m |

D12 初始 TNR 速度为 (0.015,0,0.01) m/s，各轴 ±0.002 m/s。D29/D30 为零中心、各轴 ±0.01 m/s；到达容差为 0.75 m 和 0.05 m/s，时限 240 s。全部配置保持 100–1000 m 的质心安全距离、2 m/s 的相对速度硬上限及 25 N 推力上限；此组终点仍远离物理接触区。

D12 的短暂脱离保持范围会永久记录为质量不达标，回到目标点不能抹除；缺帧或时间重复仍判证据无效。D29/D30 即使最终准确到达，只要曾违反横向走廊，也由硬安全限制否决成功。评分同时输出最长连续保持时间、最大位置误差、最大横向偏差、走廊余量、实际接近距离和推力积分。

默认 `gnss-relative-flight-v1` 对伴飞采用 GNSS 反馈，对接近采用从首份 GNSS 估计出发的五次平滑参考轨迹与反馈。方法自行决定控制力，任务契约不要求复现基线轨迹。`no-relative-thrust-v1` 为零推力对照，`unsafe-relative-flight-v1` 为沿轨道法向持续施力的违规对照。当前没有视觉导航或实体推进器布局模型。

```powershell
python tools/autonomy_benchmark.py run d12-continuous-gnss-hold-v1 --seed 1 --output run/benchmark/d12-train-1
python tools/autonomy_benchmark.py batch d29-vbar-gnss-approach-v1 --split test --output run/benchmark/d29-test
python tools/autonomy_benchmark.py run d30-rbar-gnss-approach-v1 --method-spec examples/autonomy_methods/relative-flight.json --seed 201 --output run/benchmark/d30-external-201
python tools/autonomy_benchmark.py schema --kind relative-flight
```

训练种子 1 三项均成功：D12 完整保持 300 s，最大位置误差 0.1337 m；D29 在 173.70 s 完成，最大横向偏差 0.4913 m；D30 在 173.62 s 完成，最大横向偏差 0.8551 m。三项实际原生动作/复位检查通过，契约/评分/目录/SDK/API 等 174 项后端检查通过，261 项前端检查与生产构建通过。

2026-11-06，源码 `bdc93ca` 冻结测试种子 201–210，三项均为 10/10；各自的 Wilson 95% 区间为 72.25%–100%。未依据测试结果修改方法或阈值：

| 配置 | 完成时间 | 全程约束指标 | 最终位置误差 |
| --- | --- | --- | --- |
| D12 持续伴飞 | 全部 300 s | 最大位置误差 0.0971–0.2549 m，全部 15001 帧连续有效 | 0.0186–0.0590 m |
| D29 V-bar 接近 | 173.40–173.66 s | 最大横向偏差 1.0214–2.0511 m，最小走廊余量至少 5.9489 m | 0.0160–0.0792 m |
| D30 R-bar 接近 | 173.50–173.76 s | 最大横向偏差 0.6556–2.1026 m，最小走廊余量至少 5.8974 m | 0.0128–0.0951 m |

零推力对照保留不同的实际失败原因：D12 在 34.46 s 首次离开保持范围，最终记为 `task_failure`；D29 在 240 s 未到达；D30 在 214.5 s 自然漂移至 8.0137 m 横向偏差，触发走廊安全否决。验收脚本原预期 D30 超时，因此停下；修正脚本预期后继续剩余对照，未改物理、评分或原记录。持续法向施力对照中，D12 在 14 s 超速，D29/D30 在 10 s 越出走廊，均为 `safety_violation`。

30 个冻结测试、6 个负对照、3 个独立进程方法及 3 个训练回合，共 42 个回合独立复算一致。三批在冻结源码下恢复，300 个原始回合文件的内容哈希和修改时间均未变；各回合源码符合计划，模型/网格/原生/BSK 身份一致，初始化后状态写入均为零。三项独立进程方法的种子 201 完整物理日志均与内置基线逐字节一致。

| 配置 | 测试汇总 SHA-256 | 种子 201 物理日志 SHA-256 |
| --- | --- | --- |
| D12 | `423ea74071d715844728d010ce9b74217f035f63e4fb1476a145354d604e0fcb` | `61fcd040ea4de2aced75416e59de743bfe85ecc4a840ca5a7df315ca067dfb86` |
| D29 | `b5ca3cb07d5aef826d4b0e8425a4764ff7af3d5106398853aad8502b980652d0` | `5a41682935a15762fb476a59f711b77af0caa9ad899f17fd822293f813bbf5c6` |
| D30 | `ba53da6e227b6d16ffb0ffa501bf7c4b329f59d635459f35826a786c7c88163b` | `aadae71079662d01eedea966d9ec5da46fad6027925d7f7741514726ae917d4f` |

汇总证据位于云端本人工作树 `.tmp/benchmark-relative-flight-acceptance.json`，SHA-256 为 `0df73f3a77315878c5dbfe4a99a10ef1d66305f06171e49cb1f35102589f90cd`，逐配置批次为 `.tmp/<配置 ID>-test/`。实际 Edge 在原平台完成三项独立进程成功任务、D12 零推力失败和 D29 走廊违规共五项；复算、JSON/CSV 和物理日志下载全部通过，并与 CLI 证据一致。界面显示全程最大误差/连续保持或最大横向偏差，失败保留 0/1 成功率及中文原因。浏览器无页面错误，证据在 `.tmp/benchmark-relative-flight-platform-v1/`，报告 SHA-256 为 `723c5ed787f52e6d9b4b42df0da2e4b6b15e66a7159810209c5129748387e293`。

云端独立 UE 工作树已下载 565 个 Content LFS 文件（303046347 字节），UE 5.6 Development Editor 编译通过。`BskUnreal` 自动化报告含 42 项通过、3 项带警告通过、零失败/未运行；报告 `.tmp/benchmark-ue-automation-v2/index.json` 的 SHA-256 为 `014bed4a5a135a01d1f053f7e03bfdbf158471401b814cec392060970a1e959c`。这次采用 NullRHI，只验收适配器基础功能，不计为真实图像闭环。首次手工测试命令的缓存目录超过 UE 的 119 字符限制，原失败日志保留；后续命令使用本人短路径缓存及 `InstalledNoZenLocalFallback`，未改全局设置或共享 UE 进程。正式脚本已有按工作树生成短缓存路径的逻辑，应沿用其配置入口。

## 接入自己的方法

内置方法用于复现基线。要接入自己的控制器或 Agent，可使用 [`examples/autonomy_methods/conventional.py`](../examples/autonomy_methods/conventional.py) 中的 `serve_method` SDK，将方法类替换为自己的 `reset(context)`、`act(observation)`、`close()` 实现。方法进程不加载原生场景，只接收公开任务、硬件上下文和声明观测；动作还需经过环境契约校验。SDK 可以封装外部服务调用，但当前版本不提供 HTTP 方法服务端或非 Python 启动器。

方法 JSON 与入口脚本放在同一目录。`entrypoint` 是 Python 文件，`arguments` 是按原样传入的字符串数组，不经过 shell；`files` 必须列出入口和方法使用的代码、权重、配置。所有文件必须位于该目录内。运行器使用当前 Python 环境；方法的依赖也需在该环境中安装。随机策略应在参数/实现中显式固定自己的随机种子，不能把环境私有种子当作观测。

```text
python tools/autonomy_benchmark.py schema --kind method
python tools/autonomy_benchmark.py schema --kind method-request
python tools/autonomy_benchmark.py schema --kind method-response
python tools/autonomy_benchmark.py run s02-wheel-attitude-v1 --method-spec examples/autonomy_methods/attitude-pd.json --seed 201 --output ../.tmp/external-attitude-201
python tools/autonomy_benchmark.py batch d32-cooperative-gnss-v1 --method-spec examples/autonomy_methods/waypoint-pd.json --split validation --output ../.tmp/external-waypoint-validation
```

`--method-spec` 与 `--method` 互斥。方法声明支持的任务 schema；错误类型会在启动物理环境前被拒绝。方法定义、声明文件 SHA-256 和 Python 解释器身份写入运行清单及批次计划，在启动、关闭和恢复时核验。相同方法名称不能替换冻结的实现；改动方法后必须建立新批次。

线协议为 `spacesim-autonomy-method-jsonl/1`，UTF-8、每行一个 JSON 对象。请求依次为一次 `reset`、多次 `act`、一次 `close`，包含递增的 `request_id`。响应必须返回同一协议和请求编号、`status`（`ok` 或 `error`）；成功的 `act` 响应在 `payload` 中放动作，其他成功响应的 `payload` 为 `null`。错误响应填写 `error`，不得附带动作。`stdout` 专用于协议，诊断写 `stderr`。

- 单条请求/响应上限 1 MiB；重复 JSON 字段、非有限数字、错误编号、缺少换行或错误动作均被拒绝。
- 初始化和每步回复有独立墙钟超时；回合最长墙钟仍受批次上限约束。超时、进程异常和错误回包计为 `method_error`，保留物理证据和全部样本分母。
- 复用项目现有 Windows Job/Linux 子进程监管机制，关闭只清理本回合方法进程及其后代；正常结束、异常和超时都执行清理。
- `method_calls.jsonl` 记录调用耗时与状态；`method_stderr.log` 最多保存前 256 KiB，同时继续排空管道并记录实际字节数。两者均加入证据哈希。
- 调用耗时包含 IPC，不等于纯模型推理耗时。当前仿真逐步等待回复，未把墙钟时延转换为仿真中的执行延迟；不能据此声称通过实时延迟任务。
- 此接口用于受信研究代码，不是对恶意本机程序的权限沙箱。方法仍受运行账户的文件和网络权限约束，协议隔离不能防止主动绕过观测边界。

2026-11-06 云端验证：89 项目录、物理契约、评分、协议和进程检查通过；已有 2 项原生生命周期验收继续作为物理环境证据。外部姿态、GNSS 基线分别在验证种子 101–103 获得 3/3 成功，并分别重跑测试种子 201，其完整原生物理日志与内置方法对应样本的 SHA-256 完全相同。故意在首次动作调用超时的方法在三个验证种子上均为 `method_error`，分母保持 3；物理评分为 `incomplete`，重算不会把方法失败改为物理成功。

三个批次的重算全部一致，恢复后各 36 个回合文件的哈希及修改时间未变。汇总指纹如下，运行数据留在本次云端验证目录：

| 外部方法批次 | 汇总 SHA-256 |
| --- | --- |
| 姿态 PD，验证集 | `6254f2ff54b5d7b5d1260ac8b38ef525606d5fc0f7317d2f1b0bfdbb979ca7be` |
| GNSS PD，验证集 | `21cb89a89dde24da7c78917c792b7bf53a725df5a695462b43971b1e5472585f` |
| 动作超时负对照，验证集 | `138aafa66ac474a19647cc38b9b73bf0442782485b297f6eacb4ab3c2f33a1e5` |

## 在原平台运行评测

按项目原有方式启动平台后，登录并打开“自动采集 → 卫星驾驶评测”，或直接访问 `/collection?mode=benchmark`。选择配置、方法、训练/验证/测试划分，再运行整个冻结划分或其中一个种子。界面会明确显示可运行配置数量和 122 项来源需求数量；依赖检查只检查 Basilisk 包、模型文件和原生构建指纹，不能代替原生验收。

批次通过同一 CLI 启动独立仿真进程。普通用户只能管理自己的批次；管理员可以查看和管理全部批次。同一平台数据目录最多同时运行一个驾驶评测批次；不同数据目录须使用各自进程和端口。网页只接受已注册配置和方法 ID，包括五个随项目交付的独立进程方法示例；自定义 Python 方法通过前述 CLI 在研究主机运行。

“停止批次”先请求当前回合收尾，超时后清理该批次拥有的进程树。已启动回合保留取消或失败结果，尚未启动的回合保持待运行；“继续未启动样本”不会重试失败种子。平台重启会回收旧的本机进程并核对记录，不能接管其他主机上的活动批次。任务、源码或外部方法文件变化时须建立新批次。

在批次卡片点击“查看评测数据”，进入原“数据集”页面的评测视图（`/datasets?benchmark=<批次 ID>`）。它显示全部计划样本的成功率、Wilson 区间、逐种子结果、仿真时长及对应误差指标；未结束批次不报告置信区间。停止、方法错误和基础设施失败均保留在分母中。“独立复算”读取原始证据重新评分；JSON 导出包含任务、冻结计划和汇总，CSV 导出逐种子的结果及适用数值指标。物理日志、观测、动作、评分和版本清单按登录权限下载，并检查归档哈希。

评测文件保存在配置的 `data_root` 上级目录中的 `benchmark_jobs/<批次 ID>/`。`job.json` 保存管理状态，`task.json` 保存任务快照，`batch/plan.json` 保存冻结计划，逐回合数据和评分位于 `batch/<试验 ID>/run/`。运行数据不应加入源码 Git。原有手动录制、自动采集和数据集曲线/视频流程仍通过各自入口使用；当前驾驶评测日志尚未转换为 LeRobot 训练数据。

已登录客户端可使用以下 API；均复用原平台会话，跨用户记录返回 404：

| 请求 | 用途 |
| --- | --- |
| `GET /api/benchmarks/catalog` | 注册配置、方法、冻结种子及依赖检查 |
| `GET /api/benchmarks` | 当前用户可见的批次 |
| `POST /api/benchmarks` | 创建批次：`configuration`、`method`、`split`、`selection`；单次另填 `seed` |
| `GET /api/benchmarks/{id}` | 实时状态、全部计划样本和评分 |
| `POST /api/benchmarks/{id}/stop` | 停止当前批次 |
| `POST /api/benchmarks/{id}/resume` | 继续未启动样本 |
| `POST /api/benchmarks/{id}/rescore` | 核验归档并复算 |
| `GET /api/benchmarks/{id}/export?format=json` | 导出 JSON；也可选 `csv` |
| `GET /api/benchmarks/{id}/trials/{trial}/artifacts/{name}` | 下载允许的证据文件 |

CLI 也支持 `batch ... --seed 201`，但种子必须属于所选划分。`run` 和 `batch` 的 `--stop-file <路径>` 接受停止标记文件；调用方创建该文件即可请求停止。恢复时应使用新的停止标记路径，不能复用仍存在的标记。单次选择与整个划分属于不同冻结计划，不能通过恢复命令互换。

### 平台入口验证

2026-11-06，在独立云端工作树和端口运行生产构建，使用实际 Edge 浏览器登录原平台，并从网页启动 Basilisk 回合：

- S02 验证种子 101 运行中停止，保留取消结果；继续运行 102、103 后均成功，整体为 2/3，未删除取消样本。恢复前后种子 101 的 10 个回合文件 SHA-256 和修改时间完全相同。
- D32 外部 GNSS 方法通过网页选择单次测试种子 201，成功并独立复算一致；完整物理日志 SHA-256 仍为前述 `0a8b14ec5c43368693a668245cd7d1f67fd859d7299cd2933ce81b3ada2551d6`。
- 四个回合均从日志复算一致。实际点击 JSON、CSV 和物理日志下载，返回原“自动采集”及普通“数据集”页面正常，浏览器没有页面运行错误。
- 149 项相关后端检查通过（包含 2 项真实原生生命周期检查），261 项前端测试及生产构建通过。进程用例覆盖后代清理、异常退出、源码变更拒绝恢复、跨用户权限、初始化失败和证据篡改；生命周期替身进程只报告取消/基础设施失败，不作为物理成功证据。

本次云端证据目录为工作树下 `.tmp/benchmark-platform-native-v3/`，含原始批次、浏览器截图和导出。`acceptance.json` SHA-256 为 `7a62849a32480ead86065fc44f5b1c0d53fc8aaf861211c384208fb2fab5c220`；实际网页 JSON 导出 SHA-256 为 `7337f98b96b8ac76c99d6803f88ad06ba648fc01c3420e1375ed9e811013049d`。这些记录验证了当时 S02、D32 的管理与回读入口，不构成其余任务族或真实图像、抓取任务的验收。

后续在真实 Edge 中分别选择 S09、S11 抬升、S11 降低，以独立进程方法运行测试种子 201，三项均成功。网页明确区分 501 km 和 499 km 目标，展示 Δv 矢量误差或半长轴误差/偏心率。三项均实际点击复算及 JSON、CSV、物理日志下载；CSV 保留嵌套轨道指标与矢量分量，缺失数值保持空白。三个下载的完整物理日志分别与冻结 CLI 内置基线的哈希相同，浏览器无页面运行错误。证据目录为 `.tmp/benchmark-orbit-platform-v1/`，`acceptance.json` SHA-256 为 `d6ba18352f5b3115d0ea132fa55c2058ae4cb8151accb35661c8761a7e26634f`。

## 实现边界

| 层 | 责任 | 复用入口或新增内容 |
| --- | --- | --- |
| 来源目录 | 稳定 ID、来源 ID、优先级、任务族、视觉及合作标签 | `benchmark_catalog.py`，不导入动力学 |
| 任务实例 | 初态分布、具体观测/动作、时钟、阈值、故障和随机划分 | D32 保持点、S01/S02/S03 飞轮、S09/S11 轨道机动、S14/D01 导航估计、D12/D29/D30 伴飞和轴向接近配置已实现，其余逐族落实 |
| 执行适配 | reset/step/close、控制权、权威状态和方法观测分离 | 复用既有在轨组件、MissionRuntime、策略接口及原生执行器 |
| 感知链 | 时间标记、标定、实际图像与有效测量 | 复用 GNSS、陀螺、Near23 桥和真实 UE 相机 |
| 独立评分 | 从原始权威证据计算任务/安全/资源结果 | 复用并扩展已有交会、连续性和物理抓持审计 |
| 批评测 | 固定样本、失败分母、隔离、取消/恢复和结果汇总 | CLI/API/原平台入口可运行十二个配置；转训练集待实现 |
| 用户界面 | 选择任务/方法、运行与结果呈现 | 原工作台、自动采集、数据集页面 |

定义一项可执行任务时必须明确：

1. 总航天器数、受控主体数、目标及其他物体的角色、各自合作程度；碎片数量与航天器数量分开。
2. 策略可见观测、环境私有真值、公开任务目标；非合作目标不会因共享状态总线而泄漏遥测。
3. 动作控制层次与执行器限制。推力/力矩、姿态参考、机械臂关节目标和任务阶段决策属于不同接口，分别比较。
4. 每个量的 SI 单位、参考点、参考系、四元数约定、曝光/采样/投递时间；COM、相机深度和 TCP 间隙不混用。
5. 环境成功判据、硬安全约束、合法计划接触、超时和故障条件；控制器自己声称完成不是成功证据。
6. 仿真时间与墙钟预算、种子/资产/配置版本、初态是否有效、终态与证据采样连续性。
7. 工程模型、保真度和依赖。无资源模型时不输出虚构燃耗/能耗，无真实接触时不授予抓取成功。

## 当前可复用的项目能力

最新主线的正式近距路线见 [CAPTURE26_NEAR23_FORMAL.md](CAPTURE26_NEAR23_FORMAL.md)。它已有真实远距状态接续、原图双目导航和四爪接触保持的运行证据，也明确记录了 HTTP 单次从零全程、共享网页部署及多初态的验证边界。

该路线含合作目标调姿及标志物，不能直接代表非合作、未知几何的驾驶/抓取任务。现有 7 维机械臂策略接口也需要扩展卫星姿态、平移和多主体控制能力。既有演示与独立诊断结果只提供复用基础；Benchmark 的新任务与新随机矩阵均需重新运行并独立判分。

本产品采用逐任务族的物理和观测契约，不把现有 `tools/benchmarks/` 中的性能微基准计为驾驶任务覆盖率。
