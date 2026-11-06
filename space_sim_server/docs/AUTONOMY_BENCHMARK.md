# 卫星自动驾驶 Benchmark

最终交付范围和验收合同见 [最终目标](AUTONOMY_BENCHMARK_GOAL.md)。本页说明当前可用入口，以及后续实现必须遵循的数据边界。

当前提供 122 条来源需求，以及三个可执行配置：`d32-cooperative-gnss-v1` 为合作目标 GNSS 导航下的保持点到达与驻留，`s02-wheel-attitude-v1` 为单星飞轮姿态机动与稳定保持，`s03-large-angle-wheel-v1` 为大角度限时姿态机动。它们都有原生执行、内置及外部方法接口、独立评分、冻结种子批评测和离线复算。其他任务族、API、网页和采集集成仍在实施；目录中的 122 条需求不代表 122 个可执行环境。

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
python -m pytest tests/test_benchmark_catalog.py tests/test_benchmark_execution.py tests/test_benchmark_attitude.py tests/test_benchmark_external.py -q
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

## 实现边界

| 层 | 责任 | 复用入口或新增内容 |
| --- | --- | --- |
| 来源目录 | 稳定 ID、来源 ID、优先级、任务族、视觉及合作标签 | `benchmark_catalog.py`，不导入动力学 |
| 任务实例 | 初态分布、具体观测/动作、时钟、阈值、故障和随机划分 | D32 合作 GNSS、S02/S03 单星飞轮配置已实现，其余逐族落实 |
| 执行适配 | reset/step/close、控制权、权威状态和方法观测分离 | 复用既有在轨组件、MissionRuntime、策略接口及原生执行器 |
| 感知链 | 时间标记、标定、实际图像与有效测量 | 复用 GNSS、陀螺、Near23 桥和真实 UE 相机 |
| 独立评分 | 从原始权威证据计算任务/安全/资源结果 | 复用并扩展已有交会、连续性和物理抓持审计 |
| 批评测 | 固定样本、失败分母、隔离、取消/恢复和结果汇总 | 当前 CLI 原生进程编排；API/采集集成待实现 |
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
