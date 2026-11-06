# 卫星自动驾驶 Benchmark

最终交付范围和验收合同见 [最终目标](AUTONOMY_BENCHMARK_GOAL.md)。本页说明当前可用入口，以及后续实现必须遵循的数据边界。

当前提供 122 条来源需求，以及首个可执行配置 `d32-cooperative-gnss-v1`：合作目标 GNSS 导航下的保持点到达与驻留。该配置已有原生执行、独立评分、冻结种子批评测和离线复算。其他任务族、外部方法进程、API、网页和采集集成仍在实施；目录中的 122 条需求不代表 122 个可执行环境。

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

基线 `gnss-pd-v1` 采用点质量前馈和 GNSS 估计上的 PD 控制；前馈近似与 J2 环境的差异保留在测试中。`--method no-thrust-v1` 是不推力负对照，`--method unsafe-approach-v1` 是持续朝目标推进的违规负对照。方法实现见 `simulation/benchmark_methods.py`，`reset/act/close` 输入输出协议位于 `benchmark_contract.py`。目前 CLI 仅注册这三个方法。

每个回合保存冻结任务、源码/运行身份、观测、请求动作、原生物理证据、模型和运行时指纹、评分及文件哈希。`rescore` 核对文件哈希后从原始物理状态重新计算；不需要启动 Basilisk，也不会把取消或基础设施失败改成成功。哈希用于检查归档完整性，不是可信签名。

独立评分要求从 t=0 起每 20 ms 一条证据；缺帧、重复/倒退时间、非有限数和初始化后改写状态均导致无效证据。驻留必须连续满足位置与速度要求，硬安全违规否决成功。安全检查频率为 50 Hz COM 距离和相对速度检查，此场景不宣称进行接触验收。汇总主分母保留所有计划样本，分别列出超时、安全违规、方法失败、无效证据、基础设施失败和取消，并给出完成批次的 Wilson 95% 区间。

基础测试与原生验收分别执行：

```text
python -m pytest tests/test_benchmark_catalog.py tests/test_benchmark_execution.py -q
python -m pytest tests/test_benchmark_native.py -q
```

第二条在云端实际加载原生模型，检查推力确实改变运动、复位重建物理与随机流、旧回合动作被拒绝及关闭后不能步进。普通 CI 按专用验收清单排除此测试，不能将排除或跳过称为原生通过。

## 实现边界

| 层 | 责任 | 复用入口或新增内容 |
| --- | --- | --- |
| 来源目录 | 稳定 ID、来源 ID、优先级、任务族、视觉及合作标签 | `benchmark_catalog.py`，不导入动力学 |
| 任务实例 | 初态分布、具体观测/动作、时钟、阈值、故障和随机划分 | D32 合作 GNSS 配置已实现，其余逐族落实 |
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
