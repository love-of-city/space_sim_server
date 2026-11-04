# 插头连续运动与有界参数优化

本页说明当前 `plug-insert/19-axis-contact-continuity` 的职责边界、离线验证流程及下一轮参数迭代方式。完整任务、录像验收和随机布局成功率分别记录；版本名或某条任务成功不能代表已达到连续性目标。正在修改的正式采集批次保持停止，诊断仅写入仓库外的新目录。

## 代码放置与调用关系

| 文件 | 职责 |
|---|---|
| [`simulation/scripted_tasks/plug_insert.py`](../simulation/scripted_tasks/plug_insert.py) | 生产控制器：抓取、拔出、搬运、插入、松爪、撤离；执行真实姿态/接触反馈及最终成功判据 |
| [`simulation/scripted_tasks/plug_contact_feedback.py`](../simulation/scripted_tasks/plug_contact_feedback.py) | 插拔接触参考：名义位姿跟踪、基于实际伺服刚度的有限摩擦预载、拔出速度阻尼、整体输出速度/偏差限幅及限幅回算；只输出执行器参考 |
| [`simulation/scripted_tasks/plug_profiles.py`](../simulation/scripted_tasks/plug_profiles.py) | 航空、导向、圆柱三类插头的尺寸、键向和抓取轴 |
| [`simulation/scripted_tasks/plug_motion_config.py`](../simulation/scripted_tasks/plug_motion_config.py) | 从场景读取允许的运动参数，校验参数类型、范围和覆盖顺序 |
| [`tools/benchmarks/optimize_plug_motion.py`](../tools/benchmarks/optimize_plug_motion.py) | 独立诊断入口：冻结候选与案例，限制迭代/仿真/墙钟预算，调用生产单任务入口并给出推荐或拒绝原因 |
| [`tools/benchmarks/plug_motion_metrics.py`](../tools/benchmarks/plug_motion_metrics.py) | 只读分析实际物体速度、剩余行程、阶段耗时及各阶段腕力/力矩 |
| [`collection/run_scripted_task.py`](../collection/run_scripted_task.py) | 复用正式执行链的权威动力学与任务装配；绑定实际伺服刚度和保持力配置，保存配置、阶段、轨迹、结果和运行来源 |

生产采集只依赖控制器和配置模块。优化入口位于 `tools/benchmarks/`，因为它组织同条件的测量和候选比较，不承担正式批次调度，也不会改写生产源码。日常采集入口仍见[项目结构地图](../../docs/project-map.md)。

## v19 控制器修改方向

圆柱插头没有机械键向。接触阶段只纠正轴线倾斜，保留圆柱绕自身轴的自由旋转；不再为追赶完整姿态中的轴向转角反复停推或停拉。航空、导向插头保留键向约束。推进姿态容差、爬行速度和允许失配比例都从统一配置读取，不通过复制脚本维护实验版本。

落座后的圆柱轴向自旋不再单独拖延进入松爪。轴线倾斜、径向位置、插入深度、落座状态以及必要的速度条件继续检查；最终几何验收不由候选参数改写。松爪进入撤离前，先确认卸载和双指脱离，再查询现有原生几何净空；不为一个与实际净空无关的固定额外开爪量继续等待。净空查询失败仍拒绝撤离。圆柱撤离候选复用完整路径规划的 2 mm 净空及接触过渡渐变规则，检查真实开爪时序、实际初始开度与全开包络；若首个方向不合格，继续检查其他上升方向，不让较宽松的局部检查提前选错路线。

轻微失配时使用有负载限制的低速推进，明显失配、高负载和真实通道问题仍可能触发减速或暂停。是否解决长停顿必须依据真实物体运动测量；`insertion_alignment_paused`、`insertion_speed_scale` 或非零控制命令本身都不能证明物体正在移动。

插拔跟踪由 `PlugContactFeedback` 负责。它将名义位姿参考与克服轴向干摩擦所需的有限伺服偏移分开保存，避免把每次摩擦补偿累积为位置误差。插入接近已配置的摩擦作用区时平滑建立向下预载，通过实际关节 Jacobian 和伺服刚度换算为参考偏移。拔出则建立反向预载，保持到已配置摩擦行程结束，再在额外 4 mm 行程内平滑卸载；当前保持力行程为 8 mm 时，对应 8–12 mm 卸载窗口，不在仍受静摩擦约束时提前撤去补偿。

拔出保留实测轴向速度反馈和横向纠偏：速度使用 0.005 秒滤波，速度误差增益由 `extraction_velocity_gain` 配置，当前圆柱为 6，航空/导向为 8；横向位置纠偏增益由 `extraction_lateral_gain_s_inv` 配置，当前航空为 4 s⁻¹，圆柱/导向为 3 s⁻¹。接触控制最终输出的完整关节步长按任务关节速度上限限幅，避免预载偏移绕过名义轨迹的速度限制；参考与实测关节之间的整体偏差另限制在 0.06 rad。超限时按同一比例缩放完整偏移，保留笛卡尔方向，并回算名义参考。松爪阶段通过现有卸载控制释放预载；进入接触恢复/回退时也显式禁止继续建立摩擦预载。

预载不会直接给插头写入位置、姿态或外力，也不修改碰撞、摩擦、保持力或成功阈值。插入连续性必须同时检查预载建立、真实速度、位姿/净空和分阶段载荷，不能只根据参考曲线平滑判断完成。

当前最终验收、旧版本记录的兼容审核和全过程碰撞核验范围见[任务与数据验收](../../docs/reference/task-and-data-acceptance.md)。

## 配置及可调范围

保存位置为场景的 `collection_config.plug_motion_parameters`。解析顺序为公共默认值、类型差异、场景增量、非空显式入口参数；最终解析结果写入单任务 `summary.json` 的 `config`。完整冻结快照和派生实验规则见[当前配置说明](plug-configuration-consolidation.md)。白名单外的物理、几何、保持力、载荷上限及成功容差不允许通过本入口覆盖。

接触反馈使用另一条明确的运行时绑定：权威动力学创建或恢复后、任务启动前，`run_scripted_task.py` 把实际使用的 `native.KP` 及已安装 `plug_retention.parameters` 传入 `configure_contact_feedback()`；预载幅值取 `sliding_friction_n`，作用行程取 `friction_travel_m`。生产循环和复用动力学图均经过此绑定，避免实验常量与实际伺服/保持力配置脱节。未启用保持力时预载为零；非零摩擦预载必须有六个有限正刚度和有效行程。

最终绑定保存在 `summary.json.contact_feedback`，包含算法标识、六关节刚度、摩擦幅值/行程、参考偏差限制、预载建立/卸载行程与滤波时间、拔出速度和横向反馈配置；轨迹的 `contact_preload_force_n` 保存当前实际计算的预载。实际输出速度上限同时核对 `summary.json.config.arm_speed_rad_s`。刚度和保持力取自运行时配置，预载过渡、滤波和偏差限制由控制器定义；拔出反馈增益仅允许在下表白名单范围内覆盖。比较实验时同时核对 `config`、`contact_feedback` 和保持力/原生库来源，而不是只看候选 JSON。

| 参数 | 单位或含义 | 优化入口上限 |
|---|---|---|
| `insert_speed_m_s`、`extraction_speed_m_s` | 插入/拔出速度，m/s | 0.025 |
| `extraction_lateral_gain_s_inv` | 拔出时实际横向位置纠偏增益，s⁻¹ | 6 |
| `extraction_velocity_gain` | 拔出时轴向速度误差反馈增益，无量纲 | 8 |
| `insert_orientation_tolerance_rad` | 推进对齐姿态容差，rad | 0.008 |
| `insert_slowdown_ratio`、`insert_resume_ratio` | 归一化对齐误差比 | 必须满足 `0 < slowdown < resume < 1` |
| `insert_creep_ratio` | 允许低速推进的最大归一化失配比 | `1 < value <= 1.75` |
| `insert_creep_speed_scale` | 低速推进占名义插入速度比例 | 0.5 |
| `insert_creep_delay_s` | 进入低速推进前的短暂等待，s | 1.0 |
| `insert_creep_force_fraction`、`insert_creep_torque_fraction` | 允许低速推进的负载占各自上限比例 | 0.5 |

全部参数必须为有限正数，布尔值和 NaN 拒绝；表中范围是实验边界，不代表该范围内所有组合均已通过物理验证。新增参数须先增加显式配置校验和有针对性的回归测试。

## 连续性与质量判据

本轮固定连续性门槛为：**仍有接触行程时，实际插头轴向速度绝对值低于 0.001 m/s（1 mm/s）的连续窗口不得超过 0.5 秒**。拔出和插入分别统计，不能用较快的搬运阶段平均掉接触停顿。这里的轴向方向与当前控制器一致，采用卫星本体系 z 方向。

工具按以下条件决定是否推荐：

1. 所有必测案例均完整成功；基线本身也必须成功且具有完整测量。基线可因为停顿超标而不被推荐，仍用于候选载荷比较。
2. 拔出、插入的真实速度及腕力数据完整有效，并满足上述 0.5 秒门槛。使用 `contact_remaining_m` 区分剩余接触行程与落座确认，分别报告 `release`、`verify` 耗时。
3. 候选全任务及 **extract/insert 各阶段** 的腕力、力矩峰值分别不得超过相同案例基线的 110%。较大的拔出峰值不能掩盖较小但明显恶化的插入峰值。生产 `wrist_wrench={valid,force_n,torque_nm}` 和旧六维数组均可读；缺失/无效的阶段传感器记录不能由整任务摘要替代。
4. 使用同一生产状态机、成功判据及原生通道/松爪净空检查。几何失败或运行失败不能通过推荐；本工具不额外宣称已完成全过程每个物理子步的接触审核。
5. 场景、模型和源码来源能够对应；模型文件及所记录源码在运行前后发生变化时拒绝本次结果。

`axial_target_speed_m_s` 表示实际发给接触控制的轴向目标，用于零目标计数；`axial_reference_speed_m_s` 若存在则另行统计参考为零、实际仍移动或参考非零、实际卡住的时间。缺少某字段时保留采样数量为零，不伪造证据。旧记录没有 `contact_remaining_m` 时，插入按落座间距估计，拔出保守地计入整个阶段，报告 `fallback_remaining_samples`；新旧口径比较时必须标明这一差别。

峰值来自轨迹任务采样及单任务摘要，不等于已检查所有物理子步。几何完整成功、动力学连续性通过和 UE 双相机录制通过是不同层次的验收。

## 案例与候选格式

将下列占位路径替换为真实、已保存的场景和对应模型。场景需包含真实随机化初态；不同候选必须复用同一组案例。模型路径相对于案例清单目录解析；场景文件同样支持绝对路径或相对案例清单路径。

`cases.json`：

```json
[
  {"name": "cylindrical-layout-129", "task": "cylindrical_plug_insert", "scene": "/absolute/frozen/cylindrical-scene.json", "model": "/absolute/frozen/model.xml"},
  {"name": "aviation-layout-129", "task": "aviation_plug_insert", "scene": "/absolute/frozen/aviation-scene.json", "model": "/absolute/frozen/model.xml"},
  {"name": "guide-layout-129", "task": "guide_plug_insert", "scene": "/absolute/frozen/guide-scene.json", "model": "/absolute/frozen/model.xml"}
]
```

正式恢复前应加入当前出现长停顿的布局和独立保留的复验布局；只测一个圆柱案例不能说明其他插头或所有布局已通过。

`candidates.json`：

```json
[
  {"name": "baseline", "parameters": {}},
  {"name": "short-alignment-hold", "parameters": {"insert_creep_delay_s": 0.1}},
  {"name": "bounded-creep", "parameters": {"insert_creep_delay_s": 0.1, "insert_creep_ratio": 1.5, "insert_creep_speed_scale": 0.25}}
]
```

这些是输入格式示例，未声明任何示例候选已通过。省略 `baseline` 时工具自动插入；空覆盖表示沿用场景保存的参数，并非清除场景参数。所有案例均为必测，名称唯一且只使用字母、数字、下划线、短横线。

## 运行与证据

从 `server/` 使用已配置的仿真 Python 执行：

```bash
python tools/benchmarks/optimize_plug_motion.py \
  --cases /absolute/study/cases.json \
  --candidates /absolute/study/candidates.json \
  --output /absolute/artifacts/verification/plug-motion-study-YYYYMMDD-HHMMSS \
  --max-iterations 8 --max-sim-seconds 240 --timeout-seconds 900 \
  --max-stagnant-seconds 0.5 --speed-threshold-m-s 0.001 --load-multiplier 1.1
```

每个候选/案例使用独立子进程执行生产 `collection/run_scripted_task.py --native-geometry --save-trace`，不启动正式采集或 UE 录像。`--max-iterations` 包括基线；仿真及墙钟上限按每个案例计算。达到首个全部通过的候选或预算用尽即结束当前轮；本轮验收不得通过放大 0.5 秒门槛来消除失败。

输出目录必须不存在，且不能位于已有正式批次内。输出包括输入清单副本、原场景副本、修改后的候选场景、输入/源码 SHA256、`sources/` 中对应源码字节副本、每次命令与日志、逐案例评价、`results.json`、`recommendation.json`、`report.md`。模型保留原路径以维持相对资产引用，记录并核对文件哈希；完整模型/原生库身份继续由生产入口的运行来源报告记录。

只读分析已有完成的诊断任务：

```bash
python tools/benchmarks/optimize_plug_motion.py \
  --analyse-run /absolute/diagnostic/run-a /absolute/diagnostic/run-b \
  --output /absolute/artifacts/verification/plug-motion-analysis-YYYYMMDD-HHMMSS \
  --max-stagnant-seconds 0.5 --speed-threshold-m-s 0.001
```

此模式需要各任务已保存 `summary.json` 和 `trace.jsonl`，将分析结果写入新目录的 `analysis.json`。它不会启动仿真、补写旧任务或恢复批次；多个只读输入逐条评价，不自动把第一条当作其余任务的载荷基线。

## 自动迭代与无解处理

“自进化”在这里分为可复验的有界参数搜索和基于失败证据的结构修改：先冻结案例与当前实现，运行基线，逐候选测量；完整成功、连续性和载荷门槛共同决定保留或拒绝。运行中不任意生成执行代码、改成功阈值或覆盖场景原件。

若候选都失败，保存 `no_candidate_passed` 和全部证据，按故障类型决定下一次结构修正：

- 零目标时间过长：检查对齐锁存、旋转自由度和阶段转换条件。
- 目标非零而真实移动停滞：检查接触参考、夹持滑移和控制反馈；不盲目加大推进速度。
- 已落座却停留：分别检查轴向自旋、卸载、双指接触与撤离净空。
- 峰值恶化或几何失败：拒绝候选，回到对应阶段的控制/规划实现定位。

结构修正由独立可审查的源码变更完成，更新任务版本及对应审核规则，再用新目录重新冻结基线、测试和复验；不把之前的测量当作新实现证据。只有必测案例全部通过后，才安排独立 UE 双相机完整录制与数据审核。正式批次是否恢复是后续操作；本轮停止的批次保持取消状态。

## 2026-11-03 验证范围与结果

最终源码和默认参数完成五个固定案例：圆柱布局 025、005、041，以及航空、导向布局 025。全部任务成功，剩余接触行程内的最长实际低速窗口均不超过 0.5 秒，全任务及拔出/插入分阶段载荷均不超过同案例原成功轨迹的 110%。源码哈希与最终工作区一致，插拔参考最大关节速度不超过配置的 0.35 rad/s。

| 案例 | 插入阶段原耗时（秒） | 修改后（秒） | 修改后最长低速段：插入 / 拔出（秒） |
|---|---:|---:|---:|
| 圆柱 025 | 20.845 | 3.465 | 0 / 0.045 |
| 圆柱 005 | 30.590 | 3.450 | 0 / 0.325 |
| 圆柱 041 | 28.880 | 3.470 | 0 / 0.185 |
| 航空 025 | 16.460 | 2.840 | 0 / 0.310 |
| 导向 025 | 13.895 | 4.010 | 0.160 / 0.030 |

圆柱 025 同时完成 UE 双相机完整录制：每路 2212 帧、50 Hz、1024×1024，完整视频解码、7 维动作/状态、相机配对及 200 Hz 腕力一致性验收通过。263 项针对性回归通过；五个固定案例不代表 150 个布局的成功率验收。落座后的卸载、开爪及净空确认仍占约 2.1–3.3 秒，完成确认约 0.2–0.315 秒，均单独统计，不称为零耗时。

本次机器上的完整证据位于仓库外 `artifacts/verification/plug-continuity-20261103/`：`REPORT.md`、`final-validation.json`、`final-recording-validation.json`、`final-source-snapshot/` 和 `final-cylinder-comparison.mp4`。视频以各自插入起点按实速对齐，新版完成后明确标记停留在最后一帧。旧运动按保存的 50 Hz 观测测量，旧腕力采用 200 Hz 原始记录和精确事件边界；JSON 同时保留新轨迹按相同 50 Hz 口径的对照。
