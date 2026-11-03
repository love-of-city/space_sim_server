# 任务与采集数据验收口径

核对日期：2026-10-31；核对主仓库源码基线：`30dea8f89765ff3530c6b08b730891bf8ee92c42`。本文说明实际执行的判据与审核范围，不把历史测试通过、任务版本名或目录中的 `accepted` 单独当成新批次认证。具体 episode 还须核对其 `task_version`、`task_parameters`、配置快照与运行来源记录；参数被显式覆盖时，以该条保存值和对应版本实现为准。

2026-11-03 补充四项单向任务集成：保留下文原六任务的当前规则，新增方向专用验收；本次未重新完成 UE/native 实采认证。操作入口见[项目结构地图](../project-map.md)，版本与已完成实验的边界见[采集功能基线](../../server/docs/COLLECTION_BASELINE.md)。

批任务交接见[全流程指南](../guides/simulation-data-workflow.md)，参数的覆盖优先级见[采集配置参考](collection-configuration.md)，模型与物理假设见[物理模型参考](physics-model-configuration.md)。

插头真实运动的连续性测量、v19 旋转自由度处理和有界参数搜索见[插头运动优化](../../server/docs/plug-motion-optimization.md)。其 0.5 秒停顿门槛和各阶段载荷对照属于新增诊断验收，不把历史任务成功自动升级为连续性通过。

## 1. 三层“通过”必须分别判断

| 层次 | 当前证据与入口 | 能说明什么 | 不能据此说明什么 |
|---|---|---|---|
| 任务成功 | 控制器 `result="success"`、完整阶段与实际测量；`recording/dynamics/summary.json` | 当前任务版本的动作与终态判据达成 | 视频已完整封装、每帧配对正确、全过程无非预期碰撞 |
| 录制与采后审核通过 | `dataset_status="complete"`、快速检查、完整 `validate_recording`、布局/机位绑定 | 在验证器声明范围内，录制可读、数值与来源匹配、相机配对及任务记录检查通过 | 任意布局都成功、画面无遮挡、现实机器人可直接复现、训练分布合适 |
| 训练数据资格 | 完整审核结果加训练数据集的选样、质量审查与冻结清单 | 该条被某个明确的数据集版本采用，具有可追溯原因 | 项目目前没有一个覆盖上述要求的统一 `training_eligible` 自动标志；不能把 `accepted` 改称“全部训练合格” |

自动筛选至少排除待审核、失败、中断、取消和来源无法对应的记录。训练集还应记录适用任务、版本、布局/种子、视觉质量检查、划分规则及必要的接触验收范围。这是数据集发布要求，**不是现有采集器已经全部实现的自动门禁**。

源码：[录制结束检查 `validate_episode`](../../server/collection/record_scripted_task.py)、[快速/完整审核与任务记录审核](../../server/collection/recording_validation.py)、[compact 记录读取与数值摘要](../../server/backend/space_arm_platform/compact_storage.py)。

## 2. 当前十项任务及默认启用范围

| 任务标识 | 实际控制器版本 | 默认批次启用 | 核心动作 |
|---|---|---|---|
| `drawer_cycle` | `drawer-cycle/7-closed-completion` | 是 | 打开抽屉、回推闭合、松爪、撤离 |
| `aviation_plug_insert` | `plug-insert/19-axis-contact-continuity` | 是 | 拔取航空插头，按键向插入对应孔 |
| `guide_plug_insert` | `plug-insert/19-axis-contact-continuity` | 是 | 拔取导向插头，按键向插入对应孔 |
| `cylindrical_plug_insert` | `plug-insert/19-axis-contact-continuity` | 是 | 拔取圆柱插头，插入对应圆孔 |
| `locking_rod_cycle` | `locking-rod-cycle/4-locked-completion` | 是 | 松紧固件、杆放入卡扣、扶起、重新锁紧 |
| `aviation_bolt_insert` | `aviation-bolt/5-seated-completion` | 否 | 从源孔旋出航空螺栓，搬运并旋入目标孔 |
| `drawer_open` | `drawer-open/1` | 是 | 从关闭拉起并释放，保持打开 |
| `drawer_close` | `drawer-close/1` | 是 | 从打开状态放回、锁定并释放 |
| `locking_rod_fold` | `locking-rod-fold/1-straight-approach` | 是 | 从竖起锁紧状态旋松、放倒入卡扣并释放 |
| `locking_rod_restore` | `locking-rod-restore/1-straight-approach` | 是 | 从卡扣内扶起、旋紧并释放 |


启用状态来自 [collection.json](../../server/config/collection.json)。任务存在、能列在网页中，不等于当前默认批次会采集，也不等于它已完成本轮 UE 实采验收。

三个插头的有效版本由 `PlugInsertTask.task_version()` 和对应 profile 共同确定；不能只读类定义中保留的 `version="plug-insert/2"`。`plug_insert` 是圆柱任务兼容别名，`drawer_lift` 是旧抬升/诊断入口，不是额外两个当前正式任务。

源码：[抽屉](../../server/simulation/scripted_tasks/drawer_cycle.py)、[插头控制器](../../server/simulation/scripted_tasks/plug_insert.py)、[插头 profile](../../server/simulation/scripted_tasks/plug_profiles.py)、[螺纹任务注册](../../server/simulation/scripted_tasks/threaded_tasks.py)、[航空螺栓](../../server/simulation/scripted_tasks/aviation_bolt.py)、[锁杆](../../server/simulation/scripted_tasks/locking_rod.py)。

## 3. 任务的实际关键判据

以下长度均为米换算的毫米，角度均为 rad。SARM 当前夹爪驱动范围为 0–1 rad，保存的归一化开度在该模型上数值相同；不要把其他夹爪的归一化开度直接当成 rad。

### 3.1 抽屉完整循环

当前阶段顺序为 `settle → approach → grip → lift → return → release → retract → verify`。关键默认值与在线条件：

| 项目 | 当前实现 |
|---|---|
| 初态 | `abs(drawer_m) ≤ 5 mm`；超过时失败 `drawer_not_initially_closed` |
| 打开目标 | `lift_m=80 mm`；lift 阶段相对初始位置误差 `≤5 mm`，抽屉速度绝对值 `<3 mm/s`，双指均有接触 |
| 夹持 | 实际双指接触，grip 连续确认 `0.15 s`；不是靠绑定物体或写状态实现 |
| 回推就位 | 相对初始抽屉位置误差 `≤2 mm`，速度绝对值 `<3 mm/s`；持件 TCP 位置误差 `<5 mm`，return 确认 `0.2 s` |
| 闭合判据 | 相对初始抽屉位置误差 `≤2 mm`，卡扣相对初始角误差 `≤0.025 rad` |
| 松爪确认 | 闭合成立、开度 `≥0.6`、双指接触为零；条件中断会重置 `released_closed_seconds` |
| 最终完成 | 已打开、已回推就位；夹爪 `>0.95`，TCP 到撤离目标误差 `<3 mm`；释放后的闭合条件已持续 `≥0.3 s`，verify 阶段达到自己的 `0.2 s` 连续条件 |

`0.3 s` 的释放闭合计时可在 release/retract 阶段积累，不能机械地解释为末尾固定追加 `0.3+0.2 s`。v7 的 `_closed()` 不再要求抽屉末态速度低于阈值；`_seated()` 和打开阶段仍检查抽屉速度。基类 `DrawerLiftTask.step()` 对所有阶段切换及最终 verify 仍要求机械臂六轴实测速度的最大绝对值 `<0.015 rad/s`，不能概括成“最终完全没有速度门槛”。

完整采后审核还要求阶段序列匹配版本、`open_verified`、`seated_before_release_verified`、峰值打开量 `≥75 mm`、最终相对位移绝对值 `≤2 mm`、开度 `≥0.95`，当前 v7 的 `released_closed_seconds ≥ task_parameters.verify_hold_s`。这些审核阈值并非对任意改过 `lift_m` 的诊断任务自动适配。

源码：[DrawerLiftConfig / DrawerLiftTask.step](../../server/simulation/scripted_tasks/drawer_lift.py)、[DrawerCycleConfig / _stage_reached](../../server/simulation/scripted_tasks/drawer_cycle.py)、[validate_recording 抽屉分支](../../server/collection/recording_validation.py)。策略控制另有 [DrawerCycleSuccess](../../server/simulation/policy_task_success.py) 的 `drawer-policy-success/2` 监视器，不应拿其版本号代替脚本控制器版本。

### 3.2 三种插头

共同终态要求：实际拔起过、插入正确对象对应的目标 site、径向与轴线合格、法兰落座、松爪并完成撤离。当前 v19 默认连续几何确认 `0.2 s`。

| 项目 | 航空 | 导向 | 圆柱 |
|---|---|---|---|
| 零件 body | `aviation_plug` | `guide_plug` | `cylindrical_plug` |
| 目标 site | `aviation_insert_40p5` | `guide_insert_40p5` | `cylindrical_insert_40p5` |
| profile 尖端局部长度 | `80 mm` | `95 mm` | `95 mm` |
| profile 法兰局部长度 | `25 mm` | `25 mm` | `25 mm` |
| 最终插入深度 | `>53 mm` | `>68 mm` | `>68 mm` |
| 径向误差 | `<0.4 mm` | `<0.4 mm` | `<0.4 mm` |
| 轴线夹角误差 | `<0.01 rad` | `<0.01 rad` | `<0.01 rad` |
| 法兰间隙绝对值 | `<1 mm` | `<1 mm` | `<1 mm` |
| 键向/完整姿态误差 | `<0.01 rad` | `<0.01 rad` | 不以绕自身轴的键向作为终态条件 |

深度下界由 `tip_m − flange_m − 2 mm` 得到；不是三种插头共享一个固定深度。这里的 site 名字是模型标识，颜色语言还由该条场景的颜色绑定产生。

需要同时保留的阶段条件：

- 抓取要求双指真实接触、抓取朝向误差 `<0.05 rad`、TCP 抓取位置误差 `<4 mm`、相对运动速度 `<0.015 m/s`，连续 `0.15 s`。搬运过程有滑脱检测；`ever_lifted` 依据实际零件抬高量 `>0.12 m`。
- v19 插入推进要求径向 `<0.4 mm`；航空/导向检查含键向的姿态误差 `<0.004 rad`，无键圆柱只检查轴线倾角 `<0.008 rad`（`insert_orientation_tolerance_rad` 默认按插头类型解析）。归一化误差 `0.75–1` 区间平滑降速，暂停后回到 `0.90` 内恢复；每次暂停 `0.25 s` 后，对误差比不超过 `1.25` 且腕力/力矩不超过各自上限 `50%` 的轻微失配使用 `18%` 轴向爬行速度。明显失配或负载升高时禁止爬行，目标推进领先量限制为 `2 mm`。最终径向 `0.4 mm`、轴线/键向 `0.01 rad` 及深度/落座判据保持原值；v17 历史记录保留原先统一 `0.004 rad` 推进门槛。
- v19 使用独立名义参考与有限摩擦预载，避免接触误差重复积分并扭曲下压方向；预载读取实际运行的关节刚度及保持力配置。生产控制参数和诊断验收见[优化方案](../../server/docs/plug-motion-optimization.md)。
- `insertion_alignment_pause_s` 保留对齐暂停锁存的累计时间（包含爬行）；新增 `insertion_alignment_hold_s` 统计零目标推进时间，`insertion_alignment_creep_s` 统计爬行时间，两者之和等于暂停锁存时间。它们是控制状态时长，实际物体速度另由 `plug_velocity_body_m_s` 判断。
- 圆柱在 insert→release 前额外要求法兰间隙 `<min(seat_tolerance_m, 0.1 mm)`。最终完成容差仍为上表的 `1 mm`。
- release 阶段要求卸载、双指均无接触，连续 `0.1 s`。有原生状态的圆柱随后直接查询实际净空与撤离路径，不额外等待固定开爪增量；其他分支仍要求实际左右开度达到 `min(release_open_rad−0.02, release_start_opening+0.08)`。净空不合格继续开爪，后续开爪与撤离可以重叠。
- 最终 verify 的默认开度条件为 `>0.65−0.03=0.62`，TCP 在孔平面上的距离 `>0.09 m`，几何连续确认 `0.2 s`。采后审核的末帧开度比较为 `≥0.62`，严格比较符号与在线条件略有不同。

**v15/v16/v17/v18/v19 的末尾 verify 不要求速度衰减到 `<0.002 m/s`、`<0.03 rad/s`。** 释放前的 insert 条件仍有任务差异：航空和圆柱要求插头相对线速度范数 `<0.002 m/s`；航空检查相对角速度范数 `<0.03 rad/s`，v19 圆柱只检查去掉绕自身轴自旋后的倾斜角速度 `<0.03 rad/s`。导向分支不要求这两个低速阈值。其他抓取/持件条件按各分支判断，不能概括成“所有速度条件都已删除”或“三种插头释放前都必须达到相同低速”。旧版本录制由验证器保留原来的末态速度条件。

`validate_plug_recording` 检查阶段、版本对应的抓取证据、最终对象/site、几何、抬升标志和开度。它目前**没有独立从整段保存序列重算 0.2 s 完成窗口、TCP 撤离距离或全过程接触**；控制器 `success` 及保存指标仍承担部分证明责任。要求独立物理审计的数据集应另存/审核所需轨迹。

源码：[当前配置](../../server/simulation/scripted_tasks/plug_motion_config.py)、[task_version / step](../../server/simulation/scripted_tasks/plug_insert.py)、[几何 profile](../../server/simulation/scripted_tasks/plug_profiles.py)、[圆柱释放净空](../../server/simulation/scripted_tasks/plug_clearance.py)、[validate_plug_recording](../../server/collection/recording_validation.py)。

### 3.3 航空螺栓旋出并装配

当前版本为 `aviation-bolt/5-seated-completion`。必须有源孔旋出、实际拔离/抬起、搬运、目标旋入、松爪及撤离；允许真实半圈换抓循环，阶段审核不要求每个阶段只出现一次。

| 项目 | 默认判据 |
|---|---|
| 螺纹导程 | `thread_pitch_m=0.016` 实际表示每整圈前进 `16 mm` 的双头螺纹导程 |
| 源孔旋出要求 | 按 `2π × source_clearance_m / thread_pitch_m` 计算；`source_clearance_m=58.5 mm` |
| 目标旋入要求 | 按 `2π × target_clearance_m / thread_pitch_m` 计算；`target_clearance_m=60 mm` |
| 最终累计转角 | 源/目标各自 `≥required_rotation_rad−0.15 rad`，不是仅检查最后一段腕部转角 |
| 最终位置 | 径向误差 `<1 mm`，落座间隙绝对值 `<1.5 mm`，轴线误差 `<0.035 rad` |
| 释放 | 双指接触为零，左右开度最小值 `≥0.40 rad`，已完成撤离 |
| 完成确认 | 上述条件连续 `0.3 s`；当前 v5 不要求末态线/角速度低于旧版门槛 |

完整录制审核额外要求：carry 入口实际源向位移 `≥source_clearance_m−1.5 mm`，尖端越过源孔口 `≥0.1 mm`，源累计转角 `≥required−0.05 rad`；每个旋拧段重建双指卡扣接触。不能用一条最终落座截图替代源孔旋出证据。

源码：[AviationBoltConfig / 转角计算 / step](../../server/simulation/scripted_tasks/aviation_bolt.py)、[validate_threaded_recording](../../server/collection/recording_validation.py)。

### 3.4 锁杆松开、放倒、复位和锁紧

当前版本为 `locking-rod-cycle/4-locked-completion`。最终必须同时保留 `loosen_verified`、`folded_verified`、`returned_verified`、`tightened_verified` 四项物理过程证据。

| 项目 | 默认判据 |
|---|---|
| 松开 | 紧固件滑移坐标 `≤0.35 mm`，相应阶段挂钩接触为零 |
| 放入卡扣 | 铰链角接近 `−1.53 rad`，误差 `≤0.025 rad`；杆/紧固件与卡扣接触计数之和 `>0` |
| 扶起/闭合 | 铰链角绝对值 `≤0.025 rad` |
| 锁紧位置 | 紧固件滑移在 `[4.4, 6.5] mm`，螺旋耦合误差 `≤0.10 mm` |
| 实际锁紧证据 | 正确闭合几何下出现挂钩接触且法向力 `≥1 N`，保存就位时滑移/转角和峰值反力 |
| 释放后保持 | 滑移回退不超过 `0.35 mm`；转角回退不超过 `0.35 mm × 2π / 实际螺纹导程`，继续满足闭合几何 |
| 最终释放与确认 | 双指接触为零、左右开度最小值 `≥0.40 rad`，完整过程标志成立，保持 `0.3 s` |

轨道环境下卸载后的挂钩反力可以变为零。当前要求的是“锁紧过程中确有正确的接触/反力，释放后保持已达成的就位状态”，不是“最后一帧必须仍有 1 N 预载”。当前 v4 的最终 verify 不再以末态线速度、角速度或铰链速度为门槛；旋拧阶段中的停稳/换抓条件仍存在。

源码：[LockingRodConfig / _released_locked / step](../../server/simulation/scripted_tasks/locking_rod.py)、[validate_threaded_recording](../../server/collection/recording_validation.py)。

### 3.5 两种螺纹任务共同的采后质量检查

`validate_threaded_recording` 还要求：参考运动质量有样本且数值有限；最大关节参考速度不超过该条 `arm_speed_rad_s+1e−5`；最大参考加速度 `≤1 rad/s²`；跨阶段参考速度跳变 `≤0.00501 rad/s`；旋拧样本中卡扣接触样本数至少占 `95%`，腕轴误差 `<0.05 rad`。这些是参考指令与接触证据门槛，不是硬件额定值或所有接触峰值的安全认证。

末尾确认窗口按 50 Hz 保存记录复核，代码对窗口端点作采样容差处理；不是逐条重放 400 Hz 力学。历史 previous version 仍保留各自末态速度要求。

批次默认选择 `fast_collection`：自由开爪目标为 `0.45 rad`，常规 profile 是 `0.75 rad`；两者与当前最终净空下界 `0.40 rad` 含义不同。具体速度/路径参数见 [THREAD_SPEED_PROFILES](../../server/simulation/scripted_tasks/threaded_motion.py)。

### 3.6 四项单向任务

任务专用初态在录制前生成 MJCF，不在采集中改写机构状态。抽屉放回的默认初始开度为 `0.08 m`，锁杆复位初态为约 `−1.53 rad` 放倒且紧固件松开；对应记录不得冒充本条已完成反向操作。

抽屉审核要求当前任务版本和完整阶段、正确的实际起终位置、操作阶段双指接触、释放后零接触及按配置保持的稳定状态；最终开度 `≥0.95`。锁杆放倒仅要求旋松及入卡扣的操作证据；复位要求真实初始卡扣状态、扶起和旋紧证据，不要求本条旋松/放倒历史。

独立抽屉 v1 默认释放保持 `2 s`，独立锁杆 v1 默认 `1 s`，不继承完整循环新版本的 `0.3 s`。独立锁杆 v1 的释放保持期仍检查机构速度（线速度 `<0.008 m/s`、角速度及铰链/旋转速度 `<0.05 rad/s`）；保持期间夹爪开度 `≥0.40 rad`，末帧达到配置自由开度减 `0.01 rad`。完整循环 v4 和航空螺栓 v5 的几何完成规则不自动套用到这两个独立版本。

使用及初态见[单向操作采集](../../server/docs/single-direction-collection.md)，在线判定见[抽屉拉起](../../server/simulation/scripted_tasks/drawer_open.py)、[抽屉放回](../../server/simulation/scripted_tasks/drawer_close.py)、[锁杆任务](../../server/simulation/scripted_tasks/locking_rod.py)，录后判定见[采集验证器](../../server/collection/recording_validation.py)。历史默认布局录像不代表本次合并版本或任意随机布局验收通过。

## 4. 预检几何、在线规划和失败

### 4.1 三个不同检查阶段

1. **布局入池**：`LayoutPool` 校验 model/catalog/manifest/native report 哈希、独立布局声明及 `task-box-operation-clearance/1`。报告需有 `passed=true`、`initial_states_preserved=true`、匹配的原模型/源模型哈希。v2 包另验证路径、资产哈希闭包与搬迁等价证明；迁移保留历史报告原字节，不是重新原生验收。
2. **本条初态**：执行器使用 `NativePoseProbe.validate_initial` 检查 8 维有限关节值、六轴限位、当前初始碰撞；结果明确标为 `scope="initial_state_only"`、`checked_poses=1`。不会在开始时把整个任务预演两遍。
3. **阶段内规划**：按任务与阶段决定检查范围。空手接近先尝试直线，再在预算内搜索；插头 extract/insert 和抽屉 lift/return 在取得原生实测状态时调用 `check_operation` 做名义受约束通道扫描。锁杆的接近/换抓接近和撤离调用环境规划，但 lower/raise 使用机构摆动轨迹，loosen/tighten 使用旋拧反馈；当前生产流程没有在这四个阶段入口逐一执行同类 `check_operation` 扫描。锁杆另有 `preflight_paths()` 供完整候选路径诊断使用，不能把该函数存在或历史预检通过当成当前每条生产任务已经审核全部阶段轨迹。

当前初态 `pose_collision` 默认：臂/腕非预期穿透 `>0.5 mm` 拒绝；参与运动的机构与环境非预期穿透 `>0.2 mm` 拒绝。内部四连杆接触、明确的手指抓取接触及自身配合副按策略豁免；这些不是一个全场景“零接触”保证。

自由运动默认正间隙 `2 mm`；手指在接近/撤离接触邻域的间隙可渐变，腕/臂约束继续存在。搜索边采样步长为 `0.02 rad`，最终曲线检查按 `0.006 rad` 的导数界限生成采样点；插头受约束轴向检查最多 `2 mm` 位移增量并按关节变化 `0.02 rad` 细化，抽屉通道按 `0.006 rad` 细化。独立开爪函数是否也受相同间距界限制，要按对应轨迹实现判断。

以上均为**有限离散采样**，不是连续碰撞检测证明。正间隙不足也会被拒绝；例如 `start_in_collision` 不能一概翻译为“已经发生穿透”。搜索耗尽表示预算内没找到，不证明物理不可达；首个失败接触对也不是全模型最小间距。

源码：[布局池](../../server/backend/space_arm_platform/collection_layouts.py)、[初态/接触分类](../../server/simulation/scripted_tasks/task_initialization.py)、[自由运动查询](../../server/simulation/scripted_tasks/collision_motion.py)、[搜索采样参数](../../server/simulation/scripted_tasks/obstacle_planner.py)、[受约束通道](../../server/simulation/scripted_tasks/operation_clearance.py)、[锁杆阶段规划与诊断候选路径](../../server/simulation/scripted_tasks/locking_rod.py)、[执行器](../../server/collection/run_scripted_task.py)。

### 4.2 不同任务不能共用一个“超载即失败”说法

| 项目 | 当前实现 |
|---|---|
| 抽屉 | 默认腕力范数 `>80 N` 或力矩范数 `>8 N·m` 连续 `0.1 s`，失败 `wrist_load_limit`；参数可由诊断入口覆盖 |
| 三插头 | 配置有 `40 N / 4 N·m`，但 `overload_stop_enabled=False` 默认关闭超载中止及相关接触恢复。不能把这两个数称为默认自动成功上限；若显式启用，超载累计 `0.15 s` 才触发失败 |
| 两种螺纹任务 | `ThreadedMotionConfig` 要求 `overload_stop_enabled=False`，启用会被配置拒绝；腕力峰值仍记录 |
| 腕力有效性 | 上述控制器在仿真时间超过 `0.2 s` 后仍无有效腕力时失败 `wrist_sensor_unavailable` |

还包括初态不合格、规划失败、物体滑脱、夹爪闭合但未夹住对象、机构状态缺失、阶段超时和总仿真超时等任务失败。原生数值/运行环境异常与普通任务失败分开处理。

执行器当前正常成功返回 `0`，清理完成的任务失败/初态业务拒绝返回 `1`；初态拒绝发生在恢复已有物理图之前，可保留健康 worker。真实异常进入 worker 异常处理并退役，不能仅为复用速度忽略异常。

时间上限有两种时钟，且入口默认值不同。**直接调用单条执行/录制入口**时，普通任务默认总仿真上限 `360 s`，两种螺纹任务 `900 s`。**当前按布局批次**默认每次尝试墙钟上限 `600 s`，下游另计算仿真上限：普通任务 `min(450, 600)=450 s`，螺纹任务 `min(900, 600)=600 s`。螺纹基础阶段超时默认 `120 s`，部分阶段根据路径另算。仿真秒与墙钟秒不能互换；超时失败须看实际入口及快照，不能把单条入口的 `900 s` 写成当前批次默认预算。入口覆盖规则见[采集配置参考](collection-configuration.md)。

源码：[抽屉失败](../../server/simulation/scripted_tasks/drawer_lift.py)、[插头失败与开关](../../server/simulation/scripted_tasks/plug_insert.py)、[螺纹失败与配置](../../server/simulation/scripted_tasks/threaded_motion.py)、[默认仿真时长](../../server/simulation/scripted_tasks/threaded_tasks.py)、[worker 生命周期](../../server/backend/space_arm_platform/dynamics_worker.py)、[批次墙钟预算](../../server/collection/sequence.py)。

## 5. 状态字段按所在层读取

| 所在层与字段 | 含义 | 读取注意 |
|---|---|---|
| 控制器/动力学 `result=success/failed` | 动作判据是否达成 | 与相机、编码器审核分开 |
| episode `outcome` | `success/failure/aborted/unknown` 等保存结果 | `success` 不是完整审核报告 |
| episode `dataset_status=complete` | writer 完成封装 | 之后仍可能审核失败并被隔离；不能仅按该字段训练 |
| attempt `status=accepted` 且 `full_validation_pending=true` | 快速检查通过，加入待完整审核队列 | 这时 lower-level `accepted_episodes` 也可能已有该路径，仍是待审核 |
| sequence job `pending_audit` | 对应已采集成功、完整审核尚未通过 | 是调度 `TERMINAL` 状态，含义是无需重新采动作；不是审核通过 |
| sequence job `accepted` | 完整审核和相关布局/机位验证通过 | 仅授予现有检查范围内的通过结论 |
| sequence job `skipped` | 布局准备失败、初态/动作失败、墙钟超时、审核失败或尝试异常等 | 必须看 `category/reason/error`；不是“没有记录”，证据可能在 `rejected/` |
| `cancelled` | 用户取消批次或当前尝试 | 停止时不会自动把剩余 `pending_audit` 升级成 accepted |
| `interrupted` / `infrastructure_failed` | 中断或基础设施问题 | sequence 恢复时已开始的受影响尝试标记为 interrupted 并隔离，不隐式重做该条 |
| 批次 `auditing` / `complete` | 动作阶段结束后审核 / 整个计划处理结束 | `complete` 可以包含 skipped，不表示全部计划成功 |

`progress.json` 中 `collected_successes = accepted + pending_audit`；`completed_attempts` 包括 `accepted/pending_audit/skipped/interrupted`。因此“完成 100 条尝试”“采到 100 条成功记录”“100 条完整审核通过”是不同统计量。

正常流程先关闭会话，再完整审核队列。审核失败会移除对应 accepted 路径并将 sequence job 改为 `skipped`，按布局入口把不合格 episode 移入 `rejected/`，保留原因。手动重跑一个只读审核函数不会自动修改批次状态；恢复整个批次也不是一个纯审核命令。

源码：[sequence 状态及计数](../../server/collection/sequence.py)、[attempt 快检/延后审核](../../server/collection/collect_scripted_tasks.py)、[episode 结果模型](../../server/backend/space_arm_platform/models.py)。

## 6. 数据契约与输出目录

当前脚本录制入口要求双相机 RGB、`50 Hz`；不是任意一套 recorder 配置都能用于任务审核。动力学 `400 Hz`、脚本控制与腕力 `200 Hz`，状态/动作及每路图像 `50 Hz`。遥操作 IK 默认 `100 Hz` 是另一条控制节拍，不应写成脚本任务的运行频率。

默认相机为 `teleop/camera/spacecraft_overview` 和 `teleop/camera/sarm_wrist_cam`，各 `1024×1024`。`observation.state`/`action` 为 7 维：六轴关节角加归一化夹爪开度；action 是控制目标，不是测量位置或关节力矩。

```text
<批次>/
├── config.snapshot.json / plan.json / source-scene.json
├── pool.snapshot.json / inputs.json / launch-provenance.json
├── progress.json
├── attempts/<任务>/<轮次>/<布局>/
│   ├── summary.json
│   └── <attempt>/
│       ├── scene.json / validation.json / recording.log
│       └── recording/
│           ├── summary.json
│           └── dynamics/summary.json
└── <任务>/<轮次>/<布局>/
    ├── episodes/<episode>/
    │   ├── metadata.json
    │   └── lerobot/
    │       ├── data/chunk-*/*.parquet
    │       ├── meta/                         info/platform/tasks/episodes 等元数据
    │       ├── auxiliary/context/            原始上下文、来源数值摘要
    │       ├── auxiliary/cameras/            帧身份、会话、时刻与相机元数据
    │       ├── high_frequency/wrist_wrench/   200 Hz 原始腕力样本
    │       └── videos/<相机特征>/chunk-*/*.mp4
    └── rejected/                             本布局隔离的失败/不完整录制
```

精确路径由任务计划的 `relative_output` 与本次 attempt ID 决定；`rejected/` 与该布局的 `episodes/` 并列。

默认 `compact`、`save_raw_camera_images=false`、`save_dynamics_trace=false`。缺少 `steps.jsonl` 或逐帧 JPEG 不表示丢失当前格式的数据；应使用 `compact_storage.read_observations/read_wrench`。默认也不会保存可供完整 400 Hz 接触追审的全量轨迹。H.264 视频完成解码证明编码结构/帧时序可读，不证明编码无损或图像语义正确。

普通录像配置 `max_frames=18000`；两种螺纹任务会按该条最大仿真时长提高帧上限至至少 `ceil(max_sim_seconds×50)+1`，覆盖值写入报告。直接录制入口的 `900 s` 默认对应 `45001` 帧；当前按布局批次的 `600 s` 仿真预算对应 `30001` 帧。批次实际选择 [recording-veryfast.json](../../server/config/recording-veryfast.json)，不是仅凭 [recording.json](../../server/config/recording.json) 推断实际编码设置。

源码：[共享时钟](../../server/backend/space_arm_platform/sampling.py)、[录制配置约束/螺纹帧数覆盖](../../server/collection/record_scripted_task.py)、[compact 格式](../../server/backend/space_arm_platform/compact_storage.py)、[运行来源](../../server/collection/runtime_provenance.py)。

## 7. 快速检查与完整检查到底检查了什么

| 检查项 | `validate_recording_fast` | `validate_recording` 及采集器附加绑定 |
|---|---|---|
| outcome 成功、dataset complete、总帧数一致 | 是 | 是 |
| 保存状态帧号连续、仿真纳秒时间落在 50 Hz 网格 | 是 | 是 |
| 当前任务阶段/终态 | 插头/螺纹走各任务审核；抽屉只做较小的终态检查 | 包含版本对应阶段与任务专用要求 |
| compact 数值来源摘要 | 由共同 `read_observations` 执行 | 同样执行，另比较状态/动作/原关节/腕力列 |
| 布局模型/随机变体哈希 | 未执行完整模型绑定 | `validate_recorded_layout_model`；调用方再对预期布局选择审核 |
| 视频 | 检查两文件头部帧数、尺寸、帧率 | 两文件逐帧解码，数量匹配，PTS 相邻间隔正确 |
| 双相机每帧身份配对 | 未完整遍历审核 | 每个 camera×frame 恰好一次、session/time 一致、1024²，按设置检查原图 |
| 200 Hz 腕力旁路 | 未完整审核 | 时间间隔 `5,000,000 ns`、首尾与状态对齐；N 条状态对应 `4×(N−1)+1` 个 compact 腕力样本，摘要与共有时刻值匹配 |
| 数值误差检查 | 非完整逐列比较 | 状态/动作列 `assert_allclose` 使用 `atol=1e−7`；腕力列 `atol=1e−5`，部分高频比较显式 `rtol=1e−6`；未显式指定 rtol 的调用仍遵循 NumPy 默认值 |
| 机位选择/布局选择与批次预期一致 | 延后 | 采集器在 full 或 deferred audit 时调用相应绑定函数 |
| 图像颜色对象正确、可见性/遮挡/清晰度 | 否 | 否；元数据相符不等于像素语义审核 |
| 完整 400 Hz 非预期接触轨迹 | 否 | 否；需要独立接触诊断证据 |

当前实现期望每条 episode 有两个 MP4 文件，不是一个支持任意视频分片布局的通用 LeRobot 验证器。验证大量依赖 Python `assert`，**执行审核时不能使用 `python -O` 或启用 `PYTHONOPTIMIZE`**。

## 8. 使用当前验证器

以下示例从 `server/` 执行，使用已配置的项目 Python 环境。示例是手动核查入口；本次文档整理未运行这些采集/仿真命令。

### 8.1 只读复核一条既有 episode

现有完整审核入口是 Python 函数，没有另一个通用的 `validate_recording.py` 命令。下面读取已有 attempt summary 中的预期布局/机位，把审核结果写入新的外部文件，不回写正式批次或 episode：

```bash
PYTHONPATH=.:backend:../ue-adapter/Adapters \
python - /absolute/attempt-summary.json /absolute/episode /absolute/new-audit.json <<'PY'
import json
import sys
from pathlib import Path
from collection.recording_validation import validate_recording
from space_arm_platform.collection_layouts import validate_episode_layout
from space_arm_platform.overview_camera_records import validate_episode_overview_camera

summary_path, episode, output = map(Path, sys.argv[1:])
summary = json.loads(summary_path.read_text())
entry = next(e for e in summary['attempts'] if e.get('episode_id') == episode.name)
try:
    report = validate_recording(episode, summary['task'])
    if 'layout_randomization' in entry:
        report['layout_randomization'] = validate_episode_layout(episode, entry['layout_randomization'])
    if 'overview_camera' in entry:
        report['overview_camera'] = validate_episode_overview_camera(episode, entry['overview_camera'])
except Exception as error:
    report = {'passed': False, 'error': f'{type(error).__name__}: {error}'}
report.update(episode=str(episode.resolve()), attempt_summary=str(summary_path.resolve()))
with output.open('x', encoding='utf-8') as stream:
    json.dump(report, stream, ensure_ascii=False, indent=2)
raise SystemExit(0 if report['passed'] else 1)
PY
```

保留审核时的源码/环境/原报告哈希，勿覆盖历史 `validation.json`。这里复核的是保存数据，不会重新做动作，也不会把 `pending_audit` 自动改为 `accepted`。如需维护批次状态，应使用采集器已实现的审核流程，明确其是否还会执行未完成任务。

### 8.2 原生轨迹接触审核

`validate_task_trajectory_contacts.py` 的新运行模式会执行真实 BSK/MuJoCo 任务，在每个动力学 tick 记录接触并检查覆盖、动作阶段与允许接触策略；它不录制相机。只分析已有专用诊断产物时：

```bash
python tools/diagnostics/validate_task_trajectory_contacts.py \
  --model /absolute/model-used-by-diagnostic.xml \
  --task drawer_cycle \
  --analyze-existing /absolute/native-contact-diagnostic \
  --output /absolute/new-contact-analysis
```

输入须是该工具对应的 `names.json`、`run/summary.json`、`contacts-400hz.jsonl.gz` 等诊断结构，不是随便一个普通 episode。旧 `state-contact-200hz.jsonl.gz` 只能声明历史 200 Hz 覆盖，不能补称 400 Hz 已验收。

综合 `accepted` 要同时满足任务成功、轨迹策略通过、完整覆盖、所需动作阶段完整且顺序正确、原生返回码为零。`--ignore-free-bolt` 会明确豁免相关灰色自由螺栓接触，必须在验收范围中披露。即使 400 Hz 全部采样通过，也不是连续时间碰撞、接触力上限或真实硬件安全证明。

源码：[ContactAccumulator / analyze_existing / main](../../server/tools/diagnostics/validate_task_trajectory_contacts.py)。

### 8.3 容易用错的现有工具

| 工具 | 当前范围/限制 |
|---|---|
| [validate_capture_pipeline.py](../../server/tools/diagnostics/validate_capture_pipeline.py) | Windows `UnrealEditor-Cmd.exe` 的保存状态回放，代码固定 30 Hz，用来隔离测试真实 GPU/录制链路；不是当前 Linux 50 Hz 原生动力学六任务端到端验收 |
| [audit_parallel_capture.py](../../server/tools/diagnostics/audit_parallel_capture.py) | 旧并行 benchmark 结构，直接读取 `steps.jsonl/captures.jsonl` 与旧平台 JSON 列；允许规定时长的诊断超时记录，不是当前 compact 六任务训练资格审核器 |
| [validate_task_box_native.py](../../server/tools/diagnostics/validate_task_box_native.py) | 固定 `sarm_task_box_module.xml` 的 Basilisk smoke；没有完整随机布局池/六任务轨迹认证的含义 |
| [validate_plug_retention.py](../../server/tools/diagnostics/validate_plug_retention.py) 等机构夹具 | 验证特定保持力、螺纹或机构工况；不能替代某条完整任务、双相机录制与训练数据审核 |

## 9. 旧文档冲突如何处理

历史正文保留当时条件，不应为了“统一数字”修改旧数据。当前维护与新验收按本表定位源代码，历史复核按保存的版本走相应兼容分支。

| 旧说法/易误读位置 | 当前解释 |
|---|---|
| [自动采集避障](../../server/docs/ALL_TASK_COLLISION_PLANNING.md) 列出 drawer v6、plug v14、bolt v4、rod v3 | 对应早期直线接近合并；当前是本文件第 2 节的 v7/v17/v5/v4 |
| 旧插头最终速度 `<0.002 m/s`、`<0.03 rad/s`，或“最终静止标准不变” | 当前 v15/v16/v17 的 verify 是 0.2 s 几何完成；原有历史速度检查只应用于相应旧版本，操作阶段另论 |
| 旧圆柱接触区 `0.005 rad/s`、接近全开才撤离 | v16 已恢复与其他插头一致的松爪流程，v17 延续；不能据旧段落判断当前每条的实际控制 |
| “末尾等 2 秒”或只看基类 `hold_s` / `verify_hold_s` | 插头当前完成确认 0.2 s；抽屉/航空螺栓/锁杆各自配置为 0.3 s，同时保留各阶段特定条件 |
| 快速螺纹 profile 开爪 `0.45 rad`，因此必须到 `0.44 rad` 才成功 | 当前完成净空下界为 0.40 rad；0.45 是控制目标，旧 previous version 审核仍按原配置开度减 0.01 |
| [螺纹采集说明](../../server/docs/THREADED_TASK_COLLECTION.md) 的较长单录墙钟预算 | 不能覆盖当前按布局批次 `task_wall_timeout_seconds=600`；以入口和配置快照判断 |
| 目录中有 `accepted_episodes`、任务成功或 `dataset_status=complete` | 必须同时读 pending 标志、sequence job 状态、最终完整审核报告及隔离记录 |

本文不是全场景认证清单。每次对外报告应给出任务与布局范围、源码/构建来源、完整任务还是短测、原生还是 UE 实录、全解码与接触覆盖情况，以及尚未覆盖的任务/平台/长稳条件。固定布局五任务通过不能表述成六任务或整个随机布局池通过。
