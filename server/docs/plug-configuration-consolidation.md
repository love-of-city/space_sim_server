# 三类插头的当前配置与模块边界

当前执行版本为 `plug-insert/19-axis-contact-continuity`，配置快照使用 `plug-motion-config/2`。源码只有一套当前控制器；历史阶段与验收规则由记录层解释，不通过旧版本号切换运动算法。配置整理保持三类插头原有有效默认值，仅退出当前不使用的 `hold_s` 字段。

## 唯一入口与职责

以下路径相对于 `server/`。

| 文件 | 职责 |
|---|---|
| `simulation/scripted_tasks/plug_motion_config.py` | `PlugInsertConfig`、公共默认值、`MOTION_LIMITS`、`TASK_RULES`、覆盖解析、校验与完整快照 |
| `simulation/scripted_tasks/plug_profiles.py` | 三类插头的身份、几何、键向、专属默认值和落座判定差异 |
| `simulation/scripted_tasks/plug_insert.py` | 当前11阶段控制器，仅使用已解析配置 |
| `simulation/scripted_tasks/plug_contact_feedback.py` | 名义接触参考、有限预载、速度反馈和限幅 |
| `simulation/scripted_tasks/plug_clearance.py` | 实际开爪净空和撤离路径检查 |
| `simulation/scripted_tasks/task_initialization.py` | 生成初态并冻结有效配置 |
| `simulation/scripted_tasks/native_geometry_probe.py` | 隔离的原生几何查询，沿用任务配置 |
| `collection/recorded_task_contracts.py` | 保存记录的任务版本、阶段与规则，不实现历史控制器 |
| `collection/recording_validation.py` | 批采、单条与发布入口共用的数据验收 |
| `tools/benchmarks/optimize_plug_motion.py` | 有界候选实验，保留输入和配置派生关系 |

依赖方向是控制器使用配置模块，配置模块使用 profile。插头配置不再借用 `DrawerCycleConfig`；修改抽屉默认值不会隐式改变插头运动限制。内部调用已迁移，不提供旧配置导入壳。

## 公共默认值

| 参数 | 当前值 |
|---|---:|
| `arm_speed_rad_s` / `tracking_speed_rad_s` | 0.35 / 0.35 |
| `linear_speed_m_s` / `retract_speed_m_s` | 0.10 / 0.10 |
| `grasp_approach_speed_m_s` | 0.025 |
| `insert_speed_m_s` / `extraction_speed_m_s` | 0.025 / 0.025 |
| `carry_speed_m_s` | 0.05 |
| `carry_acceleration_m_s2` / `carry_jerk_m_s3` | 0.15 / 0.8 |
| `gripper_speed_rad_s` / `release_speed_rad_s` | 0.2 / 0.2 |
| `release_open_rad` | 0.65 |
| `transport_height_m` / `pregrasp_distance_m` | 0.245 / 0.045 |
| `grasp_relative_speed_m_s` | 0.015 |
| `insert_slowdown_ratio` / `insert_resume_ratio` | 0.75 / 0.90 |
| `insert_creep_delay_s` | 0.25 |
| `insert_creep_ratio` / `insert_creep_speed_scale` | 1.25 / 0.18 |
| `insert_creep_force_fraction` / `insert_creep_torque_fraction` | 0.5 / 0.5 |

距离单位为米，角度为弧度，时间为秒；速度/加速度/加加速度对应每秒的一、二、三次方。高级接触参数的允许范围见[运动优化配置表](plug-motion-optimization.md#配置及可调范围)。

## 三类插头差异

| 规则 | 航空 | 导向 | 圆柱 |
|---|---:|---:|---:|
| 机械键向约束 | 有 | 有 | 无 |
| 尖端局部位置（m） | 0.080 | 0.095 | 0.095 |
| 抓取偏移Z（m） | -0.003 | -0.003 | -0.006 |
| 闭合轴 | 局部Y | 局部Y | 局部Y |
| `insert_orientation_tolerance_rad` | 0.004 | 0.004 | 0.008 |
| `extraction_lateral_gain_s_inv` | 4 | 3 | 3 |
| `extraction_velocity_gain` | 8 | 8 | 6 |
| 松爪前座隙门槛 | 配置值，默认1 mm | 配置值，默认1 mm | 配置值与0.1 mm取较小值 |
| 松爪前低速门槛 | 平移及完整角速度 | 无额外速度等待 | 平移及去除轴向自转后的角速度 |

圆柱的0.1 mm只用于进入松爪的时机，最终通用几何验收仍为1 mm。航空和导向不会因为字段集中而增加新的等待。profile中的几何、键向及完成规则不进入自动优化白名单。

`MOTION_LIMITS`保存固定轨迹限制：线加速度0.2 m/s²、角速度0.2 rad/s、角加速度0.4 rad/s²、关节加速度0.7 rad/s²，线/角/关节加加速度0.8/1.6/8，夹爪加速度0.4、加加速度1.6，接近减速距离20 mm。

`TASK_RULES`保存阶段规则：初始稳定1 s、抓取与落座确认各0.15 s、释放确认0.1 s、显式超载停机确认0.15 s、最终轴倾/键向0.01 rad。最终完成确认来自配置`completion_confirm_s=0.2`。当前`overload_stop_enabled=false`；40 N / 4 N·m仍作为接触推进许可等计算的基准，整理不会顺带启用超载停机。

## 解析与冻结

日常新配置的唯一解析入口是 `plug_motion_config(task_name, scene, **explicit)`，覆盖顺序为：

`公共默认值 → 插头类型差异 → scene.collection_config.plug_motion_parameters → 非空显式参数`

场景增量只接受白名单内的有限数值及合法范围；物理保持力、录制和批次计划分别由`config/plug_retention.json`、录制配置和`config/collection.json`管理，不灌入一个总参数文件。

`collection_config.plug_config_snapshot`是完整冻结结果，包含schema、规范任务名、任务行为版本、全部有效参数、固定运动限制、阶段规则、profile及规范JSON的SHA256。读取时校验哈希、字段完整性、任务/profile/规则匹配；参数冲突直接拒绝。它不会用当前默认值补齐一份不完整快照后声称恢复了旧配置。

实际关节刚度、保持力、源码、模型与原生库身份独立保存；参数快照不等同于完整运行环境。当前任务的`config`和`contact_feedback`继续用于结果核对，任务行为版本与配置schema分开管理。

优化器通过`candidate_scene()`显式派生新实验：校验原配置，保留未修改参数，写入新的完整快照，并记录源配置SHA、源输入是否已有快照及参数变化。原场景保持原字节；不会悄悄覆盖冻结配置。

## 历史记录与验证边界

历史记录的`hold_s`、任务版本与阶段由`recorded_task_contracts.py`按原契约读取，不映射为当前`completion_confirm_s`。`plug_insert`作为已支持的任务别名仍归一到圆柱；这不保留另一套控制器。

旧场景没有完整快照时，用它测试当前控制器属于新实验。场景里保存的v17来源不会让运行时切回v17，也不能证明当前配置复现了原始环境。已停止正式批次不原地迁移。

整理前的五个固定案例及圆柱双相机证据位于仓库外`artifacts/verification/plug-continuity-20261103/`；整理后的验证单独记录在`reports/code-reorganization-20261103-153614/`。配置一致性、控制参考等价、真实动力学成功、录像完整性分别核验，五个案例不代表全部150套布局。

完整动作与有界优化流程见[插头运动优化](plug-motion-optimization.md)，全仓入口见[项目结构地图](../../docs/project-map.md)。
