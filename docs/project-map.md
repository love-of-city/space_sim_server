# SpaceSim 项目结构地图

结构核查日期：2026-11-03。当前已完成任务配置/记录契约、采集验收、后端/网页、遥操作、Python原生绑定、渲染适配器与资产工具的职责整理；下文模块表反映当前实现。源码继续在 `src/space-sim/` 的稳定文件中迭代，退役实现由 Git 和整理前工作区快照保留。此次修改及验证范围见[代码整理实施报告](/home/spacesim-zmh/reports/code-reorganization-20261103-153614/REPORT.md)。

2026-10-31 的 UE 工程改进已从 `chore/ue-engineering-20261031` 集成主目录，并在主目录重建 UE/native。其历史验收见[工程改进实施报告](/home/spacesim-zmh/reports/ue-engineering-implementation-20261031/REPORT.md)。后续批次及实时运行状态以仓库外 `WORKSPACE.md` 和各批次 `progress.json` 为准；目录整合不等于所有平台已完成交付认证。

这份地图按职责维护；本机本次逐文件清单和完整文件树位于 `/home/spacesim-zmh/reports/project-structure-20261031/`。该盘点是时间点快照，后续新增文件以 Git 和实际磁盘为准。

工程改进范围与验收状态见[工程改进方案](ue-engineering-improvement-plan.md)。本页列出已经落地的目录和工具；[运行版本与构建来源](architecture/runtime-provenance.md)、[UE 与原生分层验证](guides/ue-validation.md)、[发布重建](guides/release-reproduction.md)说明使用方式和证据边界。提供工具入口不代表所有平台和发布检查已经通过。

## 工作区

```text
/home/spacesim-zmh/
├── WORKSPACE.md                    本机总入口
├── src/
│   ├── space-sim/                  Git 主工作树，已集成工程改进
│   └── space_sim_server/           README + 两个旧路径兼容软链接
├── data/
│   ├── README.md                   批次保留与来源说明
│   ├── collections/               正式采集批次，包含成功、失败、取消与审核状态
│   ├── layout-pools/               生成布局、模型、catalog、哈希及几何审核
│   └── models/                     模型权重等外部输入
├── experiments/                    独立实验代码与可复现输入、恢复来源
├── artifacts/
│   ├── build/                     构建暂存
│   └── verification/              独立验收、解压与实采验证产物
├── reports/                        维护和验收报告、清单、证据
├── releases/                       按组件保存的不可变交付包
├── archive/workspace-snapshots/    整理前源码/工作区恢复快照
├── envs/                           已安装 Python 环境
├── engines/UE_5.6/                 Unreal Engine 安装
├── worktrees/
│   └── ue-engineering-20261031/    同一Git仓库的改造工作树，保留实现与验收依据
├── migration/                      旧布局重编号计划的兼容链接
├── Library/Logs/                   UE 构建工具写入的用户级日志
└── UnrealEngine/UnrealTrace/        UE Trace 服务日志，不是第二份引擎
```

实际保留的三个正式批次 `layout1000-20261029`、`layout1000-20261030-1928`、`layout1000-20261031-0042` 均已停止，`progress.json` 为 `cancelled`，保留 `cancel.request` 和原始输入/数据。锁文件可能保留在磁盘，不能据此判断进程仍在运行。本次工程验收输出到 `artifacts/verification/ue-engineering-20261031/`，不续写这三个批次。

保留的原布局池：`data/layout-pools/random150-20261028/`。恢复证据包括 `experiments/recovery-20261030-optimized-runtime/` 和 `archive/workspace-snapshots/workspace-before-ue-engineering-20261031/`；本轮实施报告在 `reports/ue-engineering-implementation-20261031/`。环境包含 `spacesim`、`spacesim-geometry`、`epic-browser`；目录名不足以证明可删除，需查调用依赖。

兼容关系：

```text
src/space_sim_server/space_sim_server     -> ../space-sim/server
src/space_sim_server/space_sim_UE_Adapter  -> ../space-sim/ue-adapter
```

它们只让旧批次元数据的绝对路径继续有效。正式源码入口为 `src/space-sim/`；本轮开发使用上述工作树；最终完整录制验收显式使用主目录源码和重建产物。工作树共享 Git 历史，是隔离修改和构建的入口，不是通过复制目录维护另一套项目版本。

## Git 仓库顶层

| 目录/文件 | 职责 |
|---|---|
| `.github/workflows/ci.yml` | CI 构建、检查和测试任务 |
| `.github/workflows/native-validation.yml`、`ue-validation.yml`、`release-validation.yml` | 受控 runner 上的原生、UE/GPU、发布验证入口；远端 runner 和分支保护需另行配置 |
| `.github/CODEOWNERS`、`pull_request_template.md` | 维护归属和变更审查模板 |
| `.github/*settings*.json`、`branch-protection.json` | 仓库管理配置声明 |
| `.gitattributes` | 文本与 Git LFS 资产属性 |
| `.gitignore` | 排除运行目录、环境和生成产物 |
| `AGENTS.md` | 自动化协作的项目边界与命名规则入口 |
| `CONTRIBUTING.md` | 开发、集成、验证和数据管理约定 |
| `CHANGELOG.md` | 可审查的项目变更记录 |
| `README.md` | 对外项目说明和启动导航 |
| `docs/architecture/system.md` | 系统架构 |
| `docs/guides/` | 仿真采集全流程、数据保留、本机运行与维护 |
| `docs/reference/` | 物理模型与配置、各采集入口参数、任务/数据验收及验证缺口；关键规则链接到实际实现 |
| `docs/templates/` | 批次任务对齐记录模板；填写后的具体报告放在仓库外 |
| `docs/project-layout.md` | 路径与 Git/产物归属边界 |
| `docs/project-map.md` | 本结构地图 |
| `docs/naming-conventions.md` | 后续文件与目录命名规则 |
| `manifests/environments/` | Python 来源快照、Linux/Windows 运行环境状态和 UE 引擎兼容策略；不将未重装的环境声明为已认证锁 |
| `manifests/assets/ue-cook-base-assets.json` | 527 个基础 UE 资产的 SHA、明确 Cook 目录和动态加载资源；不自动纳入本地布局资产 |
| `manifests/schemas/collection-layout-pool-v2.schema.json` | 可搬迁布局池、相对路径、文件哈希和原验收迁移证明契约 |
| `manifests/schemas/{collection-provenance-v1,component-identity-v1,ue-build-manifest-v1}.schema.json` | 批次来源、组件身份及 UE 构建身份的数据契约 |
| `server/` | API、网页、任务采集、权威动力学、模型、原生计算 |
| `ue-adapter/` | Python 状态协议适配、UE渲染工程和运行插件 |
| `tools/release/` | 两类交付包的构建、启动器及测试源码 |

## server：平台和权威动力学

```text
server/
├── backend/space_arm_platform/      API、任务/进程管理、记录与数据接口
├── collection/                     正式采集调度与单条录制
├── simulation/                     Python 仿真装配、控制、规划、状态读取
│   ├── components/                Basilisk 功能组件
│   ├── model_preparation/         任务模型的物理结构准备
│   ├── scripted_tasks/            唯一当前专家、路径规划与图复用
│   ├── evaluation/                原始动作冻结、固定回放和独立物理评分
│   ├── scene_library/             场景目录、装配、坐标和证据验收
│   ├── orbital_rendezvous/        远距交会与交接
│   └── near_field/                近距导航与执行
├── native/                         C++ 动力学与计算加速源文件
├── frontend/                       网页控制台与页面逻辑
│   ├── pages/                     采集、数据集、用户管理页面
│   └── tests/                     网页模块测试
├── signalling/                     WebRTC / Pixel Streaming 信令
│   └── tests/                     信令与访问控制测试
├── config/                         采集、录像、保持力和轨道配置
│   └── orbital/                   点质量、J2和阻力示例
├── contracts/                      控制/状态 JSON Schema
├── deploy/                         部署配置模板和固定工具版本
├── model/                          MJCF、URDF、网格、CAD和来源清单
├── third_party/mujoco/             原生编译头文件、配套文件和许可
├── scripts/                        平台启动、环境选择、CI与部署脚本
├── tools/                          用户可调用的采集/资产/诊断/测速工具
│   ├── assets/                    构建、转换、布局池准备与UE导入
│   ├── benchmarks/                有明确测量条件的性能测试
│   ├── diagnostics/               几何、轨道、录像、接触检查与预览
│   └── deployment/                部署验证与认证迁移
├── tests/                          Python 业务、原生与集成测试
├── docs/                           服务端专题设计与操作文档
├── pyproject.toml                  Python 包、依赖和工具配置
├── start_deployment.sh/.cmd        Linux / Windows 日常部署入口
├── stop_deployment.sh/.cmd         对应停止入口
├── show_deployment_access.sh/.cmd  访问地址查询
├── run/                            本机状态与已构建原生库，非源码
├── logs/                           运行日志，非源码
└── reports/                        本机诊断输出，非源码
```

另有 `start/stop_local_visualization.cmd`、`start/stop_remote_visualization.cmd` 和 `show_remote_access.cmd`，服务于已有本地/远程可视化流程。`.local/`、`*.egg-info/`、缓存、`MUJOCO_LOG.TXT` 是环境/运行产物，不是业务模块。

### 采集入口：必须分清层次

| 文件 | 实际职责 |
|---|---|
| `tools/collect.py` | 日常批采 CLI 入口 |
| `config/collection.json` | 启用任务、布局轮次、时间种子、初态范围和资源限制 |
| `collection/sequence.py` | 冻结计划；布局洗牌；依次调度；取消、失败跳过、恢复和批末审核 |
| `collection/runtime_provenance.py` | 冻结实际源码、原生库、Python 环境、UE 构建和输入指纹；恢复及逐条录制拒绝混用实现 |
| `collection/collect_scripted_tasks.py` | 单任务批采、共享 UE/动力学会话和审核调度 |
| `collection/recording_validation.py`、`recorded_task_contracts.py` | 唯一录制验收入口；按保存任务版本解释阶段与历史完成规则 |
| `collection/record_scripted_task.py` | 单条 UE 双相机 / LeRobot 录制、启停边界和最终封装；首帧前验证实际 UE 进程及模块来源 |
| `collection/run_scripted_task.py` | 单条权威物理任务循环，装配/重置图、运行控制器并写动力学报告 |
| `collection/dynamics_worker.py` | 常驻采集动力学入口，创建图缓存并设置优化默认值 |
| `backend/space_arm_platform/dynamics_worker.py` | 常驻进程的请求邮箱、身份检查、资源上限与退休机制 |
| `collection/collect_parallel.py` | 隔离多进程并行采集及资源统计；不是当前顺序批次入口 |

当前调用链：

```text
tools/collect.py
  → collection/sequence.py
  → collection/collect_scripted_tasks.py
  → collection/record_scripted_task.py
  → collection/dynamics_worker.py
  → collection/run_scripted_task.py
  → simulation/scripted_tasks/* + simulation/components/*
  → simulation/local_mujoco_stepper.py
  → native/local_mujoco_stepper.cpp 构建出的共享库
```

渲染与记录分支经 `capture_transport.py`、`bsk_render_adapter`、UE Runtime、后端 recorder/writer 输出图像和数据；UE 不替代权威动力学。

### backend/space_arm_platform：平台接口和数据管理

| 模块组 | 代码职责 |
|---|---|
| `main.py`、`app.py`、`models.py` | CLI 启动、FastAPI 装配/lifespan、API/仿真数据模型 |
| `platform_config.py`、`operator_sessions.py` | 平台配置和操作者会话状态 |
| `auth_api.py`、`scene_api.py`、`episode_api.py`、`operator_api.py`、`media_api.py` | 认证、场景、录制、操作权和媒体路由 |
| `task_catalog.py` | 纯任务目录：规范ID、别名、家族与显示名；默认采集集合和发布产品子集分别声明 |
| `auth.py`、`web_security.py`、`stream_access.py` | 用户会话、Web边界、视频访问令牌 |
| `collection_api.py`、`collection_jobs.py` | 网页采集 API 和后台采集任务管理 |
| `collection_inputs.py`、`collection_layouts.py`、`layout_trajectories.py` | 固定模型/布局池输入验证、轨迹验收与哈希绑定 |
| `collection_processes.py`、`collection_runtime.py`、`dynamics_worker.py` | 进程所有权、UE会话与动力学worker生命周期 |
| `collection_randomization.py`、`orbit_randomization.py`、`overview_camera.py` | 随机化配置、轨道相位和当前相机采样 |
| `overview_camera_records.py`、`overview_camera_calibration.py` | 历史相机记录校验/恢复及纯标定几何；新建相机不切换历史算法 |
| `scene_runtime.py`、`scene_targets.py`、`task_box.py` | 场景实例与目标初态 |
| `simulation_hub.py`、`protocol.py` | API与仿真的双向网关和消息封装 |
| `capture_lifecycle.py`、`capture_receiver.py`、`recorder.py` | 按需采集启停、图像接收、权威状态/图像配对 |
| `lerobot_capture.py`、`dataset_worker.py`、`streaming_video.py` | LeRobot 写入、隔离编码/封装、流式视频 |
| `compact_storage.py`、`wrench_recording.py`、`sampling.py` | 紧凑数据表、腕力旁路与多频率采样时序 |
| `dataset_view.py`、`jobs.py`、`tasks.py` | 数据集查询、封装后台任务、持久任务定义 |
| `policy_api.py`、`policy_observation.py` | 外部策略接入与在线RGB/状态配对；不是训练器 |
| `ik_initialization.py`、`preparation_planning.py` | 隔离初态IK和起始轨迹筛查 |
| `safety.py`、`joint_limits.py`、`gripper_representation.py` | 控制权限/限幅、关节限位、训练用夹爪表示 |
| `camera_defaults.py`、`control_defaults.py`、`recording_config.py` | 共用默认值与录制配置校验 |
| `lighting.py`、`orbital_physics.py` | 可序列化光照/轨道配置 |
| `linux_deployment.py`、`linux_processes.py`、`linux_public.py`、`linux_scene.py`、`unreal_runtime.py` | Linux部署、子进程、外网网关、场景与UE路径解析 |

### simulation：物理与控制

| 模块/目录 | 职责 |
|---|---|
| `assembly.py`、`physics_ports.py`、`physics_clock.py` | 组件装配、类型化物理消息、确定性时钟 |
| `local_mujoco_stepper.py`、`local_mujoco_bindings.py` | 分别负责BSK步进/状态发布、MuJoCo C ABI加载与ctypes绑定 |
| `local_dynamics_config.py` | 不依赖 Basilisk 的物理步长/发布节拍配置，供原生步进和纯图兼容检查共用 |
| `compiled_model_cache.py`、`publication_scene.py` | 编译模型内容指纹缓存、发布副本精简 |
| `mujoco_runtime.py`、`mujoco_plugins.py`、`native_acceleration.py`、`native_integration.py` | 绑定Basilisk已加载的运行库、插件和源码匹配的原生库，配置积分 |
| `native_posture.py`、`native_plug_retention.py` | 原生姿态计算与保持力接口 |
| `components/clock.py`、`dynamics_core.py`、`orbit.py`、`orbital_gravity.py`、`drag.py` | 时钟、动力学、轨道、引力、阻力组件 |
| `components/attitude.py`、`arm_reference.py`、`teleop_ik.py` | 姿态、机械臂参考和遥操作IK组件 |
| `components/render.py`、`observation.py`、`plug_retention.py`、`fastener_demo.py` | 渲染/观测、保持力和演示装配 |
| `orbital_configuration.py`、`orbital_environment.py`、`orbital_frames.py` | 轨道配置、BSK环境、局部/惯性坐标与外力接口 |
| `attitude_control.py` | 反作用轮与卫星姿态保持 |
| `serial_chain_kinematics.py`、`pose_ik.py`、`online_elbow_ik.py` | MJCF串联运动学、离线位姿IK与在线肘部偏好 |
| `arm_preparation.py`、`preparation_planner.py`、`preparation_contract.py`、`prepare_elbow_posture.py` | 准备状态机、规划、保存契约与隔离初态工作进程 |
| `gripper_control.py`、`joint_reference_publisher.py`、`reference_governor.py`、`reference_recovery.py` | 夹爪/关节参考发布、保护和受控恢复 |
| `control_client.py`、`teleop_grasp_unreal.py`、`gamepad_control.py` | 仿真通信、稳定遥操作CLI、手柄输入组合 |
| `teleop_scene.py`、`teleop_control.py`、`teleop_session.py` | 保存场景/轨道初态、Cartesian控制/IK、装配/重置/运行与清理生命周期 |
| `policy_control.py`、`policy_task_success.py` | 策略命令执行和物理任务成功判据 |
| `observation_capture.py`、`wrench_sampling.py`、`wrench_sensors.py` | 同步观测、独立腕力采样和传感器适配 |
| `plug_retention.py` | 插头接合、引导力和轴向干摩擦 |
| `independent_task_box_layout.py`、`capture_layout.py`、`collection_assets.py`、`socket_colors.py` | 独立布局、统一保存布局/资产校验、通用资产、每条颜色身份 |
| `task_box_layout.py` | 两板整体平移的离线诊断生成器；不是独立安装单元生产布局生成或采集校验入口 |
| `layout_package.py` | MJCF 资产引用解析、完整模型语义摘要和原生验收报告的搬迁等价证明 |
| `camera_lighting.py`、`sarm_model_profile.py` | 相机补光、模型关节/执行器命名映射 |
| `motion_diagnostics.py`、`runtime_performance.py`、`runtime_progress.py` | 运动/性能/启动诊断，不改物理时钟 |
| `mjscene_threadpool.py`、`fastener_demo_schedule.py` | 显式启用的线程池实验、演示扭矩计划，不应误认当前默认采集路径 |

### scripted_tasks：自动任务与复用

| 文件 | 职责 |
|---|---|
| `drawer_cycle.py` | 抽屉打开、关闭、松开和完成确认 |
| `drawer_lift.py` | 抽屉基础任务实现及历史诊断入口 |
| `plug_insert.py`、`plug_profiles.py` | 航空/导向/圆柱插头共用控制器及不同部件参数 |
| `plug_contact_feedback.py` | 将名义接触位姿与有限摩擦预载分离，绑定实际伺服刚度/保持力配置，输出整体限幅的执行器参考 |
| `plug_motion_config.py` | 插头公共参数、运动限制和固定阶段规则，统一解析/校验及plug-motion-config/2完整快照；不依赖抽屉配置 |
| `aviation_bolt.py` | 航空螺栓旋出和装配任务 |
| `locking_rod.py` | 锁杆松开、折叠、复位和锁紧任务 |
| `threaded_tasks.py`、`threaded_motion.py` | 螺纹任务公共组织与旋拧运动 |
| `physics_graph.py` | 本次优化的物理图兼容性判断、保留、重置和释放 |
| `task_initialization.py`、`native_geometry_probe.py`、`gripper_preflight.py` | 分别负责多任务初态/配置冻结、隔离原生几何查询、夹爪几何预检 |
| `planning_context.py`、`native_state.py` | 私有规划模型上下文、任务状态和接触读取 |
| `capture_transport.py` | 权威观测与渲染/录制传输连接 |
| `approach_path.py`、`carry_path.py`、`joint_path.py` | 接近、搬运、关节路径 |
| `collision_motion.py`、`environment_motion.py`、`obstacle_planner.py` | 接触约束、环境几何与避障规划 |
| `operation_clearance.py`、`plug_clearance.py` | 操作空间/插头通道净空检查 |
| `motion.py`、`motion_options.py`、`motion_timing.py`、`pose_tracking.py` | 运动公共逻辑、选项、时间规律和位姿跟踪 |
| `randomization.py`、`instructions.py`、`disturbances.py` | 初态/颜色等随机化、任务语言指令、可配置扰动 |

插头连续性独立验证入口为 `server/tools/benchmarks/optimize_plug_motion.py`，以冻结案例和有界候选参数调用生产单任务执行链；同目录 `plug_motion_metrics.py` 只读测量实际停顿及拔出/插入各阶段载荷。输入、命令、日志及推荐保存在仓库外的新诊断目录，不自动改写生产脚本或恢复正式批次。格式、判据与无解迭代方式见[插头运动优化](../server/docs/plug-motion-optimization.md)。

`model_preparation/bolt_storage_thread.py`、`captive_fastener.py`、`locking_cycle_contacts.py`、`threaded_task_initialization.py` 负责模型中螺纹、紧固件、锁杆接触及任务初态的结构准备。

### native、网页、信令与配置

| 文件/位置 | 职责 |
|---|---|
| `native/local_mujoco_stepper.cpp` | 原生物理步进、摩擦、接触读取、分离证明、外力汇总和episode reset |
| `native/posture.cpp` | 姿态/运动学加速 |
| `native/sarm_thread_sdf.cpp` | SARM 螺纹 SDF 插件；改几何需同步检查分离证明 |
| `native/sarm_thread_sdf_check.cpp`、`mjscene_probe.cpp` | 插件与原生场景检查 |
| `frontend/index.html`、`app.js`、`platform_shell.js`、`ui.js` | 网页入口、应用/导航与通用UI |
| `frontend/pages/collection.*` | 采集控制页面；随机化和布局状态由同目录模块负责 |
| `frontend/pages/datasets.*`、`dataset_filters.js`、`dataset_charts.js`、`dataset_playback.js`、`users.*` | 数据集页面、筛选、图表、双相机回放与用户管理 |
| `frontend/api_client.js`、`operator_session.js`、`scene_session.js` | API与登录失效处理、控制权连接、场景启停/重置请求生命周期 |
| `frontend/pixel_stream_session.js`、`video_diagnostics.js`、`task_catalog.js` | 视频连接/重连、诊断、共用后端任务标签/别名目录 |
| `frontend/arm_preparation.js`、`control_status.js`、`free_camera.js`、`gamepad_input.js` | 准备、控制状态、自由相机和手柄 |
| `frontend/initial_joint_angles.js`、`joint_angles.js`、`motion_diagnostics.js`、`wrist_wrench.js` | 初态/关节显示、运动诊断和腕力 |
| `frontend/*.css`、`vite.config.js`、`package*.json` | 样式、构建、依赖与锁定版本 |
| `signalling/server.mjs`、`origin.mjs`、`peer_departure.mjs` | 信令服务、来源验证、连接离开处理 |
| `config/recording.json`、`recording-veryfast.json` | 常规与显式veryfast流式录像设置 |
| `config/collection-base-assets.json` | 通用模型/UE资源的路径和SHA清单 |
| `config/plug_retention.json`、`task-instructions.json` | 保持力参数、任务指令模板 |
| `contracts/*.schema.json` | 状态/控制消息结构契约 |
| `scripts/ci_pytest.py`、`native-test-profile.json`、`ci-exclusions.json` | 基础/原生测试分层、显式平台节点排除；声明范围内出现 skip 即失败 |
| `tools/assets/migrate_layout_pool.py` | 将 v1 布局池复制为 v2 完整包；拒绝覆盖原件、未声明资产根和路径逃逸 |
| `tools/deployment/check_runtime_provenance.py` | 检查本机采集源码、原生库、环境与 UE 构建来源是否一致，不启动正式采集 |

资产共享几何位于`tools/assets/task_box_geometry.py`与`plug_collision_geometry.py`，构建CLI与诊断工具共用；正式构建不再导入实验CLI。`task_box_proxy_experiment.py`保留为独立碰撞几何A/B诊断工具，生成候选而不替换正式模型。

布局迁移用法、包内目录和交付边界见[可搬迁布局包](guides/portable-layout-pools.md)。运行时每次选样重新校验文件哈希，历史 native 报告保持原字节，不把路径迁移当成重新完成几何验收。

## 模型：输入、正式装配与实验资产

| 目录/文件 | 职责与边界 |
|---|---|
| `model/SARM/platform/` | 平台组合MJCF、姿态配置、机制碰撞网格和来源manifest |
| `model/SARM/platform/scenarios/scenario_sarm_grasp.py` | 原生场景装配脚本，仍是正式运行依赖 |
| `model/SARM/{mjcf,meshes,urdf,config,launch}/` | 独立机器人模型、网格、URDF与ROS导出配置 |
| `model/SARM_mjcf_20261012/` | 特定导入版本的MJCF及大量原始/分解网格、转换说明 |
| `model/task_box/module/`、`free_plugs/` | 任务盒模块和自由插头模型 |
| `model/ground_validation_satellite/` | 地面验证卫星源转换、机构/碰撞变体及相关预览/验收输入 |
| `model/nasa_lro_a/` | LRO模型、网格、来源与预览 |
| `model/任务盒_v1/`、`地面验证星模型.stp` | 原始 CAD 设计源文件；不是采集日志 |

保留批次的实际物理输入来自仓库外布局池，并按任务生成变体，不能把 `model/README.md` 中某个历史默认场景当作所有采集任务的实际模型。`*_trial`、`*_validated2` 和预览目录目前仍存在，是需进一步分类的历史命名，不代表本次已全部规范化或可以直接删除。

## ue-adapter：协议和 Unreal 渲染

```text
ue-adapter/
├── Adapters/bsk_render_adapter/           正式Python状态发送/采集协议包
├── Unreal/BskUnrealRenderer/
│   ├── BskUnrealRenderer.uproject        UE项目入口
│   ├── Config/                          引擎、场景、输入和资源配置
│   ├── Content/                         已导入UE资产
│   │   ├── BSK/                        机器人、任务盒、材质等资源
│   │   ├── Planets/                    行星资源
│   │   ├── SpaceFX/                    太空效果资源
│   │   └── _GENERATED/                 Git中保留占位；本机内容须按来源分类
│   ├── ContentSource/                   Foil002/PBR0110/SarmMLI等源材质
│   ├── Plugins/BskUnrealRuntime/
│   │   ├── BskUnrealRuntime.uplugin    插件描述
│   │   ├── Shaders/CameraEffect.usf    相机着色器
│   │   └── Source/
│   │       ├── BskUnrealRuntime/       场景、帧解析、网络、相机与渲染运行时
│   │       │   ├── Public/            公开C++头文件
│   │       │   └── Private/
│   │       │       ├── Scene/        场景生命周期、对象/资产与实际运行身份
│   │       │       ├── Capture/      权威采集、GPU读回、编码、网络及落盘
│   │       │       ├── Streaming/    Pixel Streaming 输出与UE版本兼容
│   │       │       ├── Lighting/     CelestialVault 与场景光照
│   │       │       ├── Tests/        UE自动化测试
│   │       │       └── *.cpp         帧解析、TCP、相机输入及模块初始化
│   │       └── BskCameraShaders/      相机着色器模块
│   ├── Source/                          UE主工程模块及Target构建定义
│   ├── python/bsk_unreal_adapter/       仍有外部示例/包调用的UE名称适配
│   ├── examples/                        示例场景；原生场景加载器仍被正式采集导入
│   ├── scripts/                         实际工程构建、资产导入、启动与测试
│   ├── tests/                           协议、帧、设备与渲染能力测试
│   └── docs/                            UE协议、材质、网格和视频说明
├── scripts/                              包级构建/启动包装与CI工具
├── test/model/                           适配器测试用模型，不是正式布局池
├── tools/solar_spectrum_to_rgb.py         太阳光谱转RGB工具
├── tools/data/solar/                     该工具的标准光谱/色度数据
├── docs/PYTHON_RUNTIME.md                Python环境说明
├── pyproject.toml                        包与依赖配置
└── VERSION                              适配器版本标记；不能替代Git/构建指纹
```

| 关键文件 | 职责 |
|---|---|
| `Adapters/bsk_render_adapter/bridge.py` | 公开Bridge和状态发布生命周期 |
| `scene_description.py`、`device_registration.py`、`command_dispatch.py` | MJScene静态描述、设备注册、命令分派；维持原Bridge API/协议 |
| `frames.py`、`descriptors.py`、`device_adapters.py` | 坐标/姿态转换、静态描述和设备可视化 |
| `protocol.py`、`capture.py`、`recording.py` | 网络协议、相机数据解码、渲染流记录/回放 |
| `mjcf_assets.py`、`stl_conversion.py`、`native_plugins.py`、`ue_support.py` | MJCF渲染元数据、STL转换、共享运行库插件和便捷API |
| `portable_assets.py` | 按包根解析 v2 catalog 和 UE 资源路径；拒绝绝对路径、目录越界与软链接逃逸 |
| `Adapters/bsk_render_adapter/build_identity.py` | UE 源码/着色器/引擎/二进制指纹；校验进程实际加载模块及 Game 单体程序 |
| `Private/Scene/BskSceneController.cpp` | Actor 生命周期、配置和权威帧应用，保留公开反射类与对象所有权 |
| `Private/Scene/BskSceneObjects.cpp`、`BskSceneUtilities.*` | 场景对象、资源清单、几何与材质辅助实现 |
| `Private/Scene/BskRuntimeIdentity.*` | 输出实际模块路径/PID/引擎版本；提供显式 Cook 资源加载验证入口 |
| `Private/Capture/BskCapturePipeline.*`、`BskCaptureProducts.h` | 权威采集请求、RHI/GPU 读回、帧顺序及采集产品；UObject 仍由 Actor 持有 |
| `Private/Capture/BskCaptureEncoding.*` | 图像编码与相机元数据 |
| `Private/Capture/BskCaptureNetworkSender.*` | 网络发送工作线程、连接/ACK/发送期限和取消 |
| `Private/Capture/BskCaptureDiskWriter.*` | 有界异步写盘、错误回传；产品与元数据提交后写入完成标记 |
| `Private/Streaming/BskPixelStreamingOutput.cpp`、`BskPixelStreamingCompatibility.*` | 相机流输出；集中隔离 UE 5.6 私有 API 依赖 |
| `Private/Lighting/BskCelestialLightingController.cpp` | 天体光照、背景与补光配置 |
| `BskFrameParser.cpp`、`BskTcpReceiver.cpp` | 帧消息解析、TCP接收 |
| `BskCoordinateConverter.cpp`、`BskRenderWorldSubsystem.cpp` | 坐标转换、渲染世界运行管理 |
| `BskCameraPawn.cpp`、`BskCameraInput.cpp` | 相机对象与输入 |
| `BskReplaySource.cpp` | 已记录状态流回放 |
| `BskRendererGameMode.cpp`、`BskRendererHUD.cpp` | 渲染场景模式和HUD |
| `BskCaptureProvider.h`、`BskLeRobotSampling.h`、`BskPreviewCadence.h` | 采集接口、数据采样和预览节拍 |
| `BskCameraEffects.cpp`、`CameraEffect.usf` | 相机效果及着色器 |
| `examples/scenario_spacecraft_arm_grasp_unreal.py` | 正式采集导入的 `load_native_grasp_module`；与 `server/model/.../scenario_sarm_grasp.py` 一并参与运行源码指纹 |

上述 `Private/` 路径相对 `Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/`。原来的单文件场景控制器已拆分到这些职责目录；TCP 断开时同时退休旧连接的部分包和排队命令。

UE 的 `Binaries/`、`Intermediate/`、`DerivedDataCache/`、`Saved/` 若存在均为本机产物。其中 `Binaries/<platform>/ue-build-manifest.json` 绑定 Editor 构建，`ue-build-<target>-<configuration>.json` 绑定 Game 目标；它们随产物生成，不代替源码编译。导入清单是否仍被布局引用必须实际检查，不能整目录盲删。

### UE 构建、打包和验证入口

| 路径 | 职责 |
|---|---|
| `ue-adapter/scripts/build_ue_project.py` | 校验指定 UE 版本后构建 Editor 或 Game，记录实际工具链并生成构建身份 |
| `ue-adapter/scripts/write_ue_build_manifest.py` | 成功构建后的清单生成及只读核验入口，不能手工重写清单冒充重建 |
| `ue-adapter/scripts/prepare_cook_staging.py` | 校验声明资产的哈希及 LFS 内容，将候选源码与明确资产复制到新的 staging |
| `ue-adapter/scripts/package_ue_project.py`、`package.ps1` | 独立 BuildCookRun、Game 构建身份、packaged 资源 LoadObject 验证和包指纹 |
| `ue-adapter/scripts/run_ue_automation.py`、`check_test_report.py` | 启动 Automation，拒绝旧报告、零用例、失败及未执行用例 |
| `ue-adapter/Unreal/BskUnrealRenderer/scripts/{build,runtime_build,start_renderer}.ps1` | Windows 工程构建与启动链路，按内容身份判断产物是否匹配 |
| `server/tests/test_capture_on_demand_renderer.py` | 显式启用的真实 GPU 验证：启停/重置、场景复用及四种相机模式 |

NullRHI Automation、packaged 资源加载、GPU 图像、五任务完整录制和跨平台发布分别形成证据；其中一种通过不代表其他验证已通过。

## 外部环境资源

`manifests/external-assets/incredible-earth.json` 固定外部地球包的版本、108 个文件大小与 SHA256；UE 项目 `Config/ExternalAssets` 保留同版本清单，供独立部署副本启动检查。`Content/IncredibleEarth` 还原在原资源路径但不进入 Git，`Content/BSK/Environment` 与 `BSK/Maps` 继续使用 LFS。

`tools/release/external_environment_assets.py` 负责校验、ZIP64 打包与还原，输出必须位于仓库外。`server/tools/deployment/sync_unreal_runtime.py` 按文件系统同步已恢复资源到部署 UE 副本。使用方式见[外部环境资产](guides/external-environment-assets.md)。

## 发布工具

| 路径 | 职责 |
|---|---|
| `tools/release/collector/build-collector.py` | 采集器交付包构建 |
| `tools/release/collector/app/collection_cli.py`、`space-sim-collect` | 包内CLI和启动器 |
| `tools/release/layout_preparer/build-release.py` | 布局准备器交付包构建 |
| `tools/release/layout_preparer/{collect-system-libs,copy-runtime}.py` | 收集系统库与复制运行依赖 |
| `tools/release/layout_preparer/app/{main,bootstrap,check}.py`、`space-sim-layouts` | 包内应用入口、环境启动和检查 |
| 各组件 `self-extractor.template` | 自解压包模板 |
| 各组件 `licenses/` | 第三方许可 |
| `tools/release/tests/` | 打包、启动、完整性和发布验证；产物仍写到仓库外 |
| `tools/release/runtime_metadata.py` | 读取实际 Python ABI/依赖、引擎/插件及 ELF 符号需求，生成运行环境来源 |
| `tools/release/resolve_environment.py` | 在全新输出目录解析哈希锁并可选安装隔离环境；不修改正式采集环境 |

## 保留批次与新版批次的数据目录

本机最近保留批次根：`/home/spacesim-zmh/data/collections/layout1000-20261031-0042/`，现已停止。下表展示共同的批次结构；标注“新版”的来源文件由当前候选版本创建的新批次生成，不能据此声称旧批次已通过新版门禁，也不为旧数据追写认证。
下列 `<任务>`、`<布局>`、`<attempt>`、`<episode>` 为程序生成标识，不是字面目录名。

```text
<batch>/
├── config.snapshot.json                    冻结参数与实际整数种子
├── plan.json                               固定布局顺序、任务和各条种子
├── progress.json                           当前进度、成功/跳过与审核状态
├── source-scene.json                        基准场景
├── pool.snapshot.json                      布局池清单快照
├── inputs.json                             冻结配置/场景/录制参数哈希
├── launch-provenance.json                  新版：冻结源码/native/环境/UE身份、输入与已解析种子
├── collector.lock                          单采集器租约，不能运行中删除
├── collector.log                           批次调度日志
├── renderer.log / renderer-process.log      渲染器日志
├── runtime-identity-<id>.json               新版：常驻渲染器的实际模块、PID与引擎版本
├── dynamics-worker-<id>/                    worker邮箱、身份、请求/结果与退休记录
├── layouts/pass-001/<布局>/                 每轮颜色、机位、模型等准备结果
├── attempts/<任务>/pass-001/<布局>/
│   ├── summary.json                        单任务尝试汇总
│   └── <attempt>/
│       ├── scene.json                      本条初态与任务输入
│       ├── recording.log                   录制启动/运行事件
│       ├── validation.json                 本条快速或完整审核结果
│       └── recording/
│           ├── summary.json                录制启动、耗时、倍率与输出索引
│           ├── runtime-provenance.json      新版：本次尝试的运行身份，与批次身份核对
│           ├── runtime-identity.json        新版：独立渲染器报告；复用时引用批次级报告
│           └── dynamics/summary.json       物理结果、复用信息、分段计时和源码指纹
└── <任务>/pass-001/<布局>/episodes/<episode>/
    ├── metadata.json                       episode身份、任务和平台元数据
    └── lerobot/
        ├── data/chunk-000/*.parquet         状态与动作主数据表
        ├── meta/                           数据集信息、统计、任务和episode索引
        ├── auxiliary/cameras/              相机配对及相关辅助记录
        ├── auxiliary/context/              任务上下文辅助记录
        ├── high_frequency/wrist_wrench/    独立高频腕力数据
        └── videos/
            ├── observation.images.teleop_camera_sarm_wrist_cam/    腕部视频
            └── observation.images.teleop_camera_spacecraft_overview/ 全局视频
```

视频下按 `chunk-000/file-000.mp4` 分块。状态表/腕力旁路采用 Parquet。停止时 `cancel.request` 控制整批取消；任务失败隔离输出可能出现 `rejected/`。这些是运行时生成内容，不提交源码仓库，不逐帧纳入本次源码文件索引。

## 文件格式与存放原则

| 格式 | 用途 | 典型归属 |
|---|---|---|
| `.py` / `.cpp` / `.h` | Python业务与C++原生/UE实现 | Git |
| `.js` / `.mjs` / `.html` / `.css` | 网页、信令与测试 | Git |
| `.json` / `.toml` / `.yaml` / `.yml` / `.ini` | 配置、协议、项目/构建定义；运行快照另存数据目录 | 视角色区分 |
| `.xml` / `.urdf` | MuJoCo与机器人描述 | 文本Git |
| `.stl` / `.obj` / `.stp` / `.SLDPRT` / `.SLDASM` | 网格和CAD源资产 | 按`.gitattributes`管理，通常LFS |
| `.uasset` / `.umap`、源纹理 | UE交付资产和纹理 | 按`.gitattributes`管理，通常LFS |
| `.sh` / `.ps1` / `.cmd` | Linux / PowerShell / Windows入口 | Git |
| `.md` | 当前说明或独立报告 | 按来源分别放docs或工作区reports |
| `.so` / `.dll` | 本机构建原生库 | `server/run/native_acceleration/`等生成目录 |
| `.mp4` / `.parquet` / `.jsonl` | 录制视频、状态动作、腕力和日志 | 批次或独立验证目录 |

命名与后续新增文件要求见 [命名规范](naming-conventions.md)。当前已有重复通用文件名、日期模型目录、大小写不一致和历史原型；按职责可区分，不等于已完成所有命名迁移。进一步移动或退役应在冻结当前采集依赖后单独实施。

### 独立操作任务

`server/simulation/scripted_tasks/drawer_tasks.py` 注册抽屉完整循环及拉起/放回；`threaded_tasks.py` 注册航空螺栓、锁杆完整循环及放倒/复位。
`server/simulation/model_preparation/drawer_task_initialization.py` 与 `threaded_task_initialization.py` 在录制前建立对应机构初态。
四项入口与默认启用规则见[单向操作采集](../server/docs/single-direction-collection.md)，所有十项的验收见[任务与数据验收](reference/task-and-data-acceptance.md)。

新增能力继续使用现有拆分结构：后端 `mission_api.py` / `vision_api.py` 只注册领域路由，`app.py` 保持装配；网页仍只有一个控制台、一个 API 客户端及既有会话模块。功能边界与验证口径见[仿真扩展](../server/docs/simulation-capabilities.md)。
