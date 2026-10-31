# SpaceSim 项目结构地图

核查日期：2026-10-31。运行代码基线：`4cac86c651f6a19abbc4ae24b1a97f477c9738bd`（`optimized-collection/20261031`）。本说明区分实际源码、运行生成内容和工作区数据。

这份地图按职责维护；本机本次逐文件清单和完整文件树位于 `/home/spacesim-zmh/reports/project-structure-20261031/`。该盘点是时间点快照，后续新增文件以 Git 和实际磁盘为准。

后续目录调整、版本门禁、UE 验收与交付工作见[工程改进方案](ue-engineering-improvement-plan.md)。其中目标结构与拟新增文件属于待实施设计，不代表本地图中的现状已迁移。

## 工作区

```text
/home/spacesim-zmh/
├── WORKSPACE.md                    本机总入口
├── src/
│   ├── space-sim/                  唯一 Git 源码仓库
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
├── worktrees/                      临时 Git 工作树；核查时为空
├── migration/                      旧布局重编号计划的兼容链接
├── Library/Logs/                   UE 构建工具写入的用户级日志
└── UnrealEngine/UnrealTrace/        UE Trace 服务日志，不是第二份引擎
```

实际保留的正式批次：`layout1000-20261029`、`layout1000-20261030-1928`（停止），以及当前 `layout1000-20261031-0042`（按用户要求重新启动）。它们不是三份代码仓库。

当前布局池：`data/layout-pools/random150-20261028/`。当前恢复证据：`experiments/recovery-20261030-optimized-runtime/`。环境包含 `spacesim`、`spacesim-geometry`、`epic-browser`；目录名不足以证明可删除，需查调用依赖。

兼容关系：

```text
src/space_sim_server/space_sim_server     -> ../space-sim/server
src/space_sim_server/space_sim_UE_Adapter  -> ../space-sim/ue-adapter
```

它们只让旧批次元数据的绝对路径继续有效。新开发、启动和文档引用统一使用 `src/space-sim/`。

## Git 仓库顶层

| 目录/文件 | 职责 |
|---|---|
| `.github/workflows/ci.yml` | CI 构建、检查和测试任务 |
| `.github/CODEOWNERS`、`pull_request_template.md` | 维护归属和变更审查模板 |
| `.github/*settings*.json`、`branch-protection.json` | 仓库管理配置声明 |
| `.gitattributes` | 文本与 Git LFS 资产属性 |
| `.gitignore` | 排除运行目录、环境和生成产物 |
| `AGENTS.md` | 自动化协作的项目边界与命名规则入口 |
| `CONTRIBUTING.md` | 开发、集成、验证和数据管理约定 |
| `CHANGELOG.md` | 可审查的项目变更记录 |
| `README.md` | 对外项目说明和启动导航 |
| `docs/architecture/system.md` | 系统架构 |
| `docs/guides/` | 数据保留、本机运行与维护 |
| `docs/project-layout.md` | 路径与 Git/产物归属边界 |
| `docs/project-map.md` | 本结构地图 |
| `docs/naming-conventions.md` | 后续文件与目录命名规则 |
| `manifests/environments/` | 核查时 Python 环境依赖清单 |
| `manifests/assets/` | 资产索引规则；真实资产哈希以正式清单为准 |
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
│   └── scripted_tasks/            自动任务状态机、路径规划与图复用
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
| `collection/collect_scripted_tasks.py` | 单任务批采、共享 UE/动力学会话、录后快速检查与完整审核 |
| `collection/record_scripted_task.py` | 单条 UE 双相机 / LeRobot 录制、启停边界和最终封装 |
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
| `main.py`、`app.py`、`models.py` | CLI 启动、FastAPI 应用、API/仿真数据模型 |
| `auth.py`、`web_security.py`、`stream_access.py` | 用户会话、Web边界、视频访问令牌 |
| `collection_api.py`、`collection_jobs.py` | 网页采集 API 和后台采集任务管理 |
| `collection_inputs.py`、`collection_layouts.py`、`layout_trajectories.py` | 固定模型/布局池输入验证、轨迹验收与哈希绑定 |
| `collection_processes.py`、`collection_runtime.py`、`dynamics_worker.py` | 进程所有权、UE会话与动力学worker生命周期 |
| `collection_randomization.py`、`orbit_randomization.py`、`overview_camera.py` | 随机化配置、轨道相位和版本化相机采样 |
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
| `local_mujoco_stepper.py` | 本地坐标 MuJoCo 步进、状态读写与原生接口 |
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
| `control_client.py`、`teleop_grasp_unreal.py`、`gamepad_control.py` | 仿真通信、实时遥操作场景、手柄输入组合 |
| `policy_control.py`、`policy_task_success.py` | 策略命令执行和物理任务成功判据 |
| `observation_capture.py`、`wrench_sampling.py`、`wrench_sensors.py` | 同步观测、独立腕力采样和传感器适配 |
| `plug_retention.py` | 插头接合、引导力和轴向干摩擦 |
| `independent_task_box_layout.py`、`task_box_layout.py`、`collection_assets.py`、`socket_colors.py` | 独立布局、原布局原型、通用资产、每条颜色身份 |
| `camera_lighting.py`、`sarm_model_profile.py` | 相机补光、模型关节/执行器命名映射 |
| `motion_diagnostics.py`、`runtime_performance.py`、`runtime_progress.py` | 运动/性能/启动诊断，不改物理时钟 |
| `mjscene_threadpool.py`、`fastener_demo_schedule.py` | 显式启用的线程池实验、演示扭矩计划，不应误认当前默认采集路径 |

### scripted_tasks：自动任务与复用

| 文件 | 职责 |
|---|---|
| `drawer_cycle.py` | 抽屉打开、关闭、松开和完成确认 |
| `drawer_lift.py` | 抽屉基础任务实现及历史诊断入口 |
| `plug_insert.py`、`plug_profiles.py` | 航空/导向/圆柱插头共用控制器及不同部件参数 |
| `aviation_bolt.py` | 航空螺栓旋出和装配任务 |
| `locking_rod.py` | 锁杆松开、折叠、复位和锁紧任务 |
| `threaded_tasks.py`、`threaded_motion.py` | 螺纹任务公共组织与旋拧运动 |
| `physics_graph.py` | 本次优化的物理图兼容性判断、保留、重置和释放 |
| `drawer_initialization.py`、`gripper_preflight.py` | 初态、私有碰撞查询及夹爪几何预检；前者历史文件名不代表只供抽屉使用 |
| `planning_context.py`、`native_state.py` | 私有规划模型上下文、任务状态和接触读取 |
| `capture_transport.py` | 权威观测与渲染/录制传输连接 |
| `approach_path.py`、`carry_path.py`、`joint_path.py` | 接近、搬运、关节路径 |
| `collision_motion.py`、`environment_motion.py`、`obstacle_planner.py` | 接触约束、环境几何与避障规划 |
| `operation_clearance.py`、`plug_clearance.py` | 操作空间/插头通道净空检查 |
| `motion.py`、`motion_options.py`、`motion_timing.py`、`pose_tracking.py` | 运动公共逻辑、选项、时间规律和位姿跟踪 |
| `randomization.py`、`instructions.py`、`disturbances.py` | 初态/颜色等随机化、任务语言指令、可配置扰动 |

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
| `frontend/pages/datasets.*`、`dataset_filters.js`、`users.*` | 数据集浏览筛选与用户管理 |
| `frontend/arm_preparation.js`、`control_status.js`、`free_camera.js`、`gamepad_input.js` | 准备、控制状态、自由相机和手柄 |
| `frontend/initial_joint_angles.js`、`joint_angles.js`、`motion_diagnostics.js`、`wrist_wrench.js` | 初态/关节显示、运动诊断和腕力 |
| `frontend/*.css`、`vite.config.js`、`package*.json` | 样式、构建、依赖与锁定版本 |
| `signalling/server.mjs`、`origin.mjs`、`peer_departure.mjs` | 信令服务、来源验证、连接离开处理 |
| `config/recording.json`、`recording-veryfast.json` | 常规与显式veryfast流式录像设置 |
| `config/collection-base-assets.json` | 通用模型/UE资源的路径和SHA清单 |
| `config/plug_retention.json`、`task-instructions.json` | 保持力参数、任务指令模板 |
| `contracts/*.schema.json` | 状态/控制消息结构契约 |

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

本批实际物理输入来自仓库外布局池，并按任务生成变体，不能把 `model/README.md` 中某个历史默认场景当作所有采集任务的实际模型。`*_trial`、`*_validated2` 和预览目录目前仍存在，是需进一步分类的历史命名，不代表本次已全部规范化或可以直接删除。

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
│   │       │   └── Private/Tests/     实现目录内的UE自动化测试
│   │       └── BskCameraShaders/      相机着色器模块
│   ├── Source/                          UE主工程模块及Target构建定义
│   ├── python/bsk_unreal_adapter/       历史UE专属名称兼容包装
│   ├── examples/                        独立示例场景，非当前批采入口
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
| `Adapters/bsk_render_adapter/bridge.py` | 读取Basilisk对象，发布静态场景和当前状态 |
| `frames.py`、`descriptors.py`、`device_adapters.py` | 坐标/姿态转换、静态描述和设备可视化 |
| `protocol.py`、`capture.py`、`recording.py` | 网络协议、相机数据解码、渲染流记录/回放 |
| `mjcf_assets.py`、`stl_conversion.py`、`native_plugins.py`、`ue_support.py` | MJCF渲染元数据、STL转换、共享运行库插件和便捷API |
| `BskSceneController.cpp` | UE场景/资产/相机状态管理 |
| `BskFrameParser.cpp`、`BskTcpReceiver.cpp` | 帧消息解析、TCP接收 |
| `BskCoordinateConverter.cpp`、`BskRenderWorldSubsystem.cpp` | 坐标转换、渲染世界运行管理 |
| `BskCameraPawn.cpp`、`BskCameraInput.cpp` | 相机对象与输入 |
| `BskReplaySource.cpp` | 已记录状态流回放 |
| `BskRendererGameMode.cpp`、`BskRendererHUD.cpp` | 渲染场景模式和HUD |
| `BskCaptureProvider.h`、`BskLeRobotSampling.h`、`BskPreviewCadence.h` | 采集接口、数据采样和预览节拍 |
| `BskCameraEffects.cpp`、`CameraEffect.usf` | 相机效果及着色器 |

UE的 `Binaries/`、`Intermediate/`、`DerivedDataCache/`、`Saved/` 若存在均为本机产物；其中导入清单是否仍被布局引用必须实际检查，不能整目录盲删。

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

## 当前批次内部的数据目录

本机实际批次根：`/home/spacesim-zmh/data/collections/layout1000-20261031-0042/`。
下列 `<任务>`、`<布局>`、`<attempt>`、`<episode>` 为程序生成标识，不是字面目录名。

```text
layout1000-20261031-0042/
├── config.snapshot.json                    冻结参数与实际整数种子
├── plan.json                               固定布局顺序、任务和各条种子
├── progress.json                           当前进度、成功/跳过与审核状态
├── source-scene.json                        基准场景
├── pool.snapshot.json                      布局池清单快照
├── inputs.json                             冻结配置/场景/录制参数哈希
├── launch-provenance.json                  启动源码、原生库、环境开关与种子来源
├── collector.lock                          单采集器租约，不能运行中删除
├── collector.log                           批次调度日志
├── renderer.log / renderer-process.log      渲染器日志
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
