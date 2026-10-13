# SARM MJCF Platform Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans for inline execution, or superpowers:subagent-driven-development if the user selects delegation. Follow the tasks in dependency order.

**Goal:** 在现有网页平台启动、操控、重置和采集新版完整 SARM MJCF，替换临时静态预览。

**Architecture:** 以交付 MJCF 为只读源生成平台装配；以模型描述适配抓夹而不是伪造旧滑动关节。插件、控制、消息、UE 资产和场景生命周期共同绑定具体模型版本，旧模板继续兼容。

**Tech Stack:** Windows x64、Python 3.11+、平台 Basilisk/MuJoCo DLL、MSVC、UE 5.6、FastAPI、现有网页及 Pixel Streaming。

**Spec:** ../specs/2026-10-12-sarm-mjcf-platform-design.md（用户已于本轮确认）。

## Global Constraints

- 保留原始 2026-10-12 交付文件和哈希；模型来源为 SARM_convex.xml。
- 完整采用新机械臂、抓夹、任务盒、质量、惯量、碰撞与螺纹定义；平台附加结构必须单独记录。
- 保留 240 Hz 平台调度和 30 Hz 采集；新版本地积分子步不大于 1 ms。
- 物理插件注册到 Basilisk 已加载的同一个 MuJoCo DLL；不能引入另一个 Python MuJoCo DLL。
- 新模板使用指定姿态直接启动，禁用旧自动展开路径；旧模板行为不变。
- 原始参数修复须单独记录；QACC/NaN/时间回退不得被自动重置掩盖。
- 不提交本机配置、数据、凭据及编译产物。保存现有未提交文件和预览资产。
- 不自行填写 Git 作者身份。当前未配置作者身份；提交步骤在真实身份可用后执行，不阻塞本地验证。

## Review Focus

1. 缺失/错误版本插件：在启动新场景前明确失败，不删除 SDF 降级（任务 1）。
2. 新旧夹爪单位和向量长度不同：协议与数据集不能把弧度写成米（任务 3、4）。
3. 抽屉或自由件没有继承基座坐标/速度：世界装配与在轨共同运动检查（任务 2）。
4. 模型加载成功但时间发生自动回退：监测 QACC、状态有限性和时间单调（任务 3、6）。
5. 渲染仍连接静态预览/旧模型：模型哈希、实例、进程和受控关节变化联合验证（任务 5、6）。

## 执行前现场检查

- [ ] 读取根 AGENTS.md、CONTRIBUTING.md、两组件 README；检查实际分支、状态和已暂存文档。
- [ ] 使用 using-git-worktrees 技能确定隔离方式；导入资源当前是未跟踪文件，新的 worktree 不会自动带入。按 manifest 校验复制，保存已有用户改动，不 reset。
- [ ] 记录当前 Python/平台 MuJoCo 版本、编译器、插件源哈希、旧场景基线；确认没有 episode 采集中再停止预览。

## Task 1: 平台运行库的螺纹插件

**Files:** 新建 space_sim_server/simulation/mujoco_plugins.py、space_sim_server/tests/test_mujoco_plugins.py；修改 space_sim_server/tools/build_native_acceleration.py。源码引用 model/SARM_mjcf_20261012/thread_sdf.cpp。

**Interfaces:** `ensure_sarm_plugin(runtime_path: Path, plugin_path: Path, manifest_path: Path) -> dict`，返回源码哈希和已确认的运行库版本；失败抛出带路径与版本信息的异常。

- [ ] 写测试：缺失 DLL、构建清单哈希不符、版本不符时失败；重复注册同一 DLL 幂等；不导入 Python mujoco 包。
- [ ] 运行新测试确认失败，随后实现清单校验和对指定运行库的插件注册；将编译入口扩展为本机插件产物，保留旧组件构建选项。
- [ ] 单独子进程中加载 Basilisk，注册插件并构建原始完整模型；再创建两个模型副本，释放一个后验证另一个仍有效。
- [ ] `python -m pytest tests/test_mujoco_plugins.py -q` 全部通过；原生探针记录 DLL 路径、版本、模型 nq/nv 和完整异常；记录提交边界。

## Task 2: 可复现的平台装配与模型描述

**Files:** 新建 space_sim_server/tools/build_sarm_20261012.py、simulation/sarm_model_profile.py、tests/test_sarm_20261012_model.py；输出至 model/SARM/platform/sarm_20261012.xml 及同名 manifest.json。

**Interfaces:** `compose_platform_model(source: Path, platform_reference: Path, output: Path) -> dict`；`load_model_profile(model_path: Path) -> SarmModelProfile`，描述基座名、六轴关节、工具 site、抓夹类型/驱动/被动关节和自由件，不依赖数组固定索引。

- [ ] 写测试并确认失败：源文件哈希不变；源 equality/tendon/SDF 保留；所有 source mesh 引用存在；新关节质量惯量保留；平台附加件名单明确。
- [ ] 实现装配生成器：base_link 映射为平台基座，添加自由基座和独立反作用轮；避免旧基座惯量重复计入；抽屉转挂基座但保持零位变换；工具 site/相机显式映射。
- [ ] 写零位变换和质量总账测试，验证抽屉随基座平移/旋转，三个插头、螺栓和紧固件仍自由；原始几何规模只因明确平台附加项变化。
- [ ] `python -m pytest tests/test_sarm_20261012_model.py tests/test_selected_sarm_scene.py -q` 通过；两次生成结果哈希一致。

## Task 3: 真实四连杆控制、初始化与重置

**Files:** 新建 simulation/gripper_control.py、tests/test_linkage_gripper.py；修改 simulation/teleop_grasp_unreal.py、simulation/local_mujoco_stepper.py、native/local_mujoco_stepper.cpp、model/SARM/platform/scenarios/scenario_sarm_grasp.py、simulation/components/arm_reference.py。

**Interfaces:** `LinkageGripperCommand.advance(direction: float, dt_s: float, enabled: bool) -> float` 返回 tendon 驱动参考；关节/执行器通过 Task 2 模型描述寻址。原双滑动手指适配器继续存在。

- [ ] 测试源 ctrlrange 0..1 的上下限、方向、deadman/急停保持，不将两个被动关节当作独立位置控制；旧滑爪测试保持通过。
- [ ] 在新版模型加载前调用 Task 1；为 tendon 采用原始 position actuator 语义，保持源 gear/kp/kv/force range，避免套用旧 joint servo 的长度假设。
- [ ] 新模型选用每 240 Hz tick 至少 5 个子步（约 0.833 ms）；准确记录实际步长，旧模型沿用原配置。
- [ ] 初始化全部新自由体共同速度及闭环状态；重置重建完整状态，并检查 qpos/qvel 地址及发布/积分模型一致性。
- [ ] 原生测试执行 10 秒静置、至少 3 次低速抓夹开合、输入释放/失联、重置；检查所有状态有限、时间单调、无 QACC 和自动重置，记录闭环残差曲线。残差是否满足机构尺度由实测及模型约束参数判定，不用任意放宽阈值通过。
- [ ] `python -m pytest tests/test_linkage_gripper.py tests/test_teleop_target.py tests/test_local_mujoco_stepper.py tests/test_simulation_reset_native.py -q` 通过；原生测试必须实际执行，不能以 skip 当通过。

## Task 4: 新旧控制语义、观测和采集

**Files:** 修改 backend/space_arm_platform/models.py、safety.py、lerobot_capture.py、frontend/app.js；新建 tests/test_gripper_model_protocol.py，扩展 tests/test_protocol.py 和 frontend/tests 中对应状态显示测试。

**Interfaces:** 新观测含 `model_revision`、`gripper_type`、`gripper_drive_position_rad`、`gripper_drive_velocity_rad_s`、`target_gripper_drive_position_rad`；米制字段只对原滑爪使用。新动作采用显式角速度字段，后台依据当前活动实例解析，拒绝单位不一致的请求。

- [ ] 写兼容测试：旧 8 维消息仍可读，新模型不伪造两个米制手指值；错误模型/单位拒绝；缺失遥测显示未知而非零。
- [ ] 实现消息扩展与 safety 映射，新型夹爪沿用用户张开/闭合输入；初期显示驱动角，不声称归一化开度已标定。
- [ ] 采集 schema 按模型版本描述动作和观测维度，输出 joint names、units、模型哈希和运行库版本；不在单个 episode 中切换 schema。
- [ ] `python -m pytest tests/test_gripper_model_protocol.py tests/test_protocol.py -q` 及 `npm.cmd --prefix frontend test` 通过。

## Task 5: 受管场景与实时 UE 几何

**Files:** 修改 backend/space_arm_platform/scene_targets.py、scene_runtime.py、scripts/prepare_sarm_scene.ps1、scripts/start_scene_instance.ps1、scripts/run_simulation.ps1、simulation/components/render.py、tools/select_sarm_scene.py；适配器修改必要的 mjcf_assets.py 和独立外覆配置；扩展 tests/test_scene_runtime.py、test_local_simulation_launcher.py。

**Interfaces:** 模板 ID `sarm-task-box-20261012`，名称“新版 SARM 与完整任务盒（2026-10-12·实验）”；模型、插件清单和 catalog 均从模板解析。标记不支持旧自动展开 profile。

- [ ] 测试模板选择、缺失插件启动失败、直接操作姿态默认、非法自动展开拒绝、旧模板路径不变。
- [ ] 场景准备依次运行模型生成、插件检查、可见网格资产生成；保留透明碰撞几何于物理模型，但不导入为重复显示物体。
- [ ] 材质外覆通过新基座 asset 映射挂接，检查单位/镜像；UE 由真实状态帧更新每个活动 body，不使用 PreviewScene 的烘焙姿态。
- [ ] 由平台管理器负责启动/停止/重置并记录模型版本；退出临时静态 streamer，保留旧模板回归。
- [ ] `python -m pytest tests/test_scene_runtime.py tests/test_local_simulation_launcher.py tests/test_scene_reset.py -q` 通过，运行适配器受影响的资产/桥接测试。

## Task 6: 网页端到端验收与稳定性报告

**Files:** 新建 tools/validate_sarm_20261012_platform.py、docs/SARM_20261012_PLATFORM.md；报告摘要放 archive/2026_10_12_SARM_MJCF接入验收.md，原始日志/录制放本地忽略目录。

- [ ] 从平台新版模板启动真实仿真，确认实时观测模型哈希、时间增长、UE 状态帧及网页视频，保留截图和进程身份。
- [ ] 实测六轴小幅动作及抓夹开合；输入停止后检查实际速度；进行重置并逐项比对所有活动件初态。
- [ ] 分别复现三个插头低幅外力测试、抽屉/卡扣、螺栓和紧固件旋拧。任何参数修复先定位原因，记录原值/新值/影响；遇数值故障立即停止该项，不记录自动恢复为成功。
- [ ] 采集短 episode，实际解码视频并核对观测和动作单位、帧数、时间单调与模型版本。
- [ ] 停止测试实例，验证仅清理本实例进程和端口；按用户查看需求重新打开最后已验证可运行的新版场景。
- [ ] 汇总基础接入、各机构稳定性、实际抓取任务三类结果，未通过项明确保留；运行受影响测试和 git diff --check 后进行整体验证审查。

## Handoff

2026-10-12 执行记录：用户授权顺序执行，已完成插件、派生模型、tendon 控制、角度协议、采集元数据和受管场景接入；在已有 feature 分支原位执行，保留未跟踪导入源和已有 UE 资产，未另建 worktree 丢失这些输入。插件构建使用独立 tools/build_sarm_plugin.py，缓存及构建清单置于忽略目录。

已完成原生共同漂移 40 秒/三次全行程测试、实际网页开合和六方向末端输入、网页重置、两次原生全 qpos/qvel 重建比较、61 条状态与双路各 61 帧视频解码。源三种插头 free joint 世界阻尼 5→0 的必要在轨适配记录于派生 manifest 和验收文档，源文件不改。

任务 6 的三个插头外力插拔、抽屉卡扣、螺纹旋拧及实际抓取仍未作为通过项验收；不把空载运行代替任务级验证。原方案的细粒度 checkbox 保留供后续逐项扩展，当前交付范围和证据以 archive/2026_10_12_SARM_MJCF接入验收.md 为准。用户当前要求的动力学基础接入、运行、控制、重置、采集已经实际完成。

独立复审已完成：修复“仅检查有限值会让未接通 tendon 假通过”的问题，追加跟踪/行程/约束断言和负例；阻尼修正复审无阻塞项。当前未配置 Git 作者身份，且用户未要求本轮推送；没有提交或推送。
