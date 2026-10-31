# 文件与目录命名规范

适用于新建和后续修改的文件。目录职责先看 [项目结构地图](project-map.md)，存储边界见 [目录规范](project-layout.md)。

## 基本规则

1. 名称表达业务对象和职责，单看路径能判断用途。优先 `collection_runtime.py`、`validate_capture_pipeline.py`，避免 `utils2.py`、`new.py`、`fix.py`、`test123.py`、`final_final.py`。
2. 正式源码用稳定名称；版本由 Git 提交、明确的协议/任务版本与构建 manifest 表达。不要为每次迭代复制 `*_new`、`*_old`、`*_backup`、`*_latest` 源文件。
3. 新增可导入 Python 目录和模块用英文小写 `snake_case`；普通文档与外部批次目录用英文小写 `kebab-case`。不混用多套含义相同的缩写。
4. 缩写只用于领域通用名：API、CLI、IK、MJCF、UE、RGB、SDF、RTF。首次在模块说明/README 中写明含义；时间、角度、距离字段标注单位。
5. 不为统一外观直接重命名运行中的入口、冻结批次路径、UE资产或原始CAD。迁移须修改引用和测试，保留数据来源映射；旧兼容链接不作为新开发入口。

## 按类型命名

| 类型 | 规则 | 示例 |
|---|---|---|
| Python 业务模块 | `<对象>_<职责>.py` | `physics_graph.py`、`capture_transport.py` |
| Python 包目录 | 小写 snake_case | `scripted_tasks/`、`model_preparation/` |
| 构建工具 | `build_<产物>.py` | `build_native_acceleration.py` |
| 输入准备工具 | `prepare_<输入>.py` | `prepare_collection_layout_pool.py` |
| 格式转换 | `convert_<源>_to_<目标>.py` | `convert_step_to_mjcf.py` |
| 前置检查 | `check_<对象>.py` | `check_runtime_backend.py` |
| 规则/数据验收 | `validate_<对象>.py` | `validate_capture_pipeline.py` |
| 环境/集成核验 | `verify_<能力>.py` | `verify_orbital_physics.py` |
| 性能测量 | `benchmark_<对象>.py` | `benchmark_scripted_capture.py` |
| 性能剖析 | `profile_<对象>.py` | `profile_simulation_runtime.py` |
| Python 测试 | `test_<被测能力>.py` | `test_collection_physics_graph.py` |
| 现有网页 JS / Node | 继续采用所在目录的小写 snake_case；模块扩展名依现有构建方式 | `collection_randomization.js`、`peer_departure.mjs` |
| JS 测试 | `<被测能力>.test.mjs` | `collection_randomization.test.mjs` |
| 独立 C++ | 小写 snake_case | `local_mujoco_stepper.cpp` |
| UE C++ | 保持 UE 模块/类型的 PascalCase | `BskSceneController.cpp`、`BskCameraPawn.h` |
| UE 构建描述 | 遵守引擎规定 | `BskUnrealRuntime.Build.cs`、`BskUnrealRenderer.Target.cs` |
| 新文档 | 描述主题的小写 kebab-case | `project-map.md`、`naming-conventions.md` |
| 配置变体 | `<用途>-<模式>.json`；沿用目录既有规范 | `recording-veryfast.json`、`collection-parallel.json` |
| JSON Schema | `<领域>-<消息>-v<版本>.schema.json` | `simulation-state-v1.schema.json` |

已有 `README.md`、`AGENTS.md`、`CONTRIBUTING.md`、`CHANGELOG.md` 是约定入口，保留大写。已有专题大写文档可以继续引用；以后新增采用上表规范，链接完整迁移前不批量改名。

## 批次、实验、报告和发布

新建工作区产物推荐格式：`<用途>-<范围>-YYYYMMDD-HHMMSS`。时间使用北京时间，并在 manifest 中记录带时区的 ISO 时间；同秒冲突追加短ID，拒绝覆盖。

| 存储位置 | 新名称示例 | 必须记录的含义 |
|---|---|---|
| `data/collections/` | `collection-layout1000-20261031-090000/` | 实际随机种子、冻结计划、配置、源码/原生库指纹 |
| `data/layout-pools/` | `layout-pool-random150-20261031-090000/` | 数量、模型/catalog哈希、生成与验收范围 |
| `experiments/` | `physics-graph-reuse-20261031-090000/` | 问题、实验入口、基线与候选、有效结果、依赖 |
| `artifacts/verification/` | `verify-five-task-capture-20261031-090000/` | 五任务、完整/短测、是否渲染、是否全解码 |
| `reports/` | `project-structure-20261031/` | 此次整理或分析的时间、源码提交、覆盖范围 |
| `releases/<component>/` | 保留发布系统的 `<release-id>` | 平台、源码、构建来源、SHA、验收，不覆盖旧包 |
| `archive/workspace-snapshots/` | `workspace-before-layout-change-20261031-090000/` | 快照范围与恢复方法 |

本机已有 `layout1000-20261031-0042` 等名称不改写。配置字段、冻结 `summary.json` / `plan.json` / `progress.json` 以及采集器内部路径属于程序契约，也不为了外观改名。

## 模型与资产

- 新资产按语义稳定ID命名，版本/尺寸/来源写入 manifest；资产名称与目录只承载必要区分信息。
- UE 新资产遵循引擎常用前缀：`SM_` 静态网格、`M_` 材质、`MI_` 材质实例、`T_` 纹理、`BP_` 蓝图。现有引用未迁移前不改名。
- 原始 CAD 的供应方名称和中文名称保留；导出名称可规范化，但必须记录原名与导出文件映射。
- `Generated`、`validated`、日期后缀或 `trial` 不能单独作为删除依据；先查当前输入、历史复现和发布依赖。
- 不再以 `validated2` / `validated3` 表达新运行版本。实验变体写入独立实验目录，通过 manifest 说明区别。

## 职责重叠时如何命名

- `server/tools/collect.py` 是稳定日常入口，不再增加同义的 `run_new_collection.py`。
- `server/collection/dynamics_worker.py` 是采集 worker 的可执行装配入口；`server/backend/space_arm_platform/dynamics_worker.py` 是进程/请求管理实现，当前不是两套动力学。
- `server/simulation/local_mujoco_stepper.py` 是 Python 封装；`server/native/local_mujoco_stepper.cpp` 是原生实现。同名表示对应关系。
- 新增第二个 `main.py`、`bridge.py` 或 `protocol.py` 时，要求包路径足以明确边界，并在模块首行说明职责；跨层维护时引用完整路径。

## 提交要求

新增模块有简短职责说明；新增入口写明调用方式、输入输出和是否属于生产路径。目录角色或入口变化时同步维护项目结构地图；实验性能报告必须明确任务、布局、录制/无渲染、短测/完整任务、是否包含初始化，避免把不同口径统称为“最新速度”。
