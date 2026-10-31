# SpaceSim 工程整理与 UE 交付改进方案

日期：2026-10-31。状态：**待实施方案**。本次仅新增方案及文档导航；下列源码修改、目录迁移、删除、构建、验收和采集切换均未执行。

审查基线为 `769806c261a3bf50005be840251b9299e8c23c7d`，其中采集优化实现为 `4cac86c651f6a19abbc4ae24b1a97f477c9738bd`。本机审查证据见 [UE 工程审查报告](/home/spacesim-zmh/reports/ue-engineering-audit-20261031/REPORT.md)。下文 F01–F15 与该报告逐项对应；报告链接是本机证据位置，不是跨机安装依赖。

## 1. 目标与实施边界

目标是让 SpaceSim 能在干净环境重建、明确识别实际运行版本、稳定录制、按声明资产打包，并能由不同维护者安全修改。现有外部权威动力学、UE 渲染与采集、Runtime/Shader 模块以及 `server/ + ue-adapter/` 单仓库结构继续作为基础。

本轮实施应遵守以下边界：

1. 当前采集批次 `data/collections/layout1000-20261031-0042/` 使用的代码、环境、原生库、UE 产物和布局保持冻结。方案编写不触发停止或重启。
2. 开发放在独立 Git worktree；使用独立环境、构建目录和验证输出。不能在共享环境执行 editable 重装，让正在运行的进程导入另一份源码。验收记录实际模块 `__file__`、解释器与动态库路径。
3. 不在正式采集 GPU 上启动第二套 UE/GPU 验收。资源密集的 native、构建和磁盘测试同样需要独立资源或安排窗口；能写代码不等于能在生产主机同时压测。
4. 保留物理 400 Hz、控制/腕力 200 Hz、双 RGB 1024×1024/50 Hz，以及当前碰撞、完成判据、布局/颜色/机位与随机种子语义。改变这些内容属于另一项功能或性能变更。
5. 机械拆分与行为修复分开提交；协议、采集端与 UE 端需要配套时放在同一分支/PR。每个交付单元可独立验证与回退。
6. 初始化提速和稳态录制倍率分别测量。历史约 0.822 倍的固定布局无渲染实验不作为本方案双相机采集的性能目标或结果。

现有目录及每个入口的完整说明继续以 [项目结构地图](project-map.md) 为准。本方案列出目标职责和具体变更清单，不另造第二份现状清单。

## 2. 目标目录：保留主体，补齐交付能力

### 2.1 整个工作区

以下为目标职责。带 `<…>` 的名称是占位符，尚未创建；源码目录之外的内容不纳入 Git。

```text
/home/spacesim-zmh/
├── WORKSPACE.md                         本机入口、环境与当前任务位置
├── src/
│   ├── space-sim/                       主源码仓库
│   └── space_sim_server/                旧批次路径兼容入口，暂时保留
├── worktrees/
│   └── ue-engineering-<change-id>/       每个实施分支的独立工作树
├── data/
│   ├── collections/<batch-id>/          原始记录、冻结计划、来源与审核状态
│   ├── layout-pools/<pool-id>/          可复现布局包、模型、catalog、哈希
│   └── models/                          外部模型与权重
├── experiments/<experiment-id>/         实验源码、输入、结论及恢复依据
├── artifacts/
│   ├── build/<build-id>/                独立 UE/native 构建与 Cook staging
│   └── verification/<verification-id>/  故障注入、双相机和独立包验收输出
├── reports/<report-id>/                 审查、性能对照及验收索引
├── releases/<component>/<release-id>/   不可覆盖的交付包及来源清单
├── archive/workspace-snapshots/         源码、未提交内容与资产依赖恢复快照
├── envs/<environment-id>/               已安装环境；候选与正式环境分开
└── engines/<engine-id>/                 UE 安装；按实际 Build ID 验证
```

`src/space_sim_server/space_sim_server` 与 `src/space_sim_server/space_sim_UE_Adapter` 是兼容软链接，不是第二套源码。当前保留批次仍可能引用它们；删除前必须证明全部依赖已退役。新入口只使用 `src/space-sim/`。

UE 的 `Binaries/Intermediate/Saved/DerivedDataCache` 仍可位于候选 UE 工程内，遵循 UE 工具约定并被 Git 忽略；独立工作树已提供隔离，不强制把每个引擎目录外移。归档后的构建与验收产物进入 `artifacts/` 或 `releases/`。

### 2.2 源码仓库

`[新增]` 表示拟新增，其他目录已有；树中省略未涉及的文件，完整现状见项目结构地图。

```text
space-sim/
├── .github/workflows/
│   ├── ci.yml                          保留快速 Python/网页/信令检查
│   ├── native-validation.yml           [新增] 真实物理及原生库验收
│   ├── ue-validation.yml               [新增] UE 编译、Automation、GPU 分层
│   └── release-validation.yml          [新增] Cook 与独立发布包验收
├── docs/
│   ├── project-map.md                  已有结构与职责总表
│   ├── naming-conventions.md           已有命名规则
│   ├── ue-engineering-improvement-plan.md  本方案
│   ├── architecture/runtime-provenance.md [新增] 构建、启动、批次身份约定
│   └── guides/
│       ├── ue-validation.md            [新增] 分层验收与证据要求
│       └── release-reproduction.md     [新增] 干净重建、Cook、运行包验收
├── manifests/
│   ├── schemas/                        [新增] 版本化来源与布局契约
│   ├── environments/                   快照 + 经重装验证的各平台锁与 ABI 清单
│   └── assets/                         基础 Cook 输入及布局资产声明规范
├── server/
│   ├── backend/space_arm_platform/     API、部署、worker 管理与布局解析
│   ├── collection/                     采集编排、来源冻结与续采核验
│   ├── simulation/scripted_tasks/      图复用及独立任务状态；保留现有边界
│   ├── native/                         原生源与既有编译指纹机制
│   ├── tools/
│   │   ├── assets/                     构建、资产及布局迁移工具
│   │   ├── deployment/                 启动前环境/版本诊断入口
│   │   ├── benchmarks/                 明确测量口径的性能工具
│   │   └── diagnostics/                离线诊断和校验
│   ├── tests/                          保留当前目录，以 profile 区分运行依赖
│   ├── scripts/                        CI profile、启动与部署装配
│   └── config/                         默认配置；机器路径不写入通用配置
├── ue-adapter/
│   ├── Adapters/bsk_render_adapter/    渲染协议；新增轻量构建身份校验模块
│   ├── scripts/                        跨平台构建身份工具及平台包装入口
│   └── Unreal/BskUnrealRenderer/
│       ├── Source/                     项目 Target 与项目模块
│       ├── Config/                     显式 Cook/渲染配置
│       ├── Content/                    交付 UE 资产；二进制使用 Git LFS
│       ├── ContentSource/              资产源文件及生成依据
│       ├── Plugins/BskUnrealRuntime/   Runtime + 相机 Shader 模块
│       └── scripts/                    UE 编译、Automation 与打包入口
└── tools/release/
    ├── collector/                      采集器交付包构建
    ├── layout_preparer/                布局准备器交付包构建
    └── tests/                          包内容、来源和环境识别验收
```

不新增 `core2/`、`runtime_new/` 或“最终版本”副本。构建 ID、Git 提交和 manifest 承担版本管理。已有 `server/run/native_acceleration/` 是加载契约，先保留；后续如外移，必须一起修改构建、加载、部署和验证路径。

### 2.3 Runtime 模块的最终职责拆分

以下均在现有模块内部。公开反射头文件和类名先保持稳定；仅将内部实现逐块迁移。

```text
Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/
├── BskUnrealRuntime.Build.cs
├── Public/                             现有公开接口、UClass 名称与反射契约
└── Private/
    ├── Scene/
    │   └── BskSceneController.cpp      场景生命周期、子系统装配与调用顺序
    ├── Capture/
    │   ├── BskCapturePipeline.h/.cpp   权威帧、GPU 回读、编码及完成顺序
    │   ├── BskCaptureNetworkSender.h/.cpp  发送队列、连接及可取消等待
    │   └── BskCaptureDiskWriter.h/.cpp  落盘、完成回执及失败上报
    ├── Streaming/
    │   ├── BskPixelStreamingOutput.h/.cpp  预览输出与生命周期
    │   └── BskPixelStreamingCompatibility.h/.cpp  引擎内部 API 兼容边界
    ├── Lighting/
    │   └── BskCelestialLightingController.h/.cpp  天体/环境光照
    ├── BskTcpReceiver.cpp              当前接收与会话处理，先保留路径
    └── Tests/                          现有及新增 UE Automation 测试
```

新类优先使用明确所有权的普通 C++ 类型；有 UObject 生命周期需要时才引入 UObject。UObject 操作留在 Game Thread，后台线程仅持有允许跨线程的数据。保留采集在呈现插值之前发生、有界队列、背压、帧序号与终态屏障。

## 3. 实施阶段与依赖

| 阶段 | 对应审查项 | 交付结果 | 前置条件 |
|---|---|---|---|
| S0：冻结与隔离 | 全部 | 来源快照、独立工作树/环境、保留清单 | 开始任何实施前 |
| S1：入口与基础检查 | F01、F02、F15 | 首次部署路径正确；基础 CI 与 native 测试分层；文档能力准确 | S0 |
| S2：版本身份与可重建环境 | F03、F06、F12、F14 | 构建→启动→批次→交付使用统一身份 | S1；各平台环境可用 |
| S3：UE 与物理自动验收 | F04 | 编译、NullRHI、GPU、发布分层门禁 | 脚本可与 S2 并行；门禁接入依赖其身份校验 |
| S4：运行可靠性与缓存收益 | F07、F08、F09、F11 | 重连无残包；失败可见；停止有界；健康缓存保留 | S2、S3 中对应 UE/native 验收可用 |
| S5：资产与布局交付 | F05、F13 | 隔离 Cook；资产范围确定；布局可跨目录搬迁 | S2；Cook 和迁移各自验证，可与 S4 并行 |
| S6：职责拆分与发布验证 | F10及各项回归 | 模块内部职责清楚；候选版本完成完整验收 | 可靠性测试先落地；候选环境与资产清单稳定 |

F11 单独实施，不等待 F10 大类拆分；F05 先采用隔离 staging 和资产清单，不等待 F13 格式迁移。编写 CI workflow 并不等于已经具备合格 runner，实际运行证据需另行交付。

### S0：实施前冻结

- 保存 Git commit、工作区差异、未跟踪源码/测试、LFS 状态、原生库和 UE 二进制指纹。运行基线同时覆盖源码和实际产物，不能只记录 HEAD。
- 快照放入 `archive/workspace-snapshots/workspace-before-ue-engineering-<timestamp>/`；记录源位置、SHA-256、恢复方法和必要外部资产。对大数据使用只读清单与受保护原件，不无意义复制整个采集集。
- 建立 `worktrees/ue-engineering-<change-id>/`；候选 UE 本地生成资产从声明输入重建或复制到候选区域，禁止候选导入工具写回正式工程。
- 正式批次、旧保留批次、恢复实验、当前布局池、环境、原生库和兼容链接进入保留清单。未经依赖判定，不做全局 `clean` 或按日期批量删除。

完成标准：能够指出任一运行组件的来源和恢复位置；候选 Python、原生库、UE 与输出路径均不落到正式运行位置。

### S1：先修复已确认的小范围问题

| 修改位置（仓库相对路径） | 具体工作 | 验收 |
|---|---|---|
| `server/backend/space_arm_platform/linux_deployment.py` | 调用 `tools/assets/build_native_acceleration.py` 和 `tools/assets/build_sarm_plugin.py` | `server/tests/test_linux_deployment.py` 检查真实命令目标和参数；独立环境首次 prepare 完成原生构建并进入 UE 准备 |
| `server/simulation/scripted_tasks/physics_graph.py`、`server/tests/test_collection_physics_graph.py` | 通过惰性依赖或明确注入隔离图缓存规则，纯测试不导入 Basilisk | 缺 Basilisk 时缓存规则测试真实执行并通过 |
| `server/scripts/ci-exclusions.json`、`server/scripts/ci_pytest.py`、`server/tests/test_native_episode_reset.py` | 原生测试归属明确 native profile，并由实际 native job 执行；保留意外 skip 失败 | 基础 CI 无意外 skip；native 缺依赖时该层失败或未执行，不能冒充通过 |
| `.github/workflows/ci.yml` | 检查部署路径过滤器，覆盖迁移后的入口与共享依赖 | 相关文件改动会触发所需检查 |
| `ue-adapter/README.md`、`ue-adapter/Unreal/BskUnrealRenderer/README.md` | 修正异步 GPU 回读能力描述，增加实现/默认/平台/证据/限制表 | 明确权威采集、预览和回退路径；不使用旧测速代替当前性能 |

此阶段不做模型重生成或大规模测试目录迁移。部署路径修复、测试隔离、文档纠正分别提交。

### S2：用统一来源贯通构建、启动、续采与交付

拟新增下列稳定名称；先定契约，再接入口，避免各工具独立计算不同版本身份。

| 拟新增文件 | 职责 |
|---|---|
| `manifests/schemas/component-identity-v1.schema.json` | 组件来源、输入指纹、构建环境与产物哈希的公共结构 |
| `manifests/schemas/ue-build-manifest-v1.schema.json` | UE 构建结果契约，引用公共组件身份 |
| `manifests/schemas/collection-provenance-v1.schema.json` | 批次启动/续采来源契约，引用组件身份 |
| `ue-adapter/Adapters/bsk_render_adapter/build_identity.py` | 轻量 UE 身份生成/核验逻辑，不引入 UE 或 Basilisk 运行依赖 |
| `ue-adapter/scripts/write_ue_build_manifest.py` | 编译成功后生成 `ue-build-manifest.json` 的 CLI |
| `server/collection/runtime_provenance.py` | 汇总采集源码、环境、native、UE、资产及配置，核验续采 |
| `server/tools/deployment/check_runtime_provenance.py` | 可读诊断：指出哪个输入/库不一致及如何重建 |
| `manifests/environments/linux-simulation.lock`、`windows-simulation.lock` | 分平台、完整解析且经过干净安装验证的 Python 依赖锁 |
| `manifests/environments/linux-runtime.json`、`windows-runtime.json` | Python ABI、Basilisk/native、UE Build ID、插件、工具链与验证证据 |
| `docs/architecture/runtime-provenance.md` | 哈希范围、兼容规则、失败处理与旧批次兼容 |

锁文件格式在实施时选择一种工具支持的可安装格式，并记录安装器版本和命令；不能把 `pip freeze` 快照改后缀就称为可重建锁。分别记录私有/本地 wheel 或源码构建依赖的内容哈希、取得方式和许可约束。某平台未重建验收时明确标为未认证。

修改入口：

- `ue-adapter/Unreal/BskUnrealRenderer/scripts/build.ps1`、`runtime_build.ps1` 及 Linux 构建路径：编译成功后生成统一清单；从固定输入读取实际引擎、工具链、目标和模块身份，替换单纯 mtime 判断。
- `server/backend/space_arm_platform/linux_deployment.py`：检查构建身份，不再仅凭 `.so` 存在视为可用。
- `server/collection/record_scripted_task.py`、`sequence.py`：启动前核验，启动后确认实际加载身份，并自动冻结来源。已有原生源指纹检查继续使用，由统一清单引用。
- UE 运行端与 Python 适配器：为启动握手提供构建/会话身份或等效的可核验运行报告；跨端变更一并提交。新增协议能力显式协商，不能把旧客户端当作已完成新核验。
- `tools/release/layout_preparer/copy-runtime.py` 和两类发布入口：从实际传入 Python、Engine `Build.version`、ELF/平台依赖读取版本，不再写固定字符串。Linux 最低 glibc 要求从交付 ELF 的符号依赖判断，构建机版本另记。

各清单的最小内容：

| 清单 | 必须记录的字段类别 |
|---|---|
| UE 构建 | schema_version、build_id、源码/Shader/Build.cs/插件描述指纹、Engine 版本/Changelist/Build ID、平台/架构/Target/配置/工具链、模块及二进制哈希、构建时间 |
| 环境 | 解释器版本/ABI、锁文件 SHA、Basilisk 来源/能力、native 组件身份、系统 ABI、引擎/插件身份；GPU 验收另记录驱动 |
| 批次来源 | Git commit、dirty 状态及运行源码指纹、入口/参数、任务版本、实际整数 seed 及来源、冻结计划/配置/布局/模型/catalog SHA、环境/native/UE 身份、启动后实际加载结果 |
| 发布 | release_id、各组件身份、资产清单 SHA、文件 SHA、支持环境、验收状态及证据索引 |

来源文件随产物存放：`ue-build-manifest.json` 随构建；新批次的 `launch-provenance.json` 自动生成；发布包带组件与资产清单。Git 中保存 schema、工具和已验收环境声明，运行生成的 manifest 不逐次提交。

身份计算细节：

1. Git commit 记录背景，运行兼容性使用显式运行文件集合的内容指纹。集合覆盖采集装配、任务、适配器、原生源码、UE 源/Shader、相关配置；运行范围内未跟踪的源码也必须纳入或拒绝启动。
2. 普通 README/报告修改不导致物理运行身份变化。声明范围与依赖检查负责防止遗漏，不采用“只哈希五个核心文件”。
3. 文件路径统一为包内相对路径，按规范排序并使用 SHA-256；时间和本机根路径不进入内容身份。引擎/环境全量扫描在构建认证时进行，批次启动核验对应身份及关键产物，不每条任务重扫整个 UE 安装。
4. 启动前比对磁盘文件还不足以证明实际加载版本。启动后需验证模块/构建身份，并确保采集期间来源不可变；后续每条记录引用该不可变会话身份。
5. 新机制下续采运行身份不匹配时拒绝续写，默认建立新批次。本期不引入同一批次混合多个代码版本的复杂机制。
6. 旧冻结批次不回写新清单。保留读取和离线审核能力；无完整来源的历史续采不能被自动标成新机制认证通过。

验收必须覆盖：修改 C++ 或 Shader 而不重建会拒绝；模块/引擎/原生库不匹配会拒绝；完整重建后通过；仅文档变化不误拒绝；运行源码变化不混入同一新制批次；发布输入换版本能正确识别或按策略拒绝。对应新增测试建议为 `test_runtime_provenance.py`、`test_build_identity.py` 和发布环境识别测试，分别放在所属组件现有测试目录。

### S3：自动验收分层

| 层 | 调度与运行位置 | 内容 | 能证明的范围 |
|---|---|---|---|
| 快速 PR | 现有普通 runner | Python 纯单元、协议、网页/信令、部署命令与清单校验 | 不需要真实 UE/native 的逻辑 |
| Native | 受控仿真环境；相关变更与定期集成 | Basilisk/native 能力、reset、缓存失效、数值与状态等价 | 原生物理链路；不证明图像正确 |
| UE 编译/Automation | 安装获授权 UE 与工具链的受控 runner；相关变更 | Editor/Game 目标编译、NullRHI 生命周期/协议/重连测试 | UE 可构建及非 GPU 逻辑 |
| GPU 采集 | 独立 GPU runner 或已安排的验收窗口 | 双相机录制、帧序、颜色/机位、布局切换、终态边界与图像解码 | 实际渲染与录制；不等于全批数据合格 |
| 发布 | 发布候选、独立 staging | Cook、包内动态资产加载、独立环境启动、完整任务与来源核验 | 可交付候选版本 |

新增 workflow 调用已有入口及共享身份校验器；需要 Linux UE 入口时，在 `ue-adapter/scripts/` 新增 `build_ue_project.py`、`run_ue_automation.py`，封装引擎命令并明确平台限制，避免另写一套版本规则。Windows 沿用并接通已有 PowerShell 入口。

编译/Automation 对 UE 源、Build.cs、Shader、协议以及构建脚本的修改触发；GPU 对采集、Shader、场景/资产及相关协议变更触发。公共契约变更需同时验证 server 和 adapter。路径过滤器应覆盖上述依赖，不能只看 `.cpp`。

为必需的 UE 检查配置稳定状态名及分支保护；缺 runner、未执行或超时必须可见，不能因 job 被跳过而让破坏 UE 编译的变更获得完整绿灯。自托管 runner 只运行经过信任边界确认的代码，不在正式采集环境直接执行外部 PR。

交付 `docs/guides/ue-validation.md`，记录各层启动命令、环境、输入、输出和已通过范围。增加 workflow 文件不代表已经启用远端 runner 或改好了仓库分支保护；这些外部设置必须按实际完成情况登记。

### S4：分别修复运行可靠性与缓存复用

首先在现有文件位置修复行为，保护用例落地后才进入 S6 拆分。

| 问题与修改位置 | 具体改法 | 最低验收 |
|---|---|---|
| F07：`BskTcpReceiver.cpp` | 活动发送包、offset 和共享队列统一归属于 connection/session；所有断线和新连接路径重置 | 注入部分发送后 FIN、解析失败、连接异常；新连接首包不能收到旧尾部 |
| F09：`BskSceneController.cpp` 中网络发送线程 | socket 从创建起非阻塞；connect、发送、ACK 等待均有 deadline 与可取消轮询 | 不可达、半开、等待 ACK 时均可停止；建议目标为发出停止后线程 2 秒内结束，实施时记录平台计时证据 |
| F08：同文件中的磁盘 writer | 检查建目录/文本/图像写入结果；独立完成/错误回执；全部预期写入成功后才发布完成标记 | 无权限、容量不足和部分文件写失败均产生可见错误；失败记录不标成完成；正常目录成功 |
| F11：`server/collection/run_scripted_task.py`、`server/backend/space_arm_platform/dynamics_worker.py`、`server/simulation/scripted_tasks/physics_graph.py` | 初态业务拒绝显式返回；证明 live graph 未被修改且清理完整时保留健康 worker/cache；未知或状态异常仍退役 | 拒绝→成功与新 worker 的数值、时间戳、初态、相机起止一致；真异常仍触发新 worker |

UE 磁盘 writer 修复针对 `BskCaptureDir` 直接落盘路径，不能据此断言当前网络录制已经丢数据。上述网络问题也是已确认代码风险，尚无当前批次故障的现场证明。

新增 UE 测试放入当前 `Private/Tests/`：`BskConnectionLifecycleTests.cpp`、`BskCaptureDiskWriterTests.cpp`。状态/故障测试使用受控 socket 与临时输出，不向正式磁盘写满。GPU 集成验证另外执行。

F11 建议新增 `server/tests/test_worker_rejection_reuse.py`，覆盖业务拒绝、清理不完整、真实 native 异常、输入不兼容重建。至少验证连续成功→拒绝→成功，以及布局 A→B→A。拒绝条件不放宽，正常失败仍按当前规则跳过，不通过增加重试伪造成功率。

性能对照使用同一冻结布局、任务和实际 seed；分别记录 cold build、兼容 reset、拒绝恢复、录制、封装与审核。只有证明拒绝后的下一条减少了重复初始化，才能声明此项有收益；不能推导稳态物理倍率同幅提升。

### S5：资产 Cook 与布局包可搬迁

F05 的第一步采用独立 staging 和显式清单即可，不立即引入 DLC 或复杂 Asset Manager 系统。

| 文件或目录 | 具体工作 |
|---|---|
| `manifests/assets/ue-cook-base-assets.json`（拟新增） | 基础 maps、网格、材质、相机/光照及动态加载资源清单；依赖闭包和来源 SHA |
| `ue-adapter/Unreal/BskUnrealRenderer/Config/DefaultGame.ini` | 将整个 `/Game/BSK` 的笼统 Cook 规则替换为经验证的明确范围，检查 SpaceFX 字符串加载资源 |
| `ue-adapter/Unreal/BskUnrealRenderer/scripts/package.ps1` | 从声明输入创建隔离 staging 后 Cook；本机临时布局不隐式进入包 |
| `ue-adapter/scripts/prepare_cook_staging.py`（拟新增） | 解析基础清单及选定布局清单、校验哈希/LFS 完整性、生成 staging 与 cooked 资产索引 |
| `server/backend/space_arm_platform/collection_layouts.py` | 支持新布局 schema 的相对路径+显式根+哈希；旧格式读取保留 |
| `server/tools/assets/migrate_layout_pool.py`（拟新增） | 只向新目录输出迁移池、catalog 和映射报告；保留旧输入 |
| `manifests/schemas/collection-layout-pool-v2.schema.json`（拟新增） | 包内相对路径、模型/catalog/资产哈希、生成器版本与来源；实施前核对既有 schema 命名并采用下一未占用版本 |
| `server/tests/test_collection_layouts.py`、`test_prepare_collection_layout_pool.py` | 覆盖旧格式、新格式、跨根目录、路径越界/软链接逃逸、缺资源和哈希错误 |

新布局 pool 和 catalog 必须一起验证：只将 `pool.json` 改成相对路径，但 catalog 内仍绑定旧用户目录，不能算搬迁完成。模型引用的 mesh/texture、UE package path、源资产和二进制资产都应明确解析根。运行层解析到本机绝对路径，持久化包内标识保持可移植。

完成标准：

- 相同声明输入在两个干净根目录产生相同的资产集合和内容指纹，不夹带开发机 Random150/Collection 临时资产；UE 包字节一致性另外评估，不混为同一要求。
- 在 packaged 运行中验证动态材质、光照/SpaceFX、相机 shader 和选定布局资源全部加载。独立包不依赖工作树 Saved/AssetImport、开发机绝对路径或未声明环境。
- 布局包复制到不同用户名/根目录后资产哈希不变；旧冻结批次仍能离线读取和审核；迁移生成独立报告，不改历史数据和旧 pool。

### S6：分块拆分，完成候选验收

F10 按以下次序从现有 `Private/BskSceneController.cpp` 提取内部实现：

1. 磁盘 writer 与网络 sender，已有 S4 故障用例保护。
2. PixelStreaming 输出及内部 API 兼容层。将版本敏感代码集中，不在此时升级 UE。
3. 天体/环境光照控制，验证灯光重建和布局切换。
4. GPU 回读/编码 pipeline，最后迁移最敏感的权威帧与完成顺序。
5. 场景 Actor 的实现移至 `Private/Scene/`；保留公开头文件和反射名称，更新 include 与实际目录地图。

每步独立提交和编译；核验析构顺序、线程停止、渲染资源释放与 Game Thread 所有权。只根据公共头文件实际需求调整 Build.cs 的 Public/Private 依赖，不机械移动所有模块依赖。任何反射名、资产引用变更必须另有迁移理由和加载验证。

完成标准不是“每个文件低于某行数”，而是场景编排、传输、落盘、预览、光照和采集 pipeline 的职责及所有权清晰，各自错误能被上层观察，关键数据顺序和性能没有无解释回退。

## 4. 删除、保留与迁移清单

本节是实施条件清单，不表示已执行删除。没有发现仅凭名称即可安全删除的整套“旧代码”；需按调用、数据复现和资产依赖逐项判断。

| 对象 | 处理方案 | 删除/迁移前提 |
|---|---|---|
| 独立候选目录中的 `__pycache__`、`.pytest_cache`、临时下载/构建中间件 | 可清理并由工具重建 | 无进程占用；不含唯一输入；不会扫描进入正式环境 |
| 已退役候选的 UE `Intermediate`、DDC、构建临时目录 | 可按具体 build-id 清理 | 发布/验收有可恢复产物；重建成本已接受 |
| UE `Binaries`、native `.so/.dll` | 按构建身份管理，不视为普通缓存删除 | 正式加载项和回退产物保留；已退役且可重建才删 |
| UE `Saved` | 分项检查后清理 | 先分出日志、恢复文件、导入 catalog 与唯一资产生成输入，不能整目录盲删 |
| `Content/Generated`、Random150 等本地资产 | 保留当前任务依赖；Cook 时隔离非声明项 | 被保留布局引用的资产不得按 Generated/日期命名删除 |
| 历史测试/检查结果 | 压缩或移至仓库外；删除可再生重复结果 | 当前报告引用、恢复实验、发布和批次证明链完整，必要指标/配置/哈希已保留 |
| `experiments/recovery-20261030-optimized-runtime/` | 保留恢复依据及尚未合并实现 | 不能因“实验”身份删除；先逐项核对 Git 落地与恢复覆盖 |
| 正式及两个旧保留批次、当前布局池 | 按已确认保留范围保留 | 本次没有新增数据删除或改写范围 |
| `src/space_sim_server/` 和其他兼容链接 | 暂时保留、文档标明用途 | 所有保留元数据/运行入口不再依赖后才退役 |
| 重复或废弃脚本 | 查调用者、测试、文档、打包和冻结来源后删除 | 列出原路径、替代入口、迁移结果与恢复提交；无证据不删 |
| 依赖快照、锁文件、能力文档 | 区分历史与当前，减少同义重复文档 | 历史信息仍有复现价值则移入索引，不把历史状态当当前能力 |

每次实际清理生成仓库外 `cleanup-manifest.json`，至少记录路径、内容类型、原因、依赖检查、SHA/大小、替代或恢复位置及结果。整理不依赖“先删干净再找缺了什么”的流程。失败尝试与取消批次是采集事实，不能为提高成功率统计而删除记录。

## 5. 验收与性能比较口径

### 5.1 候选版本验收矩阵

下列均为拟执行要求，现有五任务证据不能直接替代修改后的验收。

| 类别 | 输入与最低检查 | 证据 |
|---|---|---|
| 缺依赖基础 CI | 无 Basilisk 的干净环境；纯图缓存用例执行；意外 skip 仍失败 | 测试结果及实际导入列表 |
| 首次部署/重建 | 声明锁+UE/工具链，在干净候选环境执行；Linux/Windows 分别记录 | 安装/构建日志、构建 manifest；未测平台单列 |
| 版本拒绝 | 篡改一项源码、Shader、库、Engine 身份；另做文档修改对照 | 拒绝原因与重建后通过证据 |
| 网络/落盘/停止 | 部分包、FIN、解析失败、不可达、半开、ACK 取消、文件失败 | Automation 与实际停止延迟/错误回执 |
| 物理状态/复用 | 固定 seed 与输入；新 worker 对照 reset、拒绝后恢复、A→B→A | 状态、任务结果、时间戳、缓存 hit/reset/rebuild 计数 |
| 完整双相机 | 五个默认任务各完整执行；颜色/机位/布局切换；全部帧解码 | 数值与时序校验、图像清单和人工画面抽查；失败如实登记 |
| 独立发布包 | 独立目录/目标环境运行；无开发机资产借用；动态资产覆盖 | 包 SHA、Cook 清单、运行来源、完整任务证据 |
| 连续运行 | 建议首轮不少于 8 小时且不少于 100 次尝试，覆盖多布局、五任务与可控拒绝 | RSS/VRAM/句柄/线程/队列趋势、异常与恢复计数 |

同平台纯重构/缓存变更优先进行逐样本数值或哈希对照；跨平台浮点差异使用预先声明的容差和物理约束，不能事后放宽判定。GPU 图像受驱动等影响，必须检查解码、帧配对、时间戳、标定与场景内容，不把跨平台图像逐字节相同设为默认要求。

连续运行验收使用候选批次配置中的显式 RSS、VRAM/显存预算、队列/超时上限，并把实际阈值冻结在报告中；不得关闭既有 worker 退休策略来追求缓存命中。现有网络/GPU 有界队列约束保留。预热后资源趋势持续上升、跨任务残留或任一队列越界均需调查，不能仅以“没有崩溃”验收。

若确有任务/几何拒绝，验收记录真实原因；它与工程故障区分，但也不把失败尝试写成成功数据。全批 deferred audit 完成前，`pending_audit` 不等于最终训练合格。

### 5.2 性能报告必填项

- 同一硬件/驱动与负载条件、同一布局池/布局、任务版本、实际整数 seed、初态/颜色/机位、频率/分辨率、编码和审核策略。
- 冷启动与兼容第二条分别统计；建议同组至少三次，报告样本数、分布和失败样本，避免单次最优值。
- 录制 RTF = 录制仿真时长 / 录制墙钟时长；端到端耗时包含该口径明确列出的初始化、录制、封装、检查。重叠阶段不能重复求和。
- 分别报告尝试数/小时、快速检查通过数/小时、最终完整审核通过数/小时。批末审核耗时单列并纳入全批总耗时。
- 保留每阶段实测值；工程整理本身不预设“达到 0.8 倍”或固定百分比提升。F11 的直接目标是减少拒绝后重复初始化成本。

## 6. 切换、回退和交付管理

### 6.1 实施提交建议

| 交付单元 | 内容 | 主责角色 |
|---|---|---|
| 01 | Linux 部署入口修复 | 平台/部署维护者 |
| 02 | 纯测试隔离、native profile 与明确执行 | 物理/测试维护者 |
| 03 | 能力文档纠正 | 对应组件维护者 |
| 04 | 统一身份/schema、构建/启动/批次/发布核验 | 平台 + UE + 发布维护者配套 |
| 05 | 平台锁、CI 分层与 runner 接通 | 构建/测试维护者；可进一步拆分 |
| 06 | 重连状态与可取消连接修复 | UE 运行时维护者 |
| 07 | UE 直接落盘结果回报 | UE 采集维护者 |
| 08 | 初态拒绝后健康 worker/cache 保留 | 物理/采集维护者 |
| 09 | 隔离 Cook 与明确资产范围 | UE 资产/发布维护者 |
| 10 | 相对路径布局 schema 与单向迁移工具 | 资产/采集维护者 |
| 11+ | 按 S6 分块提取 Runtime 内部职责 | UE 运行时维护者 |

角色是职责划分，不代表已有特定人员或 runner。单人也可按顺序完成。交付单元 04 可先提交契约与纯校验库，再提交跨端完整接入；运行门禁未接完时不能宣称已解决 F03/F12。

### 6.2 将来切换采集的步骤

只有完成候选验收并进入实际部署任务时才执行以下步骤，本次写方案不执行：

1. 候选源码、环境、native/UE 构建与资产清单固定，生成不可变 build-id 和验收索引。
2. 保存正式批次进度、来源与完整数据；按照既有停止流程结束当前任务/批次并确认进程退出，不在写文件期间替换库。
3. 选择新版本入口和环境，启动门禁核验磁盘与实际加载身份；正式新批次使用当前时间种子策略并保存实际整数。受控 A/B 验收单独使用相同显式 seed。
4. 第一轮检查五种任务的状态、视频、配对和错误计数，随后按定义执行连续运行及批末审核。
5. 如失败，停止候选；新产生的数据保留实际版本与失败原因。恢复旧代码、旧环境、旧二进制和资产指纹；以独立批次重新启动，避免在一个批次中混入两版实现。

### 6.3 最终交付清单

- 更新后的目录地图、能力表、命名规则引用和操作文档。
- 可安装并经过相应平台验证的依赖锁、Engine/ABI 清单。
- 从构建到运行/发布的一套来源契约、自动生成器与拒绝错误版本的门禁。
- 快速/native/UE/GPU/发布各层实际运行状态和证据；未执行项明确列出。
- Cook 输入清单、独立包验收、布局迁移工具与旧格式兼容证明。
- 网络、磁盘、停止、缓存与生命周期故障验证；同输入性能对照报告。
- 有保留/删除依据及恢复路径的清理清单；源码不再存放运行日志和检测大产物。

推荐先交付 01–03，再完成版本门禁与 CI，随后独立推进可靠性、Cook 和缓存改进，最后进行控制器职责拆分。每一项以其实际验收关闭，不以“目录看起来整齐”或“已合并代码”替代工程成熟度。
