# 采集功能基线与合并验收

本次采集功能整理以 `zmh_linux@5bd9a87` 为基础，合并此前本地功能，并更新简洁英文任务指令。同步时须同时核对工作区改动、未跟踪模块/测试及本地 UE 资产，不能只检查函数定义是否存在。实验数据、视频、日志及临时生成的 UE 资产不纳入源码提交。

随机布局运行依赖补交：通用部件的 UE 资产通过 Git LFS 同步，`config/collection-base-assets.json` 使用仓库相对路径与资源 SHA。布局准备和独立布局导入工具默认解析该清单，不再要求本机历史 `Saved/AssetImport` catalog。每套随机布局的七孔独立表面、配套底板和碰撞几何继续现场生成并校验；不交付固定布局代替随机能力，也不把历史池标为重新认证。通用运行资源属于代码配套依赖，区别于每次实验的临时资产。

2026-10-27 最新独立实采：抽屉、航空/导向/圆柱插头、锁杆共五种任务成功且通过数据校验；航空螺丝在接近阶段发生夹爪连杆/手指与邻近锁杆紧固件接触，120秒阶段超时，尚未开始旋拧。相同场景的12秒只读原生复现与原录像600个共同时间戳的关节真值完全一致。此问题尚未修复，不得声明六任务已全部通过。锁杆重新接近时全局视角仍有局部遮挡，黑色航空插头在黑色背景中辨识度有限；未执行本轮全过程400 Hz接触验收，动作成功不代表训练数据已全部合格。

新生成记录的指令使用简洁英文：抽屉“Open the drawer approximately 8 cm, then close it fully and release it.”；锁杆“Loosen the locking rod's fastener, fold the rod into its clip, then raise it and tighten the fastener.”；航空螺丝“Unscrew the <source color> aviation bolt from its holder, then screw it fully into the <target color> hole.”。三类插头沿用原简洁英文颜色指令，具体孔身份和成功判据仍由控制器和metadata保存。历史实验保留原始instruction，不回写为新模板。

2026-10-27 本次同步包含远端圆柱夹持高度、接触开度和抬升避障撤离修正，以及螺纹任务速度配置。圆柱同时保留本地慢速松爪，组合版本为 `plug-insert/13-cylindrical-clearance-gentle-release`；旧两种 v12 记录继续兼容。远端约58秒成功记录属于远端版本，不能作为本地组合版本的实测结果。此前本机圆柱失败采集未包含这条远端提交。

本机最新整理证据在 `/home/spacesim-zmh/migration/repository-integration-audit-20261027/`；性能索引为 `/home/spacesim-zmh/migration/performance-current.json`。索引未完成时，不能把旧测速值作为恢复后版本的结果。

## 当前应保留的功能

| 功能 | 当前实现与约束 | 主要入口/验证 |
|---|---|---|
| 任务注册 | 抽屉完整循环、航空/导向/圆柱插头插入、航空螺丝装配、锁杆循环共六种网页采集任务；旧 drawer_lift 为诊断入口 | collection_jobs.TASKS、threaded_tasks；threaded_collection_integration 与各控制器测试。代码/单测通过不等于各任务在任意随机场景必成功 |
| 初始关节 | 六轴相对基准独立均匀偏移，支持0–50°，本批±30°；保留限位、初始碰撞、接近路径检查 | collection_jobs、drawer_initialization、collect_scripted_tasks；网页/API/CLI范围一致 |
| 安装单元随机布局 | 移动安装单元与对应底板开口，保留实体间距、操作包络、撤离通道及原生几何验收；布局入池不再要求逐套运行完整任务轨迹，每条采集仍保留初态及必要路径预检 | independent_task_box_layout、build/check工具、LayoutPool、drawer_initialization |
| 布局切换 | 常驻UE按新manifest更新同ID几何/材质/灯光，避免保留旧孔位；A→B→A复用 | BskSceneController.cpp/.h、BskRuntimeTests、test_capture_on_demand_renderer |
| 七孔颜色与指令 | 七个目标孔从红/橙/黄/绿/黑/蓝/紫不重复分配，源件独立随机；pickup/storage座中性；孔/件身份配对不随颜色改变 | socket_colors、scripted_tasks/randomization；颜色SHA绑定实际模型，缺可分色几何时拒绝虚假颜色指令 |
| 英文指令 | 六种任务各20条长短英文模板，独立随机选句并冻结记录；按真实源件/目标孔颜色填入，目标身份和技能细节另存metadata | randomization.task_instruction及颜色/任务边界测试 |
| 网页固定布局输入 | 可配置固定model/catalog并绑定SHA；固定模式各条布局不变，显式随机模式使用布局池；缺可分色模型时启动前拒绝 | collection_inputs、collection_jobs/API、网页能力提示；真实受控子进程argv验证 |
| 五固定全局机位 | v7：卫星左侧实体安装面，7 cm等间距、45 cm高度、85°视野；覆盖任务板及上方工作区；新场景只用presets；历史v1–v6保留回放标定；三任务运动快照视觉验证不等于六任务全场景验收 | overview_camera、scene_runtime、采集CLI、网页说明；实体STL支撑与视锥测试 |
| 全局补光与腕相机 | 既有overview_camera_lamp挂在最终全局相机上，宽锥、2m范围；保留腕部灯及四种腕部相机模式 | camera_lighting、RenderComponent、UE相机挂灯；不能仅测helper，须验证装配和实图 |
| 初始化精简 | 状态发布副本删除不再引用的碰撞mesh；完整动力学碰撞仍由原模型加载 | publication_scene、scenario_sarm_grasp；summary保存分段与发布资产计数 |
| UE/动力学复用 | 常驻进程中每条新建场景、任务、通信会话和随机流；失败/线程遗留/条数或RSS阈值时退休 | dynamics_worker、collect_parallel→collect_scripted_tasks→record_scripted_task→run_scripted_task |
| 严格采集时序 | 400 Hz物理、200 Hz任务/腕力、双1024² RGB/50 Hz；预录对齐50Hz；终态两路RGB匹配并入writer后立即冻结采集区间，再执行OFF/封装，避免轮询间隙追加成功后的帧 | sampling、ScriptedObservationRecorder、compact/LeRobot、终态屏障测试 |
| 启动与流式编码 | Linux禁用失败的Zen自动启动重试；H.264流式编码，显式veryfast配置；有界队列、完整解码，无丢帧提速 | collection_runtime、recording-veryfast.json、streaming_video、每相机encoding_metrics |
| 保持力计算优化 | 小向量算术、阈值预计算及原生候选 integration 状态恢复/成功 step2 复用；不支持的模型保留原复制路径，频率和物理参数不变 | plug_retention及component；数值和native状态等价验收 |

## 本次发现的同步遗漏

2026-10-27同步前的stash保留了全局补光和v6机位。处理stash冲突时曾对冲突文件统一选择 `--ours`，覆盖了部分本地功能；随后仅恢复部分布局文件。后续mixed reset改变引用/index，不应误称为删除补光调用的直接原因。具体命令、时间点、文件指纹见本机相机 `HISTORY.md`。

本次逐项恢复了渲染补光接入、v6机位、UE布局/灯光重建和离线预热、终态写入屏障、预录时间对齐、分阶段超时及显式布局绑定。网页/API的±30°能力与CLI重新统一，远端新增螺纹任务保留。颜色模型先实际着色并校验SHA，再生成录制指令；冻结颜色记录SHA不匹配会被拒绝。

实图复核还发现非螺纹任务的渲染入口使用未着色的 `source_model`，导致元数据正确而视频中的部分孔和插头仍保留旧颜色。现在所有任务都把实际生成的 `model` 传给渲染器，summary记录实际渲染模型路径与SHA。验收必须比对真实视频、实际渲染模型和episode颜色记录，不能只检查元数据。

网页原先未向采集子进程传固定独立布局模型。现在通过 `SPACE_SIM_COLLECTION_FIXED_MODEL` 与 `SPACE_SIM_COLLECTION_FIXED_CATALOG` 配置，验证几何/材质绑定后保存输入SHA；未配置可分色操作台的颜色任务在启动前明确拒绝。配置及Linux启动方法见 [平台采集](PLATFORM_AUTOMATED_COLLECTION.md)。这项入口验证不等同网页已经实测使用了CLI的进程复用/veryfast参数。

## 代码通过与实验通过分开记录

- Python回归、UE自动化、实际GPU画面和完整采集分别保存证据；不能用其中一种代替其他层。
- 原先 `collection-speed-20261027-final` 两条数据只通过结构/时序验证；全局画面过暗，标为性能诊断样本，非训练合格数据。恢复后需独立新目录实采。
- `collection-speed-20261027-validated` 已恢复补光，但实图仍存在上述颜色不匹配，也标为非训练合格数据。最新完整基线采集使用 `collection-baseline-20261027` 独立目录，是否通过以其中实图审计及总报告为准。
- 布局池现按用户要求采用几何及操作间距验收，不再要求逐布局完整任务轨迹认证。`random150-20261028/pool.json` 已接入150套几何合格布局；旧轨迹诊断记录保留为历史证据，不作为入池门禁，也不代表当前任务均成功。
- 灰色螺栓核查更正：远端平台模型已包含初始孔母螺纹 `bolt_storage_thread_sdf` 及显式接触对 `physical_bolt_storage_thread`，保留 `bolt_freejoint` 不代表没有物理螺纹保持。此前实采的旧 layout000 及圆柱 episode 实际动力学模型缺少这两项，原入口也仅为航空螺丝任务补齐。现已统一为所有脚本任务共用 `write_model_variant` 检查/补装母螺纹，并记录 `model_physics`；原布局文件不改写。布局生成器同步修正两处母螺纹的安装单元归属。真实旧布局的8个任务名（含兼容/诊断入口）生成检查及独立原生静置/直拉/旋出夹具验收通过；这不等于新版本已完成所有任务双相机实采或整个随机布局池重新认证。证据在本机 `migration/unified-collection-model-20261027-203817/`。

## 后续修改与合并顺序

保存当前工作区和资产清单；逐功能对比旧本地版本、远端新增内容和最终入口；运行相关Python/网页/UE回归；重新构建变更的UE模块并核对加载文件；用连续两条完整采集检查布局/颜色/机位切换、零点复位、无丢帧与实际画面。基线完整后才进行新的性能改动。

性能统计使用已验收attempt的 `end_to_end_wall_seconds`，其中已经包括validation。记录第二条完成时间减第一条完成时间；初始化、UE启动可能重叠，不能重复求和。测试场景、任务、源码、资源、命令和构建指纹随报告保存。详见 [采集性能说明](COLLECTION_PERFORMANCE.md)。

2026-10-28 当前按布局顺序采集入口为 `tools/collect.py`，配置为 `config/collection.json`；六任务统一管理，默认启用五种、航空螺栓关闭。每个布局颜色/机位冻结，每条机械臂独立±30°及物理状态复位；失败跳过、不补成功数，UE跨任务复用。目录迁移和使用方法见 [按布局顺序采集](LAYOUT_SEQUENCE_COLLECTION.md)。网页/API保持单任务批采，旧安装包未自动重建。
