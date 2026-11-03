# 仿真数据采集配置参考

本文按 2026-10-31 工作区 `30dea8f` 的实际入口、参数模型和调用链核对，适用于源码采集器。配置、计划、实际运行结果是三层证据：配置写了什么，要看冻结文件；参数最终是否生效，要看每条 `recording/summary.json` 和 `recording/dynamics/summary.json`；录制完整性要看完整审核，训练资格还需独立的质量与选样审查。旧发布包须按包内源码与 BUILD 信息单独核对。

本文只说明行为，不执行采集或恢复。批任务操作见[全流程指南](../guides/simulation-data-workflow.md)，动力学装配见[物理模型参考](physics-model-configuration.md)，成功条件与审核边界见[任务与数据验收](task-and-data-acceptance.md)。下文 `[S1]` 等编号对应文末的实际代码出处。

## 1. 入口与配置优先级

本工程没有一个覆盖所有入口的全局采集配置加载器。尤其不能通过修改 `server/config/collection.json`，推断网页或旧安装包也会使用相同参数。

| 入口 | 配置与覆盖顺序 | 默认行为及边界 | 落盘证据 / 出处 |
|---|---|---|---|
| 日常顺序采集 `server/tools/collect.py` | `SequenceConfig` 字段默认 → `--config` 指定 JSON（未指定时 `server/config/collection.json`）→ `--layouts`；再由 `run_attempt` 强制加入录制选项 | 每个布局按启用任务顺序各尝试一次；失败不补成功数；强制 `--reuse-renderer --reuse-dynamics --offline`；默认 `recording-veryfast.json` | `config.snapshot.json`、`plan.json`、`progress.json`；[S1] |
| 同一顺序批次 `--resume` | 只读输出目录中的 `config.snapshot.json`；拒绝同时给 `--config` 或 `--layouts` | 校验原计划、输入哈希和运行版本；继续未尝试任务，不自动重试已中断或失败任务；不是只审核模式 | `inputs.json`、`launch-provenance.json`；[S1][S8] |
| 单任务批采 `server/collection/collect_scripted_tasks.py` | argparse 默认 → 明确 CLI 参数；轨道/全局相机随机化开关未给时继承 scene；录制文件独立加载 | 成功目标 `count=5`、最多 `max_attempts=15`；默认不复用 UE/动力学、不启用 offline；默认 `recording.json` | 批次 `summary.json`、逐次 `scene.json`、`recording/summary.json`；[S2][S3] |
| 并行批采 `server/collection/collect_parallel.py` | `CollectionConfig` 默认 → `collection-parallel.json` → `--workers` 截取配置的 worker 列表、`--episodes-per-worker` → 每个 worker 的 GPU/线程配置 | 仓库 JSON 配置 2 路、每路目标 2 条、最多 4 次；该 JSON 显式开启 UE/动力学复用和 offline；录制文件默认 `recording.json` | 总 `summary.json`、`worker-XX/plan.json`、`worker-XX/recording.json`、`resources.jsonl`；[S4] |
| 网页自动采集 `POST /api/collections` | `CollectionCreate` 默认 → 请求字段 → manager 生成 scene、分配 worker、CLI 参数 → 低层采集器默认 | 默认单任务、目标 5 条、2 worker、最多 15 次。manager 当前未传复用/offline/录制配置参数，因而不会自动得到顺序入口的进程复用或 veryfast 配置 | `collection_jobs/<id>/job.json`、`scene.json`、worker 日志与 summary；[S5] |
| 发布采集工具 `space-sim-collect run` | 包内 CLI 默认 → CLI 参数 →加载所选录制文件 →复制为输出目录 `recording.json` →固定子进程参数 | 当前源码默认 `recording-veryfast.json`，强制复用和 offline；动力学退休 4 条、复用预热 5 秒；旧包不随源码自动更新 | `launch.json`、`recording.json`、`batch/summary.json`；[S6] |
| 单条录制 `server/collection/record_scripted_task.py` | CLI 默认/显式值 →录制 JSON 的 Pydantic 默认补齐 →脚本录制约束 →螺纹任务帧数自动扩容 → CLI `gpu_index` 覆盖视频 GPU | 这是低层装配入口，不能直接等同于批次计划。场景运行频率被脚本采集契约覆盖 | `recording_settings`、`recording_frame_limit_override`、`scene_instance.runtime`；[S3] |
| 交互场景录制 `POST /api/episodes/start` | 后端启动时加载 `recording.json`；文件显式 `capture_rate_hz/dataset_capture` 优先于启动器默认；EpisodeStart 显式字段覆盖录制默认；已有场景的实际采样率最终约束 FPS | 显式请求 FPS 与场景不同会拒绝，不能通过录制按钮改变运行中场景的权威频率；这也不是 `/api/collections` 的批采配置 | `/api/recording/config`、场景 runtime、episode metadata；[S7] |

补充规则：

- 顺序入口的 `layout_pool/scene/recording_config/adapter/unreal` 支持 `~`，相对路径以**所选配置文件所在目录**为基准。普通单任务 CLI 的路径以进程当前目录解析。并行入口的录制文件/布局池以并行配置目录解析，不具有顺序入口相同的 `~` 展开逻辑；跨机配置优先使用明确路径或规范相对路径。
- 顺序入口的 `--layouts N` 设置 `layouts_per_pass=N`，同时清除 `layout_cycles`；`passes` 保持原值。因此它不是在所有情况下把整个批次限制为 N 个布局。
- `load_recording_config` 遇到不存在的文件会返回 `RecordingConfig()`，并非统一报错。该模型的 `streaming_encoding=false`、`scripted_native_geometry=false`，与仓库两个录制 JSON 不同。并行/发布入口显式检查文件存在；顺序入口后续计算输入哈希也会要求文件存在。直接调用低层入口时尤其要检查最终 `recording_settings`。
- JSON 不做任意深层拼接：已声明的 Pydantic 字段补默认值，未知字段被拒绝。不要把另一入口才支持的参数写入 `collection.json`。

## 2. 种子、随机流与复现

| 入口 / 字段 | 默认或生成方式 | 覆盖与运行证据 | 出处 |
|---|---|---|---|
| 顺序入口 `seed` | 仓库 JSON 为 `null`；把完整 `time.time_ns()` 转十进制字节，取 SHA-256 前 4 字节的大端整数，再与 `2**31-1` 按位与 | 配置可给 `[0, 2**31-1]` 的整数；解析后的整数写进 `config.snapshot.json`；恢复沿用，不再取时间 | [S1] `load_config` |
| 单任务批采 `--seed` | `2026101501` | CLI 覆盖；第 i 次尝试使用 `base_seed+i`，i 从 0 开始，失败也消耗一个 seed | [S2] `main` |
| 并行 `seed` | 模型及仓库 JSON 均 `2026101900` | 第 i 个 worker 从 `seed+i*max_attempts_per_worker` 开始，worker 内按尝试递增 | [S4] `worker_plan` |
| 网页 API `seed` | `20261016` | 请求覆盖；manager 按分配给前面 worker 的尝试预算累加，避免 worker 之间 seed 区间重叠 | [S5] `CollectionCreate/start` |
| 发布工具 `--seed` | `2026102800` | CLI 覆盖；调用低层单任务批采。该入口没有自动时间 seed | [S6] `main` |

时间 seed 是以启动时钟为输入的确定性散列，不是硬件真随机，也没有额外引入 `secrets/os.urandom`。范围只有 31 位，不能承诺任意两次启动永不碰撞；其用途是避免每次新批次固定使用同一个 seed，并保留复现入口。

顺序计划采用以下明确关系（下标从 0 开始）：

```text
布局排列 = PCG64(SeedSequence(seed, SHA256(ORDER_STREAM)前8字节, pass_index)) 的 permutation
ORDER_STREAM = layout-sequence-without-replacement-v1
layout_seed = seed + (pass_index * pool_size + position_in_pass) * 7
task_seed = layout_seed + TASK_NAMES中该任务的固定索引 + 1
```

单遍布局无放回；跨遍允许重复布局。`layout_cycles` 指总布局组数，并优先决定遍数；默认 1000 组不是 1000 套互不重复布局。当前 3 个启用插头任务意味着 3000 次尝试，不能写成 3000 条已合格数据。

同一个布局组固定布局、颜色和全局相机；各任务单独生成六关节初态。六关节偏移使用 `numpy.PCG64(task_seed)` 独立均匀采样 ±`range_deg`；当前显式配置为0，所有偏移为零；基准来自 source scene 的 `randomization.arm_joint_position_rad`。指定 `scene` 后，`baseline_joint_deg` 不再重写该 scene 的关节基准。顺序入口会清除旧任务的可变字段，强制开启初始化和全局相机预设随机化。颜色、任务指令等使用有名称的独立 PCG64/SeedSequence 流；全局相机当前为 v7 的 5 个固定机位，通过独立的 `random.Random(stream:seed)` 选择，轨道相位也有独立流。[S1][S9]

复现至少保留：整数 seed、冻结 plan、source scene、布局池成员及资产哈希、实际颜色/机位/关节初态、录制配置内容、任务版本、运行源码、原生库、UE build identity 和解释器环境。相同 seed 只保证同版本算法的随机输入可追溯，不能单独保证不同版本/平台/GPU下逐位一致。

## 3. 顺序入口参数表

下表“默认”是仓库 `server/config/collection.json` 的当前值；与 `SequenceConfig` 的字段默认有差异处单独说明。所有字段可在新批次所选 JSON 中修改；恢复不接受修改原冻结配置。[S1]

| 参数 | 单位 / 当前默认 | 约束与实际作用 | 最终证据 |
|---|---|---|---|
| `schema` | `layout-sequence/1` | 仅支持此版本 | `config.snapshot.json` |
| `layout_pool` | 路径；`~/data/layout-pools/random150-20261028/pool.json` | 必需；验证几何/哈希，不代表池内每套完整轨迹均成功 | `pool.snapshot.json`、plan 的 `pool_sha256`、逐组 initialization |
| `scene` | 路径；`null` | 空时生成 `sarm-task-box-20261012` 模板；否则使用指定场景为基准 | `source-scene.json` |
| `tasks` | 有序数组；仅航空、导向、圆柱三种插头启用 | 按目录顺序执行；其他七种任务关闭；至少启用一项，不允许重复 | plan 的 `task_order/jobs` |
| `passes` | 遍；1 | 1–1000；仅当 `layout_cycles=null` 时决定完整遍数 | plan 的 `groups/pass_number` |
| `layout_cycles` | 布局组；1000 | 1–1,000,000 或 null；模型默认 null；优先于 passes | plan 的组数、`total_attempts` |
| `layouts_per_pass` | 布局数；null | null 表示池全部；整数 1–1000 且不能大于池规模 | 每遍的 groups；`--layouts` 可覆盖 |
| `seed` | 整数；null | 新批次按上一节解析；恢复固定 | snapshot、plan、场景 `collection_config` |
| `range_deg` | °；0 | 有限数 0–50；0表示固定基准姿态，无随机偏移；模型/单任务CLI缺省仍为30，仓库顺序配置显式覆盖为0 | `initialization.offset_deg/half_range_deg` |
| `baseline_joint_deg` | °；`[90,-60,60,0,-90,0]` | 6 个有限值；仅默认场景生成使用；不是一般关节限位保证 | source scene、逐条 initialization |
| `randomize_orbit_phase` | bool；false | 开启后逐任务重新采样真近点角；环境变化也可能使物理图无法复用 | `orbital_phase_randomization`、environment |
| `recording_config` | 路径；`recording-veryfast.json` | 相对配置目录；原文件内容的哈希冻结，但不会自动复制文件 | `inputs.json`、recording summary 的 `recording_settings` |
| `adapter` | 路径；未写/模型 null | 默认 server 的同级 `ue-adapter`；显式值传给低层 | runtime provenance、UE实际模块路径 |
| `unreal` | 路径；未写/模型 null | 显式配置优先；其次 `UE56_ROOT`；Linux默认 `~/engines/UE_5.6`，Windows默认安装路径 | UE manifest、renderer runtime report；[S10] |
| `port_base` | TCP端口；24000 | 1024–65532；+0 control、+1 capture、+2 render、+3 signalling；四端口须不同且可绑定 | 子命令、日志、冻结配置 |
| `gpu_index` | GPU索引；0 | 0–255；同时覆盖 UE `-graphicsadapter` 与 video `gpu_index`，不会自动把 h264 改成 NVENC | recording settings、renderer启动参数 |
| `warmup_seconds` | 墙钟秒；5 | 0–60；首条或渲染场景键变化后，在初态稳定期间等待 | recorder `warming_renderer/recording` 日志 |
| `reuse_warmup_seconds` | 墙钟秒；0 | 0–60；只有模型路径、catalog、机位和颜色键相同且已有录制时生效 | 场景键、日志；不表示所有初始化为0 |
| `simulation_startup_timeout_seconds` | 墙钟秒；600 | `(0,7200]`；等待动力学就绪；UE监听另有240秒限制 | 启动阶段日志、失败原因 |
| `task_wall_timeout_seconds` | 墙钟秒；600 | `(0,7200]`；Linux外层覆盖场景准备、启动及录制；有界清理可能超出截止时刻；批末完整审核另计 | progress 的 category/timing、attempt summary |
| `max_consecutive_runtime_failures` | 次；3 | 1–20；连续基础设施异常达到阈值后停止；正常任务失败、超时或场景拒绝不全等价于此计数 | progress 的 `runtime_recovery/error` |
| `minimum_free_disk_gib` | GiB；20 | 非负；布局组/任务前检查剩余空间；磁盘满、只读、权限等致命存储错误停止 | progress 的 `error` |
| `dynamics_max_episodes` | 请求次数；50 | 1–1000；含正常失败的已完成请求，任务之间退休；不限制UE renderer的条数 | worker result 的 `completed_episodes/retired` |
| `dynamics_max_rss_mb` | MiB；8192 | 正整数；实现乘 `1024*1024`，字段名虽叫mb，实际二进制MiB；只检查动力学worker | worker `rss_bytes/retirement_reasons` |
| `threaded_speed_profile` | `fast_collection` | 仅 `default/fast_collection`；只传给锁杆、航空螺栓；低层单任务入口默认是 `default` | task config、recording summary |

顺序入口没有 `workers`、`reuse_renderer`、`reuse_dynamics`、`offline`、`randomize_overview_camera`、`validation_mode` JSON 字段：当前分别是单路、强制复用、强制离线、强制预设相机随机化、继承低层默认延后审核。需要这些不同语义时，应选择对应入口或另做受验收的实现变更，不能加入一个被模型拒绝或未消费的字段。

## 4. 录制、采样、相机与编码

脚本采集强制 400 Hz 物理、200 Hz 脚本任务、200 Hz 腕力、50 Hz 权威渲染/RGB；通用 IK 默认是 **100 Hz**，不能把脚本任务的200 Hz写成通用IK频率。双相机均为1024×1024；脚本校验器要求这套频率、相机和图像尺寸，修改普通录制配置并不能任意改变脚本数据契约。[S3][S11]

下表默认以顺序入口加载的 `recording-veryfast.json` 为准。`recording.json` 与它目前仅差是否显式提供 `encoder_preset: veryfast`；未指定 preset 表示交给编码器默认，不保证等价于 veryfast。[S12]

| 参数 | 单位 / 当前值 | 覆盖与含义 | 运行证据 / 检查出处 |
|---|---|---|---|
| `dataset_capture` | bool；true | 交互场景默认开关；脚本 `scene_copy` 强制权威采集为true，不用它关闭采集 | `scene_instance.runtime`；[S3][S7] |
| `capture_rate_hz` | Hz；50 | 通用配置允许1/2/5/10/25/50，脚本入口只接受50 | metadata `dataset_fps`、时间戳、完整validator；[S3][S11] |
| `camera_ids` | 两个ID | `teleop/camera/spacecraft_overview`、`teleop/camera/sarm_wrist_cam`；脚本必须同时具备 | metadata、platform `camera_keys`；[S3][S12] |
| 相机尺寸 | 像素；1024×1024 | 来自共享常量/相机模型，录制JSON没有width/height字段 | 视频流与逐帧数据；[S11] |
| 全局机位 | 5个固定预设，当前v7 | 顺序入口同组固定；每次新布局组按layout_seed选；整条不漂移 | scene `overview_camera`、逐次完整机位验证；[S9] |
| `capture_products` | `["rgb"]` | 当前脚本只接受RGB；没有把深度/分割当成已支持录制产品 | `recording_settings`、capture manifest；[S3] |
| `max_frames` | 帧；18000 | 通用范围1–108000；螺纹任务可升至 `ceil(max_sim_seconds*50)+1`，不修改原JSON；顺序默认600秒预算下锁杆为30001 | `recording_frame_limit_override`、最终settings；[S3] |
| `save_raw_camera_images` | bool；false | 不另存逐帧原始相机图片；仍保留训练视频 | episode文件、settings；[S12] |
| `storage_profile` | `compact` | 可选compact/diagnostic；compact使用Parquet/sidecar，不代表去掉200Hz腕力 | `lerobot/meta/platform.json`、sidecars；[S12][S13] |
| `save_dynamics_trace` | bool；false | false传入 `--no-save-trace`；动力学summary仍保留 | 是否存在 `dynamics/trace.jsonl`、summary；[S3] |
| `scripted_native_geometry` | bool；true | 传入动力学 `--native-geometry`；缺省模型值是false，不可用缺文件回退推断已开启 | dynamics `native_geometry`、preflight/plans；[S3][S14] |
| `capture_backpressure_poll_s` | 秒；0.002 | 范围0.001–0.1；等待背压解除的轮询间隔，不是每帧固定睡眠，更不是降频 | `recording_work_wall_seconds.backpressure_wait_s`；[S14] |
| `video.codec` | `h264` | 可选h264/h264_nvenc；h264是软件编码，设置GPU索引本身不切换编码器 | platform `video_encoding`、编码metrics；[S12][S13] |
| `video.gpu_index` | 索引；文件未写，模型null | 单条CLI/每worker GPU优先覆盖；显式NVENC GPU需要streaming=true | recording settings；[S3][S4] |
| `video.streaming_encoding` | bool；true | 边录边编码；模型缺省false；发布工具显式要求true | platform与video_encoding_metrics；[S6][S13] |
| `video.encoder_threads` | 线程；2 | 范围1–64；传递给每相机编码器，不是整个采集进程总线程数 | settings、encoder options；[S13] |
| `video.image_writer_threads` | 线程；0 | 范围0–64；传给LeRobot图片写入器，独立于视频编码线程 | settings；[S13] |
| `video.encoder_queue_maxsize` | 每相机队列帧数；30 | 范围1–1024；严格队列满时等待，不静默丢帧；停滞超时60秒会失败 | `frames_accepted/frames_encoded/queue_wait_*`；[S13] |
| `video.encoder_preset` | `veryfast` | h264支持x264预设，NVENC支持p1–p7；需要streaming=true；未设置时不传preset | video_encoding_metrics；[S12][S13] |
| `video.quality` | 0–51；30 | h264传CRF，NVENC传固定QP；不是百分比 | 编码设置；[S13] |
| `video.keyframe_interval` | **帧**；2 | 传编码器GOP `g=2`，不是2秒；范围1–250，NVENC至少2 | 编码设置；[S12][S13] |

终态边界是末条状态与两路权威RGB已经匹配并进入writer，随后冻结本条接受区间，再关闭采集和封装视频。WebRTC预览和权威数据通道独立；`t.MaxFPS=90` 或预览90fps不是训练数据90Hz。offline禁用WebRTC及不需要的预览路径，仍进行真实GPU权威图像采集。[S3][S10]

单条批采调用会明确传 `--substeps 1`。其 `max_sim_seconds` 对普通任务为 `min(450, task_wall_timeout_seconds或900)`，螺纹任务为 `min(900, task_wall_timeout_seconds或900)`；这是代码将同一个数值用于仿真上限的独立计算，单位依然是仿真秒。录制墙钟上限另由 `--timeout-seconds` 控制。直接调用单条录制入口则有自身任务默认值，不能套用上述批采默认。[S2][S3]

## 5. 资源、复用与退休

“复用”需要分三层核对：UE进程复用、动力学Python进程复用、物理图重置复用。PID相同只证明进程相同，不足以证明省掉了物理图初始化。

| 层级 / 参数 | 当前行为 | 覆盖入口与证据 | 出处 |
|---|---|---|---|
| UE进程 | `CollectionSession`持有一个 `PersistentRenderer`，跨任务连接；退出/异常后关闭并重新创建 | 顺序/发布强制开启，单任务CLI需 `--reuse-renderer`；查 renderer PID和runtime report | [S1][S2][S10] |
| 动力学进程 | 常驻worker重复接受请求，保留导入/native库；每条有独立request/token/结果 | 顺序/发布开启，单任务需 `--reuse-dynamics`；查 `dynamics_worker.pid/request_id/result` | [S2][S15] |
| 物理图 | 常驻worker中默认 `SPACE_SIM_REUSE_GRAPH=1` 才创建 `PhysicsGraphCache`；兼容则reset，其他值关闭 | 环境显式覆盖；查 `physics_graph_reuse_enabled/reused/id/miss_reason`，不能只看配置 | [S14][S15] |
| 兼容条件 | 模型内容指纹、environment、机位/相机随机化、腕相机模式、substeps、发布stride、control/render端口、attitude配置和catalog哈希 | 变化会重建；外扰接线或轨道drag启用时关闭图缓存 | [S16] `compatible` |
| 图重置 | 恢复被动/自由关节qpos/qvel，重置时间/调度/控制缓存/计数，重新建立通信session；不调用会重新编译publication MJScene的SelfInit | 查 `restore_existing_graph`；成功复用时 `build_physics/initialize_simulation=0` | [S16] `restore` |
| 原生初态预检 | 生产路径仍有 `validate_initial`；不执行外层完整轨迹预演；每阶段规划按需进行 | `preflight_wall_seconds=0` 仅说明外层移入worker，不能说预检完全关闭；查 `native_preflight` | [S2][S14] |
| 正常拒绝 | 初态不满足或普通任务失败可返回1；清理完成且未触及阈值时仍允许下条复用 | 请求 `returncode=1` 不自动代表进程退休 | [S15] |
| 退休条件 | 异常返回、残留线程、达到请求数上限、RSS阈值 | `retired=true` 和 `retirement_reasons`；检查发生在任务之间；父进程丢失另由watcher退出 | [S15] `serve` |
| worker线程/CPU | 并行入口 `math_threads=1`，空CPU affinity表示不绑核 | 并行在数值库导入前设置OMP/OPENBLAS/MKL/NUMEXPR线程；顺序入口不自动套用此限制，继承启动环境 | [S4] `run_worker` |
| 碰撞过滤模式 | 常驻worker对 `CYLINDER_FILTER_MODE/MESH_FILTER_MODE/THREAD_FILTER_MODE` 用 `setdefault(...,"2")` | 显式环境值优先；普通独立动力学进程不能据此推断同样生效 | dynamics `runtime_provenance.collision_filter_modes`；[S15] |
| UE缓存 | Linux使用 `InstalledNoZenLocalFallback`；`UE-LocalDataCachePath` 指向所选项目 `Saved/DerivedDataCache` | 规避失败的Zen自动启动等待；属于本机生成缓存，不随源码提交 | [S10] |

并行配置另外提供 `sample_seconds=3`（资源采样墙钟间隔）、`worker_timeout_seconds=7200`（协调器从开始到所有worker结束的总墙钟保护）、每worker `gpu_index/cpu_affinity/math_threads`。第i个worker的端口基数为 `port_base+i*10`；默认两个worker都指向GPU0，不能理解为自动分配两块GPU。资源summary的CPU百分比按单核100%计，可超过100%。[S4]

## 6. 计划、产物、审核与版本身份

顺序批次输出结构中的 `<组>` 形如 `pass-001/121-seed-...`，任务名保持协议中的snake_case：

```text
<批次>/
├── config.snapshot.json       # 解析后的批次配置，包含整数seed和绝对路径
├── plan.json                  # 布局/任务顺序、每条seed、池哈希
├── pool.snapshot.json         # 固定布局池成员；不等于复制所有模型资产
├── source-scene.json          # 本批次基准场景
├── inputs.json                # 配置/场景/录制配置内容SHA
├── launch-provenance.json     # 真正执行前冻结的运行身份；plan-only不会生成
├── progress.json              # 状态、逐job结果和批次墙钟
├── collector.lock             # 所属协调器锁，不是依赖锁文件
├── cancel.request             # 存在即请求停止
├── layouts/<组>/initialization.json
├── attempts/<任务>/<组>/
│   ├── summary.json           # 此任务的一次尝试结果
│   └── attempt-001-seed-<n>/
│       ├── scene.json / prepared-model.xml / recording.log
│       ├── validation.json
│       └── recording/
│           ├── summary.json / scene.json / model.xml
│           ├── runtime-provenance.json
│           └── dynamics/summary.json
├── <任务>/<组>/episodes/<episode-id>/
│   ├── metadata.json
│   └── lerobot/               # Parquet、视频、元数据及compact sidecars
├── <任务>/<组>/rejected/      # 未通过/中断记录，保留诊断证据
├── dynamics-worker-<id>/      # ready/request/result/retired等通信与退休记录
└── runtime-identity-<id>.json # UE实际进程、模块路径和版本报告
```

外层 `status=complete` 表示计划完成及审核阶段结束，可以同时存在 `skipped`；不能据此宣称每个任务成功。`pending_audit` 是通过快速检查但尚未通过完整审核；`collected_successes=accepted+pending_audit`，不可直接当作训练合格数。

默认延后审核：逐条快速检查成功终态、连续时间戳/帧号、状态数量、视频声明帧数/尺寸/FPS；全批采集后关闭会话并做完整审核，包括视频全解码、Parquet/状态动作比对、200Hz腕力时间网格/数值、capture manifest、布局与机位。完整审核失败的条目移入 `rejected`。快速检查报告会明确写 `videos_fully_decoded=false/full_validation_pending=true`。[S1][S2]

版本证据分层：

1. `launch-provenance.json` 在顺序入口真正执行前冻结运行源码内容（含相关未跟踪文件）、native manifest/库SHA、解释器与关键包内容、相关环境覆盖、UE build_id及批次输入。Git commit/dirty只是附加信息，不替代内容身份。纯文档修改不改变这一运行源码指纹。[S8]
2. 每条录制重新计算 `runtime_identity`，与顺序入口传入的期望SHA核对；UE build manifest验证源码/Shader/配置/编译产物对应关系。实际渲染进程报告再核对PID、引擎版本、模块路径及SHA；Linux进一步核对 `/proc/<pid>/maps`。读取磁盘上新`.so`不等于旧进程已经加载它。[S3][S10][S17]
3. 动力学 `summary.json.runtime_provenance.loaded_native` 检查本进程已经映射的helper库，包含实际路径、SHA和manifest核验结果；同时记录图复用、过滤模式、关键源码SHA。[S8][S14]
4. `inputs.json` 与 snapshot 只是固定引用与哈希，**不会自动把 recording JSON、所有布局模型、资产与环境复制进批次**。迁移/长期保存仍需保留依赖内容，并校验可用性。单任务/网页/并行入口具备逐条运行身份检查，但没有自动得到顺序入口同等的顶层 `launch-provenance` 恢复契约。[S1][S8]

## 7. 停止与恢复语义

- 顺序/单任务采集识别对应批次的 `cancel.request`；录制子进程检测后关闭采集、保存已有结果和清理所属进程。磁盘/环境故障同样落盘进度。不要用无差别进程名终止其他交互场景。
- 顺序 `--resume` 要求原配置、计划、场景、录制文件内容以及运行版本身份仍匹配。缺少合格launch身份的旧运行批次不能事后补认证；代码/环境/原生库变更后应新建批次并保留旧批次。
- 恢复不会自动删除 `cancel.request`。操作者决定恢复后，需要明确撤销本批次停止标记并保留停止记录；本文不对已停生产批次执行该动作。
- 原 `running/infrastructure_failed/cancelled/interrupted` job 在恢复时归为 `interrupted`，隔离已有录制，标记 `previous_attempt_interrupted_no_retry`；已accepted/skipped/pending_audit不重跑。只继续此前没有尝试的job。
- **`--resume` 不是audit-only**。只要存在未尝试任务，它会继续启动采集；仅在剩余工作全是待审核时，才会直接走到审核阶段。审核历史数据应调用现有离线validator，并把新报告另存；不要为审核已停长批次直接执行resume。
- 网页停止通过 `POST /api/collections/{id}/stop` 进入stopping并创建取消标记；后端重启把未完成job标为interrupted并清理原worker，要求新建任务继续，没有顺序入口式resume。并行/当前发布工具也没有顺序计划的resume选项。[S1][S2][S4][S5][S6]

## 8. 性能字段的单位与比较方法

所有 `*_wall_seconds` 是墙钟秒；`recorded_sim_seconds` 是仿真秒。倍率定义采用“仿真秒 / 墙钟秒”，越大越快；0.8倍意味着1秒仿真约需1.25秒录制墙钟，不代表每小时固定产出多少条。

| 指标 | 精确定义 / 范围 | 读取位置 |
|---|---|---|
| `recorded_sim_seconds` | `(dataset_frame_count-1)/50`，双相机配对样本数，不把两相机帧数相加 | recording summary |
| `recording_wall_seconds` | 正式录制开始到终态已配对进入writer；排除启动、预热和随后封装 | recording summary；[S3] |
| `recording_realtime_factor` | 上述仿真秒 / 正式录制墙钟 | recording summary |
| `capture_and_finalize_wall_seconds` / factor | 从录制开始到 `lifecycle.stop` 完成，包含flush/封装 | recording summary |
| `startup_to_recording_end_wall_seconds` | 从录制Python进程的模块起始计时到终态；含启动和预热 | recording summary |
| `startup_to_finalized_wall_seconds` | 同一起点到视频封装结束 | recording summary |
| `end_to_end_wall_seconds` | 必须注明层级：attempt的值含场景准备、启动、录制、封装和逐条检查；默认deferred时不含批末完整审核 | attempt summary；录制进程另有同名字段 |
| `initialization_seconds` | 导入、原生初态预检、图兼容检查、物理构建、reset、初态应用等分项 | dynamics summary；[S14][S16] |
| `recording_work_wall_seconds` | 正式录制循环内互斥分项：transport、背压等待、状态采样、任务控制、simulation、other；排除初始化/预录/封装 | dynamics summary；不等于整条RTF分母 |
| `video_encoding_metrics` | 每相机编码、统计、queue等待和worker墙钟；多个相机并行时间重叠，不能相加当总耗时 | `lerobot/meta/platform.json`；[S13] |
| `collection_wall_seconds` | 顺序批次到开始批末审核前的运行墙钟累计 | progress |
| `deferred_validation_wall_seconds` | 本轮批末完整审核墙钟；attempt也记录自身审核耗时 | progress / attempt summary |
| `wall_seconds` | 顺序批次执行、清理及审核总墙钟；resume累加先前已记录运行时间，不含停机等待时长 | progress；[S1] |
| 合格条/小时 | `3600 * accepted / wall_seconds`；统计窗口必须说明是否含审核、失败、预热和并发 | 优先用完整批次/固定窗口，pending_audit不得计合格 |

UE启动与动力学初始化存在重叠，不能直接把两者各自墙钟加起来。短段实验、完整任务、连续两条复用、不同布局、批末全解码是不同测量范围；默认配置修改本身不是速度提升证据。

## 9. 当前代码的运行证据示例

本机已有独立验收目录 `/home/spacesim-zmh/artifacts/verification/ue-engineering-20261031/five-task-capture-main/`；它是**固定seed的五任务验收**，不是本文启动的新生产批次，也不是默认1000组计划的完成证明。引用它只为说明参数在哪些真实字段落地；跨机查阅应使用随验收交付的同一份报告。

- `progress.json` 记录5条accepted、pending_audit=0，完整运行779.90秒，其中批末审核72.31秒。不能把这一布局的5条成功推广为整个布局池或长期吞吐保证。
- `recording_settings` 实际是双RGB/50Hz、compact、h264/veryfast、2编码线程、队列30、GOP2帧；四个普通任务max_frames=18000，锁杆记录了自动扩容到30001。
- 五条使用同一动力学PID；抽屉首次构建，三个插头 `physics_graph_reused=true` 且同一graph_id；锁杆 `inputs_changed` 重建新graph，但worker没有退休。这直接说明“同进程”与“同物理图”需要分开判断。
- 三个插头 `restore_existing_graph` 约0.008秒，而首条和锁杆包含物理构建/初始化；后续仍有初态预检、录制和封装耗时，不能声称第二条无需任何初始化。
- 当前五任务recording RTF约0.781/0.664/0.662/0.655/0.676，各自仿真任务时长不同。历史0.8短测不能替代这些完整任务字段，也不代表整批包括审核达到0.8倍。

## 10. 实际代码出处

| 编号 | 文件与关键检查点 |
|---|---|
| S1 | [sequence.py](../../server/collection/sequence.py)：`SequenceConfig/load_config/make_plan/source_scene/run_attempt/execute_plan/main`；[collection.json](../../server/config/collection.json) |
| S2 | [collect_scripted_tasks.py](../../server/collection/collect_scripted_tasks.py)：CLI默认、scene_key、进程复用、审核调度、CollectionSession；校验实现见[recording_validation.py](../../server/collection/recording_validation.py) |
| S3 | [record_scripted_task.py](../../server/collection/record_scripted_task.py)：`recording_settings/task_recording_settings/scene_copy/run`、终态屏障与计时 |
| S4 | [collect_parallel.py](../../server/collection/collect_parallel.py)：配置模型、worker_plan、run_worker、ResourceSampler；[collection-parallel.json](../../server/config/collection-parallel.json) |
| S5 | [collection_jobs.py](../../server/backend/space_arm_platform/collection_jobs.py)：CollectionCreate、start、_run、stop；[collection_api.py](../../server/backend/space_arm_platform/collection_api.py) |
| S6 | [collection_cli.py](../../tools/release/collector/app/collection_cli.py)：发布工具main/run/collection_command |
| S7 | [app.py](../../server/backend/space_arm_platform/app.py)：启动时recording_config/capture_defaults及recording_request |
| S8 | [runtime_provenance.py](../../server/collection/runtime_provenance.py)：source/native/environment/runtime identity、freeze_or_verify、loaded_native_provenance |
| S9 | [task_initialization.py](../../server/simulation/scripted_tasks/task_initialization.py)、[randomization.py](../../server/simulation/scripted_tasks/randomization.py)、[overview_camera.py](../../server/backend/space_arm_platform/overview_camera.py) |
| S10 | [collection_runtime.py](../../server/backend/space_arm_platform/collection_runtime.py)、[unreal_runtime.py](../../server/backend/space_arm_platform/unreal_runtime.py) |
| S11 | [sampling.py](../../server/backend/space_arm_platform/sampling.py)、[camera_defaults.py](../../server/backend/space_arm_platform/camera_defaults.py) |
| S12 | [recording_config.py](../../server/backend/space_arm_platform/recording_config.py)、[recording.json](../../server/config/recording.json)、[recording-veryfast.json](../../server/config/recording-veryfast.json) |
| S13 | [lerobot_capture.py](../../server/backend/space_arm_platform/lerobot_capture.py)、[streaming_video.py](../../server/backend/space_arm_platform/streaming_video.py) |
| S14 | [run_scripted_task.py](../../server/collection/run_scripted_task.py)：initialization、生产初态预检、录制循环、实际native加载记录 |
| S15 | [dynamics_worker入口](../../server/collection/dynamics_worker.py)、[dynamics_worker管理器](../../server/backend/space_arm_platform/dynamics_worker.py) |
| S16 | [physics_graph.py](../../server/simulation/scripted_tasks/physics_graph.py)：compatible/remember/restore |
| S17 | [build_identity.py](../../ue-adapter/Adapters/bsk_render_adapter/build_identity.py)：构建manifest与运行报告核验；[motion_options.py](../../server/simulation/scripted_tasks/motion_options.py)提供低层速度参数 |

维护本表时先查实际调用方是否传参，再查参数模型默认，最后对照落盘结果；不要只更新示例JSON或沿用历史实验结论。

## 四项独立操作的选择

`drawer_open`、`drawer_close`、`locking_rod_fold`、`locking_rod_restore` 已接入网页/API、单条、批量、并行及按布局采集；[配置](../../server/config/collection.json)中默认关闭，原五项默认启用任务不变。按布局启用后会增加每轮尝试数量，不能继续使用五任务的总量推算。初态在录制前按方向建立，反向操作不需要前置录像；用法见[单向操作采集](../../server/docs/single-direction-collection.md)，版本与验收见[验收参考](task-and-data-acceptance.md#36-四项单向任务)。
