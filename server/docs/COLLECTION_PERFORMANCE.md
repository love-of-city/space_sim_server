# 单机脚本采集性能配置

`config/recording.json` 控制新启动的采集脚本；手动平台后端需重启才会重读默认配置。原始数据不会自动转换。速度优化仍保留双相机 1024×1024 RGB / 50 Hz、400 Hz 动力学、200 Hz 脚本反馈和腕力，沿用场景的 BSK 轨道物理配置。

## 当前集成版本（2026-10-31）

`optimized-collection/20261031` 将下文 10 月 29–30 日独立实验的物理图复用、保守碰撞过滤、
私有规划几何前向、原生手指接触计数、状态读取与外力有序汇总接入正式 worker。
下文“未部署”描述的是实验发生时的状态。未采纳没有已证明收益的 SDF 精确输入缓存。

本次独立实测：兼容的第二个双相机任务从启动到录制约 **4.71 秒**，动力学准备 **1.62 秒**，
其中重建与 `InitializeSimulation()` 为零。首次、输入变化或 worker 退休后仍需完整初始化。
两个完整无画面任务与更新前的 44,518 个状态样本哈希完全相同；导向插头完整验证周期
124.206 → 100.603 秒，抽屉首次冷启动 63.201 → 64.514 秒。该带状态哈希采样的结果
不是双相机稳态吞吐或整批提速比例，不能外推到所有布局。

独立固定布局五任务双相机验收全部通过：5 accepted，38,732 帧图像完整解码，数值来源与相机配对校验通过。
本机本次报告：`reports/optimized-version-update-20261031/REPORT.md`；原生库源指纹为
`3dd7ef2af821396d`。固定场景验收与全布局/训练数据认证分别记录。

## 短时录制测速（2026-10-28）

开发优化时使用 `tools/benchmarks/benchmark_scripted_capture.py`，默认只录制同一场景的前 8 秒，
不运行整个任务轮次，也不运行布局/任务预检或完整训练集校验。固定输入可复现关节、颜色和机位。

```bash
python tools/assets/build_native_acceleration.py --component local_mujoco_stepper
python tools/assets/build_native_acceleration.py --component posture
python tools/assets/build_sarm_plugin.py
python tools/benchmarks/benchmark_scripted_capture.py \
  --scene /path/to/scene.json --model /path/to/layout.xml \
  --catalog /path/to/layout.catalog.json --output /path/to/new-probe --seconds 8
```

增加 `--physics-only` 可只测计算循环；该模式不需要 catalog 或 UE。输出包含完整命令、源码和
原生库构建指纹、日志、`timing.json`，录制模式还保存双相机视频。刻意达到时间上限会产生
`total_simulation_timeout`，属于测速诊断样本，不能当作任务成功或训练数据。

当前原生保持力实现将插头接合状态、引导力、姿态转换及 COM 反力运算移到
`local_mujoco_stepper` helper。三个插头共用每个物理 tick 的场景消息；原有 Python 公式保留为
独立对照。保持力仍通过原来的 Basilisk 消息施加，轴向干摩擦、碰撞模型、采样频率不变。
更新后须重建 helper 并重新启动采集进程；旧常驻 worker 不会自动加载新代码。

上一轮固定圆柱场景前 8 秒的短测：双相机录制 14.49→12.88 秒（RTF 0.552→0.621，耗时下降11.1%）。
两路均为401帧，时间戳一致；最大保存关节位置差约5.96e-8 rad，腕力分量差约3.58e-7 N。
这是接近阶段短测，不能外推为全部任务、接触阶段或完整采集周期的同等收益。
碰撞检测仍是主要剩余开销：原始8秒计算诊断中约4.62秒，使用原始完整碰撞几何。
实测细节见本机 `migration/recording-hotpath-20261028-201240/REPORT.md`。

2026-10-28 后续短测默认代码为 **11.947秒／8秒仿真（0.670倍，未达1倍）**。
本轮增加等价的原生批量姿态IK、SDF中心差分相位复用、保持力消息payload复用和静态外力分组；
未修改物理、控制/腕力、相机频率或碰撞几何。126项相关测试通过，两路各401帧完整解码；
最大保存关节位置差5.96e-8rad、200Hz腕力分量差1.05e-6N。

`dynamics/summary.json` 新增 `recording_work_wall_seconds`，只累计正式采集有效循环，
区间互不重叠：仿真执行10.518s、任务控制0.754s、状态读取0.374s、传输0.173s、
其他循环0.032s、背压等待0s；正式录制边界的剩余差值约0.096s。
`simulation_s` 包含BSK模块、物理、消息、渲染发布和观测回调，**不是纯MuJoCo耗时**。
另一次带组件计时的8秒诊断为12.469s，其中MuJoCo调用5.924s、动力学包装/消息1.813s、
保持力0.552s、渲染发布/观测1.229s、其余BSK模块/调度1.502s；不能把这次数字直接拼入
11.947s的普通测量。详细证据见本机 `migration/realtime-recording-20261028-210009/REPORT.md`。

新结果仍仅覆盖圆柱接近阶段。短测冷启动整条57.702s，初始化/UE预热42.942s；不能把
0.670倍当作完整批次效率。没有通过降低频率、丢帧或跳过物理来达到1倍。

## 当前单 worker 顺序采集入口

### 2026-10-28 初始化缓存与连续短录制

`tools/benchmarks/benchmark_reused_capture.py` 用同一个 UE 和动力学 worker 连续录制两条
8秒诊断片段，参数与上述短测一致，额外需要 `--unreal /path/to/UE_5.6`。为测试正常清理后的
进程复用，**仅此诊断 worker** 将预设时长导致的 `total_simulation_timeout` 作为片段正常结束；
原任务摘要仍标为失败，输出在 `diagnostic-episodes`，生产 worker 的失败退休规则不变。

新实现保留每条独立的 BSK 图、mjModel/mjData、控制器和随机流，只复用编译结果：

- 完整物理模型按 MJCF、引用资产内容、MuJoCo 运行库及 SDF manifest 的 SHA 建键，在进程内
  保存一份未修改伺服参数的 MJB；每条反序列化到独立模型并创建全新状态。只缓存最近一套，
  最大1 GiB；本场景实际489 MiB。布局、颜色或物理参数变化会重新编译，不把150套都驻留内存。
- MuJoCo 资源缓存的默认容量500 MiB小于本场景约713 MiB工作集，初始化自身就会发生淘汰。
  采集进程将容量提升至1 GiB（按需占用），保留原生资源失效机制。
- 仅在发布副本中，用占位几何替换透明、无质量的已知螺纹SDF，保留几何编号和位姿。
  完整动力学原模型不变；单独这项精简没有观察到明显提速，不把它列为主要收益。

实测如下；“修正前”是本轮缓存容量不足、第二条未命中的诊断运行，并非旧历史批次。

| 第二条（8秒仿真） | 修正缓存前 | 修正后 |
|---|---:|---:|
| 构建模型 | 15.45s | 7.30s |
| InitializeSimulation | 16.25s | 8.78s |
| 开始录制前全部准备（含上述两项、writer、同步和预热） | 42.48s | 26.69s |
| 正式录制 | 11.97s | 11.87s |
| 上一条完成到本条完成 | 56.77s | 40.90s |

修正后的首条准备37.40s，第二条完整物理模型读取1.49s（另有内容指纹0.52s），命中缓存。
两条UE/Python PID相同、无遗留会话线程，worker RSS约4.05→4.27 GiB。第二条准备缩短37.2%，
完整短测间隔缩短28.0%。该间隔含记录器子进程启动、初始化、预热、录制与封装；不包含本测试
未执行的布局生成、任务预检与批采验收，不能直接当作五任务批次耗时。

112项相关测试通过；两条各有双相机401帧/1024²/50Hz完整解码，401条状态和1601条腕力时间戳
与前次默认版本一致，所比较的关节状态、速度、动作和腕力保存值逐项相同。4秒时刻画面确认
布局、颜色、补光和相机方向保持一致。只验证接近阶段，不能替代完整任务接触验收。
当前录制约0.674倍，尚未达到1倍；本轮主要改善录制前等待。

剩余准备成本包括发布场景构造/编译、InitializeSimulation约8.78s、相机固定预热约5s和
writer准备约2.92s。后续需优先复用发布场景的编译数据，并对相机采用可验证的就绪条件；
不能直接跳过必要的状态复位或盲目缩短曝光预热。详见本机
`migration/initialization-reuse-20261028/REPORT.md`、`final-reused/timing.json` 和 `data-audit.json`。

2026-10-27 的初始化和录制优化验收使用 `collection/collect_scripted_tasks.py`，一个 worker 连续采集两条，显式开启 `--reuse-renderer --reuse-dynamics`。专用配置 [recording-veryfast.json](../config/recording-veryfast.json) 与该次实际录制设置一致：H.264 流式编码、`veryfast`、CRF 30、GOP 2。该配置仅在命令中显式传入时生效；未更改默认 `recording.json` 或默认两路调度配置。

当前性能证据以本机索引 `/home/spacesim-zmh/migration/performance-current.json` 为入口，读取其中 `status`、`current_run` 和 `timing_contract`，再打开对应运行目录中的实际命令和摘要。索引未完成时不引用其数字作为已验收结果。`drawer20-aligned-20261026/aligned-a` 属于初始化优化前历史批次，且与另一批次同时运行，不能代表当前单 worker 速度。

以下命令复用本机已验收的固定布局；源场景、模型和 catalog 保持显式绑定，输出目录应改为每次都不存在的新目录：

```bash
cd /home/spacesim-zmh/src/space-sim/server
env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
  /home/spacesim-zmh/envs/spacesim/bin/python -u collection/collect_scripted_tasks.py \
  --task drawer_cycle \
  --scene /home/spacesim-zmh/migration/collection-baseline-20261027/scene.json \
  --model /home/spacesim-zmh/migration/layout-clearance-color-20261026/layout-000-seed-2026102600/model/layout.xml \
  --catalog /home/spacesim-zmh/migration/layout-clearance-color-20261026/layout-000-seed-2026102600/ue-assets/layout.catalog.json \
  --recording-config config/recording-veryfast.json \
  --unreal /home/spacesim-zmh/engines/UE_5.6 \
  --output /home/spacesim-zmh/data/collections/drawer-sequential-next-001/batch \
  --episodes-root /home/spacesim-zmh/data/collections/drawer-sequential-next-001/episodes \
  --reuse-renderer --reuse-dynamics \
  --dynamics-max-episodes 4 --dynamics-max-rss-mb 8192 \
  --simulation-startup-timeout-seconds 600 \
  --offline --gpu-index 0 --count 2 --max-attempts 8 \
  --range-deg 30 --seed 2026102700 \
  --randomize-overview-camera --overview-camera-sampling presets \
  --warmup-seconds 5 --reuse-warmup-seconds 5 \
  --control-port 27500 --capture-port 27501 --render-port 27502 --signalling-port 27503
```

`collect_scripted_tasks.py` 本身顺序处理尝试，不读取 `config/collection-parallel.json` 的 worker 数。固定布局只用于此次性能验收；每条仍独立随机初始化关节、颜色和已启用的机位/场景参数。开始新运行前重新启动整个采集命令，以加载当前 Python 和原生库版本，不能在旧常驻 worker 上热替换优化代码。

需要随机安装布局时，先按[布局池说明](COLLECTION_LAYOUT_RANDOMIZATION.md)使用当前源码重新生成或重新校验布局池，然后用 `--layout-pool /path/to/new/pool.json` 替换 `--model` 与 `--catalog`。两种输入互斥。旧池源码/模型/资产哈希失效应明确停止；不能删除哈希约束、静默使用旧池或退回固定布局。上述固定布局验收也不代表旧布局池整体有效。

复用 UE 保留渲染进程；复用 dynamics 保留 Python 导入和原生库，每条仍创建新的 BSK 图、MuJoCo 状态、任务、随机流和通信会话。worker 失败、遗留线程、达到条数或 RSS 上限时仅在条目结束后回收。核对两条 `recording/summary.json` 的 `dynamics_worker.pid`、`result.completed_episodes` 和 `extra_threads`，才能确认实际复用和清理情况。首条 `renderer_launched.reused=true` 仅表示 renderer 由批次管理，不代表首条免除了本批冷启动。

本地动力学初始化会生成专供状态发布的精简 MJScene；完整物理碰撞模型继续由 stepper 加载。新 `recording/dynamics/summary.json` 保存 `initialization_seconds`、`publication_scene` 和 `runtime_provenance`。Linux 录制通过 `-DDC=InstalledNoZenLocalFallback` 跳过 Zen 自动启动重试；实际可写缓存路径以本次 `renderer.log` 为准。

## 编码路径

### 2026-10-27 原生摩擦计算优化

符合条件的 MuJoCo 3.7 `implicitfast`/Newton 模型在耦合静摩擦候选计算中保存并恢复完整 `mjSTATE_INTEGRATION`，复用已计算的 step1 缓存及满足原容差的最终 step2，避免每个候选的整份 `mjData` 复制及最终重复求解。失败恢复原状态及 warning 后重试。不满足条件的有状态 actuator、回调、插件、sleep/FWDINV 等模型继续走原完整复制路径；不改变物理参数、碰撞、采样频率或成功阈值。

`dynamics.friction_solver` 保存候选、成功步骤复用、整状态复制和 integration 恢复次数。原生 C++ 改动后须重建、重启 worker，并核对运行摘要和 `run/native_acceleration/local_mujoco_stepper.json` 的指纹，不能沿用旧 worker 测速。

本机 `migration/recording-speed-20261027/REPORT.md` 记录同场景和种子的两条验收：第 2 条录制 62.135 → 47.620 秒，完整周期 120.363 → 108.068 秒。相同仿真时刻的状态、动作、腕力全同，完整视频验证通过；共享主机耗时仅为观测结果。第 1 条已知非任务面板侵入仍存在，性能通过不等于训练画面全部合格。

`video.streaming_encoding=true` 时，JPEG 解码后的 RGB 直接进入每相机有界编码队列，录制同时生成 H.264 MP4，不再先写 LeRobot 临时 PNG 再读回编码。`save_raw_camera_images` 仍独立控制外层 `cameras/` 审计副本。

平台的 `StrictStreamingVideoEncoder` 保持官方 LeRobot v3 数据表、视频索引和统计格式：

- 队列满时等待，不采用上游默认的 100 ms 后丢帧策略；编码器退出或持续堵塞 60 秒则明确失败。
- MP4 使用逐帧 PTS 和 `1/fps` 时基。收尾核对视频包数与提交的图像数，批量验收继续完整解码视频。
- 统计在下采样后的 float64 像素上累计，避免 uint8 平方溢出导致错误的零标准差；统计的 count 按图像数记录。图像像素和数值训练特征不因此改变。
- 队列容量由 `encoder_queue_maxsize` 控制，默认每相机 30 帧；1024² RGB 两路队列最多约 180 MiB 像素缓冲，不靠无限内存堆积实现测速。

`codec=h264` 使用 CPU 编码，`codec=h264_nvenc` 使用 NVIDIA 硬件编码。后者要求本机 FFmpeg/PyAV 和驱动支持 NVENC，失败会中止录制，不静默切换编码器。H800 不能仅根据 CUDA 可用就推断支持此编码路径。

`quality` 对软件 H.264 表示 CRF，对 NVENC 表示固定 QP；两者相同数字不代表相同画质。默认 30。`keyframe_interval` 默认 2，保留训练随机取帧时的短解码距离。两项自定义值仅在流式模式支持；非流式沿用安装的 LeRobot 默认编码参数（软件 H.264 为 CRF 30 / GOP 2），并拒绝不生效的自定义值。

`video.encoder_preset` 只在流式编码中生效。专用 `recording-veryfast.json` 以更快的编码预设降低 CPU 编码开销；相同 CRF 不表示逐像素结果或文件大小与其他预设一致。保留完整帧配对与视频解码验收，不通过丢帧提高速度。

## 脚本与传输

`scripted_native_geometry=true` 启用仓库已有的 C++ FK/Jacobian helper，保持相同几何公式、关节参考语义和物理参数；需要已构建并通过 ABI 检查的原生库。`false` 使用 Python 实现，未安装原生 helper 的环境可选此项。独立诊断脚本也支持 `--native-geometry`。

`capture_backpressure_poll_s` 默认 0.002 秒。仅决定数据链路满时多久重新检查容量，不改变仿真步长、相机帧率或样本时间戳。渲染状态 TCP 发布器设置 TCP_NODELAY，避免小包等待。所有配对、连续帧检查和 ACK 保留。

单条录制同时启动 UE 和动力学初始化；两个端点就绪且预热完成后才开启采集。单次 `record_scripted_task.py` 默认使用独立进程；上述批量入口按显式参数复用 UE 和动力学 worker，每条仍创建独立物理状态。

## 测速与复现

`record_scripted_task.py`、`collect_scripted_tasks.py` 和 `benchmark_parallel_capture.py` 均支持 `--recording-config <path>`，用于独立对照实验。未传入时读取统一配置。每条录制的 `summary.json` 保存实际配置和以下时间：

- `recording_wall_seconds`：开启正式录制后，直到终态的状态和双相机样本配对完成并进入 writer；不会在只收到终态状态时提前停止。
- `finalization_wall_seconds`：排空、编码封口及数据集保存。
- `capture_and_finalize_wall_seconds`：上述两段之和，应结合该值判断持续采集效率。
- `startup_wall_seconds` / `total_wall_seconds`：准备耗时 / 包括准备和进程关闭的整条脚本耗时。
- `recorded_sim_seconds`：已保存的首末样本间仿真时长；相应 realtime_factor 为此时长除以墙钟时间。

`collect_scripted_tasks.py` 的批次总耗时还包括随机姿态预检查和最终完整数据验收。短片段会因人为设置的仿真上限以 task failure 结束，不能当作成功演示；测速必须区分“数据完整”和“任务成功”。

当前完整单条周期取批次 `summary.json` 中已验收 attempt 的 `end_to_end_wall_seconds`：从该次尝试开始，包含场景准备、预检、初始化、录制、封装和验收，已经包含 `validation_wall_seconds`，不可重复相加。重点看第二条时，另计算两条 `finished_at_ns` 的差除以 `1e9`，表示连续两次数据完成之间的间隔。批次 `wall_seconds` 还包含最终进程退出；拒绝初态和失败尝试应留在统计中。

动力学 `initialization_wall_seconds` 从原生模块加载前开始，而其 `wall_seconds` 从模型/任务导入完成后开始，二者起点不同；`task_loop_wall_seconds` 还可能包含等待录制结束的时间。不能简单将这些字段相加或将正式录制时间当作完整采集周期。`scene_preparation_wall_seconds`、`preflight_wall_seconds`、初始化分段和正式录制时间用于解释完整周期中的开销。

`--profile-dynamics` 在单条录制中生成 `dynamics.pstats`，仅作诊断；分析器和并行测试会干扰倍率。最终性能结论应来自没有分析器和其他测速任务竞争的完整任务。在线遥操作实例仍可能共享本机资源。

## 2026-10-28 最新合并后的完整任务观察

`8f93cd9` 加本地原生计算/编译缓存优化的实录位于本机 `migration/linux-merge-five-tasks-20261028-224314/REPORT.md`。random150第129套五次尝试，三个插头成功、抽屉和锁杆预检拒绝，整轮1213.83秒。航空/导向/圆柱分别为60.98/55.92/106.42仿真秒，211.26/147.20/511.26录制真实秒，300.86/221.21/597.42整条真实秒；L002抽屉补测29.82仿真秒、97.40录制秒、242.75整条秒。场景不同，不能与此前8秒接近片段直接计算优化比例。

三个插头实际复用同一UE与动力学进程，后两条物理模型缓存命中，加载约1.53/1.50秒；已有初始化优化保留。新路径规划在批采及动力学启动各执行一次预检；内层预检单独增加约19–22秒，写入 `initialization_seconds.native_preflight`。录制中航空/导向控制及规划约55/53秒；圆柱物理推进423.99秒，占录制约83%，松爪23.07仿真秒、静止验证30.23仿真秒。主批次三个插头 `backpressure_wait_s=0`，抽屉补测约0.27秒，不能把主要耗时归因于编码队列等待。

下一步应分别处理同指纹预检结果复用、每回合独立规划模型复用，以及圆柱接触生成/SDF/约束求解的分段分析。未改变400 Hz物理、200 Hz任务/腕力、双1024² RGB 50 Hz，也未放宽2 mm间距和静止判据。最新合并仍未达到1倍实时，锁杆尚无本轮完整录像验收。

## 2026-10-30 保守碰撞筛选实验（未接入正式采集）

在本机初始化复用实验版上增加凸包分离投影、分离方向/极值顶点复用，以及限定插件版本的公母螺纹净空证明。只有当前几何能证明不接触时才跳过原精细查询，临界状态保留原求解；400 Hz物理、摩擦和碰撞形状未修改。源码、构建、数学条件及原始数据位于本机 `/home/spacesim-zmh/migration/collision-filter-20261030/REPORT.md`，最终实现为该目录 `candidate_sources/native/local_mujoco_support.cpp`，并非生产原生库。

同一layout121抽屉/导向完整无渲染任务循环分别38.46→36.67秒、112.38→108.77秒，减少4.66%/3.21%；44,516个物理步的状态、加速度、执行器/接触/腕力哈希一致。完整shadow对照约370.5万次拟排除网格查询均无原回调接触，凸包支持查询与全顶点扫描另行核对。第二条物理构建和InitializeSimulation仍为零，属于既有复用收益。仅一组完整对照且后台正式采集仍运行，不能保证稳定提升，也不能把上述无渲染倍率写成录制倍率；未新增双相机实采或五任务长批验收，当前正式批次未切换。

## 2026-10-30 顺序优化：私有规划查询与状态读取（未接入正式采集）

最新独立实验与唯一运行入口见本机 `/home/spacesim-zmh/migration/staged-optimization-20261030/REPORT.md` 和 `README.md`，全局实验索引为 `migration/performance-current.json` 的 `latest_isolated_optimization`，不能与该索引旧 `current_run` 或正式批次混用。

本轮在初始化复用及上一轮保守碰撞候选上继续实现：原生接触计数、渲染帧内位置读取复用、私有规划中排除原判据不涉及的物体对精细查询、夹爪重复角度结果缓存、螺纹元数据生命周期管理，以及摩擦临时数组容量复用。真实动力学仍保留完整碰撞、摩擦参数与400Hz推进，规划仍保留原间距及孔位判据。

最终抽屉/导向无渲染任务循环35.47→35.26秒、105.87→99.68秒，导向规划13.45→7.28秒；44,516个物理步的状态和受力与基线完全一致，72项相关回归通过。两条8秒双相机诊断各401帧/相机及1,601条腕力通过时序和解码检查；它们主动达到任务时间上限，不是成功训练数据。导向无渲染倍率0.792×，并非录制倍率；物理推进仍85.41秒，未证实摩擦数组改动提高推进速度。单组测速、后台正式采集运行，不能保证稳定收益。最终证据在 `candidate-planning/`，中间 `candidate/` 不作为最终速度。当前正式服务源码/原生库未替换，未提交或推送。


> 维护说明（2026-10-30）：本文引用的旧 migration 实验输出已按用户要求清理；以上历史数据不是当前版本重新验收结果。见[历史清理说明](../../docs/history.md)。


## 2026-10-30 物理推进：投影提前返回与批量外力（独立实验）

在上一轮初始化复用、私有规划与状态读取优化版上，进一步让凸包分离证明只计算当前所需的投影端点，证明成立立即返回，否则继续原完整检测；并按原源顺序在一次原生调用中完成每步外力分组累加/写入。400Hz物理、摩擦、质量惯量、完整几何、容差和任务动作不变。

早期测量期间正式批采被取消且仓库迁移，负载发生变化，因此最终另做一组空闲环境下的完整顺序对照。抽屉推进 32.408→31.770s（减少1.97%），导向推进 84.294→83.140s（减少1.37%）。导向完整无渲染循环 98.014→96.014s，倍率 0.806→0.822×。推进含控制/消息和本次逐步验证，不是纯MuJoCo耗时；无相机录制，不能据此宣称录制达1×。单组测量，不保证所有布局取得同等收益。

两个任务成功，44,516个物理步的状态、加速度、执行器力、接触力和腕力哈希完全一致，阶段及规划也一致；20万组投影边界和2万组外力检查通过。SDF精确输入缓存没有可靠收益证据，未纳入最终版本。最终源码、独立入口、构建指纹和结果见本机 `/home/spacesim-zmh/migration/physics-advance-20261030/REPORT.md`、`run_advance.py`、`manifest.json`，以 `baseline-clean/` 与 `candidate-clean/` 为最终测速证据。

本轮未部署、未提交或推送。原批采保留 `cancelled` 状态；迁移后的源码路径为 `/home/spacesim-zmh/src/space-sim`，旧路径仅为兼容链接。
