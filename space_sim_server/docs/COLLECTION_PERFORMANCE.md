# 单机脚本采集性能配置

`config/recording.json` 控制新启动的采集脚本；手动平台后端需重启才会重读默认配置。原始数据不会自动转换。速度优化仍保留双相机 1024×1024 RGB / 50 Hz、400 Hz 动力学、200 Hz 脚本反馈和腕力，沿用场景的 BSK 轨道物理配置。

## 当前单 worker 顺序采集入口

2026-10-27 的初始化和录制优化验收使用 `collection/collect_scripted_tasks.py`，一个 worker 连续采集两条，显式开启 `--reuse-renderer --reuse-dynamics`。专用配置 [recording-veryfast.json](../config/recording-veryfast.json) 与该次实际录制设置一致：H.264 流式编码、`veryfast`、CRF 30、GOP 2。该配置仅在命令中显式传入时生效；未更改默认 `recording.json` 或默认两路调度配置。

当前性能证据以本机索引 `/home/spacesim-zmh/migration/performance-current.json` 为入口，读取其中 `status`、`current_run` 和 `timing_contract`，再打开对应运行目录中的实际命令和摘要。索引未完成时不引用其数字作为已验收结果。`drawer20-aligned-20261026/aligned-a` 属于初始化优化前历史批次，且与另一批次同时运行，不能代表当前单 worker 速度。

以下命令复用本机已验收的固定布局；源场景、模型和 catalog 保持显式绑定，输出目录应改为每次都不存在的新目录：

```bash
cd /home/spacesim-zmh/src/space_sim_server/space_sim_server
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
