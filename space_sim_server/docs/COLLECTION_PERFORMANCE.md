# 单机脚本采集性能配置

`config/recording.json` 控制新启动的采集脚本；手动平台后端需重启才会重读默认配置。原始数据不会自动转换。速度优化仍保留双相机 1024×1024 RGB / 50 Hz、400 Hz 动力学、200 Hz 脚本反馈和腕力，沿用场景的 BSK 轨道物理配置。

## 编码路径

`video.streaming_encoding=true` 时，JPEG 解码后的 RGB 直接进入每相机有界编码队列，录制同时生成 H.264 MP4，不再先写 LeRobot 临时 PNG 再读回编码。`save_raw_camera_images` 仍独立控制外层 `cameras/` 审计副本。

平台的 `StrictStreamingVideoEncoder` 保持官方 LeRobot v3 数据表、视频索引和统计格式：

- 队列满时等待，不采用上游默认的 100 ms 后丢帧策略；编码器退出或持续堵塞 60 秒则明确失败。
- MP4 使用逐帧 PTS 和 `1/fps` 时基。收尾核对视频包数与提交的图像数，批量验收继续完整解码视频。
- 统计在下采样后的 float64 像素上累计，避免 uint8 平方溢出导致错误的零标准差；统计的 count 按图像数记录。图像像素和数值训练特征不因此改变。
- 队列容量由 `encoder_queue_maxsize` 控制，默认每相机 30 帧；1024² RGB 两路队列最多约 180 MiB 像素缓冲，不靠无限内存堆积实现测速。

`codec=h264` 使用 CPU 编码，`codec=h264_nvenc` 使用 NVIDIA 硬件编码。后者要求本机 FFmpeg/PyAV 和驱动支持 NVENC，失败会中止录制，不静默切换编码器。H800 不能仅根据 CUDA 可用就推断支持此编码路径。

`quality` 对软件 H.264 表示 CRF，对 NVENC 表示固定 QP；两者相同数字不代表相同画质。默认 30。`keyframe_interval` 默认 2，保留训练随机取帧时的短解码距离。两项自定义值仅在流式模式支持；非流式沿用安装的 LeRobot 默认编码参数（软件 H.264 为 CRF 30 / GOP 2），并拒绝不生效的自定义值。

## 脚本与传输

`scripted_native_geometry=true` 启用仓库已有的 C++ FK/Jacobian helper，保持相同几何公式、关节参考语义和物理参数；需要已构建并通过 ABI 检查的原生库。`false` 使用 Python 实现，未安装原生 helper 的环境可选此项。独立诊断脚本也支持 `--native-geometry`。

`capture_backpressure_poll_s` 默认 0.002 秒。仅决定数据链路满时多久重新检查容量，不改变仿真步长、相机帧率或样本时间戳。渲染状态 TCP 发布器设置 TCP_NODELAY，避免小包等待。所有配对、连续帧检查和 ACK 保留。

单条录制同时启动 UE 和动力学初始化；两个端点就绪且预热完成后才开启采集。每条任务仍创建独立场景和进程，没有复用上一条的物理状态。

## 测速与复现

`record_drawer_task.py`、`collect_drawer_cycles.py` 和 `benchmark_parallel_capture.py` 均支持 `--recording-config <path>`，用于独立对照实验。未传入时读取统一配置。每条录制的 `summary.json` 保存实际配置和以下时间：

- `recording_wall_seconds`：开启正式录制后，直到接收到任务终态。
- `finalization_wall_seconds`：排空、编码封口及数据集保存。
- `capture_and_finalize_wall_seconds`：上述两段之和，应结合该值判断持续采集效率。
- `startup_wall_seconds` / `total_wall_seconds`：准备耗时 / 包括准备和进程关闭的整条脚本耗时。
- `recorded_sim_seconds`：已保存的首末样本间仿真时长；相应 realtime_factor 为此时长除以墙钟时间。

`collect_drawer_cycles.py` 的批次总耗时还包括随机姿态预检查和最终完整数据验收。短片段会因人为设置的仿真上限以 task failure 结束，不能当作成功演示；测速必须区分“数据完整”和“任务成功”。

`--profile-dynamics` 在单条录制中生成 `dynamics.pstats`，仅作诊断；分析器和并行测试会干扰倍率。最终性能结论应来自没有分析器和其他测速任务竞争的完整任务。在线遥操作实例仍可能共享本机资源。
