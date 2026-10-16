# 双场景并行采集验证

本实验只新增独立验证工具，生产动力学、UE 插件和数据格式保持原实现。结果写入被 Git 忽略的 run/two-world-benchmark 目录。

## 测量范围

1. **生产动力学**：原 BSK＋MuJoCo 3.7.0，两个完整抽屉循环。分别串行和同时运行两个独立进程。400 Hz / 单子步，脚本与六维力 200 Hz。不启动 UE。
2. **GPU 接触内核**：独立 Python 环境中，同版本原生 MuJoCo 与 MuJoCo Warp 在完全相同的模型、接触状态、保持控制量上对比两套世界。保留全部 SDF、机械结构、约束、force/torque 传感器；只覆盖接触动力学片段，不包含 BSK 环境力更新、姿态控制、脚本规划、图像或训练数据封装。不能据此宣称 GPU 已完成平台迁移。
3. **生产录制链路**：短时任务片段，真实 UE 双相机 1024×1024、50 FPS、七维 state/action、200 Hz 六维力，串行两条与并行两条。限时片段会按现有协议标记任务超时/failure；完整性合格不等于任务成功，数据单独保存在实验目录。

所有测试依次运行，避免两个基准同时争用硬件。包含启动/模型装载的总时间、稳定步进时间以及 GPU 首次编译时间分别记录。生产动力学和 GPU 接触内核不能直接混成同一个速度表。

## 工具

- tools/benchmark_parallel_capture.py：启动两个隔离实例，分配独立端口、输出和录制会话，记录总时间和进程树资源占用。--workers 1 为串行，2 为并行；--mode physics/capture。
- tools/benchmark_mujoco_warp.py：GPU/CPU 同版本接触内核对比。需要独立 mujoco-warp 环境和本机 MSVC。编译实验专用 SDF DLL，不覆盖 Basilisk 或生产 DLL。
- tools/benchmark_warp_sdf.py：五种螺纹 SDF 的实验 Warp 实现；使用原 2 微米差分梯度，运行前对 C++ 原公式进行 10,000 点对照。它使用版本相关的 Warp 扩展接口，不是生产后端。
- tools/audit_parallel_capture.py：验证主流与六维力的真实时间间隔、七维数据、相机帧与状态对应、完整视频解码、跨实例会话隔离。

## 运行方式

从 space_sim_server 目录使用原平台 Python 运行生产测试，例如：

```powershell
python tools/benchmark_parallel_capture.py --mode physics --workers 2 --jobs 2 --seconds 180 --scene run/scenes/scene-20261014-170501-01b5c238.json --output run/two-world-benchmark/cpu-parallel
```

输出目录必须是新目录，工具不会覆盖已有实验。采集模式额外用 --port-base 指定未占用的端口段。

GPU 依赖安装到 run/gpu_parallel_probe/venv：mujoco==3.16.0、mujoco-warp==3.16.0、warp-lang==1.18.0、numpy==2.4.6。不要把 Python mujoco 导入生产 Basilisk 进程。

GPU 接触测试从既有完整任务 trace.jsonl 取一个接触时刻的 qpos，并用相邻样本估算 qvel；对两种后端采用相同输入。它使用 XML 中的执行器与保持参考，不等同于生产伺服全过程。GPU float32 与 CPU float64 的轨迹和力差值单独报告，微基准速度不能替代动力学精度验收。

## 结果

2026-10-16，RTX 4090 24 GiB，Windows，系统暴露 16 个逻辑 CPU。同机器后台应用保持运行，数据是本机小规模测量，不能外推任意并发数。

| 生产动力学测试 | 两条总耗时（含进程启动与装载） | 进程树峰值 RSS | 结果 |
|---|---:|---:|---|
| 串行，两个完整循环 | 458.750 s | 4.122 GiB | 2/2 成功 |
| 并行，两个完整循环 | 241.210 s | 8.243 GiB | 2/2 成功 |

两条总吞吐提升 1.902 倍，时间减少 47.42%。每条模拟 128.335 s。相同初始状态、参数、控制脚本下，四份 200 Hz 诊断轨迹的 SHA256 完全一致：4af8e5b9e322a5c90fb9d693e1331ec940fdef4271554143b2f0331160be4024。这个测试关闭随机扰动以便对照；没有验证任意随机初始姿态都能成功。

原始记录见 run/two-world-benchmark/cpu-serial/benchmark.json 与 cpu-parallel/benchmark.json。GPU 与真实录制结果见后续追加记录。


### 相同模型的 GPU / CPU 接触内核测试

使用 MuJoCo 3.16.0、MuJoCo Warp 3.16.0、Warp 1.18.0。原模型为 68 个 qpos、61 个速度自由度、37 个 body、2,346 个 geom、1,395 个 mesh、3 条 equality、2 个 sensor、5 个 SDF 实例。取完整抽屉任务第 83 秒的提拉接触状态，两个世界使用相同初始状态与保持控制量，各推进 400 步（1 仿真秒）。

| 方案 | 两个世界的步进总墙钟时间 | 范围 |
|---|---:|---|
| 两个原生 MuJoCo CPU 进程 | 0.600 s | 模型与进程已准备好，包含唤醒/收集开销；各进程内部为 0.564 / 0.573 s |
| Warp GPU 批量 nworld=2 | 1.039 s | CUDA Graph，已编译；末尾同步完成后计时结束 |

这一次 CPU 更快，GPU 未显示两世界规模的收益。GPU 主内核首次编译/首步为 389.499 s，未混入上表；模型传输为 7.734 s。GPU 步进后显卡总占用快照为 4,071 MiB，开始时为 3,313 MiB；这不是全程显存峰值。

原 C++ 源码在实验目录重新编译到同版本 ABI；五种 SDF 与 GPU 公式 10,000 点对照，距离最大差 6.06e-9 m，归一化梯度分量最大差 0.00143，满足本实验的明示阈值。GPU 两世界状态与传感器均有限，没有接触/约束缓冲溢出。

这不是生产等价性验收。一步步推进 1 秒后，世界0相对 CPU 的六个主动臂关节最大差 9.84e-5 rad，抽屉滑动差约 10.77 微米，腕力分量最大差 0.1162 N、力矩分量最大差 0.03795 N·m。全 qpos 最大差 0.00501 对应紧固件自由关节的四元数分量，不能称作 5 毫米位置误差。GPU 两个相同输入世界也存在细微数值差异；没有证明逐位确定性、完整任务成功或长时间精度。

没有将 BSK 在轨力、姿态控制、原生伺服/任务状态机和 UE 写入链路迁移到 GPU。两套生产动力学耗时不能直接和此接触片段比较。原始报告：run/two-world-benchmark/gpu-contact-v2/benchmark.json、gpu-accuracy-summary.json。首轮因 Python 包头文件嵌套目录导致编译失败，已修复定位逻辑并保留失败记录于 gpu-contact/。


### 真实 UE 双相机录制链路

两个实例分别采集 20 秒限时片段，沿用生产 CaptureReceiver / SimulationHub / EpisodeRecorder / LeRobot 写入器，动力学 400 Hz 单子步、脚本与六维力 200 Hz、七维 state/action 和两路 1024×1024 图像 50 Hz。每条最终为 1,001 帧主数据、2,002 张图像、4,001 条六维力；约 20 秒片段因停止边界多包含一帧。

| 方式 | 两条从脚本启动至全部关闭的总耗时 | 每条正式采集 | 每条编码/收尾 | 峰值进程树 RSS | 显卡整体显存峰值采样 |
|---|---:|---:|---:|---:|---:|
| 串行 | 495.369 s（8分15秒） | 105.11 / 105.50 s | 62.41 / 61.79 s | 8.97 GiB | 6,507 MiB |
| 并行 | 292.993 s（4分53秒） | 114.72 / 115.14 s | 63.82 / 63.45 s | 20.32 GiB | 9,380 MiB |

相同数据量的总吞吐提升 1.691 倍，总耗时减少 40.85%。单条采集稍慢，但同时完成两条的总时间明显缩短。全机平均 CPU 采样从 37.2% 提高到 74.3%；GPU 利用率采样峰值从 57% 提高到 90%。这些资源数包含仍在运行的其他应用影响，显卡为整体数据，不是逐进程精确显存。

四条录制的独立审计全部通过：主流相邻时间戳 20 ms，力流 5 ms；源 observation 与 Parquet 对齐；图像与状态、会话对应；七维 state/action 保留；八个视频全部完整解码并验证 1024×1024、50 FPS、帧数和 PTS；没有不完整或拒收样本。各实例的录制会话 ID 独立。相机中间帧也已人工目视检查，见 capture-parallel/camera-preview.png。

这是限时吞吐测试，在抽屉 align 阶段停止，元数据保留 outcome=failure / total_simulation_timeout。它不代表完成了一次抽屉循环；完整循环成功证据在前面的四条无相机生产动力学运行。

原始数据：

- run/two-world-benchmark/capture-serial/benchmark.json、validation.json、episodes/
- run/two-world-benchmark/capture-parallel/benchmark.json、validation.json、episodes/
- run/two-world-benchmark/capture-comparison.json

### 本轮结论与边界

现有架构的 CPU 双实例既通过完整任务验证，也通过真实双相机录制验证，是当前两个场景规模下有证据支持的方案。GPU 上已跑通完整模型的双世界接触步进与传感器计算，但本次小批量热运行更慢，且存在数值差异，尚不能替代生产动力学或声称已实现 GPU 自动采集。没有测试更多世界，不推断 8/32/128 世界的性能。

该原型用于可重复的小规模验证，尚未实现正式的批量任务队列、失败重试、随机初始条件调度、断点续采或 GPU 的 BSK/UE 集成。所有新 Python 工具通过 Ruff 与编译检查，真实运行验收文件如上。测试实例已关闭，19760–19773 实验端口段无残留监听。
