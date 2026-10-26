# 多进程脚本采集

`tools/collect_parallel.py` 在同一台机器启动多个独立的批量采集进程。每路拥有自己的动力学、录制器和 UE，使用独立端口、种子区间、输出目录和停止文件。单卡可运行多路；多卡时在配置中逐路指定 GPU。只有通过原有完整校验的轨迹才计入吞吐量。

## 配置与运行

调度与资源设置统一放在 `config/collection.json`；相机、存储和视频编码仍由其引用的 `config/recording.json` 管理。`recording_config` 相对于采集配置文件所在目录解析，也接受绝对路径。操作台位置与朝向随机化通过 `layout_pool` 配置，网页也可启用，见[布局随机化采集](COLLECTION_LAYOUT_RANDOMIZATION.md)。

```bash
cd ~/src/space_sim_server/space_sim_server
~/envs/spacesim/bin/python tools/collect_parallel.py \
  --scene ~/migration/validation-scene.json \
  --unreal ~/engines/UE_5.6 \
  --output ~/data/collection/run-001
```

输出目录必须不存在，防止覆盖旧任务。默认两路、每路两条成功轨迹，每路最多尝试四次。小规模验收可用 `--workers 1 --episodes-per-worker 1`。`--workers` 只能选择配置中已有的前几路。Windows 使用同一 Python 入口，指定 Windows 引擎、场景和输出路径。

| 配置 | 含义 |
| --- | --- |
| `workers` | 每项代表一个独立采集进程；默认两路共用 GPU 0 |
| `gpu_index` | UE 的 `graphicsadapter` 序号及 NVENC 的 GPU 序号；混合多卡机器先核对两者枚举一致性 |
| `cpu_affinity` | 可选逻辑 CPU 列表，空列表交由系统调度；子进程继承 |
| `math_threads` | 每路 OpenMP、OpenBLAS、MKL、NumExpr 线程上限，不代表 UE 总线程数 |
| `port_base` | 第 i 路使用 `port_base + 10*i` 开始的四个端口 |
| `seed` | 第 i 路从 `seed + i*max_attempts_per_worker` 开始，任务种子不重叠 |
| `offline` | 关闭 WebRTC 相机预览和无用的离屏主视口，保留权威图像采集 |
| `reuse_renderer` | 每路保留 UE；每条仍重建动力学与录制器，隔离物理状态 |
| `warmup_seconds` / `reuse_warmup_seconds` | 首条/复用后预热时间，默认都为 5 秒；缩短后需验证首帧质量 |
| `range_deg` | 初始关节随机偏移范围，0–10 度；性能对比可用 0 固定初始姿态 |
| `randomize_orbit_phase` | 每次尝试沿输入轨道随机选择真近点角 `[0,360)`；仓库配置开启，旧配置缺省关闭 |
| `sample_seconds` | 资源采样间隔 |
| `worker_timeout_seconds` | 整批最长墙钟时间，包括启动和校验 |

GPU 显存不足或计算已满时，增加并发可能降低总吞吐量，应先比较一、两路。配置的 GPU 序号不是 CUDA 可见设备重映射；在不同设备枚举的多卡环境部署前必须单路核对实际 UE/NVENC 设备日志。

在轨位置随机化使用与网页场景创建相同的独立种子流，每次尝试重新采样并保存 `environment.orbit.true_anomaly_deg` 和采样元数据。它改变航天器在既定轨道上的初始位置及对应轨道速度，由动力学初始化实际使用；高度、偏心率、倾角和局部机械臂/任务台布局保持输入场景设定。地球背景、相对太阳方向和遮挡可能随轨道位置改变。椭圆轨道上均匀真近点角不等于均匀时间采样。回放使用已保存的角度，不再次采样。串行批量入口可用 `--randomize-orbit-phase` / `--no-randomize-orbit-phase` 显式控制。

## UE 复用与数据完整性

Linux UE 5.6 离线采集关闭 `SLS_CapturedScene` 天光，使用原有太阳光照，保留体积云、阴影和相机画质。新增捕获式天光在动态轨道场景中会导致渲染耗时逐渐增加，最终触发原有 10 秒相机配对超时；停止实时更新、只捕获一次也会复现。该修正只作用于 Linux 的 `-BskOfflineCapture`，在线预览沿用原配置。因此离线数据不会包含该可选天光提供的环境补光。启动日志会记录生效情况；相机预览也真正停止渲染，而不只是停止回传。

每路首次通过碰撞/路径预检查后启动 UE。后续轨迹重新启动权威动力学，建立新的渲染会话；现有 UE 会话切换机制取消旧采集请求、重置图像时钟并应用新的场景清单。录制器也为每条重建，并等待新动力学首帧再预热和开始采集。不会复用上一条物理积分状态。

仍保留双相机 1024×1024、50 Hz、200 Hz 力传感数据、严格图像/状态配对和反压；编码队列满时等待或报错，不丢帧。`validate_recording` 检查状态/动作、任务成功、相机配对与校验和，并完整解码两段视频。各路独立输出 LeRobot 数据集；没有让多个进程同时写同一个 LeRobot 数据集。

批量子进程、UE 及录制子进程均由对应协调器管理。某路基础设施异常会停止其他路；停止文件允许录制器完成异常收尾，超时后只清理所属进程树。Linux 端口预检查允许已经关闭的连接进入 TIME_WAIT，不绕过活动监听端口冲突。跨协调器使用端口文件锁，避免同时选择同一组端口。

## 编码对比

复制录制配置并把 `video.codec` 改为 `h264_nvenc`，在采集配置中引用副本即可。保持 `streaming_encoding=true`。每路 GPU 序号会传给 NVENC。H.264 CPU 的 `quality` 是 CRF，NVENC 是固定 QP；相同数字不表示相同画质或大小，需要分别测量。CPU 配置保留为默认，未经实测不自动切换编码器。

## 结果与计时

```text
run-001/
  summary.json                 # 整批成功条数、吞吐、每路结果、资源摘要
  resources.jsonl              # 采集进程树 CPU/RSS 与系统 GPU 采样
  worker-00/
    plan.json                  # 实际命令、GPU、线程/CPU 设置
    recording.json             # 本路生效的录制配置
    worker.log
    batch/
      summary.json
      renderer.log             # 常驻 UE 跨轨迹日志与会话号
      attempt-.../
        validation.json
        recording/summary.json # 启动、录制、收尾及子阶段时间
  episodes/worker-00/episode-.../
```

吞吐量采用 `通过校验条数 / 整批墙钟时间 × 3600`，包括预检查、首条启动、文件收尾、全视频校验和最终进程退出。短批次不是长期稳态吞吐量。资源中的进程 CPU 100% 对应一个逻辑核；RSS 相加可能重复计入共享内存。GPU 查询是整张卡的系统占用，不能在有其他 GPU 作业时归因给录制任务。

单条脚本新增 `--offline`、`--gpu-index`；串行批量脚本新增 `--reuse-renderer` 和两种预热参数。旧调用保持原来的预览与逐条启动行为。
