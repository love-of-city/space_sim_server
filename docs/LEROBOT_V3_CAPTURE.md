# LeRobot v3 原生采集链路

本分支不是历史数据转换器。`EpisodeRecorder` 在收到完整的实时权威样本后调用
官方 `LeRobotDataset.add_frame()`；结束时调用 `save_episode()` / `finalize()`，完成
Parquet、视频、任务、统计及 episode 索引。没有读取 `steps.jsonl` 再转换的阶段。

## RGB-only 采集（2026-09-21）

视觉采集仅支持 `rgb`，已移除深度和实例分割的生成、传输、校验与保存链路。
默认保留总览、腕部两个相机，30 FPS（仿真时间），以及同步状态、动作和相机内外参。
`capture_products` 默认 `["rgb"]`，请求深度或分割会被拒绝。RGB 仍编码为 H.264/MP4，
原始 RGB 文件仍保存在平台 `cameras/` 审计目录。历史采集数据不修改、不删除。

升级时先结束录制并停止场景，重新构建 UE 插件，重启后端并重新创建场景。
旧 UE 发送深度/分割产品时，后端会明确报错并将本次录制标记为失败，而非静默丢弃。

## 工作目录与运行前提

- 服务端 worktree：`C:\Users\LYH\space_sim_server_lerobot_v3`
- UE worktree：`C:\Users\LYH\space_sim_UE_adapter_lerobot_v3`
- 两端分支：`feat/lerobot-v3-dataset`
- 原工作目录中的未提交修改没有自动带入本分支。
- 服务端依赖固定 `lerobot==0.4.4`，其数据格式版本为 `v3.0`。验证环境使用 Python 3.11。
  在要运行后端的环境执行 `python -m pip install -e ".[test]"`。
  完整仿真仍需要原有 Basilisk/MuJoCo 环境；只安装本项目不会安装 Basilisk。
- 必须编译并运行本 UE worktree；旧 UE 不发送新的采样声明，不能混用。
- 路径自动发现已将上述服务端 worktree 对应到新的 UE worktree。
  显式 `-AdapterRoot` 仍优先。这里没有重启、替换正在运行的原平台。

## 一次录制的产物

每次开始/结束录制生成一个独立的标准数据集，其中 `episode_index=0`。不是把所有
采集批次追加到同一数据集，也不需要手动运行转换命令。

```text
data/episodes/episode-.../
├── metadata.json                     # 平台状态/操作者/结果
├── lerobot/                          # LeRobotDataset 的 root
│   ├── meta/
│   │   ├── info.json                 # codebase_version=v3.0
│   │   ├── stats.json
│   │   ├── tasks.parquet
│   │   ├── episodes/chunk-000/file-000.parquet
│   │   └── platform.json             # 坐标系、动作语义、场景等补充说明
│   ├── data/chunk-000/file-000.parquet
│   └── videos/observation.images.<camera>/chunk-000/file-000.mp4
├── actions.jsonl                     # 原始控制审计，非训练数据的主存储
├── steps.jsonl                       # 保留完整诊断和非采样时刻状态
├── captures.jsonl                    # 原始相机标定/来源关联
└── cameras/                          # 原始产品，便于故障调查
```

只有结束接口返回 `dataset_status="complete"` 才是可发布的数据集。
`empty`、`failed` 不自动归档为成功数据。前端会显示帧数或错误，不再把操作结果
`outcome="success"` 当作数据格式完整性的依据。

## 特征与语义

| 特征 | 含义 |
|---|---|
| `observation.state` | 实测关节位置；SARM 前六项 rad，后两项手指位移 m |
| `observation.joint_velocity` | 相应的 rad/s、m/s |
| `action` | **此时控制器持有的关节伺服目标**；6 rad + 2 m。不是下一步实测位置，也不是 API 刚收到但未执行的末端命令 |
| `observation.end_effector_pose_body` | `cubesat_bus` 本体系下 `sarm_ee` 的 xyz(m) + wxyz |
| `observation.end_effector_twist_body` | 本体系末端线/角速度 |
| `observation.images.<camera>` | RGB H.264 视频 |
| `observation.camera_metadata.<camera>` | JSON 字符串，包含内外参及 UE 采集来源 |
| `observation.platform_json` | 全部原始仿真观测，包括姿控/反作用轮/诊断扩展 |
| `observation.sim_time_ns` | 原始仿真纳秒，int64 |
| `observation.source_frame_id` | 原始权威帧 ID，int64 |
| `observation.applied_action_sequence` | 仿真确认的动作序号，int64 |
| `timestamp` | episode 内 `frame_index / fps`；第一帧为 0 |

原始遥操作命令仍在 `actions.jsonl`，`steps.jsonl` 中仅按仿真确认的序号匹配动作，
不再把 `safety.last_action` 无条件当成已经执行的动作。

## 同步、失败与结束边界

1. 动力学 **240 Hz**，IK 默认 **120 Hz**（每 2 个物理步），权威渲染和 RGB
   默认 **30 FPS**（每 8 个物理步）。采集支持 `{1,2,5,10,30}` FPS。
   `RationalPhysicsClock` 使用 `round(step_index * 1e9 / 240)` 的整数实现计算绝对
   调度时刻，并在每次任务执行中更新下一实际物理步长；不累加取整后的固定周期。
   单步间隔交替为 4,166,666 / 4,166,667 ns，240 个步长恰好 1 秒。
   IK 与 MJScene 在同一任务内运行，默认仅在偶数物理步更新参考值；渲染每 8 步
   发布，随后冻结观测。UE / 后端只接受一致的、舍入到最近纳秒的绝对采样时刻。
   不把旧的非均匀状态伪装成新时间戳，不插值/复制训练帧。
   IK 配置必须是 240 的整数约数；旧的 100 Hz 场景必须重新创建或显式更改配置。
   姿控/反作用轮控制器使用其独立配置，未随本次 IK 频率修改而改变。
仿真观测现在由同一 `graspTask` 中紧随渲染发布器的快照模块冻结；不再在外层
`ExecuteSimulation` 结束后读取较晚的关节状态并附上较早的渲染时间戳。
仿真到 UE 的发送端与接收端在采集模式下都使用有界 FIFO/背压，预览模式保持 latest-wins。

2. UE 每个权威包增加 `dataset_format=lerobot-v3`、`sampling_fps`、`sample_index`
   和 `sampling_clock=simulation`。预览/插值画面不能进入数据集。
3. 按 render session、权威帧 ID、仿真纳秒严格配对，所有配置相机与产品齐全后
   才写样本。相机可早于或晚于状态到达。录制开始前 UE 已经排队的旧权威帧，
   在首个本 episode 观测帧建立边界后按帧 ID 丢弃，不占用 128 项配对窗口。
4. 不复制相机帧、不插值状态、不将缺失时段压缩成连续轨迹；缺帧/错帧、改变
   分辨率、非法图像或采样声明、会话变更、缓冲溢出会使本次数据集失败。
5. 点击结束时冻结观测边界；持续接收已选中的末帧相机。默认连续 10 秒无完整帧进展
   或总计 60 秒仍未排空才失败；有进展的正常积压不会仅因超过 10 秒被误判。然后后台线程
   编码并封口。新观测不能延长录制。失败保留原始审计数据供排查。
6. 记录中断/强制结束进程时不能承诺有效数据集；官方 writer 会缓冲数值数据和
   暂存图片，必须正常执行结束流程。正常后端关闭会尝试以 aborted 结果收尾。
7. 默认最多 18000 帧（30 FPS 下约 10 分钟仿真时间），防止单个 episode 无限占用内存；可以通过 `max_frames`
   显式设置。达到上限会标记失败，应在到达上限前结束并开始下一次。

相机数据采集默认开启。预览专用场景可以显式关闭 `dataset_capture`，但不能从该
场景开始带相机的数据集录制。纯数值录制可显式传 `camera_ids: []`。对于托管场景，
FPS 从场景设置继承；显式提供不一致 FPS 会被拒绝。

## 读取与验证

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(
    repo_id="local/my-recording",
    root=r"data/episodes/episode-.../lerobot",
    video_backend="pyav",  # Windows 验证使用此后端
)
print(dataset[0]["observation.state"])
print(dataset[0]["action"])
```

无需联网或上传 Hub。自动化测试 `tests/test_lerobot_recording.py` 使用真实 RGB PNG，
从实时回调生成视频/Parquet，再用官方 reader 逐帧检查状态、动作、双相机 RGB、
原始时间和任务，确认不生成深度/分割特征和附件，并覆盖缺帧、损坏包和停止时迟到相机。

参考固定版本官方实现：
- https://github.com/huggingface/lerobot/tree/v0.4.4/src/lerobot/datasets
- https://huggingface.co/docs/lerobot/lerobot-dataset-v3

## 历史 v3 接入验证记录（不是本次 240/120/30 变更的结果）

- 服务端大范围回归：434 passed / 35 skipped（独立运行，不混用原生 MuJoCo 与 Python MuJoCo）。
- 采集/录制/API/同步/路径/归档专项：49 passed。
- 原生 Basilisk/MuJoCo 重置与权威观测快照：2 passed；逐帧比对渲染时刻的关节状态。
- UE C++ Development Editor 编译通过；自动化报告 26 succeeded / 0 failed。
- UE Python 协议/适配器：50 passed / 16 subtests passed。
- 前端：77 passed。

这些验证不等同于“完整实景 UE 双相机长时间采集压测”；没有把正在运行的原平台
切到本分支。旧的 STEP->MuJoCo 编译测试在此混合依赖验证环境不能使用 Python
`mujoco.MjModel`，未把它计入通过项目。


## 240 / 120 / 30 Hz 升级验证（2026-09-21）

- `tests/test_sampling_clock.py`：检查默认配置、整数分频、禁止旧 IK 100 Hz 配置、
  一小时/一天/一年后的绝对时间点；这些是时间计算测试，不是长时间完整物理运行。
- `tests/test_physics_clock_native.py`：真实 Basilisk 调度运行至 60 秒，检查物理步、
  每两步 IK、分段 ExecuteSimulation 和重置后的逐步时间戳。
- `tests/test_lerobot_recording.py`：保留 10 FPS 兼容用例，并加入双相机 30 FPS
  RGB 视频/状态/动作往返读取；测试采样起点位于一天后的绝对仿真时间。
- 原生 Basilisk/MuJoCo 场景与重置：4 passed；核对每个物理状态消息的真实时间、
  240 Hz 实际任务序列、每帧 4 次 IK 更新和每 8 步一帧的原始观测。
- 采集/时钟/场景专项：57 passed；控制/场景大范围专项：167 passed（两组有重叠）。
- UE Python 协议/发布器：18 passed / 16 subtests passed；前端：103 passed。
- UE `Development Editor -NoLink` 编译通过；只检查源码编译，没有替换正在使用的
  插件 DLL，也没有运行新版 UE 自动化测试或完整双相机实时长时间压测。

生效顺序：结束录制、停止场景 → 完整运行 UE `scripts/build.ps1`（不要加 `-NoLink`）
→ 重启平台后端 → 刷新页面并重新创建场景。两端必须同时更新，历史 episode 不转换。
旧的命令行/场景文件如显式设置 IK 100 Hz，需要改为 120 Hz；默认启动脚本已更新。
姿控 PID 增益、关节限位、力矩限幅和原有安全机制没有为适应新频率而被放宽。


## 运行插件与拥堵丢帧修复（2026-09-21）

- 完整链接了 `UnrealEditor-BskUnrealRuntime.dll`，不再停留在 `-NoLink` 检查。
  启动采集时 `scripts/runtime_build.ps1` 检查 DLL 存在且不早于插件源码；过期会明确
  提示完整构建，避免旧 DLL 静默跳过 30 Hz。
- 实际 UE 验证发现，writer 首次初始化时接收短暂停顿，原先仅 16 包的发送队列满后
  返回失败并跳过了相机帧。现在保持同样的容量，队列满时等待（10 秒上限），并保留
  正在连接/重试的权威包；预览仍采用 latest-wins。
- 后端同步样本仍有 128 的容量上限，满时通过条件变量暂停状态接收，让相机线程继续
  排空，而不是增大缓存或伪造帧。连续等待超时仍会明确失败。
- 停止后冻结录制边界；持续有完整帧进展时允许排空积压，默认无进展超时 10 秒、
  总排空上限 60 秒；随后才编码、封口。
- 回归：采集/录制/接收/停止/启动保护 50 passed。
- 隔离端口上的实际 UE + Basilisk + 后端录制完成：
  `episode-20260921-182043-a547fbee`，188 个同步样本、376 张原始 RGB、
  `dataset_status=complete`、`incomplete_sample_count=0`。两路 MP4 均为
  640×360、30 FPS、188 帧，全部视频帧可解码，官方 LeRobot reader 可读，归档完成。
- 验证产物：工作区顶层 `run/rgb30-live-verified-20260921/`。
  未修改生产身份认证信息、原失败 episode 或保存的场景配置；验证进程已清理。
  这是短录制验证，不代表长时间满负载或公网浏览器端性能保证。
