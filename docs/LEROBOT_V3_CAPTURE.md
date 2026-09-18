# LeRobot v3 原生采集链路

本分支不是历史数据转换器。`EpisodeRecorder` 在收到完整的实时权威样本后调用
官方 `LeRobotDataset.add_frame()`；结束时调用 `save_episode()` / `finalize()`，完成
Parquet、视频、任务、统计及 episode 索引。没有读取 `steps.jsonl` 再转换的阶段。

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
│   ├── videos/observation.images.<camera>/chunk-000/file-000.mp4
│   └── auxiliary/depth/<camera>/frame-000000.pfm
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
| `observation.segmentation.<camera>` | 无损 RGB 实例 ID 图像，嵌入 Parquet；不能用有损视频保存标签 |
| `observation.depth_path.<camera>` | 指向数据集内 PFM 的相对路径字符串；float32 米制 camera_z 深度，不是自动解码的 LeRobot image 张量 |
| `observation.camera_metadata.<camera>` | JSON 字符串，包含内外参、实例标签映射及 UE 采集来源 |
| `observation.platform_json` | 全部原始仿真观测，包括姿控/反作用轮/诊断扩展 |
| `observation.sim_time_ns` | 原始仿真纳秒，int64 |
| `observation.source_frame_id` | 原始权威帧 ID，int64 |
| `observation.applied_action_sequence` | 仿真确认的动作序号，int64 |
| `timestamp` | episode 内 `frame_index / fps`；第一帧为 0 |

深度保留原始精度，不伪装成 RGB 或做有损压缩。需要使用深度训练时，消费者按
`depth_path` 读取 PFM；通用 LeRobot RGB 策略不会自动加载这个附件。
原始遥操作命令仍在 `actions.jsonl`，`steps.jsonl` 中仅按仿真确认的序号匹配动作，
不再把 `safety.last_action` 无条件当成已经执行的动作。

## 同步、失败与结束边界

1. 当前 500 Hz（2 ms）动力学与 30 Hz 渲染发布器共同支持精确的 `{1,2,5,10}` FPS；默认 10。
   UE 与后端使用同一仿真时间采样网格，允许 1 微秒的整数纳秒舍入误差。
仿真观测现在由同一 `graspTask` 中紧随渲染发布器的快照模块冻结；不再在外层
`ExecuteSimulation` 结束后读取较晚的关节状态并附上较早的渲染时间戳。
仿真到 UE 的发送端与接收端在采集模式下都使用有界 FIFO/背压，预览模式保持 latest-wins。

2. UE 每个权威包增加 `dataset_format=lerobot-v3`、`sampling_fps`、`sample_index`
   和 `sampling_clock=simulation`。预览/插值画面不能进入数据集。
3. 按 render session、权威帧 ID、仿真纳秒严格配对，所有配置相机与产品齐全后
   才写样本。相机可早于或晚于状态到达。
4. 不复制相机帧、不插值状态、不将缺失时段压缩成连续轨迹；缺帧/错帧、改变
   分辨率、非法图像或采样声明、会话变更、缓冲溢出会使本次数据集失败。
5. 点击结束时冻结观测边界；最多等待 10 秒接收已选中的末帧相机，然后后台线程
   编码并封口。新观测不能延长录制。失败保留原始审计数据供排查。
6. 记录中断/强制结束进程时不能承诺有效数据集；官方 writer 会缓冲数值数据和
   暂存图片，必须正常执行结束流程。正常后端关闭会尝试以 aborted 结果收尾。
7. 默认最多 18000 帧，防止单个 episode 无限占用内存；可以通过 `max_frames`
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

无需联网或上传 Hub。自动化测试 `tests/test_lerobot_recording.py` 使用真实 PNG/PFM，
从实时回调生成视频/Parquet，再用官方 reader 逐帧检查状态、动作、RGB、无损标签、
原始时间、任务和附件路径，并覆盖缺帧、损坏包和停止时迟到相机。

参考固定版本官方实现：
- https://github.com/huggingface/lerobot/tree/v0.4.4/src/lerobot/datasets
- https://huggingface.co/docs/lerobot/lerobot-dataset-v3

## 本次验证记录

- 服务端大范围回归：434 passed / 35 skipped（独立运行，不混用原生 MuJoCo 与 Python MuJoCo）。
- 采集/录制/API/同步/路径/归档专项：49 passed。
- 原生 Basilisk/MuJoCo 重置与权威观测快照：2 passed；逐帧比对渲染时刻的关节状态。
- UE C++ Development Editor 编译通过；自动化报告 26 succeeded / 0 failed。
- UE Python 协议/适配器：50 passed / 16 subtests passed。
- 前端：77 passed。

这些验证不等同于“完整实景 UE 双相机长时间采集压测”；没有把正在运行的原平台
切到本分支。旧的 STEP->MuJoCo 编译测试在此混合依赖验证环境不能使用 Python
`mujoco.MjModel`，未把它计入通过项目。
