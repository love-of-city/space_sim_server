# 2026-10-26 航空插头拔出后的温和搬运

## 问题

v5 修复了接触前追踪振荡，但拔出后 lift/transfer/above_hole 仍套用普通运动的
0.10 m/s、0.2 m/s²、0.8 m/s³ 上限。真实 UE 录制中三个阶段首秒内分别达到
约 0.1030、0.1009、0.1034 m/s；相对于拔出目标 0.006 m/s，视觉提速明显。

另外确认了一个独立现象：extract 阶段摩擦释放时曾有约 0.122 m/s 的短促回弹，
发生在约 26.4 s，早于完全拔出后的 lift（约 37.2 s）。本次修改针对用户所说的
拔出后快速抬升/搬运，不声称消除了这个接触释放瞬态，不改变保持力或接触物理。

## 修改

版本 `plug-insert/6-gentle-carry`。航空持件阶段 lift、transfer、above_hole 共用：

- carry_speed_m_s = 0.025（2.5 cm/s）
- carry_acceleration_m_s2 = 0.015（1.5 cm/s²）
- carry_jerk_m_s3 = 0.03

与原普通轨迹上限取较小值，并复用原 C2 样条和受限加减速规划。
三个阶段仍分别起停，未宣称 extract 到 lift 或 lift 到 transfer 已成为同一条
无停车轨迹。空载、接近、拔出、插入、释放、20 N 阻力及关闭超载停止设置保持。
导向/圆柱插头不启用该持件运动约束。历史 v5 阶段和抓取验收兼容保留。
代码保留 zmh_linux，未提交/推送。

## 实测与验证

128 passed, 1 skipped；Ruff、git diff --check 通过。新增测试验证三个持件阶段
实际 FK 参考的速度、加速度、jerk 及首秒不冲到巡航速度，历史 v5 仍可验收。

同一默认布局真实 UE trace，以每个阶段开始计时，从 200 Hz 末端位置差分求速度：

| 阶段 | 旧峰值 cm/s | 新峰值 cm/s | 新首秒峰值 cm/s | 新阶段时长 s |
|---|---:|---:|---:|---:|
| lift | 10.298 | 2.507 | 0.598 | 9.010 |
| transfer | 10.092 | 2.516 | 0.467 | 9.820 |
| above_hole | 10.339 | 2.506 | 0.455 | 9.040 |

| 全流程 | 结果 | 任务时间 s | 最终间隙 mm | 外部状态写入 |
|---|---|---:|---:|---:|
| native-random-01 | success | 120.915 | 0.7920 | 0 |
| recording-02/dynamics | success | 128.295 | 0.7514 | 0 |

默认布局真实 UE 双相机和一套位置/yaw 随机布局原生全流程均通过。
首个录制 recording-01 因未收到第一对相机帧，10 秒配对看门狗中止；没有改长
看门狗或降低验收，独立重启 recording-02 后成功。失败片段不是成功训练数据。
本次不改几何与接近路径，不重复已有接近碰撞预检。随机布局覆盖有限，不保证所有布局。

## 本地证据（不入 Git）

`/home/lyh/space_enviroment/runs/aviation-carry-speed-20261026/`

final-tests.log、final-manifest.json、source-hashes.json、final-source、native-random-01、
recording-02、final-episode-validation.json、carry-speed-metrics.json、carry-speed-before-after.png。
视频 aviation-gentle-carry-dualcam-1x.mp4 为正常速度全程；aviation-carry-clip-1x.mp4
截取 32–72 s，包含拔出末段、抬升、转运、移向孔口。
