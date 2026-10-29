# 2026-10-25 航空插头直线接近与抽屉速度对齐

## 变更

用户要求接近时同步平移、旋转和张爪，运动参数参照完整抽屉脚本，并确认操作台随机初始化兼容。
保持 `zmh_linux`，未提交或推送。没有修改 UE、接触几何、插拔阻力或运行中的物体状态。

航空任务版本 `plug-insert/4-coordinated-approach`，去掉独立 raise/above_pickup 阶段。
从当前末端到预抓取位作名义笛卡尔直线，转腕和张爪共用路径进度；随后 45 mm 轴向慢速接近。
两个阶段之间仍会减速到零，未声称已经全程无停顿。实际轨迹包含伺服误差及实测物体漂移修正。

自由移动参数从 DrawerCycleConfig 取默认值：平移 0.10 m/s、慢速 0.025 m/s、
关节 0.35 rad/s、旋转 0.20 rad/s、张爪 0.20 rad/s。复用 time_joint_path
的 C2 样条及速度/加速度/jerk 约束，张爪同步限导数并随跟踪暂停一起暂停。
修复 run_scripted_task 入口强制覆盖 arm_speed=0.08 导致默认参数失效的问题。
其他两类插头保留原速度。拔出 0.006 m/s、插入 0.002 m/s、航空接触反馈关节推进
最多 0.08 rad/s；20 N 静摩擦/滑动阻力保持、默认超载停止关闭。
释放保留已有伺服卸载和实际张开确认。

## 随机布局

从本次模型读取起始座和目标孔；卡扣夹持轴来自实测物体姿态；拔出保持起始座朝向，
转运、插入采用目标孔朝向。模型切换后采集器创建对应模型的 NativePoseProbe，
检查同一执行样条与张爪进度。碰撞/逆解失败拒绝并重采样。
预检是静态离散接近检查，不覆盖搬运/插入全程或任意动力学扰动。
v2/v3 历史阶段验证保留，v3/v4 都校验拔出时双指接触和卡扣对齐。

## 验证

- 相关 Python 回归：121 passed, 1 skipped（FastAPI testclient 既有弃用警告）。
- Ruff、git diff --check 通过。
- 默认布局原生接近预检 322 位姿通过。
- 位置+朝向随机布局：原有初始关节和另三组初始关节（seed 2026102500–02）均通过，每组 322 位姿。
- 原生完整动力学默认布局、随机布局，以及 UE 双相机录制全部成功，见下表。
- 录制每路 5208 帧，1024×1024、50 Hz；生产 validate_recording 完成完整解码、帧配对、状态动作、200 Hz 腕力校验。
- 默认布局实测接近耗时 8.85 s，末端轨迹相对端点直线最大偏差约 0.78 mm。
- 速度参数是名义参考上限，不是物理实测速度硬限。200 Hz 差分显示细小跟踪振动；默认布局最后接近峰值约 0.062 m/s，虽名义限速为 0.025 m/s。
- 高接触负载仍存在，不能把成功装配当成已标定的人类操作力或硬件安全验证。
- 只实跑一套位置+朝向随机布局，不代表任意随机布局必成功。

| 验证 | 结果 | 仿真任务时间 s | 最终间隙 mm | 峰值腕力 N | 外部写状态 |
|---|---|---:|---:|---:|---:|
| native-01 | success | 103.565 | 0.7434 | 170.43 | 0 |
| native-random-01 | success | 105.160 | 0.7932 | 305.24 | 0 |
| recording-01/dynamics | success | 104.125 | 0.7486 | 170.43 | 0 |

## 本地证据（不入 Git）

`/home/lyh/space_enviroment/runs/aviation-straight-approach-20261025/`

包含 final-manifest.json、source-hashes.json、final-source、final-tests.log、
preflight.json、random_preflight.json、native-01、native-random-01、recording-01、
final-episode-validation.json，以及 aviation-straight-dualcam-1x.mp4（正常速度）。
