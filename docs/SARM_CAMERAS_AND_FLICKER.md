# SARM 相机接入与画面频闪排查（2026-09-07）

## 生效的模型与相机

平台运行入口是 `../model/SARM/platform/sarm_platform.xml`。编辑器中常用的
`../model/SARM/mjcf/SARM.xml` 是独立模型入口，`SARM_scene.xml` 会包含它。
本次在两个入口中配置了同名、同标定的相机，没有改动关节、惯量、执行器或网格。

所有位置单位为米，均相对于挂载的 body，而不是惯性世界坐标。

| 相机 | XML 挂载 body | 局部位置 | 局部瞄准参考点 | 垂直 FOV | 分辨率 |
| --- | --- | --- | --- | --- | --- |
| `spacecraft_overview` | 独立模型 `sarm_base` / 平台模型 `cubesat_bus` | `(1.25, -1.45, 1.05)` | `(0.20, 0.10, 0.10)` | 50° | 640×360 |
| `sarm_wrist_cam` | `link6` | `(0.22, 0, 0.12)` | `(0.08, 0, 0.29)` | 60° | 640×360 |

瞄准参考点用于计算 XML 中的固定四元数，不是运行时追踪目标。
腕部相机采用侧置斜视，避免腕部壳体挡住夹爪与目标。其视角跟随 `link6` 运动。

生产推流 ID：

- `BskRenderer__teleop_camera_spacecraft_overview`
- `BskRenderer__teleop_camera_sarm_wrist_cam`

`BskRenderer` 是默认 streamer 前缀，使用自定义前缀时会相应替换。
`run_platform.ps1` 不再请求旧的 `so101_wrist_cam`。
`teleop_grasp_unreal.py` 统一通过 `add_mj_scene()` 读取 XML 相机，不再额外注册旧模型的
硬编码总览相机；画中画槽位依次为 1、2。

主视口还修正了另一个绑定问题：`sarm_ee` 是 MJCF site，并不是 UE 注册的 body actor。
默认聚焦改为实际存在的 `teleop/cubesat_bus`，观察距离 2.8 m。

## 频闪根因与修复

### 1. 移动原点位置与惯性速度混用

适配器 `FrameConverter.position()` 输出浮动原点下的位置：

```text
p_local = C_LN * (p_inertial - origin_inertial)
```

而 `FrameConverter.velocity()` 输出的是惯性速度在局部轴上的分量：

```text
v_wire = C_LN * v_inertial
```

UE 原来的 `ABskSceneController::ExtrapolateFrame()` 用 `v_wire` 直接外推 `p_local`。
对于原点跟随的卫星，接收位置恒为 `(0,0,0)`，但速度仍约为 7612.6 m/s。
只要外推 15 ms，就会额外移动约 114 m；收到下一帧后又回到原位。这会表现为模型在主画面
反复消失/出现，或相机与目标相对位置异常，并非单纯的网络丢帧。

修复后，物体和天体均从相邻权威帧的**局部位置差**计算视觉外推速度。
保留惯性速度用于轨道遥测，不改动力学、不改变采集所用的权威帧。
外推仍受原有 50 ms 上限约束。

### 2. 旧前端播放器回调污染新播放器

SDK 的 `disconnect()` 可以同步触发 `webRtcDisconnected`。之前先断开旧播放器、后清空
所有权，旧回调可能创建一个重连定时器；随后创建新播放器时覆盖定时器句柄，留下未取消的重连。
此外，旧播放器的迟到事件可以修改当前 LIVE 状态。

现在先撤销旧播放器所有权，再执行断开；8 类事件处理均检查自己是否仍为当前播放器。
配置请求失败也使用统一管理的重连定时器。添加了同步断开、迟到事件、真实断线和清理的回归测试。

### 3. 相机 FOV 接口

`mjcf_assets.py` 之前将 XML 的垂直 `fovy` 直接当作 UE 的水平 FOV。
现在按分辨率宽高比转换，保留基于 sensorsize/focal 的已有水平 FOV 分支。
这是相机取景校正，不是频闪的主要根因。

## 验证结果

- UE Development Editor 模块编译成功。
- UE 自动化：15 项通过，包括新增的移动原点外推回归测试。
- 适配器 Python：40 项测试，39 通过、1 跳过，无失败。
- SARM 相机配置/挂载/视锥：5 项通过。
- 前端播放器生命周期：4 项通过，生产构建成功。
- 最终 XML 经实际 Basilisk/MJScene 启动，3 s 协议采样正常退出；manifest 包含两台相机和有效主视口聚焦目标。
- 通过浏览器 WebRTC 实际订阅主视口、总览、腕部流，保存了各自截图与连续解码帧统计。

对同一段局部姿态冻结、惯性原点移动的回放，在相同相机配置下比较重新编译前后的 UE：

| 指标 | 修复前 | 修复后 |
| --- | ---: | ---: |
| 采集到的浏览器解码帧 | 1042 | 1052 |
| 主画面采样区域亮度标准差（0–255 标度） | 0.23190 | 0.00466 |
| 相邻帧该区域平均亮度的最大变化 | 0.63329 | 0.00976 |
| 测试期间挂载的视频元素 | 1 | 1 |

标准差下降约 98%。这是本机固定输入回放的诊断指标，不代表所有运动场景或长时间运行的保证。
已有工作区修改未回退，当前 UE 构建也包含这些已有修改。

证据保存在 `run/camera-flicker-20260907/`：

- `verification-summary.json`：视频统计。
- `replay-before/`、`replay-after/`：相同输入、相同相机配置的对照。
- `main-calibrated/video.png`、`wrist-calibrated/video.png`、`overview-final/video.png`：最终取景截图。
- `ue-tests/index.json`、`ue-build.log`、`adapter-tests.log`：自动化和构建记录。
- `native-final/manifest.json`：最终 XML 实际发出的相机描述。
- `SARM.before.xml`、`sarm_platform.before.xml`、`server.before.diff`：修改前备份。

## 动力学问题的后续处理（2026-09-07）

相机/频闪修复完成时，原生场景仍有 CTRL 初始化警告、目标漂离和机械臂无法保持姿态的问题。
随后已定位并修复自由飞行关节继承阻尼/摩擦、重力执行顺序、目标速度参考系及夹爪增益问题。
详细原因、修改文件、数值证据及验证边界见 `SARM_CONTROL_DIAGNOSIS.md`。

旧模型测试样例已在后续控制修复中迁移到 SARM，超时测试也改为可控时钟。
目前服务端完整测试 48 项通过，前端测试 6 项通过；此前“7 项失败”的记录是相机修复阶段的历史状态。

## 使用

前端已重新构建，UE 模块已重新编译。刷新控制台后重新启动场景即可加载新 XML 和相机 ID；
旧 UE 进程不会热更新这些内容。相机变更不需要重新导入 STL/OBJ 网格。

后续调整位置和四元数时，请同步修改两份 XML，并执行：

```powershell
# 在服务端项目目录 space_sim_server 下，使用服务端 Python 环境
python -m pytest tests/test_sarm_cameras.py -q
npm.cmd --prefix frontend test
```
