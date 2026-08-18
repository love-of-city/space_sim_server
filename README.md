# 太空机械臂遥操作与数据采集平台

本项目负责人类操作、任务管理和训练数据记录，不接管动力学或渲染权威：

```text
浏览器键盘/手柄
      ↓ WebSocket
本项目后端（限幅、失联保护、任务和记录）
      ↓ space-arm-control/1
BSK + MJScene（500 Hz权威动力学、逆运动学与接触）
      ↓ bsk-render/2
space_sim_UE_adapter / UE5（渲染与相机采集）
      ↓ bsk-capture/1
本项目后端 → 浏览器预览 + episode数据目录
```

UE不会根据浏览器输入自行移动Actor。机械臂画面始终来自MJScene计算后的真实状态。

## 双画面通道

- **操作预览**：默认 30 Hz，只保留最新 RGB 帧；使用 15 ms 插值缓冲和最多 50 ms 的短时视觉外推，让网页遥操作更流畅。该通道不会写入训练集。
- **权威采集**：默认 10 Hz，在 UE 中临时应用精确的 BSK/MJScene 帧并同步采集 RGB、深度和分割；不使用插值或外推。后端按 `source_frame_id -> render_frame_id -> step_id` 严格配对后才保存。

两条通道共享相机定义，但具有独立的调度和队列；预览拥塞时覆盖旧帧，权威采集采用有界可靠队列并明确报告溢出。

## 当前已实现

- 默认动作空间为末端平移3维、末端旋转3维和夹爪开合。
- 从MJCF自动解析安装位姿、关节轴和工具坐标，使用阻尼最小二乘雅可比逆解。
- SO-101只有5个机械臂自由度，因此六维命令会投影到物理可实现的5维运动，并记录命令残差和雅可比秩。
- Web操作台支持键盘、浏览器Gamepad API、末端状态和相机选择。
- 单操作员控制权；其他连接自动成为只读观察者。
- 仿真模式按键和手柄输入直接生效；松键、窗口失焦、断线或250 ms超时立即停止。
- `Esc`和页面急停按钮会锁存停止状态，必须点击“恢复控制”才能解除。
- 末端线速度、角速度、关节速度、关节位置和夹爪速度均有限制。
- BSK/MJScene以500 Hz运行，前端约30 Hz发动作，UE约30 Hz显示。
- UE RGB通过 `bsk-capture/1` 回传网页；RGB、深度和分割原始产品写入episode。
- 记录用户请求、过滤后动作、关节状态、末端位姿/速度、IK残差、时间戳和相机数据。

这仍是人工遥操作，不是视觉闭环或自主抓取策略。

## 环境

- Unreal Engine：`E:\UE5.6`
- UE适配器：`E:\mujoco_demo\space_sim_UE_adapter`
- Basilisk/MJScene：Conda环境 `mujoco-dev`
- Web后端：Python、FastAPI、Uvicorn、Pydantic

```powershell
Set-Location E:\mujoco_demo\space_arm_data_platform
python -m pip install -e ".[test]"
```

## 一条命令运行

```powershell
Set-Location E:\mujoco_demo\space_arm_data_platform
.\scripts\run_platform.ps1
```

首次加载Basilisk/MJScene和模型可能需要30～60秒。网页地址为 `http://127.0.0.1:8000`。

自定义任务时长和采集率：

```powershell
.\scripts\run_platform.ps1 -Duration 600 -PreviewRate 30 -CaptureRate 10 -SimulationRate 1
```

已有17个机械臂网格会直接复用。只有源STL或材质变化时才重新导入：

```powershell
.\scripts\run_platform.ps1 -ReimportAssets
```

停止：

```powershell
.\scripts\stop_platform.ps1
```

## 键盘操作

当前为纯仿真直接控制模式，不需要按住空格。

右侧“末端平移速度”滑块可在仿真运行中随时调整 WASD/QE 的速度，默认为 `0.05 m/s`，后端安全范围为 `0.01～0.20 m/s`。所选值和后端实际应用值均会写入动作记录。

不按Shift时控制末端平移：

| 运动 | 负方向 | 正方向 |
|---|---:|---:|
| X 前后 | S | W |
| Y 左右 | A | D |
| Z 上下 | E | Q |

按住Shift时控制末端旋转：

| 运动 | 负方向 | 正方向 |
|---|---:|---:|
| Roll | Shift+E | Shift+Q |
| Pitch | Shift+S | Shift+W |
| Yaw | Shift+A | Shift+D |

夹爪始终使用 `F/R` 进行闭合/张开。

手柄默认映射：左摇杆平移XY、LT/RT平移Z、右摇杆Pitch/Yaw、A/B控制Roll、X/Y控制夹爪。

## 数据位置

每次点击“开始采集”后创建：

```text
data/episodes/episode-日期时间-随机ID/
├── metadata.json
├── actions.jsonl
├── steps.jsonl
├── captures.jsonl
└── cameras/            # RGB、深度、分割
```

网页 RGB 只用于操作预览；训练数据直接保存 UE 权威帧的原始产品，不从网页截图反推。`steps.jsonl` 中记录 `step_id` 和 `render_frame_id`；`captures.jsonl` 记录匹配的 `source_frame_id`、`sim_time_ns` 及 `authoritative_state=true`。停止 episode 时，`metadata.json` 会给出匹配、待匹配和拒绝数量。

## 单独调试与验证

```powershell
# 只启动后端和网页
.\scripts\run_backend.ps1

# 已有后端和UE时只启动仿真
.\scripts\run_simulation.ps1

# 自动化测试
python -m pytest

# 用原生MuJoCo校验MJCF正运动学
conda run --no-capture-output -n mujoco-dev python .\tools\validate_mujoco_fk.py
```

日志位于 `logs`。运行PID只临时写入 `run/platform.json`，停止脚本校验进程启动时间后再结束进程。

## 下一步

1. 增加末端工作空间、自碰撞和抓取接触约束。
2. 将轨道姿态保持与机械臂遥操作组合进同一任务。
3. 对长时间/高吞吐采集增加分片文件格式与离线完整性校验。
4. 增加末端力/力矩反馈、任务重置和场景随机化。
5. 让学习策略和人类操作复用同一个末端动作接口。
