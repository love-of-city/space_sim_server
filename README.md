# 太空机械臂遥操作与数据采集平台

本项目负责人类操作、任务管理和训练数据记录，不接管动力学或渲染权威：

```text
浏览器键盘/手柄
      ↓ WebSocket
本项目后端（限幅、失联保护、任务和记录）
      ↓ space-arm-control/1
BSK（默认100 Hz逆运动学）+ MJScene（500 Hz权威动力学与接触）
      ↓ bsk-render/2
space_sim_UE_adapter / UE5（渲染与相机采集）
      ├─ Pixel Streaming 2 / WebRTC → 浏览器操作预览
      └─ bsk-capture/1 → 本项目后端 → episode数据目录
```

UE不会根据浏览器输入自行移动Actor。机械臂画面始终来自MJScene计算后的真实状态。

## 双画面通道

- **操作预览**：网页直接集成 Epic UE 5.6 Pixel Streaming SDK，不使用 iframe。默认可选择 UE 主视口、卫星总览 RenderTarget 和腕部 RenderTarget；该画面只供遥操作，不写入训练集。
- **权威采集**：默认 10 Hz，在 UE 中临时应用精确的 BSK/MJScene 帧并同步采集 RGB、深度和分割；不使用插值或外推。后端按 `source_frame_id -> render_frame_id -> step_id` 严格配对后才保存。

两条通道相互独立：Pixel Streaming 发送 UE 当前主视口，权威采集仍通过 `bsk-capture/1` 传送相机数据。WebRTC 丢帧或网络波动不会改变 BSK/MJScene 状态，也不会污染训练数据。

## 当前已实现

- 默认动作空间为末端平移3维、末端旋转3维和夹爪开合。
- 从MJCF自动解析安装位姿、关节轴和工具坐标，使用阻尼最小二乘雅可比逆解。
- SO-101只有5个机械臂自由度，因此六维命令会投影到物理可实现的5维运动，并记录命令残差和雅可比秩。
- Web操作台支持键盘、浏览器Gamepad API和末端状态；显示 WebRTC RTT、码率、丢包、解码帧率和分辨率。
- Pixel Streaming SDK 的键盘控制被关闭，机械臂动作仍只通过 `/ws/operator` 进入后端和权威仿真。
- UE 为 manifest 相机动态创建独立 `SceneCapture2D + RenderTarget + Streamer`，浏览器切换相机不会改变仿真状态。
- 单操作员控制权；其他连接自动成为只读观察者。
- 仿真模式按键和手柄输入直接生效；松键、窗口失焦、断线或250 ms超时立即停止。
- `Esc`和页面急停按钮会锁存停止状态，必须点击“恢复控制”才能解除。
- 末端线速度、角速度、关节速度、关节位置和夹爪速度均有限制。
- IK在独立BSK控制任务中默认以100 Hz更新并缓存关节目标；MJScene以500 Hz运行PID、力和接触积分，避免在RK4阶段重复求解IK。
- 前端约30 Hz发动作，UE通过 WebRTC 以最高60 FPS预览。
- UE主视口经 Pixel Streaming 2 回传网页；RGB、深度和分割权威产品仍通过 `bsk-capture/1` 写入episode。
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

首次运行会使用 UE 5.6 自带脚本准备官方 Pixel Streaming Infrastructure（约十几 MB，并安装其 Node 依赖）；不会安装另一套 UE。随后加载Basilisk/MJScene和模型可能需要30～60秒。网页地址为 `http://127.0.0.1:8000`。

自定义任务时长、IK频率和采集率：

```powershell
.\scripts\run_platform.ps1 -Duration 600 -IkRate 100 -PreviewRate 60 -CaptureRate 10 -SimulationRate 1
```

已有17个机械臂网格会直接复用。只有源STL或材质变化时才重新导入：

```powershell
.\scripts\run_platform.ps1 -ReimportAssets
```

停止：

```powershell
.\scripts\stop_platform.ps1
```

本机默认端口：操作台 `8000`、Pixel Streaming 播放器 `8080`、UE 信令 `8888`。当前脚本面向本机 HTTP 使用；跨机器或公网部署时必须另外配置可访问的公网地址、HTTPS 以及 STUN/TURN。

## 跨机器安全模式

局域网或公网部署时显式启用 JWT 信令；本机模式默认不增加鉴权复杂度：

```powershell
.\scripts\run_platform.ps1 -RemoteAccess -PublicHost 192.168.1.50
```

脚本会生成本次运行的访问密钥和 JWT 密钥，并输出带 `access_key` 的操作台地址。安全信令在 WebSocket 握手时验证短期 JWT，token 只允许访问声明的 Streamer ID，订阅时会再次校验。
默认只有播放器端口监听外部网卡；UE Streamer 端口只绑定 `127.0.0.1`，避免外部进程冒充同名 UE Streamer。

跨 NAT 时可以注入 STUN/TURN：

```powershell
.\scripts\run_platform.ps1 -RemoteAccess -PublicHost simulator.example.com `
  -PixelPlayerPublicUrl wss://simulator.example.com/stream `
  -IceServersJson '[{"urls":"stun:stun.example.com:3478"}]' `
  -TurnUrlsJson '["turn:turn.example.com:3478?transport=udp"]' `
  -TurnAuthSecret 'replace-with-turn-rest-secret'
```

公网环境还需要在平台前方部署 HTTPS/WSS 反向代理；仓库不会自行签发证书。TURN 凭据由安全信令按 HMAC 临时生成，不会把 TURN 长期密码发给浏览器。

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

data/
├── archives/           # 完成 episode 的不可变 tar.gz 与 SHA-256
├── jobs/               # 可恢复的归档任务状态
└── tasks/              # 与 UE/WebRTC 会话解耦的任务状态
```

网页 WebRTC 视频只用于操作预览；训练数据直接保存 UE 权威帧的原始产品，不从网页视频或截图反推。`steps.jsonl` 中记录 `step_id` 和 `render_frame_id`；`captures.jsonl` 记录匹配的 `source_frame_id`、`sim_time_ns` 及 `authoritative_state=true`。停止 episode 时，`metadata.json` 会给出匹配、待匹配和拒绝数量。

停止 episode 后，后端自动提交有幂等键的归档任务：先生成逐文件 SHA-256 清单，再以临时文件写入并原子发布 `.tar.gz`。`/api/jobs` 可查询归档状态；`/api/tasks`、`/api/tasks/{id}/start` 和 `/api/tasks/{id}/complete` 提供持久化任务调度接口。

## 单独调试与验证

```powershell
# 只启动后端和网页
.\scripts\run_backend.ps1

# 已有后端和UE时只启动仿真
.\scripts\run_simulation.ps1

# 自动化测试
python -m pytest

# 平台已启动时，自动验证浏览器确实解码到了 UE WebRTC 视频帧
node .\tools\verify_pixel_streaming.mjs

# 用原生MuJoCo校验MJCF正运动学
conda run --no-capture-output -n mujoco-dev python .\tools\validate_mujoco_fk.py
```

日志位于 `logs`。运行PID只临时写入 `run/platform.json`，停止脚本校验进程启动时间后再结束进程。

## 下一步

1. 增加末端工作空间、自碰撞、抓取接触约束和力/力矩反馈。
2. 对长时间高吞吐采集增加分片数据格式和对象存储同步。
3. 增加任务重置、场景随机化，并让学习策略复用同一动作接口。
