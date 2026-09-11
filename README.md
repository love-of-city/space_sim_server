# 太空机械臂遥操作与数据采集平台

## 一键公网启动 / 停止

在远程仿真机的本仓库根目录：

- 双击 **`start_deployment.cmd`**：没有旧固定域名配置时，自动创建临时公网 HTTPS 入口；**不用填写 IP、域名、证书或初始化密码**。完成后打开访问链接/登录信息窗口。
- 双击 **`stop_deployment.cmd`**：关闭本次公网隧道及整个平台/场景。
- 双击 **`show_deployment_access.cmd`**：重新查看当前公网访问链接和自动生成的初始化密码。

自动加载已有环境，缺少工具时下载校验，密码/密钥加密保存。已有管理员自定义密码不变；已知默认密码在备份后自动安全更换。首启切换或强制重启会停止旧场景。

**这是临时公网发布，不是固定域名的正式生产托管**：自动网址重启可能变化，无服务可用性保证；WebRTC 视频仍可能需要 TURN，不能仅凭网页打开就认定视频连通。用户网页和手柄在用户自己的电脑上使用。详见 [免填写公网发布](docs/PUBLIC_DEPLOYMENT.md)。已有固定域名配置继续使用原模式，见 [高级部署说明](docs/DEPLOYMENT.md)。

## 架构与文档

- [系统总体架构设计与实现总结](docs/SYSTEM_ARCHITECTURE.md)：系统组成、进程/协议、控制和采集链路、模型/器件配置归属、部署及当前缺口。
- [仿真核心架构与扩展约定](docs/SIMULATION_ARCHITECTURE.md)：当前原生 SARM 调用链与通用架构接口的区别。
- [场景初始化太阳光照](docs/SUNLIGHT_CONFIGURATION.md)：0～20,000 倍太阳照明、实例保存、UE 参数传递与兼容说明。
- [反作用轮与姿态保持](docs/ATTITUDE_CONTROL.md)：物理参数、惯性保持闭环、遥测、限幅和验证。
- [关节卫星待抓取目标](docs/GROUND_CAPTURE_TARGET.md)：新默认场景、原始模型保护、估算物理参数、UE 资源与旧实例兼容。
- [模型目录与 Git LFS](model/README.md)：运行入口、源模型、CAD 和本地兼容路径。

> **更新（2026-09-08）**：已修复 SARM 8 项观测被后端 6 项限制拒收的问题，新增明确 SI 关节字段、三轴反作用轮及默认初始惯性姿态保持。真实 Hub/Recorder 和渲染协议已做隔离集成验证；GPU 视频/权威图像链路的覆盖边界见[架构文档](docs/SYSTEM_ARCHITECTURE.md#gaps)。

## 平台概览

本项目负责人类操作、任务管理和训练数据记录，不接管动力学或渲染权威：

```text
浏览器键盘/手柄
      ↓ WebSocket
本项目后端（限幅、失联保护、任务和记录）
      ↓ space-arm-control/1
BSK（默认100 Hz逆运动学）+ MJScene（500 Hz权威动力学与接触）

当前重构中的职责边界和扩展约定见 [仿真流程架构](docs/SIMULATION_ARCHITECTURE.md)。
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

- 场景初始化支持太阳光照强度倍率（0～20,000，默认 1），配置随实例保存并控制 UE 太阳照明，不修改星历或动力学。

- 三个正交反作用轮在 MJScene 中产生真实反力矩，BSK 100 Hz 姿态闭环保持初始 J2000 惯性姿态；前端显示姿态误差、轮速、轮矩和饱和状态。
- 默认动作空间为末端平移3维、末端旋转3维和夹爪开合。
- 从MJCF自动解析安装位姿、关节轴和工具坐标，使用阻尼最小二乘雅可比逆解。
- 默认 SARM 模型包含6个机械臂转动关节和2个夹爪移动关节；六维末端命令由雅可比逆解转换，并记录命令残差和雅可比秩。
- Web操作台支持键盘、浏览器Gamepad API和末端状态；显示 WebRTC RTT、码率、丢包、解码帧率和分辨率。
- 机械臂操作模式由页面截获相关按键，动作只通过 `/ws/operator` 进入后端和权威仿真；自由相机使用独立 `BskCameraInput` 命令直达 UE 玩家相机，不经旧键码转发，不改变物理场景。
- UE 为 manifest 相机动态创建独立 `SceneCapture2D + RenderTarget + Streamer`，浏览器切换相机不会改变仿真状态。
- 采用管理员/操作员两种登录角色；场景创建者自动拥有该场景的操作和采集权限，同一用户的新页面点击画面后会自动替换旧操作页面。
- 仿真模式按键和手柄输入直接生效；松键、窗口失焦、断线或250 ms超时立即停止。
- 点击实时画面进入键盘操作模式；`Esc`立即归零并退出操作模式，页面急停按钮单独负责锁存停止。
- 末端线速度、角速度、关节速度、关节位置和夹爪速度均有限制。
- IK在独立BSK控制任务中默认以100 Hz更新并缓存关节目标；MJScene以500 Hz运行PID、力和接触积分，避免在RK4阶段重复求解IK。
- 前端约30 Hz发动作，UE通过 WebRTC 以最高60 FPS预览。
- UE主视口经 Pixel Streaming 2 回传网页；RGB、深度和分割权威产品仍通过 `bsk-capture/1` 写入episode。
- UE 环境复现 MyProject2 的原始 Sphere、8K 地球昼夜/法线/高光、云层/大气材质、银河星空、Lumen、光追、虚拟阴影和自动/局部曝光；对象尺寸按仿真设置，Sun 的照射方向和距离照度规律仍由现有星历规则驱动。当前 SARM 场景由与重力共享的 Earth/Sun 星历驱动天体位置和姿态，装饰地球已关闭；显示光照仍有标定和成像近似边界。
- 记录用户请求、过滤后动作、关节状态、末端位姿/速度、IK残差、时间戳和相机数据。

这仍是人工遥操作，不是视觉闭环或自主抓取策略。

## 首次部署

将两个仓库克隆到同一个父目录。当前 SARM 的 MJCF、网格、原生场景控制脚本及任务盒 CAD 源文件随本服务端的 `model/` 提供；UE 网格、地球和星空资产随适配器仓库提供。两个仓库都需要拉取 Git LFS：

```powershell
git lfs install
git clone https://github.com/love-of-city/space_sim_UE_Adapter.git space_sim_UE_adapter
git clone https://github.com/love-of-city/space_sim_server.git space_arm_data_platform

git -C .\space_sim_UE_adapter lfs pull
git -C .\space_arm_data_platform lfs pull

Set-Location .\space_arm_data_platform
python -m pip install -e ".[test]"
```

还需要 PowerShell 7、Node.js/npm、Unreal Engine 5.6、Visual Studio 2022 C++/Windows SDK，以及包含 Basilisk/MJScene 的 Conda 环境 `mujoco-dev`。启动脚本会自动检查这些命令、Web 后端依赖和 Git LFS 模型。UE 会从 `UE56_ROOT`、`E:\UE5.6` 和 Epic 默认安装目录查找；其他路径可传入 `-UnrealRoot`。

## 一条命令运行

```powershell
Set-Location .\space_arm_data_platform
pwsh -NoProfile -ExecutionPolicy Bypass -File '.\scripts\run_platform.ps1'
```

默认会查找同级 UE 适配器仓库，也兼容服务端和适配器各多套一层同名目录的布局；其他布局可显式增加 `-AdapterRoot 'D:\path\to\space_sim_UE_adapter'`。默认 `ModelRoot` 是本仓库的 `model\SARM\platform`，无需另行复制外部模型；实际 XML 按场景模板选择，当前为粗碰撞体＋内部接触组合版（见下文）。也可用 `-ModelRoot` 指定兼容的模型目录。PowerShell 入口仍保留旧工作区模型和适配器内 CubeSat + SO-101 的查找回退。模型目录与本地兼容路径说明见 `model/README.md`。

首次运行会使用 UE 5.6 自带脚本准备官方 Pixel Streaming Infrastructure 并安装其 Node 依赖，同时根据当前模型路径生成本机 UE 资源映射。首次克隆、模型位置或网格内容变化时，资源准备脚本可能重新导入网格并应用构建设置，因此准备时间取决于本机缓存。地球和银河 `.uasset` 通过 Git LFS 随适配器仓库提供，启动脚本会检查它们是否完整。不会安装另一套 UE。网页地址为 `http://127.0.0.1:8000`。

`run_platform.ps1` 现在只启动控制平台（前端、后端、仿真控制/采集监听和 Pixel Streaming 信令），不会立即启动 UE 或 Basilisk/MJScene。进入网页后，在“场景实例”中选择模板、随机化配置、Seed 和是否启用权威采集，再点击“生成并启动场景”。场景会持续运行，直到用户主动点击“停止场景”或关闭平台。Seed 留空时由后端生成，并与完整随机参数一起保存到 `run/scenes/<instance-id>.json`，可用于复现实验。

初始化可勾选“沿当前圆轨道随机初始位置”（默认关闭）。开启后沿当前 500 km、倾角 51.6° 的圆轨道均匀抽取 0～360° 的起点，不改变轨道形状/平面；关闭时保留原来的 180° 起点。该开关独立于 `none` / `training-v1` 抓取随机化，同一 Seed 的目标局部位姿和关节初态不会因开关变化而改变。实际起点显示在实例摘要中，并以 `randomize_orbit_phase` 和 `environment.orbit.true_anomaly_deg` 保存。卫星与目标使用该相位对应的位置及速度，UE 继续读取同一动力学状态；刷新网页或切换相机不会重新抽样。详见[轨道起点初始化](docs/ORBIT_INITIALIZATION.md)。

设置 IK 频率和预览帧率（交互预览默认关闭高开销的权威数据采集）：

```powershell
.\scripts\run_platform.ps1 -IkRate 100 -PreviewRate 60 -SimulationRate 1
```

需要录制训练数据时，可用以下参数让前端“权威采集”默认勾选，并用 `-CaptureRate` 指定采集率；也可以在每次启动场景前直接在网页中勾选：

```powershell
.\scripts\run_platform.ps1 -EnableDatasetCapture -CaptureRate 10
```

权威采集会在 UE 游戏线程执行 SceneCapture、GPU→CPU 读回和编码，尤其实例分割还会按对象重复捕获，因此不应在只做交互预览时开启。

当前默认场景为 **SARM + 地面验证星（粗碰撞体·内部碰撞）**（`sarm-ground-validation-self-collision-grasp`），加载 `model/SARM/platform/sarm_ground_target_self_collision.xml`。保留原三个外部接触粗盒，新增两个内部接触代理，开启外侧板与本体/内侧板接触；不使用高精度三角面。铰链附近采用明确记录的近似间隙，不是工程限位或精准 CAD 碰撞。旧无内部碰撞模板、高精度实验及已保存实例不改写；需创建新模板实例生效。详见[粗碰撞内部接触](docs/COARSE_SELF_COLLISION.md)及[待抓取目标说明](docs/GROUND_CAPTURE_TARGET.md)。

当前组合场景渲染使用113个网格（原 SARM 9个 + 目标104个）。资源准备脚本检查资源映射指纹及 `.uasset` 是否存在；需要时从服务端模型重新导入，后续复用缓存。`Saved/AssetImport` 是本机生成文件，不随仓库提交。需要强制重新导入时：

```powershell
.\scripts\run_platform.ps1 -ReimportAssets
```

停止：

```powershell
.\scripts\stop_platform.ps1
```

本机默认端口：操作台 `8000`、Pixel Streaming 播放器 `8080`、UE 信令 `8888`。当前脚本面向本机 HTTP 使用；跨机器或公网部署时必须另外配置可访问的公网地址、HTTPS 以及 STUN/TURN。


## 登录与用户权限

平台现在要求登录后才能访问控制 API、启动/关闭场景、操作机械臂和采集训练数据。仅保留两种角色：

- `admin`：拥有操作员全部能力，并可新增、删除操作员及重置操作员密码。
- `operator`：可创建和关闭自己的场景、操作机械臂、开始和结束训练数据采集。

首次创建认证数据库时会建立管理员。默认本机开发凭据为：

```text
用户名：admin
密码：ChangeMe123!
```

推荐启动时显式设置管理员凭据：

```powershell
.\scripts\run_platform.ps1 `
  -AdapterRoot 'C:\path\to\space_sim_UE_Adapter' `
  -AdminUsername 'admin' `
  -AdminPassword 'replace-with-a-strong-password'
```

管理员凭据只在认证数据库第一次创建管理员时使用；登录后可从页面右上角修改自己的密码。用户和会话持久保存在 `data/auth.sqlite3`。同一操作员打开多个页面时不需要申请控制权：点击实时画面的页面会自动成为当前操作页面，旧页面立即归零并退出操作。

## 推荐部署：本地浏览器操作远程仿真机

新增可配置的 HTTPS/WSS 同源部署入口，详见 [Windows 部署指南](docs/DEPLOYMENT.md)。

- 配置模板：`deploy/deployment.example.json`，复制为 `deployment.local.json` 后填写真实主机名与 TLS 方式。
- 默认只校验：`scripts/deploy_platform.ps1 -ValidateOnly`；切换运行环境必须显式指定 `-Start`，会停止旧平台。
- 使用现有 Windows + UE/Basilisk 环境，不迁移模型；手柄接在本地浏览器所在电脑。
- 部署模式启用 Secure Cookie、Origin 白名单和登录限流，内部服务只监听 loopback；视频直连/TURN 仍需实际网络配置。
- 手柄诊断可显示设备、轴/按钮与阻断原因。首次接入需回中；非标准映射不自动发送机械臂动作。

## 跨机器安全模式（底层参数）

局域网或公网部署时显式启用 JWT 信令；本机模式默认不增加鉴权复杂度：

```powershell
.\scripts\run_platform.ps1 -RemoteAccess -PublicHost 192.168.1.50 -AdminPassword $env:SPACE_SIM_ADMIN_PASSWORD
```

脚本会生成本次运行的访问密钥和 JWT 密钥，并输出带 `access_key` 的操作台地址。安全信令在 WebSocket 握手时验证短期 JWT，token 只允许访问声明的 Streamer ID，订阅时会再次校验。
底层 `-RemoteAccess` 默认让 API 和播放器端口监听外部网卡（不等同于完整 HTTPS 部署）；推荐部署脚本改为 loopback + 代理。UE Streamer 端口只绑定 `127.0.0.1`，避免外部进程冒充同名 UE Streamer。

跨 NAT 时可以注入 STUN/TURN：

```powershell
.\scripts\run_platform.ps1 -RemoteAccess -PublicHost simulator.example.com -AdminPassword $env:SPACE_SIM_ADMIN_PASSWORD `
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
| Y 左右 | D | A |
| Z 上下 | E | Q |

按住Shift时控制末端旋转：

| 运动 | 负方向 | 正方向 |
|---|---:|---:|
| Roll | Shift+E | Shift+Q |
| Pitch | Shift+S | Shift+W |
| Yaw | Shift+D | Shift+A |

夹爪始终使用 `F/R` 进行闭合/张开。

UE Pixel Streaming 主视口支持观察模式：按 `C` 进入/退出自由视角，使用
`W/S`、`A/D`、`Q/E` 和鼠标飞行观察（默认 1 m/s，`Shift` 加速到 5 m/s），按
`Home` 或 `Esc` 返回主视角。上下、左右均支持无边界连续旋转和翻转；优先原始相对输入，仅在浏览器明确不支持时回退兼容输入；进入后请确认显示“鼠标已锁定”，否则点击视频画面重试。自由视角只影响
玩家画面，不会改变 BSK/MJScene 动力学或权威相机采集。输入链路、失焦保护与验证见
[全局自由相机说明](docs/FREE_CAMERA_INPUT.md)。

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

网页 WebRTC 视频只用于操作预览；训练数据直接保存 UE 权威帧的原始产品，不从网页视频或截图反推。每个场景实例启动前必须在网页勾选权威采集，或在启动平台时传入 `-EnableDatasetCapture` 将其设为默认值，才会生成这些权威产品。Episode 元数据会同时记录本次场景实例、Seed 和完整随机参数。`steps.jsonl` 中记录 `step_id` 和 `render_frame_id`；`captures.jsonl` 记录匹配的 `source_frame_id`、`sim_time_ns` 及 `authoritative_state=true`。停止 episode 时，`metadata.json` 会给出匹配、待匹配和拒绝数量。

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
3. 扩展更多场景模板、随机变量约束和批量无人值守训练调度。
