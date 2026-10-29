# 太空机械臂遥操作与数据采集平台

团队开发请先阅读[贡献指南](CONTRIBUTING.md)。基础 CI 不替代完整仿真、UE 和采集验收。

浏览器操作台、用户与场景管理、训练数据记录。Basilisk 在轨组件与本地 MuJoCo 负责权威动力学，[UE 适配器](../space_sim_UE_Adapter/README.md)负责渲染。WebRTC 用于操作预览；训练图像通过独立的 `bsk-capture/1` 通道采集。

默认 `local` 场景采用 BSK 引力模型逐刚体计算差分力，保留 MuJoCo 刚体/接触/伺服与局部坐标。扩展引力源、球谐模型和子步采样的方法见 [BSK 在轨组件](docs/BSK_ORBITAL_COMPONENTS.md)。旧线性潮汐仅通过 `SPACE_SIM_ORBITAL_MODE=linear_tidal` 显式选择用于回归。

新建场景默认保存地球 **J2 + 太阳点质量** 配置；旧实例缺少 `environment.physics` 时仍按点质量重放。逐物体大气阻力需要明确提供密度、面积、Cd 及参数来源，默认关闭。配置、物理边界和验收方式见 [轨道物理配置](docs/ORBITAL_PHYSICS_CONFIGURATION.md)。

## 卫星交会接入

现有工作台提供直接机械臂操作、7 km 交会、300 km 入轨后三种起始条件。连续任务使用共享 GNSS 配置；远距通过验收后接续近距，近距通过后以实际关节状态进入原操作场景，任务编号保持一致。独立 110 m 初态演示仍可通过诊断 API/CLI 使用，不作为第四个工作台入口。原模型的近距双目为参数化观测，捕获为逻辑代理，不代表真实图像闭环或接触力学验证。接口和验证边界见[连续任务整合说明](../archive/2026_10_17_连续任务与共享导航.md)及[近距方案](docs/NEAR_FIELD_RENDEZVOUS.md)。

capture26 正式配置采用 `simulation_state_demo_v1`：读取权威仿真状态，经原制导、PD、关节与飞轮控制产生动力学运动。约 110 m 远距停留 5 s 后，沿质心 V-bar 依次在 50 m、光学深度 17.4 m、5.50 m 和收拢 TCP 轴向间隙 0.5 m 的作业参考处实际连续停留 5 s，最终持续保持并接续原操作工作台。位置/速度按当前参考判断，超限清零；停泊最高 1 倍节拍，航行允许加速。

GNSS、双目、双星陀螺及导航滤波继续运行与记录，图像计数、协方差、传感器健康、姿态精度、角速度阈值和几何证书不限制本轮运动演示。服务星保留角速度阻尼，目标星被动演化；推力、关节和轮系保护及非计划碰撞处理保留。新版 `/3` 交接绑定演示规则、冻结几何和有序停泊记录，旧结果不能自动通过。实际连续任务验收进度和几何定义见[仿真状态停泊演示](docs/CAPTURE26_MOTION_DEMO.md)；[图像导航验证](docs/CAPTURE26_IMAGE_NAVIGATION.md)继续单独记录，`integration_complete=false`。



原工作台高级参数支持0—0.25 s GNSS通信时延，默认0。观测按物理采集时刻标记，交付后预测到当前控制时刻；跨阶段保留在途包、采样序号和联合不确定性。非零时延的远距阶段增加原始GNSS观测下载。模型范围、物理采样时钟修正及复验记录见[GNSS时延说明](../archive/2026_10_19_GNSS时延与物理采样时钟.md)。

个人全部关联任务、会议分工、算法输入输出及协作所需数据见[任务与集成依据](docs/AOC_SEVEN_TASKS.md)。

三个起始条件始终共用“末端六维遥操作”工作台。控制、关节角、采集、场景详情与交会遥测固定保留；自动交会期间控制入口禁用，进入操作阶段后按实际就绪状态开放。不适用的初始参数显示为禁用，避免把未使用的参数误认为本次配置。

切换阶段、重置或断线会清理上一场景的末端、IK、轮系与动作读数。原仿真状态区在交会时显示当前任务的时间和本体角速度；未提供的机械臂遥测显示为“—”，不以历史值冒充实时值。

登录原控制台，在“任务与场景”选择交会模式，使用默认 **SARM＋地面验证星** 运行 7 km 接近或约 300→500 km 升轨后接近，在 100 m 外保持。交会使用团队 LocalMujocoStepper、Basilisk 在轨组件与原生飞轮，包含大气阻力、太阳光压、双星 GNSS 误差和未知轨道扰动。创建者及管理员可配置、启停任务，在原主视窗查看 UE 画面、下载结果，并在同一页面接续遥操作与采集。任务编号贯穿交会、场景和数据集；旧 `/static/rendezvous.html` 链接自动进入控制台。模型、工程参数和验证标准见 [团队双星交会](docs/TEAM_RENDEZVOUS.md)。

后端默认使用自身 Python（需具备 Basilisk 及渲染适配器），也可通过 `SPACE_SIM_RENDEZVOUS_PYTHON` 指定仿真解释器。在本机 8768 端口接收只读 `space-sim-state/1`；UE 接收端监听 5558。团队模型任务不依赖旧示例仓库。需要历史通用模型示例时，再设置 `SPACE_SIM_RENDEZVOUS_REPO` 指向 `basilisk-rendezvous-demo`，并使用安装其依赖的 Python。
插拔件使用静摩擦保持与恒定滑动阻力，复用现有结构及孔位；默认配置与动力学验证见 [插拔件保持力](docs/PLUG_RETENTION.md)。

原操作场景提供 50 Hz 服务星模拟陀螺角速度观测，包含白噪声、偏置与饱和标志，随原观测通道录制。配置、时间与坐标定义及接入边界见[服务星陀螺反馈](docs/SERVICE_GYROSCOPE.md)。

## 环境要求

完整运行入口支持 Windows x64 + PowerShell 7。需要 Git/Git LFS、Node.js 22 LTS（含 npm）、Unreal Engine 5.6、Visual Studio 2022 C++ 游戏开发工作负载及 Windows SDK，以及支持 UE 渲染和 H.264 编码的 GPU/驱动。

腕部相机支持彩色、160° 鱼眼和黑白，创建场景前选择；使用与模型输入说明见 [腕部相机类型](docs/WRIST_CAMERA_MODES.md)。

Linux 原生一键启动/停止及网页场景管理见 [Linux 一键部署](docs/LINUX_DEPLOYMENT.md)。已有环境可在本目录执行 `./start_deployment.sh`，停止执行 `./stop_deployment.sh`。Linux 环境、原生库和真实采集验证步骤见 [Linux 基础迁移](docs/LINUX_BASIC_MIGRATION.md)。

脚本批量采集可通过 [多进程采集](docs/PARALLEL_COLLECTION.md) 配置两路隔离并发、每路 UE 复用、离线渲染与资源统计；调度配置位于 `config/collection.json`。

自动采集可从已校验布局池逐条随机选择操作台各安装单元的位置和朝向，命令行及网页配置见[布局随机化采集](docs/COLLECTION_LAYOUT_RANDOMIZATION.md)。

全局相机可逐条从 5 个固定机位随机初始化，整段保持固定，并保存可复现的相机位姿；网页、API 和采集脚本使用方式见[全局相机随机初始化](docs/OVERVIEW_CAMERA_RANDOMIZATION.md)。

Python 要求 3.11+，版本必须与安装的 Basilisk 二进制匹配。真实仿真必须安装**包含 MJScene 及项目所需控制、积分和消息 API 的 Basilisk**；普通 Python `mujoco` 包不能替代 `Basilisk.simulation.mujoco`。可用 `uv pip install --python $env:SPACE_SIM_PYTHON "bsk[all]"` 安装发行包，也可参考其[安装文档](https://hanspeterschaub.info/basilisk/Install.html)自行编译；不限定源码版本或安装方式。安装后运行 `& $env:SPACE_SIM_PYTHON scripts/check_basilisk.py` 检查功能。以下项目安装命令不会安装 UE、编译器或 Basilisk。

## 1. 获取仓库

在选定的工作区父目录执行；已有仓库跳过 clone。

```powershell
git clone https://github.com/love-of-city/space_sim_server.git
Set-Location .\space_sim_server
git lfs install --local
git lfs pull
Set-Location .\space_sim_server
```

最后一步进入统一仓库中的 **space_sim_server 服务端子目录**。后续命令均从该目录执行。适配器位于同级 ../space_sim_UE_Adapter/，SARM 模型位于本子目录 model/ 中。

## 2. Python 环境与安装

三种方式均受支持，按现有环境选择 **uv、Conda 或传统 venv/pip 中的一种**。不要在已有仿真环境上重新创建环境。下列新建命令仅供首次配置，Python 版本示例为 3.13，必须按 Basilisk 二进制版本调整。

建议在新的 PowerShell 7 终端操作。切换环境时重新设置 `SPACE_SIM_PYTHON`；该变量优先于已激活环境，避免继续使用上一次选中的解释器。

### A. uv

已有仓库或父工作区的 `.venv`/`venv` 可以自动发现；其他位置可将 `SPACE_SIM_PYTHON` 直接设为目标 `python.exe` 的绝对路径。首次新建环境（已有环境跳过）：

```powershell
uv venv .venv --python 3.13
```

```powershell
. .\scripts\python_runtime.ps1
$env:SPACE_SIM_PYTHON = Resolve-SpaceSimPython -RepositoryRoot $PWD.Path
uv pip install --python $env:SPACE_SIM_PYTHON -e "../space_sim_UE_Adapter[test]" -e ".[test,simulation]"
```

uv 环境没有 pip 是正常现象；使用 `uv pip --python` 指定环境，无需激活，也不要为此运行 `ensurepip`。本项目尚无覆盖全部原生依赖的 `uv sync` 工作流。

### B. Conda

在支持 `conda activate` 的 PowerShell 7 终端执行。若 shell 尚未初始化，可先运行 `conda init powershell`，然后重新打开终端。环境名 `space-sim` 只是示例，替换为自己的环境名。

首次新建（已有含 Basilisk 的环境跳过）：

```powershell
conda create -n space-sim python=3.13 pip
```

激活、明确选择解释器并安装：

```powershell
conda activate space-sim
$env:SPACE_SIM_PYTHON = Join-Path $env:CONDA_PREFIX 'python.exe'
& $env:SPACE_SIM_PYTHON -m pip install -e "../space_sim_UE_Adapter[test]" -e ".[test,simulation]"
```

如果已有 Conda 环境缺少 pip，在该环境激活后用 `conda install pip` 安装。Conda 管理解释器及原生依赖，项目的 editable 包通过该解释器的 pip 安装；无需安装 uv。

### C. 传统 venv / pip

首次新建（已有环境跳过；`py -3.13` 需要对应版本的 Python Launcher/解释器）：

```powershell
py -3.13 -m venv .venv
```

已有环境位于父工作区时，将下面的 `.\.venv` 改为 `..\.venv`，或使用实际路径：

```powershell
$env:SPACE_SIM_PYTHON = (Resolve-Path .\.venv\Scripts\python.exe).Path
& $env:SPACE_SIM_PYTHON -m pip install -e "../space_sim_UE_Adapter[test]" -e ".[test,simulation]"
```

也可先执行 `.\.venv\Scripts\Activate.ps1` 激活。直接调用解释器无需激活，适用于激活脚本受执行策略限制的情况。若传统 pip 管理的 venv 确实缺少 pip，可用 `& $env:SPACE_SIM_PYTHON -m ensurepip --upgrade` 修复；这一步不用于 uv 管理的环境。

### 共同检查与后续步骤

新建任何一种环境都不会自动安装 Basilisk/MJScene。真实仿真需先按团队的 Basilisk 安装说明准备匹配的原生模块，再检查依赖：

```powershell
& $env:SPACE_SIM_PYTHON -c "import sys,numpy,fastapi,pydantic,uvicorn,bsk_render_adapter; from Basilisk.simulation import mujoco; print(sys.executable); print(mujoco.MJScene)"
```

启动脚本只调用解释器，不要求特定环境管理器。统一优先级：`-Python` → `SPACE_SIM_PYTHON` → 仓库本地 `.space-sim-python` → 已激活 venv（`VIRTUAL_ENV`）→ 已激活 Conda（`CONDA_PREFIX`）→ 仓库及父工作区 `.venv`/`venv` → PATH。选中环境无效或缺少所需模块时直接报错，不自动切换。三种方式后续均使用同一套测试和启动命令；UE 资产导入仍使用编辑器内置 Python。固定解释器及 Conda DLL 加载说明见 [Python 环境选择](docs/PYTHON_RUNTIME.md)。

## 3. 前端安装与验证

每条命令成功后再执行下一条：

```powershell
npm.cmd --prefix frontend ci --no-audit --no-fund
npm.cmd --prefix signalling ci --no-audit --no-fund
& $env:SPACE_SIM_PYTHON -m pytest
npm.cmd --prefix frontend test
npm.cmd --prefix frontend run build
npm.cmd --prefix signalling test
npm.cmd --prefix signalling run check
```

CAD 转换等可选测试可能因缺少专用依赖而 skip；skip 不代表功能验收通过。UE 构建与接收端测试见适配器 README。

## 4. 启动与停止

双臂场景的三路相机及渲染资产目录统一配置在 `config/scene_render_profiles.json`。
平台启动会注册所需相机，场景启动会补齐遗漏的相机参数；无需在启动命令中重复填写。
双臂场景普通启动不自动抓取。保存场景中的 `randomization.capture_window_trial=true`
仅用于明确启用的自动抓取测试；复用旧测试场景时须将其设为 `false` 才进入手动模式。
资产导入检查会验证每个可见网格都有映射，避免只检查目录内文件而遗漏腕部零件。

视觉＋陀螺尾段自动抓取的独立复现入口为 `scripts/run_visual_gyro_capture.ps1`。
它包含可移植成功工况、视觉观测输出和完整验证流程，使用隔离端口并保留失败证据；
参数与验收边界见[完整复现说明](docs/VISUAL_GYRO_CAPTURE_REPRODUCTION.md)。普通启动仍不自动抓取。

指定本机包含 `Engine` 的目录，下例按实际安装位置修改。环境变量仅影响当前终端及其子进程。

```powershell
$env:UE56_ROOT = 'D:\UE\UE_5.6'
pwsh -NoProfile -File .\scripts\run_platform.ps1 -ApiPort 18000
```

首次启动准备 Pixel Streaming、构建缺少的 UE 模块，并按本机路径生成资产映射。需要访问 GitHub/npm；不要复制别人 `Saved/AssetImport` 中的本机 catalog。

打开 `http://127.0.0.1:18000`，登录后选择模板并点击“生成并启动场景”。平台启动成功不等于场景或视频已经就绪。全新认证数据库默认本机账号为 `admin` / `ChangeMe123!`；已有数据库使用原密码，登录后可修改。需要训练图像时先勾选“权威采集”，场景就绪后再开始 episode；只预览时保持关闭。

### 拉取代码后的 UE 运行时构建

`run_platform.ps1` 不会在每次启动时重新链接 `BskUnrealRuntime`。如果拉取更新同时修改了适配器的 `Plugins/BskUnrealRuntime/Source/`（尤其是 `.cpp`、`.h` 或 `.cs`），必须在启动平台前执行一次完整的 UE 构建。

```powershell
# 当前目录为统一仓库中的 space_sim_server 服务端子目录
$env:UE56_ROOT = 'D:\UE\UE_5.6'
pwsh -NoProfile -File ..\space_sim_UE_Adapter\Unreal\BskUnrealRenderer\scripts\build.ps1 `
  -UnrealRoot $env:UE56_ROOT
```

适配器运行时源码发生变化、运行时 DLL 缺失，或场景启动报 `Capture runtime DLL is stale` 时需要重新执行。构建成功后再运行 `run_platform.ps1`，并重新生成场景。该步骤要求 UE 5.6、Visual Studio C++ 游戏开发工作负载和 Windows SDK 可用。


### 指定操作姿态直接启动

默认新场景使用兼容标识 `teleop-zero-prepare-v1`，但六个旋转关节会在初始化时直接设置为指定的操作角度；不再从 0° 运动到操作姿态，场景启动后即可遥操作。
默认操作角度为 **[0, -67.6, -86.6, 143.2, -85.5, 0]°**，可在页面编辑。
旧场景文件若仍带有 `arm_preparation_required=true`，继续按旧版准备流程加载，以保证历史场景可复现。

### 机械臂初始关节角度（旧配置兼容）

在“场景实例”中勾选“自定义机械臂初始角度（J1～J6）”，分别输入六个关节相对于模型零位的角度，单位为 **度（°）**，然后点击“生成并启动场景”。每个输入框下方显示所选模型的有效限位；“填入当前配置基准角度”可填入当前随机化配置的基准姿态，再按需修改。

- 不勾选时保留原有基准 / 随机初始化行为。
- 勾选时六个角度精确覆盖机械臂随机初始角度，不改变夹爪开度、目标随机化、轨道或随机种子流；不自动把角度折返到 ±180°。
- 场景运行期间不可修改初始化输入。“重置状态”恢复本次保存的初始姿态；要更换角度，请先停止场景，再修改并重新生成。
- 前后端均校验输入；超限、空值或非有限数值会被拒绝。位置限位校验不等于碰撞检测，请避免自碰撞或与目标相交的初始姿态。

`POST /api/scenes/instances` 和 `POST /api/scenes/start` 支持可选字段：

```json
{"initial_arm_joint_position_deg": [-45, -20, 25, -90, -50, 275]}
```

按 J1～J6 顺序传入六个数值；省略或传 `null` 使用原行为。实例保存这组输入，并将其转换为仿真使用的 `randomization.arm_joint_position_rad` 前六项；后两项仍是以米计的夹爪位置。旧场景文件无需迁移。

检查 API：

```powershell
Invoke-RestMethod http://127.0.0.1:18000/api/health
```

结束场景使用网页按钮，结束整个平台执行：

```powershell
pwsh -NoProfile -File .\scripts\stop_platform.ps1
```

默认其他端口：播放器 8080、UE 信令 8888、控制 8766、采集 8767、渲染状态 5558。占用时查看进程或调整脚本参数。启动会停止本项目记录的旧平台/场景。

## 机械臂控制与视频配置

- 机械臂默认恢复 `ik_pose` 单步阻尼 IK，`strict` 保留作对照；已移除定制分阶段 QP 与额外纠偏/制动门槛。设置 `$env:SPACE_SIM_IK_MODE` 选择模式，重新启动仿真进程后生效。
- 输入仍经过速度限制、deadman 和超时保护，真实运动由 Basilisk/MJScene 的关节控制器执行。实测误差与速度提示只作诊断，不干预运动。参见[简化遥操作](docs/teleop_control.md)和[末端笛卡尔 IK 模式](docs/CARTESIAN_IK_MODES.md)。
- 操作预览默认目标为 **90 FPS**，可用 `run_platform.ps1 -PreviewRate 60` 调整；默认 `-EncoderMinQuality 60` 设置 H.264 编码质量下限（范围 0～100），不固定 WebRTC 码率。部署配置对应 `preview_fps` 和 `encoder_min_quality`。
- 目标帧率不代表实测帧率；前端显示接收、显示帧率和平均 QP，UE 按订阅情况安排预览工作。权威训练采集的频率与预览独立，详见[视频帧率与质量](docs/VIDEO_FRAME_RATE.md)。

## 操作、数据与协作

- 平移 W/S、A/D、Q/E；Shift 配合上述按键旋转；F/R 闭合/张开夹爪。
- Esc 归零并退出操作；页面急停锁存停止。C 切换自由相机，Home/Esc 返回。
- 手柄需正常识别并回中，映射与诊断见操作台。
- 右摇杆按夹爪自身方向俯仰/偏航，A/B 直接控制第六关节反转/正转；左杆与扳机仍在卫星本体系平移。详见[手柄末端相对操作](docs/GAMEPAD_TOOL_CONTROL.md)。
- `data/auth.sqlite3` 保存账号，`data/episodes/` 保存数据，`data/archives/` 保存归档。
- `logs/` 为日志，`run/scenes/` 为可复现实例；PID、资源映射和虚拟环境均为本机文件。
- 不提交密钥、账号数据库、日志和录制；模型与二进制资产使用 LFS。改动后执行相应测试。

## 参考文档

公网部署是可选模式，见[临时公网发布](docs/PUBLIC_DEPLOYMENT.md)和[固定域名部署](docs/DEPLOYMENT.md)；需要实际确认 HTTPS、鉴权和 TURN/视频连通。

- [系统架构](docs/SYSTEM_ARCHITECTURE.md) / [仿真扩展](docs/SIMULATION_ARCHITECTURE.md)
- [模型目录](model/README.md) / [目标与碰撞限制](docs/GROUND_CAPTURE_TARGET.md)
- [状态重置](docs/SCENE_RESET.md) / [自由相机](docs/FREE_CAMERA_INPUT.md)
- [姿态控制](docs/ATTITUDE_CONTROL.md) / [轨道初始化](docs/ORBIT_INITIALIZATION.md) / [太阳光照](docs/SUNLIGHT_CONFIGURATION.md)

## LeRobot v3 原生采集（本分支）

采集与录制默认设置统一在 [`config/recording.json`](config/recording.json)：
相机选择、采样率、最大帧数、原始图片副本和视频编码。修改后重启后端，
采样率修改需重新创建场景。参数含义与覆盖规则见 [采集配置](docs/LEROBOT_V3_CAPTURE.md)。

新的录制通过官方 LeRobot writer 直接生成 v3 数据集，不需要事后转换。
默认动力学 400 Hz、在线 IK 100 Hz（每 4 个物理步）、双相机 RGB 1024×1024、50 FPS（每 8 步）。新录制图像与视频为正方形，历史 episode 保留其原始尺寸；详见 [正方形 1K 采集](docs/SQUARE_1K_CAPTURE.md)。
使用绝对步序号的纳秒调度避免周期取整漂移；不采集深度和分割。
参见 [采集链路、特征语义、同步规则和验证](docs/LEROBOT_V3_CAPTURE.md)。


### 简化机械臂控制与速度诊断

默认 IK 恢复 `ik_pose`；移除分阶段 QP、双重目标领先门槛、额外加速度/制动切换、
持续末端纠偏。未取消模型关节限位和原有安全保护，没有引入 LeRobot/Placo。
`strict` 仍可通过 `--ik-mode` 或 `SPACE_SIM_IK_MODE` 选择。
实时遥操作不再启用回初始关节构型的零空间项；机械臂与夹爪按各自输入启用，
只动夹爪或无机械臂输入时跳过 IK，保持机械臂参考不变。80% 诊断仍只监测、不干预。

控制面板分别显示平移/旋转沿指令方向的实测速度达成率，持续低于预期 80%
时显示限位、阻尼残差、跟踪或力矩饱和等诊断证据；提示不参与控制。
更新后需重新启动场景并刷新页面；如环境中仍有 `SPACE_SIM_IK_MODE=constrained`，
需清除或改为 `ik_pose`。配置和独立验证方法见[简化遥操作说明](docs/teleop_control.md)。


## 肘部抬高优先 IK

可显式选择旧配置 `teleop-elbow-up-v1`：先离线多初值求解同一末端位姿，按本体系肘部高度
软偏好选取通过精度、限位、静态接触检查的解，再保存为初始关节姿态。
实时 `ik_pose` 现叠加有界几何肘高偏好：即使手动全零初态，也会在有效输入时主动考虑抬肘，
不靠初始化、不回 home、不跳解。`strict` 保持无偏好，`SPACE_SIM_ONLINE_ELBOW_MODE=off`
可恢复原 task-only 行为。显式初始角度、历史配置和保存的初始状态不变。
无合格解/离线依赖不可用时保留原姿态并记录原因。详见 [配置与验证](docs/ELBOW_PREFERRED_IK.md)。

在线构型偏好现同时考虑抬肘和腕部向下：在本体系中偏好 J6 低于 J4（默认高度差 5 cm）。
两项在一个二次目标中求修正，共享原有末端偏差预算；不是先后叠加两次修正，也不是硬性拱形约束。
`SPACE_SIM_ONLINE_WRIST_MODE=off` 可单独关闭腕部项。前端 IK 行显示实测 `J4−J6` 高度差。

在线偏好还包括 joint3 负角软目标（默认 q3≤−5°，达标后不继续推动）。角度先归一化并
转换为长度等效误差，与抬肘/腕部项共同求解并共享偏差预算。模型机械限位、正角初态及旧场景不变。
`SPACE_SIM_ONLINE_JOINT3_MODE=off` 可单独关闭此项。是否真正转入负角仍取决于任务与剩余预算。

## 自动任务试验

[自动抽屉提拉脚本](docs/SCRIPTED_DRAWER_TASK.md)可以在独立的 BSK/MuJoCo 动力学实例中，完成到位、转向、张开、下降、夹持和提拉，并输出实际抽拉位移与腕部载荷。`run_drawer_task.py` 提供无界面动力学验收；`record_drawer_task.py` 可启动当前场景配置的独立 UE 实例，通过现有数据链路录制双相机、关节及腕部载荷，输出与手动录制一致的 LeRobot v3 数据集。

[圆柱插头自动搬运与插入](docs/SCRIPTED_PLUG_TASK.md)使用同一套动力学和录制入口，从初始取件位夹取圆柱插头，搬到前方装配孔完成插入、松爪与撤离；目标孔与任务参数可独立配置。

当前 SARM 训练录制使用六个关节角 + 一个连续 0–1 夹爪开合量（7 维），保留原始八维诊断。格式与兼容约定见 [夹爪训练数据](docs/GRIPPER_TRAINING_REPRESENTATION.md)。


## 自动采集与数据集回放

登录后，左侧统一导航提供“仿真工作台”“自动采集”“数据集”，管理员还可访问“用户管理”。各页共用平台浅色界面，在同一标签页内切换。自动采集可分别选择抽屉完整循环、航空插头拿取插入、导向插头拿取插入或圆柱插头拿取插入，并配置 1–4 路并发、随机种子和关节初始偏移，分别查看当前与历史批次。数据集先显示可筛选的记录列表，再进入双相机视频与分组曲线详情；沿用 50 Hz 主数据、200 Hz 独立六维力和现有 LeRobot 数据格式。

用户管理支持新增账号、修改显示名称与角色、启用/停用和重置其他用户密码。已有账号库启动时自动增加显示名称、最近登录时间字段，保留用户 ID 和密码。停用或变更角色会撤销该用户的登录会话；操作员仍只能访问自己的批次与数据。

详细用法、权限、任务扩展接口及限制见 [平台自动采集与数据集回放](docs/PLATFORM_AUTOMATED_COLLECTION.md)。前端源码更新后先执行 `npm --prefix frontend run build`（在本目录），再重启后端并刷新页面。四个页面共用 Vite 构建，不再单独服务 public 下的旧查看器。

模型策略接入：参见 [Enactive / SARM 在线接入](docs/ENACTIVE_SARM_INTEGRATION.md)，包含同步关节目标控制、双相机观测和正式录制流程。
