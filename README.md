# 太空机械臂遥操作与数据采集平台

团队开发请先阅读[贡献指南](CONTRIBUTING.md)。基础 CI 不替代完整仿真、UE 和采集验收。

浏览器操作台、用户与场景管理、训练数据记录。Basilisk/MJScene 负责权威动力学，[UE 适配器](https://github.com/love-of-city/space_sim_UE_Adapter)负责渲染。WebRTC 用于操作预览；训练图像通过独立的 `bsk-capture/1` 通道采集。

## 环境要求

完整运行入口支持 Windows x64 + PowerShell 7。需要 Git/Git LFS、Node.js 22 LTS（含 npm）、Unreal Engine 5.6、Visual Studio 2022 C++ 游戏开发工作负载及 Windows SDK，以及支持 UE 渲染和 H.264 编码的 GPU/驱动。

Python 要求 3.11+，版本必须与安装的 Basilisk 二进制匹配。真实仿真必须安装**包含 MJScene 及项目所需控制、积分和消息 API 的 Basilisk**；普通 Python `mujoco` 包不能替代 `Basilisk.simulation.mujoco`。可用 `uv pip install --python $env:SPACE_SIM_PYTHON "bsk[all]"` 安装发行包，也可参考其[安装文档](https://hanspeterschaub.info/basilisk/Install.html)自行编译；不限定源码版本或安装方式。安装后运行 `& $env:SPACE_SIM_PYTHON scripts/check_basilisk.py` 检查功能。以下项目安装命令不会安装 UE、编译器或 Basilisk。

## 1. 获取仓库

在选定的工作区父目录执行；已有仓库跳过 clone。

```powershell
git clone https://github.com/love-of-city/space_sim_UE_Adapter.git
git clone https://github.com/love-of-city/space_sim_server.git
git -C .\space_sim_UE_Adapter lfs install --local
git -C .\space_sim_UE_Adapter lfs pull
git -C .\space_sim_server lfs install --local
git -C .\space_sim_server lfs pull
Set-Location .\space_sim_server
```

后续命令均从 **space_sim_server 仓库根目录**执行，不要再次进入同名子目录。两个仓库应同级；SARM 模型已经包含在本仓库 `model/` 中。

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

启动脚本只调用解释器，不要求特定环境管理器。统一优先级：`-Python` → `SPACE_SIM_PYTHON` → 已激活 venv（`VIRTUAL_ENV`）→ 已激活 Conda（`CONDA_PREFIX`）→ 仓库及父工作区 `.venv`/`venv` → PATH。选中环境无效或缺少所需模块时直接报错，不自动切换。三种方式后续均使用同一套测试和启动命令；UE 资产导入仍使用编辑器内置 Python。

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
# 当前目录为 space_sim_server 仓库根目录
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

新的录制通过官方 LeRobot writer 直接生成 v3 数据集，不需要事后转换。
默认动力学 240 Hz、IK 120 Hz（每 2 个物理步）、双相机 RGB 30 FPS（每 8 步）。
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
