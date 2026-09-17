# 太空机械臂遥操作与数据采集平台

浏览器操作台、用户与场景管理、训练数据记录。Basilisk/MJScene 负责权威动力学，[UE 适配器](https://github.com/love-of-city/space_sim_UE_Adapter)负责渲染。WebRTC 用于操作预览；训练图像通过独立的 `bsk-capture/1` 通道采集。

## 环境要求

完整运行入口支持 Windows x64 + PowerShell 7。需要 Git/Git LFS、Node.js 22 LTS（含 npm）、Unreal Engine 5.6、Visual Studio 2022 C++ 游戏开发工作负载及 Windows SDK，以及支持 UE 渲染和 H.264 编码的 GPU/驱动。

Python 要求 3.11+，版本必须与安装的 Basilisk 二进制匹配。真实仿真必须安装**包含 MJScene 的 Basilisk**；普通 Python `mujoco` 包不能替代 `Basilisk.simulation.mujoco`。按照团队使用的 Basilisk 源码版本及其[安装文档](https://hanspeterschaub.info/basilisk/Install.html)准备并启用 MuJoCo 支持。以下命令不会安装 UE、编译器或 Basilisk。

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
uv pip install --python $env:SPACE_SIM_PYTHON -e "../space_sim_UE_Adapter[test]" -e ".[test]"
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
& $env:SPACE_SIM_PYTHON -m pip install -e "../space_sim_UE_Adapter[test]" -e ".[test]"
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
& $env:SPACE_SIM_PYTHON -m pip install -e "../space_sim_UE_Adapter[test]" -e ".[test]"
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

CAD 转换等可选测试可能因缺少专用依赖而 skip；skip 不代表功能验收通过。UE 构建与接收端测试见适配器 README。[审计记录](docs/README_AUDIT.md)记录本次实际执行范围。

## 4. 启动与停止

指定本机包含 `Engine` 的目录，下例按实际安装位置修改。环境变量仅影响当前终端及其子进程。

```powershell
$env:UE56_ROOT = 'D:\UE\UE_5.6'
pwsh -NoProfile -File .\scripts\run_platform.ps1 -ApiPort 18000
```

首次启动准备 Pixel Streaming、构建缺少的 UE 模块，并按本机路径生成资产映射。需要访问 GitHub/npm；不要复制别人 `Saved/AssetImport` 中的本机 catalog。

打开 `http://127.0.0.1:18000`，登录后选择模板并点击“生成并启动场景”。平台启动成功不等于场景或视频已经就绪。全新认证数据库默认本机账号为 `admin` / `ChangeMe123!`；已有数据库使用原密码，登录后可修改。需要训练图像时先勾选“权威采集”，场景就绪后再开始 episode；只预览时保持关闭。

检查 API：

```powershell
Invoke-RestMethod http://127.0.0.1:18000/api/health
```

结束场景使用网页按钮，结束整个平台执行：

```powershell
pwsh -NoProfile -File .\scripts\stop_platform.ps1
```

默认其他端口：播放器 8080、UE 信令 8888、控制 8766、采集 8767、渲染状态 5558。占用时查看进程或调整脚本参数。启动会停止本项目记录的旧平台/场景。

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
