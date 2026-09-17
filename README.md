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

先激活已有的仿真环境，或使用仓库/父工作区的 `.venv`。解析器与启动脚本使用同一选择规则，以下命令固定后续进程的解释器：

```powershell
. .\scripts\python_runtime.ps1
$env:SPACE_SIM_PYTHON = Resolve-SpaceSimPython -RepositoryRoot $PWD.Path
```

**uv 用户使用以下命令**，不需要激活环境，也不需要在环境内安装 pip：

```powershell
uv pip install --python $env:SPACE_SIM_PYTHON -e "../space_sim_UE_Adapter[test]" -e ".[test]"
```

uv 的 `.venv` 没有 pip 是正常现象；不要运行 `ensurepip` 或换用 `python -m pip`。若使用 Conda/传统 venv 且已由 pip 管理，可选择以下替代命令；两种方式选一种：

```powershell
& $env:SPACE_SIM_PYTHON -m pip install -e "../space_sim_UE_Adapter[test]" -e ".[test]"
```

检查依赖，失败时先修复所选环境再继续：

```powershell
& $env:SPACE_SIM_PYTHON -c "import sys,numpy,fastapi,pydantic,uvicorn,bsk_render_adapter; from Basilisk.simulation import mujoco; print(sys.executable); print(mujoco.MJScene)"
```

若尚无环境，可在工作区父目录用 `uv venv .venv --python <与Basilisk匹配的版本>` 创建，再按团队的 Basilisk 安装说明准备 MJScene。不要在已存在的仿真环境上重新创建 venv。

解释器优先级：`-Python` → `SPACE_SIM_PYTHON` → 已激活 venv/Conda → 仓库及父工作区 `.venv`/`venv` → PATH。显式环境无效或依赖缺失会报错，不自动切换。这里使用 `uv pip` 管理已有环境；仓库没有承诺可由 `uv sync` 复现全部原生依赖。

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
