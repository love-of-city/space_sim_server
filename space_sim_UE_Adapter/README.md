# BSK → Unreal Engine 可视化适配器

团队开发请先阅读[贡献指南](CONTRIBUTING.md)。基础 CI 不替代完整仿真、UE 和采集验收。

Basilisk/MJScene 状态发送端与 UE 5.6 渲染接收端。动力学由仿真端推进，UE 负责渲染与相机采集，协议为 `bsk-render/2`。

| 目录 | 用途 |
| --- | --- |
| `Adapters/bsk_render_adapter/` | 通用 Python 发送端 |
| `Unreal/BskUnrealRenderer/` | UE 项目、Runtime 插件、示例和测试 |
| `scripts/` | 仓库级入口 |
| `test/model/` | 独立 Demo 模型 |

完整遥操作平台按[服务端 README](https://github.com/love-of-city/space_sim_server#readme)安装两个同级仓库。本页是适配器独立构建与验证流程。

## 1. 环境与安装

Windows x64、PowerShell 7、Git LFS、UE 5.6、Visual Studio 2022 C++ 游戏开发工具及 Windows SDK。图形运行需要支持 UE 的 GPU/驱动。Python 3.11+；真实仿真需匹配解释器且具备所需 API 的 Basilisk/MJScene。可用 `uv pip install --python $env:SPACE_SIM_PYTHON "bsk[all]"` 安装，源码编译只是可选方式，不限定源码版本。安装后运行 `& $env:SPACE_SIM_PYTHON scripts/check_basilisk.py`。mock 与基础线协议测试不依赖 Basilisk，桥接及真实设备协议测试仍需 Basilisk。

所有命令从**本仓库根目录**执行。可复用服务端的仿真环境。

```powershell
git lfs install --local
git lfs pull
```

三种方式均受支持，按现有环境选择 **uv、Conda 或传统 venv/pip 中的一种**。不要在已有仿真环境上重新创建环境。下列新建命令仅供首次配置，Python 版本示例为 3.13，必须按 Basilisk 二进制版本调整。

建议在新的 PowerShell 7 终端操作。切换环境时重新设置 `SPACE_SIM_PYTHON`；该变量优先于已激活环境，避免继续使用上一次选中的解释器。

### A. uv

已有仓库或父工作区的 `.venv`/`venv` 可以自动发现；其他位置可将 `SPACE_SIM_PYTHON` 直接设为目标 `python.exe` 的绝对路径。首次新建环境（已有环境跳过）：

```powershell
uv venv .venv --python 3.13
```

```powershell
. .\Unreal\BskUnrealRenderer\scripts\python_runtime.ps1
$env:SPACE_SIM_PYTHON = Resolve-SpaceSimPython -RepositoryRoot $PWD.Path
uv pip install --python $env:SPACE_SIM_PYTHON -e ".[test]"
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
& $env:SPACE_SIM_PYTHON -m pip install -e ".[test]"
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
& $env:SPACE_SIM_PYTHON -m pip install -e ".[test]"
```

也可先执行 `.\.venv\Scripts\Activate.ps1` 激活。直接调用解释器无需激活，适用于激活脚本受执行策略限制的情况。若传统 pip 管理的 venv 确实缺少 pip，可用 `& $env:SPACE_SIM_PYTHON -m ensurepip --upgrade` 修复；这一步不用于 uv 管理的环境。

### 共同检查与后续步骤

新建任何一种环境都不会自动安装 Basilisk/MJScene。真实仿真需先按团队的 Basilisk 安装说明准备匹配的原生模块，再检查依赖：

```powershell
& $env:SPACE_SIM_PYTHON -c "import sys,numpy; from bsk_render_adapter import BasiliskRenderBridge; print(sys.executable); print(BasiliskRenderBridge)"
# 真实仿真另需检查；仅 mock 可跳过此项
& $env:SPACE_SIM_PYTHON -c "from Basilisk.simulation import mujoco; print(mujoco.MJScene)"
```

启动脚本只调用解释器，不要求特定环境管理器。统一优先级：`-Python` → `SPACE_SIM_PYTHON` → 仓库本地 `.space-sim-python` → 已激活 venv（`VIRTUAL_ENV`）→ 已激活 Conda（`CONDA_PREFIX`）→ 仓库及父工作区 `.venv`/`venv` → PATH。选中环境无效或缺少所需模块时直接报错，不自动切换。三种方式后续均使用同一套测试和启动命令；UE 资产导入仍使用编辑器内置 Python。固定解释器及 Conda DLL 加载说明见 [Python 环境选择](docs/PYTHON_RUNTIME.md)。

## 2. 构建与测试

设置本机包含 `Engine` 的目录，下例按实际安装位置修改。依次执行，失败时修复后再继续：

```powershell
$env:UE56_ROOT = 'D:\UE\UE_5.6'
pwsh -NoProfile -File .\scripts\build.ps1
pwsh -NoProfile -File .\scripts\test.ps1 -SkipBuild
pwsh -NoProfile -File .\scripts\smoke_e2e.ps1
```

分别验证 Editor C++ 构建、Python/UE 自动化测试、mock 两次连接与重连。默认 smoke 使用 NullRHI，不验收 GPU 或 WebRTC 视频。依赖外部 Basilisk 的测试可能 skip，须检查输出。

无 UE 时可只执行 Python 部分（可选替代，不能视作完整验收）：

```powershell
pwsh -NoProfile -File .\scripts\test.ps1 -SkipUnreal
```

## 3. 独立预览

完成构建后运行 mock，60 秒后自动结束：

```powershell
pwsh -NoProfile -File .\scripts\run_demo.ps1 -Duration 60
```

C 切换自由相机，W/S、A/D、Q/E 和鼠标观察，Home 返回；M 显示任务面板。提前停止：

```powershell
pwsh -NoProfile -File .\scripts\stop_renderer.ps1
```

浏览器操作台与真实机械臂使用服务端启动入口；renderer 不会自动启动信令和网页。

## 开发与参考

- 合并前执行构建、测试和 smoke；渲染改动另做图形验收。
- 模型、纹理和 `.uasset` 使用 LFS；`Saved/`、`Intermediate/`、虚拟环境、日志和录制不提交。
- 移动路径后重新生成资产映射，不复制别人 `Saved/AssetImport` 中的 catalog。
- 发布时保持 `VERSION`、Python 包和 UE 插件版本一致。
- [Python 发送端](Adapters/README.md) / [UE 模块](Unreal/BskUnrealRenderer/README.md)
- [协议](Unreal/BskUnrealRenderer/docs/PROTOCOL.md) / [MJCF](Unreal/BskUnrealRenderer/docs/MJCF_MESHES.md) / [STL](Unreal/BskUnrealRenderer/docs/STL_MESHES.md)

Demo 8、UR5e、CubeSat、轨道抓取、录制回放与打包脚本是可选开发工具，参数见对应脚本，不是首次部署必做步骤。OpNav 闭环、异步 GPU 读回和单遍分割尚未完成。
