# 为仓库固定 Python 解释器

已有仿真环境移动到其他目录后，可以在仓库根目录放置 `.space-sim-python`，内容为当前机器的 `python.exe` 路径。该文件已由 Git 忽略，不随提交共享；不需要重建或复制虚拟环境。

先在已激活正确环境的 PowerShell 中确认 `python`，然后执行：

```powershell
(Get-Command python).Source
Set-Content -LiteralPath .space-sim-python -Value (Get-Command python).Source
```

每个仓库分别设置一次。解释器选择顺序为：显式 `-Python` 参数、`SPACE_SIM_PYTHON`、仓库根目录 `.space-sim-python`、已激活 venv、已激活 Conda、仓库或父目录的 `.venv`/`venv`、PATH 中的 Python。

选中的路径不存在或缺少必需模块时仍然立即报错，不回退到另一个环境。若临时切换环境，可显式设置 `SPACE_SIM_PYTHON`；删除本地配置文件可恢复自动发现。

对于包含 `conda-meta` 的环境，解析器会在当前启动进程的 PATH 前面加入该环境的原生库目录，使 NumPy/MKL 等 DLL 能被正确加载。此操作不写入系统 PATH，不修改环境中的包。

服务端 `scripts/python_runtime.ps1` 与 UE 仓库 `Unreal/BskUnrealRenderer/scripts/python_runtime.ps1` 应保持一致。

## 独立的离线姿态/路径规划解释器

零位自动展开需要 Python `mujoco==3.7.0` 和 NumPy，但不能把 Python MuJoCo
导入权威 Basilisk 仿真进程。可在独立虚拟环境安装规划依赖，在服务端仓库根目录创建
`.space-sim-posture-python`，内容为该环境 `python.exe` 的绝对路径。
该本机配置已被 Git 忽略；不要将机器路径提交到仓库。

`scripts/run_backend.ps1` 在启动后端前读取规划解释器，优先级为：
当前进程的 `SPACE_SIM_POSTURE_PYTHON` → `.space-sim-posture-python`。
两者都未配置时保留原有行为，由离线任务使用后端解释器；此时后端环境必须具备规划依赖。
显式选中的路径不存在、配置为空或依赖版本不符时直接报错，不静默回退。
检查依赖也是独立子进程，不改变 `SPACE_SIM_PYTHON` 或动力学解释器。

只修改 Windows 用户环境变量不能保证已经打开的终端、IDE 或启动器获得新值。
仓库本地配置不依赖这些父进程的环境刷新，更适合双击启动或从旧终端重启。
修改配置后需真正重启后端，不能只刷新网页；启动日志应出现
`Offline posture Python: ... (MuJoCo 3.7.0)`。
直接执行 `python -m space_arm_platform.main` 时没有经过该 PowerShell 入口，
仍需显式传入 `SPACE_SIM_POSTURE_PYTHON`。
