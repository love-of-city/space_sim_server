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
