# 为仓库固定 Python 解释器

在仓库根目录的 `.space-sim-python` 中写入当前机器已有环境的 `python.exe` 路径，可在新终端中继续使用同一个环境。该文件由 Git 忽略，不能提交本机路径。

在已激活正确环境的 PowerShell 中执行：

```powershell
(Get-Command python).Source
Set-Content -LiteralPath .space-sim-python -Value (Get-Command python).Source
```

选择顺序为：显式 `-Python` 参数、`SPACE_SIM_PYTHON`、`.space-sim-python`、已激活 venv、已激活 Conda、仓库或父目录 `.venv`/`venv`、PATH。每个仓库需要分别设置；无效配置会直接报错，不自动改用其他环境。

解析器发现 `conda-meta` 时会在当前进程 PATH 前面加入环境自己的原生库目录，使 NumPy/MKL 等 DLL 正确加载，不修改系统 PATH 或已安装的包。需要临时切换时设置 `SPACE_SIM_PYTHON`，或删除本地配置恢复自动发现。

该解析器与服务端 `scripts/python_runtime.ps1` 保持一致。UE 编辑器导入资产仍使用编辑器内置 Python。
