# 仓库本地 Python 选择与 Conda 原生库加载

为避免虚拟环境迁移后新终端选择其他解释器，UE 启动脚本支持仓库根目录 `.space-sim-python`。此配置被 Git 忽略，显式 `-Python` 和 `SPACE_SIM_PYTHON` 仍优先；无效路径或依赖仍直接报错。

当所选环境包含 `conda-meta`，解析器将该环境的原生库目录加入当前启动进程 PATH，支持 NumPy/MKL DLL 加载，不修改系统环境变量或已安装的包。实现与服务端一致，编辑器内置 Python 的资产导入流程不受影响。

配置、优先级与回退方法见 [解释器说明](../docs/PYTHON_RUNTIME.md)。验证复用适配器基础测试和版本检查，并在实际 Conda 环境预检 NumPy；本机配置、环境内容和绝对路径不进入提交。
