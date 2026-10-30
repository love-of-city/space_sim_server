# SARM 最终版 MJCF 资源包

本包包含 `SARM_convex.xml`、它实际引用的 1,286 个 STL 网格资源，以及双线螺纹碰撞所需的 `libtwo_start_thread.so`、源文件 `thread_sdf.cpp` 和编译脚本 `build_thread_sdf.py`。不包含控制或验证脚本，也不包含 GIF。`meshes/` 的相对路径必须与 XML 保持当前结构。

SDF 插件库是 Linux x86-64 版本，链接 MuJoCo 3.15.0。加载 MJCF 前须先注册该库，例如在已安装对应 MuJoCo 的 Python 环境中：

```python
from pathlib import Path
import mujoco

package = Path("/path/to/SARM_final_mjcf_package")
mujoco.mj_loadPluginLibrary(str(package / "libtwo_start_thread.so"))
model = mujoco.MjModel.from_xml_path(str(package / "SARM_convex.xml"))
```

若在其他 MuJoCo ABI 上使用，请在目标环境中安装 C++17 编译器和对应的 `mujoco` Python 包，然后在此目录运行 `python build_thread_sdf.py` 重新编译插件。其他操作系统上可能还需相应调整编译命令和共享库后缀。
