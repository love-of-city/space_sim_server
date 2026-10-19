# Linux 基础迁移

目标是先在一台 Linux RTX 4090 上建立可验证的单路环境，不预留千条训练数据，不要求扩容或多机调度。现阶段先迁移真实动力学、模型、数据保存和脚本入口；UE 图像与网页场景启停需要单独验收，Python 测试通过不代表完整平台就绪。

## 已选环境与路径

- Ubuntu 22.04 x86_64，普通运行用户 `spacesim-zmh`。
- 代码：`/home/spacesim-zmh/src/space_sim_server`，服务端在其 `space_sim_server/` 子目录。
- Python：`/home/spacesim-zmh/envs/spacesim/bin/python`（独立 Python 3.13，不替换系统 Python）。
- 检查日志：`/home/spacesim-zmh/migration/`。
- UE 预留位置：`/home/spacesim-zmh/engines/UE_5.6`，必须准备官方 Linux 安装包；Windows 引擎不可直接复制使用。

当前运行节点不需要 PyTorch CUDA 训练环境。PyTorch 使用 CPU wheel 用于数据处理，UE 仍通过 NVIDIA 驱动/Vulkan 使用 GPU。将来在此机训练时再独立准备训练环境。

## 环境复现

先安装系统依赖（需要 sudo）：

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential git git-lfs cmake pkg-config ffmpeg \
  libvulkan1 vulkan-tools libgl1 libegl1 libxkbcommon0 \
  libxrandr2 libxcursor1 libxi6 libxinerama1 libxss1 libasound2
vulkaninfo --summary
```

确认 Vulkan 列出的设备是 NVIDIA RTX 4090，而不是软件渲染器。驱动能运行 nvidia-smi 并不足以证明图形功能正常。

安装 uv 后，在仓库根目录执行；现有环境不要再次创建：

```bash
uv venv --python 3.13 "$HOME/envs/spacesim"
export SPACE_SIM_PYTHON="$HOME/envs/spacesim/bin/python"
uv pip install --python "$SPACE_SIM_PYTHON" --torch-backend cpu \
  -e './space_sim_UE_Adapter[test]' -e './space_sim_server[test,simulation,posture]' \
  'bsk==2.11.1' 'numpy==2.3.5' 'torch==2.10.0' 'torchvision==0.25.0' 'av==15.1.0'
```

若 PyPI 网络慢，可明确传入可信镜像的 `--index-url`，不修改机器全局配置。LeRobot 版本由项目约束固定为 0.4.4。离线 Python MuJoCo 用于模型工具，禁止将其导入运行中的 Basilisk 仿真进程。

## 原生组件与动力学

从服务端子目录执行：

```bash
cd space_sim_server
export SPACE_SIM_PYTHON="$HOME/envs/spacesim/bin/python"
export PYTHONPATH="$PWD:$PWD/backend"
"$SPACE_SIM_PYTHON" scripts/check_basilisk.py
"$SPACE_SIM_PYTHON" tools/build_native_acceleration.py --component all
"$SPACE_SIM_PYTHON" tools/build_sarm_plugin.py
"$SPACE_SIM_PYTHON" -m pytest -q \
  tests/test_local_mujoco_stepper.py tests/test_native_posture.py \
  tests/test_linux_mujoco_runtime.py
```

Linux 原生库使用 `-O2 -fno-fast-math -ffp-contract=off`。Stepper 从 Basilisk 已加载的 MuJoCo `.so` 查找符号，并检查 3.7.0 版本。库名可能由 wheel 添加哈希，不能写死为 `libmujoco.so`。

SARM 插件不绑定 Basilisk wheel 的私有 SONAME；加载前将唯一已加载运行库的符号显式提供给插件。这样真实仿真使用 Basilisk 的运行库，独立资产工具使用自己的 Python MuJoCo 运行库，不引入第二份运行库。发现多个运行库会拒绝加载。插件安装前仍执行一百万点 SDF 数值等价检查。

单独验证资产工具，避免在动力学测试进程中导入 Python MuJoCo：

```bash
"$SPACE_SIM_PYTHON" -m pytest -q tests/test_sarm_asset_plugins.py
```

真实完整动力学任务示例（不含相机，不是 VLA 训练 episode）：

```bash
"$SPACE_SIM_PYTHON" tools/run_scripted_task.py \
  --task drawer_cycle --native-geometry --no-save-trace \
  --max-sim-seconds 360 --output "$HOME/migration/drawer-cycle-validation"
```

输出目录必须是新目录。与 Windows 做对比时应额外传入同一个 `--scene`，确保轨道、初始关节和 J2 配置一致；不要只比较默认命令的耗时。

首次运行还需要 Basilisk 的 SPICE 星历：`naif0012.tls`、`pck00010.tpc`、`de-403-masses.tpc` 和 `de430.bsp`。若 NASA 下载超时，可从已验证的同版本环境复制这四个缓存文件到 Linux 的 `~/.cache/bsk_support_data/supportData/EphemerisData/`，核对文件校验和；不要复制未完成的 `tmp*` 下载。它们属于仿真依赖，不是采集数据。

## 数据与网页组件

保持 `config/recording.json` 中的采样配置、compact 格式和 CPU H.264 流式编码；迁移不改变采样时钟，也不自动启用 NVENC。

```bash
"$SPACE_SIM_PYTHON" -m pytest -q \
  tests/test_recording_config.py tests/test_streaming_video.py tests/test_compact_recording.py
```

准备 Node.js 22 后：

```bash
npm --prefix frontend ci --no-audit --no-fund
npm --prefix frontend test
npm --prefix frontend run build
npm --prefix signalling ci --no-audit --no-fund
npm --prefix signalling test
npm --prefix signalling run check
```

这些步骤只验证网页构建和信令代码。网页场景管理仍有 PowerShell 启停路径；完整 Linux 平台启动、进程回收和 WebRTC 画面需后续联调。不要将只启动 HTTP 后端误报为可用的仿真平台。

## UE 准备

优先使用官方 Linux UE 5.6 预编译包。GitHub 源码方式需要 Epic/GitHub 账号关联且需自行编译整个引擎，基础迁移无需优先选择它。拿到下载链接后，先核查下载与解压大小、工具链和缓存余量，再安装；当前约 83 GiB 初始可用空间是否足够完整 UE，以实际安装包为准。

引擎准备好后，从服务端子目录执行：

```bash
export UE56_ROOT="$HOME/engines/UE_5.6"
"$SPACE_SIM_PYTHON" tools/build_unreal_linux.py
```

此入口只构建项目编辑器目标和插件，尚需实际 Linux UE 验收。录制脚本支持 `UE56_ROOT` 或 `--unreal`，Linux 查找 `Engine/Binaries/Linux/UnrealEditor-Cmd` 或 `UnrealEditor`；Windows 原有入口保持可用。

Linux 资产准备不再需要 PowerShell。以下第一步不启动 UE，可提前生成本机绝对路径 catalog 和 STL 转换缓存；第二步必须在 UE 和项目插件编译完成后执行：

```bash
"$SPACE_SIM_PYTHON" tools/prepare_sarm_assets.py \
  --template-id sarm-task-box-20261012 --catalog-only
"$SPACE_SIM_PYTHON" tools/prepare_sarm_assets.py \
  --template-id sarm-task-box-20261012
```

入口复用已有模型生成、SARM 插件构建和资产生成器，保持 build scale=100、component scale=1、preserve normals、Nanite 关闭、LOD0 全量。catalog 保存于适配器的 `Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json`；不可复用 Windows catalog 中的绝对路径。

完整模式检查所有预期 `.uasset`，并要求 UE Python 网格配置脚本回写包含 catalog SHA256 与配置网格数量的成功确认。仅有 UE 退出码 0 不算成功。失败或中断不保留成功标记；源 catalog、配置脚本或资产文件发生变化时重新导入；`--force` 强制重做。

2026-10-19 已在目标 Linux 生成 131 个网格（其中 27 个 STL）的 catalog，6 项入口测试通过；尚未在真实 UE 中执行导入与配置。源模型中 10 个退化 STL 三角面按原策略保留，未为迁移删减几何。

同日官方 Linux 下载页在正常浏览器请求中返回 HTTP 403 并要求人工安全验证，尚未进入账号登录，因此未验证账号下载权限。需要用户在官方页面完成验证、登录并提供 UE 5.6 Linux 安装包链接；不将账号密码存入部署文件。

后续必须完成：Linux 资产 catalog、真实场景加载、双相机 5–10 秒短采集、完整任务和官方 LeRobot 解码检查。离屏采集仍须真实 GPU 渲染，不能用 NullRHI 作为图像验收。
