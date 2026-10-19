# Linux 基础迁移

目标是先在一台 Linux RTX 4090 上建立可验证的单路环境，不预留千条训练数据，不要求多机调度。2026-10-19 已跑通真实动力学、UE 双相机及完整脚本录制和 LeRobot 解码；网页场景启停仍需 Linux 适配及单独验收，不能把脚本采集通过认定为整个网页平台就绪。

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

优先使用官方 Linux UE 5.6 预编译包。GitHub 源码方式需要 Epic/GitHub 账号关联且需自行编译整个引擎，基础迁移无需优先选择它。先核查下载与解压大小、工具链和缓存余量，再安装。

2026-10-19 已将目标机磁盘、根分区和 ext4 扩展到约 1 TiB，UE 5.6.1 安装包约 28.75 GiB、解压约 62.76 GiB；实际解压成功，unzip 的逐文件 CRC 检查通过。Linux 包自带 clang 18 工具链。首次安装示例（目标目录应为空，已安装时不要重复解压）：

```bash
mkdir -p "$HOME/engines/UE_5.6"
unzip -q "$HOME/migration/Linux_Unreal_Engine_5.6.1.zip" -d "$HOME/engines/UE_5.6"
```

引擎准备好后，从服务端子目录执行：

```bash
export UE56_ROOT="$HOME/engines/UE_5.6"
"$SPACE_SIM_PYTHON" tools/build_unreal_linux.py
```

此入口只构建项目编辑器目标和插件，构建完成后还要执行下述资产、缓存和实际采集验收。录制脚本支持 `UE56_ROOT` 或 `--unreal`，Linux 查找 `Engine/Binaries/Linux/UnrealEditor-Cmd` 或 `UnrealEditor`；Windows 原有入口保持可用。

Linux 资产准备不再需要 PowerShell。以下第一步不启动 UE，可提前生成本机绝对路径 catalog 和 STL 转换缓存；第二步必须在 UE 和项目插件编译完成后执行：

```bash
"$SPACE_SIM_PYTHON" tools/prepare_sarm_assets.py \
  --template-id sarm-task-box-20261012 --catalog-only
"$SPACE_SIM_PYTHON" tools/prepare_sarm_assets.py \
  --template-id sarm-task-box-20261012
```

入口复用已有模型生成、SARM 插件构建和资产生成器，保持 build scale=100、component scale=1、preserve normals、Nanite 关闭、LOD0 全量。catalog 保存于适配器的 `Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json`；不可复用 Windows catalog 中的绝对路径。

完整模式检查所有预期 `.uasset`，并要求 UE Python 网格配置脚本回写包含 catalog SHA256 与配置网格数量的成功确认。仅有 UE 退出码 0 不算成功。失败或中断不保留成功标记；源 catalog、配置脚本或资产文件发生变化时重新导入；`--force` 强制重做。

2026-10-19 已在目标 Linux 生成 131 个网格（其中 27 个 STL）的 catalog，6 项入口测试通过。源模型中 10 个退化 STL 三角面按原策略保留，未为迁移删减几何。

同日随后已使用用户上传的官方 Linux UE 5.6.1 包完成安装，项目和 Runtime 插件在 Linux 编译成功；131 个网格已实际导入并收到配置校验确认。项目配置显式选择 Vulkan SM6，与 Windows 的 SM6 功能级别对齐。

Windows 的着色器缓存不能用于 Linux。首次部署需要准备 Linux 的派生数据和材质着色器缓存；仅有 `.uasset` 文件、网格导入成功或 NullRHI 测试通过，不保证实际相机已经显示正确材质。缓存生成可耗时数分钟以上，保存日志并检查退出码：

```bash
project="$PWD/../space_sim_UE_Adapter/Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject"
"$UE56_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd" "$project" \
  -run=DerivedDataCache -fill -ProjectOnly -TargetPlatform=Linux \
  -AllowCommandletRendering -RenderOffscreen -unattended -nop4 -nosplash \
  -DDC=InstalledDerivedDataBackendGraph
```

随后执行真实双相机 RGB 验收，检查 idle/STOP 边界、会话重置、帧标识、颜色和图像朝向。首次着色器编译可能超过默认 60 秒启动限时，部署验收可显式延长启动等待；图像与同步断言仍照常执行：

```bash
SPACE_SIM_UE_CAPTURE_SMOKE=1 SPACE_SIM_UE_STARTUP_TIMEOUT=600 \
  "$SPACE_SIM_PYTHON" -m pytest -q -s tests/test_capture_on_demand_renderer.py
```

部署验收还必须覆盖真实场景加载、双相机 5–10 秒短采集、完整任务和官方 LeRobot 解码。离屏采集仍须真实 GPU 渲染，不能用 NullRHI 作为图像验收。

2026-10-19 实机结果：29 项 UE 自动化通过；实际 Vulkan 双相机 RGB 验收通过；5 秒有界片段保存 251 帧，数据完整但任务因人为时限标记失败；完整 `drawer-cycle/5` 成功保存 1723 帧、3446 个相机样本，两路 1024×1024、50 Hz H.264 视频完整解码、200 Hz 腕力和数值校验通过，官方 LeRobot 首/中/尾采样读取通过。完整 episode 共 17 个文件、27482165 字节，未保存 `cameras/` 原始照片。录制 34.44 秒仿真数据耗时 45.403 秒（约 0.759×），包含启动、收尾的整个进程耗时 81.201 秒；这些是一条任务的实测值，不是多次吞吐基准。

已准备场景的后续单条采集示例（仍从服务端目录执行）：

```bash
"$SPACE_SIM_PYTHON" tools/record_drawer_task.py \
  --task drawer_cycle --scene "$HOME/migration/validation-scene.json" \
  --output "$HOME/migration/drawer-recording-$(date +%Y%m%d-%H%M%S)" \
  --episodes-root "$HOME/data/spacesim/episodes"
```

原始 Linux 验收证据位于执行该次验收的目标机 `~/migration/ue-linux-20261019/`；更新后的运行方式和未覆盖范围以本文为准。
