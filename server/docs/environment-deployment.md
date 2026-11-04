# 可编辑环境关卡的构建与部署

项目默认关卡是 `/Game/BSK/Maps/BskEnvironment`，对应源码工程的 `Content/BSK/Maps/BskEnvironment.umap`。编辑器、美术修改、Git 提交以源码工程为准；部署副本只用于运行。编辑方法和接管边界见[环境编辑说明](../../ue-adapter/docs/editable-environment.md)。

## 首次准备与本机运行

拉取代码与 LFS 资产后，按[外部环境资产分发说明](../../docs/guides/external-environment-assets.md)恢复 `Content/IncredibleEarth/`。该目录不随 Git 克隆提供；准备和启动会检查其清单及文件大小，缺失时明确停止。完成资产恢复后，先完整构建 **Editor** target。不要把旧 DLL、`.so` 或 `Saved/AssetImport` 复制到新目录。

Windows 从仓库根目录执行：

```powershell
$env:UE56_ROOT = 'E:\UE5.6\UE_5.6' # 改成本机引擎路径
pwsh -NoProfile -File .\ue-adapter\scripts\build.ps1 -UnrealRoot $env:UE56_ROOT
```

Linux 从服务端子目录执行（`python` 使用自己的仿真环境解释器）：

```bash
python tools/assets/build_unreal_linux.py \
  --adapter ../ue-adapter \
  --unreal /home/spacesim-zmh/engines/UE_5.6
```

构建成功后，会在关卡缺失时自动运行 UE Python commandlet，创建并保存真实 `.umap`。已经存在的关卡绝不重建，因此编辑器内保存的位置、材质、曝光等继续有效。已有地图若是 LFS 指针或明显截断则报错，不用生成器覆盖它。地图和新资源按现有 `.umap`/`.uasset` LFS 规则提交。

也可以在已编译 Editor 的情况下单独准备：

```bash
python ../ue-adapter/Unreal/BskUnrealRenderer/scripts/prepare_environment_level.py \
  --unreal /home/spacesim-zmh/engines/UE_5.6
```

Windows 对应 `Unreal/BskUnrealRenderer/scripts/prepare_environment_level.ps1 -UnrealRoot <引擎路径>`。准备工具会快速检查 UE package 文件头；需要在 UE 中实际加载并验证现有关卡与环境绑定时，Python 加 `--verify`，PowerShell 加 `-Verify`，此模式也不会替换美术编辑。首次 `-Game` 构建缺少地图会明确要求先构建 Editor；打包入口先构建 Editor 并确保地图已保存，再 Cook。

启动入口显式加载该地图：Windows `start_renderer.ps1`、图形 smoke、本地可视化入口；Linux 网页场景与批量自动采集共用的 `renderer_command`。缺图时尝试首次准备；已有编辑内容保持不变。Linux 的 `--check` 不生成地图，缺图时报告需要先构建。

## 同步到现有 Linux 部署副本

新版目录中的源码工程（按实际 clone 位置替换）：

`/path/to/SpaceSim/ue-adapter/Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject`

既有运行副本路径示例（部署目录无需跟随源码改名）：

`/home/lyh/space_enviroment/runtime/ue_adapter/Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject`

先在源码工程完成编辑并 **Save All**，然后从服务端子目录查看同步计划：

```bash
cd /path/to/SpaceSim/server
python tools/deployment/sync_unreal_runtime.py \
  --source-adapter /path/to/SpaceSim/ue-adapter \
  --destination-adapter /home/lyh/space_enviroment/runtime/ue_adapter \
  --dry-run
```

该工具按文件系统同步，因此已恢复、被 Git 忽略的 `Content/IncredibleEarth/` 也会同步。使用前先运行资产校验，避免把不完整环境发往部署目录。

该工具只同步 UE 工程的 `Config/`、`Content/`、`ContentSource/`、`Plugins/`、`Source/`、`scripts/` 和 `.uproject`，不删除目标额外文件，不同步 `Saved/`、`Intermediate/`、`Binaries/`、缓存和 `*.local.*`、`User*.ini` 等机器配置。部署的 `deploy/linux.local.json` 在服务端目录中，也不在同步范围。

第一次同步遇到目标已有不同文件时，默认报告冲突且不复制任何文件。后续根据同步清单，只自动更新目标中未再修改的旧版本；部署副本中的手动改动会再次成为冲突。已有不同的环境地图及 `Content/BSK/Environment/` 美术资源始终需要单独确认。

审核计划后停止平台与该副本的 UE 编辑器，再执行同样命令去掉 `--dry-run`。若计划中的代码差异确实应由源码覆盖，显式加 `--overwrite-conflicts`；该选项仍不覆盖不同的环境地图或 `Content/BSK/Environment/` 资源（包括可编辑太阳材质实例）。决定由源码环境替换运行副本环境时，另加 `--overwrite-environment-level`，运行副本的美术编辑应先保存回源码或另行备份。可将这些选项先与 `--dry-run` 一起使用，再应用完全相同的计划。

```bash
./stop_deployment.sh
# 执行审核后的同步命令，然后完整重编译运行副本：
python tools/assets/build_unreal_linux.py \
  --adapter /home/lyh/space_enviroment/runtime/ue_adapter \
  --unreal /home/spacesim-zmh/engines/UE_5.6
./start_deployment.sh --check
./start_deployment.sh
```

同步工具不停止服务、不构建、不修改部署 JSON，也不会自动推送到远程机器。它只同步 UE 项目，不更新 Python 渲染适配器；部署所用 Python 适配器也应按[适配器安装说明](../../ue-adapter/README.md)更新为同一源码版本。Linux 构建 wrapper 从当前源码 checkout 使用共享构建工具、引擎策略与构建指纹实现，`--adapter` 仅指定要编译和准备关卡的项目副本。更新插件源代码后必须完成上述运行副本的完整编译；不能因为旧 `.so` 存在就跳过。`Saved/AssetImport` 保留该机器自己的资源路径，后续场景准备继续按现有流程检查和更新。

## 验收

1. 打开源码 `.uproject`，不运行后端、不点 Play，确认完整环境可见；调整地球材质、灯光、曝光和预览相机并保存，关闭重开后检查。
2. 独立 Play，确认使用保存的环境与默认视角；再次停止，确保关卡编辑仍在。
3. 启动完整后端场景，核对环境对象数量未增加、位置和日照随星历更新，卫星/机械臂动态加载、控制与相机采集继续工作。
4. 在 Linux 上运行 `--check`、网页场景，以及现有 `-BskOfflineCapture` 自动采集；确认无界面仍能产生有效 RGB 与时间戳。离线禁用捕获式天光等现有 Linux 限制仍适用。

Python/脚本契约测试不能替代这些 UE/GPU、编辑器重开和完整物理采集验收。
