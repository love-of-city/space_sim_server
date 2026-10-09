# MJCF STL 网格支持

资产准备入口接受 MJCF 引用的 ASCII STL 和 Binary STL。STL 只作为源文件：脚本会保留全部三角形，生成带法线和基础 UV 的缓存 OBJ，然后由 UE 5.6 编辑器导入为 Static Mesh。运行时插件只加载已烘焙的 `/Game/...` 资产，不会在网络线程或 Game Thread 解析外部 STL。

## 通用命令

```powershell
Set-Location E:\mujoco_demo\space_sim_UE_adapter
.\scripts\prepare_mjcf_assets.ps1 `
  -MjcfPath E:\path\to\model.xml `
  -Destination /Game/BSK/Generated/MyModel `
  -CatalogPath E:\path\to\my_model.catalog.json `
  -NormalMode auto
```

`auto` 默认以 60 度夹角生成平滑法线并保留机械硬边。可以通过 `-StlSmoothingAngle` 调整，也可以选择 `preserve`（逐面法线）或 `recompute`（UE 重算法线）。Static Mesh 默认关闭 Nanite、保留完整标准 LOD0，并在构建阶段执行米到厘米转换。

转换缓存位于项目 `Saved/AssetImport`，以源文件 SHA-256、法线模式和转换器版本判定是否有效。源 STL 修改后会自动重新导入，缺失、损坏或非有限顶点会明确报错。

## CubeSat + SO-101

```powershell
.\scripts\prepare_spacecraft_arm_assets.ps1 `
  -ModelRoot E:\mujoco_demo\test\model\spacecraft_and_arm

# 带抓取目标的版本
.\scripts\prepare_spacecraft_arm_assets.ps1 `
  -ModelRoot E:\mujoco_demo\test\model\spacecraft_and_arm `
  -Variant grasp

# 一条命令导入并实时运行自由漂浮组合模型
.\scripts\run_spacecraft_arm.ps1 `
  -ModelRoot E:\mujoco_demo\test\model\spacecraft_and_arm `
  -Duration 10 -SimulationRate 1

# 原生 PID + 纯接触抓取场景，结束后保留最后一帧
.\scripts\run_spacecraft_arm_grasp.ps1 `
  -ModelRoot E:\mujoco_demo\test\model\spacecraft_and_arm `
  -Duration 10 -SimulationRate 1 -KeepRendererOpen
```

该模型使用 MJCF `<frame>` 安装机械臂；元数据遍历会穿过 frame，同时仍以实际动态 body 作为 UE Actor 层级。STL 本身不带颜色和纹理，最终颜色来自 MJCF `material/rgba`。示例中的 1 kHz 自由漂浮多刚体动力学由 Basilisk/MJScene 执行，渲染桥只约 30 Hz 采样最新状态。

原生抓取包装直接调用外部 `scenario_cubesat_so101_grasp.py` 的 `_build_simulation()` 与 `_initialize_state()`，因此使用原轨迹发生器、六个 PID、六个力矩限幅器、目标初态和纯接触模型。Windows 下 Python MuJoCo 与 Basilisk MJScene 的 MuJoCo DLL 不能安全地同时预加载；包装仅延迟原脚本用于事后指标分析和 MP4 的 Python MuJoCo API，不改变实时 Basilisk 动力学。需要原始数值验收时，应在兼容的独立进程/环境运行原脚本的分析阶段。
