# 通用 MJCF Mesh 渲染

运行时仍由 Basilisk/MJScene 提供唯一权威动力学。UE 插件只读取每个刚体的世界状态，以及 manifest 中的 body-local mesh 描述。

## UR5e 一键示例

```powershell
Set-Location E:\mujoco_demo\space_sim_UE_adapter\Unreal\BskUnrealRenderer
.\scripts\run_ur5e.ps1 -Duration 120 -SimulationRate 1
```

首次运行会把 20 个 OBJ 离线导入 `/Game/BSK/Generated/UR5e`，以后会复用 `.uasset`。强制重新导入：

```powershell
.\scripts\run_ur5e.ps1 -ReimportAssets
```

法线导入支持三种模式：

```powershell
# 保留 OBJ/FBX 中的原始法线，只生成 MikkTSpace 切线
.\scripts\run_ur5e.ps1 -NormalMode preserve -ReimportAssets

# 检测源法线；有效则保留，无效或缺失才重算（默认）
.\scripts\run_ur5e.ps1 -NormalMode auto

# 强制重算法线和切线
.\scripts\run_ur5e.ps1 -NormalMode recompute -ReimportAssets
```

模式会记录在 `Saved/AssetImport`，未更换模式且资产已存在时不会重复构建。

`Esc` 退出 UE 窗口；发送端结束后脚本也会关闭 UE。`W/S/A/D/Q/E` 和鼠标用于相机控制。

## 接入其他 MJCF

先把 MJCF 引用的 mesh 转成 UE 项目资产并生成目录：

```powershell
.\scripts\prepare_mjcf_assets.ps1 `
  -MjcfPath E:\path\to\scene.xml `
  -Destination /Game/BSK/Generated/MyModel `
  -CatalogPath .\Config\BskAssets\my_model.json
```

发送端注册场景时同时给出 MJCF 与目录：

```python
bridge.add_mj_scene(
    scene,
    namespace="my_model",
    source_path=r"E:\path\to\scene.xml",
    mesh_asset_catalog=r"E:\mujoco_demo\space_sim_UE_adapter\Unreal\BskUnrealRenderer\Config\BskAssets\my_model.json",
)
```

当前版本会传递 MJCF 的 `rgba`、`specular`、`shininess`、`reflectance`、
`emission`、纹理名称和重复比例。UE 使用 Default Lit/PBR 材质、全部材质槽、
确定性曝光和低强度观察补光，并在离线导入后重算法线及 MikkTSpace 切线。
文件纹理会由资产准备脚本离线导入 UE，目录把源图片映射为打包软对象路径；运行时
不会直接读取任意磁盘纹理。若 OBJ 的 `mtllib` 文件确实存在且 MJCF 没有覆盖材质，
UE 会保留导入资产自身的多槽材质。缺失 MTL（例如仓库中的 UR5e）仍以 MJCF material
为准。

`add_mj_scene()` 还会自动读取 `<visual><headlight>` 和 `<light>`。方向光、点/聚光、
随 body 安装的灯以及 `targetbodycom` 目标跟踪均在 UE Game Thread 创建或更新；这些
只影响画面，不参与或反向影响 MJScene 动力学。MJCF builtin checker/gradient 目前只保留
描述，尚未逐种生成与 MuJoCo 完全一致的程序纹理。

使用 Vizard 风格入口时，可按 `scList` 的顺序传入：

```python
renderer = ueSupport.enableUnrealVisualization(
    scSim,
    "simTask",
    [scene],
    namespaceList=["my_model"],
    mjcfPathList=[r"E:\path\to\scene.xml"],
    meshAssetCatalogList=[r"E:\mujoco_demo\space_sim_UE_adapter\Unreal\BskUnrealRenderer\Config\BskAssets\my_model.json"],
)
```

适配层会展开 MJCF `include`，读取 mesh、材质、缩放、父子 body 与视觉/碰撞角色，并消除 MuJoCo 编译器对 mesh 自动居中产生的补偿变换。UE 只加载 `/Game` 软对象路径；目录缺失或资产加载失败时，会回退到碰撞 primitive/占位体，不会从运行时直接加载外部磁盘文件。
