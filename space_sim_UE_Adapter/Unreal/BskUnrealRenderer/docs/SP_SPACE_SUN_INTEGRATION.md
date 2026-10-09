# SP_space 动态太阳接入

## 当前分支

`feature/sp-space-sun-integration-20260904`

本次变更只把 `SP_space/Content/SpaceFX` 太阳视觉资源迁移到
`BskUnrealRenderer/Content/SpaceFX`，没有迁移 `Space.umap`、关卡中的
`DirectionalLight` 或完整 `SP_space` 工程。

## 已接入资源

`Config/bsk_unreal_scene.json` 中的可见太阳现在使用：

```json
"sun_visual_mesh": "/Game/SpaceFX/Meshes/SM_PlanetSphere.SM_PlanetSphere",
"sun_visual_material": "/Game/SpaceFX/Materials/M_Star.M_Star"
```

当前还启用了源 `BP_Star` 使用的三个 Cascade 粒子系统：

```json
"sun_visual_effects_enabled": true,
"sun_visual_effect_scale": 1.0,
"sun_burst_particle": "/Game/SpaceFX/Particles/P_Sun_Bursts.P_Sun_Bursts",
"sun_halo_particle": "/Game/SpaceFX/Particles/P_Sun_Halo.P_Sun_Halo",
"sun_lines_particle": "/Game/SpaceFX/Particles/P_Sun_Lines.P_Sun_Lines"
```

粒子组件由 `BskSceneController` 原生创建并挂到同一个太阳 Actor 上，随太阳
可视半径缩放。如果在特定显卡或分辨率下效果过大，可以将
`sun_visual_effect_scale` 调小，例如 `0.5`；权威相机数据采集时也可以将
`sun_visual_effects_enabled` 设为 `false`。

## 权威关系

- Basilisk/SPICE：太阳位置、姿态、仿真时间和光照方向的唯一权威。
- `BskSceneController`：根据 celestial-body frame 更新可见太阳位置，并更新
  唯一的 UE `DirectionalLight`。
- `M_Star`：负责太阳表面材质的动态视觉表现。
- `P_Sun_Bursts`、`P_Sun_Halo`、`P_Sun_Lines`：负责太阳爆发、光晕和射线效果。
- `SP_space` 的 `BP_Star` 不作为独立轨道/光照驱动器运行，避免与 BSK 状态
  产生双重控制；其视觉资源由运行时直接装配。

## 验证

先在 UE 工程目录运行：

```powershell
.\scripts\validate_sp_space_sun_assets.ps1
```

再进行 C++/运行时测试：

```powershell
.\scripts\test.ps1
.\scripts\smoke_e2e.ps1
```

若使用 Git LFS，新增 `.uasset` 文件会按仓库现有规则存储。
`SP_space` 资产的再分发许可应按原工程/资产来源确认；本仓库只记录其
来源路径，不声明第三方资产许可。
