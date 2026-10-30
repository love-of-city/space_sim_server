# Foil002：单块卫星外板的视觉试验

此功能仅修改 UE 外观。MJCF、OBJ、质量、惯量、关节、碰撞和动力学保持不变。
不是 MLI 厚度、柔性变形、热学或几何褶皱模拟。

## 覆盖范围与开关

`Config/DefaultGame.ini` 的 `[Bsk.MaterialOverrides]` 通过**完整静态网格对象路径**精确匹配：

- 网格：`/Game/BSK/Generated/SARM/part_001_color_00.part_001_color_00`
- 原模型可视 geom：`target_instance_0006_part_001_color_00`
- 材质：`/Game/BSK/Materials/Foil002/MI_BskFoil002.MI_BskFoil002`

当前只覆盖固定主体的一块外板（约 0.2263 × 0.4145 m，厚度方向约 0.013 m），
不覆盖太阳翼、机械臂或抓取部件。该资源在其他场景被复用时，也会使用此覆盖。
不要使用数字 geom 索引或模糊名称匹配，因为不同碰撞版本的索引会变化。

恢复原外观：将该节 `Enabled=False` 后重新启动 UE，或启动 UE 时增加
`-BskDisableMaterialOverrides`。关闭时走原材质逻辑；资源缺失时记录警告并回退，
不使仿真启动失败。此项是 UE 进程级设置，不是按用户/实例独立切换。
已有运行中的 UE 进程不会热更新代码、材质配置，需要重新启动。

## 材质

独立的 `M_BskFoil002` / `MI_BskFoil002` 不修改 `M_BskPbrOpaque`。
5 张贴图：Color（sRGB）、NormalDX（Normalmap，线性、不翻绿通道）、
Roughness / Metalness / AmbientOcclusion（Masks，线性）。
没有接入 Displacement，不改变网格轮廓。
由于源网格没有 UV，无法提供可靠的纹理流送密度，本试验的 5 张 2K 贴图设为
NeverStream 常驻，避免被降到模糊 mip；扩展到大量材质时需重新评估显存预算。

采用物体局部坐标三平面采样，不需要 OBJ 的 UV 或切线。
法线在局部空间合成后转换为世界空间，材质关闭 tangent-space normal。
纹理随着物体平移/旋转，不是固定在世界空间；坐标单位为导入后局部厘米。
当前 SARM 网格导入 build scale=100、component scale=1。
非均匀组件缩放尚未验证；若未来缩放模型，需要重新评估贴图尺度及法线转换。

实例参数（构建脚本重建时恢复以下值）：

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `TileSizeCm` | 25 | 每个纹理周期的局部尺寸；减小会更密 |
| `NormalStrength` | 0.55 | 褶皱强度；0 可用于检查原始表面法线 |
| `RoughnessBias` | 0.16 | 在源粗糙度上增加，最终限制为 0.08～0.95 |

三平面投射会增加采样成本，本试验只给一块外板使用。
外板自身的孔洞、凹槽和拓扑保持原样，没有另加一层包覆壳。
最终明暗依赖原场景光照和反射，不应期待所有角度都像官网摄影棚材质球。

## 准备与构建

从 `Unreal/BskUnrealRenderer` 运行：

```powershell
.\scripts\build.ps1 -UnrealRoot 'C:\Program Files\Epic Games\UE_5.6'
.\scripts\prepare_foil_material.ps1 -UnrealRoot 'C:\Program Files\Epic Games\UE_5.6'
```

`prepare_runtime_materials.ps1` 已接入准备步骤。源文件指纹未变且 7 个资源都存在时跳过。
`-Force` 只重建独立 Foil002 资源；`create_foil_material.py` 不写模型或通用材质。
生成目录位于已有 `/Game/BSK` AlwaysCook 范围内。打包发行版仍需重新编译、cook 和打包，
仅重启旧打包 exe 不会载入源项目的新资产。完整打包未包含在本次验证中。

`ContentSource/Foil002/source.json` 记录下载地址、CC0-1.0 来源和每张 PNG 的 SHA256。
源 PNG 和生成的 `.uasset` 均应按仓库既有 Git LFS 规则保存。

## 验证

- `tests/test_foil_material.py`：素材校验、绑定范围、回退顺序、构建缓存和投射配置。
- UE Editor 编译：验证 C++。
- UE Python 资源构建：导入纹理、创建连接、编译材质、回读实例参数。
- 独立静态 replay：使用卫星 XML 的可视网格和零位变换检查覆盖及前后截图，不运行动力学。

预览灯光只用于独立测试配置，不会写回运行平台的太阳光照设置。
