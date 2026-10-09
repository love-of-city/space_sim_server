# SARM 主卫星：独立 UV 隔热毯视觉层

这是基于当前 `base_link.obj` 几何尺寸制作的**外观近似**，不是真实航天器 MLI 工程复刻。
不模拟隔热、柔性、材料质量或包覆层碰撞；包覆尺寸和设备预留范围都是可调整的视觉参数。

## 与目标星金箔试验的区别

- 原来的 Foil002 覆盖仍仅替换目标星 `part_001_color_00` 的材质，不受本功能影响。
- 本功能保留搭载机械臂主星 `base_link` 的原材质，另加 `SM_SarmMLI` 独立可视网格。
- 4 个侧面每面 2 块；新增主体底板环形区域、下方箱体 4 个侧面和底面，每区域 2 块，共 20 块封闭薄壳。顶部机械臂平台继续留空。
- 面片有 20 个不重叠的正面 UV 岛；背面复用 UV，闭合侧边另有非退化 UV。
- 实际网格有毫米级宏观起伏、折边带、接缝、设备开口，细小褶皱用 Foil002 法线表现。
- 正面、包边、背面分 3 个材质槽，使用独立 UV 材质，不走之前的三平面投射。

原始 XML、OBJ、惯量、质量、反作用轮、关节和碰撞没有修改。所有网格在 UE 中仍由 BSK 状态驱动。

## 参数与覆盖范围

拟合配置：`ContentSource/SarmMLI/recipe.json`，包含原 base_link SHA256。
包覆侧面平面（模型坐标，m）：x=-0.20 / 0.60，y=-0.11 / 0.64，z 范围 -0.49～0。
为边框留 18 mm，分块间留 8 mm，突出部件包围矩形额外外扩 12 mm。
这些不是已知硬件接口尺寸或认证安全间隙，只是保守的视觉开口。

- 基础外移 3 mm，轻微起伏额外约 0～2.4 mm。
- 视觉厚度 0.6 mm，包边宽 8 mm；没有建立真实多层薄膜结构。
- 20 块、54,330 个源顶点、108,596 个三角形，UE 导入后保留三角形数量。
- UV 正面坐标密度 0.45 UV/m，材质 `TextureTiling=8`，纹理周期约 27.8 cm。
- 正面 `NormalStrength=0.35`，`RoughnessBias=0.23`。
- 内边缘平滑鼓起，不覆盖每个原始螺钉细节。矩形开口不是精确贴合的裁剪模板。

原网格未改变。若原始主体换版，必须重新检查侧面平面和设备开口，不能仅复用旧包覆。

## UE 绑定及恢复

`Config/DefaultGame.ini`：

```ini
[Bsk.VisualOverlays]
Enabled=True
/Game/BSK/Generated/SARM/base_link.base_link=/Game/BSK/VisualOverlays/SarmMLI/SM_SarmMLI.SM_SarmMLI
```

UE 创建原可视网格后将隔热毯组件挂到它下面，局部变换为单位变换。
因此包覆随原网格平移/旋转/缩放，不挂到世界坐标，也不需要修改 BSK 通信协议。
覆盖只按完整资产路径匹配；复用同一 base_link 资源的其他场景也会显示隔热毯。
不会匹配机械臂 link、夹爪或目标星。

隔热毯组件：Movable，NoCollision，无 overlap，无导航影响，没有启用刚体模拟。
保留其原生材质槽；不会被通用材质替换。支持正常投影。
资源缺失时告警并只显示原模型。

关闭隔热毯：上述节设置 `Enabled=False`，或 UE 启动加入 `-BskDisableVisualOverlays`。
重新启动对应 UE 进程生效，不会热更新已经创建的组件。
此开关与 `[Bsk.MaterialOverrides]` / `-BskDisableMaterialOverrides` 相互独立。

## 重建

从 `Unreal/BskUnrealRenderer` 运行：

```powershell
.\scripts\build.ps1 -UnrealRoot 'C:\Program Files\Epic Games\UE_5.6'
.\scripts\prepare_sarm_blanket.ps1 -UnrealRoot 'C:\Program Files\Epic Games\UE_5.6' -Python '<含 numpy 的 python.exe>'
```

`prepare_runtime_materials.ps1` 已接入准备步骤。资源/源文件指纹一致时跳过；用 `-Force` 重建。
源 OBJ 由 `generate_sarm_blanket.py` 根据 recipe 生成，输出单位是 **UE cm**；recipe 使用 m。
UE import/build scale=1，避免二次转换；它和原 base_link 导入后的坐标一致（包含相同的 OBJ 轴转换）。
源网格文件位于 `ContentSource/SarmMLI`；没有写入仿真模型目录。

`create_sarm_blanket_assets.py` 在无窗口编辑器中生成/导入：
- `SM_SarmMLI`
- `M_SarmMLI_UV`
- `MI_SarmMliFace` / `MI_SarmMliHem` / `MI_SarmMliBacking`

仅更新 `/Game/BSK/VisualOverlays/SarmMLI`。共用已有 CC0 Foil002 纹理，来源见
`ContentSource/Foil002/source.json`。源 OBJ/生成 uasset 按仓库已有 Git LFS 规则管理。
材质创建时可能出现未连完节点的临时编译警告；脚本检查 `SARM_MLI_GRAPH_READY` 后
完整材质的编译，并回读实例参数、网格材质槽、三角数、UV 通道和尺寸。

## 验证与边界

- 自动测试：闭合拓扑、边方向、非退化三角形、20 个 UV 岛、等密度 UV、外移距离、
  顶部开口和底部覆盖、设备预留、接缝、精确绑定及无碰撞子组件。
- UE 编译及独立 runtime replay：原始模型/包覆后/单独包覆/背面检查。
- 使用相同静态零位姿态和独立检查灯光做 A/B；不修改平台太阳光配置。
- 未验证全部机械臂运动路径的视觉遮挡，不声称完成动态干涉认证。
- 没有完整重新 cook/package。源码项目 UE 重启即可；旧打包 exe 需要重新打包。
- 新增约 10.9 万三角形的视觉成本；未做长期 WebRTC 帧率基准测试。
- 仅视觉层，无物理接触：已有碰撞距离仍以原主体为准，视觉包覆有数毫米偏移。


## 2026-09-19 底部补覆

实测原始网格的下方凸出箱体：x=-0.15～0.45 m，y=0.065～0.465 m，
z=-0.69～-0.49 m。新增的包覆包括：

- 主体底板 z=-0.49 m，绕开下方箱体的占用区域（额外预留 8 mm）。
- 下方箱体的四侧及底盖 z=-0.69 m，边缘预留 5 mm，以保持分块接缝。
- 不覆盖 z=0 的顶部机械臂安装平台。

旧的 8 块侧面顶点、UV、三角形和材质分区完全保留（有自动回归测试）。新增
UV 岛向后续行扩展，部分 UV 大于 1；这是用于重复材质的有效坐标，不是 0～1
独占烘焙 atlas。保留物理纹理密度以及原有侧面的映射，避免侧面外观变化。

底部补覆不改变物理碰撞；视觉网格的几毫米外移仍是美术近似。
独立底部视角及原版对比位于工作区 `material_previews/SarmMLI/`。
正在运行的 UE 不会自动重新创建或加载包覆；重新启动对应场景后显示新版。
