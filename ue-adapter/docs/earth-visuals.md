# 地球地表、云层与云影

打开 `/Game/BSK/Maps/BskEnvironment` 即可预览，编辑与保存以源码 UE 工程为准。
地球 `Surface` 使用 `/Game/BSK/Environment/MI_EarthPreview`，`Clouds` 使用
`/Game/BSK/Environment/MI_EarthCloudsPreview`。在内容浏览器打开这两个材质实例，
勾选需要覆盖的参数、调整并保存，无需重启或运行后端。

## 当前默认值

| 实例 | 参数 | 默认值 | 作用 |
| --- | --- | ---: | --- |
| MI_EarthPreview | LandNormalStrength | 0.25 | 降低陆地地形法线的浮雕感；0 为无凹凸法线 |
| MI_EarthPreview | OceanNormalStrength | 0 | 海洋不使用陆地地形的凹凸法线 |
| MI_EarthPreview | OceanAlbedoScale | 0.65 | 海洋底色的线性倍率，不改变陆地；属于美术校准 |
| MI_EarthPreview | LandRoughness / OceanRoughness | 0.85 / 0.38 | 海陆粗糙度 |
| MI_EarthPreview | LandSpecular / OceanSpecular | 0.35 / 0.25 | 海陆反射，水面正入射 F0 约为 2% |
| MI_EarthPreview | NightLightIntensity | 100000 | 保留已有夜景增益 |
| MI_EarthPreview | CloudHeightRatio | 0.001 | 云高/地球半径；对应当前 6.371 km 云壳高度 |
| MI_EarthPreview | CloudShadowStrength | 0.65 | 材质云影强度，0 可对照关闭 |
| 两个实例 | CloudOpticalDepth | 1.6 | 云光学厚度；修改时两个实例应保持一致 |
| 两个实例 | CloudDensityPower | 1.35 | 从密度图提取浓淡；修改时两个实例应保持一致 |
| MI_EarthCloudsPreview | CloudOpacityScale | 0.88 | 云遮挡强度；提高到接近 NASA 参考图的云层密度；0 可临时查看地表云影 |
| MI_EarthCloudsPreview | CloudSunLuminance | 28000 | 参考日照下云层正入射亮度；提高云层白度和局部对比度 |
| MI_EarthCloudsPreview | CloudNightLuminance | 1 | 夜面云层的微弱底亮度 |
| MI_EarthCloudsPreview | CloudTint | 白色 | 云层色调 |

材质实例默认继承父材质，无需先手工填写所有参数。海洋没有加入随机噪声或虚构风场；
反射形状由现有粗糙度决定，云层遮挡提供有地理对应关系的变化。

## 渲染方式与边界

- 地表仍为 Default Lit。地形法线按海陆分别缩放，再归一化；昼夜分界继续使用几何法线。
- 云图作为线性密度数据读取（关闭 sRGB、Masks 压缩），不是作为彩色照片采样。
- 云采用 Unlit 透明材质，显式使用太阳方向着色，避免两盏太阳同时照亮透明云层。
  透明度取决于密度与观察路径，夜面保持遮挡，因此城市灯光不会无条件穿过浓云。
- 地表云影从地表沿太阳方向与云球壳求交，再采样云图。通过几何切线基计算，
  不依赖绝对世界坐标大数、固定经线或地球中心的硬编码。
- 云影衰减地表颜色与 Specular，不衰减夜景 Emissive；夜灯的遮挡由实际云壳沿观察方向完成。
- 这是球壳云和材质云影近似，没有体积云、自阴影或真实阴影贴图。
  云影也会影响地表间接光响应；掠射角 Fresnel 不等同于完整的直射光遮挡。
- 云光照方向会跟随现有 SkyAtmosphere 太阳方向，太阳距离、地球坐标与姿态的后端机制未修改。
  `CloudSunLuminance` 当前是保存的美术亮度，不自动跟随太阳照度、光色和日食状态变化；
  涉及距离大幅变化或光照协议控制的采集，应另验收这项近似。
- 云影高度应与云组件的相对缩放一致；手工改变云高或单独旋转云层时，需同步调整投影参数或实现对应 UV 偏移。

## 修改的资产与维护脚本

本次保存：

- `Content/Planets/Earth/M_Earth.uasset`
- `Content/Planets/Earth/M_Clouds.uasset`
- `Content/Planets/Earth/8k_earth_clouds.uasset`
- `Content/BSK/Environment/MI_EarthPreview.uasset`（新增）
- `Content/BSK/Environment/MI_EarthCloudsPreview.uasset`（新增）
- `Content/BSK/Maps/BskEnvironment.umap`（绑定两个实例）

`Unreal/BskUnrealRenderer/scripts/improve_earth_materials.py` 保存材质图的构建逻辑。
无需在每次启动时运行。UE 编辑器 Python 控制台中可使用：

```python
import runpy
from pathlib import Path
import unreal
script = Path(unreal.Paths.project_dir()).resolve() / "scripts/improve_earth_materials.py"
tools = runpy.run_path(str(script))
tools["apply_material_graphs"]()  # 只构建与编译，保留已有参数值，不保存
# 视觉检查通过，且 BskEnvironment 没有其他未保存编辑时：
tools["install_instances"]()    # 只保存以上资产与当前环境地图
```

脚本会保留同名实例的参数覆盖；遇到同路径的其他资产或不匹配的父材质会报错。
它依赖当前项目已有的地表、海陆粗糙度及夜景参数图，不是任意地球材质的通用转换器。
首次或大幅改造前应备份源材质与地图。运行时与打包不依赖 Python。

部署需同步以上资产及地图并重新 Cook，按[部署同步说明](../../server/docs/environment-deployment.md)
处理目标环境资源冲突。不要复制 Windows 的 Binaries 到 Linux，也不需要同步 Saved 内的诊断图片。

## 本次验证

2026-10-27 在 Windows UE 5.6 中检查了陆地法线对照、云影开关、整球日面和北美夜面。
材质最终编译完成（截图时 shader jobs 为 0），重复构建没有增加节点或重置参数。
无后端 Play 验证保存的两个材质实例被运行世界复用，夜景仍可见；停止后关卡无未保存修改。
没有在本轮执行完整后端、Linux Vulkan 或无界面采集验收。

本机修改前备份、参数读回与对比图位于工程的
`Saved/Diagnostics/EarthRealismPreview_20261027/`，该目录不提交仓库。
