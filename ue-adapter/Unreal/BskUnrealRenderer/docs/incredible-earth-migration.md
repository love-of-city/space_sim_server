# IncredibleEarth80K 真实尺度场景迁移

首次迁移日期为 2026-10-29，修复和视觉对齐于 2026-10-30 继续进行。视觉来源为用户指定的 `/Game/IncredibleEarth/IncredibleEarth80K` 主关卡，活动入口保持 `/Game/BSK/Maps/BskEnvironment`，由原有 `BskEnvironment` 和 Runtime 插件管理后端接管。范围是迁移原场景并适配真实尺寸，不增加真实地形、GIS 或地表行走功能。

## 2026-11-04 发布集成

`Content/IncredibleEarth` 已改为外部包；[恢复与分发说明](../../../../docs/guides/external-environment-assets.md)提供版本、完整性检查及部署步骤。关卡、项目材质和代码仍在 Git/LFS 中。下面记录的编译、截图和采集证据属于迁移前的本机版本，不能作为此次与远程模块化 Runtime 合并后的验收结论。

## 迁移时的状态与验收范围

当前对齐包含真实月球预览半径、源太阳显示设置、独立于物理太阳角度的 3° 显示圆盘、原大气内嵌比例及长度恢复，以及按实际曝光差修正的自发光补偿。下文先说明当前实现，再保留各阶段历史证据。历史的 35 项测试、24 张 RGB、8 个视角和旧关卡哈希均早于当前改动，不能作为当前实现全部通过的依据。

当前 12 个适配资产已保存，UE MCP Data Validation 结果为 12 valid、0 invalid，主检查通过并记录于 `validation_calibrated.json`。Editor 最新编译及链接通过，见 `build_editor_retry.log`；8 项 `BskUnreal.Environment` 自动化全部通过，见 `automation_verified.log`、`automation_verified.json`。第一轮曾有 1 项测试失败，原因是测试夹具的星空组件没有网格、太阳遗漏发光属性；补齐后重跑通过，并检查初始照度和可见性。最终 Game 构建通过，见 `build_game_final.log`。6 相机、双会话 Runtime 图像检查完成，证据为 `smoke_runs/20261030_181215`：退出后复核共 198 张 RGB 及配套元数据，两会话六路相机均收到 frame 0，所有元数据引用的图像均存在。实际画面覆盖地球昼夜面、大气边缘、极光、月球和太阳；相机世界位置另按元数据核验。第二会话没有采完全部计划帧，因此本轮证明重连与视觉采集正常，不作为满帧率或吞吐验收。

当前证据集中在 `Saved/IncredibleEarthMigration/alignment_20261030`。`source_render_verified` 和 `target_render_verified` 均已完成，各包含 6 个视角及 HDR 诊断，截图前等待资源、着色器并预热渲染帧。源、目标实际预曝光分别约为 `0.5333333` 和 `0.00003125`，支持当前亮度适配。脚本已修正 UE Python 一元负号可能原地修改 Vector 的问题，并检查相机朝向、昼夜相位、太阳方向与地平线切点。

太阳同视角对照中，核心及暖色光晕外观接近；`sun_image_comparison.json` 记录半径 35–55、55–100、100–160 像素环带的平均绝对 RGB 差分别为 0.4225、0.7473、0.8499（按 0–255 通道值计算）。背景星点存在差异，不能宣称整张图逐像素一致。地球昼面光照相位一致，但源目标地理朝向不同，因此这些图不作为同一地理区域的像素比较。源月球朝阳面本就饱和，目标保留此视觉特征；检查确认目标地球、月球半径分别为 6371 km、1737.4 km。

## 场景和可编辑资产

- 地表、动态云层/闪电、夜侧城市灯光、夜侧大气边缘、Niagara 极光、星空和月球使用 IncredibleEarth 资源。
- 太阳通过原主关卡的方向光大气圆盘绘制；星空材质中的 `SkyAtmosphereLightDiskLuminance` 节点提供圆盘。旧 SpaceFX 太阳粒子已解除引用，保留不可见的轻量绑定对象以兼容原控制器接口。
- 原始资源保存在 `Content/IncredibleEarth`。此前逐文件校验确认来源、目标副本及首次迁移记录一致：108 个文件，共 17,901,852,851 字节，约 16.67 GiB。该校验属于相应阶段的资产完整性证据，不代表每次后续改动都重新计算了所有哈希。源电影关卡、自动旋转与轨道控制蓝图不接管平台活动关卡。
- 适配材质和极光资源位于 `Content/BSK/Environment/IncredibleEarth`，主要编辑入口为 `MI_Earth80K_Main`、`MI_Clouds_Main`、`MI_NightAtmosphere_Main`、`MI_Stars_Main` 和 `MI_Moon_Main`。太阳、曝光、预览相机和天体绑定仍由 `BskEnvironment` 编辑。
- 源 `PostProcessVolume_2`、`ColorCorrection` 中已启用的白平衡、调色、Film、Bloom、色差、暗角和运动模糊已迁入；保留源体积顺序、优先级与混合属性，未启用的源参数不会额外启用。本轮没有修改项目 Config，也没有全局恢复源 AO、反射配置；保留平台 Lumen、TSR、手动曝光及物理光照。
- 源 `MoonLight` 是附着月球的蓝色艺术补光，保留源相对方向和颜色，照度按物理日照比例适配为 `0.5 × (132821.5 / 13) ≈ 5108.519 lux`。它只使用光照通道 2，地球各层使用通道 1、2，月球使用通道 1，航天器使用原通道 0，因此该光不照亮月球和航天器。这不是新增月光物理模型，也不能用它解释月球自身发暗。

## 尺度与大气

| 项目 | 当前值或规则 |
| --- | --- |
| 地球几何半径 | 6371000 m，即 6371 km；直径 12742 km |
| UE 几何单位 | 100 cm/m，地球网格半径 637100000 cm |
| 源地球网格实测半径 | 1081.9337830737304 km |
| 放大倍数 | 5.888530425494475 |
| 保存预览月球半径 | 1737.4 km |
| 保存预览地月中心距离 | 384400 km |
| SkyAtmosphere bottom radius | 6356.668457 km，保留源底面相对地表内嵌比例 |
| SkyAtmosphere height | 147.213257 km |
| AerialPerspectiveStartDepth | 5.88853025 km，源值 1 km 同比例放大 |

地球 Actor 默认缩放为 1，半径写入网格组件缩放，避免 manifest 接管覆盖 Actor 缩放时二次放大。保存预览球心沿用平台 `(0, 0, -687100000) cm`，运行时由后端位置和姿态覆盖。月球预览不再保留早期约 1009.594506 km 的错误半径；后端接管仍按 manifest 声明半径绘制。

云壳约高 11.467 km，夜侧光晕网格约高 59.440 km，均保持源层间比例。源 SkyAtmosphere 底半径为 1079.5 km，本就略低于源网格表面，因此目标大气底面不能简单强制设为 6371 km。大气长度按放大倍数适配，散射/吸收系数按比例减小，以保留原光学厚度。

Runtime 捕获保存预览的大气底半径、高度和 AerialPerspectiveStartDepth；后端改变地球半径或几何单位时，三者按 `后端半径 / 保存半径 × 后端 cm/m / 保存 cm/m` 缩放。manifest 移除地球或释放后端控制时恢复保存长度，避免接管之后丢失源大气内嵌比例或恢复不完整。

地表和云层材质的 `EarthSize`、世界空间 `Displacement` 按同一放大倍数适配。源地表位移为 0，仍为 0；云层位移从 4 变为约 23.554121。未加入真实地形。

`NS_Aurora_Attached` 为源 Niagara 系统的副本，改用局部空间以跟随后端地球运动和姿态。极光组件独立于父级缩放，通过 `User.SystemScale` 获取地表实际缩放，剔除范围随半径更新。材质在原 `World → Local` 与 `ObjectScale` 之间加入由 CPD 索引 0 提供的尺度除数，恢复原组件缩放参与的坐标换算。两处纹理长度以及 `User.Speed`、`User.Speed2` 同比例适配，保留原颜色、遮罩、纹理分布和动画节奏。重复初始化使用 `reinitialize_system()`，避免源永久粒子重复激活后堆积。

## 月球和曝光补偿

月球发暗的迁移错误是只查找 `MakeMaterialAttributes_0`，漏掉月球实际位于 `_2` 的自发光连接。现在按节点类型和实际连接查找，曝光补偿只乘自发光，保留源基础色、法线、月相遮罩和 `EmissiveBoost=3000` 等参数。报告记录 `emissive_compensation_applied` 及实际读回倍率，避免未接入却报告成功。

平台相机保持 ISO 100、1/125 s、f/16、曝光补偿 0；源相机为 ISO 100、1/60 s、f/4、曝光补偿 +9。实际 HDR 预曝光测量也确认二者分别约为 `0.00003125` 和 `0.5333333`。因此当前自发光补偿为：

`BSKEmissiveGain = 0.5333333 / 0.00003125 ≈ 17066.6667`

这取代早期按日照比例得到的约 10217.03846。日照比和相机曝光比不是同一量；前者继续用于物理照度适配，后者用于源城市灯、夜侧光晕、极光、星点和月球等自发光外观。脚本和保存资产已采用该曝光比，`validation_calibrated.json` 检查通过；最新源/目标截图已包含月球等视角。

原项目月牙本来就有明显 Bloom 与高亮区饱和。修复目标是恢复原有视觉效果，不能将其描述为真实月面辐射定标，也不承诺朝阳面保留全部纹理细节。

## 太阳显示与物理光照

按用户要求，保存预览的太阳显示恢复源主关卡参数：视直径 3°、软角 0、圆盘 RGB `(1, 0.5002009868621826, 0.249767005443573)`，保留源 Bloom。主太阳开启色温，值为 6500 K，并将半透明投影恢复为源设置 `false`。平台日照照度继续由物理模型提供，不把源 13 lux 写入航天器光照。

当前 `M_Stars_Physical` 将星点与动态大气太阳盘分开：星点乘 `BSKEmissiveGain`；太阳盘使用 `BSKSunVisualAngularDiameter` 控制显示大小，并乘独立的 `BSKSunDiskRadianceScale`。太阳盘已响应物理太阳照度，不能再次乘完整的星点曝光倍率。为补齐日照比与相机曝光比的差额，材质另有 `BSKSunVisualExposureGain ≈ 1.6704`，只作用于太阳盘；该适配已保存，参数检查通过，最新太阳对照的核心及暖色光晕外观接近。

后端接管时，主太阳和天体太阳方向光的 `LightSourceAngle` 继续按真实太阳半径、距离计算，典型值约 0.533°，用于物理阴影等计算。星空材质中的显示圆盘保持保存的 3°，不随该物理角度缩小。Runtime 以 `物理圆盘立体角 / 显示圆盘立体角` 更新 `BSKSunDiskRadianceScale`，抵消扩大显示遮罩时带来的圆盘总能量增加；实现使用稳定的正弦平方比。方向、距离平方照度与日食遮挡仍由原星历逻辑控制，材质不再额外乘一次 SolarVisibility。

新后端会话仅在星空材质具备两个运行时参数时创建动态材质实例，写入 `BSKSunVisualAngularDiameter` 和 `BSKSunDiskRadianceScale`；独立曝光参数保留资产作者设置。相同会话复用实例，新会话先恢复原材质，返回预览时恢复保存材质与预览光角。缺少这些参数的旧材质保持原行为。日志分别记录物理 `angular_diameter` 与显示 `visual_disk_diameter`，便于检查两者是否混淆。

## 历史修复与证据

以下均为发生时的历史结果，不替代上文当前验收。

### 2026-10-30 上午：几何、极光和完整性修复

修复了根网格组件变换被重置导致月球移位的问题，恢复地月距离和星空约 23.0790217° 的 yaw，并使用一致的半径计算防止重复迁移漂移。当时月球预览仍沿用平台约 1009.594506 km 的半径；当前已改为 1737.4 km。

跨关卡读取的 `StructBase` 可能借用源对象存储，源关卡卸载后曾使后处理对比度向量变为全 0，画面出现灰色噪点。`property_snapshot()` 已对结构体调用 `.copy()`，覆盖后处理、大气、太阳、极光颜色和月球补光。源 FFT Bloom 保留。另修复同会话 manifest 移除地球后的预览恢复，以及 RGB 第 0 帧资源初始化时序。

当时 Editor、Game 构建通过，35 项 `BskUnreal` 自动化通过，保存关卡检查通过，53 项项目资源依赖存在。证据为 `build_editor_repair.log`、`build_game_repair.log`、`automation_repair_mcp.json`、`validation.json` 和 `repair_hash_validation.json`。当时独立持久化 commandlet 包含 `BSK_ENVIRONMENT_PERSISTENCE_OK`，旧关卡 SHA-256 为 `67B36118949BF3AEE8106E2ACEA5F1A84A35D25C5CE5D7A7BC8E0A8B008BFCAE`；该哈希不代表现在修改后的地图。

一次在活动编辑器内执行临时关卡持久化检查触发 `World Memory Leaks` 断言；`validate_environment_level.py` 因此加入 commandlet-only 防护。不要在用户活动编辑器内重复执行该检查。

历史烟测 `smoke_runs/20261030_111134` 每会话发送 60 帧，两个会话共产出 24 张 RGB，第 0 帧均交付。历史多视角记录 `render_repair_20261030_105525/report.json` 包含约 98.7 秒、8 个视角，极光粒子数 100、地球移动旋转后的中心误差 0。这些检查早于当前月球、太阳和曝光对齐。

### 2026-10-30 月球自发光与太阳偏白修复

`moon_sun_review_20261030_114040` 记录补接月球自发光、太阳预览恢复 3° 和同视角对照；修改前备份为 `moon_sun_backup_20261030_114040`。该阶段使用的是旧约 10217 倍补偿，并未修改 C++。

`sun_color_20261030_161441` 保存源参数和曝光诊断；`sun_color_adjust_20261030_163543` 保存接线修复、`validation.json`、`after_same_view.png`、`sun_comparison.png` 和备份。该阶段已纠正将整个 `星点 + 太阳盘` 输出乘星点倍率的重复增亮错误，暖色光晕恢复；曾用于诊断的临时圆盘倍率已撤销。该阶段也没有修改 C++，但当前新增的物理角度/显示角度分离和大气恢复确实涉及 C++，不能沿用旧阶段的“未修改运行时”结论。

### 2026-10-30 全量复审的边界

`full_reaudit_20261030_165826` 保存 `source_disk.json`、`target_live.json`、`asset_integrity.json`、`ephemeris_smoke.json` 和 `smoke_runs/20261030_172021`。该烟测两个会话共产出 10 张 RGB，证明加载、重连与采集链路可运行，但相机未有效覆盖待验收天体。

该 RGB 相机沿 +Y，位置约 `(0, -5.6, 2) m`，水平视场 60°、垂直约 35.98°。地球顶部位于下方约 21.99°，超过垂直半视场约 17.99°；月球水平约 84.29°，超过水平半视场 30°；太阳在相机后方。主视口 `runtime.png` 又是另一轨道视角。RGB 几乎全黑、灰度约 0–12，非空文件检查不能据此证明地球、月球或太阳外观正确。当前验收需另用明确朝向目标的源/目标相机，并检查实际画面。

## 加载、验证和维护

用户已确认首次冷加载生成大型缓存可能较慢，缓存建立后再次加载快得多。取消固定等待 15 分钟，改以着色器编译完成、异步资源加载完成及连续流送帧后的实际画面为依据。未取得的 VT 驻留计数不能作为全量驻留证明。

工程启用虚拟纹理，并沿用源 `r.VT.TileSize=512`、禁用 Crunch 和 TextureFormatOodle 的既有迁移配置。新增 Niagara 模块/插件依赖后需要重编译 Editor 与 Game。当前代码新增 `SunVisualDiskDoesNotChangePhysicalLighting`、`SavedAtmosphereLengthsSurviveBackendScaling` 回归检查，覆盖显示与物理照明分离、材质会话恢复、大气长度缩放与恢复。

历史 RTX 4060 Laptop 8 GB、2345 × 1833 编辑器视口出现约 682 MB 显存超预算，属于当时硬件与视口的性能记录，不能外推所有分辨率或打包运行。此前 Python 104 项测试与 19 个子测试记录未在本轮重新执行。未进行本轮 Linux 运行或打包验收。

`scripts/migrate_incredible_earth.py` 是显式迁移工具，启动不会自动执行；再次运行会重写适配材质和相关环境图层，需先备份后续美术调整。`validate_incredible_earth.py` 检查保存关卡，`validate_moon_sun_appearance.py` 检查月球/太阳材质与参数；它们不替代实际画面验收。打包只需活动 `BskEnvironment` 及依赖，不要将源电影关卡全部加入活动地图列表。

## 回退

首次修改前备份为 `Saved/IncredibleEarthMigration/backup_20261029_225505`，包含配置、项目描述、Runtime 插件、`Content/BSK`、脚本及当时 Git 状态和用户原有未提交差异。上午修复前另有 `repair_backup_20261030_100020`，该备份只为 `DefaultEngine.ini` 提供 Config 哈希基线，不覆盖其他 4 个 Config 文件；此基线仅描述当时检查范围。本轮 Config 保持现状，当前对齐证据、备份与日志以 `alignment_20261030` 及其中实际记录为准。

关闭目标工程后逐项恢复对应阶段涉及的资产、源码和配置，再重编译；不要整目录覆盖用户后续仿真或模型改动。新增源资源和适配资源在解除引用后可另行归档。备份、测试日志、缓存、截图、`Saved`、`Intermediate` 和编译产物不提交仓库；项目自有 `.uasset`、`.umap` 遵守 Git LFS 规则；外部 IncredibleEarth 源包单独分发并保留资产许可。
