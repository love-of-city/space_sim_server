# 配置驱动的目标星任务盒装配

入口是 `tools/preview_target_asset.py`：读取安装配置，生成独立模型和初态，检查装配不变项，导入缺失的 UE 网格，然后用真实 UE 渲染静态检查图。原始卫星文件不改写，输出目录必须是新目录。

目前已适配两颗资产，尚不支持把任意未标注模型直接变成可运行任务。新卫星先标注安装面、局部基准、可切除网格和保护部件，再复用生成流程。静态检查通过也不等于动力学、抓持、转运或视角覆盖通过。

## 已有配置

| 配置 | 安装规则 | 当前能力 |
|---|---|---|
| `config/target_assets/ground_validation_end_face.json` | 原地面验证星 −Z 端面；箱体和太阳翼同倍率 4.7；接口保持原尺寸；移除该端面附件并给任务盒后部开槽 | 自动装配、装配审计、UE 静态检查 |
| `config/target_assets/nasa_lro_visual.json` | LRO 金色箱体侧面；源模型倍率 0.07；仅切除所列箱体网格的任务盒开口 | 仅视觉适配；无校准的质量、惯量、碰撞和捕获接口 |

倍率相对于各自源文件，尤其 LRO 的 0.07 不是经过校准的真实物理比例。地面验证星采用既有 `fixed_mass` 缩放约定，也不是重新完成物理参数辨识。

两种装配都保持任务盒世界位姿和 L 型关系、操作星、原操作臂初态及原安装相机。地面验证星使用已保存的对接臂关节种子；它不是任意缩放参数下都成立的自动抓持解。LRO 没有借用其他卫星的接口伪装成已抓持状态。

## 配置与坐标约定

- `source_model`、`baseline` 相对服务端项目根目录解析；源 XML 的 SHA-256 不一致时拒绝生成。输出 XML 的机器绝对路径应在目标机器重新生成，不能直接搬到 Linux 使用。
- `mount` 指定安装面、平面基准与开槽范围。地面验证星 `rotation_columns` 是目标星坐标轴在任务盒局部坐标中的三个列向量；LRO `basis_source` 是安装面坐标轴在源模型坐标中的三个列向量。
- 固定任务盒世界位姿后，通过安装面基准反解卫星的整体姿态和位置。不会为了贴合新卫星而移动任务盒。
- 地面验证星 `panel_size_m` 记录现有 0.45 × 0.55 m 任务面的尺寸，不能通过修改此字段缩放任务机构。`clearance_m`、`recess_depth_m` 控制让位；原结构部件和保护名单由该资产适配器识别。
- LRO `center_source` 使用源模型坐标，`pocket_low_m/high_m` 使用缩放后目标星局部坐标；`pocket_meshes` 是允许切除的原网格名。其他模型若有多层变换或不同源格式，需要补充适配器，不能套用 LRO 网格名。
- `constraints` 和 `missing_capabilities` 记录资产用途边界。`asset-audit.json` 独立检查实际生成结果，不直接以配置声明判定成功。

## 生成和 UE 检查

在服务端根目录运行，使用独立几何 Python 环境，需有 NumPy、SciPy、trimesh 及项目依赖；不要与实时 Basilisk 原生库混入同一解释器。UE 工程需已编译 BSK 渲染插件，资产目录 JSON 指向该工程已有的资产。

```text
python tools/preview_target_asset.py --profile config/target_assets/ground_validation_end_face.json --output <新输出目录> --adapter <UE适配器根目录> --unreal <UE安装根目录> --catalog <已有UE资产目录.json>
```

换用 `config/target_assets/nasa_lro_visual.json` 就生成第二颗卫星。`--catalog` 可以重复，用于复用多个已导入资产目录。缺失的视觉网格会自动生成导入清单、调用 UE 导入和配置，并核对导入结果。匹配使用源网格哈希和实际 `.uasset` 文件存在性。

仅生成模型时：

```text
python tools/build_target_asset.py --profile config/target_assets/ground_validation_end_face.json --output <新输出目录>
```

已有生成目录可使用 `--reuse-built` 接续检查，要求配置哈希一致；新生成清单也校验模型和初态哈希。若已有 `inspection.bskrec`，请用新目录重跑，保留原结果。

产物包括：`generation.json`、`preheld-dual.xml`、`initial-scene.json`、`assembly.json`、`asset-audit.json`、`inspection.bskrec`、`UE-inspection.png`、`pipeline.json` 和各阶段日志。流水线失败会保留已经生成的文件和日志，不把失败阶段记为通过。

`inspection.bskrec` 是静态位姿录制，四路安装相机取自原模型；主视口是独立自由观察视图。UE 截图中的 “LIVE” 是渲染器画中画标签，不表示这份录制进行了动力学积分。UE 默认截图显示两个画中画，其余相机仍在录制清单中，可按 4–7 切换。

需要交互查看时，在输出所在机器启动：

```text
<UE>/Engine/Binaries/Win64/UnrealEditor.exe <适配器>/Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject -game -BskPort=5589 -BskReplay=<输出>/inspection.bskrec -DDC=InstalledDerivedDataBackendGraph
```

W/S、A/D、Q/E 移动，鼠标转向。Linux 使用 `Engine/Binaries/Linux/UnrealEditor`；代码已按平台选择可执行文件，但本轮仅在 Windows 验证。此入口不注册到网页的动力学场景索引，也不修改正在使用的八项任务配置。

## 验证与待解决项

本轮两颗资产均通过独立装配审计并取得真实 UE 截图。静态 XML 正运动学在地面验证星上与 MuJoCo 计算的 62 个部件位姿一致。配置拒绝条件、关节锚点与参考角处理、开槽几何由定向测试覆盖。

LRO 尝试原生模型编译时发生过 Windows 访问异常，失败日志保留在本机输出目录。最终可视化走不加载动力学插件的静态 XML 流程，因此不能据此宣称其原生运行已修复。LRO 预览仍有太阳翼遮挡操作区，布局未验收；地面验证星也未在本轮重跑完整任务或全轨迹干涉检查。

扩展资产库时分别保存“生成成功”“UE 渲染成功”“视角与装配合格”“动力学可运行”“任务通过”状态。先为每颗卫星保存一份人工确认的安装配置，再批量调用相同入口；自动找到安装面、自动选比例以及自动求解对接臂姿态尚未实现。

## 连续封闭外壳的生成与验收

地面验证星配置新增 `hull_exterior=closed_except_task_panel_footprint`。生成器保留
原有壳板，在原箱体包络内侧增加连续薄壁封板；安装面只开一个任务盒安装口。
开口由固定任务盒边框的真实视觉/碰撞几何包络计算，不能用 450×550 mm 名义
尺寸代替。移动抽屉、插头不用于扩大开口。当前封板厚 1 mm、内缩 0.1 mm，
安装缝隙为 0.2 mm；后部 107 mm 让位槽不允许把第二个外表面开穿。
原质量和惯量约定保持不变，此处没有新增质量属性辨识。

`hull-closure-audit.json` 将六个表面按实际补板边界精确分区，拒绝非任务区漏板、
任务口被堵、超出箱体以及可见/碰撞补板不一致。`audit_hull_clearance.py` 显式
检查补板与其他硬件的距离，包括正常接触过滤会排除的几何对；不能用碰撞排除
掩盖新补板穿透。源任务盒的原箱体接触排除则按原约定映射到新 owner。

完整本地流程可追加两个参数：

```text
python tools/preview_target_asset.py --profile config/target_assets/ground_validation_end_face.json --output <新输出目录> --adapter <适配器根目录> --unreal <UE安装根目录> --catalog <已有资产目录.json> --native-python <独立Basilisk解释器> --install-index run/imported_scene_library.json
```

顺序为：生成 → 装配/六面封闭审计 → 显式补板间隙 → 两秒原生保持及独立校验 →
UE 导入/实拍 → 本地场景登记。任何阶段失败都保留失败目录并停止后续阶段。
`--install-index` 只更新第 2、4 类现有配置，保留原索引备份，且仍标记完整任务
未验收。不传该参数则只生成独立资产。安装要求有效的原生保持与 UE 实拍证据。

`inspection_view` 只配置静态检查主视口，不改变安装相机。统一旋转检查坐标系
等价于改变观察方向，各部件相对位姿保持不变。该截图使用检查光照，不是
传感器数据或实时轨道动力学画面；现场截图另从网页相机流取得。

## 两星任务盒内部随机初始化

参考 `origin/main` 的布局池流程（核对版本 `98449ee`），复用 13 个安装单元
XY/yaw 生成、底板重建、独立孔套、UE catalog 以及 `task-box-layout-selection-v1`
选择流。两只任务盒的安装框架不动，移植的是内部布局；卫星、太阳翼、接口、
双臂和相机继承当前场景。自由插头随目标星的取件座移动，操作星的取件座保持空置。

本地 `/2` 操作空间检查比上游增加了实际夹爪方向及双向退出空间，不覆盖成
上游较旧规则。当前池明确为 `native-3d-candidate`：通过安装几何、底板孔套、
拼装后初始碰撞检查才可登记，但完整机械臂路径仍未验收，禁止开启数据采集。
原自动采集 `LayoutPool` 的校验入口保持不变，不接受这个候选池。

在服务端根目录，使用独立几何 Python：

```text
GEOMETRY_PY tools/prepare_dual_taskbox_pool.py --base <已确认封闭外壳目录> --base-catalog <该模型的inspection.catalog.json> --adapter <UE适配器> --unreal <UE根目录> --platform-python <平台Python> --output <新目录> --count 2 --seed 2026110300
```

入口依次生成布局、检查原生孔位/碰撞和孔套网格、导入独立 UE 资产、移植到
两星任务盒、检查初始接触和封闭外壳、通过实际 Basilisk 模型加载，再原子更新
本地第 2、4 类场景索引。组合时为不同文件的重名网格建立唯一文件名副本，
避免 Basilisk VFS 按文件名装载时冲突；UE catalog 按相同源文件哈希保留资产映射。
两套布局组成 A/B、B/A 两种配对；启动随机选择的是预构建配对，不声称两侧
所有组合均已生成。`--existing-donor <目录>` 可重复传入已有的完整检查/导入产物，
登记仍核对报告和文件哈希，不接受失败产物。发布前保留原索引备份。

网页选择第 2 或第 4 类任务，勾选“随机初始化两个任务盒的内部布局”，填写
种子后启动。勾选时自动关闭数据采集；关闭随机化开关仍用原固定布局。
API `/api/scenes/start` 示例请求：

```json
{"template_id":"library/04_cross_transfer/aviation_plug_insert","randomize_task_box_layout":true,"seed":101,"dataset_capture":false,"capture_rate_hz":2,"ik_rate_hz":100}
```

实例保存 `task_box_layout_randomization`（选择种子、配对 ID、池/模型/catalog
哈希、两侧生成种子与位置/朝向）及 `task_box_layout`。复位使用保存的实例；
重启同一种子和同一版本池复现同一配对。没有有效池、源文件改变、请求数据采集
都会明确拒绝，不静默退回固定布局。检查工具的 `--preview-candidate` 只允许几何
检查/资产预览，不赋予完整路径或采集验收。Linux 路径由参数解析，本轮仅本机验证。
