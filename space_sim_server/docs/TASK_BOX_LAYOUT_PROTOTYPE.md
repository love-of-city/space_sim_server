# 操作台各安装单元独立随机布局

该原型在仿真初始化前生成独立 MJCF，按随机种子重新布置台面上的 **13 个安装单元**。
位置覆盖可用台面；运行期间仍固定安装。可选各单元绕台面法线独立随机转向；
插入/开合等初始状态、机构内部关系、关节及驱动参数和既有 20N 插拔摩擦规则保持原配置。
单布局生成默认不随机转向，以便复现既有布局。已增加[自动采集布局池及网页开关](COLLECTION_LAYOUT_RANDOMIZATION.md)，需配置并校验布局池后启用。

## 独立布置范围

| 安装单元 | 数量 | 保留的装配关系 |
| --- | ---: | --- |
| 航空、导向、圆柱插头初始座 | 3 | 各自带着初始插在座内的插头 |
| 三种插头的 40.1 / 40.5 目标孔 | 6 | 每个孔独立，精细孔壁碰撞体与 site 同步 |
| 螺栓存放孔、螺纹装配孔 | 2 | 存放孔带着螺栓，装配孔带着螺纹 SDF |
| 抽屉机构 | 1 | 抽屉、卡扣、安装框和底板开口同步 |
| 锁紧机构 | 1 | 锁紧杆、防脱紧固件、挂钩、限位片与安装支撑同步 |

原来共用的两块安装板按孔位之间的中分面切成 11 个独立安装片。
切面离孔中心至少 25 mm，孔中心周围半径 22 mm 内的原始配合几何保持。
插头、螺栓、抽屉和锁紧机构本体的网格保持原样。不是整块操作台或整块安装板共同平移。

底板填补旧开口，并为每个安装片/抽屉在新位置重新开口，锁紧支撑一起搬迁。
碰撞体先作凸分解，再按实际开口作半空间切割，防止凸近似堵住新孔。
精细孔壁与粗安装片分属不同碰撞 body，粗凸体的接触排除不会同时关闭孔壁接触。

## 生成与验证

以下从服务端根目录执行。所有输出放在仓库之外。
几何 Python 需要 `numpy trimesh manifold3d scipy networkx coacd`；
动力学和录制使用原 Basilisk / MuJoCo 环境，二者不要混用原生库。

```bash
LAYOUT_PYTHON=/home/lyh/space_enviroment/envs/layout-geometry/bin/python
SIMULATION_PYTHON=/home/lyh/space_enviroment/envs/spacesim/bin/python
LAYOUT_SOURCE=/home/lyh/space_sim_server/space_sim_server/model/SARM/platform/sarm_20261012.xml
LAYOUT_MODEL=/absolute/path/outside-repository/new-seed/layout.xml

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 "$LAYOUT_PYTHON" \
  tools/build_independent_task_box_layout.py \
  --source "$LAYOUT_SOURCE" --output "$LAYOUT_MODEL" --seed 20261021

"$SIMULATION_PYTHON" tools/check_independent_task_box_layout.py \
  --source "$LAYOUT_SOURCE" --layout "$LAYOUT_MODEL" \
  --report /absolute/path/outside-repository/geometry-check.json

"$SIMULATION_PYTHON" tools/validate_plug_retention.py \
  --model "$LAYOUT_MODEL" --case extraction \
  --case partial_insertion --case partial_extraction \
  --output /absolute/path/outside-repository/force-check.json
```

输出包括模型、配套姿态配置、几何、`.layout.json` manifest 和布局包络计划。
`--seed` 可以复现坐标；`--placements FILE` 可提供全部 13 项的 `[dx,dy]`（米）。
`--plan-only` 只生成几何切分和布局计划，不生成可运行模型。
源模型与源资产不被覆盖。输出 MJCF 必须是新文件，各布局应使用独立输出目录。

### 位置与朝向同时随机

增加 `--yaw-range-deg 180` 后，每个单元在台面内独立采样 `[-180°,180°]` 转角，
以及连续的 XY 位置。可设更小的角度半范围，例如 `--yaw-range-deg 30`。
此处只改变绕台面法线的安装角，不增加倾斜、翻转或任何机构内部关节随机化。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 "$LAYOUT_PYTHON" \
  tools/build_independent_task_box_layout.py \
  --source "$LAYOUT_SOURCE" --output "$LAYOUT_MODEL" --seed 20261023 \
  --target-mounts inset --pickup-mounts inset --yaw-range-deg 180
```

新 manifest 使用 `task-box-layout/3`，保存原安装锚点、XY 位移、各单元转角（弧度）、
绕锚点旋转再平移的变换，以及新运动包络。`--placements FILE --yaws FILE`
可重放全部 13 项显式位移/转角；不能同时与 `--yaw-range-deg` 混用。
旧 `/2` 平移布局和 `/1` 两板布局仍保留兼容。

初始座带着插头/螺栓一起旋转，目标孔的套筒、精细孔壁、site、螺纹 SDF 与底板开口
同步旋转；锁紧支撑、抽屉框架与机构本体也使用同一个安装变换。
自由物体的初始位姿改变，但它们相对各自起始座的装配关系保持。

间距检查采用旋转后保守运动包络的轴对齐外接矩形，保持 5 mm 台面边距和 8 mm 包络间距。
大机构先采样，重试次数有限；这种方式可能排除一些能紧凑放置的方案，
也不保证所有合法布局具有相同概率。抽屉/插拔脚本读取实际抓取和目标朝向；
这些目标坐标正确不等价于任意布局都能完成机器人任务。

底板碰撞开口的轮廓在半空间切割前清除 10 nm 以下数值细边，
防止旋转 CAD 轮廓的近重复边令切割残留大块碰撞体。验证仍检查导出网格、
完整底板填孔、原生孔道与既有 20N 摩擦；不会以提高穿透容差接受受阻的孔。

位置和转角都应作为 `layout_id` 的组成部分参与训练/验证/测试分组。
正式采集前仍需增加可达性与完整任务筛选，并使相似目标孔的可见标记和任务指令一致。

### 目标孔窄孔沿版本

生成时增加 `--target-mounts inset`，可移除 8 个目标孔外围的不规则安装片，只保留
原始套筒、完整孔口及窄孔沿。3 个起始插座、抽屉和锁紧机构仍使用原方案。
默认 `--target-mounts plates` 可以继续复现整片切分版本。

起始插座也可单独增加 `--pickup-mounts inset`，采用同样的窄孔沿处理。
它只裁切 3 个起始插座的外围安装片和粗碰撞体；原始套筒、孔口和座内插头的
初始相对位置、姿态及 20N 插拔摩擦配置保留。两个选项可独立使用，默认均为 `plates`。
若要 11 个孔单元都采用窄孔沿，指定 `--target-mounts inset --pickup-mounts inset`。

窄孔沿外轮廓由原始套筒和完整孔口的凸包向外扩张 1 mm 得到。导向孔的键槽也参与
轮廓计算，避免只按下方套筒裁切时切断键槽外侧孔沿。孔口高度仍为台面上方 5 mm，
孔深、插入定位、924 个精细孔壁碰撞体和螺纹 SDF 保持原值，并随对应孔独立迁移。
本版本不是完全齐平的台面孔。

台面填补旧的大开口，并按套筒/孔口包络外扩 0.25 mm 为各孔重新开口，孔沿覆盖缝隙。
可见安装片及原有粗碰撞体同时裁切，不会留下原安装片的隐形碰撞。
填孔后的原台面先验证与 450×550×25 mm 规则实体的差异，再替换其板体；
上方机构仍保留。碰撞采用规则板体、迁移后的原始上方凸碰撞体及逐孔精确切割，
避免对带大量小圆孔的布尔网格执行不稳定的 CoACD 分解。
窄孔沿与台面使用双精度 OBJ，分别在 0.1 微米和 1 微米容差内处理布尔细缝；
逐面法线保留板面/孔壁的硬边，
避免跨孔壁平均法线导致平板出现三角形明暗。UE 导入核对全部 12 项的 LOD0 三角面数。

```bash
"$LAYOUT_PYTHON" tools/build_independent_task_box_layout.py \
  --source "$LAYOUT_SOURCE" --output "$LAYOUT_MODEL" --seed 20261021 \
  --target-mounts inset

"$LAYOUT_PYTHON" tools/check_inset_target_geometry.py \
  --layout "$LAYOUT_MODEL" --report /absolute/path/outside-repository/inset-geometry.json
```

窄孔沿检查会读取实际导出网格，与源模型核对孔口、台面下方套筒、孔沿外粗碰撞体、
新补台面及套筒/台面干涉。它使用网格格式的数值容差，不能替代上文原生 MuJoCo
的孔道/初始状态检查和 20N 摩擦动力学验证。
检查器会按 manifest 中启用的样式审计目标孔、起始插座或两者；原生检查还会核对
3 个起始插座中插头的初始就位及沿轴向拔出 4 个位置时的底板干涉，共 36 个通道样本。

本机窄孔沿证据目录为 `/home/lyh/space_enviroment/runs/task-box-inset-targets-20261022/`。
最终模型是 `model-flat/layout.xml`，配套资产为 `ue-assets-flat/layout.catalog.json`。
`inset-geometry-flat.json` 的 8 项网格检查和 `native-geometry-flat.json` 的 24 项
插入通道检查均通过，底板最大穿透为 0；全部 6102 项渲染几何与编译模型匹配。
`normal-fix-physics-equivalence.json` 验证法线修正前后的惯性、接触、关节和驱动数组
完全一致。`recording-flat/summary.json` 记录真实 UE 下脚本 `drawer_cycle` 成功：
34.42 秒、每路 1722 帧、抽屉最大拉出约 80 mm，再回推并释放。
`target-holes-before-after.png` 使用相同安装位置和机位的真实 UE 首帧。

3 个起始插座也改为窄孔沿的证据目录是
`/home/lyh/space_enviroment/runs/task-box-inset-pickups-20261022/`。
最终配置使用 `model-obj/layout.xml` 与 `ue-assets-obj/layout.catalog.json`。
起始插座孔口仍高出台面 5 mm，套筒深 15 mm，插头保持原始就位状态。
`inset-geometry-obj.json` 的 11 项网格检查、`native-geometry-obj.json` 的 36 项通道检查
及 `force-check-obj.json` 的三种插头起始座拔出/20N 恒定阻力检查均通过。
真实 UE 资产导入的全部 12 项 LOD0 三角面数一致；脚本抽屉循环成功，34.40 秒，
两路相机各 1721 帧，详见 `recording/summary.json` 与 `preview-check.json`。
`starting-sockets-before-after.png` 为同摆放位置、同机位的修改前后画面。

采样器按最大空闲矩形布置，各单元分别选择位置，固定姿态。
默认台面边缘至少留 5 mm，机构保守运动包络额外扩张 1 mm，单元之间至少留 8 mm，
避开原有固定台面凸起。它是可复现的约束采样，不是每个坐标独立均匀分布。
找不到合格位置时明确失败，不接受重叠布局。

几何检查核对 DOF、关节顺序/参数、驱动/等式约束、初始 qpos、姿态、sites、
精细孔壁和全部原机构碰撞几何的相对位置；并检查 6 个目标孔各 4 个轴向位置是否撞底板。
原有空 board body 和动态机构的父子关系保持，已有脚本仍可按 body/site 查找新坐标。
生成器将每个 body 自身的 geom 排在子 body 之前，使 XML 渲染元数据顺序与 MuJoCo
编译顺序一致；检查器逐项核对全部 geom 的 body/mesh，防止新增网格绑定错位。

## UE 与双相机采集

每个布局生成 12 个新可视网格（11 个安装片和 1 个底板），导入独立 UE 目录：

```bash
"$SIMULATION_PYTHON" tools/prepare_independent_layout_catalog.py \
  --layout "$LAYOUT_MODEL" \
  --base-catalog /absolute/path/adapter/Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json \
  --adapter /absolute/path/adapter --unreal /absolute/path/UE_5.6 \
  --destination /Game/BSK/Generated/UniqueIndependentLayoutName \
  --output /absolute/path/outside-repository/new-asset-import
```

该工具会导入、配置并检查全部 12 项；UE 的显式完成文件和 LOD0 三角面数验证通过后，
输出完整 `layout.catalog.json`。目标 UE 目录与输出目录必须全新，避免替换其他场景资产。
底板导出前在 0.1 微米容差内简化布尔接缝，丢弃浮点导出产生的极微小三角面，
并在 manifest 中记录被移除的面数/面积；不会通过跳过 UE 三角面核对来接受资产缺失。

在已有脚本录制命令中增加：

```bash
--model "$LAYOUT_MODEL" --catalog /absolute/path/new-asset-import/layout.catalog.json
```

录制与动力学进程都绑定模型哈希、全部 12 个新网格哈希、catalog 和场景 manifest。
每个 episode 保存布局种子、各安装坐标和元数据。重新布置后的抽屉指令不再错误地标为“最右侧”。
示例脚本仍是生产 `record_drawer_task.py`，需要像普通独立采集那样指定 scene、adapter、UE、
独立端口和输出目录；后台公开实例无需切换或停止。

## 验证范围和限制

本机证据目录：`/home/lyh/space_enviroment/runs/task-box-independent-layout-20261021/`。

- 两个种子已通过原生 MuJoCo 几何检查：每个布局 24 次孔道检查，底板最大穿透为 0；
  nq=63、nv=57，初始关节状态和机构姿态保持，1459 个原有几何的迁移/接触参数通过检查。
- 20N 真实轨道动力学直接施力测试中，三种插头的拔出、半程插入、半程拔出均通过。
  这不等价于机械臂完整搬运成功；导向插头完全就位的旧问题仍需独立调试。
- 98 项布局、脚本任务、采集和摩擦相关单元测试通过；新增/修改 Python 文件通过 Ruff。
- `seed-20261021-renderfix/layout.xml` 与 `ue-assets-final/layout-renderfix.catalog.json` 是本次
  实际录制使用的模型/资产配置；`geometry-check-renderfix.json` 检查最终模型的几何和全部 2814 项渲染对应。
  `force-check-final.json` 的模型与该模型经编译后的物理数组完全相同，见
  `render-order-physics-equivalence.json`。
- `recording-drawer-renderfix/summary.json`：随机布局下真实机械臂完成 `drawer_cycle`，任务成功。
  抽屉最大拉出 80.343 mm，随后回推、释放并确认关闭；任务结束时抽屉位置约 -0.00072 mm。
  这是一个布局上的脚本控制验收，不是 VLA 模型推理实验。
- 双相机采集完整：34.70 秒，每路 1736 帧，1024×1024 / 50 Hz。
  原视频在 `episodes/episode-20261021-224817-092421a7/lerobot/videos/`。
  `independent-layout-comparison.png` 为真实 UE 初始帧对比，
  `independent-layout-dualcam-3x.mp4` 为双相机三倍速预览。

凸分解仍是底板碰撞近似。切割中移除极薄数值碎片，相关体积写入 manifest；
锁紧支撑根部在底板内部延伸 10 微米，避免 CAD 接缝形成数值孤立点。
底板材料体积、自动计算的质量/惯性可能随几何处理略变，不能宣称严格恒定。

单元之间不重叠和孔道畅通不代表机械臂能到达任意位置，也不保证整条轨迹无碰撞。
当前没有实现机器人全程可达性筛选或随机布局任务批量成功率验收。
若要用于大批量训练采集，应先对候选布局增加 IK/轨迹检查，再按任务结果筛选 episode。

旧的 `task-box-layout/1` 两块板小范围平移工具继续保留以复现早期试验；
它们不代表这次全部安装单元独立随机化的实现。旧证据目录为
`/home/lyh/space_enviroment/runs/task-box-layout-prototype-20261021-01/`。
