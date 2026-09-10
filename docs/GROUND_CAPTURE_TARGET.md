# SARM 待抓取目标：地面验证星（外侧板铰链）

更新：2026-09-09。高精度三角网格仍不默认加载；现已在粗碰撞方案上开启目标内部接触，默认模板为 **`sarm-ground-validation-self-collision-grasp`**。
目标仍为带外侧板铰链的地面验证卫星，不是换回小方块。高精度试验模型/工具和历史验证文件全部保留，暂不默认加载。

## 运行入口与替换范围

| 模板 ID | 运行 XML（相对仓库） | 用途 |
| --- | --- | --- |
| **`sarm-ground-validation-self-collision-grasp`** | **`model/SARM/platform/sarm_ground_target_self_collision.xml`** | 当前默认；粗碰撞体＋外侧板与固定目标内部接触 |
| `sarm-ground-validation-grasp` | `model/SARM/platform/sarm_ground_target.xml` | 旧粗盒；没有目标内部接触，保留历史实例/试验输入 |
| `sarm-ground-validation-mesh-grasp` | `model/ground_validation_satellite/mesh_collision_trial/sarm_mesh_collision.xml` | 高精度实验保留，须手动选择，不默认加载 |
| `spacecraft-arm-teleop` | `model/SARM/platform/sarm_platform.xml` | 原小方块目标，保留旧实例/离线回归 |

只改变目标的碰撞版本；主卫星、六轴机械臂、两指夹爪、三轴反作用轮、姿态保持、两路相机 ID 和控制协议保持不变。
目标沿用独立自由体标识 `capture_target`；`satellite_inner_panel` 与本体固定连接，
`outer_panel_hinge` 是外侧板相对内侧板的一个被动自由度，不进入机械臂的 8 路关节/IK/执行器数组。

原文件 `model/ground_validation_satellite/ground_validation_satellite_articulated.xml`、原 STEP/OBJ、审查与运动预览均未覆盖。
不能用这个仅包含目标的原始 XML 直接替代完整 SARM 场景，否则机器人、控制执行器与相机会消失。
组合场景保留目标的全部 208 个可视 geom（104 个唯一网格）、位姿、颜色和铰链层级；没有凭空增加抓取把手。

### 选择与资产校验

- `backend/space_arm_platform/scene_targets.py` 统一管理默认模板、XML 路径、碰撞类型与局部初态。
- `tools/select_sarm_scene.py --check` 默认仅校验粗盒组合及其源文件指纹，不读取高精度碰撞清理副本；显式选择高精度模板时才校验该版本的全部原始网格和碰撞副本。所选版本缺失/过期时明确失败，不静默换成其他模板。
- `scripts/prepare_sarm_scene.ps1` 按模板准备实际运行 XML 对应的 UE 资源；直接启动平台时使用共享默认值，启动已保存实例时使用该实例的模板。
- `ModelRoot` 仍是 `model/SARM/platform`，供适配器找到原生场景脚本；仅 `native.MODEL_PATH` 按模板解析到组合 XML。
- 原生 `_load_scene` 为 BSK 的 VFS 同时收集 `asset/mesh` 和 `flexcomp` 的文件，避免只提供视觉网格、遗漏碰撞专用文件。

## 当前默认碰撞与真实性边界

当前使用 `sarm_ground_target_self_collision.xml`：原三个隐藏粗盒的形状和位置不变，继续与机器人/夹爪接触；
另加两个仅用于内部接触的板件粗盒，以显式 `<pair>` 开启**外侧板 ↔ 本体、外侧板 ↔ 内侧板**。
新模板没有目标 body 排除，没有全局关闭父子过滤，也没有添加铰链角度限位。新增代理质量为零，不增加运动自由度。
为避免原粗盒的铰链重叠，内部代理在铰链两侧各留出约 15.95 mm 的近似避让区；原视觉模型与外部接触粗盒没有被裁掉。
这是粗碰撞几何，不是真实铰链间隙或精准 CAD 接触。详见[粗碰撞内部接触](COARSE_SELF_COLLISION.md)。
原来的 208 个可视 geom、质量/惯量、被动铰链、时间步、控制器、相机及轨道保持不变，不加载高精度 `flexcomp`。

### 保留的高精度实验版（非默认）

以下能力和限制仅适用于显式选中的 `sarm-ground-validation-mesh-grasp`，不属于当前默认运行配置。
高精度模板使用 MuJoCo 3.7 的 `flexcomp type="mesh" dim="2" rigid="true"`：
三角表面的全部顶点绑定所属刚体，不增加柔性自由度，不把目标变成软体。
208 个表面实例共保留 1,694,919 个三角形，不进行凸包化、减面、缩小或补洞。
只清理导入 float32 后严格零面积的面，原始网格文件不修改；每面接触半径为 10 µm。
详见 `model/ground_validation_satellite/mesh_collision_trial/README.md` 的逐三角形转换精度与接触验证记录。

- **移除目标的三个粗碰撞盒和整对目标刚体排除；活动外侧板与本体、内侧板启用接触。**
- 固定本体与固定内侧板属于同一刚性装配，固定部分之间不建立内部接触。
- 视觉 geom 仍仅渲染，真正接触发生在对应的刚性三角表面上；UE 不承担权威物理积分。
- 铰链切分表面及附件全部参与接触，没有为了消除抖动而关闭这些表面的碰撞。
- 原铰链只是预览用表面切分，不是真实铰链机构的重建；**零位已有接触/干涉，可能阻碍转动或引起反向响应**。
- 全分辨率接触**显著慢于实时**；历史 BSK 组合短测仅 10 ms 物理时间约耗时 4.1 s。
- 这是一种带接触厚度的三角表面碰撞，不是实体布尔算法；静态角度扫描有接触，不代表任何速度/时间步下都不穿透。

### 质量、惯量和关节仍是临时仿真参数

| 刚体 | 估算质量 | 惯量来源 |
| --- | ---: | --- |
| `capture_target` | 8 kg | 原组合生成器的均匀包围盒惯量 |
| `satellite_inner_panel` | 1 kg | 同上 |
| `satellite_outer_panel` | 1 kg | 同上 |
| 合计 | **10 kg** | 不是 CAD 材料密度或实测值 |

切换高精度表面没有自动校准质量、质心、惯量或摩擦。原铰链仍无驱动、无弹簧、零阻尼/摩擦、无虚构角度限位。
真实 Earth/Sun 重力继续作用于全部新增刚体；姿态保持只控制 SARM，目标本身不增加姿态控制。
外侧板局部 `capture_target_grasp` site 只是定位标记，不代表已实现自动接近、锁紧或搬运。

旧 `sarm-ground-validation-grasp` 仍没有内部接触；当前默认 `sarm-ground-validation-self-collision-grasp` 已开启粗碰撞内部接触。两者是不同的实例物理配置，不要混用。

## 初始化与旧实例兼容

新旧关节卫星模板共用目标基准 `(0.94, 0.039086, 0.40)` m、单位四元数（wxyz）。
这只是相对初始主卫星的场景位置，真正惯性坐标还要叠加轨道位置。
`none` / `training-v1`、500 km 圆轨道、独立轨道起点随机化、星历和太阳光照逻辑不变；
同一 Seed 下，粗盒版和高精度版的**初始随机参数相同**，后续动力学因碰撞不同而可以不同。

铰链初态固定为 `target_hinge_position_rad: 0.0`、初始转速 0，实例加载器仍拒绝其他初始角度。
零位允许加载不等于高精度版零位无接触；旧粗盒版的自由转动测试不能用来证明高精度版也能无阻碍转动。

**已有实例不自动迁移。** 四个 ID 对应各自物理配置；之前的无内部碰撞粗盒实例不会自动开启接触，需重新创建“粗碰撞体·内部碰撞”实例。高精度实例身份也不变。
实例和实时观测会给出 `capture_target.runtime_model`、`collision_model`、`runtime_warning`，以及估算质量/铰链角度。
当前默认碰撞标识为 `coarse_boxes_with_target_self_collision`，旧粗盒为 `coarse_boxes_no_target_self_collision`，高精度为 `original_triangle_rigid_flex`。前端会区分三种碰撞模式并提示近似/实验限制。
原生场景脚本的离线默认 XML 保持小方块版，平台遥操作入口按模板选模型。

## UE 资源和使用步骤

资源目录仍为 `/Game/BSK/Generated/SARM`，本机 catalog 为 `Saved/AssetImport/sarm_platform.catalog.json`。
高精度和粗盒版本复用 113 个可视网格（SARM 9 个 + 目标 104 个）；刚性碰撞表面不额外作为可视几何导入 UE。
首次导入/重建可能需要数分钟，后续通过指纹复用。
组合 XML 保持同一 body 的 geom 排在子 body 前面，避免渲染桥按编译索引映射时错配。

1. 若平台/后端仍在运行旧代码，重启后端/平台并刷新操作台；本次切换未替用户停止主场景。
2. 选择 **SARM + 地面验证星（粗碰撞体·内部碰撞）**，重新生成并启动实例；不要复用之前无内部接触的粗盒或高精度实例。
3. 查看实例目标摘要和 `capture_target.runtime_model`，确认是 `model/SARM/platform/sarm_ground_target_self_collision.xml`。
4. 使用原遥操作和两路相机；相机位姿不变，不保证每个姿态都能完整看到目标。
5. 以后需要继续高精度试验时，手动选择 **高精度三角网格碰撞·实验**；该模板及文件未被删除。

仓库根目录校验与生成：

```powershell
# 平台启动使用的快速选型/指纹检查（仅标准库）：
python tools/select_sarm_scene.py --check
# 校验当前默认组合；仅在维护高精度实验时才运行后一个检查（依赖 NumPy）：
python tools/build_sarm_ground_target.py --check
python tools/build_satellite_mesh_collision.py --check
# 需要强制重新导入 UE 可视资源时：
.\scripts\run_platform.ps1 -ReimportAssets
```

不要手工修改派生组合 XML。修改源模型/估算参数后需要重新生成、验证，两个仓库新克隆后均需 `git lfs pull`。
试验 `manifest.json` / `validation_report.json` 中的 `default_scene_changed: false` 等字段是**转换试验当时的历史快照**，不是运行选择器；
本次切换没有重写这些历史记录或改变几何，当前默认以 `scene_targets.py` 为准，实验/非工程验收限制继续有效。

## 当前内部接触验证

新增 MuJoCo 3.7 接触回归涵盖初始无假接触、显式 pair 法向力、10 s 连续转动/持续力矩、128 组随机初态、两指接触、原质量与外形不变。
原生 BSK 和真实遥操作链路证据记录在本机 `run/coarse-self-contact-20260909/`，详细验证边界见[内部接触专题](COARSE_SELF_COLLISION.md)。

### 此前回退至无内部接触粗盒的历史验证

默认选型、前端默认/启动请求、停止的高精度实例不覆盖下次粗盒选择，以及高精度资产异常不影响默认粗盒选择均有回归覆盖。
本次通过：后端/选型 40 项、MuJoCo 组合 8 项、独立 BSK 2 项、前端 53 项（共 103 项），前端构建与 PowerShell 语法检查通过。
真实默认模板 → BSK → Hub/渲染协议短测确认 15 个 body、208 个目标可视 mesh、**3 个目标粗盒**、8 路臂观测和姿态保持；地球/太阳与轨道状态一致，状态有限。
0.1 s 物理推进约用 0.219 s 墙钟时间，含冷启动总耗时约 28.26 s；仅是本机短测，不保证长时或浏览器端帧率。
本轮复用既有 UE 可视资源目录，未重新导入或启动 UE；未重做 GPU/浏览器验收，下一次正式启动会按实例模板重新校验资源映射。
原粗盒生成器 `--check` 与显式高精度文件/源网格指纹校验均通过，原模型和高精度几何均未改动。
本次隔离真实仿真验证记录位于 `run/mesh-rollback-20260909/`。没有重启主平台、改写既有实例、删除高精度资产或提交推送。

### 此前高精度启用阶段的历史验证（不是当前默认配置）

- 切换回归覆盖默认 ID、三种模板的准确路径和碰撞元数据、保存实例不迁移、同 Seed 初始布局不变，以及缺失/过期碰撞资源拒绝启动。
- 前端模板、实例回显和实验提示回归通过；原生 VFS 回归覆盖仅被 `flexcomp` 引用的文件。
- 高精度启用阶段真实默认模板 → BSK → Hub/渲染协议联调通过：15 个 body、208 个目标可视 mesh、0 个旧目标粗盒、8 路臂观测、姿态保持启用；轨道/地球/太阳位置一致，状态有限。0.04 s 物理推进约用 11.08 s 墙钟时间，含进程启动总耗时约 131.88 s；这是本机短测，不是性能保证。
- UE 113 个网格资源准备成功。独立 UE 进程回放上述真实协议，已查看主视口和两路相机预览；两路权威 RGB 文件也有输出，但冷启动首帧明显偏暗，**未验收采集曝光/图像质量**，未以回放代替浏览器 WebRTC 验收。
- 高精度启用阶段通过：后端/选型 38 项、MuJoCo 3.7 几何与组合 20 项、独立 BSK 回归 2 项、前端 52 项；前端构建、PowerShell 语法和两个生成器 `--check` 通过。粗盒间隙回归只查询 11 cm 范围以验证 >10 cm 阈值，避免 MuJoCo 3.7 对远离凸几何使用过大距离窗口时返回退化零值；不改变任何生产碰撞几何/求解器。
- 高精度启用阶段证据保存于本机 `run/mesh-switch-20260909/`，主平台未被重启，已有实例未迁移。
- 原三角面转换、静态接触及 BSK 10 ms 短测记录保留在 `mesh_collision_trial/validation_report.json` 和本机 `run/mesh-collision-20260909/`。
- 原粗盒替换阶段的 128 组随机间隙、两指接触、4 组轨道联调及 UE 三路 GPU 截图在 `run/ground-target-integration/`；这些结论只属于当时的粗盒配置，不能挪用为高精度机构验证。

尚未完成：真实铰链机构建模/限位、工程质量惯量标定、长期或高速连续碰撞、自动抓取闭环、实时性能和浏览器 WebRTC 全链路验收。
