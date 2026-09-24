# 本地任务盒接触模块

两个无卡扣插头的独立活动实验、实际孔槽测试和带载验收见 [活动插头说明](TASK_BOX_FREE_PLUGS.md)。下文保留固定装配模板的复现方式。

本功能在 GitHub 卫星场景中组合本地任务盒，启用任务盒的接触检测与接触力求解。它是**固定装配接触验证阶段**，不是插头拔出、抓取、锁止解除已经完成的声明。

## 所有权与文件

| 范围 | 来源及处理 |
| --- | --- |
| 任务盒本体、抽屉、圆柱/导向/航空插头、挂钩、紧固件、螺栓、锁紧杆 | 本地 SolidWorks 导出，9 个零件的 STL、装配变换及来源哈希保存在 `model/task_box/module/assembly.json` |
| 卫星本体、封盖、外层包材 | 保留上游模型；不导入本地 CAD 的卫星和卫星封盖 |
| 机械臂、抓夹、控制器、关节、惯量 | 保留上游配置，本模块不修改 |
| 地面验证目标星及其面板 | 保留上游配置 |

场景为 `model/SARM/platform/sarm_task_box_module.xml`；旧默认场景不变。所有操作件当前刚性连接到 `cubesat_bus`，仍随主卫星整体运动。未引入新的自由关节或插拔执行器。整星惯量保持上游值，没有用未核实的 CAD 质量覆盖它。

这不是把所有零件熔成一个刚体 STL，也没有生成新的 `.SLDASM` 文件：它在 MJCF 装配层组合网格并保留来源。本阶段零件运动暂锁定，但各零件资源独立，后续可继续配置活动连接。

## 如何生成碰撞

1. 从工作台 `geometry.json` 中只选择上述 9 个零件；不选择本地机械臂、抓夹、卫星或封盖。保存原始 STL 及 CAD 哈希，统一使用米。
2. 按每个零件的装配变换，再通过明确的 CAD→卫星坐标变换，将网格转换到卫星基座坐标。该变换保存在配置中，换了装配布局必须重新核对。
3. 上游 `base_link.obj` 是融合网格，已经含旧任务盒表面。生成器按零件边界和三角面三个顶点与 CAD 表面的距离（50 μm 容差）匹配旧表面，替换为本地表面。不同曲面三角化的弦中点不用于判定。共享 z=0 平台面仅在任务盒投影范围内裁掉重叠片段，范围外片段保留，避免旧面和新面叠加闪烁。**原始上游文件不写入、不覆盖**，修改记录在生成清单中。
4. 原来的整块基座凸包会封住接口孔洞。将其分成任务盒区域外的凸体；区域内保留上游接口表面，周边小实体使用分实体凸包。任务盒自己的 9 个零件使用 `flexcomp type="mesh" dim="2" rigid="true"` 三角面接触，保留原始孔、槽和针孔。这里 `rigid` 表示刚性三角面，不是软材料。
5. 固定装配碰撞面使用 `contype=0, conaffinity=3`：接受上游机械臂的 bit 1 和目标物的 bit 2，避免同一固定装配之间做无意义接触。**单看 contype=0 不代表关闭碰撞**。视觉网格则两个掩码均为 0，避免视觉凸包封孔。
6. 明确启用 `contact`、`constraint`，关闭全局参数 `override`。各零件的 `friction` 和 `solref` 在 `assembly.json` 中分别配置；重新生成后生效。当前值是仿真假设，不是实测标定。

刚性三角面是有微小厚度的接触表面（半径 10 μm），不是精确 CAD 实体布尔运算。高速撞击、复杂穿透初态和极小间隙仍需单独验证。周边上游凸体也保留近似性质。

## 复用与验收命令

在仓库根目录执行，`python` 应指向对应环境。不要在同一进程同时加载独立 Python MuJoCo 和 Basilisk 的 MuJoCo DLL。

```powershell
# 离线生成环境：numpy、scipy、trimesh、rtree；不需要 SolidWorks 在协作者机器上运行。
python -m pip install numpy scipy trimesh rtree
python tools/build_task_box_module.py

# CAD 更新后：重新导出 geometry.json，先重新导入，再生成。
python tools/build_task_box_module.py --geometry <工作台导出的geometry.json>

# 独立 MuJoCo 3.7.0 环境
python tools/validate_task_box_module.py

# 另一个带 Basilisk 的解释器；仅加载 Basilisk 的 DLL
python tools/validate_task_box_native.py

# 平台依赖环境：配置、来源、旧场景兼容性
python -m pytest tests/test_task_box_module.py tests/test_selected_sarm_scene.py tests/test_scene_runtime.py tests/test_scene_process_helpers.py -q
```

离线验证分别检查台面/侧壁接触、导向孔及槽内空隙、航空插头针孔空隙和错位接触力；再让自由探针以 1 cm/s 碰台面，检查是否穿透。它还检查零位假接触、1000 步数值稳定性、自由度/质量不变及掩码兼容。探针能进入不等于整个插头已完成机械配合；两排接口深度不同，不能共用一个深度假设。

原生检查加载同一文件和现有控制器，短时验证状态有限、原有 15 个 Body/8 个机械臂控制通道保留。结果写入 `reports/task-box-native.json`。短时加载检查不能代表长时实时性，也不等于 UE/RGB/深度/分割验收。

## 在网页中使用

重启使用本分支代码的后端后，在场景模板中选择 **本地任务盒（接触验证·固定装配）**，创建并启动场景。启动流程会选择新 MJCF 并准备 UE 资源。

- UE 资源目录：`/Game/BSK/Generated/TaskBox`。
- Catalog：适配器项目 `Saved/AssetImport/sarm_task_box.catalog.json`。
- 原场景的 `SARM` 资源和 `sarm_platform.catalog.json` 不覆盖。
- 切回原模板即可回退；当前运行的旧实例不会因为拉取代码自动切换。

也可以先只准备资源，不启动/切换当前场景：

```powershell
./scripts/prepare_sarm_scene.ps1 -AdapterRoot <适配器仓库> -ModelRoot ./model/SARM/platform -UnrealRoot <UE安装目录> -TemplateId sarm-task-box-contacts
```

## 协同开发

CAD 来源更新由任务盒负责人完成，在本目录提交 STL、配置、生成资产、MJCF 和清单。模型文件使用 Git LFS；协作者拉取后执行 `git lfs pull`。

合入上游更新后重新运行生成器和验证，不用旧整场景覆盖上游 XML。模型选择器会校验上游源场景、基座和本模块资产的哈希；不一致时拒绝启动，要求重新生成和验收，避免悄悄使用过期组合。

不提交机器绝对路径、运行日志、录制、虚拟环境和 UE 的 Saved/Intermediate。PR 应说明范围、实际通过的测试及没有验证的环节。独立模块也可能在场景注册或共同修改的基座文件上发生冲突，应保留双方变更后重新生成、验证。
