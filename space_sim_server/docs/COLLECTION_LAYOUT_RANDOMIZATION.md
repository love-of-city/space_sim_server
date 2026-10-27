# 自动采集中的操作台布局随机化

自动采集支持每次尝试从预先生成并校验的布局池选择一个布局。每套布局含 13 个安装单元的独立位置，可含绕台面法线的独立朝向；插头就位、抽屉关闭等机构初始状态及既有 20 N 插拔摩擦配置保持。

2026-10-23 在相机等待阶段超时的记录属于历史原型，见仓库 `archive/2026_10_23_随机布局接入自动采集.md`。当前运行入口与最新性能证据见[单机采集性能](COLLECTION_PERFORMANCE.md)。2026-10-27 的顺序两条性能验收显式使用一个已校验固定布局，不代表历史布局池全部有效。随机采集必须使用当前源码重新准备并通过哈希校验的池；旧池失效时停止，不静默复用或退回固定布局。

## 先准备布局池

移动固定安装件需要同步重建底板开口、碰撞几何和 UE 资产，因此在开始并发采集前串行准备，采集进程仅只读使用。下面生成 10 个布局（耗时较长），输出必须是新的、位于仓库外的目录：

```bash
SIM_PY tools/prepare_collection_layout_pool.py \
  --geometry-python GEOMETRY_PY \
  --count 10 --seed 2026102300 --yaw-range-deg 180 \
  --adapter /path/to/ue_adapter --unreal /path/to/UE_5.6 \
  --base-catalog /path/to/sarm_20261012.catalog.json \
  --output /path/to/data/layout-pools/pool-001
```

`SIM_PY` 为安装 Basilisk 的仿真 Python；`GEOMETRY_PY` 为安装 numpy、trimesh、manifold3d、coacd、scipy、MuJoCo 的独立几何 Python。禁止把普通 Python MuJoCo 导入实时 Basilisk 进程。朝向范围为 `±yaw-range-deg`，`0` 保持原朝向。

已有布局可重复指定 `--existing MODEL CATALOG NATIVE_REPORT` 注册，亦可与生成新布局组合。池至少需要两个不同模型。准备工具逐套执行布局生成、原生几何校验、底板/孔套几何校验、独立 UE 资产导入；全部通过后才发布 `pool.json`。失败时保留日志且不发布半成品池。UE 导入使用新命名空间并对同一项目加锁。

布局池约束保证几何和资产一致，并不保证所有任务可达或成功。自动采集对每次选中的布局重新编译原生预检模型，检查关节限制和接近路径；不通过记为 `initialization_rejected`，仍占用尝试次数。

## 使用自动采集

单批脚本 `collect_drawer_cycles.py`、`collect_plug_tasks.py`、`collect_scripted_tasks.py` 增加：

```bash
--layout-pool /path/to/data/layout-pools/pool-001/pool.json
```

并行采集在 `collection.json` 中设置 `"layout_pool": "/path/to/pool.json"`，相对路径按该配置文件所在目录解析。默认 `null` 沿用原固定布局。各 worker 共享只读池，保留原先互不重叠的尝试种子及端口。

网页后端启动时设置：

```bash
export SPACE_SIM_COLLECTION_LAYOUT_POOL=/path/to/pool.json
```

运行本版本后端和前端后，自动采集的新建批次表单会显示“操作台位置与朝向随机化”及可用布局数量。勾选后 API 传递 `layout_randomization: true`；没有配置或校验失败时开关禁用。启用请求缺少有效池会明确失败，不会退回固定布局。后端创建批次时保存池成员快照。

## 选择与复现

每次尝试按 `task-box-layout-selection-v1` 独立随机流、尝试 seed，从池中均匀有放回选择一个布局，允许相邻两次选中同一布局。布局的几何采样本身使用有边界和间隙约束的拒绝采样，不代表对所有可行布局均匀采样。

批次 `summary.json` 的每条 attempt 和 episode `metadata.json` 内的 `scene_instance.collection_config.layout_randomization` 保存布局 ID、池哈希、选择种子、生成种子、模型和 catalog 哈希、实际 XY 偏移与 yaw；`scene_instance.task_box_layout` 保存完整生成清单。

同一次尝试的碰撞预检、脚本目标、动力学、UE 场景与双相机录制使用同一个模型/catalog。每次选择重新核对模型、清单、原生校验报告、catalog 和 12 个生成视觉资产的哈希。开启其它噪声或轨道随机化不会改变布局选择流。只有原有任务与双相机完整校验及布局元数据一致性检查均通过的轨迹才计入成功。

当前选择池中已有布局；不会在每条 episode 录制时现场重建资产。增加多样性应先扩充池，再开始新批次。复用 UE 的每次场景切换仍由原有 session/reset 协议执行。

## 每条数据的七色外观与指令

七个任务目标孔为三类插头各自的 `40p1`、`40p5` 孔，以及 `bolt_assembly_hole`。每条尝试将 red、orange、yellow、green、black、blue、purple 无放回分配给七孔；三个插头和螺栓从同一色板独立无放回选取四色。孔组和零件组允许同色，物理配对仍由稳定的 body/site 决定，颜色不改变兼容关系。取件座与 `bolt_storage_hole` 在每条数据中使用中性灰，不属于七个任务目标孔，也不参与指令颜色分配，避免与有色目标孔混淆。

`prepare_episode` 使用独立 `hole-colors-v1`、`source-colors-v1` 随机流，将一次分配冻结到 `collection_config.episode_color_assignment`，重放不会重新取色。插头任务使用简洁英语，例如 `Pick up the red aviation plug, align it with the blue socket, and insert it fully.`；螺栓任务使用 `screw it in fully.`。抽屉任务没有颜色指令。

录制前由 `write_model_variant` 将同一分配应用到独立模型中现有的非接触可见表面，另写外观变体和摘要；源模型、布局池、mesh、碰撞、关节和目标 site 不变。默认整体模型的多个孔共用一块视觉网格，不能独立实现七孔配色；带颜色指令的任务使用该模型时会明确报错，须传入具有完整独立孔面绑定的模型。无颜色语义的默认抽屉和诊断入口继续可用。

保存的 `assignment_sha256` 必须与实际孔/零件颜色一致；改动颜色后沿用旧摘要、重复颜色、缺少可见表面绑定或跨类型目标都会在启动前被拒绝。颜色、指令和材质的静态检查不替代真实相机画面的可辨认性验收，也不代表七个孔的物理任务全部通过。
