# 插拔件恒定摩擦保持

插头在孔内靠静摩擦保持，沿轴插入或拔出时承受恒定滑动阻力，脱离作用区后自由运动。没有沿插拔行程变化的弹簧势垒；行程中途松手后先减速，再停在当前位置，不自动吸入或弹回。

复用现有插拔件、孔位和碰撞几何，不修改 MJCF、网格、质量或 UE 资产。此规则是等效干摩擦模型，并不求解孔壁真实压力分布。

## 启用与配置

网页遥操作和 `tools/run_scripted_task.py` 共用 `config/plug_retention.json`，新启动场景加载：

```json
{
  "enabled": true,
  "parameters": {
    "static_friction_n": 20.0,
    "sliding_friction_n": 20.0,
    "friction_travel_m": 0.008
  }
}
```

- `static_friction_n`：静摩擦上限，默认 20 N；实际静摩擦按当前负载取值，不会一直施加 20 N。
- `sliding_friction_n`：滑动阻力，默认 20 N；插入和拔出相同，与作用区内的深度和滑动速度无关。
- `friction_travel_m`：从到位位置向外的有效接合行程，默认 8 mm；超出后轴向摩擦归零。

默认值与确认的示意图一致：静摩擦上限、插入和拔出的滑动阻力均为 20 N。调整时必须保持静摩擦上限不小于滑动阻力。这些默认值并非实物标定结果。实际机械臂所需力还包含接触、惯性、导向等，不能把 20 N 理解为腕部传感器总力恒定。最后一个停止物理步的摩擦可以小于 20 N，以免减速过冲反向抖动。

`enabled: false` 关闭规则。环境变量 `SPACE_SIM_PLUG_RETENTION_CONFIG` 可指定其他文件；未知参数报错。旧的 `pull_out_peak_n`、`insertion_peak_n`、`detent_travel_m`、`axial_damping_ns_m` 已移除，旧外部配置应迁移到上述字段。

目前只支持 `local` 动力学后端。修改配置后重新启动仿真场景生效，无需重建 UE。

## 接合与导向

支持航空、导向和圆柱插头，每个插头复用初始放置位和对应的 `_insert_40p1`、`_insert_40p5` 装配孔位。孔位固定在可自由运动的 `cubesat_bus` 上；装配高度沿用现有任务的法兰偏移。

插件向内进入最后 8 mm、距孔轴不超过 1 mm、角度误差不超过 5°时接合。初始化时已对齐且静止的部分插入件也可以直接保持。不会在远处吸附；脱离后的向外运动不会重新接合。航空、导向插头检查键向；圆柱插头允许绕自身轴旋转，只检查倾斜。侧向偏离超过 20 mm 或姿态偏离超过 45°时脱开。

保留横向、角度的有限柔性导向，及 `z < 0` 的到位背挡保护；这些力与轴向滑动摩擦独立。轴向 `0 ≤ z < 8 mm` 区域没有位移恢复项或速度阻尼项。孔口碰撞仍由原有模型决定，不会通过瞬移修正装配失败。

`retained` 表示到位容差内，`held` 表示行程中途静止，`extracting` / `inserting` 表示轴向运动，`free` 表示脱离。这些状态不是整段机械臂任务成功判据。

## 动力学与记录

组件每个 400 Hz 物理步读取完整场景状态以确定接合与导向。轴向摩擦由唯一的原生 MuJoCo 积分核心在每个子步求解：在独立的临时动力学数据中预测当前负载、接触和伺服下的滑移，求出有界停止力，然后只对权威状态执行原有积分。临时预测会丢弃，不发布、不写回物体位置或速度。

静摩擦补偿下一子步即将发生的滑移，避免单纯 `sign(v)` 的低速抖动或黏性阻尼的缓慢爬移。MuJoCo 3.7 对部分自由刚体使用中点积分，对关节连接的基座使用末端速度积分，因此求解条件使用该子步实际积分的轴向位移增量除以时间，而非仅将步末速度置零；没有存储轴向目标位置或设置位移恢复力。数值滑移容差为 1e-6 m/s，有限精度下可能存在微小漂移。

接触求解器的全局代价停止条件有时会在轻插头的微小滑移响应上留下数值跳变。坐标二分保留已实测的最佳候选，并检查全部接口的共同残差；若仍无法满足原滑移容差，则恢复试解前的摩擦记录，在同一未推进的状态上将内层 Newton 接触精度收紧至 `1e-14` 重试。接受后，唯一一次真实积分采用相同精度，随即恢复模型原设置。静/动摩擦上限和 `1e-6 m/s` 容差不变；重试失败仍明确报错。

超过静摩擦上限后改用恒定滑动摩擦。三个插头同时接合时联合考虑基座反作用；每对力作用在同一接口点，并对双方质心偏移施加配对力矩，保持自由基座及系统动量关系。所有摩擦预测和真实积分使用 Basilisk 已加载的同一个 MuJoCo 运行库。

启动与采集配置版本为 `plug-retention/2-dry-friction`，逐帧新增 `friction_mode`、`axial_friction_force_n`、`axial_velocity_m_s`（上一个完成子步的平均轴向滑移速度）；总 `force_inertial_n` 包含摩擦和导向/背挡。遥操作及脚本采集继续保存配置与状态，旧 episode 和旧版配置记录保持原样。

## 构建与验证

原生源码更新后先重建（使用含 Basilisk 的项目解释器）：

```bash
python tools/build_native_acceleration.py --component local_mujoco_stepper
python -m pytest -q tests/test_plug_retention.py tests/test_scripted_capture.py tests/test_assembly.py tests/test_local_mujoco_stepper.py tests/test_independent_plug_tasks.py tests/test_scripted_plug_task.py tests/test_compact_recording.py
python tools/validate_plug_retention.py --output run/diagnostics/plug_retention/production-dry-friction.json
```

测试覆盖静摩擦无爬移、不同深度/速度下双向恒定摩擦、途中撤力停留、脱离自由漂移、错位不吸附、圆柱轴向自由转动及力/力矩配对。生产验收使用原始模型、自由基座、现有接触和 BSK 轨道组件直接施力，保存逐步曲线并核对原模型哈希；它不包含机器人抓取、路径规划或 UE 画面验收。

临时预测增加计算开销，真实场景的实时帧率需要结合机器负载评估。
