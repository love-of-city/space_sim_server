# SARM 反作用轮与惯性姿态保持

更新：2026-09-08。本功能接入平台默认入口；独立查看用的 `model/SARM/mjcf/SARM.xml` 不等于运行场景，不在本次增加器件的范围内。

## 1. 默认行为与文件归属

- 启动时锁定卫星本体相对 **J2000** 的初始姿态，随后自动抑制姿态偏差和角速度。
- 不是对地定向、太阳定向或轨道坐标系跟踪；不会随着绕地运动主动改变目标姿态。
- 姿态反馈来自 MJScene 真值，未模拟陀螺仪、星敏感器、噪声、估计误差或导航滤波。
- 机械臂依然接受浏览器遥操作；姿态闭环不依赖浏览器持续发送指令。
- 松键、失焦、遥操作超时和页面机械臂停止按钮不关闭星上姿态保持。该按钮不是全系统断电/全局急停。

| 文件 | 负责内容 |
| --- | --- |
| [sarm_platform.xml](../model/SARM/platform/sarm_platform.xml) | 三个轮的安装位置、质量、惯量、轴向、自由转动关节、电机限矩和最大轮速 |
| [attitude_control.json](../model/SARM/platform/attitude_control.json) | 默认开关、姿态模式、100 Hz 控制频率、PD 增益、轮速保护裕量 |
| [attitude_control.py](../simulation/attitude_control.py) | BSK 导航/参考/误差/控制/分配/消息转换/驱动保护与独立遥测 |
| [scenario_sarm_grasp.py](../model/SARM/platform/scenarios/scenario_sarm_grasp.py) | 创建并保留姿态链对象，安装到原生 simulation/process/scene |
| [teleop_grasp_unreal.py](../simulation/teleop_grasp_unreal.py) | 保留原生姿态闭环、给全部轮体接重力、发布 SI 关节及姿态/轮遥测 |
| [models.py](../backend/space_arm_platform/models.py)、[Schema](../contracts/space-arm-control-v1.schema.json) | 兼容旧观测布局、验证 SARM SI 单位和新遥测字段 |
| [app.js](../frontend/app.js) | 显示姿态模式、误差、角速度、轮速和实际电机力矩 |

修改 XML 或 JSON 后需要重新创建仿真实例；运行中的模型不热加载。前端发布文件已可由 `npm run build` 更新。已有后端进程需要重启才能使用新的 Pydantic 校验模型。

## 2. 暂定物理参数与质量预算

这是用于当前仿真验证的**暂定理想平衡转子配置**，不是实际选型、厂商规格或在轨认证参数。

| 参数 | X 轮 | Y 轮 | Z 轮 |
| --- | --- | --- | --- |
| body / hinge | `rw_x` / `rw_x_spin` | `rw_y` / `rw_y_spin` | `rw_z` / `rw_z_spin` |
| 本体系轴向 | +X | +Y | +Z |
| 本体系安装位置 m | `(0.02986, 0.27487, -0.31454)` | `(0.18986, 0.43487, -0.31454)` | `(0.18986, 0.27487, -0.47454)` |
| 转子质量 kg | 1 | 1 | 1 |
| 旋转轴惯量 kg·m² | 0.01 | 0.01 | 0.01 |
| 两个横向惯量 kg·m² | 0.0054 | 0.0054 | 0.0054 |
| 电机力矩上限 N·m | ±0.2 | ±0.2 | ±0.2 |
| 相对轮速额定边界 rad/s | 628.318530717959 | 同左 | 同左 |
| 换算 rpm / 相对动量容量 N·m·s | 6000 / 6.2831853 | 同左 | 同左 |

`rw_max_speed_rad_s` 在 XML 的 `<custom><numeric>` 中；BSK 读取 XML 中的转动惯量、电机范围和最大轮速，不另写一份硬件常数。

当前明确将三个转子作为原 **162.76 kg 本体预算内的分配**，不是在原本体上无条件额外增加 3 kg：剩余刚体质量为 159.76 kg，且其质心与完整惯性张量已用平行轴定理相应修正。剩余本体 + 三个锁定转子恢复原始总质量、质心 `(0.18986, 0.27487, -0.31454)` 和原始惯性张量。机械臂质量仍另计。

若实际硬件是新增负载而非原预算内器件，需要重新指定质量预算；修改轮子质量/位置/惯量时，也应同步核算剩余本体，而不是只修改一侧。守恒测试保护当前预算假设。

轮轴显式设置 `damping/frictionloss/stiffness/armature=0`，不继承机械臂的摩擦/阻尼；轮体不参与碰撞。暂未包含轴承耗散、非平衡、壳体结构、功耗、温升或电机动态。

## 3. 控制链与调度

```text
MJScene 卫星状态 → SimpleNav（无噪声真值）
初始姿态锁存 → inertial3D
两者 → attTrackingError → mrpFeedback（PD + 轮动量补偿）
    → rwMotorTorque（三轴控制分配）
    → ArrayMotorTorqueToSingleActuators
    → WheelDrive（限矩、限速方向保护、启停）
    → MJScene 三个 hinge motor → 机械耦合产生本体反力矩
轮轴 stateDot → ScalarJointStatesToRWSpeed → mrpFeedback
```

- 保留 `MJScene` 为唯一积分器，没有独立 `spacecraft` hub，没有直接设置姿态来实现闭环，也没有额外施加第二份本体反力矩。
- `sarmAttitudeTask` 默认 100 Hz，process 内 task 优先级 `-10`：与动力学同时刻时先执行动力学，再算控制，新指令在下一次动力学评价中被使用。
- task 内顺序：初始参考/惯量锁存 1000、真值导航 900、惯性参考 800、误差 700、轮速转换 600、PD 500、分配 400、数组转换 300。
- 三个 `WheelDrive` 在 MJScene 动力学子任务中以 `6500-i` 执行，在正向运动学/机械臂控制之后读取当前轮速，故安全保护不只依赖 100 Hz 的控制更新。
- 初次有效状态时锁存非零或零初始姿态；不会强制初始姿态为单位四元数。参考不会在后续周期重新追随实测姿态。
- 根据初始机械臂姿态、各 body 的真值位姿及 XML 的局部中心惯量组装整星锁定惯量：包含机械臂/转子，排除自由目标，以整星质心为参考点并表达在本体系。发布后再次 Reset `mrpFeedback`，使其读取而非继续使用初始化占位惯量。
- 控制器使用初始锁定惯量近似；不是完整柔性/多体逆动力学补偿。机械臂运动引起的惯量和内部动量变化主要作为闭环扰动处理。

默认增益：`K=8 N·m/MRP`，`P=12 N·m·s`；`Ki=-1` 关闭积分。小角度下 MRP 约为转角的四分之一，不能把 `K` 当成按转角定义的比例增益。

## 4. 限幅、饱和与诊断开关

电机命令同时受驱动侧和 XML 的 ±0.2 N·m 限幅。达到最大相对轮速的 98% 后，驱动禁止继续沿加速方向输出，但允许制动。非有限状态/指令导致明确错误，而不是静默注入 NaN。

这属于**驱动保护，不是硬改速度的物理钳位**。外部冲量或本体耦合仍可能使转速超过额定边界；这种情况以 `overspeed` 上报，不瞬间抹除角动量。

`attitude_control.saturated` 汇总限矩、限速及超速；前端显示“惯性保持 · 饱和”。轮速到达动量容量后不能保证继续抵抗任意扰动。尚无推进器卸载、失效轮容错和备份轮。

- 持久关闭：在 `attitude_control.json` 中设 `enabled=false`，重建实例。
- A/B 诊断关闭：`run_simulation.ps1 -DisableAttitudeControl`，或原生遥操作入口参数 `--disable-attitude-control`。
- 关闭只使电机指令为零，转子仍安装在机械模型中、仍自由转动；不冻结轮轴，不改变质量，不直接恢复本体姿态。

## 5. 观测协议迁移

继续使用 `space-arm-control/1`，对旧客户端做兼容性扩展：

- 原三个 `joint_*` / `target_joint_*` 别名允许**等长的旧 6 项或 SARM 8 项**，不接受 7 项或互相长度不一致；雅可比秩上限修正为 6。
- SARM 同时发明确 SI 字段：`arm_joint_position_rad`、`arm_joint_velocity_rad_s`、`target_arm_joint_position_rad` 各 6 项；`gripper_position_m`、`gripper_velocity_m_s`、`target_gripper_position_m` 各 2 项。
- 新 SI 字段一旦提供，就必须完整，且与旧别名拼接结果一致。旧 8 项别名的最后两项仍是 m 或 m/s，不能全量转成角度。
- `attitude_control`：开关/模式、参考系、当前与参考四元数、角速度、角度误差、饱和、状态和控制时间。
- `reaction_wheels`：三轮名称、相对转速/动量、请求/实际电机力矩、额定限制及每轮限矩/限速/超速标记。**轮状态绝不附加到机械臂 8 项数组中。**
- 新遥测随 Observation 经 Hub/WS/Recorder 传递。前端优先使用明确的夹爪 m/s 字段。
- 100 Hz 控制输出采用零阶保持。`state_time_ns` 是读取的本体原生状态时间，`control_time_ns` 是最近控制计算时间；记录的实际力矩是最近驱动评价值。它们不承诺与外层旧 `sim_time_ns` 标签完全一致。

原生输出与渲染帧的采样并非每次重合：已有 30 Hz/2 ms 快照标签边界仍保留在总体架构的待验证事项中。新字段不能被误解成修复了所有权威采集时序问题。

## 6. 验证与复现

[test_attitude_control.py](../tests/test_attitude_control.py) 覆盖质量/质心/惯量预算、三轴反力矩符号与线/角动量守恒、非零参考锁存、机械臂运动开关对比、姿态/角速度扰动恢复、限矩、轮速保护与制动、关闭控制及非有限输入。

[test_sarm_observation.py](../tests/test_sarm_observation.py) 覆盖真实 Hub 连续接收 SARM 8 项观测、轮遥测入 Recorder、SI 单位/长度约束、秩和 Schema 一致性；旧 6 项测试仍保留。

**独立轨道链路检查**：[validate_attitude_control.py](../scripts/validate_attitude_control.py) 使用临时端口启动真实原生仿真，连接真实 `SimulationHub` 和 `EpisodeRecorder`，并以只接收协议帧的测试端替代 UE。不会重启/占用当前平台，不产出 GPU 图像：

```powershell
# 当前 Python 环境需具备后端依赖；SimulationPython 指向具备 Basilisk 的环境。
python scripts/validate_attitude_control.py `
  --adapter-root <UE适配器仓库根目录> `
  --simulation-python <Basilisk环境的python.exe>
```

可选 `--catalog` 指定已生成的 SARM 资产映射，`--output` 指定隔离的本地结果目录。默认 12 秒；动作窗口结束后主动停止遥操作输入，验证超时后姿态控制仍保持开启。

2026-09-08 最终回归：服务端 **82 项通过**（含 12 项姿态/轮动力学测试），前端 **8 项通过**，前端构建成功；服务端有一条第三方弃用警告。

12 秒轨道探针两次均得到 360 条真实后端合法观测和 360 条 Episode 状态；渲染协议帧首轮 361、最终轮 360（该桥不保证每个源帧无损到达）；三轮对象均在 manifest 中，120 组物理时间完全一致的本体四元数比较通过。末端指令使机械臂最大变化约 0.1096 rad，夹爪行程变化约 0.01033 m；本次轨迹姿态误差峰值约 0.0190°，最终约 0.00186°。这些数值只代表该测试轨迹，不是任意操作下的稳定性保证。

**实时性能边界**：最终本机探针的 12 秒仿真消耗约 19.17 秒墙钟，实时倍率约 0.626；100/500 Hz 均指仿真时钟频率，不表示目前已实现墙钟 1× 实时。当前验证侧重物理正确性与链路，新增转子/控制后的实时性能仍需专项优化；不能只因画面连续就宣称已达实时指标。

另一个零重力隔离测试使 joint2 在 1–4 秒平滑运动 0.3 rad：开启控制时峰值约 0.3155°，20 秒时约 0.00436°；测试同时验证关闭控制后的残余误差显著更大。

所有测试仍不能替代长时间动量饱和、实物参数标定、真实 UE GPU/Pixel Streaming 和完整权威图像采集验收。本次没有改动 UE C++，轮体由现有 MJScene manifest/状态桥自然输出。
