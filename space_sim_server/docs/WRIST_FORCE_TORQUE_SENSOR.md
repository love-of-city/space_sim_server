# 腕部六维力/力矩传感器

## 使用范围

新版 SARM 与完整任务盒（2026-10-12·实验）模型增加腕部测力。选择该模型后，网页遥测显示腕部 X/Y/Z 三向力（N）、三向力矩（N·m）、测量坐标系和采样时刻。旧滑动夹爪模型不安装本传感器，字段为 `null`，界面显示“—”。

该功能最初在 `feature/wrist-force-torque-sensor` 独立 worktree 开发，现已合入原工作目录 `C:\Users\LYH\space_sim_unified` 的 `perf/sarm-thread-sdf` 分支。重启该目录下的仿真场景后使用新动力学助手，网页加载本目录的新前端构建。UE 沿用已有状态渲染协议，本次没有修改 UE C++。

## 安装在哪里

```text
机械臂 link5
└─ ee_base：保留 joint6 和腕部相机
   └─ wrist_ft_tool：固定连接，无额外关节
      ├─ wrist_ft_site：测力原点和坐标轴
      ├─ 原 ee_base 的质量、惯量、几何
      └─ 原整个夹爪的子刚体和连杆
```

传感器相当于在第六关节输出端与整个夹爪之间设置一个测量截面。`wrist_ft_tool` 相对 `ee_base` 的位置为零、姿态为单位旋转；原夹爪质量和惯量整体移到截面下游，运动自由度保持不变。按刚体名称指定的碰撞排除规则也随原夹爪几何迁移到 `wrist_ft_tool`，关节和相机引用保持在 `ee_base`。原交付模型不修改，平台模型由 `tools/assets/build_sarm_20261012.py` 生成。

这是理想刚性、零新增质量的传感器，没有模拟真实测力器的厚度、弹性、噪声、温漂或量程饱和。以后有实体型号参数时，可以再扩展这些特性。

## 怎么计算和读取

1. BSK 环境组件和控制器照常提供环境与驱动输入。
2. 权威 `LocalMujocoStepper` 使用原生 `implicitfast` 步进。
3. MuJoCo 原生 `<force>` 与 `<torque>` 计算截面载荷，均绑定 `wrist_ft_site`。
4. 原生助手保留最后一个子步的传感器样本；Python 只读取该样本，不为测力额外执行 `mj_forward`。
5. 观测消息携带 `wrist_wrench`；后端校验后发给网页，并交给现有录制链路。

读数来自真正推进动力学的 MuJoCo 实例。用于发布/渲染的 MJScene 副本不另算测力。参考点 O 重定位会触发状态重建，但不会覆盖已经保存的传感器样本。

六维读数是**原始接口载荷**，包括下游夹爪的惯性载荷、环境作用和接触载荷。它不是单独的接触力，也不等于第六关节电机的驱动力矩。想得到纯接触力时，需要依据任务做补偿/估计；本次没有实现该算法或 VLA 控制器。

## 坐标、方向、时刻

- `force_n = [Fx, Fy, Fz]`，单位 N。
- `torque_nm = [Tx, Ty, Tz]`，单位 N·m，取矩点为 site 原点。
- 全部分量沿 **site 自身坐标轴** 表达，随腕部一起旋转；不是惯性系或卫星本体系分量。
- `sign_convention = "mujoco_force_torque"`：直接保留原生符号。已知载荷测试中，固定工具在 x=0.2 m 处受到 +10 N 的 Y 向外力和 [1,2,3] N·m 外加力矩时，读数为 [0,-10,0] N、[-1,-2,-5] N·m。转换到另一坐标系时，力和力矩都要旋转；改变取矩点还需要力臂修正。
- `sample_phase = "last_substep_start"`：原生步进计算该子步起始状态上的传感器读数，随后推进位置和速度。因此读数的 `sample_time_ns` 略早于本次观测的 `sim_time_ns`。
- `interval_end_time_ns` 与本次观测的 `sim_time_ns` 完全一致；两个时间戳都用十进制字符串传输，避免浏览器整数精度问题。
- 初始化和重置后的第一帧尚未完成动力学步进，`valid=false`，数值零仅是占位，界面不显示成“测得零力”。随后每个有效样本满足 `sample_time_ns < interval_end_time_ns`。

合并后的运行参数沿用性能优化分支：240 Hz 主时钟，默认每步 1 子步（可配置），默认每 8 个物理步发布运动学状态、更新一次差分环境力。原生测力仍随 MuJoCo 每个子步计算；默认情况下最新样本比状态时刻早约 4.167 ms，配置 5 子步时约为 0.833 ms。保留优化后的 SDF 插件和相机布置，不因测力恢复旧的 5 子步默认值。

## 录制与扩展

现有观测发布网格为 30 Hz。录制继续使用选择的 1/2/5/10/30 Hz，每帧取最新样本，不做平均；更快的接触峰值可能发生在两次记录之间。未来做高频力控，需要直接在动力学控制环读取样本或增加独立高频记录通路。

LeRobot 新增字段：

| 字段 | 内容 |
| --- | --- |
| `observation.wrist_wrench` | float32[6]，Fx_N/Fy_N/Fz_N/Tx_Nm/Ty_Nm/Tz_Nm |
| `observation.wrist_wrench_valid` | int64[1]，有效标记 |
| `observation.wrist_wrench_sample_time_ns` | int64[1]，原始仿真采样时间 |
| `observation.wrist_wrench_interval_end_time_ns` | int64[1]，本次状态时刻 |

`observation.platform_json`、`steps.jsonl` 保留完整观测；`lerobot/meta/platform.json` 保留坐标和符号定义。没有传感器的旧模型不生成这些数值特征。同一 episode 中传感器消失或定义变化会报错，避免混入不同测量含义的数据。

通用读取接口为 `LocalMujocoStepper.read_wrench(force_sensor, torque_sensor)`。Python 的 `WrenchSensor` 用具名配置描述一对传感器，腕部只是其中一个配置。增加其他测力位置时，可以复用这两层，但需另外添加对应观测字段和录制定义。

## 构建和验证

在本 worktree 的 `space_sim_server` 目录，使用平台仿真 Python：

```powershell
$env:PYTHONPATH="$PWD\backend;$PWD;$PWD\..\space_sim_UE_Adapter\Adapters"
$env:SPACE_SIM_RESET_ADAPTER=(Resolve-Path ..\space_sim_UE_Adapter).Path
python tools/assets/build_native_acceleration.py --component all
python tools/assets/build_sarm_plugin.py
python -m pytest tests/test_wrist_wrench_native.py tests/test_wrist_wrench_recording.py tests/test_sarm_20261012_model.py -q
python -m pytest tests/test_simulation_reset_native.py -k sarm_20261012 -q
python tools/diagnostics/validate_sarm_20261012_platform.py --output run/wrist-ft-development/cycles.json
python tools/benchmarks/benchmark_wrist_sensor.py
npm --prefix frontend ci --no-audit --no-fund
npm --prefix frontend test
npm --prefix frontend run build
```

运行重置测试前需要按本机设置 `SPACE_SIM_POSTURE_PYTHON` 为隔离姿态环境解释器；不要在同一进程混用 Python MuJoCo 和 BSK 内嵌 MuJoCo。

原生验收覆盖已知力和力臂、site 旋转、接触载荷、共同加速度、测量不改变轨迹、重定位缓存；真实场景测试覆盖两次重建和 BSK 轨道环境下的有效样本。另有 40 秒、三次夹爪全行程测试和真实 LeRobot 写入/读回。这里的自动测试没有包含完整 UE 图形界面操作验收。

## 原功能 worktree 的历史耗时对照（2026-10-14）

以下结果来自合并前的传感器 worktree（主分支基础、5 子步、原 SDF），不代表合并后性能优化分支的实时速率。使用相同原生 DLL，在该模型上分别保留传感器和撤销测量截面/传感器，交替执行各三次。每次预热 1 秒仿真时间，再测量 3 秒夹爪张开；主时钟 240 Hz、每步 5 子步，测力版本同时以 30 Hz 读取与组装观测。

| 项目 | 未安装 | 已安装 |
| --- | --- | --- |
| 实际耗时中位数 | 9.729 s | 9.288 s |
| qpos / qvel 数量 | 68 / 61 | 68 / 61 |
| 最大接触数 | 29 | 29 |
| 最大求解迭代数 | 10 | 10 |

中位耗时变化约 -4.53%，没有观察到明显减速；本机短测受运行负载和模型组织影响，不能据此宣称传感器能够加速仿真。对照运行最终 qpos/qvel 的最大绝对差均为 0。这个短期、固定动作对照不代表所有接触工况；它未启用完整 BSK 轨道环境，也不包含 UE、网页、网络或录制成本，不能直接当成平台实时倍率。

可通过 `tools/benchmarks/benchmark_wrist_sensor.py` 重现方法。本机原始记录在忽略目录 `run/wrist-ft-development/benchmark.json`；测试日志和构建日志也在该目录，不提交到仓库。

合并前功能 worktree 的自动测试结果：服务端基础测试 727 项通过；前端 172 项通过且生产构建成功；适配器基础测试 72 项通过（另 3 个子测试通过）；原生测力与真实场景重置 9 项通过；碰撞规则迁移后的模型/协议补充测试 27 项通过。完整 UE 图形启动与手柄操作尚未人工验收。

## 合入性能优化分支（2026-10-14）

六维测力功能提交 `5159d3a` 已通过合并提交 `0875b9f` 合入 `perf/sarm-thread-sdf`。实际工作目录为 `C:\Users\LYH\space_sim_unified`。保留该目录已有的默认 1 子步、30 Hz 运动学发布/差分环境力更新、优化 SDF 插件及相机配置。重叠文件按共同基线整合，平台 MJCF 与 manifest 由合并后的生成器重新生成。

合并后验证：服务端基础测试 729 项、前端 172 项、适配器 77 项（另 3 个子测试）全部通过；原生测力、差分环境力保持及场景重置 14 项通过，其中真实新版场景分别用默认 1 子步和自定义 3 子步验证两次重置。动力学助手已重新编译，SDF 优化插件校验通过，前端生产构建通过。

原工作目录既有的性能优化和资产修改仍保留为未提交修改；本次未将这些原有工作混入传感器提交，也未推送远程。343 个非重叠的原有修改文件已核对字节哈希一致。重叠文件原件及校验日志保存在本机忽略目录 `space_sim_server/run/wrist-ft-merge-20261014-133126/`。

正在运行的旧平台进程不会自动更换已加载的 Python 模块、模型和 DLL。使用合并版本时，从原工作目录重启平台及仿真场景并刷新网页。本次没有执行完整 UE 图形界面的人工操作验收。
