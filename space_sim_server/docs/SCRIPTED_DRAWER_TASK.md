> 运行示例中的 `SPACE_SIM_PYTHON` 需先按 README 配置为本机平台 Python；场景路径需替换为本机实际生成的场景 JSON。

> 当前默认频率已更新为动力学 400 Hz、脚本执行和六维力 200 Hz、主数据和相机 50 Hz；下文旧 240/30 Hz 记录属于此前验证。详见 [多频率采集](MULTIRATE_CAPTURE.md)。

# 完整抽屉循环与批量采集

当前 `run_drawer_task.py`、`record_drawer_task.py` 默认任务为 `drawer_cycle`，流程为：settle → raise → above_drawer → align → open → descend → grip → lift → hold → return → seat → release → retract → verify。旧的仅提拉任务通过 `--task drawer_lift` 保留。

- 抽出 8 cm 后保持 2 秒；回程参考线速度上限 0.012 m/s。
- seat 阶段增加 1 mm 的限速目标压入以克服伺服负载偏差，必须小于最终 2 mm 关闭容差；并保留原载荷保护。
- 回程时夹爪仍压住卡扣，因此先检查位置归位，再松爪；不要求卡扣在受夹持时复位。
- 最终成功须已验证抽出和归位、夹爪开合量大于 0.95、末端撤离到悬停位、抽屉距初始位置不超过 2 mm、速度小于 3 mm/s、卡扣距初始角不超过 0.025 rad，并连续稳定 2 秒。这些是当前仿真模型的验收容差，不是硬件精度保证。
- grip 阶段还要求两个手指都与抽屉或卡扣产生接触，避免仅凭手指没闭合误判抓取。接触信息从权威 MuJoCo 模型只读取得。
- 所有运动通过执行器目标完成。任务执行期间不写 qpos、不焊接夹爪、不更改卡扣或接触几何。

## 批量入口

```powershell
python tools/collect_drawer_cycles.py --scene run/scenes/<你的场景ID>.json --output run/my-drawer-batch --count 5 --seed 2026101501 --range-deg 10
```

每次尝试使用 `seed + attempt_index` 的独立 PCG64 随机流，六个关节相对来源场景姿态独立均匀采样 ±10°，夹爪固定闭合、关节初速度为零；原场景文件不变。光照、轨道、动力学参数保持来源配置。关节观测噪声、偏置、动作噪声、延迟、腕力噪声、外力扰动全部关闭；本入口尚不支持开启这些扰动，不能将配置标记设为 true 并期望其生效。

预检查使用同一 Basilisk MuJoCo 运行时、原模型几何与独立的模型状态：检查关节限位；初始及接近路径 244 个离散姿态检查超过 0.5 mm 的机器人相关穿透；忽略夹爪内部四连杆接触；并验证抓取、抽出、返回与撤离的 IK 可达性。预检查不是连续碰撞证明或动力学成功保证，接触阶段仍靠真实运行验证。

批量默认顺序执行独立仿真和 UE 实例，240 Hz 动力学、1 子步、30 Hz 采集、双路 1024×1024。独立端口默认为 19766/19767/16558/19888。每条录制复用正式 CaptureReceiver / SimulationHub / EpisodeRecorder / LiveLeRobotWriter，不从离线 trace 伪造训练数据。

每轮保存原始采样、预检查结果、场景副本、阶段日志和录制结果。失败录制保留 outcome=failure，初始化拒绝也进入批次 summary；不计入成功数量。最多尝试次数可通过 --max-attempts 限制。每条成功轨迹还要通过逐帧 JSONL/Parquet 状态、动作、腕力一致性、图像源帧/时间、图像尺寸及完整 MP4 解码检查，才加入 accepted_episodes。

数据在 data/episodes，批次汇总和验证在 --output。若进程被中断，已写入的录制与批次审计保留，重新启动前应先检查本次子进程与端口；输出目录不覆盖已有目录。

以下是旧提拉任务的实现背景和入口参数说明（需要仅提拉时显式指定 --task drawer_lift）：

# 自动抽屉提拉任务

第一版可运行的自动操作脚本，针对 `SARM_20261012` 模型最右侧的抽屉。它在独立的权威动力学实例中运行，使用平台原有的 BSK 地球/太阳引力、自由基座、姿态控制、本地 MuJoCo implicitfast、夹爪执行器、卡扣及碰撞几何。

## 运行

在服务端目录，用已安装 Basilisk 的平台 Python 执行：

```powershell
# 从统一仓库根目录进入服务端
Set-Location .\space_sim_server
& $env:SPACE_SIM_PYTHON tools\run_drawer_task.py
```

不需要启动网页或 UE。脚本独立创建仿真实例，不接管当前网页中正在运行的场景。返回码 `0` 表示任务验收成功，`1` 表示失败。每次运行创建一个独立目录 `run/drawer-task/年月日-时分秒/`，已有目录不会被覆盖。

其他机器应将解释器路径换成该机器的平台 Python。原生本地步进 DLL、SARM SDF 插件及 SPICE 资源须已按平台启动说明准备好。

```powershell
# 显式指定物理子步和结果目录
& $env:SPACE_SIM_PYTHON tools\run_drawer_task.py --substeps 3 --output run/drawer-task/my-trial
```

默认沿用平台的子步设置；当前平台默认 1。物理外层频率为 240 Hz，任务决策与日志为 30 Hz，不使用墙钟 sleep 安排动作。机器变慢时，完成所需现实时间会增加，动作的仿真时间顺序不变。

## 动作与成功标准

1. 从当前代码的标准操作姿态 `[90, -60, 60, 0, -90, 0]` 度初始化，短暂停稳。
2. 留出高度，再移动到抽屉上方。
3. 原地调整夹爪方向，让两指跨在手柄两侧。
4. 张开，缓慢下降到抓取高度。
5. 闭合，通过实际夹爪开度判断是否碰到了抓取物。
6. 沿抽屉滑动轴提起 **80 mm**。
7. 检查抽屉实际位移与目标相差不超过 5 mm、抽拉速度低于 3 mm/s、夹爪保持夹持开度且与抓取点偏差小于 5 mm，并持续保持 **2 秒**。空载定位容差为 3 mm；承载提拉时允许伺服在接触载荷下有 5 mm 的位置偏差。

只下发关节位置/速度和夹爪执行器参考。运动开始后不直接修改物体位置，不增加焊接约束，不移除卡扣，不修改模型碰撞。抽屉必须通过真实接触随夹爪运动，机械臂到位而抽屉没跟上会报错。

抓取点在 `drawer_base` 局部坐标下为 `[0, 0, -0.005]` m；抽屉关闭时，在卫星本体系约为 `[0.455, 0.105, 0.035]` m。抽拉方向从 `drawer_base_slide` 的轴及刚体姿态读取，因此共同轨道平移不会改变任务目标。

臂关节参考速度不超过 0.08 rad/s，夹爪参考速度为 0.2 rad/s。腕部原始载荷持续 0.1 仿真秒超过 80 N 或 8 N·m、传感器不可用、动作超时、IK 不可达、提拉失败都会结束任务并记录失败原因。这是位置控制脚本加状态/载荷检查，还不是力控制器。

## 输出与验收范围

- `summary.json`：成功/失败、原因、模型哈希、参数、阶段时间、实际抽屉位移和动力学配置。
- `trace.jsonl`：30 Hz 的关节实测值、关节参考、末端姿态、抽屉/卡扣状态、带原始采样时刻的腕部六维读数、局部物理状态。

腕部读数来自正在积分的本地 MuJoCo 实例，是原始接口载荷。日志中的最大载荷是 **30 Hz 记录样本的最大值**，不代表所有物理子步中的峰值。详细定义见 [腕部六维测力](WRIST_FORCE_TORQUE_SENSOR.md)。

单独运行 `run_drawer_task.py` 只输出任务验收记录。需要与手动录制相同的数据集时，使用下面的真实 UE 录制入口。

当前路径针对给定模型及默认初始姿态验收，不是任意布局通用的避障规划器。`--initial-joints-deg`、`--grip-offset-m`、`--lift-m` 可供实验，但修改后应重新进行动力学验收。卡扣的材料、摩擦与结构精度仍取决于原模型。

## 完整录制：真实 UE 相机与原数据链路

```powershell
& $env:SPACE_SIM_PYTHON tools\record_drawer_task.py --scene run\scenes\<你的场景ID>.json
```

将 `--scene` 指向要使用的场景配置。脚本创建该配置的独立实例，沿用模型、轨道、光照、初始姿态、场景种子和操作者归属；不控制网页里正在运行的实例。当前要求此任务盒模板、240 Hz 动力学、30 Hz 采集，以及已准备好的初始姿态。需要本机 UE 5.6、已编译插件及对应的 `sarm_20261012.catalog.json`。

任务策略 → 关节执行器参考 → 原权威动力学 → RenderComponent / ObservationComponent → UE 双相机 → CaptureReceiver / SimulationHub → EpisodeRecorder → 原 LeRobot v3 写入器。开始录制的确认到达后才开始动作；停止时先冻结采样边界、关闭采集并排空已接受图像，再完成数据封装。队列满时暂停推进仿真，按仿真时钟保持 30 Hz 配对采样。

- 数据保存到 `data/episodes/episode-日期-时间-标识/`，与网页录制同目录、同 `space-arm-episode/2` 和 LeRobot v3 格式。
- 两个相机为 `spacecraft_overview` 和 `sarm_wrist_cam`，新录制均为 1024×1024 RGB、30 FPS（此前验收记录仍为 640×360），最终存为 MP4；配套 Parquet 包含关节状态、速度、末端位姿、动作参考和六维腕部载荷。
- `action` 为采样时实际保持的七维训练目标（六个关节角 + 0–1 夹爪开合量）；原始八维参考另存为诊断字段，见 [训练表示](GRIPPER_TRAINING_REPRESENTATION.md)。自动策略来源、阶段和成功判定放在已有 `observation.platform_json.scripted_task` 内，不增加新的训练特征列。不生成虚构的键盘/遥操作事件，因此没有手动操作时不会生成相应的 `actions.jsonl`。
- 默认使用独立端口 18766 / 18767 / 15558 / 18888。启动前检查占用，只清理本次创建的进程。可用 `--adapter`、`--unreal`、`--episodes-root`、`--output` 指定路径。
- 完成后检查图像数 = 状态数 × 2、无缺失及拒收样本；动作失败仍封装为 failure，程序返回 1。详尽日志及运行汇总保存在 `run/drawer-recording/`。

## 代码组织

- `simulation/scripted_tasks/drawer_lift.py`：任务参数、连续 IK 路径、仿真时间状态机、成功/失败判定；只依赖观测数据和运动学，不操作物理状态。
- `tools/run_drawer_task.py`：复用现有组件装配，创建仿真、读取权威状态、发布执行器参考、保存结果。
- `simulation/scripted_tasks/capture_transport.py`：将脚本观测与执行器参考接入现有可靠传输和权威相机帧。
- `tools/record_drawer_task.py`：独立 UE 实例及生产采集服务的启动、录制、停止与完整性检查。
- `tests/test_scripted_capture.py`：场景隔离、观测兼容、参考指令及完整性检查。
- `tests/test_scripted_drawer_task.py`：速度约束、时间规则、载荷保护、传感器丢失、不可达目标和错误成功判定的回归测试。

增加其他自动任务时可以在 `simulation/scripted_tasks/` 下添加独立策略，继续通过观测 → 参考指令接口驱动同一套动力学和执行器。

```powershell
python -m pytest tests/test_scripted_capture.py tests/test_scripted_drawer_task.py tests/test_serial_chain_kinematics.py tests/test_linkage_gripper.py -q
```
