# 多频率控制与采集

默认使用统一仿真时钟：动力学 400 Hz（2.5 ms，默认单子步）；在线 IK 100 Hz；抽屉脚本执行/反馈检查 200 Hz；六维力采样 200 Hz；状态和双 1024×1024 相机 50 Hz。频率集中定义在 backend/space_arm_platform/sampling.py。

## 执行关系

物理步始终按绝对整数纳秒调度，不累计近似周期。在线 IK 每四个物理步求解一次。抽屉脚本保留阶段进入时的路径 IK 规划；每两个物理步按仿真时间插值关节目标、检查力和到位条件，不把重复求解固定路径作为 100 Hz 在线 IK。

原生关节伺服在每个物理步执行。位置/速度限值、阶段持续时间仍用秒和 SI 单位定义。机体/站点状态、UE 发布及姿态导航保持相位一致；当前姿态链从 30 Hz 调为 50 Hz，控制增益保持不变。BSK 差分环境力默认仍按每个发布区间计算并保持，因此从 30 Hz 变为 50 Hz；可独立配置其 stride。可选大气组件保持 10 Hz。

## 六维力数据

WrenchSampler 放在动力学之后的 SENSOR 段，直接读取原生 sensordata，不依赖 50 Hz 的机体消息。每两步得到一条真实样本，按批次附在 observation.wrist_wrench_samples；50 Hz 状态帧通常携带四条力样本。队列有界，溢出、缺失、重复、传感器变化会报错，不补值或静默丢样。

现有逐帧 observation.wrist_wrench 保留。完整 200 Hz 流另外保存为：

`episode/lerobot/high_frequency/wrist_wrench.jsonl`

`lerobot/meta/platform.json` 的 high_frequency_wrist_wrench 描述其路径、200 Hz 频率、数量、坐标系与时间定义。七维 state/action 和八维原始关节诊断不变，schema 保持 space-arm-lerobot/2，新增字段为兼容扩展。六维力噪声继续关闭。

### 时间语义

- sim_time_ns / interval_end_time_ns：完成该物理步后的仿真时刻，用于和主状态对齐。
- sample_time_ns：MuJoCo 实际测量所在的最后子步起点，不能冒称与步末完全同时；单子步时比 interval_end_time_ns 早 2.5 ms。
- timestamp：相对第一帧主数据的区间末时间，单位秒。

录制开始前的样本被裁掉。若第一帧已经完成物理步，则第一帧保留一条对齐的样本，以后每个 20 ms 区间四条；因此 N 帧通常对应 4(N-1)+1 条力数据。仿真零时刻尚无有效测量，不伪造初始力样本。

录制器保留 LeRobot 的单一 50 Hz 主表，通过独立文件提供不同频率的信号，不把 200 Hz 序列平均成 50 Hz。停止时只保存至最后完整主帧，图像和状态仍严格一一对应。

## 兼容、启动与查看

旧 30 Hz 数据仍可读取/回放。新在线采集支持 1、2、5、10、25、50 Hz，均为 50 Hz 发布频率的整除数。旧场景文件仍记录旧频率，不能直接当作新配置：交互平台应重新创建场景；脚本录制入口会生成新的场景副本，并同步更新 dynamics_rate_hz、dynamics_step_s、ik_rate_hz、capture_rate_hz 等字段，不改写原场景或历史记录。

修改 UE 的 BskLeRobotSampling.h 后必须完整编译 Runtime 插件并重新启动 UE。旧后端/UE 进程也需重启以加载新代码。

独立 data_display 查看器增加 200 Hz 原始力/力矩曲线，采用独立时间轴；50 Hz 逐帧曲线继续保留。CSV 导出使用各信号时间点的并集，没有采样的单元格留空，不插值制造数据。

## 验证

验证真实调度器的 400/100 Hz 网格、物理步后 200 Hz 采样、跨帧连续性、前滚裁剪、漏样/重复/缺尾拒绝、历史 30 Hz 数据兼容和 50 Hz 相机编码。完整抽屉运行及真实 UE 录制报告存放在 run/drawer-cycle-400hz-baseline 和 run/drawer-cycle-400-200-50-final。运行数据不提交。
