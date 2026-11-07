# Benchmark 抓取臂动作与编码器边界

`CaptureArmCommand` 和 `CaptureEncoders` 是接触任务的公开组件接口，供后续完整环境接入。它们不注册 D54/D55/D62，不自行创建仿真，也不证明关节已按参考运动。

动作只控制原捕获臂：TCP 为 `grip_frame`，本体参考为 `cubesat_bus`，提供 TCP 线速度、本体/工具轴表达的角速度、末关节附加角速度，以及轴向/径向机构速度。不允许传入 qpos、目标真值、直接接触力或替换控制主体。`enabled=false` 必须同时为零运动请求。

当前版本明确限制 TCP 线速度范数 0.02 m/s、每个声明坐标系的角速度范数 0.1 rad/s、末关节附加速度 ±0.25 rad/s、轴/径向速度 ±0.003 m/s。前两项是本组件声明的 Benchmark 指令范围，不是实物硬件性能。实际参考仍经过原 `CaptureArmTarget` 的 IK、前六轴 ±0.25 rad/s、相对编码器 ±0.05 rad 领先限制及全部原关节范围。下面的原生绑定负责连接原伺服力限和真实执行读回；其完整物理验收单独记录。

编码器接口分别给出六个臂关节的 rad/rad·s⁻¹、轴向 m/m·s⁻¹、凸轮 rad/rad·s⁻¹及四爪 m/m·s⁻¹，带回合标识和整数采样时刻。此版本声明理想、无噪声、无时延的关节编码器；不包含星体位姿、目标状态、原生步数或控制参考。观测不会把越界数值裁回关节范围，以免掩盖物理约束违反。

`CaptureArmReferenceController` 只从这些编码器和动作计算十二通道伺服参考，不接触原生状态赋值。时间顺序为：

1. 初始化后，环境使用原始初始参考；每次 reset 重建控制器。
2. 在当前物理发布边界提交新动作，同一时刻的重复/未来/旧动作拒绝。
3. 下一个 400 Hz COMMAND 时刻使用紧邻的上一拍编码器，经原 IK 计算参考，然后由环境推进 PHYSICS。
4. 动作在 200 ms 时失效；原控制器只在失效转换时锁定一次实测位置，继续保持该参考。重新收到有效动作后从当前编码器继续，不回到折叠初态。

回拨、漏拍、重复控制拍、跨回合或不匹配的编码器均在更新参考历史前拒绝。输出 `CaptureJointReference` 明确是请求参考，不是测量或实际电机力；其内部混合 SI 十二维顺序对应原 `INITIAL_JOINTS`。

测试检查类型与私有字段边界、各轴单位、凸轮与四爪耦合、堵转时参考不持续积累、时钟拒绝、动作过期和新实例复位。这些是纯控制组件检查。完整环境仍须连接轨道、图像闭环、接触评分和整回合验收。

源码 `a3149c2` 在 SpaceSim 云端完成 **54 项接口/原捕获序列及视觉控制回归与 Ruff，零跳过**。报告 `.tmp/benchmark-capture-arm-checks-v1/acceptance.json` SHA-256 为 `78de548bb3078dbb1fac4971d7d3461396d5555832ba6694eddbdd0071b81f3f`。本套件不启动原生积分或 UE，尚不能据此确认物理执行。

云端检查命令：

```text
python -m pytest tests/test_benchmark_capture_arm.py -q -ra
```

## 原生编码器、伺服与私有执行回执

`simulation.benchmark_capture_arm_runtime.NativeCaptureArm` 只安装捕获臂，在首次物理初始化前绑定完整模型及其资源清单，并将十二路参考接到原伺服。所有通道须无其他控制者；它不接管操作臂、飞轮、推进器或目标星。环境仍负责合法初态、其他执行器、轨道、400 Hz 调度、证据记录和关闭；reset 必须重建原生核心和绑定。

`NativeCaptureEncoderReader` 读取每个原关节的位置/速度消息，同时核对原生整数时钟、步数和消息写入时间。未写入、陈旧或不在 400 Hz 栅格的消息，以及初始化后的物理状态赋值均被拒绝。公开返回值仍只含原 `CaptureEncoders`；实际星体/目标真值、伺服参考和私有诊断不进入方法观测。

原生发布器在 COMMAND 阶段写入当前拍参考，物理核心随后积分；RECORD 阶段调用 `evidence` 得到 `spacesim-autonomy-capture-arm-actuation/1`。回执分别保存编码器、因果参考、原生积分后请求 PID 诊断和实际求解器力，每路声明 N 或 N·m。实际力需与对应原生执行器读数精确相同，不能用请求力或限幅公式替代。t=0 尚无积分结果，`effort=null`，不伪造零力样本。发布器发生时序错误后锁定失败，须重建回合。

接线时修复了旧 `PhysicsPorts.servo_effort` 的通道映射：消息按伺服连接顺序存储，不能按包含飞轮等电机的完整 MJCF 执行器序号索引。混合/重排通道的检查不依赖 Basilisk。新原生夹具使用完整团队模型，对比原 `CaptureArmTarget` 直接驱动、接口驱动、读回开/关与重建后的每拍物理字节，另设无动作对照；该夹具仍为原始零重力装配布局，不属于轨道飞行、渲染或成功抓取验收。

端口/模型/原控制/调度范围在 `9c85f31` 通过 **120 项云端检查及 Ruff，零跳过**，报告 `.tmp/benchmark-capture-arm-runtime-checks-v1/acceptance.json` SHA-256 为 `abfaa0e63cc4d9476d2ea013080a1eb47cb8a6a2df99bc9b58cc555de3ec8518`。`af9c8e5` 补充拒绝零时刻复位残留旧动作或关闭的核心，**68 项受影响检查及 Ruff 通过，零跳过**，报告 `.tmp/benchmark-capture-arm-runtime-checks-v2/acceptance.json` SHA-256 为 `a8639ba1f0e3665c848f6e1b6920e5007a9e4f7d4593c2c2fb21b33665085600`。两套范围有重叠，不相加为独立用例数。

真实原生夹具在独立 `benchmark-capture-arm-v1-20261107` 工作树冻结为 `af9c8e595d0cc4a8b455d790a25191eb75532a32`，完整 1588 项原资产绑定已经核验。新建 native DLL SHA-256 为 `23186827e739d56707ab4c19a0c3af7e280149bed5b961867765a186e8fcfe17`，构建清单为 `386fa0a9bf1354a89c705db67b670aa644c94aefba88602b3482a3d821da82a6`；SDF 插件为 `bd09d255090eaded3b1f01dfeca3f7a29374e37e0e32542a4ce3ae76b989b93d`。夹具等待原接触组件释放所在通道，尚未运行；最多两路本人原生任务的限制保持。基础端口检查和构建成功不替代这项实际动力学验收。
