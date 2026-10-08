# 在线姿态偏好原生化与 MJScene 内部线程池实验

## 范围与默认值

- `SPACE_SIM_POSTURE_BACKEND=native`（默认）使用 `native/posture.cpp`。
- `SPACE_SIM_POSTURE_BACKEND=python` 保留原实现。
- 原生模块只计算参考修正：同一目标、约束、偏好权重、累计偏差预算、最多四次回溯。
  基础 DLS IK、实际状态、PID、反作用轮保护、动力学、接触和 RKF45 均不替换。
- 原生模式也将共享的 FK／Jacobian 几何遍历接入同一 C++ 内核，避免修正内外
  重复经过 Python 小矩阵运算；保留按关节数值缓存和公开接口返回独立数组的规则。
  基础 DLS 的线性代数求解与控制策略不变。普通 `SerialChainKinematics` 仍默认 Python，
  只有显式选择原生实时后端或调用 `enable_native_geometry()` 才切换几何实现。
- 编译产物保存在忽略目录 `run/native_acceleration/`，不提交 DLL。
  加载时核对源文件、二进制 SHA256 和 ABI；缺失/过期直接报错，不伪装成原生运行。
- 当前 C++ 模块支持六轴；其他关节数使用 Python 后端。
- **线程池仅在离线性能工具中开放，不接入生产场景默认启动。**
  它与在线姿态原生化是独立开关，不需要一起启用。

## 构建

使用与仿真匹配的 Python，从服务端仓库根目录运行：

```powershell
python tools/build_native_acceleration.py --component posture
```

需要已安装 MSVC x64 Build Tools 和 Windows SDK。构建助手读取编译器环境，
使用优化编译但禁用快速浮点近似（`/fp:strict`），不安装 Python MuJoCo，
不替换 Basilisk 或其他环境包。

线程池探针另需**完全匹配 3.7.0** 的 MuJoCo C 头文件：

```powershell
python tools/build_native_acceleration.py --component mjscene_probe `
  --mujoco-include <MuJoCo-3.7.0的include目录>
```

可以使用独立规划环境中的 `mujoco/include`，这里只读头文件，
**不将该环境的 MuJoCo DLL 或 Python 包加载到仿真进程。**

## 测试与测速

```powershell
$env:SPACE_SIM_POSTURE_BACKEND = "native"
python -m pytest tests/test_online_elbow_ik.py tests/test_native_posture.py `
  tests/test_native_acceleration_config.py -q
```

等价测试覆盖随机状态、偏好开关、非默认上方向、低速/零指令、累积预算、
释放与恢复、重置，以及非法输入。对照保留的 Python 函数，而非仅验证结果有限。

公平测速使用**独立子进程依次运行**，不要同时运行其他重负载测试：

```powershell
python tools/benchmark_native_acceleration.py `
  --adapter-root <UE适配器仓库> `
  --scene-instance <已保存场景JSON> `
  --output-root run/native-benchmark-new `
  --duration 8 --motion linear_x --speed 0.05 --repeat 2
```

场景仅复制到输出目录，并明确从其保存的操作姿态启动。
不连接实际后端、UE，不写采集数据，不改原场景；积分器、容差、
240 Hz 动力学／120 Hz 控制／30 Hz 渲染状态输出保持原设置。
统计排除启动、GPU、网络和数据集写盘，所以 **compute RTF 不是完整平台实时倍率**。
工具记录每次命令、耗时、观测、约束数量、CPU 时间和状态最大差异。
重复次数和时长仅是测速参数，不能代替长时间接触稳定性验收。

单独测试组合：

```powershell
python tools/profile_simulation_runtime.py `
  --adapter-root <UE适配器仓库> --scene-instance <场景JSON> `
  --output run/native-one.json --initial-state operating `
  --duration 8 --motion linear_x --speed 0.05 `
  --posture-backend native --mj-threads 2
```

`--mj-threads 0` 表示无池的串行基线；1/2/4/8 表示额外工作线程数，
**还要加上调用线程**。0 也执行相同的计时区间外观测，保持对照一致。

## 线程池适配边界

目前仅支持 Windows x64、Basilisk 2.11.1、其实际加载的 MuJoCo 3.7.0。
版本不符直接拒绝；不猜测 C++ 对象字段偏移。

SWIG 未暴露 `getMujocoData/getMujocoModel`，实验适配器使用当前 DLL 导出的
公开 C++ getter（Windows x64 ABI），再把线程池绑定到该场景实际使用的 `mjData`。
头文件编译的探针检查绑定地址、栈、模型维度；不会另建第二个物理世界。
只在初始化完成、栈为空时绑定；只在同步仿真结束后拆除并销毁。
MuJoCo 3.7.0 没有公开解绑函数，因此清除绑定使用版本限定的小型 C 探针；
不允许运行中切换、复用旧池或在工作线程尚活动时释放。
在升级 Basilisk/MuJoCo 后必须重新审查并验证，不应直接修改版本检查来绕过。

池越大可能越慢，特别是只有少量接触或单个约束岛的模型。
优先看墙钟耗时、状态差异和长帧；CPU 占用升高不等于仿真加速。

## 本机实测结果（2026-09-28）

测试环境为 Windows x64、16 逻辑 CPU 的虚拟机，Basilisk 2.11.1 /
MuJoCo 3.7.0。使用已保存的
`scene-20260927-215941-d07c642a`，复制后从其操作姿态开始；
不启动或控制前端正在使用的场景。这里的“原生”包含姿态偏好修正和共享
FK／Jacobian 几何内核，不是反作用轮或动力学原生化。

所有对照保持原来的 RKF45、相对／绝对容差 `1e-4`、
240 Hz 动力学、120 Hz IK 和 30 Hz 状态输出。
本节数据使用当时的 J6 力矩上限 `0.35 N·m`；之后的 `0.70 N·m`
单参数试验单独记录于 `docs/JOINT6_TORQUE_TRIAL.md`，不混入本节性能结论。
保留现有 `extraEoMCall=false`，不靠减少步数、放宽容差或关闭接触取得加速。
独立进程顺序执行，重复轮次交换顺序；测速时不并行运行其他回归测试。

### 计算耗时

连续 X 向平移 `0.05 m/s`，推进 **8 秒仿真**，每组两次：

| 在线后端 | MJScene 额外工作线程 | 执行耗时中位数 |
| --- | ---: | ---: |
| Python | 0（无池） | 8.200 s |
| 原生 | 0（无池） | **7.515 s** |
| 原生 | 1 | 8.034 s |
| 原生 | 2 | 9.919 s |
| 原生 | 4 | 11.566 s |
| 原生 | 8 | 12.909 s |

原生且无池的耗时下降 **8.36%**。所有线程池配置均慢于“原生且无池”，
不是说 1 个工作线程一定比原 Python 慢：它仍保留了部分原生化收益。

延长与补充工况：

| 工况 | 每组重复次数 | Python 无池耗时 | 原生无池耗时 | 耗时下降 |
| --- | ---: | ---: | ---: | ---: |
| X 向平移 `0.05 m/s`，20 秒仿真 | 2 | 19.255 s | **16.661 s** | **13.47%** |
| Roll `0.5 rad/s`，8 秒仿真 | 1 | 7.274 s | **5.965 s** | **18.00%** |

20 秒测试的 compute RTF 中位数从 **1.039** 提升至 **1.201**。
耗时和 RTF 分别取每次结果的中位数；“耗时下降百分比”不等同于
“吞吐率提高百分比”。转动只是单轮补充测试，不能与重复测试视为同等强度的证据。
转动加 2 个工作线程耗时为 9.833 s，同样慢于无池原生方案。

### 线程池确实工作，但不适合本次模型

探针核对的是 MJScene 实际使用的 `mjData` 上的池地址和实际线程数，
不是新增 Python 线程，也不是另建一个物理世界。
例如一轮 8 工作线程测试，墙钟耗时 11.374 s，而进程 CPU 时间为
101.203 s，平均约占用 8.90 个 CPU 核，仍慢于无池方案。

平移测试在 30 Hz 采样点的最大接触数为 0，最大约束岛数为 1；
转动测试的采样最大接触数为 1，最大约束岛数仍为 1。
这与“小模型可并行工作量不足，线程调度／同步成本抵消收益”的解释一致；
并非逐 RK 子步的完整接触记录，也不表示已分别测出每种内部开销。
**因此保留线程池为离线实验，不接入正式启动。**

### 数值一致性与范围

原生专项回归最终为 **119 passed**，包括随机 Python/C++ 对照、
连续状态／预算、几何与 DLS 对照、错误配置和测速的场景运行保护。
另外在不设置后端环境变量时，默认 Python 回归为 **131 passed**；
显式选择原生后端的遥操作／参考保护补充回归为 **43 passed**。
这些运行存在重叠测试，不将三组数字相加作为不同测试用例总数。
另外，对上述全部 19 次完整仿真试验，按相同仿真时间戳比较保存的观测：

| 量 | 相对各工况 Python 基线的最大绝对分量差 |
| --- | ---: |
| 机械臂实际关节角 | `1.56e-8 rad` |
| 机械臂实际关节角速度 | `6.45e-8 rad/s` |
| 机械臂参考关节角 | `4.39e-10 rad` |
| 末端位置 | `7.08e-9 m` |
| 星体惯性姿态四元数分量 | `7.82e-9` |
| 星体角速度 | `7.90e-9 rad/s` |
| 反作用轮转速 | `2.15e-5 rad/s` |
| 反作用轮实际力矩 | `1.11e-7 N·m` |

在这些采样点，姿态控制饱和、反作用轮力矩限制、速度限制和超速标志
均无不一致。实际状态仍由原来的权威动力学链路推进；原生部分仅替换
参考计算的实现。浮点运算次序不同，结果并非逐位相同；
短时对照不能代替长时间复杂接触／临界切换验证。

### 留档与当前启用状态

原始命令、每轮日志、计时、复制的场景和观测均在忽略目录：

- `run/posture-native-20260928/final-matrix/summary.json`
- `run/posture-native-20260928/final-long/summary.json`
- `run/posture-native-20260928/final-rotation/summary.json`
- `run/posture-native-20260928/final-state-audit.json`
- `run/posture-native-20260928/regression-acceptance.log`
- `run/posture-native-20260928/regression-python-default.log`
- `run/posture-native-20260928/regression-native-control.log`

同目录早期原型数据不参与上述结果汇总。
这些均为**仿真计算区间**的结果，不包括启动、UE 渲染／回读、
网络与录制写盘，不能直接当作前端端到端实时倍率。
重复次数有限且虚拟机存在调度波动，不保证任何运动／接触工况都有同样比例的收益。

此次只完成实现和离线验证：正式启动默认已切到原生（生产插头模型的扫动工况用 Python 姿态后端达不到实时，见 [LOCAL_DYNAMICS_BACKEND.md](LOCAL_DYNAMICS_BACKEND.md)）；要回到 Python，
需在启动服务／仿真进程的环境中显式设置 `SPACE_SIM_POSTURE_BACKEND=native`
并重启对应进程，已运行进程不会因另一终端设置变量而自动切换。
没有自动重启服务、操作实际采集，也没有提交或推送代码。

## 回退

```powershell
$env:SPACE_SIM_POSTURE_BACKEND = "python"
```

重新启动仿真子进程即可；不需重建场景模型或改物理参数。
当前生产入口没有线程池开关，因此实验不会改变现有服务的线程配置。
