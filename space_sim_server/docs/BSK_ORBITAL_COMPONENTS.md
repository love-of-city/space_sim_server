# BSK 在轨环境组件与本地 MuJoCo 的接入边界

日期：2026-10-04。适用于默认 `local` 后端；不需要修改或重新编译 Basilisk/MJScene。

## 1. 职责与默认行为

- BSK `Spacecraft` 仅推进参考点 O，参考点仅受本页注册的引力场作用。
- BSK `NBodyGravity.computeAccelerationFromSource()` 与原生 `GravityModel` 计算各刚体质心和 O 处的场；没有手写点质量或球谐引力公式。
- MuJoCo `implicitfast` 是局部刚体、关节、接触和执行器状态的唯一推进者，现有隐式伺服不变。
- MJScene 继续作为兼容状态/消息发布接口，UE 协议不变。
- 正式 SARM 会话默认 `bsk`，包括地球和太阳的逐体差分引力。`linear_tidal` 是明确选择的旧地球线性潮汐回归路径。
- 独立构造 `LocalMujocoStepper` 并不自动创建星历或场景环境；由场景装配组件安装 provider。

这里的“完整场求值”是相对于线性潮汐近似而言，并非开启全部环境模型：默认仍是地球/太阳点质量，不自动启用 J2、阻力、太阳辐射压、磁力矩或燃耗。

## 2. 分层与扩展接口

| 层 | 实现 | 扩展规则 |
|---|---|---|
| 参考点、共同星历 | `components/orbit.py` | 继续使用 BSK `Spacecraft`、`gravBodyFactory`、SPICE |
| 场景安装 | `components/orbital_gravity.py` | 在 Orbit/DynamicsCore 之后安装 `OrbitalGravityComponent` |
| 原生引力适配 | `orbital_environment.py` | `add_gravity_source(name, GravBodyData, SpicePlanetStateMsg)` 同步注册到参考点和局部环境 |
| 坐标/求力契约 | `orbital_frames.py` | `OrbitalForceProvider` 与无 BSK 依赖的参考点插值 |
| 星历子步取样 | `LinearEphemerisSampler` | 可注入另一个实现 `sample(...)` 的采样器，不改物理 core |
| 物理步进 | `local_mujoco_stepper.py` + 原生 helper | 只读取 provider 的 `(nbody,3)` 力，不识别地球/月球/J2 |
| 非引力作用 | `PhysicsPorts.add_body_wrench()` | 按来源叠加，与引力缓冲区分离，不能互相覆盖 |

provider 契约是 `reset()`、`begin_interval()`、`forces(positions_local, masses, origin_position, nanos)`、`telemetry()`。
输出为当前模型 body 顺序排列的有限 COM 力，单位 N、惯性/局部平行轴表示；world 和零质量项为零。
改变环境模型不需要新增另一个积分器或修改 MuJoCo 的状态布局。

## 3. 坐标、中心天体与时序

局部系 L 的原点是 O，轴始终平行于 N，不是随轨道旋转的 LVLH：

```text
r_i = r_O + x_i
a_local,i = sum_s [g_s(r_O+x_i, t) - g_s(r_O, t)]
F_local,i = m_i * a_local,i
```

`r_O` 是中心天体相对坐标。传给 NBodyGravity 的位置先加上该中心天体在 SPICE 原点下的位置。
第三体对中心天体的间接加速度同时出现在目标与 O 的加速度里，做差后抵消；不得再减第二次。
所有源必须使用同一星历原点/J2000 轴；不能混用月心、地心、太阳系质心消息而不转换。
参考点和局部适配共享相同的 `GravBodyData`/`GravityModel` 配置；不要把推力、阻力等实际卫星效应器直接挂到参考点。

一个外层区间 `[t0,t1]` 的运行顺序：

1. 外层任务更新 SPICE，将参考点推进至 t1。
2. 本地 core 取得上一时刻和当前时刻的 O 位置/速度。
3. provider 快照星历；每个子步起点 tk 用 Hermite 插值获得 O(tk)。
4. 原生 `lms_body_coms()` 从真正的物理模型刷新 COM 运动学，不依赖 MJScene 的发布 stride。
5. 在 tk 采样星历，调用 BSK 模型计算差分引力。
6. 将引力缓冲区与推力/阻力等用户 wrench 相加，调用一次 `mj_step()`。
7. 子步边界采用整数纳秒，步长由相邻边界相减。末端合成全局状态并按原规则发布/重定位。

BSK provider 生效时 C++ 旧潮汐系数必须为零；原生层也拒绝二者同时开启，避免双重施力。

默认星历采样器按消息速度短区间外推位置，按姿态导数外推再投影为正交旋转矩阵。
它不是额外的轨道/星历求解器，也不是无误差插值。默认拒绝距消息 epoch 超过 1 s 的采样；静态测试可显式扩大范围，真实场景应提高发布频率或提供更合适的采样器。
对非点质量模型，缺失/非法天体姿态会报错，而不是偷偷假设单位矩阵。

## 4. 如何扩展

### 添加月球或其他引力源

在初始化仿真前、`OrbitalGravityComponent` 安装后：

```python
from simulation.components.orbital_gravity import BskGravitySourceComponent

reference, factory = ctx.ports.orbital_providers()
moon = factory.createMoon()
# moon_state_msg 必须由已配置的星历模块提供，原点/坐标轴与现有 SPICE 一致。
ctx.install([BskGravitySourceComponent("moon", moon, moon_state_msg)])
```

组件同时注册到 O 和局部环境，并保持对象生命周期。新星历模块由场景按 `Slot.ORBIT` 明确调度；添加一个源不会自动下载星历内核或扩展当前 Earth/Sun 渲染配置。

### 修改地球的引力模型

先安装 Orbit/DynamicsCore，在安装 OrbitalGravity 之前取得 `factory.gravBodies["earth"]`，通过 BSK 的配置 API 设置球谐模型/系数，再安装 OrbitalGravity。
不需要修改 `lms_step()` 或增加一份给 O 专用的 J2 公式。测试包含原生二阶球谐、天体旋转及 BSK Spacecraft 轨迹对照。

### 替换采样方式

```python
ctx.install([OrbitalGravityComponent(ephemeris_sampler=my_sampler)])
```

`my_sampler.sample(payload, epoch_ns, nanos, *, point_mass)` 返回同一坐标约定的 `SpicePlanetStateMsgPayload`。
不要为求过去的子步数据而倒退运行外层 BSK task。

### 其他环境模型

阻力、太阳辐射压、推力、磁力矩继续通过标准力/力矩端口接入，不能混入引力差分缓冲区。
本次没有替换现有默认关闭的 PlateDrag，也没有宣称所有 BSK DynamicEffector 都能直接挂到 MuJoCo；依赖 Spacecraft 连续状态的模型需要专门适配。

## 5. 启动、回归与构建

切换源码后需要匹配本地原生 DLL（仅本项目 helper，不重编 Basilisk）：

```powershell
. .\scripts\python_runtime.ps1
$python = Resolve-SpaceSimPython -RepositoryRoot $PWD.Path
& $python tools/build_native_acceleration.py --component local_mujoco_stepper
& $python -m pytest -q tests/test_orbital_frames.py tests/test_orbital_environment.py tests/test_local_mujoco_stepper.py
```

默认无需额外参数。`run_simulation.ps1` / `start_scene_instance.ps1` 支持 `-OrbitalMode bsk` 或 `-OrbitalMode linear_tidal`；Python 入口及离线 profiler 支持 `--orbital-mode`，也可设置 `SPACE_SIM_ORBITAL_MODE`。
优先级：显式参数 > 环境变量 > `bsk`。`basilisk` 后端保留原路径，不使用这个局部适配器。
旧路径只用于回归，不在新路径异常时自动回退。

日志/observation 的 `orbital_dynamics` 和性能汇总的 `local_dynamics.orbital_environment` 标识实际配置与求值次数。

## 6. 能力边界

- 仅限固定质量、非旋转平移局部系；运行时变质量会报错。
- 引力源/中心天体/模型及系数在安装前配置，运行中变更需重建会话。常见的 mu/模型对象/中心标志修改会检测并拒绝；不要原地修改系数容器。
- 逐 COM 引力不自动包含单个有限尺寸刚体自身的重力梯度力矩；若需加入须单独建模并检查重复计力。
- 点质量奇点、未知坐标约定或缺失星历不是可静默恢复的情况。
- 引力模型不再线性化，不代表消除了 MuJoCo 积分、采样或接触模型误差；仍需任务级精度验证。
- 本次不改变 UE、采集协议、关节控制、轨道参考点积分器或 MJScene 发布机制。
