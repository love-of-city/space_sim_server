# 轨道物理配置：J2 与逐物体大气阻力

适用版本：2026-10-16，默认 `local` + `bsk` 轨道环境路径。

## 生效方式与兼容性

新建场景默认使用地球点质量 + J2、太阳点质量。J2 是轨道引力摄动，不是关节限制或位置夹断；MuJoCo 继续唯一推进实际刚体、关节和接触状态。

新场景将配置保存到 `environment.physics`。旧场景缺少该字段时，明确恢复为地球/太阳点质量，保证旧数据可重放。已有运行进程不会热更新；重启后端后创建新场景才能采用新默认值。已有实例仍遵守自己的保存配置。

配置优先级：Python 入口 `--orbital-config JSON` > 场景 `environment.physics` > 无实例启动时的默认值。旧 `basilisk` / `linear_tidal` 路径必须显式使用 `point_mass` 且关闭配置化阻力；不支持的组合报错，不静默降级。

创建场景 API `POST /api/scenes/start` 新增 `orbital_physics` 请求字段。例如旧物理基准请求体：

```json
{"seed":123,"orbital_physics":{"earth_gravity":"point_mass","drag":null}}
```

普通新建场景不传此字段，后端会保存默认 J2。浏览器不新增开关，使用保存实例、API 或 Python 参数选择环境；不更改控制、UE 或数据写入协议。

提供三个 JSON 文件：

- `config/orbital/j2.json`：J2 + 太阳，阻力关闭。
- `config/orbital/point_mass.json`：点质量回归。
- `config/orbital/drag_synthetic_example.json`：J2 + 两物体阻力的**合成参数示例**，仅用于敏感性分析/接口验收，不是硬件标定参数。面积分别只分配到 bus 与 target bus；面板面积未计入。

`--atmospheric-drag` 现在仅用于要求阻力已被明确配置；缺少密度和逐物体参数时立即报错。旧入口只构造默认组件，但没有设置 BSK 密度和地球半径；在本地 BSK 2.11.1 中默认 baseDensity=0、planetRadius=0，不能视为已有有效的 500 km 大气模型。

## 地球 J2 与共享引力场

BSK `SphericalHarmonicsGravityModel` 设置 `maxDeg=2`：

```text
C00 = 1
C20_normalized = -J2 / sqrt(5)
其余非中心项为 0
```

默认 J2=0.001082616，来源为 Basilisk 2.11.1 `architecture.astroConstants.J2_EARTH`。该数值被显式保存在配置中，不随之后升级 BSK 自动变化；它是该版本的常数近似，不声称为最新 EGM 系数。地球 μ 与参考半径沿用 BSK `createEarth()`，当前为 3.98600436e14 m³/s² 与 6378136.6 m，实际值写入遥测。需要替换 J2 时必须同时填写真实的 `coefficient_source`。

SPICE 提供 Earth-fixed `IAU_EARTH` 姿态。BSK 在地固系求球谐场并转换回 J2000；同一 `GravBodyData.gravityModel` 同时用于轨道参考点 O 和各物体质心场求值：

```text
F_gravity,i = m_i Σ_sources [g_source(r_O+x_i,t) - g_source(r_O,t)]
sources = Earth(point mass + J2), Sun(point mass)
```

点质量已包含在球谐场中，不能再额外加一份。局部轴平行 J2000，仅随 O 平移，所以不添加 LVLH 的科氏/离心项。太阳原有引力保留；不增加月球。

当前仍为固定质量、逐质心引力。没有新增有限刚体自身的重力梯度姿态力矩；各刚体质心之间的差分引力继续自然形成多体作用。

## 大气密度、相对气流与差分阻力

配置中的指数密度采用轨道高度处的锚点：

```text
rho(h) = rho_ref * exp(-(h-h_ref)/H)
BSK.baseDensity = rho_ref * exp(h_ref/H)
BSK.planetRadius = Earth.radEquator
```

这里 h 为 BSK 指数大气模型的球形径向高度，不是椭球大地高。没有空间天气、风场或 MSIS。密度曲线是用户显式给出的近似，只能用于其参数适用范围。

`reference_density_kg_m3`、`scale_height_m`、每个 body 的 `area_m2` 和 `drag_coefficient` 必须为正有限数；参考高度非负；禁止重复 body、未知配置字段、空参数来源和指数上溢。`density_source`、`parameter_source` 记录参数来源/假设，不等于软件自动完成了物理标定。

配置化阻力使用 SPICE 的地固旋转矩阵 C 和导数 Cdot 计算共转大气速度：

```text
r_P = C_PN (r_N - r_Earth_N)
v_atmosphere_N = v_Earth_N - C_PN.T Cdot_PN (r_N - r_Earth_N)
v_relative_N = v_body_COM_N - v_atmosphere_N
F_drag_N = -0.5 * rho * Cd * A * |v_relative_N| * v_relative_N
```

这保留了地球实际旋转轴方向；不把固定 J2000 z 轴当作地球当日转轴。输入必须有有限星历和有效正交姿态。

每个已配置刚体分别创建 BSK 指数大气取样器与外力发布器。力通过 `PhysicsPorts.add_body_wrench` 作用在该物体质心，以惯性轴表示。配置化入口压心偏置固定为零，不新增显式气动力矩；多体中不同作用点的力仍可能自然影响系统姿态。质量继续由 MJCF 所有，配置不重复输入质量，不修改惯量。

O 仅用于引力参考传播，不能把气动力挂到 O。若给多个刚性连接 body 分配面积，应按实际暴露面积拆分，不可给每个 body 重复分配整星迎风面积。自由目标也须单独设置；不要复制参考项目的 ISS/Dragon 数值。

本仓库尚无本体/目标已标定的迎风面积、Cd 和密度数据，因此生产默认 `drag=null`。合成示例证明接入正确，但不代表实际阻力的大小已经可靠。

## 时序与输出

- 物理仍为 240 Hz，MuJoCo 原有子步设置不变。
- 引力当前每 8 步求值并保持，约 30 Hz；`SPACE_SIM_ORBITAL_FORCE_STRIDE=1` 用于逐子步对照。当前真实频率仍在 `orbital_dynamics.force_stride_steps` 中。
- 阻力 10 Hz，采用已对齐刚体消息发布时间的分频调度，步间保持外力。
- `orbital_physics_configuration` 启动日志与 observation 的 `orbital_physics` 保存实际配置、J2 来源、μ、半径、历元和坐标系。
- observation 的 `atmospheric_drag` 保存每物体的密度、相对气流、惯性力、力矩和采样时间。字段表示力模型采样时刻，不能当作每物理步更新。
- `orbital_dynamics` 保留已有场模型、天体来源及求力周期遥测。

## 验证与命令

从服务端目录，使用已经包含 BSK 的环境运行：

```powershell
..\.venv\Scripts\python.exe -m pytest -q tests/test_orbital_physics_config.py tests/test_orbital_physics_acceptance.py tests/test_orbital_environment.py tests/test_atmospheric_drag.py
..\.venv\Scripts\python.exe tools/verify_orbital_physics.py --output ..\reports\orbit-acceptance.json
```

验收工具使用原生 BSK 做两圈 500 km、51.6° 轨道传播，验证点质量的能量/角动量、J2 总势能对应的能量与极轴角动量，以及 J2 升交点进动。为了对照解析结果，它使用固定地球转轴并关闭太阳；这与生产使用 SPICE/太阳的场景不同。

J2 中两体能量及角动量模长本来就不恒定；不能用它们判断 J2 积分是否守恒。工具分别输出这些诊断量和真正适用的守恒量。

完整 SARM 动力学可用已有离线 profiler 验证，包含真实 SPICE、MuJoCo 模型、控制、场景消息装配，但不连接 UE、后台或采集器：

```powershell
..\.venv\Scripts\python.exe tools/profile_simulation_runtime.py --adapter-root ..\space_sim_UE_Adapter --scene-instance <已有场景JSON> --orbital-config config/orbital/j2.json --duration 10 --motion hold --output <新的结果JSON>
```

输出名必须全新。换成 `drag_synthetic_example.json` 可检验两物体气动力。它不修改已有场景文件或运行进程。测试用独立两自由体也验证了气动力随面积变化、实际减速度随质量变化，以及外力来源可以累加。

## 来源与范围

参考 OWM 提交 `78696accd8eebdc0319e616dcc1e72308cd3bb48` 的 `iss_numerical` 摄动组织、逐物体弹道参数及轨道验收思路。生产引力调用 BSK 原生模型，没有复制 OWM 的 JAX/RK4 推进代码，也没有新增 JAX/astrojax 依赖。用户最初链接的 `lerobot_writer.py` 未修改或引入。

月球、J4/J6、太阳光压、传感器、飞轮卸载、燃耗、电热和柔性仍属于后续范围。独立 BSK 轨道验收不代表 MuJoCo 多体系统达到同样的长时精度，实际误差仍受局部积分、外力保持、接触和控制影响。
