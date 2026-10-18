# SpaceSim 双星交会与场景交接

本页说明远距阶段的模型、受力和验收门槛。当前工作台默认使用共享GNSS和连续远距→近距→操作流程，见[连续任务说明](../../archive/2026_10_17_连续任务与共享导航.md)。未传`gnss`配置的独立CLI仍保留旧六状态单星滤波及独立噪声，便于历史回归；不能与统一入口默认值混用。近距设计历史见[NEAR_FIELD_RENDEZVOUS](NEAR_FIELD_RENDEZVOUS.md)，逻辑捕获不代表真实连接。

交会和机械臂场景使用同一个统一仓库、模型、动力学核心及UE适配器。两种远距初态分别为约7 km相对距离，以及约300 km升至目标约500 km轨道后接近。远距阶段止于100 m外；连续任务随后接续近距参数化控制和原操作场景。独立API未启用`continuous`时仍可手动交接。真实图像导航和物理抓捕尚未完成。

## 模型与共同动力学

源模型为 `model/SARM/platform/sarm_ground_target_self_collision.xml`。生成的远程飞行配置保留全部 15 个刚体、113 个唯一网格、217 个网格实例、225 个几何体以及质量、惯量和子刚体变换。8 个臂/指关节及目标外侧板关节固定在模型零位，3 个反作用轮自由度保留。SARM 组合质量 169.398546 kg，目标 10 kg；后者沿用团队的临时仿真质量。

最新适配器不向渲染器发送 5 个完全透明的碰撞盒，画面包含 220 个可见几何体（217 个网格实例、3 个飞轮圆柱）；225 个物理几何体仍全部保留。该上游显示过滤不改变碰撞模型或动力学。

`LocalMujocoStepper` 是唯一多体积分器，与标准 SpaceSim 场景相同。Basilisk 负责组件调度、Earth/Sun 引力模型、环境、导航和姿态控制；MJScene 初始化模型并发布消息，不再次积分。默认外层调度 20 Hz，每拍 12 个内部子步，物理积分 240 Hz。报告保存组件调度及 `external_state_writes=0` 诊断；初态之后不回写位置/速度来实现轨迹。

远近距均显式设置`force_every=1`，在每个240Hz内部积分子步重新求值引力。标准遥操作当前使用团队400Hz积分、100Hz IK、200Hz腕力和50Hz采集配置。不能把其引力保持策略直接套到20Hz外层的交会入口，否则会改变实际求值频率；报告与独立验收同时检查交会的`force_stride_steps=1`。

平移通过组件质心处的三轴合力执行器，范数上限 25 N。反作用轮沿用原始轮体惯量、轴、电机、限矩和转速保护；轮速由动力学推进。平移 PD 使用双方 GNSS 滤波结果，目标轨道估计持续更新。姿态保持使用原平台 MRP 反馈，仍采用真值姿态导航假设。

轨道倾角 51.6°，目标初始真近点角 180°，初始 SPICE 历元 `2026 SEPTEMBER 02 00:00:00.000`，与团队场景一致。Earth/Sun 状态同时服务物理与渲染。升轨采用连续有限推力平滑参考；不包含火箭发射、燃耗、喷嘴布置、最小脉宽、J2 或姿态传感器误差。

## 环境和误差配置

`mission_options.json` 保存完整实际配置。网页提供常用参数；API 和 CLI JSON 支持 `FlightOptions` 的全部字段，并拒绝未知键、非有限数或越界值。三个独立随机流分别用于测量、初始状态和未知外力；同一配置和种子可复现。

| 参数 | 默认值和含义 |
|---|---|
| `seed` | 默认7；部分历史浏览器回执使用19，以各次mission_options.json为准 |
| `gnss_position_sigma_m` / `gnss_velocity_sigma_m_s` | 每星每轴 1σ：3 m / 0.5 m/s，独立 2 Hz 观测 |
| `initial_position_sigma_m` / `initial_velocity_sigma_m_s` | 每星初态每轴 1σ：10 m / 0.02 m/s |
| `orbit_acceleration_sigma_m_s2` / `orbit_correlation_s` | 双星独立 Gauss–Markov 未知物理加速度，1σ 为 2×10⁻⁵ m/s²，相关时间 60 s |
| `drag_enabled` / `srp_enabled` | 默认均开启 |
| `density_300km_kg_m3` / `scale_height_m` | 指数大气工程参数：300 km 密度 2.418×10⁻¹¹ kg/m³，标高 60 km |
| `drag_coefficient` / `reflection_coefficient` | Cd=2.2、Cr=1.3 |
| `chaser_area_m2` / `target_area_m2` | 等效面积 1 m² / 4 m² |
| `handoff_distance_m` | 保持距离 110 m，为 100 m 边界留出噪声余量 |

上表GNSS项为旧独立CLI默认值。统一任务默认`GNSSConfig`使用10Hz成对测量：单星每轴σp=1/√3 m、σv=0.002/√3 m/s；近零基线的相对误差下限为每轴σp=0.03 m、σv=0.0015 m/s，相关长度10⁶m，基线增大时共同误差相关性降低。单星3D RMS参考PODRIX，其他相对指标和采样率为工程假设。两星12状态滤波保留交叉协方差，接收机随机流、采样序号和协方差跨段继承；不是原始载波差分或模糊度解算。

10/8会议的旧3m、0.5m/s在这里按每轴1σ解释，不称为接收机规格。密度、面积和系数同样是可配置仿真假设，并非实测硬件数据。

大气组件采用Basilisk ExponentialAtmosphere，阻力复用团队力函数和共转大气相对速度。光压采用球形等效面积模型，1 AU太阳辐照度1361 W/m²，按距离平方反比变化，并由Basilisk Eclipse的地影因子遮挡。未知扰动力真实施加到两个刚体，控制器不读取该随机过程。轨道滤波通过动力学预测和Joseph协方差更新估计轨道，不以真值代替观测。连续任务目前拒绝非零GNSS传输延迟，避免把延迟观测误当作当前时刻更新。

模型公式参考 [Basilisk 指数大气](https://hanspeterschaub.info/basilisk/_downloads/4628ece49f0fa96a05b63562dd03bf90/Basilisk-atmosphere-20190221.pdf)、[Basilisk 地影](https://www.hanspeterschaub.info/basilisk/Documentation/simulation/environment/eclipse/eclipse.html)、[Basilisk 光压](https://hanspeterschaub.info/basilisk/_downloads/86e242b81759c9aa6b3a9f4475ed82ff/Basilisk-RadiationPressure-20170712.pdf)和 [NASA 太阳辐照度](https://earth.gsfc.nasa.gov/climate/projects/solar-irradiance/science)。

## 工程验证标准

本轮在完整运行前固定以下标准，替代历史小噪声场景的 100.5 m 设定；不是团队签署的实物指标。

- 2 Hz 原始记录中的实际组件质心间距始终 ≥100 m。
- 最后连续保持 120 s，距离在配置设定值 ±5 m 内，Hill 系相对速度 <0.2 m/s。
- 初始捕获 600 s 后跟踪误差 <5 m；全程姿态误差 <0.5°，推力、轮速、电机力矩满足原模型限值，状态有限且无导航中止。
- 升轨初态使用约 300 km 轨道加记录的随机误差；最终高度与实际传播的目标轨道相差 <10 m。目标受外力传播，不再假定永远处于精确 500000 m。
- 两星 GNSS 独立，采样噪声统计符合配置，滤波降低位置噪声；默认阻力、光压、随机扰动力均有实际遥测。
- 使用团队 local 核心和 BSK 引力，运行中外部状态覆盖次数为零，源码和模型指纹匹配。

`tools/verify_team_flight.py` 从 `team_telemetry.npz` 独立复算指标，不只读取生产报告布尔值。记录还包含环境分力、密度、地影、推力、导航、轮速/力矩及姿态。2 Hz 采样判据不等于对采样间极小距离的数学证明。短时诊断未完成任务会返回失败判据，不会生成可接受交接。

## 运行与网页流程

统一仓库安装方法见[根 README](../../README.md)。本次云端使用 Python 3.13、bsk 2.11.1、MuJoCo 3.7.0，匹配团队内置原生头文件。其他版本必须检查 ABI；仅成功 import 不足以证明兼容。以下命令在 `space_sim_server` 目录、使用已配置解释器执行：

```powershell
python -m simulation.orbital_rendezvous.run --scenario in-orbit --output-dir run/team_flight
python -m simulation.orbital_rendezvous.run --scenario orbit-raise --output-dir run/team_raise
# --config 接受 FlightOptions JSON；省略字段使用公开默认值
python -m simulation.orbital_rendezvous.run --config mission-options.json --output-dir run/configured_flight

# 加入团队资产绑定和渲染记录；--live 向运行中的 UE 发送状态
python -m simulation.orbital_rendezvous.run --output-dir run/team_live --catalog ../space_sim_UE_Adapter/Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_platform.catalog.json --live
python tools/verify_team_flight.py run/team_live
python tools/verify_team_recording.py run/team_live/team_rendezvous.bskrec
python tools/verify_team_handoff.py run/team_live/handoff.json

# 步长与导航失效诊断；导航失效结果应拒绝交接
python -m simulation.orbital_rendezvous.run --step 0.025 --output-dir run/team_half_step
python -m simulation.orbital_rendezvous.run --navigation-loss 100 --output-dir run/team_nav_loss
```

登录原控制台，在“任务与场景”选择交会模式：配置误差、启动、在原主视窗查看实时 UE 画面与右侧导航遥测、停止或等待完成、下载报告/原始遥测/配置/交接文件。完成后点击“接续遥操作”，同一页面切回团队的控制和采集面板。旧 `/static/rendezvous.html` 自动重定向至控制台。平台自动启动所属工作区的 UE；与标准机械臂场景互斥。后台使用 `SPACE_SIM_RENDEZVOUS_PYTHON` 指定原生解释器，`SPACE_SIM_TEAM_CATALOG` 可覆盖资产目录。继续运行下一场交会时复用渲染器；切换到机械臂场景时释放所属渲染器，界面允许短暂重连。任务编号关联交会输出、源 run_id、场景实例与 LeRobot 采集；刷新只恢复状态，不自动启动或接续。

只有已通过完整判据且源模型匹配的结果可点击“交接到机械臂场景”。管理员 API 为 `POST /api/rendezvous/handoff`，在启动锁内完成场景切换。交接后使用主页面的原有控制、采集、重置与停止功能。当前任务只验收交接、控制响应和双相机录制，不声明完成抓取。

## 连续性交接

`handoff.json` 保存双星原点 J2000 位置/速度、wxyz 姿态、本体角速度、原始可动关节零位、飞轮角度和转速、双方组件质心的 GNSS 估计与协方差、仿真时刻、环境参数及随机过程状态。原点状态不能当作组件质心状态使用。

标准场景仅在初始化后、首步积分前调用 `apply_handoff()`；推进 SPICE 历元至交接时刻，恢复轨道原点、轮动量及环境状态。初始状态读回误差进入标准观测，来源运行 ID 和导航快照同时保留。恢复后的标准遥操作场景仍使用其原有姿态/机械臂控制，导航快照为后续近程导航接入提供初值。独立恢复工具还核对姿态、关节和轮速。

UE 使用完整团队网格、材质与 Earth/Sun。主视口默认“双星交会总览”，只移动相机来容纳两星，模型和双星坐标保持真实尺度。蓝色 Chaser、黄色 Target 方框是固定屏幕大小的定位标记，不代表卫星尺寸；公里距离下真实卫星可能小于一个像素。距离为模型组合质心间距，与导航遥测的定义一致，左下比例尺随总览取景更新。

原播放器工具栏可切换“追踪星近景”“目标星近景”查看模型细节，默认近景相机距离为 2.8 m；近景画面不能用于估算另一颗星的距离，画面明确提示视野外目标。C 进入自由相机，Home 返回当前观察视角。主视口相机由同一 UE 实例共享，新标签页读取实际相机模式，不重置他人的视角；机械臂安装相机仍通过原相机选择器访问。

选择“固定尺度总览”会保持切入时的总览视距，继续跟随双星中点和连线方向；两星随实际接近在画面上靠拢，比例尺不随距离自动缩放。接近后想看清两颗星，可切回自动取景重新构图。窗口尺寸或视频分辨率变化仍会改变像素与米的对应关系，以画内比例尺为准。

光照保持团队太阳和地影模型。双星进入完整地影时 HUD 显示 ECLIPSE，定位标记及距离仍可读；没有为展示修改物理光压或权威采集图片。视频连接、物理黑暗和视野外目标应分别判断。

预览默认选择部署允许范围内不超过 60 FPS 的最高档位，仍可手动选择 90/120 FPS（取决于部署上限），重连保留本页面的选择。视频帧率不同于物理状态发布频率：UE 使用连续播放时钟和有界状态缓冲，按观测到的仿真倍速进行插值和有限外推；长暂停后重建播放基准。该处理仅作用于预览，不改变动力学、原始遥测或 30 Hz 权威采集。低频状态之间的平滑显示不能替代更高频的物理观测。

新 manifest 清除上个场景已不存在的对象、天体和相机，防止旧通用模型残留。长路径 checkout 的 DDC 使用短的每工程缓存目录，避免导入阶段越过 UE 长度限制。`verify_team_recording.py` 逐帧用刚体姿态与模型质心偏置重建双星相对位置，和原始动力学状态比较，不能只凭网格数量判定渲染接入正确。

## 与最新团队模型共同维护

本分支合入团队 main `662d829`（2026-10-12 SARM MJCF / PR #17），保留其四连杆夹爪、任务盒与自由操作件、螺纹 SDF、rad 单位语义、外覆尺度修复、空节点和腕部相机修复。直接遥操作的原场景选择器保留所有团队模板；新 `sarm-task-box-20261012` 仍按团队定义标为实验配置，说明见 [SARM 新模型](SARM_20261012_PLATFORM.md)。

远程交会继续使用团队默认`sarm-ground-validation-self-collision-grasp`；交接显式选择同一模板并校验源模型SHA256，不能因为先前选择过实验模板而换掉飞行模型。新版自由插头、抽屉与螺纹机构没有被强行冻结后冒充整场交会验收。其控制、重置及采集与交会共用平台，模型身份和数据单位保留各自来源。100m内现有参数化控制流程不等于真实图像自主导航、物理抓捕或自动对接成功；新机构的完整任务能力也需另行验收。

个人任务对应及复现入口见 [AOC_SEVEN_TASKS.md](AOC_SEVEN_TASKS.md)（保留历史文件名）。源代码与适配器配套变更在同一分支交付；运行数据、认证、讲稿和生成 UE 资产不提交 Git。
