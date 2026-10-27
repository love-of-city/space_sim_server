# 平台自动采集与数据集回放

2026-10-16。自动采集与数据查看共用现有 FastAPI 后端、用户会话和正式数据目录，不再需要另起 `data_display/server.py`。

## 使用

1. 按原方式启动平台并登录，从左侧导航选择“仿真工作台”“自动采集”或“数据集”。管理员还可进入“用户管理”。导航在同一标签页内切换，窄屏显示顶部导航。
2. 在 `/collection` 设置成功条数、同时运行数（1–4）、最多尝试数、种子和关节初始化角偏移（半范围 0–50°，默认 10°）。默认两路。
3. 点击开始，后台为各路启动独立的生产仿真/UE 采集进程。当前批次显示阶段、已记录帧数、每路尝试及验收结果；已结束批次归入可展开的历史列表。切页或关闭网页不终止采集。
4. 点击成功记录或进入 `/datasets`，按来源、结果、任务、日期和文本筛选记录；进入详情查看两路同步视频，以及关节与夹爪、六维力、末端状态、任务状态四组曲线。支持逐帧、播放倍速、图表缩放和 CSV 导出；返回列表保留筛选。`?episode=<id>` 可直接定位记录，`?batch=<id>` 可限定批次。
5. “停止该批次”先请求录制正常收尾。超过 180 秒则强制清理该批次进程，已验收记录保留，未完成记录不计入成功数量。

任务菜单提供六个独立任务：抽屉完整循环，航空插头、导向插头、圆柱插头各自的拿取与插入，航空螺丝旋出与装配，以及杆状机构放倒与复位。每条 episode 只包含所选任务，不串行操作多个插头。抽屉流程为接近、抓取、抽出、放回、松爪、撤离和最终闭合检查。插头流程为跟踪接近、抓取、提起、搬运到前方对应 40.5 mm 装配孔、对准插入、松爪撤离和就位检查。航空螺丝与杆状机构见[螺纹任务采集](THREADED_TASK_COLLECTION.md)；`drawer_lift` 为诊断任务，不在网页菜单中。默认关闭运动/观测/六维力噪声和环境随机化；新建采集批次时可分别启用并配置参数。初始化默认使用项目操作姿态，加六轴独立 ±10° 均匀偏移，支持独立关闭。偏移半范围支持 0–50°；本批设置为 30°，即六个基准关节角各自独立加上 `Uniform(-30°, 30°)`。每次先检查模型关节限位、初始碰撞及实际接近路径；不通过时拒绝该初态并重采样，不把角度钳位到限位。配置语义见下方“可配置噪声与随机化”。

## 网页固定布局与颜色指令

颜色指令要求模型中的七个目标孔具有独立可着色表面。原始整块台板模型不能实现该映射；未配置时，网页会提示不可用，API 在创建采集进程前明确拒绝颜色任务，不生成与画面不符的颜色指令。

服务器可通过两个环境变量配置固定采集布局，二者必须成对提供。路径属于本机运行配置，不写入通用源码：

```bash
export SPACE_SIM_COLLECTION_FIXED_MODEL=/absolute/path/to/model/layout.xml
export SPACE_SIM_COLLECTION_FIXED_CATALOG=/absolute/path/to/ue-assets/layout.catalog.json
./start_deployment.sh --mode local
```

也可将两行 `export` 保存在本机 `.local/collection-fixed-layout.env`，先执行 `source .local/collection-fixed-layout.env`，再按原部署方式启动。Linux 启动链 `start_deployment.sh → scripts/linux_platform.sh → linux_deployment.environment → backend` 保留这两个环境变量；直接启动后端同样支持。现有 `linux.local.json` 不接受未声明的字段，不应把环境变量名添加为 JSON 键。已经运行的后端需按原部署流程重启才能读取新配置。

未勾选布局随机化时，配置的固定 model/catalog 用于本批次全部记录，不随 seed 或 worker 改变。勾选后仍只使用 `SPACE_SIM_COLLECTION_LAYOUT_POOL` 指定的有效池；固定配置不会替代失败的随机池。固定输入须通过布局 manifest、底板/孔位资产和七孔着色绑定检查；批次保存 `fixed-layout.json` 及模型、catalog、manifest、姿态控制配置的 SHA，子进程启动前再次检查，并显式传入 `--model --catalog`。保存场景已有的布局身份若与配置不符，会明确拒绝。

`/api/collections/catalog` 的 `fixed_layout` 表示固定输入的可用性，各任务的 `requires_color_layout` 表示是否需要它。未配置固定布局时，原抽屉等非颜色任务仍可使用原模型。此配置只衔接网页模型输入，不改变网页现有并发数、录制设置或进程复用行为。几何与材质校验不等同于所有任务已成功，采集仍执行每条原生姿态/路径预检和完整数据验收。

录制频率沿用 400 Hz 动力学、100 Hz 在线 IK、50 Hz 状态/动作/图像与独立 200 Hz 六维力，两个相机均为 1024×1024。数据使用 LeRobot v3、七维 state/action（六轴+夹爪0–1），默认 compact 保存及独立力数据 sidecar，不替换采集器、不重采样。

脚本任务以 200 Hz 更新关节参考，绕过在线遥操作 IK；100 Hz 是在线 IK 的默认配置，不是脚本策略频率。场景保存的 J2/点质量引力及显式大气阻力配置也应用到脚本动力学。

脚本录制入口每次启动读取 `config/recording.json`，使用其中的原始照片开关、视频编码设置和帧数上限。自动任务要求 50 Hz、总览与腕部双相机 RGB；不兼容的配置会在启动前报错，不会静默覆盖。显式启动自动采集会开启该独立场景的权威采集，不受手动场景默认开关影响。默认 `max_frames=18000` 对应 360 秒仿真时间，达到上限会报错，较长任务须提前提高此值。

`save_raw_camera_images=false` 时不要求 `cameras/` 文件；验收仍检查双相机清单与状态的帧号、会话、时间戳配对，并完整解码两路 LeRobot 视频、检查帧数和时间间隔。旧记录和开启照片保存的记录仍校验原始图片。非流式编码可能产生 LeRobot 自身的临时图片，不等同于外层 `cameras/` 审计副本。

默认 `storage_profile="compact"`：LeRobot 主表保留数值特征；相机、任务和在轨上下文改存压缩 Parquet sidecar，200 Hz 力独立保存。默认不写重复的 `steps.jsonl`、`captures.jsonl` 和 `observation.platform_json`。`save_dynamics_trace=false` 关闭自动录制的全量动力学调试 trace；阶段事件和摘要保留。查看器和验收兼容旧诊断格式，详见 [精简保存格式](COMPACT_RECORDING.md)。独立 `run_scripted_task.py` 的诊断入口仍默认保存 trace，可用 `--no-save-trace` 关闭。

从服务端子目录运行脚本（解释器按本机环境调整；输出目录必须是新目录）：

```powershell
& ..\.venv\Scripts\python.exe tools\collect_drawer_cycles.py `
  --scene run\scenes\<scene-id>.json `
  --output run\drawer-batch-001 --count 1 --max-attempts 3 --seed 20261016
```

`collect_drawer_cycles.py` 负责随机初始化、碰撞预检查、重试和完整验收；`record_drawer_task.py` 录制单次任务；`run_scripted_task.py` 单独运行时只输出动力学诊断，没有训练视频。网页批量任务支持抽屉完整循环、三种独立插头任务、航空螺丝和杆状机构，后两者经 `collect_threaded_tasks.py` 进入共用采集器；详见[螺纹任务采集](THREADED_TASK_COLLECTION.md)。`drawer_lift` 仅作为独立诊断/轨迹校验任务。

流式编码、原生几何计算、配置覆盖和测速口径见 [单机采集性能](COLLECTION_PERFORMANCE.md)。单条摘要记录启动、录制、收尾和总时间，不能将正式录制倍率当成包含启动与验收的全流程倍率。

## 抽屉分段速度（2026-10-18）

抽屉完整循环默认使用 `drawer-cycle/5`。自动采集页面增加“移动速度（m/s）”，默认 0.1；该项只对抽屉任务启用。后端参数为 `linear_speed_m_s`，CLI 使用 `--linear-speed`，取值必须大于 0 且不超过 0.1。页面步长为 0.005。其他任务维持原有速度。

| 阶段 | 默认运动目标 |
|---|---|
| 连续曲线接近、撤离等空程 | 0.1 m/s |
| 曲线接近的末端直线段 | 主体 0.1 m/s；最后 2 cm 降到约 0.025 m/s |
| 抓稳后的抽出 | 主体 0.1 m/s |
| 放回抽屉 | 主体 0.1 m/s；最后 2 cm 降到约 0.012 m/s |
| 最后归位、卡扣扣合 | 速度上限 0.012 m/s，与放回轨迹连续连接，实际稳定后确认 0.2 秒 |
| 夹爪朝向旋转 | 0.2 rad/s（约 11.46°/s），与接近、张爪同步执行，进入最后 6 cm 前完成转向 |
| 夹爪开合 | 最高 0.2 rad/s，平滑起停；张爪与接近/撤离重叠，抓取按真实接触确认 |

这些是轨迹目标/上限，实际速度还受关节限速、起停加减速、伺服跟踪和接触影响。放回与最后 1 mm 预压属于同一段轨迹，不再单独安排一秒归位动作。用户选择更低的主体速度时，接近和归位阶段也不会超过该主体速度。

只对抽屉完整循环把脚本关节参考速度上限从 0.08 提到 0.35 rad/s（约 20.05°/s）；这是本次仿真任务的速度覆盖，不代表实体机械臂允许该速度，也没有修改 MJCF、URDF 或其他任务的限速。较低的旧关节限速会限制末端能否达到 0.1 m/s。

实现仍在各阶段开始时求 IK 路径；`joint_path.py` 用 not-a-knot 三次样条通过这些点，使关节路径的位置、一阶和二阶导数连续，并检查点间插值是否越过关节限位。`motion_timing.py` 使用五次速度过渡，平滑连接起步、巡航、末段降速和停止。由 200 Hz 脚本时钟产生关节参考，在线遥操作 IK 仍为 100 Hz。

平移参考加速度/跃度配置为 0.2 m/s²、0.8 m/s³；转动为 0.4 rad/s²、1.6 rad/s³；机械臂各轴为 0.7 rad/s²、8 rad/s³。通过样条导数的解析界和整体时间缩放计入路径曲率，而不是只限制路径进度。夹爪参考采用同样的平滑计时，加速度/跃度为 0.4 rad/s²、1.6 rad/s³。这些是正常连续执行时的参考限制；接触引起的实际运动和保护停机不受同一个平滑上界保证。

夹爪参数独立于末端平移：`grip_speed_rad_s` 为闭合主速度，`grip_contact_range_rad` / `grip_contact_speed_rad_s` 支持闭合末段的可选限速。默认主速度、末段上限均为 0.2 rad/s，仅采用平滑起停；较低闭合速度在本次对照中未稳定降低接触尖峰，因此不作为默认。力和角速度记录保持原始值，没有为使曲线好看而滤波。

原有跟踪与接触保护继续生效：关节跟踪误差超过 0.06 rad 暂停轨迹推进；腕部合力超过 80 N 或合力矩超过 8 Nm 持续 0.1 秒则失败；仍须双指接触、实际抽出、回位、松爪、撤离并连续稳定确认 2 秒。没有直接改写物体状态或新增焊接约束。

速度配置保存在批次 `job.json`、任务摘要和每条观测的 `scripted_task.task_parameters` 中；任务版本为 `drawer-cycle/5`。加速度、跃度、夹爪参数也一并保存，旧数据保持原参数。支持从以下任一入口传入，例如：

```powershell
python tools/collect_drawer_cycles.py --scene <scene.json> --output <new-output-directory> --count 1 --max-attempts 3 --linear-speed 0.1
```

首次提速对照见根 `archive/2026_10_18_抽屉分段提速与采集验证.md`；平滑改进和接触限制见 `archive/2026_10_18_抽屉速度曲线平滑与接触验证.md`。更新后构建前端并重启后端，刷新网页即可使用新参数；无需重新编译 UE。

## 上一版并行动作与等待（drawer-cycle/4，2026-10-19）

以下记录上一版行为；当前默认版本的连续动作见后文。

安全抬高后，`above_drawer` 在同一条平滑位姿路径上同时平移和旋转，不再安排独立的 `align` 停靠。初始化碰撞预检也检查这条实际执行的组合路径；低位时仍先抬高再旋转。夹爪开合和带载抽出/归位保持原接触顺序。

开始时仍根据真实关节速度和有效测力读数确认稳定，默认窗口由 1 秒缩短为 0.25 秒；已经高于安全高度时不再给零位移的抬高动作分配 1 秒轨迹。空程到位确认默认 0.1 秒。抓稳保持 1 秒、抽出保持 2 秒、带载归位确认 2 秒仍保留。

松爪撤离期间持续检查抽屉位置、速度、卡扣和双指无接触。该段可以计入原有 2 秒连续关闭验证，任一条件失效即重新计时。抵达撤离终点后仍确认机械臂稳定至少 0.2 秒；若关闭窗口不足 2 秒则继续等满，不能提前报成功。观测新增 `released_closed_seconds` 便于追溯。

录制仍走原始同步采集链路，不裁视频、不删状态/力样本；渲染器预热本来就在录制前，视频封装也不属于任务动作时长。任务阶段元数据升级为 `drawer-cycle/4`，旧版录制仍按旧阶段序列验收，采样频率和数据维度不变。

## 当前连续抓取流程（drawer-cycle/5，2026-10-19）

当前阶段为 settle → approach → grip → lift → return → release → retract → verify。接近阶段直接从当前位姿沿连续曲线到抓取位姿，不经过正上方停靠点；平移、朝向旋转与张爪使用同一个轨迹进度。到把手上方 6 cm 时已经完成转向和张爪，随后继续沿抓取轴接近，最后 2 cm 低速进入，曲线连接处没有中间停止。初态高度不够或组合路径碰撞时明确拒绝该初态，交给批次重采样。

approach_path.py 提供带平滑收尾的空间曲线；motion_timing.py 除关节限制外，还计入弯曲路径的平移、转动和夹爪开度导数。三者共享限速后的时钟，跟踪保护暂停接近时，张爪也一起暂停。原速度、加速度、跃度和载荷上限保持。

原生预检覆盖完整接近曲线和实际张爪开度。gripper_preflight.py 依据原 MJCF 的连接点验证平行四连杆关系，并仅为预检的私有模型补齐被动关节；不能把主动杆打开后仍将被动手指留在闭合姿态。不同闭环几何会明确拒绝，实际动力学仍由原 MuJoCo 约束求解，没有修改物理状态或碰撞几何。

抓取仍先到位再闭合，双指稳定接触确认缩短为 0.15 秒。提起必须真实达到目标位移、保持双指接触并停稳，之后直接返回，不再独立 hold 两秒。放回与最后 1 mm 预压在同一路径内完成，归位实测条件稳定 0.2 秒后松爪。release 阶段一旦实际开度达到 0.6、双指无接触且卡扣关闭，就可以撤离；尚未完成的张爪轨迹原样继续，不重新起步。最终仍要求夹爪充分打开、末端撤离到位及抽屉连续关闭至少两秒。

继承的 hold_s 仅用于旧阶段兼容，v5 不再有独立 hold；grip_confirm_s、seat_confirm_s、release_clearance_opening 为当前有效过渡参数。新旧版本的录制都按各自阶段序列验收，50 Hz 状态/动作/双相机、200 Hz 六维力和 6+1 格式保持不变。

## 结构与扩展

- `backend/space_arm_platform/collection_jobs.py`：持久任务管理、并发预算、随机种子/端口隔离、进度与生命周期；本层不加载 MuJoCo/Basilisk。
- `collection_processes.py`：数据目录独占协调器锁、Windows Job Object 进程树所有权。后端异常退出时 Windows 回收采集子进程；重启将未完成任务标记为 interrupted，不擅自恢复。
- `collection_api.py`：鉴权 API、独立场景配置创建与数据/视频读取。创建自动采集场景不会覆盖遥操作管理器的当前实例。
- `dataset_view.py`：纯数据读取与时间轴构建，保留独立传感器时间轴，缓存签名包含力数据文件。
- `frontend/platform_shell.js`、`platform.css`、`ui.js`：统一导航、路由、公共样式和表单。`frontend/pages/` 中的 collection、datasets、users 分别提供页面模板与 activate/deactivate/reset 生命周期，保持原生 ES modules，不引入新框架。后端四个页面入口返回同一 Vite 构建，替代原 public 下的独立 HTML。
- `tools/collect_scripted_tasks.py`（共享批次与验收）、`collect_drawer_cycles.py` / `collect_plug_tasks.py`（兼容入口）、`record_drawer_task.py`：复用已验证的真实 BSK/MuJoCo → UE 相机 → CaptureReceiver/SimulationHub → EpisodeRecorder → LeRobot 链路；增加协作停止与运行路径透传。

新增任务在 `TASKS` 注册任务名、显示名称、场景模板和独立采集 CLI。适配 CLI 接受管理器传入的 count/max-attempts/seed/range-deg、场景/输出/数据目录、四个端口、UE/adapter 路径和 cancel-file；原子写入 summary.json，报告 attempts、accepted_episodes 和 status。各任务自行实现成功条件与数据校验，不能只依据进程退出码或录制时长判成功。插头的几何参数集中在 `simulation/scripted_tasks/plug_profiles.py`，控制器共用 `plug_insert.py`。新增插头时显式配置物体、取件位、装配孔、插入长度与键槽要求，并独立注册任务；不修改底层动力学和录制链路。

离开工作台时先停止遥操作输入，再释放该页的控制连接及视频资源；返回后重新连接。离开数据详情会暂停视频。退出登录会清空当前页面数据和表单，避免切换账号后出现上一账号的内容。后台采集任务的生命周期由服务端管理。

## 用户管理与升级

`/users` 仅向管理员开放。支持搜索、角色/状态筛选、新增账号、修改显示名称和角色、启用/停用、重置其他用户密码；显示最近登录时间。角色沿用 `admin` 与 `operator`。当前管理员不能停用自己或取消自身管理员权限；自己的密码通过侧栏“修改密码”更新。

停用、角色变更或重置密码会撤销相应登录会话；已有 WebSocket 会在会话校验时断开。停用保留采集数据，后台批次需到自动采集页另行停止。旧操作员接口继续兼容。

认证库启动时通过增量迁移添加 `display_name` 和 `last_login_at_ns`，保留已有身份、密码哈希及归属。历史账号初始显示名称沿用用户名，首次新版本登录后开始记录最近登录时间。更新后需构建前端、重启后端，再刷新网页：在 `space_sim_server` 目录执行 `npm --prefix frontend run build`。此界面更新无需重新编译 UE。

## 持久化、权限与限制

- 数据：配置的 `data_root`，默认 `data/episodes/`。自动与手动记录使用同一目录及查看接口。
- 批次：`data_root` 同级 `collection_jobs/<id>/`，含 job.json、场景快照、分路报告与日志。运行数据已加入 Git 忽略。
- 操作员仅可查看/停止本人批次，读取本人记录与视频。管理员可查看所有记录；没有用户归属的历史记录只向管理员展示。
- 每个数据目录只运行一个管理服务；同一时刻接受一个批次，批次内 1–4 路并行。此版本不是跨机器队列，也不自动断点续跑。
- 成功条数和尝试上限分配给各路；各路拥有独立种子区间和尝试预算。某一路耗尽预算会令批次失败，不会借用其他路未使用的预算；已经成功的数据保留。
- CPU/UE 并发沿用已验证实现，没有切换到实验 GPU 动力学。4 路长时间完整任务的吞吐与资源上限仍需专项测量。
- 可以保留遥操作实例，但它与自动实例共享本机算力。页面不承诺实时倍率。
- 只读取原数据根下的 mp4，视频响应支持 Range；API、数据和视频请求均复用登录鉴权。

## 接口

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | /api/collections/catalog | 任务注册表与采样配置 |
| POST / GET | /api/collections | 创建 / 列举本人可见批次 |
| GET | /api/collections/{id} | 进度、分路尝试与成功记录 |
| POST | /api/collections/{id}/stop | 协作停止 |
| GET | /api/datasets | 列举本人可见历史/新记录 |
| GET | /api/datasets/{id} | 原生时间轴与曲线数据 |
| GET | /api/datasets/{id}/media/{path} | 鉴权视频读取 |
| GET / POST | /api/users | 管理员查询 / 新建用户 |
| PATCH | /api/users/{id} | 修改名称、角色与启用状态 |
| POST | /api/users/{id}/reset-password | 重置其他用户密码并撤销会话 |

## 验证

自动化测试覆盖计划边界、四路预算和种子/端口隔离、用户权限、协作取消、持久化恢复、独占管理锁、Windows 子进程树回收、两种采样时间轴、缓存刷新、视频 Range 及路径限制。真实采集与浏览器验收结果记录于统一仓库 archive/2026_10_16_平台自动采集与数据集回放.md。

## 独立插头任务与耗时

- `aviation_plug_insert`：航空插头，前方 `aviation_insert_40p5` 孔；末端长度 80 mm，检查完整朝向和插入深度。
- `guide_plug_insert`：导向插头，前方 `guide_insert_40p5` 孔；末端长度 95 mm，抓住后先对中、对正存放导槽再拔出，插入时检查键槽朝向。
- `cylindrical_plug_insert`：圆柱插头，前方 `cylindrical_insert_40p5` 孔；末端长度 95 mm，拔出前根据实际位姿对中存放孔。旧 `plug_insert` CLI 仍指向圆柱任务。

三种插头均保持自由刚体。抓取前持续跟踪实际抓取点，使用适合当前头部的稳定夹爪朝向；双侧手指接触后保持抓持，再按实际物体位姿对孔。该策略针对当前模型的上方抓取区域，不承诺任意翻滚或高速逃逸目标均可抓住。六维力用于载荷保护，尚未加入阻抗/导纳控制。

新任务在仿真时间零点等待录制 ON，相机的实际启动耗时不会额外推进初始物体。录制中继续使用完成确认与背压。运动速度、关节跟踪速度和保护阈值随任务参数写入记录；无模型位置强制更新、焊接、碰撞关闭或新增阻尼。

批次 `summary.json` 的 `wall_seconds` 从批次主流程开始计时，包含原生预检查、UE/仿真启动、录制、视频封装及逐帧验收；`attempts[].end_to_end_wall_seconds` 是单次尝试开始至验收完成的耗时。单次录制摘要还拆分 `recording_wall_seconds` 与 `finalization_wall_seconds`，任务的 `elapsed_sim_seconds` 为仿真时间。前端同时显示批次耗时与已验收尝试耗时。失败和重试不能计入成功条数。

CLI 示例（从服务端目录，使用平台 Python）：

```powershell
python tools/collect_plug_tasks.py --task aviation_plug_insert --scene <scene.json> --output <new-output-directory> --count 1 --max-attempts 3 --range-deg 0
```

首次验收使用零关节偏移，先确认各自任务的基础流程。页面支持偏移半范围 0–50°，默认 10°；不同随机初态需要预检查和实际重复验证，不能由单次成功推断成功率。

无相机运行入口的默认自由目标位置也从当前 MJCF 读取；正式验收仍显式传入同一场景文件，避免旧抓取演示的初态影响旁边物体及接触结果。导向件保持夹持时以连续就位几何判定是否可以慢速松爪；最终成功继续要求松爪撤离后低于 2 mm/s 的相对线速和 0.03 rad/s 的相对角速。

2026-10-16 固定初态正式验收：三项插头任务各有一条完整成功记录，详情见统一仓库 `archive/2026_10_16_独立悬空插拔件自动采集.md`。成功一轮从批次启动至完整校验分别约 35 分 1 秒、31 分 27 秒、28 分 57 秒；圆柱此前一轮受力保护失败已保留。以上为任务部分重叠运行时的实测。


## 可配置噪声与随机化

新建采集表单提供初始化独立开关、八组额外选项及参数。API 使用 CollectionCreate.initialization_enabled、range_deg、randomization；后者由 backend/space_arm_platform/collection_randomization.py 统一校验。GET /api/collections/catalog 返回参数单位、范围和默认值，前端从同一目录生成控件。旧请求不传这些字段仍采用旧默认行为。

| 选项 | 生效环节与默认参数（开关默认关闭） |
|---|---|
| joint_observation_noise | 六轴录制角度高斯噪声 σ=0.02°、角速度 σ=0.1°/s；200 Hz 取样，50 Hz 取对应样本 |
| joint_bias | 每条六轴独立均匀固定偏置 ±0.1°，整条不变 |
| action_noise | 六轴关节参考叠加二级低通 OU 随机偏差，驱动噪声尺度 0.03°、相关时间 0.2 s；幅度限制在 3 倍尺度内并保留关节限位；过滤后的实际标准差不等于输入尺度 |
| observation_delay | 关节本体观测及由它推导的末端位姿/速度延迟 20 ms；5 ms 网格，最大 200 ms；保留 joint_measurement_time_ns/age_ns，不伪造 RGB 时间；不延迟 RGB、力传感器、控制或验收 |
| wrist_wrench_noise | 每轴独立高斯噪声：力 σ=0.1 N、力矩 σ=0.005 N·m，传感器 site 坐标系；200 Hz 与同刻 50 Hz 读数相同；measurement 标为 simulated_noisy_interface_wrench，附 ground_truth_force_n/torque_nm |
| external_disturbance | 默认 ee_base 质心，力尺度 0.1 N、力矩尺度 0.005 N·m，每 1 s 独立采样并用五次曲线连续过渡；每轴截断 3 倍尺度；力为惯性/局部平行坐标，力矩为刚体坐标；仅任务执行时通过 BSK 标准力/力矩消息作用到原生 MuJoCo |
| lighting | 每条太阳光强相对原配置均匀 ±20%，经既有 RenderComponent/SceneSettings 传给 UE；不改曝光或图片像素 |
| dynamics | 每条全模型质量/惯量统一均匀 ±5%、摩擦统一均匀 ±5%；两个尺度独立；质量和惯量同倍率，显式接触 pair 摩擦同步缩放；几何和原模型不改 |

这些参数是工程试验起点，未依据实机标定。较大扰动可以导致失败，成功/失败仍按原接触、跟踪、载荷和归位判据记录。脚本控制反馈与安全保护使用真实状态；观测噪声用于训练数据，不代表已验证带噪闭环策略鲁棒性。录制的 action 为实际施加的参考，诊断 trace 另存 nominal_joint_reference。

动力学随机化生成每条录制私有的 standalone MJCF，绝对引用原网格，并复制姿态控制配置。BSK MJScene 和本地 MuJoCo 同时加载变体，保证引力、质量和惯量一致；渲染继续使用相同几何的原资源目录。暂不支持含 include 的模型；当前 SARM 平台 XML 无 include。模型变体及源模型 SHA-256、实际质量/摩擦尺度保存到 scene_instance.collection_config，不能只改 MuJoCo 一侧质量。

每次尝试使用独立 seed；初始化保留旧 PCG64 序列，其他选项以稳定名称拆分随机流；开关其他选项不会消耗本选项的随机序列。随机参数、实际偏置/倍率、版本、采样语义保存到每条记录的 scene_instance.collection_config。并行 worker 的 seed 区间隔离。所有选项关闭时运动和原始传感器值保持不变；6+1 状态/动作、50 Hz 双相机与 200 Hz 六维力的采样规格保持不变。

CLI 批采集从源 scene.collection_randomization 读取与 API 相同的配置对象，每次 randomized_scene 独立采样。单条 record_drawer_task/run_scripted_task 也支持该对象或已有 collection_config.randomization_options。不能仅编辑旧 disturbances 布尔标记而不提供参数配置。
