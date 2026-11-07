# 静止双星抓捕与搬运采集

复用 `zmh_linux` 的 `tools/collect.py`、布局批次、权威图像与观测配对、
`EpisodeRecorder`、`LiveLeRobotWriter`、LeRobot v3 保存及审核流程。
任务入口仍为 `capture_target_grasp`，任务文字包括抓捕、送达操作位置和持续保持。本任务使用底部四爪机械臂，
因此状态、动作、初态和成功判据按该机构定义。

## 任务边界

双星在初始化时具有相同惯性线速度，角速度为零。随后保留自由刚体、
机械臂反作用、轨道重力、姿态控制和原生接触；不持续锁死两星相对位姿。
不执行交会飞行、V-bar 或 PnP。保留原场景的环境、模型、材料、
太阳光、地球、云层、大气和原相机；不加载 ArUco 附件，不运行靶标估计。
原配置未启用的可选物理模块不会额外启用。

阶段：稳定初态 → 对齐 → 接近 → 闭爪 → 建立夹持 → 稳定保持20秒 →
搬运目标星 → 在操作位置稳定保持20秒。夹持力、接触峰值和碰撞约束
继续有效；没有运行时焊接、瞬移或改写目标星位姿。

对齐采用相对线速度和角速度前馈，补偿自由基座运动与目标机构的转动
力臂。相对跟踪修复先用原0.4 mm门槛验证；为减小搬运时偏心载荷，
整段任务进一步收紧到0.1 mm位置门槛和0.003 rad角度门槛。

搬运复用操作任务的原生几何查询、RRT连接搜索、C2样条和轨迹计时。
私有规划模型将目标星作为被携带的几何体，实际执行仍靠力控和接触。
终点取自 wenquandan777 的 `aacce0d` 提交中
`space_sim_server/model/scene_library/original-root-v2/initial-scene.json`：
目标星服务坐标位置 `[0.79285, 0.01, 0.311035]` m，四元数
`[0.5,0.5,0.5,-0.5]`（wxyz）。该场景改过安装座，不能直接复制其关节角。
终点要求位置误差≤5 mm、角误差≤1°，四爪均有支撑、爪轴向合力4.5–5.5 N持续20秒。
搬运 IK 优先从已完成物理送达的正腕角姿态分支求解，再保留起始姿态及其他
种子作为回退。该种子只用于求解，目标位姿、整星碰撞、力及四爪支撑检查不变。
任务版本为 `capture-target-grasp/5-preferred-handoff-branch`；旧版本的运行结果保留原身份。
这验证保存的目标星位姿；后续操作任务接续还应在同一模型下验证。范围扫描、物理测试和批采必须显式使用
该目标星自带操作台的模型及配套 catalog；默认旧模型中的服务星操作台不能替代它。

## 初态与范围验收

距离是**初始TCP到分离机构的轴向间隙**，不是两星中心距离。
位置方向锥、机构倾斜锥、绕轴角和机械臂初始关节姿态分别采样。
先搜索与测试，再冻结范围，用独立种子验证整个抓捕和搬运任务，并补测距离及方向锥边界。
几何跳过、执行失败和图像/保存不合格均计入候选总数。
范围及控制器、相机、环境身份一起保存；改变控制器或相机后重新标定。
预检通过只说明有限采样下未发现遮挡、不可达或碰撞，不等于抓稳。
最终范围与运行证据保存在外部运行目录，不把历史实验冒充新版本验收。

## 输入输出对照

| 项目 | zmh 操作任务 | 本抓捕与搬运任务 |
|---|---|---|
| 专家控制输入 | 仿真位姿、关节、速度、接触/力和阶段 | 机构相对位姿和相对速度、capture关节、接触/力和阶段 |
| 是否用RGB求专家动作 | 状态专家不需要 | 不需要 |
| RGB是否采集 | 是 | 是 |
| LeRobot `observation.state` | 当前四连杆夹爪配置为7维：6关节角+归一化开度 | 12维：6关节角、轴向滑台、凸轮角、4爪位移 |
| LeRobot `action` | 7维目标关节/开度；原始参考另存 | 12维当前已应用的伺服目标 |
| 图像 | overview、operating wrist | global、capture approach、capture jaw side |
| 频率 | 400 Hz物理、200 Hz控制、50 Hz图像 | 相同 |
| 目录 | data、videos、meta及平台辅助证据 | 相同 |

capture12的SI单位依次为 `[rad×6,m,rad,m×4]`。动作不表示夹持力或
成功标签，也不是尚未应用的下一拍命令。目标真值仅供专家/审核，
不写入VLA的状态特征。实际训练可以用图像、关节状态和任务文字预测动作；
保存的力信号是否用于模型输入由训练配置决定。

## 相机与照明

### 给后续操作脚本的相机交接

当前三路主要供调试和抓夹验收使用，不要求最终训练模型必须输入这三路。
正式训练可以采用合并流程中统一固定的相机；用于训练的RGB必须在真实采集时同步录下。
未录制的机位不能靠修改元数据补成另一个训练视角，VLA部署时也应提供与训练一致的相机输入。

当前默认实现仍实际录制这三路，并不是已经将它们排除在数据集之外。
替换时需同步修改 `capture_cameras.py` 中的挂载、`CAMERAS` 和 dataset/preview 路由，
以及 `recording-capture-grasp.json` 的 `camera_ids`；`recording_settings` 会检查两者一致。
如保留三路调试相机而另录训练相机，需明确各相机的采集用途，不能只改播放器布局。

预检目前使用末端接近相机的投影与遮挡条件，相关参数在 `capture_initialization.py`
和 `capture_preflight.py`。更换负责观察机构的相机后，要同步其外参、视场和可见检查，
重新生成与新相机/控制器身份匹配的校准和初态池；不能沿用旧相机的通过结论。
相机工作灯影响整个场景；调整灯位或开关后，也要检查最终训练视角的曝光和阴影。
相机固定、前后半段在同一动力学图内连续接续并统一动作表示后，
先完整试采一条、看抓夹与搬运画面并通过LeRobot审核，再启动小批和正式批采。

末端接近相机用于观察机构和接近过程，爪侧相机用于观察闭爪与接触区域。
全局相机观察两星及搬运路径；三者均为新增单目RGB相机，不是原双目的
左右镜头。原六相机保留挂载和光学参数，作为预览/视觉路由可用。
新增相机具有共享场景的工作灯，原太阳与原灯保留。曝光不做视频后处理。
全局机位相对服务星位于 `[1.8,-2.0,1.15]` m，朝向 `[0.95,0,-0.08]` m，
65°视场、f/8；比原机位更近并向原画面左侧移动。全局工作灯保留原挂载
`[2.4,-2.4,1.5]` m及朝向，不随此次镜头调整移动，末端相机和灯不变。
该机位已用一条已验收轨迹的UE重渲染检查全程构图；这不是新的动力学或LeRobot验收。
机位版本更新后，后续正式采集须重新标定，旧数据和冻结报告保持原身份。
验收必须看初始化、闭爪、夹持和搬运各阶段的实际渲染画面；
遮挡不能靠加亮解决，物理成功也不能替代清楚的抓夹画面。爪侧相机可看到近侧爪闭合入槽；背面接触被实体
遮挡，不声称同一相机能同时看清四个接触面。保留闭爪、保持、搬运的连续图像供审核。

## 运行

GitHub 源码包含本任务使用的保存场景：
`model/scene_library/original-root-v2/preheld-dual.xml`，配套
`ue-catalog.json` 使用可重定位模型根。执行 `git lfs pull` 后，
在下面的准备命令中显式传入该模型与 catalog；不要使用服务星自带操作台的旧模型。
已采数据、校准日志和生成的候选池保留在仓库外，不随源码提交。
`config/collection-capture-grasp.json` 是批采配置示例，需替换池和UE引擎路径；
改相机后先重新生成校准输入，不能直接使用旧的相机指纹。

在 `server/` 目录准备少量完整物理候选的命令如下。先复制采样配置到仓库外，
将 `sample_count` 改为少量，例如8；`--output` 必须是新的仓库外目录。

```bash
/path/to/native/bin/python tools/prepare_capture_initial_pool.py \
 --sampling-config /path/to/pilot-sampling.json \
 --model model/scene_library/original-root-v2/preheld-dual.xml \
 --catalog model/scene_library/original-root-v2/ue-catalog.json \
 --workers 1 --output /path/to/new-pilot
```

其 `pool.json` 是已实测的离散初态池，可用于完整试采和前后半段联调；
正式连续随机采集仍采用下述冻结范围池，不把少量离散成功初态重复称为随机覆盖。

依次使用 `tools/scan_capture_workspace.py` 做几何搜索、
`tools/prepare_capture_initial_pool.py` 做真实动力学验证，
冻结范围后用 `tools/prepare_capture_range_pool.py` 生成随机候选。
正式批采通过 `tools/collect.py` 的 sequence 配置运行，
指定 `capture_initial_pool`、`recording-capture-grasp.json` 和
`render_environment=platform_main`。审核同时检查视频、LeRobot官方读取、
50 Hz配对、400 Hz接触记录及最后20秒的独立位姿误差。

## 框架对应与复现

| 职责 | 直接沿用 zmh | 本任务适配 |
|---|---|---|
| 批次入口 | `tools/collect.py` | 只选择 `capture_target_grasp` |
| 顺序、种子、冻结、恢复和失败隔离 | `collection/sequence.py` | `capture_initial_pool` 替代工作台布局池 |
| UE/动力学进程管理 | `collect_scripted_tasks.py`、`record_scripted_task.py` | 抓捕模型、三相机、独立动力学解释器 |
| 权威图像与状态配对 | `ScriptedCaptureTransport` | 抓捕状态读取和任务上下文 |
| 实时 LeRobot v3 写入 | `LiveLeRobotWriter` | 12维机构状态/动作和单位 |
| 完成屏障、完整解码和数值审核 | `recording_validation.py` | `validate_capture_grasp.py` 的物理搬运验收 |

批采顺序和进程框架相同。抓捕使用不同自由度和自由目标星，动力学图每条重建，
不能把它称为与操作台任务完全相同的内部图复用。模型采用 wenquandan 的
`original-root-v2/preheld-dual.xml`；在准备副本中关闭预持有焊接，保留源文件。
准备副本还会恢复保存场景遗漏的太阳敏感器安装结构（原 CAD 的
`target_instance_0077_part_034_color_00`）。按同组保留零件核对 2.1 倍缩放与
安装位置，保留原材质、零质量和禁用碰撞属性，固定属于目标星。
源场景和旧录像不改写；准备器指纹改变后，旧冻结范围池不会自动继承验收。
工作台和可拆零件随目标星初始化并在原有保持物理中搬运，不能把服务星自带的
工作台误当成已拉过来的工作台。

先用 `scan_capture_workspace.py` 和 `prepare_capture_initial_pool.py` 做几何与完整动力学
验证；校准报告保存候选总数、所有失败/跳过和模型/平台哈希。
冻结后用 `prepare_capture_range_pool.py --domain /path/to/domain.json` 生成新随机候选，
每条在正式录制中执行一次，不能只重放少量已成功初态来冒充随机覆盖。
`config/capture-initial-sampling.json` 只是候选参数模板，使用生产随机池仍须提供匹配的
校准证据。范围内有限测试全成功也不能保证连续区域每一点成功。
当前首批候选模板的位置偏角锥、目标倾斜锥各为 20°；距离继续在
0.30–0.50 m 内随机，roll 与关节扰动范围不变。64 个同分布随机几何样本通过
61 个；另一个固定随机种子的 8 个完整任务候选完成 7 个，1 个初态视野不合格，
成功样本均经 400 Hz 力/支撑及末段实测位姿复核。有限样本不保证整个连续域每点成功。
10°/15°/20°/25°/30°随机几何预检分别为64/64、64/64、61/64、57/64、54/64；
30°有完整成功示例，整体候选通过率较低，保留作边界探索。
当前匹配证据与1000个新随机候选池位于仓库外
`/home/xzj/grasp-runs/capture-angle-domain-20261107-133055/`。
旧冻结池保留原 3° 身份，不能当成新范围的验证证据；正式入口读传入的初态池，
改采样模板后还需生成对应的新池。
视野预检允许中心射线穿过空心机构落到后板；仍要求检测区域无前景遮挡，
且至少三个采样点实际命中机构实体，不以孔中心必须为实体作为可见条件。

正式批次沿用 `layout-sequence/1` 配置，指定 `capture_initial_pool`、
`tasks=[{"name":"capture_target_grasp","enabled":true}]`、
`recording_config` 和 `render_environment="platform_main"`。
在 `server/` 中运行：

```bash
SPACE_SIM_DYNAMICS_PYTHON=/path/to/native/bin/python \
 /path/to/lerobot/bin/python tools/collect.py \
 --config /path/to/collection.json --output /path/to/new-batch
```

完整审核后批次的 `progress.json` 才将数据从 `pending_audit` 变为 `accepted`。
`episodes/` 保存通过数据，失败数据隔离到 `rejected/`，不删除诊断证据。
一条数据是一整段成功抓捕及搬运轨迹，内部包含同步图像、状态和动作的很多帧。
所有运行数据、校准报告和视频留在仓库外；本地修改不表示已经推送 GitHub。

## 2026-11-07 本地交付验收

首批模板冻结为轴向间隙0.30–0.50 m、位置方向锥≤20°、目标倾斜锥≤20°，
roll±3°，J1±20°、J2/J3±3°，其余关节固定。
独立随机完整候选7/8完成，1条视野拒绝；7条经400 Hz力/支撑与末段实测位姿复核。
另在独立交付目录用标准collect.py完成45 cm、双角度20°边界验收，
13442帧状态/动作及40326帧三路图像，
accepted=1、pending_audit=0，官方LeRobot读取和完整视频解码通过。
30°有成功示例，但也有50 cm、单倾斜30°样本出现非预期碰撞，未将30°作为主批范围。
交付包和范围图位于仓库外`/home/xzj/grasp-runs/capture-grasp-delivery-angle-domain-20261107/`。
该验收只覆盖前半段；未启动正式批次，未验收与后续操作的连续联调。
