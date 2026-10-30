# 实际末端双目预对准与视觉导航接口验证

> 2026-10-30上传说明：本文件保留2026-10-28/29试验的配置与结果口径；当前整合范围、机位版本和复验入口以[本周工作总入口](WEEKLY_TARGET_POINTING_VISION_20261030.md)为准。文中运行数据与视频仅本地保存，未提交Git；历史成功或失败不自动代表最新组合版本验收。


日期：2026-10-28。独立分支：`feature/cooperative-target-pointing-20261028`。未提交、未推送、未接管运行中的网页场景；相机参数、23块靶标、模型、材质和灯光原文件未修改。

后续已接通实际完整双臂动力学上的“GNSS短段到站→停稳→目标星调姿→等待视觉”，并完成三个候选距离、180°及显式GNSS诊断续行的五组验证。建议先在原17.4 m停泊阶段预对准；这不表示远处视觉已可用，也没有改网页主任务。新结果、误差及适用范围见[顺序动力学验证](TARGET_POINTING_SEQUENTIAL_DYNAMICS_20261028.md)，下文的简化动力学结果作为先前分项验证保留。

## 已实现的链路

`导航相对位置＋合作目标姿态/角速度＋服务星姿态＋同帧抓取臂编码器` → 实际双目位置和朝向 → 目标星期望姿态 → 受限力矩 → 对准/取景连续5秒检查。

`simulation/end_stereo_pointing.py`复用`SerialChainKinematics`，按`bottom_joint1`至`bottom_joint6`和`grip_frame`求实际抓取臂正运动学。两眼安装直接读取现有`capture26_stereo.json`；同时核对渲染安装、光学外参、双目标定和成像参数的一致性。用于预对准取景的是外围最大靶标的已知三维区域投影，不是“必须检测到几块”的新规则；原图像检测器未改。

对准检查要求姿态误差<1°、相对参考角速度误差<0.001 rad/s、导航位置主轴σ<0.05 m、四类输入有效新鲜、主靶标区域在两眼前方和画面内、正面朝向相机，连续5 s。控制更新中断、输入过期或相机朝错方向会撤销对准。输入失效时该模块输出零力矩并交回任务异常策略。它没有平移/合爪接口，也不产生视觉就绪或任务放行许可。

各状态需预测到共同控制历元。静态参考测试的参考角速度为0；运动时可调用`EndStereoGeometry.reference_motion()`，输入导航的J2000相对速度、服务星角速度、关节角和关节角速度，计算同一安装几何下的参考姿态及参考角速度，再传给控制器。其相对速度必须是本体原点相对位置在J2000中的导数，不能直接套用V-bar旋转系速度。内部采用当前速度在±1 ms局部预测后求旋转导数，不差分前后两条带噪姿态。它没有执行轨道传播或机械臂运动控制。已知三维区域投影通过不证明真实图像无遮挡、能解码或达到测姿精度。

## 发现了必须处理的朝向冲突

实际光学轴是抓夹−Z，渲染相机自身+X与光学系自身+Z代表同一条光轴。不能将渲染旋转矩阵第三列直接当作光轴。上一轮文档中的方向已修正。

抓取臂零位时，实际双目光轴在服务星本体系中约为`[−1.25e−5, −4.24e−6, 1]`。原V-bar方案把目标大致放在服务星+X方向。因此，即使目标星已转到正面朝向相机，相机仍可能拍不到目标。

候选解决办法之一是提前将抓取臂`bottom_joint1`预置到−90°，其他五关节为0。此时光轴约为`[1, −4.24e−6, 1.25e−5]`，可对准服务星+X接近方向。它只是候选预置姿态，未写回默认场景或主线任务。

在独立Python MuJoCo 3.7.0进程中，使用原完整模型、原SDF插件，以1°间隔检查0→−90°的91个静态构型：未发现筛选范围内抓取臂连杆与本体/另一臂的新穿透接触。测试将目标放远，其他臂关节为原模型零位；不代表任意操作臂姿态、运动动力学、力矩、完整抓夹内部接触或连续扫掠间隙通过。脚本为`tools/diagnostics/check_end_stereo_prepoint.py`，证据在`run/prepoint_collision_20261028/`。

原停泊共通条件要求关节接近零位，故采用该候选还需同步修改远距机械臂参考姿态和相应停泊检查；不能只改关节指令。另一选择是保持收拢臂并调整服务星参考姿态，两者应结合总体制导方案确定。本次没有绕过原放行条件。

## 实际安装下的动力学验证

脚本：`tools/diagnostics/validate_end_stereo_pointing.py`。读取当前双臂模型和相机配置，先对三个关节构型核对Basilisk原生MJScene正运动学：最大位置差`3.64e−16 m`、最大旋转矩阵差`2.20e−15`。这验证坐标计算一致性，不是物理标定精度。

随后采用原目标星/面板零位质量、质心和惯量组成10 kg刚体，使用Basilisk MJScene RKF45、名义240 Hz物理/20 Hz控制、默认GNSS噪声和相对EKF。目标星姿态/角速度及服务星姿态明确采用理想遥测，抓取臂固定在候选预置构型；没有轨道引力、展开动力学、接触或图像检测。转动通过≤0.02 N·m等效力矩积分产生，循环中不覆盖目标星状态。

下表距离是**预期对准构型的机构中心在左眼中的光学深度**。代码按相机/靶标安装偏移求初始本体原点间距，未将光学深度与本体原点间距混用；初始倾斜会改变实际光学深度。

| 参考光学深度 | 初始姿态偏差 | 对准并连续保持5 s的时刻 | 该时刻姿态误差 | 该时刻角速度误差 |
| --- | --- | --- | --- | --- |
| 17.4 m | 0° | 5.00 s | 0.0163° | 0.0000199 rad/s |
| 17.4 m | 30° | 38.85 s | 0.1563° | 0.0000602 rad/s |
| 17.4 m | 90° | 42.55 s | 0.4183° | 0.0006020 rad/s |
| 17.4 m | 180° | 58.45 s | 0.0517° | 0.0003244 rad/s |
| 50 m | 90° | 42.55 s | 0.4180° | 0.0006072 rad/s |
| 5.5 m | 90° | 42.65 s | 0.4158° | 0.0006016 rad/s |

六项通过。另有零位收拢臂、目标在+X方向的对照：目标星姿态误差降至约0.0065°，但两眼取景不合格，正确保持未对准状态。所有案例还核对了导航输入过期后撤销对准并停止该模块的力矩输出。七项合计通过；初始偏差绕测试轴`[1,2,3]/√14`施加，不代表所有方向遍历。

结果：`run/end_stereo_pointing_20261028/summary.json`及各轨迹JSONL。

## 补充轴向及运动参考验证

脚本：`tools/diagnostics/validate_end_stereo_tracking.py`，继续使用上述简化动力学、实际相机安装及固定候选机械臂构型。新增11项全部通过：

| 工况 | 数量 | 完成时刻 | 末帧姿态误差 | 末帧角速度误差 |
| --- | --- | --- | --- | --- |
| 绕目标参考X/Y/Z轴，分别±90°及180° | 9 | 34.65—61.55 s | 最大0.2823° | 最大0.0003022 rad/s |
| 服务星绕视线缓慢滚转，目标有初始角速度 | 1 | 52.30 s | 0.0249° | 0.0004716 rad/s |
| 目标侧向直线平移，服务星缓慢转动 | 1 | 52.55 s | 0.0242° | 0.0004205 rad/s |

后两项的服务星初始角速度为0.0011 rad/s；目标初始角速度分别为`[0.01,−0.015,0.005] rad/s`及0，侧向平移案例的初始目标质心速度为`[0,0.02,0] m/s`。运动参考率使用带噪GNSS/EKF位置与速度、服务星理想姿态/角速度和零关节速度计算，没有读取目标平移真值。每项达到原5 s对准要求后，又保持对准状态10 s才记为完成。

九项轴向工况扩充了初始姿态覆盖，不代表所有初始姿态、绕轨运动、外扰及遮挡都已遍历。两项运动参考是无引力自由体试验，不能称为完整轨道跟踪已验收。各项力矩仍≤0.02 N·m，导航输入过期均撤销对准并停止本模块输出。证据在`run/end_stereo_tracking_20261028/`。

## 视觉给导航的接口

新增`simulation/image_relative_navigation.py`。只接受当前会话、正确左右眼、图像算法已接受的`joint_multicamera`结果，不要求每眼两块；同一帧不能重复更新。图像采样时间保留，拒绝过期/未来图像；机械臂与服务星变换必须对应图像历元，不能直接套用处理完成时的当前末端位置。

设图像输出为目标本体系到左眼光学系的`R,t`，则目标原点相对服务星原点的位置为：

`z_N = R_N←B × (p_B←C + R_B←C × t_C←S)`。

其中相机位置和旋转由抓取臂编码器/FK及固定安装外参得到。输出可交给现有`RelativeEKF.update()`的三维位置更新：

| 字段 | 定义 |
| --- | --- |
| `source` | `stereo` |
| `stamp` | 原图像物理采样时刻，s |
| `z` | 目标本体原点−服务星本体原点，J2000轴，m |
| `R` | 同一位置估计在J2000轴下的总协方差，m²，由接入方显式提供 |
| `rotation_n_from_target` | 图像估计的目标本体系到J2000旋转，作为附加输出；当前位置EKF不融合该旋转 |
| `quality` | 使用的ID、原始重投影误差等质量信息 |

不能直接把重投影像素误差填成位置协方差。安装、服务星姿态和编码器误差需纳入总不确定度并校准；本次没有给实时运行设置未经验证的默认协方差。该适配器不自动满足旧版参数化`VisualReadiness`，接入时还需确定图像质量和连续接受观测的规则。

## 历史实际图像结果回放

脚本：`tools/diagnostics/validate_image_navigation_replay.py`。读取原网页灯光下的历史`webpage_force_hold225_20261024`：原视觉位姿、抓取臂编码器、同一图像历元的服务星姿态。丢弃图像记录中的渲染外参诊断信息。转换全部完成后，才另读真值评分。没有重新识别图片、改照明或让真值参与转换。

2249组中2219组转换并评分，原算法拒绝的30组保持拒绝；2249组均找到同历元编码器/姿态。以下只覆盖该历史近距抓取路径：

| 评价区间及参考点 | 位置RMSE | 最大位置误差 |
| --- | --- | --- |
| 全历史路径，导航所需两本体原点相对位置 | 0.515 mm | 2.486 mm |
| 全历史路径，分离机构中心相对抓夹 | 0.243 mm | 1.024 mm |
| 0.5 m→5 cm，导航所需两本体原点相对位置 | 0.586 mm | 2.027 mm |
| 0.5 m→5 cm，分离机构中心相对抓夹 | 0.294 mm | 1.024 mm |

0.5 m→5 cm有1420组有效结果，采用原1 mm初始距离分组容差。参考点不同，所以误差数值不同。机构中心误差与原评分相符，说明这次坐标转换未改变原视觉估计结果。拒绝帧不计成零误差。

回放调用接口时显式提供了1 cm标准差的**API测试协方差**以及0.15 s测试交付延迟；它们未被用于导航控制或宣称真实滤波性能，不是实测协方差/延迟。服务星姿态作为历史理想遥测。结果在`run/image_navigation_replay_20261028/`。

## 当前进度与连续交接边界

| 工作 | 当前结果 |
| --- | --- |
| 17.4 m视觉验收条件 | 已梳理现有三维位置接口；测量类型/质量门槛仍需团队确定 |
| 目标星调姿 | 实际安装/FK、取景、5 s对准检查及完整模型短段停泊调姿顺序试验通过；网页自动触发与实际姿控硬件未验收 |
| 视觉结果交给导航 | 适配器及现有EKF接口测试通过；历史实际位姿/编码器/姿态回放通过；总协方差待校准 |
| 连续任务交接 | 尚未通过完整实时任务，不能宣称17.4 m已视觉闭环接近至抓取 |

当前完整连续交接有明确限制：17.4 m处5 cm黑框仅约2.315 px，现有编码检测不能可靠读码；收拢相机朝向与原V-bar姿态/关节放行条件存在冲突。此前默认环境的MuJoCo版本问题，后续已通过找到原双臂试验保留的兼容3.7.0运行时解决；原生后端13项测试及新增GNSS动力学诊断见[记录](GNSS_DYNAMICS_AND_VISUAL_OVERLAP_20261028.md)。没有放宽版本校验、改灯或将真值填入导航。完整远近距、调姿、视觉和抓取交接仍未验收。

下一次整场景验证前，需落实相机预指向构型及相应关节/姿态参考、匹配权威动力学运行环境，并明确远距成像/测量方案和图像观测协方差。本次改动保留为独立模块与可复验工具，供接入和评审。

## 复验命令

从本独立工作区的`space_sim_server/`执行；下例源路径指向现有双臂实验工作区，仅读取其模型、配置和历史结果。`python`应使用现有含相应依赖的解释器。静态接触工具必须独立运行，不能与Basilisk导入同一进程。

```powershell
python -m pytest tests/test_cooperative_pointing.py tests/test_end_stereo_pointing.py tests/test_differential_gnss.py tests/test_gnss_transport.py tests/test_near_field.py -q
```

```powershell
python tools/diagnostics/validate_end_stereo_pointing.py --scene-model ../../space_sim_main_dualarm_20261022/space_sim_server/model/SARM/platform/sarm_dual_capture26.xml --camera-config ../../space_sim_main_dualarm_20261022/space_sim_server/config/vision/capture26_stereo.json --output run/end_stereo_pointing_20261028
```

```powershell
python tools/diagnostics/validate_end_stereo_tracking.py --scene-model ../../space_sim_main_dualarm_20261022/space_sim_server/model/SARM/platform/sarm_dual_capture26.xml --camera-config ../../space_sim_main_dualarm_20261022/space_sim_server/config/vision/capture26_stereo.json --output run/end_stereo_tracking_20261028
```

```powershell
python tools/diagnostics/check_end_stereo_prepoint.py --scene-model ../../space_sim_main_dualarm_20261022/space_sim_server/model/SARM/platform/sarm_dual_capture26.xml --plugin ../../space_sim_main_dualarm_20261022/space_sim_server/run/native_acceleration/sarm_thread_76cadd5c27eab99e/sarm_thread.dll --output run/prepoint_collision_20261028
```

```powershell
python tools/diagnostics/validate_image_navigation_replay.py --source-directory ../../space_sim_main_dualarm_20261022/space_sim_server/run/end_stereo_20261023/results/webpage_force_hold225_20261024 --scene-model ../../space_sim_main_dualarm_20261022/space_sim_server/model/SARM/platform/sarm_dual_capture26.xml --camera-config ../../space_sim_main_dualarm_20261022/space_sim_server/config/vision/capture26_stereo.json --interface-test-sigma-m 0.01 --output run/image_navigation_replay_20261028
```

上述pytest命令共97项通过；两组原生取景/调姿共18项通过，其中1项为预期不放行的相机朝错方向对照。接口回放的`passed`还要求已转换帧的两本体原点及机构中心最大位置误差均<3 mm；这一项只针对该历史近距数据。
