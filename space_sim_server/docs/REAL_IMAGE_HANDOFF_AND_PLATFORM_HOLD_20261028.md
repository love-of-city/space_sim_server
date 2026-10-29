> **状态更正（2026-10-29）：额外长焦相机已按用户要求撤回。下文为历史试验记录，17.4 m、5.5 m及远近切换结论不适用于当前原相机方案；复现长焦的入口已禁用。仅原100°双目的0.5 m驻留结果继续适用。当前结论见[流程报告](RENDEZVOUS_FLOW_AND_NODE_ACCEPTANCE_20261028.md)，靶点信息见[第7节补充](TARGET_MARKER_REPORT_20261029.md)。**

> 2026-10-30上传说明：本文件保留2026-10-28/29试验的配置与结果口径；当前整合范围、机位版本和复验入口以[本周工作总入口](WEEKLY_TARGET_POINTING_VISION_20261030.md)为准。文中运行数据与视频仅本地保存，未提交Git；历史成功或失败不自动代表最新组合版本验收。


# 原灯光下的真实双目接入、5.5 m停泊和0.5 m驻留验证

2026-10-28。当前在独立分支、独立UE与动力学进程完成，尚未替换网页主任务，也没有宣称50 m到抓取已在同一任务中连续通过。

## 本轮实际完成的变化

保留成功近距场景的23块方形ArUco、原100°双目、太阳、地球、金色外皮、接口材质、原主视口及右眼工作灯。增加**被动6°长焦双目候选通道**，渲染焦距和检测内参一起改变，不用数字放大代替光学分辨率，也不为新通道创建工作灯。仍是1920×1920、0.12 m基线、10 Hz、2 ms、F4的仿真设计；尚未选定实体镜头或验证额外机壳的质量、安装和碰撞。

成功近距23靶标配置单独保存为`config/vision/capture26_near23_reference_20261028.json`。不覆盖最新main的默认相机配置。

长短焦通道目前共用名义安装外参，表示光学设计候选；不能据此认为四台独立实体相机已经完成结构布置。

新增远距求解路线：**合作目标星姿态遥测＋两张真实图片→相对位置**。远处平面靶标正对相机时，完整图像PnP可能存在多个近似等价朝向；原来的完整PnP仍保留歧义拒绝。新路线明确使用合作姿态，单独优化三维平移。平移没有输入GNSS位置、目标位置真值或渲染外参；姿态不能表述成独立视觉测量。近距继续用OpenCV PnP初值与双目联合重投影优化求完整图像位姿。

修复`RelativeEKF.update()`的时间比较：整数纳秒转浮点秒后，同历元图像偶尔比控制时刻大约10⁻¹⁵ s，原代码误判为未来观测并拒收。现在只容许1 ns的数值容差，真实未来、过期和非法时间仍拒绝。保留原位置、重投影、完整靶标及视觉连续计数要求。

## 17.4 m处：调姿、图片进入导航、短段续行

本次从名义17.9 m独立初始化，目标星倾斜90°。完整原双臂动力学、400 Hz积分、20 Hz控制，GNSS持续工作。

| 仿真时间 | 实际事件 |
| --- | --- |
| 0–5.20 s | 在初始站位获取并检查导航，目标保持初始倾斜姿态 |
| 5.20–41.25 s | GNSS引导到17.4 m名义光学节点对应的固定质心参考 |
| 41.25–46.25 s | 到站后连续稳定5 s |
| 46.25 s | 允许目标星受限力矩调姿，服务星继续保持固定质心参考 |
| 94.10 s | 完成预对准检查，进入真实图片等待 |
| 95.10 s | 连续10组实际图片被EKF接受，协调器放行 |
| 110.30 s | 结束17.4→17.3 m名义短段，继续保持10 s |

目标星控制根据GNSS相对位置、合作姿态和编码器计算期望朝向，使主靶标面朝向相机中点，并保持图像上方方向。误差以旋转矩阵计算，通过有界PD输出力矩；本次最大力矩0.020 N·m。该等效执行器没有验证喷嘴/飞轮硬件，不直接写目标星姿态。

94.2–120.3 s共262组真实双目图片，262组均进入EKF。图像单次**目标本体原点位置**误差RMSE14.454 mm、最大40.490 mm；同一图像区间，GNSS与图像融合后的位置误差RMSE3.887 mm、最大6.499 mm。实际质心相对当前运动参考的偏差RMSE3.684 mm、最大4.408 mm；后者不能写成某个固定停泊点的到站偏差。

此处采用理想合作姿态、服务星姿态与编码器假设，图像观测总位置不确定度显式设为每轴25 mm。独立静态敏感性测试中，6°通道在17.4 m的30组图片均解出位置：理想合作姿态下最大26.016 mm；确定性加入0.5°姿态偏差后最大45.253 mm。该扰动测试只覆盖一个斜轴的正负扰动，不等于完整姿态传感器标定或统计覆盖率验证。

## 0.5 m工具间隙：原近距图像闭环

这是一组独立初始化的20 s驻留试验。机构中心初始位于夹爪−Z方向0.5 m处，横向目标为零；相机前置关系使光学深度约0.55 m。因此没有把光学深度0.5 m当作工具间隙0.5 m。

200组原100°真实双目图片全部被EKF接受，使用完整图像PnP与双目联合优化，图像观测位置不确定度显式设为每轴1 mm。1–6 s连续合格，6 s进入驻留；6–11 s再次连续合格，完成检查。此后保持至20 s，未出现进入后质量丢失锁存。

| 1–11 s连续验收区间指标 | RMSE | 最大值 | 要求 |
| --- | ---: | ---: | ---: |
| 实际机构中心相对指定工具位置的三维偏差 | 1.687 mm | **1.969 mm** | <6 mm |
| 实际有符号轴向间隙偏差 | 0.297 mm | **0.402 mm** | <6 mm |
| 融合导航位置估计误差 | 0.458 mm | **0.857 mm** | 单列报告，不代替上两项 |
| 实际质心旋转系相对速度 | — | **0.839 mm/s** | <2 mm/s |

实际位置和间隙另从权威目标、夹爪刚体状态复算；与日志FK评分的最大差约2×10⁻⁹ m。无星间接触、无动力学外部状态覆盖。这里只验收当前保存的非零机械臂构型下的位置、间隙、速度及视觉连续驻留，未覆盖主任务要求的零位收拢机械臂入口等全部共通条件。

同一1–11 s窗口的实际固定质心站位偏差RMSE1.687 mm、最大1.963 mm，也小于6 mm；与工具相对位置、轴向间隙分别复算，未用一个量代替另一个量。

包含启动段时仍存在约27.9 mm的首次GNSS初始化误差，不将其从全程统计删除。1–11 s是实际连续验收区间，不是“滤波一启动就小于1 mm”。

## 5.5 m：真实视觉与稳定停泊联合检查

本次独立从名义6 m初始化，目标已基本朝向相机，表示先前完成17.4 m调姿后的候选入口，并未继承前一试验状态。41.75 s到达固定5.5 m站位，停稳5 s后启用对准保持；66.9 s进入真实图像等待，67.9 s起视觉就绪，**67.9–72.9 s连续满足视觉、位置、速度和对准条件**；72.9 s放行，前进0.1 m，88.1–98.1 s末端保持完成。

| 连续联合验收窗口67.9–72.9 s | RMSE | 最大值 | 要求 |
| --- | ---: | ---: | ---: |
| 实际固定停泊点三维偏差 | 3.965 mm | **4.154 mm** | <20 mm |
| 融合导航位置估计误差 | 1.052 mm | **1.469 mm** | 单列报告 |
| 实际质心旋转系速度 | — | **0.586 mm/s** | <2 mm/s |

全图像区间共312组双目均被EKF接受，图像单次本体原点位置RMSE1.230 mm、最大2.953 mm；同期融合导航RMSE1.001 mm、最大1.907 mm。6°通道沿用合作姿态辅助求位置，观测位置不确定度显式设为每轴3 mm；理想姿态及编码器、保存的非零臂构型等假设仍然适用。以上通过的是视觉连续性、位置和速度分项，不代表主任务全部共通放行条件已验收。

保留失败对照`station5p5_full_image_s7`：虽然同通道静态30/30图片都能求完整位姿，实际动态完整PnP试验在放行后115.3 s出现朝向歧义拒绝，控制器锁存SafeHold。未降低歧义门槛，改为上述明确带合作姿态的路线重新验证。不能将此写成仅靠独立图像姿态也已稳定通过5.5 m闭环。

## 远近相机重叠区

真实UE静态图片测试中，6°通道在5.5 m处的30组、0/15/30°两轴倾斜及微小位置偏移全部通过完整图像位姿求解，位置RMSE1.268 mm、最大3.345 mm。12°通道同组30/30、最大5.625 mm；上述动态失败说明静态有效率不足以替代闭环试验。

1.5 m光学深度处，6°与原100°两路各30组静态图片均通过完整图像位姿求解：长焦最大0.751 mm，原广角最大5.992 mm。广角在0°和15°全部满足24 px就绪尺寸要求；30°时12组中只有6组满足，不能据此允许任意大倾角切换。

已补做同一动力学运行中的远近通道切换：`range_transfer_kinematic_images_s7`独立从1.9 m初始化，到1.8 m停稳后对准并前进至1.3 m。两对相机按相同帧号/历元采集实际图片；实际图像和滤波连续合格后，在63.6 s、**真实光学深度1.602154 m**切换至原100°广角。切换后只融合广角图像，长焦更新数为零，84.2 s完成减速，保持至94.2 s，无视觉中断、无状态覆盖或星间接触。

| 切换后63.6–94.2 s指标 | RMSE | 最大值 |
| --- | ---: | ---: |
| 广角单次图像本体原点位置误差，307/307组被接受 | 5.517 mm | 15.919 mm |
| GNSS与广角融合导航位置误差 | 1.858 mm | 3.833 mm |
| 实际质心相对当前运动参考的偏差 | 1.858 mm | 2.881 mm |

本段通过的是通道交接、实际视觉连续性与末端保持，**不满足“全段导航误差<3 mm”**；该范围也不属于此前0.5 m至5 cm抓取误差统计。24 px就绪尺寸不能等同于单次图像已有毫米级精度。使用长焦每轴3 mm、广角每轴6 mm位置不确定度假设；切换前两对图像顺序更新同一EKF，额外假设两通道图像噪声独立，未验证实际交叉相关。候选相机仍共用名义安装外参，未验收四台独立实体相机的结构布置。原网页12 m/5 m保护没有改动。

保留未通过对照：`range_transfer_images_s7`人为等对准完成才启动图片，在160 s预算内停留TargetPointing，未完成切换；`range_transfer_continuous_images_s7`图片始终开启，但未建模本体原点转动并使用较大过程噪声，90.15 s时仍无法保持连续对准，明确中止，保存`interrupted.json`。本段成功试验另用下述原点预测补偿，不把中止运行计为通过。

### 调姿时的本体原点预测补偿

近距重叠试验发现，原导航跟踪两星本体原点，而平移控制按整星质心施力。目标转动会改变本体原点相对质心的速度；此前用较大的未建模加速度过程噪声覆盖这部分运动，近处会使速度估计和期望视线角速度波动，连续对准检查难以完成。

新增显式可选的`simulation/origin_lever_arm.py`，输入已知质量/关节几何、同历元合作姿态、角速度和编码器位置/速度；没有目标平移真值输入。设两星质心相对各自原点的惯性系差为`ℓ=R_T c_T−R_B c_B`，则本体原点相对位置为`r_origin=r_COM−ℓ`。预测增加`Gℓ−Δv_ℓ/Δt`，其中`v_ℓ`由角速度和关节运动学计算。用模型补偿转动引起的原点加速度，原姿态和角速度门槛保持。

补偿通过`--kinematic-origin-prediction`单独启用，原试验默认行为保留以便复现。目标/服务星恒角速度的解析方向、关节引起的质心运动、引力梯度偏移和无效时间输入检查通过。当前仅适用于明确声明的理想合作角速度与编码器假设；实物带噪声角速度的差分、滤波及协方差需要另验。相关测试共154项通过；原基础CI资源缺失情况保留。

## 推荐的整体顺序与尚未完成项

GNSS到50 m检查位置→GNSS到17.4 m固定站位并停稳→目标星预对准→真实长焦位置观测合入GNSS导航→5.5 m做视觉就绪与稳定驻留联合验收→重叠区验证并切换原广角双目→工具间隙0.5 m前后各5 s检查→原近距视觉机械臂抓取。

当前新增验证覆盖17.4 m调姿后真实图像接入、5.5 m视觉与稳定停泊联合检查、1.9→1.3 m实际远近相机切换、0.5 m工具间隙驻留四个独立环节。仍需把各环节接回网页连续任务、从同一状态跑通17.4→5.5→相机切换→平台驻留，验证通信/丢帧延迟与全部共通门限，最后进入原实际接触抓取。历史0.5 m至5 cm抓取成功不能替代这次连续任务。额外长焦硬件也需要确认可装入航天器。

## 新视频及证据

`run/dual_range_vision_20261028/review_delivery/`：

- `17p4m_real_visual_handoff.mp4`：26.2 s，实际图像输入的94.2–120.3 s采样段；已经调姿后的接入与续行，未包含此前90°旋转画面。
- `5p5m_real_visual_dwell.mp4`：31.2 s，本次5.5 m处实际双目、连续联合停泊检查、续行与末端保持。
- `0p5m_real_visual_hold.mp4`：20 s，本次工具间隙驻留的实际左右输入。
- `long_to_wide_real_handoff.mp4`：94.2 s，本次1.9→1.3 m独立切换试验；按实际活动通道显示左右原始输入，切换时视场从6°变为100°。
- `independent_audit.json`：逐帧真值复算及分项结论。
- `optical_overlap.json`：1.5 m重叠区的完整靶标像素和解算质量。
- 各视频有同名JSON，记录单次运行、图像时段、源图像哈希、视频哈希和完整解码通过。

视频只做显示缩放和独立字幕带，没有修改算法输入、亮度、材质、背景或帧间插值。按仿真10 Hz采样播放；物理等待UE图像返回，因此尚未验证真实机器上的10 Hz端到端时延。

## 复现入口

在`space_sim_server`下，使用兼容的Basilisk 2.11.1/MuJoCo 3.7.0环境：

```powershell
python tools/diagnostics/validate_gnss_proximity_dynamics.py --basilisk-root <兼容运行时> --source-server <原成功工作区的space_sim_server> --scene-model <原sarm_dual_capture26.xml> --source-scene <原baseline_300s_input.scene.json> --camera-config config/vision/capture26_near23_reference_20261028.json --output run/<新目录> --mode pointing_handoff --depth-m 17.9 --pointing-depth-m 17.4 --end-depth-m 17.3 --duration-s 200 --initial-target-angle-deg 90 --image-channel long --long-focus-fov-deg 6 --image-position-sigma-m .025 --cooperative-attitude-images --images-after-prealignment --terminal-hold-s 10

python tools/diagnostics/validate_gnss_proximity_dynamics.py --basilisk-root <兼容运行时> --source-server <原成功工作区的space_sim_server> --scene-model <原sarm_dual_capture26.xml> --source-scene <原baseline_300s_input.scene.json> --camera-config config/vision/capture26_near23_reference_20261028.json --output run/<另一个新目录> --mode hold --depth-m .55 --tool-gap-m .5 --duration-s 20 --image-channel wide --image-position-sigma-m .001

python tools/diagnostics/validate_gnss_proximity_dynamics.py --basilisk-root <兼容运行时> --source-server <原成功工作区的space_sim_server> --scene-model <原sarm_dual_capture26.xml> --source-scene <原baseline_300s_input.scene.json> --camera-config config/vision/capture26_near23_reference_20261028.json --output run/<5p5新目录> --mode pointing_handoff --depth-m 6 --pointing-depth-m 5.5 --end-depth-m 5.4 --duration-s 160 --initial-target-angle-deg 0 --image-channel long --long-focus-fov-deg 6 --image-position-sigma-m .003 --cooperative-attitude-images --images-after-prealignment --terminal-hold-s 10 --station-tolerance-m .02 --visual-dwell-s 5

python tools/diagnostics/validate_gnss_proximity_dynamics.py --basilisk-root <兼容运行时> --source-server <原成功工作区的space_sim_server> --scene-model <原sarm_dual_capture26.xml> --source-scene <原baseline_300s_input.scene.json> --camera-config config/vision/capture26_near23_reference_20261028.json --output run/<切换新目录> --mode pointing_handoff --depth-m 1.9 --pointing-depth-m 1.8 --end-depth-m 1.3 --duration-s 130 --initial-target-angle-deg 0 --image-channel dual --long-focus-fov-deg 6 --image-position-sigma-m .003 --near-image-position-sigma-m .006 --cooperative-attitude-images --terminal-hold-s 10 --kinematic-origin-prediction
```

这些入口都需要原成功场景的已准备渲染资产、本机UE5.6、原信令和网页工作灯接口；它们不会写原`scene_runtime.json`或原项目灯光设置。运行数据和视频不提交Git。
