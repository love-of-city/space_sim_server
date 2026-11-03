# 本地场景库

当前操作臂安装修正为 `original-root-v2`，位于 `D:/mingti/outputs/vertical-arm-mount-20261103`。
恢复原 `link1` 根部 `(0,0,0)`，删除额外斜向支撑；重新求解关节初态并保持旧版夹爪预抓取位姿。
目标星、对接臂和 L 型内嵌任务盒不变。21 项初态审查、显式安装干涉检查及两秒原生保持通过，139 项相关测试通过。
原根部连接处碰撞代理的约 8.285 mm 重叠单独记录，不能当成零穿插设计验收；完整作业尚未复验。
第 2、4 类八项初始化配置已登记，实际 UE 状态以本目录 `scenes/verification.json` 和 `review.html` 为准。
细节见[恢复操作臂原竖直底座](../../archive/2026_11_03_恢复操作臂原竖直底座.md)。

本地嵌入式候选 `recess-v5` 位于 `D:/mingti/outputs/recessed-taskbox-20261102`。
目标任务盒工作面与宽背面齐平，后部 102 mm 机构/内衬位于箱体内部；实际箱壁、内部 CAD 曲面和碰撞体均开槽。
空腔深 107 mm，面内留 5 mm 余量，抽屉最深行程至内部背板仍有 5 mm 间隙。
目标箱体仍为 2.1 倍、双任务面方正 L 型；任务面中心调至操作星坐标 `(0.62,0.10,0.26) m`，
用于避让两星箱体，重新求解对接臂关节初态。两臂根部、操作臂初态和相机安装参数保留 facing-v1。
原接口和靶标的目标本体坐标与尺寸保持。新模型的 21 项初态审查与两秒保持独立核验已通过。
本轮 138 项相关测试通过；开放曲面裁剪测试在原生环境缺 trimesh 而跳过，已在几何环境单独核验面积、外边界和法向。
26 个开槽网格已导入 UE，第 2、4 类八项配置已登记。航空插头初始化的四路原相机实拍及模型来源核验 1/1 通过，
另有三张实际自由观察图；见 `review.html`、`scenes/verification.json` 和 `scenes/visual-review.json`。
侧视确认外装厚壳消失、任务面贴箱面和两臂根部位置；原总览仍有操作臂遮挡和目标盒顶部裁切，对接腕部为极近景。
这不是所有视角或完整八项作业通过，历史截图未冒充本版实拍。

以下为前版外装任务盒和对接臂安装记录。

当前对接臂安装候选为 `facing-v1`，目录 `D:/mingti/outputs/facing-capture-arm-20261102`。
两臂根部都属于操作星 `cubesat_bus`：操作臂 `link1` 安装位置、支座和初态保留；
对接臂 `bottom_mount` 从 -Y 侧面改装至朝向目标星的 +X 箱面，根部位置 `(0.6,0.2,-0.35) m`，
根部轴向 +X。81 个真实箱壁三角网格采样确认其 11 cm 方形安装区域贴合 `X=0.6 m` 箱面。
目标星位姿、2.1 倍箱体、方正 L 型任务盒、原接口和靶标均沿用 `square-v2`。
对接臂只改变固定根部位姿和初始关节角；操作臂始终安装在操作星上。
20 项初态审查、两秒保持独立核验、129 项相关测试及 Ruff 通过；八项配置已登记，
冻结索引 `installed-index-facing-v1.json`。实际 UE 原图及来源核验见 `scenes/verification.json`。
四路实际截图来源核验 1/1 通过；补充侧视图可见两臂根部均连接操作星及对接臂新安装面。
原总览仍有操作臂局部遮挡和目标盒裁切；完整图集 `review.html` 如实保留这些限制。
对接臂腕部相机随新臂姿态运动；全部相机的本体安装参数、总览机位及操作臂初态均保持。
本版不是完整八项作业验收，仍仅本地修改。

以下为上一版方正布局记录，其对接臂还在 -Y 侧面。

当前方正 L 型候选为 `square-v2`（2026-11-02），位于 `D:/mingti/outputs/horizontal-task-box-20261102`。
目标星水平偏角已归零，目标任务面法向为 `(-1,0,0)`，操作星任务面法向为 `(0,0,1)`；
目标盒水平边沿操作星 Y 轴，竖直边沿 Z 轴。任务面中心为 `(0.59,0.10,0.235) m`，
盒背板下缘比本星任务面高 10 mm；箱体缩放仍为 2.1，任务盒直接贴宽背面并向箱面一端平移。
删除该偏移安装区内额外一处固定后缘凸起 `instance_0077`，完整保留原接口和靶标保护面。
双臂安装、操作臂初始关节角和原系统相机不变。20 项初态审查、两秒保持核验、119 项测试通过。
第 2、4 类八项配置已登记；冻结索引 `installed-index-square-v2.json`，完整作业仍待复验。
实际 UE 实拍与来源核验见 `scenes-square-v2/verification.json`；视野覆盖是否满足必须结合原图判断。
本次四路实拍来源核验 1/1 通过：总览已呈方正 L 型，但操作臂仍遮挡右侧部分任务区，目标盒上缘裁切；
操作臂腕部能看见目标任务面及孔位，对接臂腕部仍是原接口近景。不能标记全部视角覆盖通过。
`square-v1` 的底座约 7.965 mm 干涉及后缘凸起冲突记录保留，未登记。

上一横置候选为 `wide-v8`，因水平斜转导致盒边不方正，已替换，以下为历史记录。
箱体缩放从 5.6 降至 2.1，目标星长轴横置，任务盒贴 `-Y` 宽背面；水平偏转 -35°。
目标任务面中心为操作星坐标 `(0.67,0.45,0.25) m`，法向 `(-0.8191520,0.5735764,0)`。
完整任务盒下缘高出本星任务面 25 mm，两面成直角；`horizontal-contract.json` 核对这些几何条件。
原接口、四靶标的尺寸和本体安装坐标、双臂安装、操作臂初态及原相机保持。
清除背面安装区域内固定板、凸起件及对应固定板碰撞代理，保留原背盖、外侧活动板与铰链。
初态全部安装/干涉审查、两秒原生保持核验和 118 项相关测试通过。八项配置已登记，
冻结索引为 `installed-index-wide-v8.json`；完整八项任务仍为 `full_task_accepted=false`。
UE 实拍状态以同目录 `scenes-v8/verification.json` 为准，图集入口为 `review.html`。
wide-v7 虽通过几何和保持，但实际总览中目标盒被操作臂明显遮挡，保留为历史。
本轮只在本地执行，未推送、未合并或验证 Linux。具体变化及失败候选见根目录横置宽箱面归档。

上一版修正候选为 `preserved-face-v3`（2026-11-02）：两星箱体各有完整任务盒，
操作星任务面水平、目标星任务面竖直；目标星壳体扩大，原 WF10 接口及原靶标安装坐标保持不变。
本版修正了自由插头和螺栓的初始变换顺序，几何与自由件落座复审、2 秒原生保持独立核验已通过，
八个配置已登记，`full_task_accepted=false`。共享初态已实际加载并取得四路 UE 原图，来源核验 1/1 通过；
总览及主视口仍有裁切、遮挡，不能标记全部视角满足要求，完整八项双臂任务尚未复验。
v2 虽通过箱体、任务盒与原接口审查，仍因自由件未随目标最终位姿转换而被拒绝。
该历史版本冻结索引为其输出根目录的 `installed-index-v3.json`，修订名为 `preserved-face-v3-candidate`。
此前网页索引 `hull-l-layout-v4-candidate` 及其 4 路 UE 原图作为独立历史保存；
不能将这些截图或早期 14 项成功记录沿用于本版候选。当前运行入口以平台实际索引和对应回执为准。

## 原接口与固定靶标的上一版布局候选（2026-11-02）

本地候选目录为 `D:/mingti/outputs/hull-task-box-native-20261102/preserved-face-v3`。
`tools/diagnostics/build_preserved_face_scene.py` 使用真实源模型、v4 双臂安装及初态构建，
`simulation/scene_library/preserved_target.py` 对可变目标件绕目标本体坐标
`(0.28865, 0.04975, 0.04207870013) m` 等比放大 5.6 倍。
原 `+X` 侧壁和 `+Y` 保护面的基准平面不平移；这是保留接口的壳体修正，不是整星全部零件统一放大。

原 WF10 视觉件和 898 个碰撞件，以及原前板与外侧附件，共 909 个 geom 的原属性、
mesh 和安装位姿保持不变。原前板不再叠加一份放大模型；保护区域外以四块薄封板补齐扩展前面，
封板最前端不超过原 `Y=0.04975 m`。原四块 5 cm 靶标仍使用 `target_aruco_5cm` 的
260 个原始渲染几何及 `fixed_body_geometry`，不跟随外壳缩放。
它们在目标本体内的中心依次为 `(0.10,0.0578,0.15)`、`(0.25,0.0578,0.15)`、
`(0.10,0.0578,-0.13)`、`(0.25,0.0578,-0.13) m`；整星移动会改变世界坐标，安装坐标保持原值。
本轮不验收对接视觉算法，`fiducial_calibration_valid=false`。

目标任务盒采用总深 102 mm、壁厚 2 mm 的一体中空后壳，背面直接贴目标星 `+X` 箱体侧壁，
没有悬置支杆。删除原侧面实例 0045、0046、0048、0049、0050 五个挡物，变更名单保存在
`assembly.json`。原五个有效箱体碰撞代理按实际壳体重建，保留接口窗口和全部 WF10 碰撞；
另更新原已禁用的调试包围盒，不通过关闭有效碰撞绕过安装检查。
原声明总质量仍为 10 kg，惯量按 5.6² 放大；这是明确记录的工程假设，不是重新计算的 CAD 质量属性。

两任务面的操作星坐标仍为：本星中心 `(0.35,0.34,0)`、法向 `+Z`，
目标中心 `(0.74025,0.49,0.225)`、法向 `-X`。目标根位置改为
`(1.1309,0.25975,0.17792129987) m`，姿态为 `wxyz=(0,0,0,1)`。
操作臂安装、v4 操作初始关节角及所有相机属性保持不变；仅捕获臂为新星体位置重求保持姿态。
目标仍是独立自由刚体，经捕获臂末端理想连接保持，未装到操作星甲板上。

历史 v2 的 `preserved-geometry.json` 记录上述保留合同全部通过且差异为零。
其 `mount-audit.json` 记录后壳完整贴壁、侧壁上下安装余量各约 5 mm、抽屉最内收后仍有约 5 mm 后隙，
无新增任务盒穿插或靶标遮挡；原接口初态最深接触约 0.318 mm，小于 1 mm 检查上限。
原靶标白边与旧前板筋的历史相交单列保存，没有被改写为本轮新增问题或无相交。
随后独立复核发现 v2 在安装自由件之后才设置目标最终位置，使世界自由刚体没有随其供体槽位移动，
所以这些局部通过结果不足以接受 v2 初态。v3 已改为先设置目标最终位姿、再安装任务盒及自由件，
本版 `mount-audit-final.json` 已独立通过新增的自由件相对槽位位置/姿态检查及完整机构包络检查，
`preserved-geometry.json` 也重新通过，未直接沿用 v2 结论。
`hold-validation.json` 绑定本版模型和初态，2 秒原生理想锁定保持通过：最大抓取位置误差约 2.48 µm、
姿态误差约 0.0001724 rad、目标相对漂移约 21.40 µm，预期接口最大穿入约 0.3184 mm；
无禁止接触或目标/操作星硬件碰撞。短时保持不等于仅摩擦抓持或完整作业通过。

八项配置已注册并完成来源清单复算，冻结索引为输出根目录的 `installed-index-v3.json`。
双击同目录 `Open-Scene-Library.cmd` 启动已有网页仿真入口；`使用说明.txt` 给出四基础场景与任务选择步骤。
`review.html` 展示本版构型、证据和实际 UE 图，`scenes/verification.json` 来源核验为 1/1。
本次实拍第 4 类航空插头这一共享初态的四路视角，运行实例为 `scene-20261102-172919-299337d3`，
不将其写成八次独立场景实拍。原相机参数保持不变：总览能看到 L 型双任务面，但目标盒上缘裁切，
操作臂遮挡本星任务盒右侧；操作腕部能看抓取和任务局部，捕获臂末端被近处原接口结构占据。
主视口目标星上下也有裁切，画中画遮住部分区域。实际加载和图像来源通过，完整视野覆盖仍需调整布局后复核。

补充的实际自由观察帧为 [整体双臂](D:/mingti/outputs/hull-task-box-native-20261102/freeviews-v4/free-assembly.png)
和 [原接口及四靶标区域](D:/mingti/outputs/hull-task-box-native-20261102/freeviews-v4/free-interface.png)，
记录见 [freeview-inspection.json](D:/mingti/outputs/hull-task-box-native-20261102/freeviews-v4/freeview-inspection.json)。
它们临时改变主视口自由观察视图，拍摄后已恢复主视口，未修改任何安装相机；
图中仍保留右侧捕获夹爪和 SARM 腕部画中画。辅助自由视图与原相机截图分开展示，不能替代原机位覆盖验收。
首次 `freeviews` 因 keepalive 断开回到默认视图，`freeviews-v2/v3` 为构图诊断，均不纳入交付图集。

本轮只在本地构建，尚未推送 GitHub 或合并、重建验证 Linux。
失败的 `preserved-face-v1` 和 `build-v1.log` 保留：首次写入遇已有 `initial-scene.json`，
独占创建以 `FileExistsError` 拒绝覆盖，不能作为完整构建或注册成功记录。
详细来源与边界见根目录 `archive/2026_11_02_原接口与固定靶标的双任务盒实模修正.md`。

## 双臂初态更正（2026-11-01）

第 2、4 类的“两星已连接”指：操作星上的对接臂已经抓住目标星并将其拉近，
在操作臂作业期间持续保持；目标星是独立刚体，不放在操作星甲板上。
当前范围版本为 `local-phase-one/3-dual-arm-preheld`。旧甲板装配的运行记录保留，
但不再满足第 2、4 类的当前验收，旧版 14/14 不能作为双臂场景完成证据。

`tools/diagnostics/build_preheld_dual_scene.py` 生成独立目标星、双臂和任务盒的候选初态；
`--cross` 将接收端放在操作星上，供体留在目标星。默认目标星位置为
`[0.45, -0.55, -0.12] m`。当前使用夹爪与目标星之间的理想锁定约束和正常关节伺服，
保持约束没有连到操作星本体；这不代表摩擦抓持或物理捕获验收。
`tools/diagnostics/verify_preheld_capture.py` 对实际状态和接触记录独立检查保持误差、漂移和对接臂干涉。
完整操作仍须通过任务验收，并绑定同一模型的保持检查。

```text
python tools/diagnostics/build_preheld_dual_scene.py --reference /output/capture-reference.json --cross --output /output/cross-preheld
python tools/diagnostics/verify_preheld_capture.py --model /output/cross-preheld/preheld-dual.xml --scene /output/cross-preheld/initial-scene.json --native /output/native-run --output /output/hold-check.json
```

省略 `--cross` 为第 2 类目标星作业候选。操作星保留原平台壳体，目标星复制该壳体
并安装作业模块；新增壳体质量来自原网格（当前 12.034 kg），在装配报告中显式记录。
保持报告绑定同一次完整运行的状态与接触文件指纹，短时保持通过不能替代全程检查。

2026-11-02 复核发现：旧 `candidate-03` 使用的保持力模型将供体反作用力施加到操作星，
不能作为正确双臂受力链的验收。原始记录保留，新验收要求 `plug-retention/3-moving-owner`：
供体反力作用于目标星，接收端反力作用于实际安装星；同一插头不能同时由两个静摩擦通道保持。
拔出控制沿供体的实时轴线，插入和评分使用接收孔的实时坐标。

第 2、4 类构建入口现使用双臂候选，需要显式传入已保存的 `--capture-reference`。
默认操作臂安装平移 `[0.2,-0.2,0.25] m`，新增支撑质量和尺寸在装配报告中记录。
候选采用较硬的末端理想锁定约束，保持伺服为 1500/80，执行器上限仍为 80 Nm。
靶星航空及导向插头预设使用已通过全程保持核验的 2500/80；增益随初态清单保存。
较高的 5000/160 增益在实跑中出现抖动，已拒绝使用。
可拆离抽屉及跨星抽屉搬运继续暂缓。

`audit_preheld_scene.py` 在实际初态显式检查目标星外壳与任务硬件、两星固定硬件的距离；
`verify_preheld_capture.py` 还检查完整原生轨迹中的保持误差、非预期接触和保持力归属。
两项与任务独立评分都通过后才可进入当前限定范围的已验收库。抽屉可用
`--seating-overtravel .0016` 调整推回参考预压，仍保留 2 mm 关闭公差、40 N / 4 Nm 保护。
双臂圆柱插头预设使用 `--release-speed .02` 减小松爪扰动，阶段时限覆盖实际张开行程，
松爪后的 0.03 rad/s 转速门槛保持原值。`compose-preheld-operation` 和完成记录绑定均校验该参数。
跨星接收板沿用保留的本星任务盒框架与原始安装位置，不再增加侵入航空插头通道的多余支柱。

基线是本地 `benchmark-asset-library` 的 `6de2848`，叠加工作区中未提交的
36 个源码修复。复制清单和 SHA256 位于根目录 `docs/local-scene-baseline.json`。
未用远端 main 覆盖本地功能，未推送。新增代码使用 pathlib、配置路径和子进程
参数数组；Windows 原生 DLL 验证不能代替 Linux 的重新构建与验收。

## 范围

原图顺序保留为八类、27 个任务条目。第一阶段包括本星操作、对接后操作靶星、
交会、跨星转移，合计 15 个条目。插头子类型和星体变体在同一条目内继续扩展。
阶段二的抓取观测、双臂收纳拼接、剪切、加注目前仅登记需求。

用户已确认第 2、4 类从“两星已连接”的初态验收操作；可拆离抽屉及其跨星搬运
暂缓。因此当前第一阶段必验 14 项，原图的 27 项目录与 15 项初始分组仍完整保留。
`status.phase_one` 按这个范围汇总，不能用它声称完成物理对接；旧回执不改写。

| 图中部分 | 子任务 | 复用的场景组成 | 阶段 |
| --- | --- | --- | --- |
| 1 本星操作 | 抽屉回合、航空插头、导向插头、圆柱插头、杆状螺丝回合 | 操作星、单臂、本星任务盒 | 1 |
| 2 靶星操作 | 对接臂保持期间执行上述五类任务 | 操作星、对接臂、操作臂、独立靶星、靶星任务盒 | 1 |
| 3 交会 | 交会目标卫星 | 操作星、靶星、相对导航和飞行控制 | 1 |
| 4 跨星转移 | 三类插头从靶星拔出并插入操作星；抽屉搬运暂缓 | 两星、对接臂持续保持、操作臂转运、供体、接收端 | 1 |
| 5 抓取与观测 | 太阳翼、漂浮螺栓/插头、漂浮抽屉、光敏/天线、石头、手眼拍摄 | 操作星、单臂、目标物、手眼相机 | 2 |
| 6 双臂 | 螺栓/插头收纳、光敏/天线收纳、石头收纳、一臂持工件另一臂持底座拼接 | 操作星、双臂、工件、带盖收纳箱或配合底座 | 2 |
| 7 剪切 | 剪断靶星线缆 | 两星、机械臂、剪刀、线缆 | 2 |
| 8 加注 | 加油头加注 | 两星、机械臂、加注头、接口及流体状态模型 | 2 |

“螺栓/插头”等仍按原图分组，不把一个组内的每种物体误算成已实现的独立任务。
模型外形变体先登记资产，装配锚点、碰撞体、质量惯量、运动学和验收必须分别复核。

`specified` 是任务规格，`native_runner_available` 表示存在运行适配器，
`accepted` 只能来自实际运行记录。家族内全部子任务完成才算该家族完成。
远距交会的成功只覆盖其记录的起点和保持终点，不包含近距视觉或物理对接。
可继续使用已有近距模块承接远距运行的实际 handoff：

```text
python -m simulation.near_field.run --handoff /output/flight-run/native/handoff.json --output-dir /output/near-field
python tools/diagnostics/verify_near_field.py --run /output/near-field --output /output/near-field-independent.json
```

近距独立核验与远距回执分别保存；当前近距视觉是参数化观测，捕获是逻辑代理，
不能据此声称图像算法或实体锁扣已经验证。运行时长须覆盖该输入 handoff 对应的接近过程。

## 脚本入口

构建当前第一阶段的 14 个任务实例（需要现有 Basilisk 环境和独立几何 Python 环境）：

```text
python tools/diagnostics/scene_library.py build-phase-one --geometry-python /path/to/geometry/python --capture-reference /output/capture-reference.json --output /output/phase-one
python tools/diagnostics/scene_library.py run-library /output/phase-one/library.json --geometry-python /path/to/geometry/python --output /output/phase-one-runs
python tools/diagnostics/scene_library.py status /output/phase-one-runs
```

`--native-python` 可指定 Basilisk 解释器；重复 `--scene-id` 可只构建选定任务。
构建会创建靶星作业、跨星转运两种双臂装配候选，依次做几何检查和原生初态搜索，再生成绑定来源的实例；
构建成功不等于任务成功。`run-library` 校验清单指纹后顺序运行，保留各项失败记录。
预设是待验收候选，只有绑定实际通过回执的实例才算已验证。在 Linux 上使用相同命令
重新构建，不能直接将 Windows 的绝对路径或 DLL 当成 Linux 运行产物。

全部必验任务通过后，可以收集对应的精确实例和证据指纹：

```text
python tools/diagnostics/scene_library.py export-verified /output/all-runs --output /output/verified-library
python tools/diagnostics/scene_library.py run-library /output/verified-library/library.json --geometry-python /path/to/geometry/python --output /output/repeat-runs
```

导出在缺项时拒绝执行，后续发现安装穿插的历史成功也不会被选入。
`phase_one_accepted` 表示已记录的限定范围验收；重跑仍产生新的原生报告。

在 `space_sim_server` 目录，用现有可运行 Basilisk 的 Python：

```text
python tools/diagnostics/scene_library.py list --phase 1
python tools/diagnostics/scene_library.py assets
python tools/diagnostics/scene_library.py compose 01_self_operation/drawer_cycle --seed 42 --output /output/drawer.json
python tools/diagnostics/scene_library.py validate /output/drawer.json
python tools/diagnostics/scene_library.py run /output/drawer.json --output /output/drawer-run
python tools/diagnostics/scene_library.py compose 03_rendezvous/rendezvous --seed 7 --output /output/flight.json
python tools/diagnostics/scene_library.py run /output/flight.json --output /output/flight-run
python tools/diagnostics/scene_library.py status /output
python tools/diagnostics/scene_library.py batch /output/drawer.json /output/flight.json --output /output/batch
```

路径示例需替换为本机目录。`run --python` 可指定现有解释器。不要混用 Python
MuJoCo 与 Basilisk 的 DLL 进程；几何诊断在独立 Python MuJoCo 进程执行。
原生 helper 与 SARM 插件必须与本工作区源码和 MuJoCo ABI 匹配。
`batch` 顺序运行明确提供的实例，失败项保留并继续下一项；开始前校验全部文件，
每项使用独立目录。批次结束分别报告指定范围是否通过和完整场景是否通过。
当前 seed 不等于任务盒位置随机化；默认是固定布局，随机布局的完整验收尚未接入。

输出目录必须不存在。实例保存模型及资产实际 SHA256；运行前重新检查依赖，
运行保存源码哈希、原始命令、初态、stdout/stderr 和原生评分。拒绝覆盖结果。
新运行额外记录 JSON 运行配置、原生 helper 和插件二进制指纹；历史输出保持原样。
40 N/4 Nm 及原有连续超限保护不变。几何编译、脚本结束、独立物理评分分别记录。

## 历史甲板装配开发模型（不满足当前双臂范围）

以下命令仅保留旧模型的复现能力，不能用于当前第 2、4 类的构建与验收。

```text
python tools/diagnostics/scene_library.py build-mounted-fixture --scale 1.5 --deck-raise .45 --output /output/mounted
python tools/diagnostics/scene_library.py build-mounted-fixture --cross-transfer --output /output/transfer
```

这些是已经刚性连接的开发初态：复用真实星体模型，旋转宽面后安装原尺寸任务盒，
复制独立星体网格，默认按恒密度计算质量 s³ 与惯量 s⁵。星体缩放不改变插接尺寸、
夹爪或任务盒。源模型保持不变。每个目录保存 XML、assembly.json 和初态配置。
原自由目标停放在 5 m，工作目标为 `docked_target`，不得将二者混淆。

当前安装变体按主体而非太阳翼的包围尺寸布置，新增四个支撑柱；外翼初始折叠
180°并保留被动铰链。仅将机械臂 `link1` 安装根平移 X=-0.1 m；靶星操作
默认抬高 0.25 m，跨星转移候选默认抬高 0.08 m；支座标称质量分别为
1.25 kg 与 0.4 kg。上述均为记录在 assembly.json 中的工程装配参数，
需要逐个变体重新验证，不是硬件测量结论。
原 `base_link` 网格包含操作星外壳，必须保持其源坐标，不能随臂座移动。
可用 `--arm-mount-offset X Y Z` 设置支座位置；支座高度跟随 Z，质量按同一
标称密度随高度变化。安装高度和机械臂位置是候选参数，不自动成为已验收默认值。

跨星候选把接收板和精密孔碰撞体共同归入 `service_receiver_mount`，供体归入
`docked_target`。这是装配候选，仍需验证接近、力学、安装支撑、完整路径与接收状态。
接收板移至操作星的独立侧面安装区（相对原位置 X=-0.655 m，安装高度 0.15 m），
配套灰色螺栓与接收板一起移动；供体仍留在靶星，不把同一位置换名字当作跨星运输。
`--receiver-shift-xy X Y` 可整体调整接收板、精密孔、螺栓初始位置和安装柱，
不得单独挪动孔位或忽略螺栓碰撞。螺栓保持原自由度，运行中仍可能漂移。
当前抽屉仍为滑动机构，不能称为完成可拆离抽屉的跨星搬运。

先在独立几何 Python 中运行显式安装检查：

```text
python tools/diagnostics/audit_scene_mount.py --model /output/mounted/mounted-target.xml --output /output/mounted-inspection.json
python tools/diagnostics/audit_scene_mount.py --model /output/transfer/mounted-target.xml --output /output/transfer-inspection.json
```

再用 Basilisk Python 调用统一脚本入口：

```text
python tools/diagnostics/prepare_scene_initial_pose.py --model /output/mounted/mounted-target.xml --scene /output/mounted/initial-scene.json --output /output/mounted-pose
python tools/diagnostics/scene_library.py compose-mounted-operation 02_docked_operation/drawer_cycle --assembly /output/mounted/assembly.json --initial-scene /output/mounted-pose/initial-scene.json --mount-inspection /output/mounted-inspection.json --output /output/mounted-operation.json
python tools/diagnostics/scene_library.py run /output/mounted-operation.json --output /output/mounted-run
python tools/diagnostics/scene_library.py compose-mounted-operation 04_cross_transfer/aviation_plug_insert --assembly /output/transfer/assembly.json --initial-scene /output/transfer-pose/initial-scene.json --mount-inspection /output/transfer-inspection.json --transport-clearance .16 --output /output/transfer-operation.json
```

跨星初态同样先对 transfer 模型运行初态搜索。实例同时校验装配文件、模型全部资产、
初态来源、完整覆盖操作星外壳的安装检查和运行参数；改变任一文件后必须创建新实例。
缺少外壳组、存在穿插或模型哈希不匹配时拒绝实例。操作通过仅记录
`operation_accepted=true`，完整场景 `accepted` 仍为 false，退出码 0 表示该次
已明确限定范围的操作通过。`status` 保留这个区别。可拆离抽屉仍不接受运行，
不能借用普通抽屉回合替代。

杆状螺丝已接入候选机械臂控制与独立评分，可用同一 `compose/run` 入口调用
`01_self_operation/locking_rod_cycle`，或用 `compose-mounted-operation` 创建靶星变体。
运行先调用 `prepare_locking_rod_path.py`，保存私有原生几何中的双侧夹持、放倒、
复位、旋紧、松爪和撤离路径；预检失败不进入动力学。控制器只发送八个关节参考，
不对杆施加演示力矩，不重写关节状态。独立评分读取实际杆角、螺母行程、螺旋关系、
双侧接触、锁钩法向力、释放后的保持与撤离，以及 40 N/4 Nm 保护。
路径预检通过不代表该控制器已经通过完整任务验收，以原生 acceptance 为准。
当前杆回合使用源模型允许范围内的 -0.5 rad 摆角，独立评分要求实际达到
-0.45 rad 以下后复位；这是当前模型的摆动回合，不是 90° 折平或硬件行程认证。
锁紧还要求螺母行程 4–7 mm、锁钩法向力大于 1 N，并在松爪撤离后保持 2 s。

插头可用 `--part-velocity-damping 1` 显式启用轴向速度阻尼，用 50 ms 一阶滤波的
实际轴向速度抑制插入和落座时摩擦释放后的过冲，拔出与姿态控制保持原控制。
旧的六轴速度反馈试验发生接触振荡和超时，已保留失败记录；轴向候选标记为
`plug-insert/4-axial-damping`。当前靶星导向插头装配在增益 1 下已通过原生验收，
这属于旧甲板装配试验；双臂预设默认未启用。实例和原生报告记录参数，孔位、插入
成功容差、法兰间隙要求和腕部保护保持原值，需单独验证该控制变体。
`--transfer-offset-xy X Y` 只偏移高位运输的中间航点（各方向限于 ±0.25 m），
再经过原接近孔位进入插入。它不移动最终孔位或物体，也不取消漂移物体的碰撞。
启用以上候选控制参数时，控制器版本标为 `plug-insert/3-scene-variants`。
`--release-speed .01` 可降低夹爪的实际张开速度，减小松爪时的扰动；默认仍为
0.05 rad/s。该参数不改变松爪后的速度、几何和保持时间验收要求。
旧固定轨迹仅在新增选项均为未启用的默认值时兼容回放，不能把非默认控制参数
混入冻结配置；原模型、资产、初态和安全配置的匹配检查仍然保留。

安装高度改变后，先用 `tools/diagnostics/prepare_scene_initial_pose.py` 搜索并保存无碰撞初态，
再传入其输出的 `initial-scene.json`。原本星关节初态不保证适用于抬高后的任务盒。
插头可显式指定 `--transport-clearance`；其含义是相对最高配合面的运输净高，
不能通过此参数改变插入深度、成功阈值或腕部载荷限制。

预连接操作符合用户确认的第一阶段范围，但不能替代完整物理对接与连续交接。当前近距模块
仍含逻辑捕获代理；缺少能力明确保留在目录中，不能通过放宽阈值或改写成功布尔值补足。

## 历史第一阶段完成记录（2026-11-02，v4 布局变更前）

以下是旧双臂库的独立历史结果。当前第 2、4 类已更换为箱体双任务盒 L 型 v4，
原八项操作通过记录不构成新模型验收；本节路径和回执保留用于追溯。

- 已完成 14 个指定实例：本星 5 项、双臂靶星 5 项、远距交会保持 1 项、跨星插头 3 项。
- 八项双臂操作完整任务、接触与抓持保持检查通过；目标星最大相对漂移 4.490 mm，小于原 5 mm 阈值。
- 本星五项复用已核验的本地成功证据；本轮新跑八项双臂及完整交会。125 项相关回归和 38 项 UE 自动化测试通过。
- 14 项均已接入本地 UE 实际轨迹回放；画面抽查、对象数量与哈希核对单独记录，不将回放当作新的动力学验收。
- 已验收库：`D:/mingti/outputs/scene-library-dual-preheld-20261102/verified-library-v2/library.json`。
- 本地入口：`D:/mingti/outputs/scene-library-dual-preheld-20261102/Open-UE-Scene-Library.cmd`；画面证据见同目录 `ue/visual-acceptance.json`。
- 本结果针对库内记录的模型和初态，不覆盖全部资产排列组合或随机布局。
- 暂缓：可拆离抽屉及跨星搬运。保留本星、靶星抽屉回合。
- 后续阶段：物理对接与操作连续交接。
- Linux 分支集成和重建验证待后续执行；当前未推送远端。

本文件说明接口与验收边界；实时运行状态以实际 receipt/acceptance 文件为准。
`status` 也列出直接运行原生验证器产生的操作报告，保存模型哈希和失败原因；
这些单项记录不会自动提升完整场景状态。
汇总按模型 SHA256 关联安装检查；同一模型后续被发现穿插时，历史操作原始结果
保留，但 `operation_with_mount_check_passed` 为 false，不能选作通过安装检查的场景。
缺少最终回执的输出列入 `incomplete_runs`，可查看最后阶段；它可能仍在运行或曾被
中断，汇总不会仅凭日志存在就声称进程存活或任务通过。

## 独立几何预览

`tools/diagnostics/preview_scene_model.py` 必须在独立 Python MuJoCo 环境运行。支持初态或
原生状态轨迹、指定时刻、以操作星为中心的相机。轨迹须携带相同模型哈希的
原生 summary。`--role-colors --diagnostic-lighting` 仅修改诊断图的着色和灯光，
并写入预览元数据；交付 UE 手腕灯强度直接用于 MuJoCo 会过曝，不能把预览
灯光当作视觉基准配置。预览不代替全轨迹接触、安装结构或 UE 渲染验收。
`tools/diagnostics/audit_scene_mount.py` 额外检查星体、任务硬件、接收板和操作星之间的显式
几何距离，即使它们属于刚性连接而被引擎常规碰撞过滤跳过；结果仍不等于
支座承力能力或完整装配验收。
## 导入完整网页仿真系统（2026-11-02）

正式场景入口是现有 Space Arm 网页工作台的“场景模板”菜单，名称以“场景库 · ”开头。
它通过原有 scene/mission 管理器启动原生动力学和 UE 实时渲染；下面的独立轨迹回放入口不等同于导入系统。

用户入口按 4 个基础场景组织：本星作业、双臂保持靶星作业、交会、双臂跨星转运。
选择基础场景后，再选“任务 / 初始化配置”，数量分别为 5、5、1、3。
14 项是任务配置与验收组合，不是 14 套独立平台。第 2、4 类的八项配置复用同一双臂、双完整任务盒 MJCF，按任务选择目的孔和控制配方；目标星保留原机构名，操作星机构加 `service_` 前缀。当前已登记 `preserved-face-v3-candidate`，初态安装、2 秒保持及共享初态 UE 来源核验通过，视角覆盖仍有裁切遮挡、完整八项作业未复验；v4 索引和回执作为历史保留。
已有 `library/<分类>/<任务>` ID 保留为脚本/API 的直接配置入口；网页不会把它们全部展开到基础场景菜单。

`tools/diagnostics/import_platform_scene_library.py --library <已验收库/library.json> --replay-root <已有资产目录索引> --output run/imported_scene_library.json`
将前四类 14 个固定实例注册为平台模板。`--replay-root` 仅复用哈希匹配的资产 catalog，不启动回放。
导入时逐一比对相机名称、父刚体及全部 MJCF 相机属性；启动时校验模型依赖、相机基线、来源实例和回执哈希。
本机输出路径不硬编码进运行模块；迁移 Linux 时须重新安装资产和生成索引，目前尚未进行 Linux 验证。

首次导入的模型与初始状态使用已验收实例；后续布局变更必须另行登记候选，不能沿用原任务验收结论。本星 5 项、双臂靶星 5 项和跨星 3 项从固定初态启动；交会项进入原系统的实时飞行任务。
双臂初态中，抓取臂保持独立目标星，操作臂作业。抓持使用已连接约束，不代表本轮验证了物理捕获和对接。
此阶段不包括可拆离抽屉转运。

系统相机保持原安装位姿和视场角：本星总览与操作臂腕部；双臂场景另有抓取臂末端及原配置的双目定位相机。
第 2、4 类已完成对接，不以对接双目作为视角验收要求，截图工具也不再获取其双目图像。
主视口使用系统原有观察视图，不作为安装相机标定依据。若某个原视角遮挡或覆盖不到任务区，应记录为布局适配问题，不能移动相机后宣称原视角满足要求。

`tools/diagnostics/inspect_platform_scene_library.py --output <本地截图目录>` 通过真实网页 API 逐一启动场景，等待原生状态和当前场景标识匹配，
再调用 `scripts/inspect_platform_cameras.mjs` 保存工作台截图和实际 UE WebRTC 帧。每个视角确认连接和新解码帧，保存对应状态与相机配置。
截图运行检查的是完整系统加载和视角覆盖，不重新宣称所有任务轨迹完成。原有任务动力学证据分别保留。

此前第 2、4 类曾登记总览适配候选：目标星中心在操作星坐标系约 `(0.80, 0.15, 0.00) m`，相对旧姿态偏航 180°；相机与机械臂安装位置不变，重新求解双臂初态。其后登记了下面的箱体双任务盒 v4；本轮保留原接口的新候选见本文件开头。
`tools/diagnostics/reorient_preheld_scene.py` 生成版本化模型，`tools/diagnostics/register_layout_candidate.py` 在保留原索引的前提下替换任务初始化，明确记录 `full_task_accepted=false`。
该历史布局的静态装配及预抓取位检查通过，正式 UE 总览可见两星作业区，但目标星本体仍有部分遮挡；这些截图不代表后续 v4 或本轮候选的显示结果。
详见根目录 `archive/2026_11_02_已对接双臂场景总览覆盖调整.md`。截图校验可用 `--index <该批截图的索引快照>`，避免用新布局索引解释旧截图。

历史 v4 使用 `tools/diagnostics/build_l_taskbox_scene.py` 构建的完整双任务盒 L 型装配：操作星任务面中心
`(0.35, 0.34, 0.00) m`、法向 `+Z`；目标星任务面中心 `(0.74025, 0.49, 0.225) m`、法向 `-X`。
以上均为操作星坐标。目标星根位置为 `(1.09, -0.0527, 0.10) m`；相对 v3 仅沿 `+Y` 平移 0.15 m，
目标星姿态、操作臂初始关节角及全部相机属性保持不变。捕获臂重新求解 IK，继续通过既有连接约束保持独立目标星；未采用 `parked` 停放初态。
目标星和太阳翼整体等比放大 3 倍，任务机构保持原尺寸；任务盒均安装在箱体，目标星未刚性安装到操作星上。

第 2 类插头使用目标星原接收孔，第 4 类使用操作星 `service_*_insert_40p5` 孔。
原三类自由插头初始位于目标星，操作星相应插头位为空；两星完整任务盒结构始终保留。
本地八项登记位于 `D:/mingti/outputs/hull-task-box-20261102/registrations-v4`。
`l-layout-v4/change-contract.json` 记录上述不变项；同目录 `fixed-arm-check.json` 的固定操作臂与张指走廊检查通过，
`mount-audit.json` 的显式初态几何检查通过。它们不覆盖完整任务轨迹、动态保持或摩擦抓持。
v4 的正式网页与实际相机截图保存在 `D:/mingti/outputs/hull-task-box-20261102/scenes-v4`，
冻结索引 `installed-index-v4.json` 对应的实际导入与截图核验为 1/1：第 4 类航空插头初态，4 路 UE 原图及对应工作台截图。
本次实拍的是八项配置共用的初始化模型，并非八次独立运行。总览中目标任务面已避开操作臂，
操作星任务盒右侧仍有原操作臂遮挡；操作腕部是局部视图，对接腕部主要被近处抓持结构占据。
图集和启动说明见 `D:/mingti/outputs/hull-task-box-20261102/review.html`，不标记全部视角覆盖或完整任务通过。
v1、v2 的失败记录及 v3 历史保留，详细变更见根目录 `archive/2026_11_02_双星箱体任务盒L型初态.md`。

## 在 UE 中观看原生轨迹（独立回放）

`tools/diagnostics/view_scene_library.py --config <本机配置.json> --scene-id 04_cross_transfer/guide_plug_insert`
打开已验收轨迹的 UE 回放；省略 `--scene-id` 显示任务菜单，`--rate 2` 使用两倍回放速度。
默认操作任务为原速，5120 秒的交会过程为 20 倍速；显式 `--rate` 可覆盖，范围为 0.1–20。
第一次选择时转换保存的 qpos，以后校验哈希后复用缓存。C 切换自由相机，W/S、A/D、Q/E 移动，鼠标转向，Esc/Home 回到全景。
切换场景前关闭上一个 UE 窗口。它不重新运行物理求解，也不接收机械臂控制。
第 3 类使用飞行求解时同步生成的 `team_rendezvous.bskrec`，须通过 `verify_team_recording.py`
的实际位置核验并绑定录制、原始遥测和核验文件哈希；只有位置的旧遥测不能伪造完整姿态回放。
在运行交会时传入 `--catalog <本地资产目录.json>` 可同步录制完整姿态。

本机配置需要 `library`、`output`、`project`、`editor`、`catalogs`、`scene_config`、`derived_data_cache` 字段。
`scene_configs` 可按场景 ID 覆盖机位配置，适配本星、双臂及远距交会视图。
必须在独立的 Python MuJoCo 几何环境中运行，禁止与 Basilisk 原生求解进程混用。
UE 资产通过源文件哈希重绑定；缺失资产会报错，不会静默替换成方块。生成文件和安装路径配置放在输出目录，不提交 Git。
