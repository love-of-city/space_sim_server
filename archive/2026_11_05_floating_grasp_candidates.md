# 漂浮物体候选抓法

在同一冻结场景中比较物体局部抓取点与方向。19类物体共104个候选，支持指定候选、参考抓法优先排名与seed随机选择。`floating-release/4`先采样物体参考位置和朝向，`floating-grasp/1`独立冻结候选、IK、检查结果及模型/场景身份。

`server/simulation/scripted_tasks/floating_grasps.py`维护候选与选择；原生预检涵盖张开接近、闭合双侧接触、连续IK和携物回收路径。静态接触不对称仅作警告与排名惩罚，真实稳定性由动力学验收判断。控制器`floating-capture/8`增加pregrasp阶段，保持2秒、回收与到位稳定1秒的成功门槛保持。

三个入口collect/record/run均使用冻结选择，每帧保存候选ID。场景JSON或私有批量YAML的scene可加入：

```yaml
floating_grasp:
  selection: random
  candidate_ids: [baseline, shallow, deep]
```

默认ranked优先无不对称警告的参考抓法。原参考抓法保留，抽屉新增两肩`(±.04,0,.025)`候选。参考轴角范围0–180°、默认8°；固定参考旋转轴名改为`reference_z`。历史episode保持原样。

## 验证结果

最终采集源码`cb1e2cd`。原生验证包含首轮`e5499d8`的15次尝试和修订后`cb1e2cd`的10次尝试，合计25次、21次成功，覆盖19类。六条正式采集全部成功，共14229帧，均通过原生recording validator、LeRobot v3回读、两路相机实图及网页列表/详情核验。受影响测试集合204通过，2项本地缺少Basilisk跳过。

| 正式示例 | Episode | 帧数 |
|---|---|---:|
| 抽屉肩部抓取 | episode-20261105-121045-4fd36b34 | 2061 |
| 外侧太阳翼高位回收 | episode-20261105-121330-0bac47d4 | 2915 |
| 内侧太阳翼高位回收 | episode-20261105-121733-7d806ac9 | 2913 |
| 任务箱底座向外/高位回收 | episode-20261105-122101-b769589f | 2465 |
| 数传天线初始旋转90° | episode-20261105-121745-6b6978e4 | 1808 |
| 同类异色双螺栓、随机抓法 | episode-20261105-122024-1ddb0379 | 2067 |

抽屉与两块太阳翼本轮回收目的地`[.30,.35,.55]`，任务箱`[.30,.55,.55]`；公共默认`[.30,.35,.36]`保持。大物体需显式配置携带空间，本轮均用现有单臂成功。

首轮数传天线/shallow、安装板2/shallow未建立双侧接触，安装板1/edge-slide-+0.04和圆柱插头/shallow回收失稳，恢复参考抓法后均成功。104个注册候选并非全部动态成功；数传天线的薄边抓法仍无成功证据。

证据目录：03 `/home/glx/data/spacesim/validation/candidate-grasps-20261105/`；正式数据仍为`/home/glx/data/spacesim/episodes`，批次`2026-11-05_candidate_grasps`。方案、代码和关键修订经过独立reviewer审核。完整逐物体配置、负结果与研究分析由LabDesk研究目录的`data-diversity/14-候选抓法.md`维护。
