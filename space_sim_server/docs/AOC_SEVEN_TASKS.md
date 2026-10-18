# 个人姿轨控任务与统一仓库复现

文件名保留历史的AOC_SEVEN_TASKS。2026-10-18全量核对的任务表为94条、revision1058，其中本人直接或共同执行的10条：8条线上已完成，“合并三个任务模式”和“代码管理+仓库合并”仍为进行中。连续任务和共享GNSS的新实现及实际验收见[当前整合说明](../../archive/2026_10_17_连续任务与共享导航.md)。线上状态与源码交付分别记录，不以本地报告自动修改飞书。

以下任务表、独立GNSS参数和复现命令保留2026-10-10快照的历史口径，不作为最新线上状态或统一工作台默认参数。

本轮依据 [10/9 合作者迁移指南](https://lcnrp8hlo6f2.feishu.cn/docx/Yo0FdbIxWoO7gBxViPdcpqVenRg) 与 [10/8 会议原文](https://acndpm3fvl11.feishu.cn/docx/I1OPdBVJsorEAAxAsz6cfKvdnAh)：优先完成与 SpaceSim 的整合，加入大气阻力、太阳光压、GNSS 误差及未知轨道扰动。用户确认这两份规范足够。100 m 内自主接近与抓取不属于本人的当前任务；通信延迟按会议暂缓。

## 任务与实现

| 飞书记录 | 当前任务／线上状态 | 实现及应核验的证据 |
|---|---|---|
| recvuDDwIMmgvr | BSK 调研与官方 Demo／已完成 | 固定官方示例与 ISC 许可位于 `tools/vendor/`，`run_official_rendezvous.py` 生成报告和原始曲线 |
| recvuDxoc2CQT1 | 卡尔曼滤波＋PD／已完成 | `navigation.py` 的六维轨道估计、交会平移 PD、平台原生 MRP 反馈；双星滤波误差和执行器遥测 |
| recvuDyq2I8Wud | 飞轮动力／静态属性／已完成 | 团队 MJCF 原生三轴轮、惯量与电机；`test_attitude_control.py` 验证反作用、保护和动量 |
| recvuGFbfikK8K | 实际仿真飞轮驱动 Demo／已完成 | 三轴轮关节动力学、扰动恢复测试、原始轮速/力矩/姿态记录 |
| recvuDDwIMSVVc | SpaceSim 整合及缺失的大气物理／延期 | 共同 LocalMujocoStepper、标准组件与真实团队网格；UE 自动启动、网页操作、结果下载、连续交接到标准场景 |
| recvw5Z4qlhkrf | 300→500 km 升轨后交会／进行中，10/10 | `orbit-raise` 完整 11120 s；双星 GNSS、误差与外力、100 m 外保持、原始数据独立复核 |
| reczz28KuW3GWuBG | GNSS 误差＋随机轨道误差／进行中，10/15 | 每轴 3 m / 0.5 m/s 默认噪声、随机初态、独立相关物理扰动力；网页配置、种子复现、原始观测统计 |
| recvw6cgNrMWw5 | 代码共享管理／进行中，10/14 | 本人的 server＋adapter 配套变更迁入正式统一仓库同一分支；共享任务有 6 位参与者，不代表他人部分完成 |
| recvw5TEOevemU | 历史认领的 7 km→100 m 外任务 | 当前完整表不再返回该记录，不推测原因；功能保留，以 `in-orbit` 完整 5120 s 验证 |

[项目任务表](https://acndpm3fvl11.feishu.cn/wiki/KXyiw0KMgiWJGhktSKocw5EznAf)是线上进度来源。新成果不以历史“七项全部完成”的措辞代替验收。

## 算法输入输出

| 模块 | 输入 | 实际方法与输出 |
|---|---|---|
| GNSS | 双方组件质心真值，仅传感器生成器读取 | 2 Hz 独立六维观测；默认位置每轴 1σ=3 m，速度每轴 1σ=0.5 m/s |
| 轨道估计 | 观测、上拍状态/协方差、已知推力加速度 | 中心引力预测与雅可比，Joseph 协方差更新；双方 J2000 位置/速度及 6×6 协方差 |
| 参考 | 初始/目标轨道、持续更新的目标估计、时长和保持距离 | 五次平滑轨迹，连续位置/速度/加速度；目标未知扰动不会被当作已知前馈 |
| 平移控制 | 双星估计与参考、组合质量 | 重力/参考加速度前馈、位置比例 0.0004 s⁻²、速度微分 0.04 s⁻¹；质心处合力 ≤25 N |
| 姿态与轮分配 | 本体姿态/角速度、原模型惯量、轮轴与轮速 | BSK MRP 反馈 K=8、P=12，20 Hz；原生轮电机力矩、限矩与转速保护 |
| 环境 | BSK Earth/Sun、位置/速度、配置与独立随机流 | 指数大气阻力、含地影的球形光压、Gauss–Markov 未知加速度；通过共同物理端口真实施力 |
| 积分 | 所有刚体、引力及执行器力/力矩 | 团队 LocalMujocoStepper，默认 240 Hz 内部子步；MJScene 发布状态，不作第二次积分 |
| 交接 | 已接受结果、源模型指纹、双星/关节/飞轮/导航/环境状态 | 原始可动模型初态恢复、SPICE 历元连续、平台控制与双相机采集入口 |

官方 `scenarioRendezVous` 包含瞬时速度修改，保留为调研示例；当前有限推力验证使用独立实现。CW/Hill 和 Lambert 是调研背景，不冒称为本场景实际调用的算法。仿真动力学轮也不等于硬件在环；飞轮不提供净平移推力。姿态导航仍沿用平台真值假设。

默认保持 110 m 以应对新的 GNSS 误差。120 s 保持、±5 m 范围、Hill 速度 <0.2 m/s、600 s 捕获后跟踪 <5 m 等本轮预先固定的工程标准、全部参数和模型边界见 [TEAM_RENDEZVOUS.md](TEAM_RENDEZVOUS.md)。

## 云端复现

开发、编译、仿真和真实 UE 验证在云电脑运行，本地只保存源码与证据。正式团队目录为统一仓库下 `space_sim_server/` 与 `space_sim_UE_Adapter/`；不再使用两份独立仓库开发。环境按[根 README](../../README.md)配置，已验证 Python 3.13、bsk 2.11.1、MuJoCo 3.7.0。下面在 server 目录、使用正确解释器执行：

```powershell
python tools/run_aoc_acceptance.py --output-dir run/aoc_acceptance --catalog ../space_sim_UE_Adapter/Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_platform.catalog.json
# 对同一版本的已有结果重新独立复核，不再次运行完整轨道
python tools/run_aoc_acceptance.py --output-dir run/aoc_acceptance --verify-existing
```

该工具执行固定官方 Demo、原生/控制/环境/入口测试、两个完整场景、原始遥测复核、原模型恢复和渲染记录检查。每步失败返回非零，先清除旧成功标记。输出 `acceptance.json` 只覆盖列出的离线数值/记录证据；不能替代实际网页视频、控制与采集验证，也不证明共享管理任务中所有成员已完成。

独立检查工具如下：

```powershell
python tools/verify_team_flight.py <运行目录>
python tools/verify_team_handoff.py <运行目录>/handoff.json
python tools/verify_team_recording.py <运行目录>/team_rendezvous.bskrec
```

数值校核从原始数据重算距离、目标轨道、GNSS 统计、轮速与推力限值，检查源码/模型指纹以及共同物理核心。恢复工具独立加载原始可动模型并验证双星、关节和飞轮连续性。渲染记录检查网格绑定、Earth/Sun 与日照；真实 UE 外观仍需查看。

## 本轮证据位置与状态

统一仓库外的工作区 `validation/` 保存原始证据，避免把运行产物放进 Git：

- `unified-flight/in-orbit/`、`unified-flight/orbit-raise/`：默认种子 7 的全程数值、独立验证与交接恢复。
- `unified-browser/result.json`：种子 19 的真实登录、参数设置、启动/停止、两种完整场景、视频与下载；组合脚本后续读旧场景缓存的失败也保留在此。
- `unified-handoff-ui/result.json`：按当前实例匹配观测，实际点击交接按钮，验证标准场景、真实关节响应、双相机采集、重置和停止。
- `unified-native-final.xml`、`unified-component-tests.xml`：原生飞轮、环境、共同积分器、组件及入口测试。
- `unified-server-ci-interactive.xml`、`unified-adapter-python.xml`、`unified-web-tests-final.log`、`unified-signalling-tests.txt`：基础回归。
- `unified-ue-automation/index.json`：UE 自动化结果；真实图片位于 browser 证据目录。

各文件只有真实执行成功才具有通过意义；本轮最终结果由[根目录变更记录](../../archive/2026_10_10_交会完整整合.md)汇总。讲稿、汇报网页、认证与机器配置均不进入本次 Git 交付。GitHub 推送、团队主线合并和飞书更新遵循用户授权边界。
