# 王镱溥的交会任务与集成依据

本文件保留原路径以兼容已有链接，只维护当前职责、实现入口及完成条件。源码交付与飞书线上状态分别核验，不根据本地测试自动修改任务或周报。

## 任务来源

- [项目表](https://acndpm3fvl11.feishu.cn/wiki/KXyiw0KMgiWJGhktSKocw5EznAf)：最近核对 revision 1083，共 97 条；本人直接或共同执行 10 条，8 条标为已完成，2 条进行中。姿轨控与远程交会项目截止时间为 2026-10-31。
- [10/15 人工分工](https://applink.feishu.cn/client/chat/open?openChatId=oc_d18831213328163e9a80d6357f61bacc&position=112)与[差分 GNSS 补充](https://applink.feishu.cn/client/chat/open?openChatId=oc_d18831213328163e9a80d6357f61bacc&position=116)：负责远距交会、差分 GNSS、远近距传感器同步及仿真合并。
- [10/15 会议](https://acndpm3fvl11.feishu.cn/docx/Pou4dAxcrozvqqxVaYNcPydcnEH)及[原文](https://acndpm3fvl11.feishu.cn/docx/UgZqdB6fjoqERzxMNepcxeornkc)：恢复全局和手眼相机，统一 UI，验证大气阻力实际施力，与张怿心共同完成仓库合并。人工分工优先于智能纪要中的模糊归属。
- [10/8 原文](https://acndpm3fvl11.feishu.cn/docx/I1OPdBVJsorEAAxAsz6cfKvdnAh)与[迁移指南](https://lcnrp8hlo6f2.feishu.cn/docx/Yo0FdbIxWoO7gBxViPdcpqVenRg)：沿用团队统一仓库、原工作台、真实模型与动力学。通信时延现已接入，不再沿用早期“暂缓”口径。

## 全部关联任务

| 飞书记录 | 任务／线上状态 | 实现与完成依据 |
|---|---|---|
| recvuDDwIMmgvr | BSK 调研与官方示例／已完成 | `tools/vendor/` 保留来源和许可；官方瞬时变速示例与本项目有限推力实现分别说明 |
| recvuDxoc2CQT1 | 卡尔曼滤波＋PD 姿轨控／已完成 | `simulation/orbital_rendezvous/` 中估计、轨道参考及控制；原始估计误差、姿态和推力数据 |
| recvuDyq2I8Wud | 飞轮动力和静态属性／已完成 | 团队 MJCF 轮轴、惯量、限矩与限速；`tests/test_attitude_control.py` 的实际角动量交换检查 |
| recvuGFbfikK8K | 真实飞轮驱动交会／已完成 | 同一物理引擎内的本体与飞轮反作用；姿态、轮速、电机力矩遥测 |
| recvuDDwIMSVVc | 远程交会接入 SpaceSim／已完成 | 原工作台、LocalMujocoStepper、UE、相机、控制和采集贯通；实际网页验证 |
| recvw5Z4qlhkrf | 300→500 km 升轨及双星 GNSS／已完成 | 从 300 km **在轨初态**开始；有限推力升轨、接近、保持和独立数值复核；不代表火箭发射仿真 |
| reczz28KuW3GWuBG | GNSS 与随机轨道误差／已完成 | 共享接收机配置、联合协方差、随机初态及独立扰动力；噪声统计与种子复现 |
| reczz28MIzMlzVW1 | 大气阻力等物理量／已完成 | 阻力、太阳光压及扰动进入物理端口；外力闭合和轨迹响应检查，不仅检查参数存在 |
| reczz28NVX8mZCV2 | 合并三个任务模式／进行中 | 一个任务编号依次推进远距、近距与操作；相同工作台，状态恢复、停止、交接、真实相机及数据回读均需验证 |
| recvw6cgNrMWw5 | 代码管理与仓库合并／进行中 | 服务端与适配器在统一仓库同一 PR；跟随 main，保留共享页面和功能。本人交付不等于团队合并完成 |

10/18 周报中的“UI 已合并”不能替代实际行为验收：选择起始条件后若隐藏遥操作或采集区，仍未满足统一前端要求。仓库由维护者审查合入 main。

[项目群的周报要求](https://applink.feishu.cn/client/chat/open?openChatId=oc_d18831213328163e9a80d6357f61bacc&position=44)：每周日 20:00 前提交覆盖周一至周日的进度，每份关联一个任务，实验结果附链接；工时按本人实际投入填写。线上周报与任务状态不随源码修改自动更新。

## 当前输入、输出和验证口径

| 模块 | 输入与配置 | 输出与核验 |
|---|---|---|
| 差分 GNSS | 两星装配质心真值只供传感器生成器使用；`backend/space_arm_platform/gnss_config.py` 为远近距共同配置 | 定位结果层的相关位置/速度误差、联合协方差及交叉协方差；控制器使用观测和估计，不直接使用目标真值 |
| ROE 导航接口 | `simulation/relative_navigation.py` 接收同历元双星估计和完整 12×12 联合协方差 | 输出目标相对对接星的 J2000 位置/速度及瞬时 ROE、两种协方差、时间和参考点；保留两星交叉协方差，不使用真值填充 |
| GNSS 时延 | 10 Hz 默认采样；原工作台支持 0—0.25 s 时延，默认 0 | 独立采集/接收/控制历元；跨阶段保留在途包、采样序号、随机流和后验；见 `tests/test_gnss_transport.py` |
| 轨道与环境 | 团队真实质量、惯量及几何；J2000 状态、BSK Earth/Sun/J2、环境和执行器参数 | 同一引擎积分，导航和制导使用同一原生场与独立星历查询；不通过覆写状态伪造轨迹；`tests/test_flight_environment.py` 与 `tools/verify_team_flight.py` 检查实际施力和完整记录 |
| 连续交接 | 已接受阶段、源模型指纹、引力配置及轨道/姿态/关节/飞轮/导航/环境状态 | `backend/space_arm_platform/mission_runtime.py` 保持任务身份，验收失败或停止禁止推进；两段文件和采集数据保留来源 |
| 统一工作台 | 三种选择仅表示起始条件；权限和动作依据服务器实际状态 | 固定“末端六维遥操作”标题与原控制、关节、采集区；自动交会时显示但禁用操作，不切换页面或覆盖面板 |
| 相机与采集 | 主视口、全局模型相机、腕部相机；独立权威采集通道 | 实际 UE 新帧、控制回执、Parquet/MP4 回读和时间戳；帧率以本次数据元信息为准，旧文档中的 30 Hz 不覆盖当前配置 |

默认绝对位置每轴 σ = 1/√3 m，绝对速度每轴 σ = 0.002/√3 m/s；近距差分残差每轴 σ = 0.03 m、0.0015 m/s。绝对 3D RMS 与每轴 σ 不可混用。差分误差与相关长度是仿真假设，不是厂商保证；当前没有原始码/载波模糊度解算，也不由 GNSS 解算姿态。历史单独飞行模式的 2 Hz、3 m、0.5 m/s 默认值不适用于连续任务。

`relative_navigation` 随实时状态、终端报告及交接输出；远距 NPZ 另保存 `roe` 和 `roe_covariance`。ROE 采用 [Koenig、Guffanti 与 D'Amico 的定义，式 (2)](https://slab.sites.stanford.edu/sites/g/files/sbiybj25201/files/media/file/space2016_koenigguffantidamico.pdf)：对接星为 chief，目标星为 deputy，顺序为 `[δa, δλ, δex, δey, δix, δiy]`，δa 按 chief 半长轴归一化，角量用 rad。输入是当前预测历元的估计；观测采集/接收历元继续由 GNSS 时延字段单独记录。圆轨道不依赖未定义的近地点；赤道、退化及非椭圆轨道将 ROE 标为不可用，不伪造数值。

这是联合导航后验的坐标与协方差转换，输出 **osculating** 根数；没有将其冒充消除 J2 短周期项的 mean 根数、独立 ROE 状态滤波器或 ROE 制导器。近距沿用现有“目标绝对估计与相对估计独立”的协方差近似，并明确标注；参考点从装配质心变为本体原点。单星 GNSS 位置/速度不能提供本体姿态，接口将 `attitude.available=false`，姿态仍来自单独 AOCS/视觉接口。双目替换由李一辉负责，本人负责参数、时钟和交会流程集成。

运行与检查在已配置的云端工作区执行，见[平台 README](../README.md)。本次输出写临时目录；必要结论进入 PR，避免重复堆积历史验证文档。

## 需要协作交付的输入

依据[近距方案及需求表](https://lcnrp8hlo6f2.feishu.cn/docx/NXYMdMUW5oFKqSxYjUlcQpZnnid)和[靶点方案](https://lcnrp8hlo6f2.feishu.cn/docx/UQEFdWjQUoAW9wxMt4GcFP5cnMh)：

| 负责人 | 集成需要的数据与依据 |
|---|---|
| 张怿心：近距总体、V-bar/CW | PR27 已交付组合质心 V-bar、四节点联合驻留、视觉就绪门控与锁存 SafeHold；实际算法、参考点和接口见[近距实现](NEAR_FIELD_RENDEZVOUS.md)，不将其改称计划到达时间的 CW 求解器 |
| 李一辉：腕部双目 | 左右真实图像及曝光时刻、内参/畸变、双目外参、相机到腕部/TCP 变换、同步精度 |
| 席梓杰：PnP 与目标姿态 | 靶点几何及编码、相对位姿/时间戳/协方差/失效标志、靶点到抓取接口变换、靶面朝向 |
| 刘丹旭：抓取机构 | 实际夹爪与目标几何、允许载荷、禁触区、接触/锁定/捕获确认条件 |

本人负责已交付模块的接口接入和全流程验证；其他成员尚未交付的算法、相机及机构不列为本人当前未完成开发任务。当前固定本体参数化双目、两指夹爪和逻辑 CaptureHold 不证明真实腕部图像闭环、物理锁定或载荷传递。[1009 调研](https://lcnrp8hlo6f2.feishu.cn/docx/M65TdsVeDoH7YdxgkA9cAIT5nag)中的选型及安装参数属于工程初值，必须与最终实际资产和标定对应。

双星装配质心距离、本体原点距离、原点到抓取点模长、相机光轴深度与 TCP 间隙必须分开。当前近距依次通过质心沿轨50 m、光学深度17.4 m、光学深度5.50 m和收拢TCP抓取面间隙0.5 m四节点，每点满足联合条件连续5 s。5.50 m放行前保持质心 V-bar，之后平滑引入抓取偏置；几何由原模型质量和FK求得，随manifest与交接保存。0.5 m参考定义依据用户授权作工程选择，不能称为团队已签署约定；参数化导航也不代表真实图像识别。控制条件与独立复核见[近距接口](NEAR_FIELD_RENDEZVOUS.md)。
