# S13：有摄动的连续轨道维持

`s13-gnss-orbit-maintenance-v1` 使用单颗团队 SARM、原生 400 Hz 动力学、50 Hz 物理记录和 2 Hz 带噪 GNSS 轨道估计。初始二体密切圆轨道高度为 500 km ±5 m、倾角 51.6°，初始真近点角为 180° ±2°。实际传播包含地球 J2、太阳引力、大气阻力、太阳光压及有种子的加速度扰动。当前评测是 20 分钟内的密切轨道带维持，没有把短回合结果外推为多日平均轨道或星座轨道维持性能。

前 120 s 用于收敛，随后从 120 s 至 1,200 s 的**全部物理采样**均须满足：半长轴相对 500 km 目标圆轨道偏差不超过 25 m，偏心率不超过 `1e-5`，轨道面与初始轨道面夹角不超过 `1e-4 rad`。这一固定时窗包含 54,001 个采样点；中途一次越界也会保留为任务失败，回到正确终态不能清除超限记录。运行不足整个时窗不能获得成功。

观测是装配体 COM 的 J2000 轨道状态估计；GNSS 测量噪声标准差为位置 0.2 m、速度 0.002 m/s，投递延迟为零。方法没有本体姿态或物理真值。动作沿用理想有界 J2000 COM 力模型，模长上限 5 N，通过原生本体轴电机实际执行；航天器姿态被动演化。高度 450–550 km、偏心率 0.01 和累计等效 Δv 20 m/s 是硬限制。

内置 `gnss-orbit-maneuver-v1` 在此任务中持续跟踪由首次 GNSS 估计确定相位及轨道面的目标圆轨道参考，以位置/速度反馈补偿真实摄动，不进入转移任务的停推锁存状态。该基线不使用目标真值，也没有将参考轨迹写入物理状态。方法可换成外部 `examples/autonomy_methods/orbit-pd.json`。零推力对照测量自由摄动漂移，持续沿速度方向满推力对照检验安全否决。

轨道根数统一从实际 COM 状态独立计算，使用固定地球引力参数的瞬时二体密切根数定义。允许在维持时窗内施加修正推力；评价持续的轨道带约束、最坏误差、超限采样数和实际推力积分。恒定质量模型只输出等效 Δv，不虚构推进剂或电气能耗。

```powershell
python tools/autonomy_benchmark.py spec s13-gnss-orbit-maintenance-v1
python tools/autonomy_benchmark.py run s13-gnss-orbit-maintenance-v1 --seed 1 --output run/orbit-maintenance-train
python tools/autonomy_benchmark.py batch s13-gnss-orbit-maintenance-v1 --split test --output run/orbit-maintenance-test
python tools/autonomy_benchmark.py rescore run/orbit-maintenance-train
```

当前在个人隔离分支开发，云端原生训练、冻结测试及平台验收尚未完成。数学评分用例验证固定时窗、中途超限、证据连续性和安全优先级，不作为物理运行结果。

云端相关轨道、任务目录、外部方法、执行、数据集和平台检查共 138 项通过。首轮仅有 API 目录的固定来源清单漏加 S13，更新后复测通过；评分和实际参考控制检查没有放宽。261 项前端检查（零跳过）、生产构建及 Ruff 通过。新用例确认维持时窗内继续修正 GNSS 误差、一次中途越界不能被正确终态掩盖，以及连续证据与执行器硬约束的否决优先级。
