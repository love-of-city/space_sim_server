# S10：连续低推力轨道机动

`s10-continuous-low-thrust-v1` 对应 S10、P2、V0。它使用原单星 SARM、Basilisk 在轨动力学与质心原生力执行器，评测一次长时间低推力机动及其实际轨道变化。最大推力模长为 0.25 N；常质量、理想标定推力，不声明电推进器电源、热状态、推力建立时间或燃料模型。

参考初轨约 500 km、51.6°，半径 ±20 m、相位 ±2°。在 5–800 s 允许机动，要求实际推力积分除以模型质量得到的 J2000 矢量接近 `(0, -0.5, -0.6) m/s`，三维误差不超过 0.002 m/s。这是外力的积分，不将其误称为存在引力时的末速度差。总时限 900 s，原生积分 400 Hz、证据 50 Hz、GNSS 和控制 2 Hz。

除上述矢量积分，独立评分还要求：实际推力模长不低于 0.05 N 连续至少 600 s；由实际质心状态计算的瞬时二体半长轴相对本回合初值增加至少 1000 m；随后在推力不超过 0.005 N 的条件下连续保持 20 s。允许按此配置执行一段有限时长的连续机动，不声明长寿命螺旋转移。

连续时间依据原生执行器回读的完整相邻采样区间累计。首帧不增加持续时间；停推重置当前计数；缺帧、重复帧和初始化后状态写入不能通过。最终总冲量相同也不能代替连续机动。全程轨道高度 450–550 km、偏心率不超过 0.01、累计推力等效 Δv 不超过 1 m/s；违反硬约束否决成功。

默认基线复用已公开的理想标定执行器方法，以 0.2 N 执行给定矢量冲量，末次命令按剩余积分缩短，然后停止。评分读取实际回执，不相信方法自己的积分。GNSS 解作为可替换方法的公开观测；该理想执行器基线采用前馈冲量执行，不据此宣称轨道规划或未知推力故障恢复能力。

`pulsed-orbit-burn-v1` 使用同一目标冲量与硬上限，以 0.25 N 推进 4 s、停止 1 s 循环；它用于检验连续性。另保留零推力以及持续满推力的负对照。外部进程示例复用 `examples/autonomy_methods/orbit-pd.json`，原任务与结果页面展示最长连续推力时间和实际半长轴增量。

```text
python tools/autonomy_benchmark.py run s10-continuous-low-thrust-v1 --seed 1 --output run/s10-train-1
python tools/autonomy_benchmark.py run s10-continuous-low-thrust-v1 --seed 1 --method pulsed-orbit-burn-v1 --output run/s10-pulsed-1
python tools/autonomy_benchmark.py run s10-continuous-low-thrust-v1 --seed 1 --method-spec examples/autonomy_methods/orbit-pd.json --output run/s10-external-1
python tools/autonomy_benchmark.py batch s10-continuous-low-thrust-v1 --split test --output run/s10-test
```

训练/验证划分为 1–3 / 101–103，测试划分 201–210。本次 P2 验收预先固定 201–203，其余七个不计入本次验收分母。先完成训练种子 1/2、原生低推力与复位检查（种子 3）及 S09/S11 回归，再运行冻结测试、脉冲/零推力/违规对照、独立进程、批次恢复、复算、数据集、CLI 取消和网页流程。原生验收尚未完成，注册不代表验收完成。

源码 `a93dd68` 已在独立云端 CI 工作树通过 567 项相关后端检查、适配器元数据检查、261 项前端检查、Ruff 与生产构建，无跳过。报告 SHA-256：`b3d890658af5f6daaa2abdcc1805a44b615d537d38f2825784f8404b36d45a8a`。原生验收分支保留同一提交的精确已测源码字节，计划串行运行在 S23 完整流程之后，避免同时启动第三个物理作业。
