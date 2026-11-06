# S23：GNSS 失效期间的推算与导航恢复

`s23-gnss-outage-recovery-v1` 对应 S23、P1、V0。一个团队收拢 SARM 在约 500 km、51.6° 轨道自由传播；方法只能输出 J2000 质心位置与速度估计，没有推进器或飞轮控制权。故障作用于 GNSS 解的实际投递：发生期间公开观测中的解为 `null`，不会用真值、环境滤波后验或旧解填充。接收机有效标记可见，具体故障开始/结束时间仅进入环境的私有证据。

完整回合为 120 s，原生积分 400 Hz、物理证据 50 Hz、GNSS 与估计输出 2 Hz。GNSS 独立噪声标准差为 1 m / 0.02 m/s；故障开始在 20–30 s、持续 20–30 s，均在传感器网格上均匀抽样，采用独立种子流。观测零投递时延。动力学保留地球点质量、太阳引力、阻力、太阳光压和未知加速度扰动。故障是临时 GNSS 解不可用模型，不声明接收机硬件或热失效模型。

独立评分同时检查：

- 全时窗位置/速度 RMSE 不超过 0.8 m / 0.02 m/s，峰值不超过 5 m / 0.15 m/s。
- 缺失 GNSS 解期间仍逐次输出当前时刻估计，位置/速度峰值不超过 3 m / 0.08 m/s。
- 解恢复后 10 s 内，确认连续 5 s 位置误差不超过 0.8 m、速度误差不超过 0.03 m/s。
- 整个实际故障时窗、所有投递与估计均有连续证据；缺失、重复、错序、故障期间仍投递解、未触发故障不能通过。最后的准确估计不会清除此前误差。
- 全程轨道高度为 450–550 km，偏心率不超过 0.01；任何超出数值容差的实际推进力都会否决。初始化后的物理状态回写属于无效证据。

默认 `gnss-outage-recovery-v1` 只由公开 GNSS 初始化。解可用时使用原轨道滤波器；中断时用点质量轨道模型传播状态与协方差，恢复时重新测量更新。它不读取故障实现时刻。`freeze-during-outage-v1` 在中断期间保持过期状态，信号恢复后再更新，检验评分是否错误地仅重视恢复后的终点。`zero-recovery-estimate-v1` 始终输出零估计。由于此任务不授予执行器控制权，不提供伪造推力的策略；执行器安全否决由独立评分测试验证。

私有物理日志包含实际 GNSS 投递回执，公开观测日志保留实际输入。评分器读取物理状态、投递与估计证据独立复算。任务、基线和原生随机流均可复位；数据集保留中断时的空解和有效标记。原工作台使用同一任务与评分，展示中断期间最大位置误差及恢复确认延迟。

```text
python tools/autonomy_benchmark.py run s23-gnss-outage-recovery-v1 --seed 1 --output run/s23-train-1
python tools/autonomy_benchmark.py run s23-gnss-outage-recovery-v1 --seed 1 --method freeze-during-outage-v1 --output run/s23-frozen-state-1
python tools/autonomy_benchmark.py run s23-gnss-outage-recovery-v1 --seed 1 --method-spec examples/autonomy_methods/navigation-recovery.json --output run/s23-external-1
python tools/autonomy_benchmark.py batch s23-gnss-outage-recovery-v1 --split test --output run/s23-test
```

训练/验证划分为 1–3 / 101–103，测试划分为 201–210。本次 P1 验收预先固定使用 201–203，其余七个不计入本次验收分母。计划先完成训练种子 1/2、原生故障投递与复位（种子 3）及原 S14 回归，再冻结源码运行三个测试种子、两个负对照、独立方法进程、批次恢复、离线复算、数据集、CLI 取消和网页流程。当前实现尚未完成该原生验收；注册不表示通过。
