# D19：双方主动机动的协同交会

`d19-gnss-cooperative-rendezvous-v1` 对应 D19、P2、V0、合作双星，采用两个独立的团队收拢 SARM 刚体和原生质心推力执行器。每颗星都有独立带噪 GNSS 解和轨道滤波器，集中式方法通过合作链路获得双方估计，分别输出 J2000 推力。两颗星的轨道和姿态实际传播，没有固定位置锚点。当前不声明分布式决策、星间时延或对接能力。

参考轨道约 500 km、51.6°，以第 0 颗星瞬时 TNR 定义初始相对位置 `(-600, 0, 0) m`，位置各轴 ±5 m、速度各轴 ±0.01 m/s。目标为 `(-150, 0, 0) m`，误差不超过 0.75 m、相对速度不超过 0.05 m/s，连续保持 20 s。总时限 500 s；实际距离须缩短至少 400 m。配置整个初态域在 500 m 外，整个到达域在 200 m 内。

独立评分还要求两颗星各自实际推力积分至少 5 N·s，且存在连续 10 s 双方实际推力模长均不低于 0.05 N 的机动区间。它从 50 Hz 原生执行器回读累计；首帧、重复帧、缺帧不能补充持续时间。仅一星完成接近、两星交替施力或短促同时施力，即使最终位置准确，也不能通过这项协同任务。

全程任意星间距离为 100–1000 m，相对速度不超过 2 m/s，逐星推力模长不超过 5 N。400 Hz 原生积分保留 J2、太阳引力、阻力、太阳光压及未知加速度扰动。姿态被动演化；不使用未声明姿控或状态回写。报告逐星推力积分和等效 Δv；燃料、电量没有有效模型，保持 `null`。

默认 GNSS 方法共享连续轨迹参考，以双方公开状态和实际质量计算相对控制，再分配双方推力。额外提供 `one-sided-formation-v1` 对照：第 0 颗星始终零推力，另一颗星承担等效相对控制。该对照用于检验“到达目标”能否错误替代“双星主动协作”。零推力和持续危险接近对照继续保留。

```text
python tools/autonomy_benchmark.py run d19-gnss-cooperative-rendezvous-v1 --seed 1 --output run/d19-train-1
python tools/autonomy_benchmark.py run d19-gnss-cooperative-rendezvous-v1 --seed 1 --method one-sided-formation-v1 --output run/d19-one-sided-1
python tools/autonomy_benchmark.py run d19-gnss-cooperative-rendezvous-v1 --seed 1 --method-spec examples/autonomy_methods/formation.json --output run/d19-external-1
python tools/autonomy_benchmark.py batch d19-gnss-cooperative-rendezvous-v1 --split test --output run/d19-test
```

训练/验证种子分别为 1–3 / 101–103，测试划分为 201–210；本次 P2 验收预先固定使用 201–203，另外七个不计入本次验收分母。当前已实现任务、评分、对照及原工作台入口；原生训练、冻结矩阵、复位、取消和网页验收待完成。配置注册不表示验收通过。
