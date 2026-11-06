# D05：合作目标远距交会

`d05-cooperative-far-rendezvous-v1` 对应 D05、P0、V0，采用原团队收拢 SARM 与目标星模型。配置从约 7 km 分离开始，只有追踪星的实际质心力受控；合作目标提供带噪 GNSS 滤波解，双方姿态均被动演化。目标轨道为约 500 km、51.6°；400 Hz 原生积分保留地球 J2、太阳引力、阻力、太阳光压及随机扰动。

初态目标 TNR 位置为 `(-7000, 200, 100) m`、相对速度为零，位置各分量在 ±20 m、速度在 ±0.01 m/s 内抽样。终点为 `(-150, 0, 0) m`，位置误差不超过 0.75 m、旋转坐标系相对速度不超过 0.05 m/s，并连续保持 20 s；总时限 3000 s。配置的整个初态分布须在 5 km 之外，整个到达域须位于 300 m 以内，同时要求至少 6000 m 实际距离缩减，不能用近距保持点初始化替代交会。

全程 COM 间距为 100–9000 m，相对速度上限 8 m/s，推力模长上限 25 N。方法每 0.5 s 获取独立带噪双星 GNSS 滤波状态，位置/速度标准差为 0.2 m / 0.002 m/s，时延为零。基线从实际公开初始估计建立连续五次轨迹参考，再根据当前公开 GNSS 状态闭环控制；参考不是物理轨迹回写，所有运动仍由原生力积分产生。无隐藏姿态控制或目标推力。

独立评分复用 50 Hz 权威状态和实际电机回执，额外核验实际初态距离、全过程距离变化、近距终态及连续驻留。不要求近距 V-bar/R-bar 走廊，也不会把远距航行中的正常位置误差视为保持点违规。任何硬安全约束违反、采样缺口或运行中物理状态写入均否决成功。燃料、电能没有有效模型，继续输出 `null`；报告实际推力积分及等效 Δv。

这是有限初态域的合作目标 7 km 外段交会配置，不包含跨轨道面交会、数百公里转移、视觉导航或抓取；这些能力分别由对应任务评测。正在运行的正式 HTTP 7 km→Near23→抓取全程是另一条集成验收，不能代替此配置的方法接口、冻结矩阵及网页验收。

```text
python tools/autonomy_benchmark.py spec d05-cooperative-far-rendezvous-v1
python tools/autonomy_benchmark.py run d05-cooperative-far-rendezvous-v1 --seed 1 --output run/d05-train-1
python tools/autonomy_benchmark.py run d05-cooperative-far-rendezvous-v1 --seed 1 --method-spec examples/autonomy_methods/relative-flight.json --output run/d05-external-1
python tools/autonomy_benchmark.py batch d05-cooperative-far-rendezvous-v1 --split test --output run/d05-test
python tools/autonomy_benchmark.py rescore run/d05-train-1
```

当前配置已实现，原生训练、完整冻结矩阵、对照、外部方法、复位、取消和网页验收仍待完成。训练/验证/测试使用既定的 1–3 / 101–103 / 201–210；未运行测试集不构成成功样本。
