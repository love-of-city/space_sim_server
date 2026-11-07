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

训练/验证种子分别为 1–3 / 101–103，测试划分为 201–210；本次 P2 验收预先固定使用 201–203，另外七个不计入本次验收分母。

源码 `67c9558a2cb773d91e34d4db970bfcfe6bff48e8` 在独立云端工作树完成完整生命周期验收。三个冻结种子全部成功，Wilson 95% 区间为 43.85%–100%；训练和对照不计入这三个样本。任务指纹为 `8ccbefd95d2dd5411dc38137f1051ae186c703882693ac21408fc8d81201ed1a`。

| 种子 | 初始/最终距离 m | 实际缩短 m | 连续共同机动 s | 各星实际推力积分 N·s | 连续到达保持 s |
|---|---|---|---|---|---|
| 201 | 595.1601 / 150.0329 | 445.1272 | 451.0 | 268.1938 / 268.1938 | 20.48 |
| 202 | 597.4441 / 150.0081 | 447.4360 | 451.0 | 270.0029 / 270.0029 | 20.46 |
| 203 | 601.3742 / 150.0358 | 451.3383 | 450.5 | 270.5802 / 270.5802 | 20.10 |

三回合于 462.0–462.5 s 结束。单边施力对照虽然连续保持 57.98 s、最终构型误差仅 0.02347 m，仍因第 0 星实际推力为零且没有共同机动判为 `task_failure`。零推力在 500 s 失败，危险推力在 68 s 因相对速度超限安全否决。外部方法的完整物理轨迹与内置方法一致，冻结批次恢复及公开数据集回读通过。

原生复位、M02 轨迹回归、CLI 取消、原工作台实际浏览器操作、离线复算和下载全部完成。最终审计重新评分七个冻结测试/对照/外部回合，58 个冻结文件的字节和修改时间保持不变，源码始终冻结。证据在本人云端工作树 `benchmark-fault-v1-20261107/.tmp/`：

- 训练/原生复位：`benchmark-cooperative-training-v1/acceptance.json`，SHA-256 `b53bb35cb4fb9646f32ab0bbcf09da7dcc6c6a5b1366a7115331836d0218607d`。
- 冻结矩阵及对照：`benchmark-cooperative-frozen-v1/acceptance.json`，SHA-256 `ad675a1352c0bdcdf15d42daffe22b15d98c32ae2826def517ca284a54bb61c2`。
- 取消：`benchmark-cooperative-cancel-v1/acceptance.json`，SHA-256 `2a11b0bb8bd133e66da109d5d5261761d1d46e8cbdac9b5cecc9ee405a6ae957`。
- 原工作台：`benchmark-cooperative-platform-v1/acceptance.json`，SHA-256 `2ab7fad1bee0135a03081f8c03eb58f6b902ebbc17fc17d101f95c11e73fa6b8`。
- 完整生命周期：`benchmark-cooperative-lifecycle-v1/acceptance.json`，SHA-256 `a4baaa49c396a782a2302737137687a11aa2c654769cf8088986441e6c93e212`。

该有限 P2 配置已验收；合作双星、集中决策、GNSS 和理想推力执行边界保持不变。它不证明分布式协同、视觉对接或完整 122 项产品已完成。
