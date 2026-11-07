# D11：追踪星持续指向合作目标星

`d11-cooperative-target-pointing-v1` 对应 D11、P1、C、V0。原团队 SARM 与目标星作为两颗独立航天器在原生动力学中传播，仅追踪星的三轴实体飞轮可控。模型保留两星原有几何、质量及收拢构型，移除平移执行器；目标星没有执行器。动作不能改变目标轨道或直接设置追踪星姿态。

追踪星初始轨道高度 500 km、倾角 51.6°、真近点角 180°。目标相对追踪星的初始位置为追踪星旋转 TNR 坐标中的 `(500,100,50) m`，每轴均匀扰动 ±5 m；相对速度为 `(0,0.4,0) m/s`，每轴 ±0.01 m/s。TNR 为沿轨切向、轨道法向、径向，所有位置与速度以完整装配体质心为参考。转换到 J2000 时包含轨道坐标转动项。初始追踪星 +Z 轴相对实际目标方向偏转 5°–15°，本体角速度每轴 ±0.001 rad/s。

方法接收 50 Hz 带噪姿态遥测、陀螺、飞轮编码器，以及两个独立带噪质心 GNSS 解。目标解通过声明的合作链路零延迟传递；GNSS 位置/速度标准差为 0.2 m / 0.002 m/s。GNSS 不提供姿态。任务同时保留已知的地球/太阳星历，不提供任一航天器的物理真值。此配置不代表非合作目标、视觉跟踪、激光终端精密指向或链路性能。

基线用公开的两星位置、速度求即时视线和视线变化率，驱动实际飞轮电机。60 s 后直到 240 s 结束，+Z 轴对实际目标质心的角误差始终不得超过 0.5°，视线方向变化率误差不得超过 0.001 rad/s。独立评分使用真实两星状态、追踪星姿态和角速度，逐样本检查整个规定时窗；最后一帧对准不能抵消期间的失败。绕视线方向的滚转不受精度指标约束，但仍受全程本体角速度安全限制。

两星间距须在 100–1000 m 内，旋转 TNR 相对速度不得超过 3 m/s；本体角速度上限 0.15 rad/s、飞轮上限 628.319 rad/s、每轴电机力矩上限 0.2 N·m。地球遮挡、初态越界、物理日志缺失或重复、初始化后的状态回写均不能获得成功。400 Hz 原生积分、50 Hz 物理证据，保留 J2、太阳引力、阻力、太阳光压及随机扰动。资源只报告实际飞轮力矩积分，燃耗和电能无有效模型，保持 `null`。

内置基线为 `cooperative-target-pointing-v1`。负对照包括始终保持首次目标参考方向、零飞轮力矩和持续满力矩；外部方法见 `examples/autonomy_methods/target-pointing.json`。任务复用原工作台、批次、独立复算和公开观测/动作数据集。

```text
python tools/autonomy_benchmark.py run d11-cooperative-target-pointing-v1 --seed 1 --output run/d11-train-1
python tools/autonomy_benchmark.py run d11-cooperative-target-pointing-v1 --seed 1 --method frozen-target-pointing-v1 --output run/d11-frozen-direction-1
python tools/autonomy_benchmark.py run d11-cooperative-target-pointing-v1 --seed 1 --method-spec examples/autonomy_methods/target-pointing.json --output run/d11-external-1
python tools/autonomy_benchmark.py batch d11-cooperative-target-pointing-v1 --split test --output run/d11-test
```

训练/验证/测试划分为 1–3 / 101–103 / 201–210。P1 验收预先固定测试种子 201–203，其他七个不计入本次验收分母；先运行训练 1/2、原生两星传播/轮电机/独立 GNSS 复位（种子 3）及原 S02/S05 回归，再运行冻结矩阵、三个对照、外部方法、恢复、复算、数据集、CLI 取消和四种网页流程。

云端源码 `f06c296` 的 608 项 Benchmark 后端测试、适配器元数据检查、261 项前端测试、Ruff 及前端构建通过，无跳过。报告 `.tmp/benchmark-target-pointing-checks-v1/acceptance.json` 的 SHA-256 为 `f0027ce2e82b46e7ef9a9de2a8b835fa9f6f920b9f37c26ea58e891e27b9e13d`。专用 `benchmark-target-v1-20261107` 工作树已核验源码、113 个模型资源并独立构建原生库，完整原生验收排在 D19 流程之后；必须等待完整飞行与 D19 的所有进程退出才启动。目前没有 D11 原生回合结果，不能将基础检查通过视作任务已验收。
