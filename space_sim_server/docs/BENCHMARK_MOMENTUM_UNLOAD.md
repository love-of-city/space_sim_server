# S08 飞轮动量卸载

`s08-wheel-momentum-unload-v1` 对应 P2 / V0 / 单星任务。沿用团队收拢 SARM、三个原飞轮、轨道环境和带噪姿态/陀螺观测，新增明确声明的理想外部纯力矩执行器。它评测卸载和姿态保持，不代表具体推进器布局、喷流、最小脉冲、推进剂或电耗模型。

初始三个轮速为 `(600,-580,560) rad/s`，自旋惯量均为 `0.01 kg·m²`；姿态相对目标随机偏转 0.02–0.05 rad，本体各轴角速度在 ±0.001 rad/s 内。原飞轮每轴最大力矩 0.2 N·m，硬轮速上限 628.3185307179587 rad/s，本体角速度模长上限 0.15 rad/s。积分 400 Hz，方法和权威日志均为 50 Hz，最长 480 s。

动作包含三个飞轮关节力矩和本体系外部力矩向量。外部力矩模长上限为 0.02 N·m，通过质心 site 的三个纯旋转 motor 进入原生物理，保留原模型全部质量、惯量、轮系和收拢几何；没有平移命令。六分量 gear 的前三项全零、后三项分别为本体单位轴，依据 [MuJoCo site actuator 定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-general)。复位后禁止直接修改物理状态。

方法只见带噪姿态、陀螺和理想轮速编码器。基线复用现有 `WheelUnloading` 的 70% 启动、50% 结束滞回策略，并给原姿态 PD 的飞轮命令加入等值外部力矩前馈，补偿机体反作用。卸载策略内部状态随回合复位。外部方法通过同一 JSONL 协议接入。

独立评分同时要求：

- 每个飞轮绝对转速不超过 350 rad/s；轮系角动量向量模长较初始降低至少 3.5 N·m·s。
- 原生外部力矩模长的时间积分至少为 3.5 N·m·s。
- 外部力矩关闭至模长不超过 `1e-6 N·m` 后，姿态误差不超过 0.5°、本体角速度不超过 0.001 rad/s，连同前两项连续保持 10 s。
- 初态分布、连续物理时钟、外部请求与实际执行器读回一致；任何硬安全违反否决成功。

轮系角动量使用三个原转子自旋惯量乘相对轮速后构成的本体向量，不是整星角动量。力矩积分也不等于燃料或电能；这两项输出为空。只减小轮速、只保持姿态、一直施加卸载力矩均不能通过。负对照包含仅姿态保持、无外部力矩的飞轮制动及持续满飞轮力矩。

在云端服务目录使用统一入口：

```text
python tools/autonomy_benchmark.py spec s08-wheel-momentum-unload-v1
python tools/autonomy_benchmark.py run s08-wheel-momentum-unload-v1 --seed 1 --output ../.tmp/unload-training
python tools/autonomy_benchmark.py run s08-wheel-momentum-unload-v1 --seed 1 --method-spec examples/autonomy_methods/momentum-unload.json --output ../.tmp/unload-external
python tools/autonomy_benchmark.py rescore ../.tmp/unload-training
```

当前实现已接入目录、评分、CLI、外部方法、数据集与原网页入口。先运行训练 1/2、原生复位 3 和 S02/S22 轨迹回归；P2 验收冻结测试 201/202/203，明确保留其他七个测试种子为未运行。冻结后不依据测试结果改参数或重试选优。对照、外部方法、取消、数据集回读、网页和离线复算全部完成后才计入已验收来源。

源码 `6e194cc` 在云端通过 668 项相关后端检查、适配器元数据检查、261 项前端检查、Ruff 和生产构建，无跳过。首次检查暴露外部方法测试未创建日志目录及平台目录断言漏加 S08，修复后完整复跑通过。报告 SHA-256：`78565c0c6adc0f6988390b30f3f85e37fab3cde7b8242be776e5298ce8649092`。

独立原生工作树保留上述精确源码字节及已核验的 113 个模型资源。独立构建 DLL SHA-256 为 `9316d4a78442fbd8c051ba6d20186a2ebe41e4dc39f28235dc23c895f5411592`，构建清单 SHA-256 为 `49d2c286093d26925b0be0d89c91b62116db358f4a16406bcf3147def321b8af`。原生训练与完整验收已排队，尚未打开训练或测试回合，不能据此声称原生卸载成功。
