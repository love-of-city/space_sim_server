# D40：接近中止与安全撤离

`d40-gnss-proximity-abort-v1` 评测合作目标近距操作外段的自主中止：追踪星已在约 150 m 处以约 0.5 m/s 接近目标，方法需根据 GNSS 状态估计判断风险，实际制止接近，撤到 250 m 保持点并连续驻留 20 s。它覆盖 100 m 安全边界之外的撤离，没有宣称验证米级接触前中止、视觉失锁恢复或非合作目标导航。

任务使用团队 SARM 与目标星的收拢双星模型，目标只提供合作导航数据，只有追踪星平移受控。初态 TNR 位置为 `(-150, 0, 0) m`、速度为 `(0.5, 0, 0) m/s`，每轴分别有 ±1 m、±0.01 m/s 的有界扰动。目标位于 500 km、51.6° 轨道；实际传播使用 400 Hz 原生积分，包含 J2、太阳引力、阻力、太阳光压和有种子扰动。两星姿态被动演化。

初态必须确实处于接近运动：所有允许初态的径向接近速度至少为 0.4 m/s，且按声明的 120 s 线性预测会进入 100 m 安全球。规范用区间上界校验整个初态分布，不只检查均值或有限几个角点。这个线性模型定义风险场景及基线告警，并不代替真实传播或独立安全评分。

成功须同时满足以下物理证据：

- 在开始后 40 s 内，确认至少连续 5 s 的向外运动，距离变化率不低于 0.02 m/s。短暂反向、相互分隔的片段或重复/丢失采样不能拼成确认时间。
- 到达 TNR `(-250, 0, 0) m`，位置误差不超过 0.75 m、旋转坐标系相对速度不超过 0.05 m/s，并连续驻留 20 s。
- 最终距离比该回合实际初始距离至少增加 75 m。已在保持点或已经撤离的初态不能作为中止成功。
- 全程满足 COM 距离 100–600 m、相对速度不超过 1.2 m/s、实际推力模长不超过 5 N。超限否决成功；晚于截止时间才制止接近，即使最终到达保持点，也保留为任务失败。

最长回合为 360 s。方法每 0.5 s 接收两星独立带噪 GNSS 轨道滤波估计，位置/速度测量噪声标准差为 0.2 m / 0.002 m/s，投递延迟为零。动作是理想 J2000 COM 力，沿用已有原生电机执行适配器；没有物理状态写入或隐藏姿态控制。恒定质量模型仅报告实际推力积分及等效 Δv，不生成推进剂、电能或接触性能指标。

基线 `gnss-relative-flight-v1` 用公开 GNSS 估计计算有限时域线性最近距离；检测到接近风险后锁存撤离决策，执行有界制动和连续退离参考。风险消退不会重新切回接近，`reset` 清除锁存。零推力与持续法向推力方法作为负对照，外部接口仍为 `examples/autonomy_methods/relative-flight.json`。评分器只使用 50 Hz 权威 COM 状态和原生电机回执，不读取方法的风险判断或阶段名称。

```powershell
python tools/autonomy_benchmark.py spec d40-gnss-proximity-abort-v1
python tools/autonomy_benchmark.py run d40-gnss-proximity-abort-v1 --seed 1 --output run/proximity-abort-train
python tools/autonomy_benchmark.py batch d40-gnss-proximity-abort-v1 --split test --output run/proximity-abort-test
python tools/autonomy_benchmark.py rescore run/proximity-abort-train
```

配置已在个人分支实现，原生训练、10 个冻结测试种子、负对照、复位及网页验收待完成。数学轨迹与时序用例用于核对评分规则，不能视为已经通过真实动力学运行。

整合源码 `d605b00365c4491d7db97cf51ba24bd9076a936f` 在 SpaceSim 云端通过 431 项相关后端检查、261 项前端检查、Ruff 和生产构建，均无跳过的测试。这组检查包含初态风险分布约束、晚中止后到达正确终态仍失败、间断反向运动不能累计、证据和安全否决，以及 GNSS 风险触发、撤离锁存和方法复位。报告 `.tmp/benchmark-abort-integration-checks-v1/acceptance.json` SHA-256 为 `cd8eb140afa34f468c6b65a0e4b338758915e7b0dc6f3d4e4eef26feb08d5b52`。实际原生执行器及复位用例单独运行，不计入上述检查数。
