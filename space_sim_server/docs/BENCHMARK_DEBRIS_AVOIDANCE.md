# S16：单星与被动碎片的自主避碰

`s16-catalog-debris-avoidance-v1` 对应 S16、P0、V0。一个团队收拢 SARM 受控，另有一个独立自由传播的合成碎片；碎片不计入航天器数量。配置、方法、独立评分和原网页入口已实现，真实原生训练及冻结验收仍待完成。

碎片为本仓库生成的半径 0.25 m、质量 50 kg 的均匀刚性球，惯量按实体球公式定义，不声称来自真实卫星或实测碎片。没有碎片推进器、控制器、遥测发送器或主动配合。两个原生刚体都接受 Basilisk 地球 J2、太阳引力、大气阻力、太阳光压和声明的随机扰动；保持原有 400 Hz 积分及 50 Hz 权威记录。只有合法初始化允许写入状态，运行中的规避动作通过 SARM 原生质心力电机产生。

SARM 在约 500 km、51.6° 近圆轨道。碎片初态在 SARM 旋转 TNR 系中为 `(0, -1500, 5) m`、`(0, 10, 0) m/s`，位置和速度各分量分别有 ±1 m、±0.005 m/s 的有界扰动，约 150 s 后横穿轨道面。规范检查整个初态分布的闭合方向、预测会遇时间及筛查距离，不只抽几个角点。私有证据再次检查实际抽样初态，已安全远离的初态不能计为规避成功。

方法每 0.5 s 接收自身带噪 GNSS 滤波解，以及独立噪声的外部碎片目录状态。自身位置/速度标准差为 0.2 m / 0.002 m/s；目录为 3 m / 0.01 m/s。目录模拟持续可用的地面估计，零时延、2 Hz 更新，没有建模真实地面覆盖、CDM 更新周期、完整协方差或目标通信。方法只见公开状态估计与采样/投递时间，不见实际状态、故障实现或评分结果。

基线使用公开估计进行有限时间线性最近距离筛查，危险时施加有界径向推力，预测间距达到控制器目标后锁存滑行。动作上限 5 N，累计等效 Δv 上限 3 m/s。完整物理传播和评分不使用线性预测替代。该基线仅针对声明的横穿轨道面场景，不宣称是通用最优避碰规划器。

整个 320 s 回合内 COM 间距须保持 100–5000 m，惯性相对速度不超过 20 m/s，SARM 满足轨道高度和偏心率约束。独立评分要求：实际最近接近发生在 100–200 s，最终距离至少 1000 m、向外运动至少连续 20 s，末段无推力滑行至少 30 s，且已执行至少 0.05 m/s 的实际机动。不能在最近接近之前提前判为成功。评分使用 50 Hz 物理样本及相邻样本弦段最小间距；这不是连续曲线或碰撞几何的数学安全证明。任何硬约束违反、重复/缺失采样、初始化偏离或运行中回写物理状态均否决成功。

本配置评价确定的安全距离约束。碰撞概率、推进剂质量、电能没有有效模型，输出 `null`。NASA CARA 明确区分接近筛查和结合轨道不确定性进行的碰撞风险评价；本任务不能作为业务风险概率或实际任务安全认证。[NASA 说明](https://www.nasa.gov/cara/step-2-close-approach-risk-assessment/)

```text
python tools/autonomy_benchmark.py schema --kind debris-avoidance
python tools/autonomy_benchmark.py run s16-catalog-debris-avoidance-v1 --seed 1 --output run/s16-train-1
python tools/autonomy_benchmark.py run s16-catalog-debris-avoidance-v1 --seed 1 --method-spec examples/autonomy_methods/debris-avoidance.json --output run/s16-external-1
python tools/autonomy_benchmark.py batch s16-catalog-debris-avoidance-v1 --split test --output run/s16-frozen
python tools/autonomy_benchmark.py rescore run/s16-train-1
```

训练种子 1–3、验证 101–103、测试 201–210。完整验收包括零推力和持续推力负对照、外部方法一致性、真实碎片传播、动作和传感器复位、既有单星轨控回归、取消、数据集及网页运行。此处不把单元测试轨迹当成真实动力学验收。
