# S12：轨道面调整

`s12-gnss-orbit-plane-v1` 对应 S12、P2、V0。单星 SARM 从约 500 km、倾角 51.6°、升交点赤经 0° 的近圆初轨出发，利用实际质心推力达到倾角 51.65°、升交点赤经 0° 的目标轨道面。倾角以 J2000 的 XY 面为参考，升交点赤经从 J2000 的 X 方向计量；轨道面使用有方向的角动量单位矢量定义。该有限配置验证小角度倾角调整，不代表所有跨轨道面转移。

初始轨道半径 ±20 m、相位 ±2°。目标高度仍约 500 km，要求瞬时二体半长轴误差不超过 10 m、偏心率不超过 2×10⁻⁶、轨道面角误差不超过 2×10⁻⁵ rad，并在实际推力不超过 0.05 N 的条件下连续保持 20 s。任务时限 600 s。评分从真实质心位置、速度计算 `h = r × v`；改变航天器姿态或仅达到正确高度，均不能替代轨道面调整。

使用原生 400 Hz 动力学、50 Hz 物理证据和 2 Hz 单星 GNSS/控制。GNSS 噪声标准差为 0.2 m / 0.002 m/s，零投递时延；方法只获得声明的滤波估计。物理模型保留地球点质量、太阳引力、阻力、太阳光压和未知扰动。最大推力 25 N、累计推力等效 Δv 上限 40 m/s；全程轨道高度 450–550 km、偏心率不超过 0.01。姿态被动传播，推力经过质心；燃料、电量没有有效模型，保持 `null`。

基线从第一次公开 GNSS 构建原始轨道面，生成平滑旋转的三维轨道参考。位置、速度和加速度保持导数一致，包含转动参考所需的角速度与角加速度项；控制器依据公开状态反馈输出有限推力。该参考只参与控制计算，从不写入传播状态。达到目标后依据公开估计停止推力，独立评分检查实际轨道与无推力驻留。

`unchanged-orbit-plane-v1` 保持原轨道面而忽略调整指令，用来验证到达轨道高度不能错误替代轨道面目标。另保留零推力与持续违规推力对照。外部方法复用 `examples/autonomy_methods/orbit-pd.json`；原工作台显示轨道面误差和实际改变角度。初始化、缺帧、重复帧、状态回写和硬安全否决沿用原轨控任务的证据规则。

```text
python tools/autonomy_benchmark.py run s12-gnss-orbit-plane-v1 --seed 1 --output run/s12-train-1
python tools/autonomy_benchmark.py run s12-gnss-orbit-plane-v1 --seed 1 --method unchanged-orbit-plane-v1 --output run/s12-unchanged-1
python tools/autonomy_benchmark.py run s12-gnss-orbit-plane-v1 --seed 1 --method-spec examples/autonomy_methods/orbit-pd.json --output run/s12-external-1
python tools/autonomy_benchmark.py batch s12-gnss-orbit-plane-v1 --split test --output run/s12-test
```

训练/验证划分为 1–3 / 101–103，测试划分为 201–210。本次 P2 验收预先固定 201–203，另外七个不计入本次验收分母。先完成训练种子 1/2、原生法向推力与轨道面/传感器复位（种子 3）以及原 S09/S11 回归，再运行冻结矩阵、三个对照、外部方法、批次恢复、独立复算、数据集、CLI 取消及网页流程。

云端源码 `3c26632` 的 585 项 Benchmark 后端测试、适配器元数据检查、261 项前端测试、Ruff 和前端构建通过，无跳过；报告 `.tmp/benchmark-orbit-plane-checks-v1/acceptance.json` 的 SHA-256 为 `e5fe2150955904d1d0adedab2ed7a22adc3d6f43b69f4f03c18f4b054ff0521f`。专用 `benchmark-plane-v1-20261107` 工作树固定相同源码字节，核验 113 个源模型资源并独立构建原生库。

原生法向推力/复位、S09/S11 原物理轨迹回归、训练种子 1/2 已通过。预先固定的 P2 测试种子 201–203 全部成功，均在 426.5 s 结束，保持无推力驻留 20.48 s。终态目标轨道面角误差为 4.631–4.864×10⁻⁷ rad，实际轨道面改变约 0.04998°，半长轴误差 0.167–0.749 m。保持原轨道面及零推力对照失败，持续违规推力被安全否决。外部方法物理轨迹与内置种子 201 相同；批次不重跑恢复和 2 回合/2053 转换的可移植数据集通过。冻结矩阵报告 SHA-256：`db7d496ce543f038b923193ff01c043759a14083f09ed5ebbc8234a680589e5d`。

CLI 取消及所属子进程退出检查通过，报告 SHA-256 为 `d4f9db1c8497b58a14cac76234e5947f69e8574389873ee1c692a1bbb28c57ab`。原工作台四个真实浏览器流程分别验证外部方法成功、保持原平面失败、违规推力安全否决和主动取消；JSON/CSV/物理日志下载、显示指标、独立复算与数据集一致，没有浏览器错误。网页报告 SHA-256 为 `58e9f2f16e38eaf1a45324cc07dba877ef5ec1a4195ed6b6f4aa93208449319f`。

最终生命周期审计在全部阶段结束后再次复算七个冻结/对照/外部回合，保留 58 个原始工件的 SHA-256 和修改时间，并确认冻结源码未变。S12 在上述有限配置和预先声明的三个 P2 种子范围内完成验收；其余七个测试种子未运行。完整报告 `.tmp/benchmark-orbit-plane-lifecycle-v1/acceptance.json` 的 SHA-256 为 `6d5bded6df013c19d5b5529bac854c262024cfe8fb201306810832338c93cfe1`。
