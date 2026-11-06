# D27：真实双目图像与自身陀螺的目标角速度估计

配置 `d27-stereo-gyro-spin-v1` 评估目标自旋与角速度估计。它使用两颗被动自由飞行航天器、团队 SARM 和 10 kg 目标星、原生 400 Hz 积分及真实 UE 双目图像。没有受控航天器，也没有目标遥测、目标陀螺或目标主动调姿。已知被动标志物属于 P 场景；本配置不代表无标志非合作目标或高速翻滚 D28。

目标初始本体角速度以相同概率从正、反两个方向抽取：均值为 `[0, ±0.06, 0] rad/s`，各分量再叠加独立均匀 `±0.015 rad/s` 偏差。追踪星也有非零随机本体角速度。初态阶段按真实 COM 偏置修正本体原点速度，后续运动由原生动力学推进；所有平移及飞轮电机命令保持零，不进行姿态锁定或状态改写。

方法每 0.5 s 接收实际双目 PNG、静态标定及自上一图像起的完整自身陀螺测量。相机为 640×480、2 Hz、0.12 m 基线；图像瞬时曝光，模拟投递时延为零。自身陀螺复用项目 `Gyroscope` 模型，在实际 50 Hz 物理发布时刻采样，含白噪声、初始偏置和偏置随机游走。这些参数是工程模型假设，不宣称来自实物标定。公开测量只包含追踪星角速度、时间、方差及饱和有效性，没有目标真实运动或陀螺内部偏置。

输出包括左光学相机系下的目标位姿，以及**目标本体坐标轴下、相对于 J2000 的目标角速度**，单位 `rad/s`。不能将相机与目标的相对旋转率直接当成目标自身角速度。内置 `image-gyro-spin-v1` 在独立 OpenCV 进程中估计图像位姿，积分自身陀螺以补偿相机旋转，再用 2 s 图像历史拟合角速度。失去图像后丢弃旧拟合历史；陀螺失效不能当作零角速度继续获得估计信用。

20 s 回合包含 40 次位姿输出。前 2 s 为声明的运动估计预热，之后 36 次角速度输出必须全部有效；角速度 RMSE 上限为 `0.015 rad/s`，峰值上限为 `0.04 rad/s`。评分逐时刻对照权威原生目标本体角速度；位姿误差、图像/物理对应、安全距离、实际电机输出、接触与状态写入约束仍单独检查。末端额外图像、最后一次正确输出或预热期输出不能抵消中间失败。

`zero-image-spin-v1` 使用真实图像位姿但故意输出零目标角速度，检验角速度指标；`missing-image-spin-v1` 始终不给出位姿和运动估计。外部示例 `examples/autonomy_methods/vision-spin.json` 使用同一公开接口。源码接口、数据集和原工作台均区分位姿与角速度指标。

```powershell
python tools/autonomy_benchmark.py spec d27-stereo-gyro-spin-v1
python tools/autonomy_benchmark.py run d27-stereo-gyro-spin-v1 --seed 1 --output run/spin-training-1
python tools/autonomy_benchmark.py run d27-stereo-gyro-spin-v1 --seed 201 --method zero-image-spin-v1 --output run/spin-zero-control
python tools/autonomy_benchmark.py rescore run/spin-training-1
```

开发与验收分开记录。新配置的真实原生/UE 训练运行、冻结测试、负对照、复位和网页验收尚未完成；接口检查和合成数学用例不构成物理运行成功证据。当前在独立开发工作树推进，既有 D23/D24 冻结矩阵保持原源码及资产不变。

2026-11-06 云端完成相关后端检查、261 项前端检查和前端生产构建。实际加载原生环境类生成公开上下文，两个 OpenCV 独立进程方法及缺失估计对照均完成 reset/close；父进程保持 Basilisk 与 OpenCV 隔离。第一次初始化暴露了通用外部方法 schema 白名单漏项，修复后补充三个实际方法定义检查并通过复测。该检查尚未运行物理场景或处理评测图像，报告 `.tmp/benchmark-motion-protocol-v2/acceptance.json` SHA-256 为 `c0679c5b95adee058c9362a26941a71c9b4df41b8975b8d1d1926df7c2fe7511`。
