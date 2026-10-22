# 服务星陀螺角速度反馈

按 [10/22 卫星仿真群需求](https://applink.feishu.cn/client/chat/open?openChatId=oc_d18831213328163e9a80d6357f61bacc&position=133)，为抓取侧提供服务星残余转动的模拟观测。`teleop_grasp_unreal.py` 的原场景组件图在物理积分后、输出前，以 50 Hz 读取 `cubesat_bus` 的惯性角速度，并生成独立的 `service_gyro` 观测。包括主线双臂抓取场景；不修改物理状态、轮力矩或既有姿态控制器。

## 接口

`space-arm-control/1` 的 observation 可包含 `service_gyro`；老记录和禁用配置为 null。原有 Hub/API/WebSocket 及 `steps.jsonl` 保存该字段。当前不新增 LeRobot 张量特征，算法可读取同帧 JSON 观测。进程内通过 `assembly.extra['service_gyro'].latest` 读取最新样本，读取不会重新抽噪声。

| 字段 | 含义 |
|---|---|
| `angular_velocity_body_rad_s` | 服务星相对 J2000 的角速度，表达在服务星本体轴，rad/s |
| `sample_time_ns` / `sample_rate_hz` | 原物理采样时间／50 Hz；与相机曝光或输出时间分别使用 |
| `error_variance_rad2_s2` | 三轴未校准误差的先验方差，包含白噪声、初始偏置和累计随机游走；轴间独立 |
| `saturated_axes` / `valid` | 量程截断标记；任一轴饱和即不可用于控制，方差不再适用于截断结果 |
| `frame` / `reference_frame` | `spacecraft_body` / `J2000`，不是相机坐标，也不是相对目标星的角速度 |

采样读取原消息的 `timeWritten()`，必须等于当前物理历元；过期或未写入的源状态导致明确错误，禁止给旧状态重新贴时间戳。50 Hz 对齐 400 Hz 物理网格及支持的 1/2/4/8 步状态发布周期。流必须从零开始，重建场景时重置，重复读取同一时间不改变随机状态。低于 50 Hz 的数据集仅保存对应输出帧的最新样本，不声称独立全速 IMU 归档。

## 配置与误差模型

可在既有 scene-instance JSON 的顶层配置 `sensors.service_gyro`；省略时使用以下默认值。随机种子沿用 `scene.seed`，通过独立命名随机流生成；不影响场景、GNSS 或其他传感器随机数。无 scene-instance 的命令行运行使用 seed 0。启动时输出解析后的参数与种子。

```json
{
  "sensors": {
    "service_gyro": {
      "enabled": true,
      "white_noise_std_rad_s": 0.0001,
      "initial_bias_std_rad_s": 0.00001,
      "bias_random_walk_rad_s_sqrt_s": 0.000001,
      "range_rad_s": 0.5
    }
  }
}
```

测量为真实本体角速度加初始随机偏置、偏置随机游走和白噪声，最后按量程截断。初始偏置为各轴独立零均值高斯；游走每 0.02 s 的增量标准差为配置值乘 `sqrt(0.02)`；白噪声配置为**每样本**标准差，不是噪声密度。误差先验方差为 `white_std² + initial_bias_std² + walk² × elapsed_s`。偏置是时间相关误差，不能把连续观测视为完全独立，也不能直接把该先验方差用作已估计偏置滤波器的白噪声 R。

上述数值均为工程初值，未对应已选型硬件；未模拟安装误差、比例因子、温漂、量化、加速度计或传输时延。传感器建模分层参考 [Basilisk IMU](https://hanspeterschaub.info/basilisk/Documentation/simulation/sensors/imuSensor/imuSensor.html)，此实现仅提供本体角速度通道，不声称调用完整 IMU 模块。

## 接入与验证边界

抓取算法应检查 `valid`、样本年龄及外层 `render_session_id/reset_generation`，再按自己的相机／本体变换使用测量。陀螺本身不输出绝对姿态，不观测目标星运动，也不能单独消除平移误差。包中不包含真实姿态、实际隐藏偏置或真值角速度；原 `attitude_control` 仍是独立的真值诊断。

本提交交付被动陀螺反馈接口。尚未将测量闭环接入张怿心的姿态估计或刘丹旭的盲区抓取预测；不能据此声称已解决盲抓误差或完成图像导航与实体锁定。

基础 CI 检查三轴与时间语义、白噪声统计、偏置随机游走、种子复现、饱和、无真值泄露的包结构，以及真实 Hub／JSON 录制回读。云端环境另运行：

```powershell
& $env:SPACE_SIM_PYTHON -m pytest tests/test_gyroscope.py tests/test_gyro_observation.py tests/test_gyroscope_native.py tests/test_sarm_observation.py
```

`test_gyroscope_native.py` 使用真实 Basilisk 消息与调度器验证发布顺序、时戳和过期拒绝；不替代整个平台动力学验收。完整验收还需在既有云端启动原模型与双臂场景，给机械臂运动输入，比较独立真值诊断与带噪观测，检查场景重置、录制回读及实际抓取消费端。运行结果以 PR 的验证证据为准。
