# Enactive / SARM 在线接入

本接口把已有 Enactive 策略服务接到正在运行的平台场景。模型服务仍由 Enactive 的 checkpoint、inference server 和共享 ManipulationSimulatorClient 管理；平台继续拥有 BSK 轨道环境、MuJoCo 刚体动力学、UE 图像与 LeRobot 录制。

## 运行链路

1. 平台从同一权威帧读取机械臂状态、全景 RGB 和腕部 RGB。
2. SARM environment profile 把双相机、7 维实测状态和任务指令转换成 Enactive 现有 canonical observation。
3. 共享客户端调用 Enactive 现有策略服务。等待推理时，当前场景的仿真时间保持不变。
4. 策略输出绝对关节目标序列。每行是 J1–J6（rad）和夹爪开度（0 关闭、1 最大指令开度），每行执行 20 ms 仿真时间。
5. 平台通过现有参考限速、reference governor 和关节伺服执行；物理碰撞、接触和力传感器仍由原来的动力学计算。模型控制跳过末端 IK，不写 qpos。
6. 平台返回实际执行行数、最终配对观测和执行回执。客户端再据此发起下一次推理。

动力学保持 400 Hz，图像/状态与策略动作时钟为 50 Hz，六维力仍为 200 Hz。手动模式的 IK 保持 100 Hz。策略输出 chunk 长度与训练 action_horizon 是两个独立参数；一次 HTTP step 最多执行 100 行。

## 数据与动作契约

- 数据集沿用 LeRobot v3：observation.state 和 action 均为 7 维。原始关节与六维力继续保留在原来的额外字段中。
- 两路视图：base 对应 teleop/camera/spacecraft_overview；left_wrist 对应 teleop/camera/sarm_wrist_cam，当前为 1024×1024 RGB。
- 图像和状态必须有相同的 render_session_id、render_frame_id、sim_time_ns。缓存不拿最新状态替换迟到图像。
- 第一版模型 proprio_state 只包含 7 维。六维力通过平台观测和录制保留，未宣称模型已学会利用六维力。
- execution.requested_actions 是模型请求；execution.applied_actions 是经过限速/保护后的实际参考；action_limited 标示是否不同。训练录制继续保存实际持有的伺服目标。
- 夹爪是驱动行程归一化值，不是夹持力或指尖距离。

## 使用条件

启动更新后的后端和 SARM 场景，选择 local 动力学、50 Hz 数据集相机采集。机械臂若采用零位启动，应先完成平台的操作姿态准备。脚本任务、网页遥控和模型不能同时控制一个场景。

Enactive 中使用 sarm backend 和 joint_position_7d 数据 adapter。示例、训练数据配置和轨迹回放工具见 Enactive 仓库的 docs/closed_loop/manipulation_environments.md 的 SARM 小节及 tools/smoke/replay_sarm.py。

连接配置包含 base_url、instruction、task_name、timeout_s、idle_timeout_s、finalization_timeout_s、record、max_steps，以及可选 username_env/password_env。凭据通过环境变量提供，默认读取 SARM_USERNAME 和 SARM_PASSWORD。使用平台正常的登录认证，不复用或绕过用户数据库。

普通平台请求默认等待 120 秒；等待下一组模型动作的空闲租期默认 180 秒，可设为 10–3600 秒。录制关闭默认单独等待 720 秒，为原有视频写入器的 600 秒收尾期限预留传输和排队时间。reset 也会先收尾上一条录制，因此客户端使用收尾与普通请求两个时间预算之和。上述墙钟等待不改变 20 ms 的仿真动作周期。

record=true 使用现有 EpisodeRecorder/CaptureLifecycle 和正式 UE 相机通道，输出原来的 LeRobot 数据格式。record=false 只临时启用权威图像配对，不创建录制目录。模型会话运行期间，不允许从另一个页面单独启停录制；先结束模型会话再切换。

## API

| 接口 | 用途 |
| --- | --- |
| POST /api/auth/login | 使用现有平台账号获得 session cookie |
| GET /api/policy/status | 查看是否有策略会话占用控制权 |
| POST /api/policy/sessions | 创建会话，可选择先复位并启用正式录制 |
| POST /api/policy/sessions/{id}/step | 提交唯一 request_id、期望的观测身份和 7 维目标数组 |
| POST /api/policy/sessions/{id}/reset | 复位当前场景，开启新的录制/配对周期 |
| DELETE /api/policy/sessions/{id} | 释放控制并完成录制收尾 |

身份验证、场景归属和管理员权限与平台现有规则一致。step 不允许并发、重复 request_id 或旧观测。网络中断后不自动重发可能已执行的动作。动作执行结果不确定时，会话保持失败/占用状态，必须恢复连接并确认释放或复位后再操作。

关闭接口缓存最近 64 个已关闭会话的结果，并继续校验归属。重复关闭只返回原结果，不会再次关闭新会话的相机通道。

复位使用当前场景保存的初态；第一版不会把 Enactive seed 参数伪装成平台随机化。不同初始条件应先通过平台已有场景配置创建。

## Windows 使用示例

在 Enactive 仓库目录中使用它自己的 Python 环境，填写平台账号对应的 SARM_USERNAME、SARM_PASSWORD 环境变量，并修改 scripts/configs/simulators/sarm__remote.yaml 的 base_url。record=true 会走正式录制链路。不要把模型运行依赖安装到 BSK 的 Python 环境中。

已有匹配 checkpoint 的策略服务启动后，运行：

```powershell
.\.venv\Scripts\python.exe -m cli online client --backend sarm --env-config scripts/configs/simulators/sarm__remote.yaml --server-url http://127.0.0.1:9998/act --action-chunking sync --action-execute-horizon 16 --episodes 1 --output outputs/sarm
```

若只验证连接、物理动作和录制，不需要模型权重；将 SARM_DATASET_ROOT 环境变量设为兼容示范的 lerobot 目录，再执行已有动作序列回放：

```powershell
.\.venv\Scripts\python.exe tools/smoke/replay_sarm.py --env-config scripts/configs/simulators/sarm__remote.yaml --dataset-root $env:SARM_DATASET_ROOT --episode-index 0 --max-steps 100 --chunk-size 20 --output outputs/sarm-smoke/replay.json
```

100 步对应 2 秒动作时间，用于检查链路；完整任务需使用兼容初态与完整示范。该工具保存 execution_complete、close_complete、实际动作回执及物理任务结果，失败时仍保存诊断并返回非零退出码。

## 验证与限制

使用已有成功示范的目标序列，可以验证真实相机、关节控制、暂停、复位、录制及客户端格式。这类回放不等于模型推理，也不自动等于完整物理任务成功。成功状态只能来自平台的物理任务判据；没有判据时明确返回 success_available=false。

真实 VLA 推理还需要匹配此 7 维动作契约、双相机映射与归一化统计的 checkpoint。其它机器人或动作空间的权重不能仅通过改 backend 名称直接用来驱动本机械臂。

2026-10-18 的实际验收已完成：经 Enactive 共享客户端回放 6300 个动作行，真实抽屉完成拉出、放回、松爪及撤离，正式录制得到 6302 帧双相机数据和 25205 条 200 Hz 六维力；两路视频封装、首中尾解码及 Enactive 公共数据入口回读全部通过。此结果来自已有示范回放，不是已训练策略的任务成功率。

本轮联调记录见统一仓库 archive/2026_10_18_Enactive_SARM_接入.md。此变更不要求重编译 UE 插件；它复用已经存在的权威 RGB 图像协议。
