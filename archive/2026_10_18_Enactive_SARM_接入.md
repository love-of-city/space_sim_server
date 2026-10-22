# Enactive 与 SARM 平台接入

日期：2026-10-18。Enactive 分支：`feature/sarm-platform-integration`；统一平台分支：`feature/independent-plug-collection`。本文同时记录平台端接口与独立 Enactive 仓库的配套接入工作；本仓库仅包含平台端改动，Enactive 客户端和数据适配需在其自身仓库单独提交。

## 问题与最终职责

平台已有同步 RGB、7 维 LeRobot 数据、关节伺服和真实物理执行。Enactive 原有 joint-position loader 假定 14 维，且缺少当前平台的远程环境适配。本次补齐共享数据入口和在线执行接口，复用现有模型、训练器、推理服务及 ManipulationSimulatorClient。

| 职责 | 所有者 | 本次行为 |
| --- | --- | --- |
| BSK 环境与 MuJoCo 刚体/碰撞 | 平台原有权威动力学 | 继续使用现有 400 Hz 动力学及子步配置 |
| 状态与相机 | 平台 UE capture + 观测配对缓存 | 同一 session/frame/sim-time 的两路 1024×1024 RGB 与 7 维状态 |
| 模型执行 | Enactive 原有 canonical policy server | 未新建模型 host 或训练器 |
| 环境动作 | SarmEnvironmentProfile + 平台策略会话 | 六个绝对关节角 rad 和一个 [0,1] 夹爪开度；每行执行 20 ms |
| 关节限速与伺服 | 平台 reference governor 和原有控制器 | 绕过末端 IK，目标仍通过原有限制；不直接写入物理 qpos |
| 录制 | 原有 EpisodeRecorder/CaptureLifecycle/LeRobot writer | 50 Hz 双相机/状态，200 Hz 六维力；保留正式数据格式 |
| 数据加载 | Enactive adapter registry + 原有 joint-position loader | 注册通用 joint_position_7d，保留原有 14 维路径 |
| 任务成功 | 平台只读物理监测器 | 已实现 drawer_cycle，其余任务明确 unavailable |

## 会话与时钟

模型、人工遥控、脚本控制互斥；使用平台原有登录 cookie、场景归属与管理员权限。动作基于精确的上一次观测身份提交，request_id 必须是 UUID。旧帧、重复动作、并发 step 和不确定执行结果不自动重放。等待模型时冻结仿真时间，每个动作行推进 20 ms，不把推理墙钟时间算进轨迹。

普通 HTTP 等待默认 120 秒，空闲租期 180 秒（可设 10–3600 秒），录制关闭等待单独为 720 秒。reset 先结束上一条录制，使用两个等待预算之和；上一条封装失败会显式报错。关闭在请求取消后仍完成清理；确认释放前保留占用。最近 64 个已关闭会话保留带归属校验的结果，避免重复 DELETE 关闭新会话的相机。

未接入场景随机化；Enactive seed 明确标记为不支持场景随机化。当前 reset 恢复平台当前场景初态。训练 action_horizon=16 是数据配置，与在线 chunk_execute_horizon、50 Hz 物理动作时钟分别管理。

## 物理任务判定

抽屉成功使用真实导轨位移、卡扣角、双指接触、夹爪开度、关节速度、末端位置/朝向和持续时间。必须先夹稳、拉出约 8 cm、放回归位、再松爪撤离并保持。不会以脚本结束、模型 STOP 或到达步数预算代替成功。

首次完整回放揭示了平行双指夹爪的 180° 等价朝向歧义。监测器 v2 对两个等价夹持朝向计算完整主转角，仍要求工具朝下；侧倾、朝上及 90° 错向均有反例测试。没有放宽接触、位移、速度或保持时间阈值。旧运行日志的 success=false 原样保留，新的在线结果单独验收。

## Windows 数据路径

统一使用真实跨进程文件锁：POSIX flock / Windows msvcrt.locking；替换七处原有 fcntl 依赖。导航专属 map builder 仅在对应 provider 中导入，SARM 公共数据入口无需导入无关 Decord。未安装模型权重或 CUDA 训练环境，也未修改生产 BSK 的依赖。

数据配置以 ENACTIVE_SARM_LEROBOT_DATASET 环境变量指定外部 LeRobot 数据，默认相对路径 datasets/sarm/lerobot。六维力继续存在于平台数据中，本版未追加到模型 7 维 proprio_state，也不宣称模型已具备力控能力。

## 验证记录

- 平台最终基础 CI：927 项通过，未跳过（按仓库规则排除 deploy 组）；原生策略与物理任务监测测试另行 101 项通过。
- Enactive 客户端协议：69 项通过，涵盖认证、同帧双相机、7 维动作、限幅回执、复位、错误收尾和独立超时。
- Enactive 数据、原有 joint loader、配置、文件锁及 scheduler 单元测试：428 项通过，零排除；真实跨进程文件锁测试包括竞争、异常、进程终止和超过 11 秒等待。
- 扩展数据契约：54 项通过、6 项明确排除。相关导航依赖、Gloo、optimizer-step 细节保存在 Enactive outputs/tmp/sarm-data-verification/portability-verification.md。
- 实际公共数据入口解码既有示范的第 0/3208/6416 行：state [7]、action [16,7]、两路 [1024,1024,3]，尾部 mask 正确。checkpoint 元数据往返保留 7 维、horizon、相机与归一化契约。
- Windows 原有多 worker 资源探测仍使用 Linux /proc/self/cgroup；较广 scheduler process 测试在该前置条件失败。本次单场景远程客户端不依赖此路径。没有声明 Windows 原生分布式训练/worker pool 已可用。
- strict docs build 与范围内 Ruff / diff 检查通过。最终真实验收与新数据回读结果见下文。

## 真实场景运行

验收使用独立端口、独立临时账号和独立子进程，未占用用户正在使用的场景。Basilisk/BSK 环境、Enactive 客户端环境、含 Python MuJoCo 的 CI 环境分离，避免两种 MuJoCo DLL 同进程加载。

短录制：run/enactive-validation/20261018-145816/report.json。100 个真实动作行，仿真精确推进 2 秒；正式录制含开始/边界帧共 102 帧、204 张双相机采集，0 不完整样本、0 拒收，相机视频和 LeRobot metadata 均 complete。包含启动、复位、接口检查、执行、视频封装和清理共 216.359 秒。这个短片段不代表抽屉任务完成。

上一次长录制 run/enactive-validation/20261018-142631 因旧客户端 120 秒关闭超时，未完成封装；其记录不计作成功数据集。失败原因与修复后的验收分开保留，未将原始日志离线改造成成功录制。

完整真实验收：run/enactive-validation/20261018-150300/report.json，success=true、physical_task_success=true。正式数据位于 run/enactive-validation/20261018-150300/episodes/episode-20261018-150608-4b5e3c57/lerobot。

- Enactive 共享客户端实际执行 6300 行目标，从仿真时间 0.24 s 推进至 126.24 s，动作时间精确 126.00 s。平台在 chunk 边界确认成功后结束回放，未继续执行源示范剩余的保持动作。
- 抽屉最大位移 0.08036828296 m。真实在线判据依次在 81.24 s 确认稳定抓取、88.06 s 确认拉出保持、101.88 s 确认归位、109.80 s 确认松爪、126.24 s 确认最终成功。最终悬停位置误差约 1.32 微米，稳定保持 2.00 s；这些是单次回放结果，不是泛化精度承诺。
- 官方录制包含开始/边界帧，共 6302 帧、12604 张相机采集，dataset_status=complete、outcome=success；0 不完整样本、0 拒收。
- 两路 H.264 视频均为 1024×1024、50 FPS、6302 帧，实际解码第 0/3151/6301 帧通过；state/action 均为 7 维，数据时间戳连续 20 ms。
- 六维力共 25205 条，时间戳连续 5 ms（200 Hz），所有力/力矩数值有限。读文件验收记录为 recording-verification.json；物理判据时序证据为 physical-task-verification.json。paired-rgb-preview.png 为真实同帧双相机预览。
- 新生成的完整录制再次通过 Enactive 的 build_vla_data 公共入口：首、中、尾样本的 state [7]、action [16,7] 与 parquet 一致，双相机解码和尾部 mask 通过。证据：Enactive outputs/tmp/sarm-data-verification/new-complete-episode/real-episode-verification.json。短录制也做了同样回读，见 new-short-episode 目录。
- 从验收 harness 启动到全部清理共 1296.656 秒（约 21 分 37 秒），其中 Enactive replay 子进程含复位、执行和正式录制封装为 1162.453 秒。这不是模型推理性能测试，也不是纯动力学性能基准。
- 独立测试的后端、UE、动力学、Enactive 和视频 worker 进程均已退出，19800/19866/19867/19558/19888 测试端口没有残留监听。


## 使用入口

平台：space_sim_server/docs/ENACTIVE_SARM_INTEGRATION.md。
Enactive：docs/closed_loop/manipulation_environments.md 的 SARM 小节；scripts/configs/simulators/sarm__remote.yaml；scripts/configs/data/joint_position_7d__sarm_platform__h16.yaml；tools/smoke/replay_sarm.py。

通过回放验证的是数据、观测、控制和录制集成。真实学习策略还需要匹配 7 维绝对动作、双相机和归一化的 checkpoint；本轮没有执行真实模型推理、训练或 GPU 性能验收。
