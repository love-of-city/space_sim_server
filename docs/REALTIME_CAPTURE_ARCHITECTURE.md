# 实时采集架构（2026-09-23）

固定契约：动力学 240 Hz、IK 120 Hz、权威 RGB 30 Hz、双相机、LeRobot v3。
不增加深度/分割，不复制图片、不插值权威状态、不压缩缺失时段。

## 数据通路

```
Basilisk 权威状态 -> 有界保留队列 -> 控制 TCP -> SimulationHub -> 帧/时间戳配对
                    <- 顺序 ACK，重连重发未确认状态 <-              |
UE 权威 RGB -> 有界非阻塞出队 -> 图像 TCP -> CaptureReceiver ---------+
                 <- 单包 ACK / 重发 / 接收端去重 <-                  |
                                                        有界串行 I/O 队列
                                                         /                                                        JSONL/原始 JPG     独立 LeRobot 进程
                                                                  Parquet/MP4/meta
```

### 准备与录制边界

- `DatasetWriterService` 使用 Windows 支持的 `spawn` 独立进程。
- 后端 lifespan 预热官方依赖；准备过程不持有录制器状态锁。
- 点击开始后，只有 writer 和 episode 目录均准备好才发布 active episode。
- `preparing` / `writer_ready` / `writer_preparation_error` 在 `capture_sync` 中可见。
- 同一个进程复用预热后的 writer；不是每次录制重新导入训练依赖。
- `run_platform.ps1` 给初始化留出至多 180 秒，不再用旧的 20 秒启动上限误杀预热。

### 实时控制与写入隔离

- 状态锁只保护配对、计数和队列，不能覆盖 LeRobot 导入、RPC、磁盘写入和编码。
- 单独 I/O 线程按顺序写审计文件，并把完整样本发送给 LeRobot 子进程。
- 有界 I/O 队列：512 项、128 MiB（包含正在执行的图片作业字节）。
- 队列满时等待发生在接收/工作线程，Condition 等待释放状态锁；API 状态查询不中断。
- 异步 API 中记录动作使用 `asyncio.to_thread`，控制事件循环不等待磁盘锁。
- `dataset_frame_count` 表示子进程确认写入的帧，而不是只进入队列的帧。

### 权威状态可靠传输

- 新客户端声明 `reliable_observations_v1`，发送稳定的 `observation_stream_id` 和递增的十进制序号。
- 接收端返回 `observation_ready` / `observation_ack`；只在回调接收完成后推进确认序号。
- 仿真端保留最多 128 个未确认状态；满时暂停生产，而不是继续物理推进后丢弃发送失败的帧。
- 断线保留未确认数据，重连按序重发；确认丢失不导致重复写入。
- ACK 停滞 30 秒会明确失败并报出 pending/last_acked/transport_error，不宣称数据集完整。
- 重连不重放旧控制动作；deadman、reset generation、仿真 session 屏障保持有效。

### RGB 确认及反压

- 权威图片增加 `ack_required: true`，接收端入队后返回一个 `0x01` 字节。
- UE 网络线程同时只有一个未确认发送包；重连/ACK 超时会重发该包。
- 接收端按 session/camera/capture_sequence 去重，最近 1024 个标识有界保存。
- UE 最多保留 128 包 / 256 MiB（包括等待 ACK 的包），不在游戏线程阻塞等待。
- 游戏线程在取下一权威渲染帧前检查完整相机批次的容量；不足则让帧留在接收 FIFO。
- 接收器队列满不再因未处理的 `queue.Full` 退出。半包/半个长度头遇到 socket timeout 时保留已收字节。
- 网络 ACK 表示进入接收链路，不等于磁盘持久化；只有最终 `dataset_status=complete` 才通过保存验收。

### 配对失败与结束录制

- 每一采样时刻必须有全部相机，且 session、帧号、时间戳一致。
- 观测 tick 缺口立即报 `expected_tick/received_tick/frame`，不等到 128 项缓存污染后才报 RGB 超时。
- 图片早到且配对缓存满时实施反压，不默默拒收当前 episode 的图片。
- 停止先冻结观测截止点，再等末尾图片、I/O 作业及编码完成；不先等待永远仍有新帧进入的全局队列。
- 失败保留原始审计文件，不发布成功 archive，不把操作者的 success 当作数据完整性证据。

## 升级与验证

**必须重启后端并重新启动使用新 DLL 的 UE 场景。** 仅刷新浏览器或仅重启 UE 不够。
旧后端不支持状态/RGB ACK，不能与新客户端混用。不要在有活动 episode 时替换运行进程。

隔离验证（不会读写生产认证库，不占生产端口，不把诊断录制混入训练数据）：

```powershell
python tools/verify_realtime_capture.py `
  --adapter <UE仓库路径> --unreal <UE安装路径> `
  --scene run/scenes/<场景实例>.json `
  --output run/realtime-validation-<唯一名称> `
  --seconds 120 --episodes 2 --reconnect
```

该工具使用真实 UE、Basilisk、双相机 JPEG 和官方 LeRobot 写入器，施加小幅非零控制，
主动断开状态连接以检验重传。它的 episode 标记 diagnostic/not-demonstration，不能当作成功抓取示范。

自动测试覆盖冷初始化控制锁、慢 writer、满队列、半包超时、丢 ACK 重连、状态缺口、
重复图片、首次/第二次子进程录制及官方 v3 读回。普通单元测试注入进程内 writer 来保持速度；
`process_writer` 测试和隔离运行工具使用真实 spawn 实现。

以上机制不是断电/进程崩溃后自动续写方案。磁盘满、进程被杀或持续吞吐不足仍必须明确失败，
不能承诺任何硬件条件下保持墙钟实时速度；30 Hz 始终指仿真采样时钟。

## 本机验证结果（2026-09-23）

- 针对性回归：230 passed / 13 skipped；跳过项为可选环境/工作树相关测试，不计作通过。
- UE 5.6 Development Editor 完整编译成功。
- 独立冷启动双相机录制：600 帧 / 1200 JPG，complete。
- 同一服务第二次录制：600 帧 / 1200 JPG，complete。
- 重新冷启动约 120 秒录制：3601 帧 / 7202 JPG，complete。
- 三次均有小幅非零关节动作；每次主动关闭一次状态连接，最终 rejected/incomplete 均为 0。
- 官方 LeRobotDataset 读回成功，所有 Parquet 源帧号连续，时间戳严格落在 30 Hz 绝对整数网格。
- 六个 MP4 全部逐帧解码，视频帧数与状态行数一致，抽查图像分辨率 640x360x3。
- 验证程序事件循环最大采样间隔约 0.156–0.157 秒（包含启动阶段）；未再出现首次录制的多秒锁阻塞。

验证产物位于 `run/realtime-validation-20260923-short`、
`run/realtime-validation-20260923-long`，详细完整性检查见
`run/realtime-capture-integrity-20260923.json`。这些是诊断数据，不是训练示范。
本轮未修改生产认证数据库、未自动替换旧后台服务；更新生效需要完整重启平台。

启动兼容性保护与后续计算优化见 `STARTUP_AND_RUNTIME_PERFORMANCE.md`。
