# 长录制反压与异步采集修复（2026-09-29）

本次针对“连续录制后待发送图片增长、队列满时阻塞或退出”修复。
未调整动力学参数、RKF45、积分容差、240 Hz 动力学、120 Hz IK、30 Hz 仿真采样网格，
也未降低双相机分辨率、JPEG 质量或通过丢弃严格帧来追求实时速度。
JPEG95 是原有严格采集格式；这次只是将同样的编码工作移出 UE 游戏线程。

## 1. 在物理推进前反压，而不是在物理回调里等队列

`simulation/teleop_grasp_unreal.py` 在每次 `ExecuteSimulation` 之前检查
`SimulationControlClient.has_observation_capacity()` 和
`RenderPublisher.has_frame_capacity()`，预留两个输出槽位（包含初始 t=0 情形）。
不足时在外层循环短暂等待；独立网络线程仍可接收 STOP、reset 和最新控制。
预览时也检查严格 FIFO 的空间，避免 START 在检查后、回调前到达的竞态。

严格渲染入队改为非阻塞操作。容量不足仍是明确的契约错误，绝不默默丢帧；
生产主循环必须先执行容量检查，不能把底层 `publish_frame` 当作无限容量接口。
这去除了正常拥塞时在 Basilisk/SWIG `UpdateState` 中等待 10 秒再抛异常的路径。

## 2. 用完成确认限制整个链路的超前帧数

新版客户端声明 `capture_pair_ack_v1`。后端收到采样状态后，只有
**对应双相机到齐，且 writer 的 append 返回**，才发送该状态的可靠 ACK。
因此 ACK 不再仅表示“状态已接收”，它同时归还下一帧的生产额度。

- 严格采集最多允许 16 个未确认状态；每次预检预留 2 个槽位。
- 预览保持原有 128 个状态的传输容量。
- 较早到达的图片仍以 session/frame/time/episode 配对，不改为“取最近图片”。
- 图像 TCP 的单包 ACK 仍只表示接收链路接受；不能将它解释为磁盘持久化。
- 状态 ACK 中的 append 确认也不等于断电安全的 fsync 或最终 MP4 封装完成。
- 无新能力标记的客户端保留旧状态 ACK 行为。完整新反压机制必须同时更新后端和仿真端。

这里有意保留严格采集所需的流量约束：如果 GPU/磁盘持续比生产端慢，
仿真的墙钟推进率会让步，但采样时间网格和已接受的帧不能因此丢失或重采样。
增加 RAM、无限队列或取消全部反压都不能解决持续的吞吐差额。

## 3. UE 异步 GPU 回读 + 后台 JPEG

`BskSceneController.cpp` 的严格 RGB 路径现在为：

1. 应用权威状态并渲染本次双相机。
2. 在恢复展示插值前，冻结帧号、时间戳、episode 和相机内外参，提交 GPU copy。
3. 后续 Tick 检查 readback 是否完成，按实际行跨度拷贝像素并处理 BGRA/RGBA。
4. 独立线程池任务使用私有像素和私有 ImageWrapper 编码 JPEG95，不访问 UObjects。
5. 按提交顺序将完成产品交给有界网络/磁盘队列，后台任务乱序完成也不改变输出顺序。

最多 16 个异步相机任务、256 MiB 入队字节预算；它是队列预算，不是整个 UE 进程的内存上限。
游戏线程不等待队列空位。正常 STOP 后仍轮询、排完已接受任务；会话重置和退出才取消旧任务。
保留 `-BskSynchronousCapture` 作为同步路径对照，不作为默认启动配置。
网络发送使用非阻塞 socket 和短时可中断等待，可靠包在重连前保留。

## 4. 故障不再要求动力学进程跟着退出

UE 可发送带 episode/session/frame/camera 的 `capture_error`。后端忽略旧 episode 的错误，
对当前采集明确标记失败。缺图或 writer 长时间没有完成进展也标记失败，而非伪造完整数据。
watchdog 独立执行冻结截止点、发送 OFF 和失败收尾；不占住控制事件循环。
STOP 会唤醒等待状态 ACK 的配对等待者，已接受的截止点内样本仍按既有逻辑排空。

当前失败等待阈值仍为 10 秒。网络长期不可用、磁盘满、GPU 挂起等不能保证成功录制；
此修复也不是进程崩溃后续写方案。保留诊断文件，失败 episode 不发布为有效数据集。

## 5. 监控

- 仿真输出 `capture_backpressure`，包含严格渲染积压和未确认状态计数。
- `capture_sync` 增加 `oldest_unwritten_sample_age_s`、`last_written_frame_id`。
- `/api/state` 的 `capture_channels` 增加接收队列数量；`pipeline` 包含 UE 待渲染帧、待发图片、
  异步任务数、回读/编码耗时和源状态到采集耗时。
- 跨机器的墙钟时间差要求机器时钟同步；不应将它当作单调时钟的绝对延迟证明。

## 6. 隔离验证与部署

验证不使用生产端口，不改历史 episode，也不将诊断录制放入正式数据目录。

`tools/validate_capture_pipeline.py`：回放离线原生状态和真实场景资源，测 GPU/网络/写盘。
需显式提供保存的 `hello/manifest/frame/observation` JSON bundle；不是动力学性能测试。

```powershell
python tools/validate_capture_pipeline.py --bundle <bundle.json> `
  --output run/<新的诊断目录> --seconds 600 --flow-control --write-dataset
```

`tools/verify_realtime_capture.py`：真实 Basilisk + UE + 控制连接 + writer，支持连续两次录制、
主动重连和诊断场景的 `--operating-pose`。已更新为按需 START/OFF 和完成确认协议。
Windows 如禁止默认端口，请显式指定四个可用的隔离端口。该测试不含浏览器 WebRTC 接收端。

### 本轮已完成的十分钟链路测试

产物：`run/capture-fix-20260929/async-dataset-600s/summary.json`。

- 真实场景资源、两路 640×360、JPEG95、30 Hz 仿真采样、真实 LeRobot 写盘。
- 18,000 状态 / 36,000 图片，最终 `dataset_status=complete`。
- 独立读回 Parquet 的 18,000 行，并逐帧解码两个 MP4，各 18,000 帧，分辨率正确。
- incomplete、unmatched、rejected、stale 均为 0。
- 采集阶段墙钟 633.58 秒，约 28.41 对/秒；不是宣称动力学达到 0.947 倍实时。
- 本回放工具设定的未完成帧上限 14，实测最大 14。
- 源状态到采集延迟中位数 107.18 ms、P95 154.71 ms、最大 395.58 ms；
  首尾为 61.39/114.27 ms，没有增长为数秒的历史积压。
- STOP 后官方数据集视频封装仍耗时约 7 分钟，是独立的收尾成本，不包含在 633.58 秒中；
  不能据此宣称长录制停止按钮立即完成。这部分未在本次改写。

### 真实动力学与控制链路检查

`run/capture-fix-20260929/native-end-to-end/summary.json` 保存另外两次原生运行结果：
初始姿态为诊断副本的操作姿态，含非零末端移动、每次录制主动断开一次状态连接后重连。
两次均完整收尾，分别 601 状态/1,202 图片、602 状态/1,204 图片，无缺样本/拒收。
分别推进 20.00/20.03 仿真秒，用时 34.20/26.28 墙钟秒，RTF 约 0.585/0.762。
这些是完整仿真路径的隔离观测，不包含浏览器连接；不是受控的前后加速比。
说明此次解决录制拥塞退出，并不证明动力学、IK、渲染合计已在所有动作下达到墙钟实时。

真实 GPU 冒烟测试还验证了：空闲不产生严格图片、连续两个 episode、STOP 后排空、
异步任务在途时切换 session，以及逐帧左右交替的红色标记与源帧号匹配、图像不颠倒。
同步对照路径也通过相同检查。

后端针对性回归 184 passed / 8 skipped；之后补充的失败关闭竞态检查与相关回归 26 passed
（与前一批有重叠，不相加）。适配器 31 passed / 16 subtests passed。UE 5.6 Development Editor
插件已完整编译并链接；真实 GPU 同步/异步检查均通过。
`gpu-pixel-comparison.json` 对比 30 张移动标记图像，同步/异步单张 RGB 平均绝对差最大
约 0.168/255；这不是逐像素完全相同的承诺。

必须完整重启后端，再启动加载新 DLL 的 UE/仿真场景，才能同时启用新协议。
仅刷新浏览器或仅重启 UE 不足。此次没有替换正在运行的生产后端、提交或推送代码。
