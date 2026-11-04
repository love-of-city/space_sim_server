# 按需开启严格采集（2026-09-23）

## 用户行为

- 场景的 `dataset_capture` 仅表示具备采集能力，不表示正在截图。
- 未点击“开始采集”：UE 仍渲染交互视频，但不执行数据集权威 RGB 回读、JPEG 编码、发送。渲染状态采用 latest-wins，不累计历史预览状态。
- 点击“开始采集”：先准备 recorder，再发送 `capture_control`，仿真在下一次渲染状态发布时采样开关。响应在带有对应 request ID 的状态到达后返回，不把 socket 发送成功当作生效确认。
- 点击成功/失败结束：先冻结后端已接纳状态的末帧边界，再关闭新增严格帧。此前已入队的严格状态和匹配图像继续排空、写盘，之后恢复最新状态预览。
- 未采集时恢复启动配置中的相机预览频率；采集时保留原有预览降频策略。此变更不承诺正式采集时已消除所有吞吐瓶颈。

## 跨进程协议

1. 后端与仿真协商 `capture_on_demand_v1`。新后端拒绝对不支持该能力的仿真开始相机采集，启动预检也要求后端具有该能力。
2. 控制包：`{protocol: "space-arm-control/1", type: "capture_control", episode_id, request_id}`。空 episode ID 表示关闭；相同请求可安全重发。
3. render bridge 每次发布帧仅采样一次开关，给渲染帧以及对应 observation 附上相同的 `capture_episode_id` / `capture_request_id`。
4. UE 用 `capture_episode_id` 字段是否存在区分新协议与旧独立 demo。字段存在且为空：预览；非空：严格 FIFO 与权威采集。旧消息不带该字段时保留旧策略。
5. RGB 元数据携带 `capture_episode_id`。Recorder 仅接纳本 episode 的状态/图像，排除开始前的预览和上一次采集的迟到图像。
6. START/STOP 按同一个仿真 observation stream 重连重发；新仿真进程不继承上一次采集。关闭确认超时会标记失败，不伪报完整数据集。

Python 与 UE 都把严格 FIFO 和 latest-wins 预览分开；STOP 不能清空已接纳的严格帧。后端只排空冻结边界内的样本，不继续等边界外的新状态。

## 不变项

不修改 MJScene 动力学模型、质量/惯量、控制律、积分器或物理步长，也不跳过物理积分。RGB 编码格式、采样率和相机标定保持原配置。旧预览状态可以覆盖，但正式采集样本不能用这种方式丢弃。

## 部署与验证

同时更新后端、Python adapter 和 UE 插件。完整链接 UE DLL（不能只 `-NoLink`），重启后端及场景后生效；旧进程不会热更新。前端帮助文案已更新，构建后刷新页面。

主要回归：`tests/test_capture_on_demand.py`；adapter 的 `test_capture_on_demand_transport.py`；UE `BskUnreal.Capture.OnDemandBoundaries`。另有 recorder/可靠传输/场景重置回归。运行前不需要修改任何历史 episode 数据。

真实 UE/GPU 截图回归：设置 `SPACE_SIM_UE_CAPTURE_SMOKE=1` 和指向本机 UE 5.6 安装目录的 `UE56_ROOT` 后运行 `tests/test_capture_on_demand_renderer.py`。它使用独立端口和简单合成场景，不连接生产后端、不操作用户场景；验证空闲 0 张、两次采集的帧号/episode/双相机数量、停止后不新增图像，并解码实际 JPEG。
