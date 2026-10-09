# 视频帧率、编码质量配置与验证

## 范围

本次仅修改视频链路：主视口保持 1280×720，两路模型相机保持 640×360；
不修改动力学、30 Hz 渲染状态发布、轨道、IK、10 Hz 数据集采集及默认关闭采集的行为。

`deploy/*.json` 增加可选 `preview_fps`（整数 1～120，缺省 90）。
数据流为部署 JSON → `PreviewRate` → 后端 runtime 配置 → 场景启动器 → UE，
`/api/client-config.pixel_streaming_fps` 同时给浏览器 SDK 相同目标。
主视口的 `t.MaxFPS` 和 `PixelStreamingWebRTCFps` 跟随配置；附加相机用
`SetStreamFPS` 显式设置同一帧率，不再被监督脚本限制到 30。

本功能同时修改服务端仓库与配套 UE 适配器。迁移到另一台机器时必须同步两者，
并重新编译 `BskUnrealRendererEditor`；只复制部署 JSON 不会获得新的采集调度。

## 防止长时间推流后过度压缩（2026-09-17）

部署 JSON 现在还支持 `encoder_min_quality`：**整数 0～100，缺省 60**。
`deploy/deployment.example.json` 和本机的 `fixed.local.json`、`ip.local.json` 都设为 60；
自动启动入口优先使用 IP 配置时，不会漏掉这项设置。旧 JSON 未写该键时同样取 60。
固定地址配置重新生成时保留用户选定值，包括用于回退的 0。

```json
{
  "preview_fps": 90,
  "encoder_min_quality": 60
}
```

质量下限经部署脚本 → `run_platform.ps1` → 后端
`--runtime-encoder-min-quality` → 场景启动器 → 配套适配器 `start_renderer.ps1`，
最终转成 UE 5.6 Pixel Streaming 2 的 **`-PixelStreamingEncoderMinQuality=60`**。
`/api/client-config.pixel_streaming_encoder_min_quality` 给浏览器 SDK 同一个
`NumericParameters.MinQuality` 初始值；显式的 0 不会被前端缺省值覆盖。
`run/platform.json` 和新场景的 `run/scene_runtime.json` 也记录此项，便于检查生效配置。
此增量只改启动参数与前端，不要求额外编译 UE C++；之前的帧率调度修改仍需相应编译产物。

这是**编码质量下限**，不是“60% 的画面保真度”、QP=60 或固定码率。
本次不强制 TargetBitrate，不抬高 WebRTC 的最低/最高码率，不改变
90 FPS 目标、主视口 1280×720、模型相机 640×360、动力学或数据集采集。
提高质量下限可能需要更多带宽；带宽或算力不足时仍可能丢帧、降帧或增大延迟，
不能同时无条件保证公网清晰度与 60+ 实际显示 FPS。
60 是待实测的起点，不是已证实适用于所有网络的最佳值。

网页统计栏新增 **平均 QP**，采用相邻采样的 `ΔqpSum / ΔframesDecoded`，
不会用全会话平均值掩盖最近的压缩变化。同一编码器下一般 QP 越高压缩越强，
但不能跨编码器直接比较，也不要与 0～100 的 MinQuality 混淆。
不支持该统计、首个采样、无新增解码帧、计数回退或切换流/编解码器时显示 `—`；
断线重连会清空基线。较窄窗口可以横向滚动统计栏查看全部指标。

**生效与回退：** 本次修改不会自动重启平台或现有场景。
在方便中断时重启平台、重新启动场景，再刷新网页，才会使用全链路的新配置。
要恢复 UE 原先不限制最低质量的策略，在实际使用的部署 JSON 中设置
`"encoder_min_quality": 0`，再按同样流程重启。
这只回退质量下限，不回退已有帧率修复。

**如何判断是否有效：** 在同一路视频持续运行并移动视角时，同时观察接收帧率、
显示帧率、码率、丢包、平均 QP 和分辨率。若变糊伴随 QP 上升或码率下降，
符合编码压缩加重的表现，但单独一个统计量不能证明根因；若 QP/码率稳定却只在
特定表面或运动中模糊，还需要检查 UE 源画面、纹理加载、抗锯齿或运动模糊。
下面原有的本机帧率测试结果不能当作本次质量修改后的公网长时验收结果。

## 采集和排期

最近的 UE 5.6.1 日志中曾持续出现 MediaCapture 的 `No output frame set!` 和
`Failed to obtain a produce buffer.`。启动器现在显式设置
`-PixelStreamingUseMediaCapture=false`，选择 UE 原生 RHI/GPU-fence 采集路径。
这是绕开出错路径的措施，不是对引擎内部故障根因的完整证明；没有修改已安装引擎源码。

附加相机的画中画和独立推流仍共用一张 RenderTarget。其预览采集使用单调墙钟，
按上一次目标时间推进期限，避免 `next = now + period` 在抖动下使 90 Hz 更新近似减半。
落后时跳过错过的期限，不在单帧补采多次。权威采集仍按仿真时间单独排期。

采集还按实际订阅者分配预算：浏览器选中的相机按完整目标帧率更新；
未选中的相机只在主视口有人观看时为 HUD 小窗以最多 15 Hz 更新；
没有相关观众则跳过预览采集。切换到该相机立即恢复完整目标，
不是将选中的视频降成 15 帧。这样避免为没人看的视角持续做 90 Hz SceneCapture。
仅在 `RenderOffscreen` 服务端模式下，当无人订阅主视口而有人观看模型相机时，
暂停无人观看的主视口三维渲染；相机 SceneCapture 仍运行。重新订阅主视口即恢复，
离开场景会还原原来的 viewport 设置。不会关闭本地窗口模式的世界渲染。
这些优化不跳过权威数据集采集。

`-BskVideoDiagnostics` 是 UE 启动时可选的诊断参数，每五秒记录实际 Game Thread
节拍、场景 Tick 工作耗时、逐相机新画面采集率与订阅状态；正常运行不打印这些统计。
UE 自动化 `BskUnreal.Preview.FramePacing` 覆盖 60/90/120 Hz 抖动、停顿和不补帧突发。

没有打开独立定时器重复发送旧图来制造“高 FPS”；`DecoupleFramerate=false`，
相机 streamer 保持 coupled 模式。浏览器分别展示完整接收视频帧计数差分和
播放计数减丢弃计数的差分；零帧就是零，不以目标值填充。

## 验证工具

只针对隔离测试实例，不能指向正在操作的生产场景。测试会登录并依次选择各路相机，
不启动/停止/重置场景，不发机械臂命令。需要 Node 22+ 和本机 Edge：

```powershell
$env:BSK_STREAM_TEST_URL = 'http://127.0.0.1:58000'
$env:BSK_STREAM_TEST_USER = 'fps-test'
$env:BSK_STREAM_TEST_PASSWORD = '<隔离测试账号密码>'
$env:BSK_STREAM_TEST_SECONDS = '180'
$env:BSK_STREAM_TEST_WARMUP = '15'
$env:BSK_STREAM_TEST_CYCLES = '2'  # 包含切回主视口的复测
$env:BSK_STREAM_TEST_OUTPUT = 'run/stream-fps'
node scripts/verify_stream_fps.mjs
```

输出 `fps-results.json`：各路逐秒接收/解码/显示帧率、最小值、5%分位和均值，
码率、丢包增量、RTT、卡死计数增量和视频尺寸。每路暖机后每个采样窗口的
接收/解码帧率均需 ≥60 才通过；缺连接窗口不能被静默丢弃。
测试没有关闭浏览器帧率或 vsync 限制来抬高显示数字。

本机通过只说明这台机器和该测试链路；公网带宽、丢包、TURN 路径、客户端解码负载和
屏幕刷新率仍需在用户电脑验收。不要把“收到 90 帧”当成“屏幕显示 90 帧”，
也不要把 30 Hz 仿真状态插值到 90 帧当成增加了物理采样。


## 2026-09-17 本机隔离验收

使用真实 SARM 动力学进程、UE 5.6.1、真实浏览器 WebRTC 播放器，loopback 网络，
单浏览器依次订阅主视口、总览、腕部，两轮切换，每路每轮暖机 15 秒、采样 90 秒。
共 540 个约一秒窗口；不将暖机和视角协商阶段算入稳态达标窗口。

| 画面 | 接收均值 FPS | 接收最低 FPS | 解码最低 FPS | 屏幕显示计数均值 FPS |
|---|---:|---:|---:|---:|
| 主视口 | 84.3 | 74.6 | 74.6 | 56.5 |
| 卫星总览 | 90.0 | 88.1 | 88.1 | 76.6 |
| 腕部相机 | 90.0 | 88.6 | 87.6 | 76.4 |

全部接收/解码采样窗口 ≥60 FPS，持续运行和切回主视口验证通过；
`Failed to obtain a produce buffer.` 和 `No output frame set!` 均为 0。
UE 原生相机诊断同时观测到约 90 Hz 的实际 SceneCapture，并非只提高发送数字。

**屏幕显示帧率未达到全程 ≥60，不能宣称端到端显示已经稳定 60。**
本机 headless Edge 的播放计数和合成节奏受客户端环境限制，表内显示统计如实保留。
公网链路、多客户端并发、手动转动自由相机及更重场景也不在此吞吐验收保证范围内。

本地证据：`run/preview-90fps-20260917/verification-summary.json`、
`soak-priority/fps-results.json`、`soak-priority/UE.log` 和每路切换截图。
服务端配置/部署/场景相关测试 122 项、前端测试 70 项、UE 自动化 24 项通过。
正式平台未启动，隔离测试进程已退出；下次通过原启动器启动平台/场景即可使用新设置。
