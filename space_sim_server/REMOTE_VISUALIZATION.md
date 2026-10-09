# 从自己的电脑操作远端仿真

## 使用方法

1. 在服务器上双击 `start_remote_visualization.cmd`。首次启动会将本机账号复制到部署使用的 `data/auth.sqlite3`，不会覆盖已有部署数据库，也不会重置自定义密码。
2. 在你自己电脑的 Edge/Chrome 直接打开并收藏 `https://117.50.185.8/`，不需要复制带密钥的链接，也不需要打开远程桌面的浏览器。
3. 使用原账号和密码登录。当前部署已按要求关闭额外的 `access_key` 门槛，未关闭账号认证、操作授权或视频信令令牌校验。
4. 点击“生成并启动场景”，等待“仿真在线”和 `LIVE WEBRTC`，点击画面进入操控。仿真、UE、场景资源都在服务器上运行，本机只需浏览器。
5. 网页“停止场景”保留平台入口，下次直接打开同一网址。服务器上的 `stop_remote_visualization.cmd` 会停止整个平台和 HTTPS 入口；需要恢复时双击启动脚本，网址不变。

当前采用原仓库的公网 IP 模式，入口为 `https://117.50.185.8/`。普通平台重启不再更换网址；前提是云平台保持这个公网 IP。不要将旧的 `127.0.0.1:18000` 或 trycloudflare 临时网址用于当前模式。

`deploy/ip.local.json` 中 `require_access_key=false`、`show_access_window=false` 已持久保存；固定部署启动器会保留这些值，不会在重启时恢复额外密钥要求。`show_remote_access.cmd` 只作为可选的地址查看入口，不再是使用步骤。旧标签页请使用 Ctrl+Shift+R 强制刷新，再登录。

## 当前部署

- 配置：`deploy/ip.local.json`，按现有忽略规则不提交。远程启动器优先使用此文件，原 `deploy/remote.local.json` 作为临时隧道备用配置保留，不同时启动。
- 预览上限 90 FPS，网页可切换 30/60/90 FPS，主视口 1280×720；物理仿真倍速、步长和默认关闭的数据采集不变。帧率选项影响共享 UE 实例，并非每个观看者独立设置；重新加载网页默认使用部署上限。
- 公网视频优先流畅：`deploy/ip.local.json` 的 `encoder_min_quality=35`，允许网络不足时更强压缩；适配器 `Unreal/BskUnrealRenderer/Config/DefaultEngine.ini` 的 `PixelStreaming2.WebRTC.MaxBitrate=8000000` 将自适应视频码率上限从引擎默认 40 Mbps 降至 8 Mbps，不强制固定编码码率，也不提高最低码率。此上限不是网卡总流量上限，多观看者、多个视频轨道和 TURN 会增加总流量。修改部署设置需要重启平台，修改 UE 配置需要新建场景。
- 90 FPS 是目标，不保证外网接收/显示达到 90；60 Hz 屏幕不能显示每秒 90 个独立画面。卡顿时先选 60 FPS，继续卡顿选 30 FPS，并点击“复制诊断”收集最近 60 次真实接收/显示帧率、区间丢包、RTT、码率、QP 和 ICE 路径类型；不包含 IP、账号或令牌。诊断在视频重连/换相机后清空，避免混用计数器。
- 接收有帧但画面不动时，查看新增的“解码帧率”和“播放状态”：暂停会提示继续播放；前台页面持续接收但 5 秒没有新解码帧，会提示解码停滞，并尝试请求关键帧（每次停滞最多两次，间隔至少 15 秒）；解码继续而媒体时间不前进时提示播放停滞。无法恢复时点击画面上的重连按钮，只重建浏览器视频连接，不停止场景。后台标签页不会触发关键帧恢复。诊断同时包含播放/暂停、readyState、媒体错误代码、解码器信息和关键帧计数；“显示帧率”仍是播放质量计数估算，不能单凭零值判定解码失败。
- 保留 `PixelStreamingDecoupleFramerate=false`，不靠重复发送旧帧增加数字。UE 场景控制器在游戏线程把 WebRTC 帧率请求同步到 `t.MaxFPS`，不超过启动参数设定的上限；否则 UE 5.6 的耦合捕获路径会忽略网页的降档请求，选择 30/60 仍实际发 90 帧。此修复需要编译适配器的 `BskUnrealRuntime`，未修改引擎源码或开放任意控制台命令。降档也会降低 UE 预览/相机捕获的可用刷新率，不改变独立仿真的物理步长；严格采集任务请按既定采集配置使用。
- 网页、API、操作 WebSocket 和视频信令经过本机 Caddy 的 HTTPS/WSS 入口，不再经过 Cloudflare 隧道。
- WebRTC 尝试浏览器与 UE 直连，也可使用 `117.50.185.8:3478` 的 STUN/TURN；已配置 UDP 和 TCP 两种 TURN 客户端连接方式，媒体中继 UDP 端口限定为 50000–50199。
- eturnal 1.12.3 安装在 `C:\Program Files\eturnal`，作为自动启动的 Windows 服务运行。TURN shared secret 已随机生成；配置文件限制为 SYSTEM/管理员访问，Erlang 管理端口只监听回环。安装器在 `sys.config` 写入的 UTF-8 BOM 导致启动失败，已移除；升级安装后应重新检查。
- Caddy 使用正式 IP 证书，存储在忽略的 `deploy/certs/acme/`，需保持平台运行以便自动续期。停止平台不卸载或停止 TURN 服务；需要停用 TURN 时使用 Windows 服务管理器。
- 后端 18000、信令 8080/8888、控制 18766、采集 18767 保持回环监听。没有为这些端口添加公网放行规则，也没有修改系统防火墙策略。
- Caddy 使用仓库固定版本并校验哈希，安装在忽略的 `run/deployment-tools/`，不是系统服务。cloudflared 仅保留为备用，当前不运行。eturnal 安装器来自官方 HTTPS 下载目录，SHA256 为 `e42c0881c5636ff1a6d53bfbc526c77274f3699738ac148ab4a82856b1be3903`；厂商未提供独立校验文件，安装器无 Authenticode 签名，该摘要仅作本机安装记录。
- 使用已有 mujoco-dev Python，补齐了后端依赖。PowerShell 7 和 UE 5.6 沿用已有安装。
- 原账号与自定义密码保持不变。移除的是额外共享链接密钥，不是允许匿名使用：匿名用户仍不能读取运行配置、控制场景或获取视频令牌。TURN/视频签名密钥与部署凭据仍通过 Windows DPAPI 加密保存在忽略的 `deploy/secrets/`，不要上传或分享这些文件。账号密码现在是用户进入平台的主要认证凭据，请保密。
- 为避免 Uvicorn 的 WebSocket INFO 日志包含访问密钥，远程无访问日志模式同时使用 warning 日志级别。
- 本机启动器现在优先使用 `data/auth.sqlite3`，以免切换入口后账号密码不一致。原 `visualization-test-auth.sqlite3` 保留为迁移前副本。

## 验证范围与网络限制

已从服务器测试公网 HTTPS/WSS 登录、场景启动、控制反馈和持续视频解码，并增加强制 TURN 中继路径验证，避免只测到同机直连。密码登录模式的回归不会携带 URL 密钥或访问密钥请求头。这仍不等于已验证用户自己的电脑和网络。

如果自己的电脑能登录、能看到仿真在线，但画面长期 CONNECTING，应检查 WebRTC ICE/NAT/防火墙连通性，不要仅提高帧率。当前已配置 TURN；还要确认客户端网络允许访问 3478 TCP/UDP，以及服务器侧 50000–50199 UDP 中继端口可达。特别受限的网络仍可能需要另行部署 TURN/TLS 入口。启动器会从本机 eturnal 配置读取共享密钥并加密保存，不要把共享密钥发送给用户。

若已有内网/VPN，也可使用项目的直接 HTTPS 部署模式，但需要另行配置可达地址、证书、Origin 和信令。不要为了省事禁用认证、证书验证或系统防火墙。

## 维护和验收

- 当前网址及账号类型报告：`run/public-access.json`；应显示 `require_access_key=false`。密码登录模式部署日志：`run/password-only-start.log`，验收日志：`run/password-only-e2e.log`。
- 本次 IP 部署日志：`run/ip-start.log`；后续双击启动的输出在启动窗口中。场景日志：`logs/scene-*.launcher.*.log`、`logs/scene-*.simulation.*.log`；HTTPS 日志：`logs/deployment-proxy.err.log`；TURN 日志：`C:\Program Files\eturnal\log\eturnal.log`。
- 空闲时运行 `scripts/verify_remote_platform.ps1` 进行实际公网入口回归：使用临时操作员测试，结束后删除该操作员，保留平台。已有场景运行时拒绝替换它。
- 该验证会产生实际仿真与短暂低速控制，不要在正在操作的场景上执行。
- 在 PowerShell 7 中设置 `$env:PIXEL_STREAMING_FORCE_RELAY='1'` 和 `$env:PIXEL_STREAMING_TURN_TRANSPORT='udp'`（或 `tcp`）再运行验证脚本，可强制浏览器走 TURN，并检查实际选中的 relay candidate 及持续解码帧；测试进程结束后清除这两个变量。该设置只作用于验证浏览器，不修改正常用户页面。
- 额外设置 `$env:PIXEL_STREAMING_FPS_TEST='1'` 可在临时测试场景上执行 90→60→30→90 FPS 切换，每档采样 20 秒，结果保存在 `run/remote-smoke-*/fps/fps-results.json`。包含实际码率上限、每秒接收/解码/显示计数差值、码率、丢包和 ICE 路径。此测试会切换相机/共享视频目标，仅限没有正在操作或采集任务时使用；通过门槛为所有采样持续连接、接收与解码最低帧率至少为目标的 80%，平均接收不超过目标的 120%，不是“稳定达到目标”的认证。
- 远程与本机启动模式不要同时运行；端口相同。切回本机前先运行远程停止脚本。
