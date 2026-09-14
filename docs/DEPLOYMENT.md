# 本地浏览器 → 远程 Windows 仿真机部署
> **免填写公网入口已提供**：首次没有 `deploy/deployment.local.json` 时，双击 `start_deployment.cmd` 默认自动分配临时公网 HTTPS 网址，并自动生成登录信息，不再进入本文的地址/证书向导。详见 [一键临时公网发布](PUBLIC_DEPLOYMENT.md)。
>
> **下文保留的是固定域名/直接 HTTPS 模式**，已有配置继续兼容；需要首次手动配置该模式时执行 `start_deployment.cmd -Mode Direct`。不要把临时隧道的 `deploy/public.local.json` 传给底层 `deploy_platform.ps1`。

本指南保留现有 Windows + UE 5.6 + Basilisk/MJScene 环境，不迁移模型、不使用 RDP 传递手柄。手柄接在运行网页的**本地电脑**上；远程桌面只用于维护。

> `scripts/deploy_platform.ps1` 默认只校验配置；只有显式传入 `-Start` 才会停止旧平台并切换部署。校验不等于已获得证书，也不等于跨网视频已连通。

## 日常使用：只运行一个入口

在**远程仿真电脑**的仓库根目录双击：

- **`start_deployment.cmd`**：启动部署。自动找到 PowerShell 7、加载已有 `space-sim-server` Conda 环境、恢复密钥，并调用底层部署脚本。
- **`stop_deployment.cmd`**：停止平台、场景、串流和本项目的 HTTPS 代理。停止不需要 Conda、Caddy 路径、配置文件或密钥仍然可用。

也可直接在终端运行同名 `.cmd`，不必再手动执行环境激活、复制 JSON、生成密钥等多条命令。入口不依赖终端的当前工作目录。它复用现有 Python/Node/UE/仿真环境，不会自动安装另一套仿真环境。

### 第一次双击会发生什么

1. 如果没有 `deploy/deployment.local.json`，询问**平台对外提供的统一 HTTPS 网址（不是访问者 IP 白名单）**及证书方式，验证后保存。不要填一个无法解析/访问的任意域名。选择 `manual` 时还会询问证书路径。
2. 询问并确认管理员初始化密码；已有账号不会被自动重置。如果检测到现有管理员仍使用默认密码，会在停止任何服务前拒绝切换，请先在现有本机网页修改密码，再双击启动。
3. 自动生成两份随机串流密钥，将密码/密钥以 Windows DPAPI 加密的 CLIXML 保存到 `deploy/secrets/`（Git 忽略），按配置文件路径隔离。以后自动恢复，**不会每次生成新密钥**。已有进程环境变量可用于首次导入；已有加密存储优先于环境变量，避免意外轮换。
4. 如果没有 Caddy，下载 `deploy/caddy-release.json` 固定的官方 Windows 版本，验证仓库中固定的 SHA-512，再仅提取 `caddy.exe` 到 `run/deployment-tools/`。已配置的自定义路径若无效会报错，不会擅自替换。首次下载需要联网；失败可以重试，或手动提供 `caddy_executable`。
5. 完成预检后，调用底层脚本切换平台并启动 HTTPS 代理。**首次切换会停止现有平台/场景。** 在完整的已记录部署仍运行且配置/密钥未变时，重复双击不会重启场景；入口共享启动/停止互斥锁，防止重复点击竞争。
6. 显示网页入口。可输入 `C` 将**含访问密钥的完整链接**复制到远程电脑剪贴板，再粘贴到**本地电脑浏览器**。不在日志或控制台中打印密钥。复制需要剪贴板可用；剪贴板/历史属于敏感位置，用后清理，不要分享链接。

DPAPI 文件只能由同一台电脑、同一个 Windows 账号正常解密。换账号/换机器需要通过受控方式迁移秘密并重新初始化存储；文件损坏或解密失败时脚本不会静默删除并换一套密钥。不要删除 `deploy/secrets/` 来排障，也不要提交这些文件。更换配置文件路径也需要重新配置对应的秘密存储。

**服务器脚本不能代替本地网络/证书配置：** `acme` 仍需正确的域名解析及证书验证网络条件；`internal` 仍需在本地操作电脑信任 Caddy 的 CA 根证书；跨网视频可能需要实际可工作的 TURN。启动进程不等于这些条件已经满足。

### 面向公众使用：网址与用户 IP 是两回事

`public_url` 是**用户在地址栏打开的平台网址**，不是允许访问的用户 IP。不同地点、不同运营商、不同客户端 IP 的用户都使用同一个网址，不需要逐个登记客户端地址。

```text
用户 A（任意客户端 IP） ─┐
用户 B（任意客户端 IP） ─┼─→ 平台统一 HTTPS 网址 → 公网入口 → 仿真机
用户 C（任意客户端 IP） ─┘
```

- 当前项目没有基于 `public_url` 建立客户端 IP 白名单。`allowed_origins` 校验的是网页来源（协议、站点名、端口），不是用户电脑的地址；不要为了支持不同客户端 IP 把它改成 `*`。
- 公开入口需要实际的网络部署：例如域名解析到公网可达的服务器/网关，或另行配置隧道及 HTTPS 入口。私网地址不会因为启动程序或监听 `0.0.0.0` 就自动对互联网可达；`0.0.0.0` 也不是发给用户的网站地址。
- 公共域名可使用 `acme` 获取证书；`internal` 仅适合能管理客户端根证书信任的内网/VPN 环境，不适合作为面向公众的免配置浏览器入口。公网入口存在也不代表 WebRTC 视频一定可达，可能仍需 TURN。
- “允许来自不同 IP 的用户访问”不等于取消登录、访问密钥和操作权限，也不等于每个用户都有独立仿真实例。当前仍是单实例/单活动操作页；自助注册、公众体验流程、多用户独立仿真是另外的产品与架构工作。

一键入口负责在已有机器上启动服务，**不负责购买域名、分配公网 IP、开通隧道或完成公网网络配置**。首次指定网址后，日常仍只需启动/停止两个入口，不需要随着用户 IP 变化重新配置。

### 可选的检查和强制重启

```powershell
# 只读检查：可读取已保存的密钥；不生成配置/密钥、不下载、不启停服务。
.\start_deployment.cmd -ValidateOnly

# 明确重新部署，即使已有部署仍在运行；会停止当前场景。
.\start_deployment.cmd -Restart
```

`-NonInteractive` 可用于已有配置的自动化启动：不会询问输入或复制剪贴板。缺少管理员/TURN 秘密或首次配置时会报错；停止入口不要求秘密。非标准 Conda 安装位置可用 `-CondaRoot` 或 `SPACE_SIM_CONDA_ROOT` 指定。

> 以下是底层部署脚本的架构与**高级手动配置**说明。使用上面的桌面入口时，不需要逐条手动执行这些准备命令。底层 `scripts/deploy_platform.ps1` 仍保留默认只校验、显式 `-Start` 才切换的行为。

## 1. 架构与边界

```text
本地浏览器（键盘 / 标准 Gamepad API）
  ├─ HTTPS 网页/API、WSS /ws/operator → Caddy → 127.0.0.1:8000
  ├─ WSS /stream                    → Caddy → 127.0.0.1:8080
  └─ WebRTC 视频/数据通道            ↔ UE 直连或经 TURN 中继

远程机内部：UE → 127.0.0.1:8888（信令）
             仿真控制 8766、采集 8767、渲染桥 5558
```

- `/stream` 和 `/ws/operator` 是不同连接，均保留原始路径；Caddy 原生处理 WebSocket 升级。
- 不要把所有 `127.0.0.1` 改成公网 IP。API、信令和仿真内部端口不需要直接对外开放。
- 网页入口用 HTTPS；浏览器控制连接自动用 WSS。前后端同源，不需要放开通配 CORS。
- 本版本仍是**单实例、单后端进程、单活动操作页**；不要加 Uvicorn 多 worker，也不要部署多个副本争用仿真端口。
- 不自动更改 DNS、hosts、防火墙或证书信任，不自动购买/创建 TURN 服务，不自动安装 Windows 服务。

## 2. 前提

1. 原有 `scripts/run_platform.ps1` 在这台机器已能正常运行。
2. 使用 PowerShell 7，激活已有的 `space-sim-server` 环境，确保 `python`、`conda`、`npm.cmd`、`node.exe` 可用。仿真环境继续沿用现有配置。
3. 手动使用底层脚本时，安装官方 Caddy 2，放入 PATH，或在配置里填写其绝对路径；桌面入口会自动准备缺少的 Caddy。
4. 确定本地电脑能够访问的主机名，以及直连/VPN/公网路线。推荐先在团队内网或 VPN 验收。
5. **先在当前页面修改已有管理员的默认密码。** 初始化密码参数不会更新现有数据库；HTTPS 模式发现任何管理员仍使用文档中的本机默认密码时会拒绝启动。

## 3. 创建本机配置

在仓库根目录执行一次（不要覆盖已有的本机配置）：

```powershell
Copy-Item -LiteralPath .\deploy\deployment.example.json -Destination .\deploy\deployment.local.json
```

编辑 `deploy/deployment.local.json`：

| 字段 | 含义 |
| --- | --- |
| `public_url` | 所有用户访问的统一 HTTPS origin，不是客户端 IP 白名单；填写你实际配置的域名，可含端口，但不能带路径、查询参数或账号密码 |
| `tls_mode` | `acme` / `internal` / `manual`，见下节 |
| `caddy_executable` | `caddy.exe`（PATH 中），或完整文件路径 |
| `adapter_root` / `model_root` / `unreal_root` | 空字符串沿用现有自动发现；非空相对路径以仓库根目录为基准 |
| 六个 `*_port` | 内部监听端口，通常保持默认；不能重复或等于外部 HTTPS 端口 |
| `admin_username` | 初始化管理员用户名；已有账号不会因此改名 |
| `ice_servers` | STUN 地址数组，例如 `[{"urls":"stun:your-server:3478"}]` |
| `turn_urls` | TURN 地址数组，例如 `["turn:your-server:3478?transport=udp"]` |

`sim.example.com` 是模板占位符，校验会拒绝未修改的模板。所有 `*.local.json` 已加入 Git 忽略列表。

### TLS 方式

- **`acme`**：适用于你控制的、可签发公共证书的域名。正确配置 DNS 与证书验证所需网络访问；Caddy 管理证书续期。默认会使用 HTTP 80 做重定向/证书验证及 HTTPS 端口（通常 443）。
- **`internal`**：适合内网/VPN 测试。Caddy 使用本地 CA；必须由你将该 CA 的**根证书**可信地导入本地操作电脑。只在服务器信任证书不够；不要将“忽略证书错误”当作部署方案。不要分发 CA 私钥。
- **`manual`**：填写 `certificate_file` 和 `certificate_key_file`，采用已有可信证书（如组织内部 PKI），由你维护证书续期。文件内容不会写入生成的配置，但路径会出现其中。

不要以 Caddy 进程存在为证书可用的证据。用本地浏览器实际访问，确认没有证书警告。Caddy 的证书数据通常在运行账号的应用数据目录下；更换运行账号前要规划证书存储与私钥保护。

## 4. 配置进程环境中的秘密

在用于启动部署的同一个 PowerShell 窗口中设置：

```powershell
# 输入至少 12 字符的非默认初始化密码，不把密码写进命令历史。
$env:SPACE_SIM_ADMIN_PASSWORD = Read-Host '管理员初始化密码' -MaskInput

# 首次生成。正式使用时请保存在受控的秘密存储中，并在后续启动时恢复同一值。
$env:SPACE_SIM_STREAM_JWT_SECRET = [Convert]::ToHexString([Security.Cryptography.RandomNumberGenerator]::GetBytes(48))
$env:SPACE_SIM_STREAM_ACCESS_KEY = [Convert]::ToHexString([Security.Cryptography.RandomNumberGenerator]::GetBytes(24))

# 仅配置 TURN 时需要：必须与 TURN 服务的 REST shared secret 匹配，不能随便生成后只填一端。
# $env:SPACE_SIM_TURN_AUTH_SECRET = Read-Host 'TURN REST shared secret' -MaskInput
```

这些变量仅在当前进程及其子进程中有效。项目**不会自动加载 `.env`**。不要提交秘密、私钥或完整的访问 URL；部署脚本不会把密码/JWT 密钥放到子进程命令行，也不打印访问密钥。环境变量并非秘密保险箱，仍需保护远程机器账号。

密钥变化后旧访问链接和旧 JWT 会失效；同时重启后端与信令，不要让两端使用不同密钥。

## 5. 校验，再明确切换

```powershell
# 安全检查：不写文件、不停服务、不发运动指令。
.\scripts\deploy_platform.ps1 -ValidateOnly

# 仅在实验已停止、可以切换平台时执行；会重建前端并重启平台。
.\scripts\deploy_platform.ps1 -Start
```

启动时会先检查配置、秘密与 Caddy 配置，再复用现有平台启动逻辑。生成配置保存在 `run/deployment/Caddyfile`；代理 PID/启动时间保存在 `run/deployment.json`。自动启动的进程隐藏运行，不额外弹出终端窗口。

启动并不自动创建仿真场景。用本地浏览器打开以下形式的地址（把占位符换成实际密钥）：

```text
https://你的主机名/?access_key=<SPACE_SIM_STREAM_ACCESS_KEY 的值>
```

登录、选择场景、点击启动。完整 URL 属于敏感信息。页面/API/信令都经过同一 HTTPS origin，不能混用 IP 地址、另一个域名或 HTTP 入口，否则 Cookie/Origin 策略可能拒绝连接。

停止整个部署：

```powershell
.\scripts\stop_platform.ps1
```

脚本只按记录的 PID 与启动时间停止本项目的 Caddy，不会直接按进程名杀掉其他 Caddy 实例。还原本机模式时重新运行原来的 `scripts/run_platform.ps1` 即可；不要把本机 HTTP 模式直接暴露公网。

## 6. 视频、端口和安全

| 端口/连接 | 建议可达范围 |
| --- | --- |
| HTTPS（通常 TCP 443）及证书/重定向用的 TCP 80 | 本地操作电脑能访问，按团队边界限制来源 |
| 8000、8080 | 部署模式只绑定远程机 loopback，由 Caddy 代理 |
| 8888、8766、8767 | 继续限制为远程机内部 |
| 5558 渲染桥 | 不对外开放；同时检查 UE 适配器的监听方式与主机防火墙 |
| WebRTC / TURN / TURN 中继范围 | 按选定网络拓扑及 TURN 服务配置单独放行，不等同于网页端口 |

- 网页正常但黑屏：分别检查 `/stream` 的 WebSocket、ICE 连接、UDP 可达性和 TURN 中继。STUN 不是视频中继，也不保证所有网络可直连。
- 信令已修复 UE 5.6 SDK 消息工厂丢弃 `iceServers` 的问题；浏览器与 UE 端的实际配置消息均有回归测试。
- TURN 必须是可工作的外部服务，配置 URL 不会部署 TURN。使用认证及临时凭据，不要搭开放中继。
- HTTPS 登录 Cookie 使用 `Secure`、`HttpOnly`、`SameSite=Strict`，API 不缓存。浏览器写请求和控制/播放器 WebSocket 校验明确的 Origin。
- 非浏览器 API 客户端进行写请求时也要提供匹配的 `Origin`，并处理登录会话与访问密钥；Origin 不是认证的替代品。
- 登录按可信客户端地址限流（默认每分钟 10 次，成功/失败都计数）。只信任 loopback 代理转发头；不配置 `*`。当前限流为单进程、有界内存实现，不是 DDoS 防护。
- 部署默认关闭后端访问日志和 Caddy 访问日志，避免记录 URL 中的凭据。排错时不要开启会泄露完整请求 URL 的调试日志；分享日志前仍应脱敏。
- 内部服务不负责 TLS；不要将它们绕过反向代理直接暴露到公网。

## 7. 手柄验收（本地浏览器）

右侧“实时指令”新增“手柄诊断”，即使没进入操作模式，也会在登录后显示：

- 设备名称、索引和映射；原始摇杆轴、按下的按钮；
- 未发现设备、API 不可用、权限被阻止、等待回中、映射待适配；
- 场景未就绪、无权限、未进入操作、自由相机模式、急停等阻断原因。

步骤：
1. 在本地电脑连接手柄，直接打开部署网页，保持前台；按一下按钮再释放。
2. 观察设备及轴值是否变化。新接入/重新接入的手柄须回中、释放扳机及控制按钮后才会输出。
3. 当前只自动控制浏览器标记为 `standard` 的完整双摇杆映射。非标准设备会显示原始输入但不猜测机械臂映射；可以切换设备的标准/XInput 工作模式，或后续新增适配。
4. 降低页面速度，点击视频进入操作模式，再小幅推杆；保留已有 Esc、页面停止按钮和 250 ms 看门狗。
5. 分别验证释放输入、拔出手柄、网页失焦、网络断开。按设计应停止继续推进目标；不应把它误认为清零物理速度或硬件急停。
6. 再验证重新连接时不会自动恢复旧运动。断开 RDP 后检查 UE 渲染是否持续；脚本不是 Windows Session 0/GPU 无人值守部署的保证。

只读控制台诊断：`window.__gamepadDiagnostics()`。此接口不发送运动、不授予控制权。

## 8. 数据与运维

- 保留并备份 `data/auth.sqlite3`、场景实例、采集数据；备份 SQLite 时停写或使用 SQLite backup API，不要只复制正在写入的主数据库文件。
- 保护部署账号和证书数据目录，设置日志/采集文件轮转与磁盘容量告警。
- 当前交付为可配置的单机部署，不包含自动服务安装、跨机器调度、多租户强隔离或网络质量保证。网络、真实手柄、GPU/视频需要实际环境验收。

## 9. 回归检查

在已安装测试依赖的后端环境中：

```powershell
python -m pytest tests/test_web_security.py tests/test_deployment_scripts.py tests/test_app.py tests/test_operator_sessions.py tests/test_stream_access.py tests/test_safety.py -q
npm.cmd --prefix frontend test
npm.cmd --prefix signalling test
```

配置测试不执行 `-Start`。信令测试只启动随机 loopback 端口上的独立测试进程，并在结束后清理；不连接正在运行的 UE，不发送机械臂动作。

官方参考：Caddy 的 reverse_proxy/TLS 文档、FastAPI Behind a Proxy、Epic UE 5.6 Pixel Streaming Hosting and Networking Guide。
