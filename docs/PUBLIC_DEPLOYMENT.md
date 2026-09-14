# 一键公网部署（自动 TURN + 隧道入口）

日常只使用根目录的三个入口，不需要再填写域名、服务器 IP、证书或密码：

- **启动**：双击 `start_deployment.cmd`
- **停止**：双击 `stop_deployment.cmd`
- **查看访问链接/初始化密码**：双击 `show_deployment_access.cmd`

启动脚本会自动选择部署方式：

1. 已存在手工配置 `deploy/deployment.local.json`：沿用原配置，不做任何改动。
2. 已存在 `deploy/ip.local.json`：使用 **公网 IP 固定入口**——地址就是公网 IP，重启不变。
3. 检测到本机 eturnal 可用：使用 **TURN 增强模式**——网页/WSS 入口走隧道，媒体与手柄流量走本机公网 TURN。
4. 其他情况：退回普通 **Cloudflare Quick Tunnel 临时公网模式**。

强参数（正常使用不需要）：

```powershell
.\start_deployment.cmd -Mode Ip       # 公网 IP + Let's Encrypt IP 证书（地址固定，推荐）
.\start_deployment.cmd -Mode Turn     # 显式 TURN 增强模式（隧道网址，重启会变）
.\start_deployment.cmd -Mode Public   # 普通临时隧道
.\start_deployment.cmd -Mode Fixed    # 固定域名 + ACME（需要能签出域名证书）
.\start_deployment.cmd -Mode Direct   # 手工 deployment.local.json
```

## TURN 增强模式（默认优先）

当检测到本机 eturnal 配置可用时，启动脚本会自动完成：

1. 从 eturnal 配置中读取监听端口、公网 relay IPv4 和 REST shared secret。
2. 生成或更新 `deploy/turn.local.json`，并写入 `stun:<公网IPv4>:3478` 与 UDP/TCP 两条 `turn:<公网IPv4>:3478` 地址。
3. 把 TURN secret 用当前 Windows 账号的 DPAPI 加密保存到 `deploy/secrets/`，运行时只注入子进程；不会写入命令行、日志或访问报告。
4. 通过 Cloudflare Quick Tunnel 获取 HTTPS/WSS 入口，再启动现有 FastAPI / 信令 / MuJoCo / Basilisk / UE 平台。
5. 网关开放后从公网地址回探 `/api/health`；只有回探通过才宣告成功，失败会停止本次启动的进程并给出明确错误。

这样做的原因：WebRTC 视频与手柄控制不经过 HTTPS 隧道，而是通过 TURN 中继直连；即使浏览器与服务器之间的直连被 NAT/防火墙阻断，媒体也能经由本机 TURN 转发。

依赖条件：

| 条件 | 说明 |
| --- | --- |
| 公网 IPv4 | eturnal 配置里的 `relay_ipv4_addr`，缺失时脚本会尝试自动探测 |
| TURN 端口 | 3478 TCP/UDP 需要公网可达 |
| relay 端口段 | eturnal 默认 49152–65535 UDP，需要在云安全组/防火墙中放行 |
| eturnal 服务 | 必须是运行状态；脚本不会修改或重启 eturnal 服务 |
| 出口网络 | 需要能建立 Cloudflare Tunnel 出站连接 |

如果 eturnal 仍在使用安装默认占位 secret，脚本会打印警告但继续部署：功能可用，但管理员应尽快把 `eturnal.yml` 中的 `secret` 换成随机值，然后重新启动部署。

## 公网 IP 固定入口（推荐，`-Mode Ip`）

入口地址就是本机公网 IP，例如 `https://117.50.152.105`，**重启部署不会变化**。证书由本机 Caddy 通过 Let's Encrypt 的 **IP 证书**自动签发与续期。

关键实现细节（缺一不可）：

| 要点 | 原因 |
| --- | --- |
| `profile shortlived` | Let's Encrypt 只在 short-lived profile 下签发 IP 证书（约 6 天有效期） |
| `default_sni <公网IP>` | 客户端连接 IP 时**不会发送 SNI**（RFC 6066 禁止在 SNI 中使用 IP 字面量），不设置该项时 Caddy 选不中证书，所有握手都会以 `no certificate available` 失败 |
| `storage file_system deploy/certs/acme` | 证书与账户密钥落在项目内，重启后复用，不重复签发 |
| 80 + 443 入站可达 | HTTP-01 走 80，HTTPS 与 TLS-ALPN-01 走 443 |

与域名模式的区别：IP 证书**不需要域名、不需要 DNS 配置**，而且实测在本机网络环境下可以签发成功，而 `sslip.io` 域名因 SNI 干扰会被 `Connection reset by peer` 拒绝。

运行要求：

- 证书约 6 天有效，由 Caddy 通过 ACME Renewal Info (ARI) 自动续期，**ARI 续期豁免 Let's Encrypt 全部速率限制**。
- 续期依赖本机 Caddy 持续运行；长时间关机后重新执行 `start_deployment.cmd` 会自动补签。
- 公网 IP 必须保持稳定；若运营商更换 IP，证书中的 IP 会失配，需要重新生成配置。

## 固定域名模式（显式 `-Mode Fixed`）

`-Mode Fixed` 使用 `https://<公网IPv4>.sslip.io`（或 `SPACE_SIM_FIXED_PUBLIC_HOST` 指定的自有域名），由本机 Caddy 向 Let’s Encrypt 申请证书，入口地址固定、重启不变。

它要求 80/443 的入站路径稳定，并且 ACME 验证流量能到达本机。部分云网络/运营商线路会对未备案域名的 80/443 入站做拦截，表现为证书签发失败或回探失败；如果遇到这种情况，请使用默认的 TURN 增强模式。启动脚本在固定模式启动失败时会停止本次启动的代理和平台，不会留下半开状态。

## 安全与账号

- 公网可达不等于匿名可用：登录、访问密钥、操作授权和断连清理全部保留。
- 不会因为部署在服务器上就跳过这些检查，也不会关闭 `gamepad` 权限策略。
- 访问窗口中的初始化密码默认遮挡，报告文件只保存网址和用户名分类，不保存明文密钥。
- 现有的自定义管理员密码不会被重置；默认密码仍会触发启动前检查失败，需要先在本地网页修改。

### 关闭访问密钥（不推荐）

若确实需要去掉 `?access_key=`，把配置里的 `require_access_key` 设为 `false`，然后重启部署：

```powershell
# 编辑 deploy/ip.local.json，加入 "require_access_key": false
.\start_deployment.cmd -Mode Ip -Restart
```

行为说明：

| 项目 | 关闭后的表现 |
| --- | --- |
| 网页 / WSS 入口 | 不再需要 `?access_key=`，直接访问网址即可 |
| 登录 | **仍然必需**，未登录访问 `/api/client-config`、`/api/state` 等仍返回 401 |
| 操作 WebSocket | 仍校验登录会话；无会话连接返回 403 |
| 流媒体 JWT | 不受影响，仍按 Streamer ID 签发短时令牌 |

**风险**：公网 IP 一旦被扫描到，任何人打开网址即可看到登录页，安全只剩账号密码一道防线。缺省值始终是 `true`；配置里没有该项时按 `true` 处理。

### 关闭启动时的凭据弹窗

每次成功启动都会弹出凭据窗口，用于展示含访问密钥的链接和初始化密码。不需要它时，把配置里的 `show_access_window` 设为 `false`：

```powershell
# 编辑 deploy/ip.local.json，加入 "show_access_window": false
.\start_deployment.cmd -Mode Ip -Restart
```

关闭后终端仍会打印入口地址与安全警告，只是不再弹出窗口；随时可以运行 `show_deployment_access.cmd` 手动打开。该项缺省为 `true`。

## 限制与边界

| 项目 | 当前能力/边界 |
| --- | --- |
| 网页入口 | `-Mode Ip` 使用公网 IP + IP 证书，重启不变；TURN/隧道模式使用 Cloudflare 临时域名，重启会变化 |
| 固定域名 | 仅在显式 `-Mode Fixed` 且 80/443 ACME 可用时提供 |
| WebRTC 视频 | 已自动配置 STUN/TURN；仍建议在真实用户网络和浏览器中验收 |
| TURN 中继 | 使用本机 eturnal，3478 与 relay 端口段必须放行 |
| 多人使用 | 当前仍是单仿真实例/单活动操作页，不实现每人独立仿真 |
| 进程运行 | 沿用当前 Windows 登录会话，不是开机服务，不保证退出 RDP 后 UE GPU 会话继续正常 |
| 敏感数据 | 公网部署前请确认所在单位的数据与网络安全要求 |

## 与旧部署的关系

- `deploy/deployment.local.json` 仍然有效，`-Mode Direct` 不会改变手工配置。
- `deploy/public.local.json` 仍然保留，用于普通临时隧道模式。
- 不引入参考仓库的 MySQL、Prisma、Center/Host-Agent 结构，现有 FastAPI、SQLite、MuJoCo/Basilisk、UE 适配器、手柄和 SARM 场景逻辑保持不变。

## 高级选项

```powershell
.\start_deployment.cmd -ValidateOnly -Mode Turn -NonInteractive   # 只验证配置，不创建文件、不启停服务
.\start_deployment.cmd -Restart                                    # 强制重启当前部署
.\start_deployment.cmd -Mode Turn -CondaRoot 'D:\miniconda3'      # 指定 Conda 安装根目录
```

环境变量覆盖（用于特殊网络或测试，正常使用不需要）：

- `SPACE_SIM_PUBLIC_IP`：强制指定 TURN 使用的公网 IPv4。
- `SPACE_SIM_FIXED_PUBLIC_HOST`：固定域名模式下覆盖为已经解析到本机的自有域名。
- `SPACE_SIM_ETURNAL_CONFIG`：指定 eturnal 配置文件路径。
- `SPACE_SIM_CLOUDFLARED_EXE`：指定自行准备的可信 cloudflared。

诊断文件：

- `logs/deployment-proxy.*.log`：固定域名模式的 Caddy 日志。
- `logs/public-tunnel-*.log`、`logs/public-proxy.*.log`：隧道与入口代理日志。
- `run/public-access.json`：当前入口和账号分类，不含明文秘密。
- `deploy/secrets/`：DPAPI 加密秘密，不要删除或外传。

任何故障日志在分享前都应先隐藏访问密钥、令牌和 URL 查询串。
