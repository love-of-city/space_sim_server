# Linux 一键公网部署

## 公网入口（2026-10-20 更新）

当前机器已启用固定公网入口 `https://117.50.205.199`，直接打开网址并使用账号、密码登录即可，视频和控制连接自动授权，无需在网址中填写访问密钥。同源代理网页、API、操作 WebSocket 和 `/stream` WSS；后端和 UE 信令始终保留在回环地址。启动成功必须通过公网 HTTPS 健康回探和本部署标识校验，否则回收本次进程。

```bash
cd /home/lyh/space_sim_server/space_sim_server
./start_deployment.sh
./show_deployment_access.sh
./stop_deployment.sh
```

`show_deployment_access.sh` 在本机终端显示访问网址、用户名和初始密码。不要分享包含密码的输出。已有账号保留原密码；秘密保存在权限 600 的 `deploy/linux-secrets.local.json`，普通状态报告不保存秘密。公网模式使用 Secure Cookie、Origin 白名单和自动签发的短期 JWT；未登录用户无法获取视频凭据或连接控制接口。已有管理员仍用默认密码时，启动会失败。

Linux 默认 `require_access_key: false`，当前机器也使用该设置；部署启动时会清除继承自终端的旧访问密钥。只有管理员显式设置 `require_access_key: true` 才额外要求含 `access_key` 的访问链接，此兼容选项不属于普通登录流程。修改配置后运行 `./start_deployment.sh --restart` 生效。

本机出口公网 IP 已核实为 `117.50.205.199`。固定入口代码已支持 IP ACME `shortlived`、`default_sni` 和自动续期，本机已完成低端口授权。新机器的管理员可在任意目录执行一次：

```bash
sudo setcap cap_net_bind_service=+ep /home/lyh/space_sim_server/space_sim_server/run/deployment-tools/caddy-2.11.4-linux-amd64/caddy
```

确保云安全组、服务器防火墙和 NAT 的 80/TCP、443/TCP 入站指向本机，再运行 `./start_deployment.sh --mode ip --restart`。入口为 `https://117.50.205.199`，只有证书可信且公网回探通过才报告成功。把本机 JSON 的 `mode` 改为 `ip` 可保留选择。二进制升级替换后需重新 setcap。

`mode` 支持 `auto`、`public`、`ip`、`fixed`、`local`。`auto` 优先使用配置的 `public_url` 或 `public_ip`，其余使用临时隧道。`fixed` 要求 DNS 指向本机并设置 HTTPS `public_url`。`tls_mode` 默认 `acme`；手工证书使用 `manual`、`certificate_file`、`certificate_key_file`。证书持久化在 `deploy/certs/linux/`。固定入口失败不会静默换临时网址；手动回退用 `./start_deployment.sh --mode public --restart`。仅本机运行用 `--mode local`。模式覆盖只影响本次启动；`--restart` 会停止当前场景。

Linux Caddy/cloudflared 首次按仓库固定版本下载，验证 amd64/arm64 校验值。可通过 `caddy_executable`、`cloudflared_executable` 或 `SPACE_SIM_CADDY_EXE`、`SPACE_SIM_CLOUDFLARED_EXE` 使用已有工具。

### TURN 与外网视频

HTTPS 隧道不转发 WebRTC 媒体。本机已准备 `/home/lyh/space_enviroment/bin/turnserver`，通过 `managed_turn: true` 随平台启停，不修改系统服务。REST shared secret 使用随机值，仅写入权限 600 的临时配置并注入信令子进程；停止删除临时配置。信令给客户端签发限时 HMAC 凭据。

云安全组、防火墙和 NAT 必须放行 `3478 TCP/UDP` 及 `49160–49259 UDP`，映射端口保持不变。可通过 `turn_port`、`turn_relay_min_port`、`turn_relay_max_port` 修改。公网 IP 由 `public_ip` 提供；内网 relay 地址自动探测。已有 TURN 可配置 `managed_turn: false`、`turn_urls` 以及权限 600 的 `turn_secret_file` 或环境变量 `SPACE_SIM_TURN_AUTH_SECRET`，也支持标准 `ice_servers`。

本次公网网页登录、权限拒绝、Secure/HttpOnly Cookie、WSS 和三路真实 UE 连续非黑帧通过。浏览器在服务器本机，媒体选用 host 候选。TURN 经私网地址 UDP/TCP 鉴权及 relay 分配通过；经公网 IP 3478 未成功，可能受安全组或 NAT 回环限制，仍需外部电脑验收，不能等同于跨网络视频已通过。

停止会先关闭公网入口，再封存采集并关闭场景，最后退出信令和本部署 TURN。停止也能按原进程身份恢复回收崩溃监督器留下的网关、隧道和 TURN。网关 18443、管理 18444 仅监听回环。公网状态报告为 `run/linux-public-access.json`，日志为 `logs/linux-gateway.log`、`logs/linux-turn.log`、`logs/linux-tunnel-<部署标识>.log`。

下列说明中的 `127.0.0.1:18000` 和 SSH 转发适用于显式 `local` 模式；公网模式使用启动结果与 `show_deployment_access.sh` 提供的 HTTPS 访问链接。

入口位于服务端子目录 `space_sim_server/`。已有 Linux 仿真环境、Node.js 22、UE 5.6 和完整 LFS 资产时，无需再次安装 UE。首次安装环境参考 [Linux 基础迁移](LINUX_BASIC_MIGRATION.md)。

## 当前电脑

本机已创建被 Git 忽略的 `deploy/linux.local.json`，使用 `/home/lyh/space_enviroment/` 中的 Python、Node、缓存和 UE 运行副本，复用 `/home/spacesim-zmh/engines/UE_5.6`。

```bash
cd /home/lyh/space_sim_server/space_sim_server
./start_deployment.sh
```

启动命令准备前端、检查信令和原生库，然后在后台启动生产后端与 JWT 信令。命令返回后可以关闭该终端；不会因 SSH 终端退出而停止服务。重复执行会报告正在运行的地址，不重复启动或停止已有场景。

本机公网模式直接打开 `https://117.50.205.199`；其他部署可运行 `./show_deployment_access.sh` 查看网址；显式 local 模式打开 **http://127.0.0.1:18000**。用户名默认 `admin`；首次创建账号时的随机密码保存在本机 `deploy/linux-secrets.local.json` 的 `admin_password` 字段，文件权限为 600，不提交 Git。已有认证数据库保留原账号和密码，启动脚本不会重置账号。可在本机终端查看首次密码：

```bash
python3 -c 'import json; print(json.load(open("deploy/linux-secrets.local.json"))["admin_password"])'
```

登录后选择场景模板，点击“生成并启动场景”。Linux 原生场景监督器会准备对应本机资产、启动离屏 UE 和真实物理；网页可停止、重置及再次启动场景。最新 SARM 抽屉场景请选择 `sarm-task-box-20261012`，随机化配置为“指定操作姿态直接启动”。其他模板首次使用可能需要额外资产导入时间。

停止整个平台，包括当前场景、后端和信令：

```bash
./stop_deployment.sh
```

停止会先让生产后端结束采集、封存数据并关闭场景，然后关闭信令。录制较长时收尾可能需要等待。重复停止可安全执行。脚本只处理本部署记录的 UID、PID 和进程创建时间，不按程序名、端口或全局 `pkill` 杀进程。

## 远程访问

显式 `local` 模式默认只监听远程电脑的回环地址。在自己的电脑上通过 SSH 转发网页和信令端口（将主机名替换为实际 SSH 地址）：

```bash
ssh -N -L 18000:127.0.0.1:18000 -L 18080:127.0.0.1:18080 lyh@YOUR_SERVER
```

随后访问本机浏览器的 `http://127.0.0.1:18000`。SSH 的两个 TCP 转发只负责网页和信令；WebRTC 媒体还需要浏览器与服务器 ICE 候选地址之间的网络连通。跨网络/NAT 无法连接时，在本机配置的 `ice_servers` 中提供可访问的 STUN/TURN。这里的浏览器验收是服务器本机真实 WebRTC，不保证另一台电脑的网络条件。

显式 local 模式需要局域网直接访问时，可设置本机 JSON 的 `host` 为 `0.0.0.0`，停止后再启动，用服务器实际 IP 访问 API 端口；配置 `allowed_origins` 为浏览器实际使用的网页 origin。启动前会检查已有管理员是否仍使用默认密码。公网模式要求 `host` 为 `127.0.0.1`，通过本说明开头的 HTTPS 网关对外访问。

## 其他 Linux 电脑

复制 `deploy/linux.example.json` 为 `deploy/linux.local.json`，填写本机路径；或传入 `--config /absolute/path/config.json`。解释器选择顺序为 `SPACE_SIM_PYTHON`、JSON 中的 `python`、系统 `python3`。Python 必须已有项目依赖和 Basilisk；脚本不更改系统 Python。

可选配置：`model_root`、`data_root`、`cache_root`、`encoder_min_quality`、`allowed_origins`、`ice_servers` 和 `signalling_url`。`cache_root` 需指向已准备的 Basilisk 星历缓存所在 XDG 缓存根目录，避免在独立终端启动时重新下载。`signalling_url` 留空时，网页使用当前网页主机及 `player_port`。

| 端口配置 | 默认值 | 用途 |
| --- | --- | --- |
| `api_port` | 18000 | 网页、后端 API、操作 WebSocket |
| `player_port` | 18080 | 浏览器 Pixel Streaming 信令 |
| `streamer_port` | 18888 | UE 信令，回环地址 |
| `control_port` | 18766 | 权威物理控制，回环地址 |
| `capture_port` | 18767 | 双相机采集，回环地址 |
| `render_port` | 18558 | UE 渲染状态接收，回环地址 |

已占用的端口会导致明确失败，启动失败会回收本次已启动的子进程，不接管占用端口的其他程序。UE 项目/插件二进制缺失时自动构建；更新 UE 插件源码后应先手动运行 `tools/build_unreal_linux.py --adapter <本机运行副本> --unreal <引擎目录>`，同步更新运行副本源码，避免使用旧二进制。

## 检查与日志

```bash
./start_deployment.sh --check
./scripts/linux_platform.sh status
```

`--check` 检查环境与配置，不启动服务。状态文件为 `run/linux_platform.json`、`run/scene_runtime.json`，日志位于 `logs/linux-platform.log`、`logs/linux-backend.log`、`logs/linux-signalling.log` 和按 scene ID 命名的日志目录。账号库、录制与日志会保留，停止不会删除数据。

Linux 场景直接使用既有模型文件并校验来源，不在启动时重生成跟踪的 XML。缺少对应模型或来源校验失败时应按原有模型构建流程准备，脚本不会自动切换为其他模板。
