# 本机完整 SARM 平台

跨电脑浏览器直连请使用 `start_remote_visualization.cmd`，具体见 `REMOTE_VISUALIZATION.md`。两种入口不要同时运行；切换前先停止当前模式。

## 日常操作

1. 在远端桌面双击 `start_local_visualization.cmd`，保留启动窗口。默认启动完整平台，不再自动运行 CubeSat 演示。
2. 浏览器打开 `http://127.0.0.1:18000/`，使用原有账号登录。
3. 在右侧“场景实例”保留默认 SARM 模板，点击“生成并启动场景”。初次加载资源需要等待；未启动场景时“仿真未连接”正常，但不应显示“运行时未配置”。
4. 等待“场景运行中”“仿真在线”和 `LIVE WEBRTC`。点击画面进入操作：W/S、A/D、Q/E 平移，Shift 配合切换旋转，R/F 控制夹爪；Esc 退出操作。C 进入自由视角，Home 返回主视角。
5. 点击“停止场景”只停止物理仿真和 UE，网页保持在线。完整场景没有 120 秒或一小时自动退出限制。
6. 双击 `stop_local_visualization.cmd` 停止整个平台；再次启动后可重新生成场景。

“生成并启动场景”使用项目原有 `start_scene_instance.ps1` 和 `simulation/teleop_grasp_unreal.py`，包括后台控制连接、状态反馈、UE 渲染。数据集采集默认关闭，交互和预览验证不等于 RGB/深度/分割数据采集已验收。

## 当前机器配置

- 后端 Python：优先当前仓库 `.venv`，不存在则复用桌面旧 `space_sim_workspace/space_sim_server/.venv`。
- 物理仿真：`C:/tools/miniconda3/envs/mujoco-dev/python.exe`。
- 2026-09-17 合并新版后，启动器通过 `SPACE_SIM_PYTHON` 将上述仿真解释器传给场景和资源准备脚本，采用上游统一选择逻辑；后端仍使用独立的后端解释器。
- 场景管理 PowerShell：`C:/tools/powershell-7.4.13/pwsh.exe`（官方 ZIP 下载并校验 SHA256）。外层启动器兼容 Windows PowerShell 5.1。
- UE：`C:/Program Files/Epic Games/UE_5.6`，使用离屏渲染，无需弹出 UE 窗口。
- 模型：本仓库 `model/SARM/platform`；Adapter：同级 `space_sim_UE_Adapter`。
- 已导入 113 个 UE 资源；目录为 Adapter 下 `Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_platform.catalog.json`。后续启动会核对缓存，无修改时不会重复导入。
- 后端和信令每次共享随机 JWT 密钥。保留原账号数据库 `data/visualization-test-auth.sqlite3`，不重置密码。

路径可通过 `scripts/local_visualization.ps1` 的 `-BackendPython`、`-SimulationPython`、`-AdapterRoot`、`-ModelRoot`、`-UnrealRoot`、`-PowerShellExe`、`-Node` 调整。`-Mode Demo -Duration 3600` 才是之前的独立 CubeSat 演示，不用于网页遥操作。

## 端口与排查

全部绑定回环地址，在远端桌面浏览器观看。18000 是网页；18766 是物理仿真控制连接；5558 是 UE 场景接收；8080/8888 是视频信令；18767 是可选采集接收。不要打开 8080 当网页。只转发网页端口不保证跨机器 WebRTC 可达。

状态：`powershell -NoProfile -File scripts/local_visualization.ps1 -Action Status`。
平台日志：`logs/local-visualization-*`；场景日志：`logs/scene-*.launcher.*.log` 和 `logs/scene-*.simulation.*.log`。

端口占用时不自动终止未知程序。如果启动窗口被强制关闭，先运行停止脚本，再启动。场景启动失败时先查看右侧状态和对应日志，不要重复点击或重新下载整个仓库。

## 验证

平台停止后，用后端 Python 执行 `tools/verify_local_platform.py`。它实际启动平台和 SARM 场景，使用独立随机账号数据库测试登录、场景 API、观测时间推进、低速控制脉冲与 IK 目标反馈、浏览器持续视频解码、停止场景后网页仍在线，最后清理测试进程。需要已安装的 httpx、websockets、Node 和 Edge。

测试结果、截图和隔离数据库存于忽略的 `run/`，不修改原账号。不要在已有平台运行时执行这项端到端验证。
