# 双操作臂卫星：完整系统预览

安装仓库声明的 Python 依赖、Git LFS 资产、Node.js 22，并编译随仓库提供的 UE 5.6 项目。
从 `space_sim_server` 目录运行（使用本机已安装的路径）：

```powershell
./scripts/run_dual_service_platform.ps1 -Python '<Basilisk环境>/python.exe' -UnrealRoot '<UE_5.6目录>'
```

只有 `camera_version_1`、`camera_version_2` 是本次发布的冻结场景。
工作姿态见 `workspace_pose.json`；原始收拢姿态保存在各版本 XML 内。
场景依赖同一仓库的 `model/SARM_mjcf_20261012`，因此需要完整克隆并执行 `git lfs pull`。

![版本 1 三视角](previews/version_1.jpg)
![版本 2 三视角](previews/version_2.jpg)

保温层外观修正：`/Game/Generated/DualServiceMLI` 是原 PBR0110 网格和三个材质实例的独立副本。
本体放大 2 倍，因此 TextureTiling 从 3 调到 6，保持原有物理纹理密度；颜色、法线强度和粗糙度沿用原材质。
双臂模板单独启用地球/天空环境反射捕获，排除 1 km 内本体几何，每 20 s 更新。
两个手眼灯跟随各自腕部，照明范围 1 m。实例目录的 `preview_lighting.json` 可用
`{"wrist_intensity": 0}` / `{"wrist_intensity": 500}` 做关灯/开灯实渲对照，并自动保存新的同步三视角。
这只是渲染灯光强度，尚未标定为真实灯具的功率或光通量。
网页切换到全局或某侧手眼视角后，使用“工作灯”按钮独立开关对应的场景灯。
按钮现在复用模型中的灯，不再另外创建一盏无法关闭原灯的补光。
自动对照也可向 `preview_lighting.json` 写入 `global_enabled`、`left_enabled`、`right_enabled`
布尔值；左右按卫星实际摆放位置命名。只切换灯光，不改变太阳照明或物理状态。

两份冻结模型分别通过平台模板 `dual-service-camera-1` 和 `dual-service-camera-2` 启动。
机械臂、任务盒、碰撞体及保存的作业姿态相同，仅全局相机和对应虚拟避碰区域不同。
冻结文件及校验值见 `camera_versions.json`，共同初始姿态见 `workspace_pose.json`。

使用 PowerShell 7 执行 `scripts/run_dual_service_platform.ps1`，打开
<http://127.0.0.1:18080>，在场景列表选择“ 双操作臂卫星 · 相机版本 1/2 · 地月环境”。
平台使用独立端口，不占用默认平台端口：API 18080、控制 18766、RGB 18767、
UE 15558、Pixel Streaming 18888/18081。场景切换前停止当前场景。

## 环境和状态

- Basilisk SPICE：2026-11-05 00:00:00 UTC，Earth/J2000，地球、太阳、月球。
- 500 km 圆轨道，51.6° 倾角，初始真近点角 180°。
- 400 Hz 原始 MuJoCo 模型；本地步进器与 Basilisk 地球 J2、太阳/月球重力相连。
- UE 使用同一套天体状态，实际地球纹理、Celestial Vault 月球和太阳光照。
- 卫星包覆沿用平台 `SarmMLI_PBR0110` 独立保温层及原生 UV 材质，随本体放大 2 倍并按
  同一安装面中心偏移；保温层仅参与显示，保留原物理模型和操作面开口。
- 原始位置执行器保持已保存的作业姿态，卫星自由基座参与动力学。
- 全局和双手眼相机均为 1024×1024；左右标签按当前实际摆放位置映射。
- 本地 UE 窗口显示整体环境和三路相机，同时保留网页视频流。

这是完整动力学与 UE 渲染链路中的静态作业姿态预览。尚未为这两个模板接通双臂遥操作、
随机物体任务或训练数据采集验收；不把姿态保持或画面导出作为抓取任务成功。

## 可复核输出

每次启动在 `run/dual_service/<instance_id>/` 保存运行状态和有界预览记录：

- `runtime.json`：模型 SHA-256、完整实例、物理步数、天体状态、采样穿透量。
- `scene.bskrec`：前 15 s 的原始渲染消息。
- `ue_preview/`：10–12 s 的三相机同步 RGB/内外参，以及整体视图截图。

材质预热后可写入实例目录的 `refresh_preview.request`，运行程序会再次保存三组同步
相机图和带独立时间标识的整体截图，已有原始证据不会被覆盖。

`tools/diagnostics/export_dual_full_system_preview.py` 选择三相机共有的最后一个时间戳，
复制原始 RGB 和元数据，生成带标签的三视角对比图与校验记录。这里的图像来自 UE 直接渲染；
图像拼接仅增加标题，不改变相机内容。
