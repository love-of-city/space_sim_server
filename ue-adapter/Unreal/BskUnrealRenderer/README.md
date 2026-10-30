# BskUnrealRenderer

UE 5.6 项目和 Runtime 插件。安装、构建、测试与启动统一使用[仓库 README](../../README.md)的根目录命令，本目录不再维护第二套部署步骤。

## 运行架构

Python 发送端通过 `bsk-render/2` 提供 manifest、最新动态帧和有界事件；`FBskTcpReceiver` 在网络线程解析，Game Thread 中的 `UBskRenderWorldSubsystem` 与 `ABskSceneController` 更新对象。网络线程不访问 UObject。

发送端将惯性坐标转换到局部右手坐标，线上姿态为主动 body-to-local `(w,x,y,z)`。UE 镜像 Y 轴并将米转换为厘米，位置/姿态使用双精度与 LWC。UE 不推进权威动力学。字段定义见[协议](docs/PROTOCOL.md)。

## 预览、采集与回放

- Pixel Streaming 2 经信令服务器向浏览器发送操作画面；主视口与 manifest 相机可提供独立流。
- `bsk-capture/1` 发送精确源帧的 RGB、深度与实例分割，包含时间戳、相机参数和浮动原点。预览帧不能替代权威训练图像。
- `.bskrec` 支持录制与回放；`BasiliskRenderBridge` 可使用 `recording_path`，离线录制使用 `RecordingOnlyPublisher`。
- 任务面板默认隐藏，M 切换显示；仅白名单命令可回传仿真线程。

## 配置与资产

配置文件为 `Config/bsk_unreal_scene.json`。Earth/星空主要使用 `Content/Planets` 的纹理和材质；有 Earth/Sun manifest 时位置、姿态与照明方向由仿真星历驱动。Celestial Vault 是回退资产，不负责动力学星历。

本地资产映射优先于 manifest；随后尝试 UE 资产、可选 USD、MJ primitive 和占位体。外部磁盘资产默认禁用。模型路径改变后应重新生成 `Saved/AssetImport`，不能复用原机器的绝对路径。

- [MJCF 导入](docs/MJCF_MESHES.md) / [STL 导入](docs/STL_MESHES.md)
- [太阳资产来源与开关](Content/SpaceFX/README.md)

## 扩展与限制

项目插件可实现 `IBskRenderExtension` 并通过 `RegisterRenderExtension` 注册；扩展返回空 Actor 时继续尝试内置工厂。`IBskCaptureProvider` 提供采集扩展。所有扩展遵守 Game Thread 与动力学权威边界。

当前未完成 OpNav 闭环、异步 GPU Readback、单遍分割、严格锁步及 Vizard 全部专用面板。NullRHI 测试不能作为图形或视频质量验收。
