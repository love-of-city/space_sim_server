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

权威 RGB 默认使用 `FRHIGPUTextureReadback` 与后台 JPEG 编码，并按提交顺序输出。`-BskSynchronousCapture` 保留同步对照；预览路径仍可能同步读取。GPU 回读、编码、网络与直接落盘队列有容量限制，权威帧通过背压保序；不将异步实现等同于所有平台均已验收。

UE 直接落盘模式使用每帧 `.complete.json` 作为完整提交标记。图像、元数据或标记写入失败会产生错误结果；部分文件不代表完整记录。完成/失败计数可通过控制器读取，停止时等待已接纳的磁盘工作并消费结果。

`BskUnrealRuntime/Private/` 内部职责：`Scene/` 管理场景、配置与生命周期，`Capture/` 管理采集、输出和完成回执，`Streaming/` 管理预览与 UE 5.6 兼容接口，`Lighting/` 管理天体和环境光照。公开反射类名与 Game Thread 边界保持稳定。

启动参数 `-BskRuntimeReport=<绝对路径>` 写出实际模块位置、进程与引擎版本。独立 Cook 包可用 `-BskValidateCookAssets=<清单>` 和 `-BskCookValidationReport=<结果>` 验证每项动态资产实际可加载，完成后以 0/1 退出；此模式不启动帧接收。资产可加载验收仍不等同于图像质量验收。

当前未完成 OpNav 闭环、单遍分割、严格锁步及 Vizard 全部专用面板。NullRHI 测试不能作为图形或视频质量验收。
