# Pixel Streaming 2 多相机输出

UE 主视口继续使用 Pixel Streaming 2 默认 BackBuffer Streamer。运行时插件还可为 `scene_manifest.cameras` 中的相机创建独立的：

```text
Camera Actor → SceneCaptureComponent2D → UTextureRenderTarget2D
             → FVideoProducerRenderTarget → IPixelStreaming2Streamer
```

相机 Streamer ID 由基础 ID 和稳定 `camera_id` 组成，`/` 等字符转换为 `_`：

```text
BskRenderer__teleop_camera_spacecraft_overview
BskRenderer__teleop_camera_so101_wrist_cam
```

相关启动参数：

```text
-BskPixelStreamingURL=ws://127.0.0.1:8888
-BskPixelStreamingBaseId=BskRenderer
-BskPixelStreamingCameras=teleop/camera/spacecraft_overview+teleop/camera/so101_wrist_cam
-BskPixelStreamingCameraWidth=640
-BskPixelStreamingCameraHeight=360
-BskPixelStreamingCameraFps=30
```

相机流仅用于低延迟预览。权威 RGB、深度、分割仍由 `bsk-capture/1` 按精确仿真帧采集，二者不共享时间语义，也不会将 WebRTC 插值、编码或丢帧写入训练数据。
