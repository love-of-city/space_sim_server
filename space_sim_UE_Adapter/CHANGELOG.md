# Changelog

## Unreleased

- 增加默认关闭的 `-BskCameraDiagnostics` 只读回传，支持比对 WebRTC 输入、Pawn/Camera 和实际 POV；配合服务端定位输入问题并验证串流双向两圈旋转；服务端已撤回强制普通鼠标模式，恢复 raw 优先并增加异常位移保护。

- 全局自由视角改为局部轴四元数旋转，取消上下/左右角度边界，支持连续翻转；隔离 UE 控制器的默认俯仰/滚转限制，补充两圈旋转、极点、实际相机输出与返回主视角测试。

- 修复浏览器全局自由相机：明确模式/键状态命令绕过 IME 旧键码和 Slate 焦点；相对鼠标固定增益、独立于画面尺寸/FOV，移动按帧时长积分；增加失焦/断流超时保护与相机自动化测试。

- 新增场景级 `sunlight_intensity_scale`（0～10，默认 1）：保留原星历/距离衰减/地影，仅调节太阳照明；支持明确关闭直射光及旧 manifest 重置，并增加 Python/UE 光照组件测试。

- 支持 MJCF 引用的 ASCII/Binary STL：确定性缓存转换、完整 LOD0、源指纹重导入、角度感知法线和基础 UV。
- MJCF 元数据遍历支持 `<frame>` 安装层级，可发现 MjSpec 组合模型中的嵌套 mesh。
- 增加自由漂浮 CubeSat + SO-101 一键实时示例，并支持在高频动力学任务中约 30 Hz 限频采样。
- 增加原生 SO-101 PID/纯接触抓取的 UE 实时包装，并规避 Windows Python MuJoCo/Basilisk MuJoCo DLL 预加载冲突。
- UR5e 示例改为六个关节依次平滑转动固定角度并保持，默认每关节 20°。
- MJCF 动态目标灯设为 Movable，避免每帧 Mobility 警告造成 Game Thread 卡顿。

## 0.2.0 - 2026-08-10

- 建立统一的 BSK→UE5 单仓库版本基线。
- 提供渲染器无关的 `BasiliskRenderBridge` 和 `bsk-render/2` 协议。
- 提供 UE 5.6 Runtime C++ 接收插件、断线重连、最新帧、浮动原点和插值。
- 支持 Spacecraft、MJScene 刚体/几何、天体、航天器仪器可视元素和录制回放。
- 提供双航天器、Demo 8 和 UR5e 示例。
- UR5e Static Mesh 使用完整标准 LOD0、关闭 Nanite，并在构建阶段完成米到厘米转换。
