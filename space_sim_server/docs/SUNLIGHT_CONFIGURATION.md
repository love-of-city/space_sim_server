# 场景初始化：太阳光照强度

更新：2026-09-08。此选项是 **UE 渲染光照倍率**，不是太阳的物理辐照度 W/m²，也不改变 Basilisk 的动力学模型。

## 使用方式

在前端“场景实例”设置中填写 **太阳光照强度（倍率）**，再点击“生成并启动场景”。

| 输入 | 含义 |
| --- | --- |
| `1`（默认） | 保持原来的太阳照明标定 |
| `0.5` | 太阳直接照明强度为原来的二分之一 |
| `2` | 太阳直接照明强度为原来的两倍 |
| `0` | 关闭太阳直接照明，包括局部物体及天体表面的太阳照明通道 |

允许 `0～20,000` 的有限数值；空白、负数、超过上限、NaN/Infinity、布尔值和字符串形式的 API 数值会被拒绝。运行中的输入框锁定，场景摘要显示实际保存的倍率。修改后需停止场景并创建新实例，不是运行中的实时调光接口。

**0 倍不保证画面全黑**：非物理环境补光、材质自发光、太阳视觉代理、天空及曝光设置不在本选项控制范围内。自动曝光和色调映射也可能使屏幕像素亮度不按倍率线性变化。本次没有修改这些显示设置。

## 配置与传递

```text
前端输入 sceneSunlightIntensity
  → POST /api/scenes/start 或 /api/scenes/instances
       sunlight_intensity_scale
  → SceneInstanceCreate 校验
  → 场景 JSON：environment.lighting.sunlight_intensity_scale
  → 原生仿真 _load_scene_instance 校验/兼容旧实例
  → SceneSettings.sunlight_intensity_scale
  → bsk-render/2 scene_manifest.settings.sunlight_intensity_scale
  → UE SceneSunlightIntensityScale
  → SunLight / CelestialSunLight
```

请求示例（其余字段可按原接口填写）：

```json
{
  "seed": 123,
  "randomization_profile": "none",
  "sunlight_intensity_scale": 2.0
}
```

对应的场景实例片段：

```json
{
  "environment": {
    "lighting": { "sunlight_intensity_scale": 2.0 }
  }
}
```

完整场景还保留原有历元、参考系和轨道字段。Episode 的 `scene_instance` 元数据会保留该配置，用于复现。旧请求、旧场景 JSON 和旧 manifest 没有该字段时均按 **1 倍**处理。仿真未传入场景实例文件时也使用 1 倍。

## 不变的物理与照明关系

- 太阳位置、光线方向仍来自同一组 SPICE 星历。
- 卫星轨道、重力源强度、质量/惯量、姿态控制、机械臂指令均不读取此参数。
- 保留原来的参考照度、距离平方反比衰减以及地影可见比例。正常距离范围内，局部太阳照度约为：

  `参考照度 × (参考距离 / 实际距离)² × UE 标定倍率 × 本场景倍率 × 太阳可见比例`

- 天体表面通道也乘本场景倍率，但仍不使用卫星所在位置的地影可见比例，避免卫星进地影时整颗地球一起变黑。
- 现有 UE 配置 `sun_illuminance_scale` 是渲染器自身的标定；新场景倍率独立保存并与之相乘，不覆盖它。
- 每次应用 manifest 都重设场景倍率；旧场景缺失字段时恢复 1，而不是沿用上一场景的 0 或其他数值。
- 0 通过独立倍率明确表达，避免原有参考照度字段将非正值解释为“使用默认照度”的兼容行为。
- 没有新增太阳辐射压力、热模型、功率模型或传感器光电响应模型。

## 修改位置

服务端：

- `backend/space_arm_platform/lighting.py`：默认值、范围和场景文件校验。
- `backend/space_arm_platform/models.py`、`scene_runtime.py`：API、目录默认值和实例保存。
- `simulation/teleop_grasp_unreal.py`：场景读取和 manifest 参数传递，输出 `lighting_configuration` 日志。
- `frontend/index.html`、`app.js`：初始化选项、校验、锁定和实际值显示。

UE 适配器仓库：

- `Adapters/bsk_render_adapter/descriptors.py`：新 SceneSettings 字段与序列化校验。
- `BskProtocolTypes.h`、`BskFrameParser.cpp`：可选字段解析；显式检查 JSON number 类型，防止字符串/布尔值被隐式转换。
- `BskSceneController.cpp/.h`、`BskCelestialLighting.h`：场景倍率应用于初始照明与逐帧星历照明。

旧版 UE 二进制不会使用这个新增字段，因此服务端与适配器源码需一起更新并编译。本机本次已执行 UE 模块构建和前端构建；重新启动后端、刷新前端并创建新场景即可使用。

## 验证边界

- 服务端测试覆盖创建 API、文件保存/读取、旧实例兼容、非法值、Episode 配置保留，以及相同 Seed 的轨道/随机状态不变。
- 前端测试覆盖 0 值发送、非法值拦截、运行中锁定、实际倍率显示和旧目录默认值。
- 适配器 Python 测试验证倍率确实进入 manifest，且没有修改补光设置。
- UE 自动化测试创建真实 DirectionalLight 组件，验证 0/0.5/1/2/10 倍、局部与天体照明、补光不变、标定不被覆盖、距离衰减、非法类型拒绝和旧 manifest 重置。
- 原生隔离探针运行 0、1、2.5 倍三个短场景：经真实 Hub 收到观测，三个 manifest 的倍率分别正确；相同 Seed 的最终机械臂关节及本体姿态一致，参考太阳照度仍为原值。

本机测试证据位于 `run/sunlight-20260908/`。UE 自动化使用 NullRHI；本次没有做浏览器 WebRTC 截图或 GPU 像素亮度标定，不将组件照度测试等同于屏幕像素线性验证。

本次回归结果：服务端 **97 通过**、前端 **12 通过**、适配器 Python **41 通过 / 1 跳过**、UE 自动化 **17 通过**；UE 编辑器模块与前端均构建成功。
