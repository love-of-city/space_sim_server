# 外部方法的大图像传输

原捕获双目为 1920×1920，每张公开 PNG 的上限为 16000000 字节；两张图像经 base64 编码后可能超过旧方法接口的 4 MiB 限制。JSONL v2 为这类公开观测提供显式、有界的传输版本，不缩小图像，也不把私有文件路径或渲染位姿传给方法。

| 方法配置 `schema_version` | 线协议 | `act` 请求上限 | `reset` / `close` 请求及所有响应上限 |
|---|---|---|---|
| `spacesim-autonomy-python-method/1` | `spacesim-autonomy-method-jsonl/1` | 4 MiB | 4 MiB |
| `spacesim-autonomy-python-method/2` | `spacesim-autonomy-method-jsonl/2` | 48 MiB | 4 MiB |

48 MiB 容纳两个达到声明上限的 PNG 的 base64 内容，并留有超过 1 MiB 的观测元数据空间。它是编码后整行的上限；不是无限队列，也不会改变任务自己的相机、图像大小或观测白名单。旧配置序列化不添加新默认字段，指纹保持原语义；选用 `/2` 会明确改变方法身份。运行中不能更换协议、脚本或配置。

## 接入

方法配置使用 `/2`，入口 SDK 同时显式选用 `IMAGE_PROTOCOL`：

```python
from space_arm_platform.benchmark_external import IMAGE_PROTOCOL, serve_method

raise SystemExit(serve_method(MyMethod(), MyContext, MyObservation,
                             protocol=IMAGE_PROTOCOL))
```

`MyContext`、`MyObservation` 和动作仍须符合具体任务的公开契约。现有 `serve_method` 默认继续使用 v1，不进行隐式协商或失败后切换协议。完整示例 [vision-pose-v2.json](../examples/autonomy_methods/vision-pose-v2.json) 与 [vision_pose_v2.py](../examples/autonomy_methods/vision_pose_v2.py) 复用原图像位姿方法，可用于已注册 D23/D24；这些任务的相机仍为 640×480。示例采用新传输协议并不使它们变成完整 1920 双目抓取任务。

在满足现有任务依赖的 SpaceSim 环境中，可通过原 CLI 指定该方法：

```text
python tools/autonomy_benchmark.py run d24-marked-stereo-pose-v1 --seed 1 --method-spec examples/autonomy_methods/vision-pose-v2.json --output run/benchmark-d24-jsonl-v2
```

调用顺序仍为一次 `reset`、多次 `act`、一次 `close`。请求编号必须连续，响应须匹配编号和协议。重复 JSON 字段、非有限数、无换行、未知协议和超出对应操作容量的消息拒绝。SDK 也限制返回动作，不能因为允许大图像请求就发送 48 MiB 响应。每次只允许一个在途调用；原超时、标准错误截断、拥有进程树的清理与方法文件身份检查继续生效。

## 验证边界

新测试使用两张真实编码、无损无压缩的 1920×1920 RGBA 合成 PNG，单张超过 14 MB，通过真实独立方法进程核对原字节哈希、分辨率、源帧、曝光时刻和投递时刻。相同输入仍须被 v1 拒绝；另核验控制请求/响应容量、协议错配、配置变化、进程清理和旧配置指纹。交付的 v1/v2 位姿示例还用同一组声明尺寸的空白图像验证输出一致，且正确报告无位姿。

源码 `8d200ca5a159af3037ddffc3ead9c74097372cf4` 在 SpaceSim 云端完成 **270 项新旧方法进程、图像、执行/复算、数据集及平台检查和 Ruff，零跳过**。报告 `.tmp/benchmark-image-transport-checks-v1/acceptance.json` SHA-256 为 `a434d85a56667640dc13338cb36eecbe0f1e11112a001b5fa151ec1579b3c92c`。检查包含上面的原尺寸双目实际 IPC、旧版容量拒绝、进程清理及交付示例一致性。

这些是传输和公开输入边界检查，合成图片不计为实际 UE 图像或感知成功。新抓取环境仍需把真实运动挂载双目、时延、公开观测、机械臂动作和全回合评分一起接通；当前没有新增完整抓取任务的注册或验收。
