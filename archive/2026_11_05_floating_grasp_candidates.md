# 漂浮物体候选抓法

在同一冻结场景中比较物体局部抓取点与方向。`floating_initial_state` 使用 `floating-release/4`，参考坐标采样不再被旧唯一抓法的 IK 过滤；`floating_grasp` 独立保存选中的候选和原生检查结果。历史 episode 不改写。

场景 JSON 或 `server/batch_collect/configs` 中 YAML 的 `scene` 可加入：

```yaml
floating_grasp:
  selection: random
  candidate_ids: [baseline, shallow, deep]
```

默认 `ranked`，省略 `candidate_ids` 则比较该物体全部注册候选。候选在 `simulation/scripted_tasks/floating_grasps.py` 维护。原生检查涵盖有限采样的张开接近、双侧闭合和保持相对工具位姿的携带路径；真实摩擦夹持和动态避碰由控制器及录制验收判断。

控制器版本 `floating-capture/8`，阶段为张开、预抓、拦截、夹紧、保持两秒、回收和稳定。成功阈值保持原值。选择在 collect、record、run 三种入口均冻结，场景、候选定义、模型、控制器变化会使旧选择失效，每帧记录候选 ID。

本地相关测试：162 通过、2 项缺少 Basilisk 跳过。批量/入口相关测试在补齐项目现有视频与表格依赖后继续核对。原生动态及UE采集结果随本轮验证更新。
