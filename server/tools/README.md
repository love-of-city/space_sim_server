# 工具目录

日常采集只运行 `python tools/collect.py --output /absolute/new-run`，修改 `config/collection.json`。
完整流程、六任务开关、失败跳过和输出结构见 [按布局顺序采集](../docs/LAYOUT_SEQUENCE_COLLECTION.md)。

- `assets/`：模型/布局生成、格式转换、UE导入。
- `diagnostics/`：几何检查、视频/接触诊断、预览和独立验收。
- `benchmarks/`：测速与性能分析。
- `deployment/`：部署、认证与环境验证。

实际采集内部实现位于 `collection/`；六种任务控制器位于 `simulation/scripted_tasks/`。
