# 项目文档

- [目录与产物边界](project-layout.md)
- [完整项目结构与代码职责](project-map.md)
- [文件与目录命名规范](naming-conventions.md)
- [UE 工程整理与交付改进方案及验收状态](ue-engineering-improvement-plan.md)
- [系统架构](architecture/system.md)
- [运行版本、构建来源与批次恢复门禁](architecture/runtime-provenance.md)
- [本机工作区与采集批次](guides/linux-workspace.md)
- [数据管理](guides/data-management.md)
- [可搬迁布局包及历史验收保留](guides/portable-layout-pools.md)
- [UE、原生与 GPU 分层验证](guides/ue-validation.md)
- [独立发布重建、Cook 与环境来源](guides/release-reproduction.md)
- [发布工具](../tools/release/README.md)
- [历史记录清理说明](history.md)
- [采集功能基线](../server/docs/COLLECTION_BASELINE.md)
- [按布局顺序采集](../server/docs/LAYOUT_SEQUENCE_COLLECTION.md)
- [性能口径](../server/docs/COLLECTION_PERFORMANCE.md)
- [Linux部署](../server/docs/LINUX_DEPLOYMENT.md)
- [原生加速](../server/docs/NATIVE_ACCELERATION.md)
- [仿真架构](../server/docs/SIMULATION_ARCHITECTURE.md)
- [模型说明](../server/model/README.md)
- [渲染适配器](../ue-adapter/README.md)

2026-10-31 的工程改进在 `worktrees/ue-engineering-20261031/` 候选工作树验收；三个正式批次 `layout1000-20261029`、`layout1000-20261030-1928`、`layout1000-20261031-0042` 均已停止并保留。独立验证产物写入工作区 `artifacts/verification/ue-engineering-20261031/`，不是正式批次续采。具体目录职责以[项目结构地图](project-map.md)为准。

专题正文保留在子项目 docs，避免重复维护。旧试验、原型和历史版本说明只适用于其注明的场景，不能替代当前验收；CI 入口已纳管与远端检查实际通过也是不同状态。
