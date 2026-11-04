# 项目文档

## 仿真与数据采集任务对齐

- [全流程、阶段交接与变更影响](guides/simulation-data-workflow.md)
- [物理模型加载、配置、单位与坐标](reference/physics-model-configuration.md)
- [采集入口、参数优先级、随机种子与录制配置](reference/collection-configuration.md)
- [任务版本、成功阈值与数据验收标准](reference/task-and-data-acceptance.md)
- [验证缺口、补齐条件与证据边界](reference/simulation-validation-gaps.md)
- [每批任务对齐记录模板](templates/collection-task-contract.md)

## 工程与运行

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

2026-10-31 的工程改进已同步到 `src/space-sim/` 主目录，重建 UE/native，并完成主目录独立固定布局五任务录制审核。当时三个旧批次 `layout1000-20261029`、`layout1000-20261030-1928`、`layout1000-20261031-0042` 均已停止并保留；后续新批次单独记录，实时状态以对应 `progress.json` 为准。独立验证产物写入工作区 `artifacts/verification/ue-engineering-20261031/`，不是正式批次续采。具体目录职责以[项目结构地图](project-map.md)为准。

专题正文保留在子项目 docs，避免重复维护。旧试验、原型和历史版本说明只适用于其注明的场景，不能替代当前验收；CI 入口已纳管与远端检查实际通过也是不同状态。
- [外部环境资产恢复与分发](guides/external-environment-assets.md)
