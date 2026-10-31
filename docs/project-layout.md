# 项目目录规范

标准布局为 space-sim/server 和 space-sim/ue-adapter；GitHub远端名称与Python包名保持不变。

各目录与代码入口的具体职责见[项目结构地图](project-map.md)，后续新增文件遵循[命名规范](naming-conventions.md)。

| 位置 | 内容 | 管理 |
| --- | --- | --- |
| server/backend、collection、simulation、native | 平台、编排、物理与原生源 | Git |
| server/frontend、signalling、tests、tools、scripts | 网页、信令、测试与工具 | Git |
| server/model、UE Content/ContentSource | 交付模型、渲染资产、源材质 | 文本Git，二进制LFS |
| docs、manifests、tools/release | 当前说明、清单、发布源码 | Git |
| server/run、logs、reports | 状态、原生库、日志、验证输出 | Git忽略 |
| UE Binaries、Intermediate、DerivedDataCache、Saved | 本机构建与编辑器状态 | Git忽略；先区分依赖 |
| 工作区 data | 采集、布局池、训练权重 | 仓库外 |
| 工作区 experiments | 新实验与诊断 | 仓库外 |
| 工作区 artifacts/build、verification | 暂存与解压验证副本 | 仓库外，可恢复后清理 |
| 工作区 releases | 不可变发布包、校验、版本说明 | 仓库外 |
| 工作区 reports、archive/workspace-snapshots | 维护报告、恢复快照 | 仓库外 |

本机根为 /home/spacesim-zmh，源码在 src/space-sim。环境保持 envs，UE保持 engines。server/run/native_acceleration 是现行加载器约定；外移需配套改造构建、加载和启动代码。

旧 src/space_sim_server/ 只有两个兼容软链接，服务于保留批次的冻结绝对路径，不含源码副本，不用于新开发。当前数据完整退役前保留兼容入口。

新资产使用稳定ID、版本、来源、许可和输入哈希，不按旧日期、Generated或validated等名称直接删除。
