# 开发与维护

服务端位于 server/，渲染适配器位于 ue-adapter/。配套协议、运行时、资产和采集变更放在同一分支/PR，遵循根 PR 模板，使用 feat/fix/docs/test/chore 等提交前缀。

## 最新工作区与版本

同步或重构前保全未提交源码、未跟踪功能/测试、资产清单和原生库指纹。HEAD 不一定代表实际加载代码。不要批量用 ours/theirs 处理功能冲突，不通过 reset/clean 删除未提交优化。先核对[采集基线](server/docs/COLLECTION_BASELINE.md)。

大型变更更新 CHANGELOG 和相关当前文档。运行证据保存在仓库外对应批次，当前文档说明验证范围和已知问题。不要再次把逐次实验日志堆入源码 archive。历史数据是否删除遵循用户明确的保留范围，清理时记录必要依赖和原件归属。

修改物理模型、采集参数或任务成功规则时，同步维护[全流程对齐指南](docs/guides/simulation-data-workflow.md)链接的配置/验收参考；写清参数单位、覆盖顺序、实际保存证据和复验范围。在线成功判定、采后校验与任务版本一起核对。每批实际选择与未验证项可用[任务记录模板](docs/templates/collection-task-contract.md)保存在仓库外，不把运行默认值或历史实验结论直接当作新批次验收。

## 验证

- 服务端：从 server/ 执行已有 Python 测试；路径变更核对平台解析、布局池、采集恢复、部署与原生库加载。
- 网页：npm --prefix frontend test，以及 npm --prefix frontend run build。
- 信令：npm --prefix signalling test，以及 npm --prefix signalling run check。
- 适配器：遵循 ue-adapter/README.md，基础 Python 测试与 UE/GPU 验证分别报告。
- 部署入口变更同步检查 .github/workflows/ci.yml 的路径过滤器；移动测试同步更新 ci-exclusions.json。
- 原生变更须核验源码、编译指纹及实际加载二进制；短测、完整任务和完整批次结果分别记录。

条件不具备的验证报告为未执行/跳过，不能算通过。不要在用户要求停止的正式批次上运行验收或自动恢复任务。

## 文件管理

目录与入口职责见[项目结构地图](docs/project-map.md)；新增文件遵循[命名规范](docs/naming-conventions.md)。Python 使用有明确业务含义的 snake_case，文档/外部批次按对应规则命名；保留 UE、协议、原始资产与冻结数据的既有契约。新增模块写明职责，目录/入口变化同步更新地图。

保留第三方许可。模型/CAD/UE交付资产使用LFS。环境、录制、日志、认证数据库、机器配置、Saved、Intermediate与构建产物不提交。Generated目录同时含通用资产与本机布局，按清单精确分类。

发布源码在 tools/release/，输入 staging 和输出包在仓库外。发布包不能原地覆盖为新版本，其验收不能代表后续源码。见[目录规则](docs/project-layout.md)。
