# 开发与维护

服务端位于 server/，渲染适配器位于 ue-adapter/。配套协议、运行时、资产和采集变更放在同一分支/PR，遵循根 PR 模板，使用 feat/fix/docs/test/chore 等提交前缀。

## 最新工作区与版本

同步或重构前保全未提交源码、未跟踪功能/测试、资产清单和原生库指纹。HEAD 不一定代表实际加载代码。不要批量用 ours/theirs 处理功能冲突，不通过 reset/clean 删除未提交优化。先核对[采集基线](server/docs/COLLECTION_BASELINE.md)。

大型变更更新 CHANGELOG 和相关当前文档。运行证据保存在仓库外对应批次，当前文档说明验证范围和已知问题。不要再次把逐次实验日志堆入源码 archive。历史数据是否删除遵循用户明确的保留范围，清理时记录必要依赖和原件归属。

## 验证

- 服务端：从 server/ 执行已有 Python 测试；路径变更核对平台解析、布局池、采集恢复、部署与原生库加载。
- 网页：npm --prefix frontend test，以及 npm --prefix frontend run build。
- 信令：npm --prefix signalling test，以及 npm --prefix signalling run check。
- 适配器：遵循 ue-adapter/README.md，基础 Python 测试与 UE/GPU 验证分别报告。
- 部署入口变更同步检查 .github/workflows/ci.yml 的路径过滤器；移动测试同步更新 ci-exclusions.json。
- 原生变更须核验源码、编译指纹及实际加载二进制；短测、完整任务和完整批次结果分别记录。

条件不具备的验证报告为未执行/跳过，不能算通过。不要在用户要求停止的正式批次上运行验收或自动恢复任务。

## 文件管理

保留第三方许可。模型/CAD/UE交付资产使用LFS。环境、录制、日志、认证数据库、机器配置、Saved、Intermediate与构建产物不提交。Generated目录同时含通用资产与本机布局，按清单精确分类。

发布源码在 tools/release/，输入 staging 和输出包在仓库外。发布包不能原地覆盖为新版本，其验收不能代表后续源码。见[目录规则](docs/project-layout.md)。
