# AGENTS.md

这是太空机械臂遥操作与数据采集平台的统一仓库。网页实时显示 UE 仿真画面。

- `space_sim_server/`：后端、网页、信令、权威动力学、模型和采集。
- `space_sim_UE_Adapter/`：Python 渲染发送端、UE 项目与 Runtime 插件。
- 两部分的协议或运行时变更在同一个分支、同一个 PR 中提交。
- GitHub 配置和协作流程位于根目录。修改部署入口时同步检查根 CI 的部署路径过滤器。
- 模型和 UE 资产使用 Git LFS；不提交环境、运行数据、密钥、日志或生成的 UE 目录。
- 阅读根 CONTRIBUTING.md 和对应子项目 README 后执行验证。
