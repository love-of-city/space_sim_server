# CONTRIBUTING.md

## 日常流程

从最新 main 创建 feature/、fix/ 或 chore/ 短期分支，通过根[PR 模板](.github/pull_request_template.md)记录问题、变更、验证、兼容性与风险。服务端和 UE 适配器的配套变更放在同一个 PR 中。提交采用 feat:、fix:、docs:、test:、chore: 等前缀。

大型变更在根 archive/ 中添加 20XX_XX_XX_具体内容.md。子项目历史 archive/ 保留原记录。

## 验证与资产

根 CI 的 required checks 应包含 server-python、server-web、adapter-python。基础 CI 不等于完整仿真兼容；涉及动力学、UE 或采集时，另提供相应运行验收证据。

部署路径过滤器位于根 .github/workflows/ci.yml。未涉及部署的 PR 可跳过服务端 deploy 组；main push 和手动运行始终执行完整基础测试。修改启动脚本或部署依赖时更新过滤器。

模型和 UE 二进制资产使用 Git LFS。保留两边资产许可，不提交密钥、认证数据库、录制、日志、虚拟环境、机器配置、Saved、Intermediate 或构建产物。

## 本次历史导入

仓库合并 PR 必须使用 **Create a merge commit**。Squash 或 rebase 会破坏导入历史与 main 的祖先关系；合入后需再次验证两个原主线 SHA 均为 main 的祖先。其他日常 PR 继续使用仓库允许的合并方式。
