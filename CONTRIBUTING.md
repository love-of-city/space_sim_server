# CONTRIBUTING.md

## 日常流程

从最新 main 创建 feature/<描述>、fix/<描述> 或 chore/<描述> 短期分支。一次 PR 解决一个明确问题，按照 [PR模板](.github/pull_request_template.md) 填写详情、测试证据、兼容性与风险等。main 通过 PR 更新，合并，完成后自动删除开发分支。

提交信息采用 feat:、fix:、docs:、test:、chore: 等前缀，正文说明原因。

对于大型变更，提交中需要包含一份详细的变更文档，按照 20XX_XX_XX_具体内容.md 这样的格式，存在项目目录下的 archive/ 文件夹。

## 其他

模型及 UE 二进制资产使用 Git LFS。不提交密钥、账号数据库、录制、日志、Saved、Intermediate、虚拟环境或机器绝对路径。

基础 CI 成功不等于完整仿真兼容，发布前另提供真实仿真、渲染及采集验收证据。
