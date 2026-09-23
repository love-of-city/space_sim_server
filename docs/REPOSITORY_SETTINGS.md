# GitHub 仓库设置

目标：main 必须经 PR 合并；不要求人工批准，作者可以自行合并。CODEOWNERS 用于通知，不强制代码所有者批准。管理员同样遵守保护规则。

## 启用顺序

1. 推送治理分支并创建 PR，等待基础 CI 首次运行，确认检查名已注册。
2. Settings → General → Pull Requests：仅启用 squash merge，启用 Automatically delete head branches。
3. Settings → Branches 为 main 创建保护：Require a pull request before merging；关闭 Require approvals 和 Require review from Code Owners。
4. 开启 Require status checks to pass、Require branches to be up to date、Require conversation resolution、Do not allow bypassing the above settings。关闭 force pushes 和 deletions。
5. 必需检查：server-python、server-web；不要配置尚未注册或可被路径过滤跳过的检查。
6. 通过 PR 合并并回读 GitHub API 的 branch protection 和仓库 merge 设置。

当前状态以 GitHub 实际设置为准；此文件只声明目标，不能证明保护已生效。私有仓库可能受账号套餐限制。无权限或功能不可用时，保留失败原因和待配置项，不伪称启用。

2026-09-23 检查：当前凭据 Hyperlovimia 有 push 权限、无 admin/maintain 权限，无法启用保护和修改合并设置。管理员可在 CI 首次通过后，从本仓库根目录执行（需安装并登录 GitHub CLI）：

```powershell
gh api --method PATCH repos/love-of-city/space_sim_server --input .github/repository-settings.json
gh api --method PUT repos/love-of-city/space_sim_server/branches/main/protection --input .github/branch-protection.json
gh api repos/love-of-city/space_sim_server/branches/main/protection
gh api repos/love-of-city/space_sim_server --jq '{allow_squash_merge,allow_merge_commit,allow_rebase_merge,delete_branch_on_merge}'
```
