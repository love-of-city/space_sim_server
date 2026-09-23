# 团队贡献指南

## 日常流程

从最新 main 创建 feature/<描述>、fix/<描述> 或 chore/<描述> 短期分支。一次 PR 解决一个明确问题，填写测试证据、跨仓库影响及未验证项。main 通过 PR 更新，采用 squash 合并，完成后自动删除开发分支；现有历史分支不自动清理。

提交信息采用 feat:、fix:、docs:、test:、ci:、chore: 前缀，正文说明原因。协议、动力学及鉴权改动建议邀请其他成员审核。

默认 CODEOWNER 为 @love-of-city，负责接收审核通知。不强制人工批准或代码所有者批准；GitHub 不允许作者批准自己的 PR，但允许作者在必需 CI 通过、分支更新至最新 main、讨论解决后自行合并。

## 环境与本地检查

安装方式及启动命令见 [README](README.md)，环境、协议和配套提交见 [兼容记录](docs/COMPATIBILITY.md)。允许 uv、Conda、venv；CI 固定 Windows + Python 3.11，JavaScript 固定 Node.js 22。Basilisk 按功能检查，不限制安装来源或源码版本。

安装项目测试依赖后，从仓库根目录执行：

```powershell
python -m pip install "ruff==0.15.7"
python -m ruff check .
$env:PYTHONPATH = (Join-Path $PWD 'scripts') + [IO.Path]::PathSeparator + $env:PYTHONPATH
$env:SPACE_SIM_RESET_ADAPTER = (Resolve-Path ../space_sim_UE_Adapter).Path
$env:PYTEST_ADDOPTS = '-p ci_pytest --ci-basic --junitxml=reports/python.xml'
python -m pytest -ra
# 基础检查完成后，恢复普通测试行为。
Remove-Item Env:PYTEST_ADDOPTS
```

服务端需同时安装同级适配器及 `.[test,simulation,posture]`，并运行 frontend 的 npm ci / npm test / npm run build 和 signalling 的 npm ci / npm test / npm run check。独立 Python MuJoCo 用于离线姿态测试；不得与 Basilisk MuJoCo 在同一测试进程加载。

基础套件通过 scripts/ci_pytest.py 加载显式排除清单 scripts/ci-exclusions.json。只有 --ci-basic 生效时才分组；常规完整测试入口不受影响。排除理由和节点写入 reports/coverage-scope.json；任何未预期 skip/xfail 均令基础检查失败。新增基础测试默认纳入，新增排除必须说明所需环境并在 PR 审核，不允许通过批量跳过隐藏失败。

Basilisk 原生测试、独立 Python MuJoCo 测试必须分进程执行；完整测试请按实际依赖分组，不在同时装有两种 MuJoCo 的环境中直接执行全量 pytest。

## 资产与协议

模型及 UE 二进制资产使用 Git LFS；同一不可合并资产先协调负责人再编辑。不提交密钥、账号数据库、录制、日志、Saved、Intermediate、虚拟环境或机器绝对路径。

协议改动需同步生产者与消费者测试、文档和配套 PR。破坏性变更升级协议版本，不静默复用已有版本。配套提交需使用完整 SHA；基础 CI 成功不等于完整仿真兼容，发布前另提供真实仿真、渲染及采集验收证据。

## GitHub 设置

[保护规则及启用步骤](docs/REPOSITORY_SETTINGS.md)区分目标规则与实际启用状态。仓库中的文档不会自动开启远程保护。
