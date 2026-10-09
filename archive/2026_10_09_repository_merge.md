# 2026-10-09 统一服务端与 UE 适配器仓库

## 问题与结果

原服务端与 UE 适配器通过两个独立 main 配套运行，服务端 CI 会拉取适配器当前 main，无法通过一个提交固定完整系统版本。现在沿用 love-of-city/space_sim_server，在一个 Git 仓库中维护两个同级子目录。现有 Python 包、渲染/控制/采集协议和服务端子目录内的启动命令继续使用。

GitHub 配置、贡献指南与安装入口放在根目录。原项目各自的模型、UE 资产、技术文档和 archive/ 保留在子目录中。工作区外层 Git、Basilisk 源码、本机环境和运行数据不导入。

## 基线、历史与备份

| 项目 | 导入的 main |
| --- | --- |
| 服务端 | 82d896963cd4b742d4d52600fbf689f0fa9b57a4 |
| 适配器 | bf6d3f3892abd66bb892e188463eee3428dce93f |

服务端以普通移动提交进入 space_sim_server/；适配器通过不带 --squash 的 git subtree add 进入 space_sim_UE_Adapter/。原提交 SHA、作者及时间保留；旧提交仍使用其历史目录布局。

纯移动与导入完成时，1,445 个源文件的 Git blob 和文件模式与基线完全一致，两个原 main 均为导入提交 a0347691e8406ec5e7862a85f75a71ae17eda418 的祖先。后续只修改明确的 CI、文档、路径及相应测试，最终变更清单见同目录 repository_merge_audit.json。

两边的所有本地引用及当前远端分支、标签保存为独立镜像和经过验证的 Git bundle；LFS 对象另行备份。备份位置在操作者的工作区记录中，不提交本机路径。适配器未进入 main 的本地 chore/team-governance 分支留在备份中，没有自动合入。

原适配器标签以 adapter/v0.2.0 保留，仍指向原 annotated tag 对象，表示历史适配器版本。

## LFS、路径与 CI

- 保留子项目原 .gitattributes 与忽略规则；根规则只管理公共文件和环境缓存。
- 适配器全部 754 个历史 LFS 对象已上传到服务端仓库的 LFS 存储。当前 checkout 包含双方共 711 个 LFS 文件；对象完整性检查通过。
- 新布局默认使用当前仓库内的适配器。缺失 bundled renderer 时直接报错；明确传入 AdapterRoot 仍可覆盖。旧独立 checkout 的路径发现保留兼容逻辑。
- 修正若干测试和诊断工具的旧双层目录假设；可视化及远端验证脚本采用已有 Python 解释器解析器。
- 根 CI 保留 server-python、server-web 并加入 adapter-python。各 job 只 checkout 当前仓库一次，删除跨仓库拉取。工作目录、npm lock 路径、缓存命名与报告路径适配新布局。
- 部署路径过滤器按新仓库路径匹配，并覆盖适配器启动脚本与两边依赖文件。main push/手动运行继续执行完整基础测试。
- 两套 CI 排除清单与 stale node 守卫保留。子目录中失效的 GitHub 配置移除，统一根 PR 模板和治理配置。

## 验证

| 验证 | 结果 |
| --- | --- |
| 独立 Python 3.11 环境安装、依赖一致性 | 通过 |
| 服务端 Ruff、失败/skip/xdist 守卫、基础 Python | 707 passed |
| 适配器 Ruff、失败/skip/版本守卫、基础 Python | 71 passed，2 deselected |
| 前端测试 / 构建 | 166 passed / 通过 |
| 信令测试 / 语法 | 8 passed / 通过 |
| actionlint；三个 job 单仓库 checkout | 通过 |
| 部署过滤器正负路径 | 10 个场景通过 |
| UE 5.6 Editor 完整构建 | 通过 |
| UE BskUnreal 自动化 | 27 项通过 |
| mock 断开及重新连接 | 两次连接通过 |
| GPU 权威 RGB 采集 | idle/STOP、两个 episode、在途 reset、帧身份与方向通过 |
| Basilisk/MJScene 真实动力学及重置 | 6 passed |
| 完整平台 | 控制 20 条命令、网页解码视频、90 帧/180 张权威图像、reset、停止与 API 健康检查通过 |
| 远端新 clone | 分支发布后验证并更新本记录 |

原本机 Basilisk 2.12 开发环境所带 MuJoCo 与项目内置 3.7.0 头文件不匹配，无法用于 local stepper 验收。本次另建 Python 3.13 + Basilisk 2.11.1 环境完成匹配的原生测试，未改变原环境或动力学实现。仅基础 CI 成功不代表 UE/GPU 或仿真验收通过。

## 切换与回退

1. 此迁移 PR 必须通过 Create a merge commit 合入；禁止 squash/rebase。合入后重新确认两个原主线 SHA 均为 main 的祖先。
2. 协作者全新 clone 统一仓库并拉取 LFS，重新安装两个 editable 包，再构建 UE。运行时按新路径生成 catalog，不复制旧 Saved/、Intermediate/、run/ 或 DLL。
3. episode、认证数据库和机器配置由各部署单独保留并迁移；原 checkout 和备份保留至切换验收完成。
4. 管理员将实际 required checks 配置为 server-python、server-web、adapter-python。根 branch-protection.json 记录期望配置，不自动改变 GitHub 规则。
5. 完成切换后，在原适配器仓库保留迁移说明并由管理员归档。适配器旧 PR、Issue 和发布记录继续留在原仓库供查询。
6. 需要回退时通过新的 PR 逆向恢复原布局，不改写 main 历史。LFS 备份、原仓库和原始 SHA 可用于恢复。
