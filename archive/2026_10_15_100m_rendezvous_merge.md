# 100 m 近距交会分支合并与验收

日期：2026-10-15（Asia/Singapore）。源分支为 `feature/100m-rendezvous-integration`，目标分支为 `feature/yipuwang-rendezvous-integration`。

## 分支基线与合并方式

- 源提交：`cc305c39027cf0307c96028f43c8b8ea20046c75`。
- 合并前最新目标：`c62c90d5799f6f310e116371799cc7f89c6e6969`。
- 核对时最新 main：`4f1a3217a9ca01dda50bfdfa8dbe704f1612ce24`，已是目标分支祖先。
- 双方共同祖先：`7930c4de3529502e0ae6bcb1d30f85fc560868fa`。最新目标独有 19 个提交，源独有 1 个提交；原本地目标落后远端 19 个提交。
- 在独立 worktree 中先快进目标至远端，再执行 `git merge --no-ff --no-commit feature/100m-rendezvous-integration`，验证后创建双亲合并提交。

源变更共 28 个文件，涵盖近距动力学与参数化导航、任务 API、网页、UE HUD/显示补光、文档和根 CI 部署路径过滤器。四个共同修改的文件自动合并，无文本冲突；目标分支的任务恢复选择保护、腕部测力和性能改动一起保留。

## 基础与 UE 验证

以下均在合并 worktree 执行。基础环境为现有 Python 3.11.15 CI 环境，原生环境为现有 Python 3.13.5/Basilisk 环境；本地 Node.js 为 24.14.0。Python 导入路径指向合并 worktree。

| 检查 | 结果 |
| --- | --- |
| uv 依赖兼容性、两项目 Ruff、CI guard、适配器版本一致性 | 通过 |
| server-python：`pytest -p ci_pytest --ci-basic -n 4` | 817 passed，含部署组，0 skipped |
| server-web：前端测试、构建 | 197 passed，构建通过 |
| signalling：测试及语法检查 | 10 passed，通过 |
| adapter-python：基础 CI | 97 passed，另 3 subtests，2 deselected |
| 原生解释器下适配器完整 Python 合约测试 | 120 passed，另 19 subtests |
| UE 5.6 Editor C++ 全新 worktree 构建 | 通过 |
| `BskUnreal` NullRHI 自动化 | 33 项 Success，退出码 0 |
| mock 两次连接/断开/重连 smoke，端口 5561 | 通过 |

基本 CI 的 deselection 属于既有排除清单；不把基础 CI 当成完整仿真验收。图形结果另见下文。

## 近距原生与独立数值验证

重新构建当前源码对应的 LocalMujocoStepper 和 SARM 插件，再运行 seed=7 的正常、Grasp 阶段失视觉场景。原生进程不导入独立 Python MuJoCo；静态几何核查使用 CI 环境。

| 场景 | 运行验收 | 独立复核 | 结果 |
| --- | --- | --- | --- |
| `run/merge-near-nominal-retry` | 11/11 | 12/12 | 1819.85 s 进入 CaptureHold，保持至 1824.85 s |
| `run/merge-near-loss` | 8/8 | 12/12 | 1810.85 s 进入 SafeHold，闭合禁止、手指只张开、臂参考保持、无 latch |
| `run/merge-near-handoff-retry` | 6/6 | 专项检查 | 非零时刻、姿态、速度、飞轮、估计及协方差连续 |
| `run/merge-near-geometry.json` | 4/4 | 82 个静态配置 | 所采样配置接触数为 0，预抓取与终点可达 |

正常终态独立真值：TCP 间隙 18.7736 mm，横向误差 0.13998 mm，角误差 0.02774°，相对速度 0.087685 mm/s；最大合力 10.2568 N。传播期间 external state writes 为 0。近距运行继承目标性能配置，重力遥测的 `force_stride_steps=8`。

可从 `space_sim_server/` 复现：

```powershell
& $env:SPACE_SIM_PYTHON -m simulation.near_field.run --fixture --config simulation/near_field/configs/nominal.json --output-dir run/merge-near-nominal
& $env:SPACE_SIM_PYTHON -m simulation.near_field.run --fixture --config simulation/near_field/configs/vision_loss.json --output-dir run/merge-near-loss
& $env:SPACE_SIM_PYTHON tools/verify_near_field_handoff.py --output-dir run/merge-near-handoff
```

独立复核使用 CI 环境运行 `tools/verify_near_field.py --run <输出目录> --output <结果.json>`；静态核查使用同一工具的 `--geometry`。

## 网页、图形与本地证据

使用合并 worktree 的服务端、前端、适配器 Python 和新构建 UE，在本机 18001 端口启动平台。通过浏览器登录、选择近距模式并启动实际任务，保存阶段截图与浏览器结果。

正常网页任务完成，结果为 `CaptureHold`，1280×720 视频解码正常，无 pageerror；任务面板及 UE HUD 显示近距阶段和逻辑捕获状态。对应原生输出为 `run/team_rendezvous/48802ec8c6a2443995ad094141a46f9b`，独立复核 12/12 通过。

Grasp 失视觉网页任务完成，结果为 `SafeHold`，同样有 1280×720 视频且无 pageerror；页面显示“安全保持 · 闭合已禁止”，不开放远距任务的遥操作交接按钮。输出为 `run/team_rendezvous/63c52865d9b34080af68250f25f81971`，独立复核 12/12 通过。已实际查看两种终态截图，确认 UE 模型和阶段 HUD 可见。验收平台及本轮渲染进程已停止。

本地证据统一保存在 worktree 的 ignored `tmp/merge-validation/`：基础检查日志、UE 构建/自动化/smoke 日志、数值汇总、`browser-normal/` 与 `browser-loss/` 的 JSON 和阶段截图。运行产物另在 ignored `space_sim_server/run/`，不提交数据与录制。

环境处理与失败记录：

- CI 环境由 uv 管理、不含 pip，使用 `uv pip check --python <CI解释器>` 完成依赖检查。
- 并行 UE 构建期间，首次正常场景与交接探针分别出现原生初始化异常和进程退出；构建日志同期记录内存压力。未改实现，构建结束后重跑全部通过，未据此断言具体根因。
- 完整适配器测试需使用具备 Basilisk 的原生解释器；CI 解释器用于既有基础 CI 筛选。
- 新 worktree 的 UE 材质生成触发 119 字符缓存路径限制；通过进程级 `UE-LocalDataCachePath` 指向短目录后完成准备。该变量需传给后续平台/UE 子进程。
- 复用旧 editable 环境时，图形验证使用临时 import bootstrap 确保适配器来自合并 worktree；验证后删除。UE 重新导入产生的已跟踪资产改动恢复至合并 index，不纳入此次合并。

本次证据为合并后的代码、基础检查、原生数值仿真、UE 自动化与网页图形验证。双目仍为参数化观测，捕获仍为逻辑代理；交接专项使用合成输入，不代表真实远距 accepted 产物至捕获全程连跑、图像算法或真实接触/载荷传递验收。
