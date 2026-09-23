# 零位展开扩展验收

## 范围与状态

验证自定义六轴目标、取消/断线后的停止与显式恢复、连续场景启动和采集，以及浏览器视频与仿真时钟推进。原生动力学、离线路径规划和网页验收分开执行，不把静态规划成功当作真实运动成功。

本轮扩展验收执行中；全部检查通过前不合并主分支。

## 多目标真实动力学

模型为默认粗碰撞体内部碰撞场景，seed=123。每组在独立 Basilisk 进程中执行，首帧六轴为零；到位后继续检查 90 条稳态观测的位置误差和速度。下表误差为最终观测值，耗时为准备阶段的仿真时间，不包含 UE 启动或进程初始化。

| 用例 | 目标 J1–J6（度） | 路径段数 | 准备时间（秒） | 最终最大误差（度） |
| --- | --- | --- | --- | --- |
| 默认 | 0, -67.6, -86.6, 143.2, -85.5, 0 | 2 | 15.258 | 0.0515 |
| 正向基座与末端 | 12, -67.6, -86.6, 143.2, -85.5, 18 | 4 | 28.508 | 0.0448 |
| 反向基座与末端 | -12, -67.6, -86.6, 143.2, -85.5, -18 | 4 | 28.508 | 0.0449 |
| 加深肘部弯曲 | 0, -70, -90, 145, -85.5, 0 | 2 | 15.425 | 0.0524 |
| 末端转动 90° | 0, -67.6, -86.6, 143.2, -85.5, 90 | 3 | 25.025 | 0.0095 |

以上五组均通过真实动力学测试，不只是参考轨迹测试。报告保存在忽略目录 `run/preparation-matrix/`，每组单独输出 JSON；测试本身在 `tests/test_arm_preparation_native.py`。

### 正确拒绝的目标

- `[0, -60, -75, 130, -75, 0]`：`goal_contact`，目标自身存在碰撞。首次按可达目标运行时被拒绝，之后作为拒绝用例保留，而非隐藏失败或改变原目标。
- `[0, -65, -85, 141, -85, 0]`：`no_valid_candidate_in_finite_search`，当前有限候选路径无一通过；不等于证明该目标不可达。
- 两组拒绝均验证不生成可启动的场景配置文件；覆盖在 `tests/test_preparation_planner.py`。

## 测试隔离

完整回归发现采集 API 测试读取了工作区中正在运行的场景文件，因此触发了新准备门禁。已将该用例的项目根目录放到 pytest 临时目录，避免单元测试受真实平台状态影响；没有放宽生产端的准备到位校验。

本地基础 CI 要求 PowerShell 7 的 `pwsh` 在 PATH 中；缺少该路径导致的跳过不能算测试通过。

本轮已完成基础 CI `620 passed, 23 deselected`（无意外跳过）、前端 `159 passed`、信令 `8 passed`、准备控制专项 `9 passed`、原生场景复原 `6 passed`；原生多目标为五个独立进程各 `1 passed`。其中基础 CI 在扩展异常恢复用例合入前开始收集，新增用例另行专项执行；最终 PR 的 CI 会重新收集全部用例。

异常恢复回归调用实际前端的退出操作、页面可见性、断线、撤权、急停和复原回调，不只修改测试中的布尔变量。取消与失效心跳触发限加速度制动；同一次控制器生命周期内旧请求不能重放，必须实测回零并显式发送新请求。完整复原则由外层 reset-generation 屏障丢弃旧输入；`ArmPreparation.reset()` 自身会清空请求历史，不能将跨复原防重放归功于该模块。

## 复现方式

在服务端仓库根目录执行，使用已有项目 Python 环境；以下原生测试不要与独立 Python MuJoCo 测试拼接到同一 pytest 进程。

```powershell
$env:SPACE_SIM_RESET_ADAPTER = (Resolve-Path ../space_sim_UE_Adapter).Path
$env:SPACE_SIM_RUN_PREPARATION_NATIVE = '1'
$env:SPACE_SIM_PREPARATION_REPORT_DIR = Join-Path $PWD 'run/preparation-matrix'
foreach ($case in @('default', 'positive-base-wrist', 'negative-base-wrist', 'deeper-elbow', 'wrist-quarter-turn')) {
    & ./.venv/Scripts/python.exe -m pytest tests/test_arm_preparation_native.py -k $case -q
    if ($LASTEXITCODE -ne 0) { throw "Native acceptance failed: $case" }
}
Remove-Item Env:SPACE_SIM_RUN_PREPARATION_NATIVE
```

网页验收会替换当前场景，但不允许中断已有采集。验收数据应标注为基础设施测试，不混入抓取示范训练集；日志、浏览器配置、登录会话和数据集均不提交 Git。

### 网页循环验收命令

先启动平台，再在服务端电脑执行。工具使用现有账号数据库创建短期测试会话，结束时撤销，不修改密码或创建管理员。执行期间不要在其他页面手动控制该测试场景。

```powershell
$origin = (Get-Content deploy/ip.local.json -Raw | ConvertFrom-Json).public_url
& ./.venv/Scripts/python.exe tools/verify_arm_preparation.py `
    --origin $origin --auth-database data/auth.sqlite3 `
    --output "run/arm-acceptance-$(Get-Date -Format yyyyMMdd-HHmmss)" `
    --cycles 3 --hold-seconds 600 --record-seconds 5 --allow-replace-scene
```

- 第一轮：默认目标，运动中取消，检查制动和不自动重启，复原后显式重试。
- 第二轮：J1/J6 正向目标，运动中真实刷新页面，依据新文档、服务器新 operator ID 和新 WebSocket 确认断线重连，再验证停止、不自动恢复与复原重试。浏览器刷新未必发送 CDP 的旧连接关闭事件，不能把该事件缺失等同于未断线。
- 第三轮：J1/J6 反向目标，连续观察 600 秒；同时检查后端及浏览器仿真时钟、视频帧计数，超过 5 秒停滞即失败。帧计数速率不是远端客户端显示帧率保证。
- 每轮到位前采集应返回 409；到位后执行短暂真实键盘遥操作并确认实测关节发生变化，再采集至少 5 秒。
- 采集结束检查 LeRobot 元数据、Parquet 行数、连续帧号与时间戳，逐帧解码两路 RGB 视频并核对帧数一致。网页按钮状态按实际异步轮询完成后再检查，不把瞬时 UI 更新延迟误报为采集失败。
- 只有以上检查成功才输出 `status=passed`；失败保留截图、JSON 和数据，并只清理可确认属于本次测试的场景/采集。另有 6 项工具安全测试保证不会中断已有或身份不明的采集。

工具允许 `--goals` 指定 JSON 格式的多组六轴角度。报告、截图保存在指定的 `run/` 子目录，采集数据保存在 `data/episodes/<episode_id>/`。正常结束时保留最后一轮场景运行，测试浏览器与临时会话会关闭。

## 限制

五组姿态通过不代表任意目标可达。短期连续运行不等于小时级或全天稳定性证明；本机浏览器经过部署入口的帧率也不代表其他网络客户端的表现。规划仍是有限候选加静态接触采样，不是连续动态无碰撞证明。
