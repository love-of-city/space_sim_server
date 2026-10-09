# 点击开始后才启用严格采集

## 背景、范围与跨仓库关系

旧实现将 `dataset_capture`（场景采集能力）等同于正在录制：未点击开始也执行 UE 权威截图、回读、压缩、发送，并使用可靠渲染 FIFO。下游变慢会积压旧状态，甚至反压仿真线程。本次将能力和活动 episode 分离；不更改动力学模型、质量/惯量、积分参数、控制律、相机标定或 RGB 编码格式。

此 PR 依赖 `love-of-city/space_sim_UE_Adapter` 同名分支 `feature/on-demand-capture-20260924` 的 Bridge 标记、分离队列及 UE 按需采集实现。adapter 保持旧消息兼容，建议先合并 adapter PR，再合并本 PR；运行部署必须同时更新两端并重启，不得仅刷新浏览器或替换 Python 文件后使用旧 UE DLL。

## 改动组织

- `CaptureLifecycle` 统一 episode、任务、停止场景及关闭后端的启停入口，串行化转换；冷启动准备和 HTTP 取消均有收尾，避免遗留孤立 episode。
- START 先准备 Recorder，再发 `capture_control`。只有带相同 request ID 与 episode ID 的权威帧 observation 到达才确认生效；不把 socket 发送成功等同于开始采集。
- 控制客户端保存采集状态，Bridge 每次发布仅采样一次；该帧和紧随其后的 observation 使用同一份标记，不影响原始权威载荷。
- Recorder 只接受本 episode 的状态与图像，排除开始前的预览、上一次采集迟到的图像。图像可以先于对应 observation 到达并正常配对。
- STOP 先冻结已接纳状态的末帧边界，再发 OFF。未配对且超出边界的图像不会变成新样本；已接纳样本继续等待对应图像并排空写盘，禁止清空严格队列来伪造成功。
- 同一个 observation stream 重连会重发预期模式，新仿真进程不继承旧活动 episode。启停未确认时记录错误，不宣称数据完整。
- `capture_on_demand_v1` 纳入启动能力预检；旧后端或旧仿真进程必须更新后再开启相机采集。无相机的纯状态记录保留原行为。
- 更新控制协议 schema、前端帮助文字和操作说明。场景开关表示“具备采集能力”，不是“立即截图”。

## 验证证据与界限

2026-09-23 功能实现验收：

- 相关服务端、Recorder、LeRobot、传输、重置和时钟测试 189 通过、8 跳过；跳过不计作完整验收。
- 配套 adapter Python 测试 29 通过，另有 16 个子测试通过；UE 5.6 插件完整编译链接成功，UE 自动化 27 项通过。
- 独立端口合成场景真实 UE/GPU 双相机 smoke 连续两轮通过：未开始 0 张；第一轮 6 个状态对应 12 张图像；停止后无新增；第二轮 3 个状态对应 6 张；停止后无新增。检查帧号、episode、相机和实际 JPEG 解码。

2026-09-24 基于最新 main 整合后，重新执行相关 Python/网页回归及仓库规定的 CI 入口，远程 PR 的最终提交 Actions 为基础 CI 证据。没有重启或操控用户正在运行的生产场景。基础 CI 不含 UE/GPU：真实截图测试按文件明确列入 `scripts/ci-exclusions.json`，其余按需生命周期与网络测试仍运行，禁止用意外 skip 充当成功。

可复现入口：`tests/test_capture_on_demand.py`；真实图像检查为 `tests/test_capture_on_demand_renderer.py`，需显式设置 `SPACE_SIM_UE_CAPTURE_SMOKE=1` 和环境对应的 `UE56_ROOT`，adapter 可通过 `SPACE_SIM_RESET_ADAPTER` 指定。测试使用独立端口和临时目录，不读取用户账号库或写入真实 episode。

## 风险与后续

- 该方案消除未采集时的权威截图开销，不保证正式采集时没有吞吐瓶颈。采集中仍保留严格 FIFO 和原来的预览降频策略。
- STOP 可等待队列收尾；图像或确认超时会失败，不静默缺帧。
- 完整部署须重启后端、仿真及 UE，并确认新 DLL 已链接；性能结论仍需真实生产场景测量。
- 不提交日志、录制数据、账号库、机器绝对路径、构建产物或本地环境。另一个仓库中原有材质资产改动不在此次提交范围内。


### 2026-09-24 提交前复验

- Ruff 0.15.7、CI 失败防护、前端 159 项测试及 production build、signalling 8 项测试与语法检查通过。
- 旧 API 生命周期测试更新为两种显式语义：没有仿真时相机采集返回 409 且不创建 episode；纯状态记录 `camera_ids=[]` 仍正常启停。没有放宽正式采集的握手条件。
- 首次完整 basic profile 在本地原生动力学环境执行：610 passed、6 failed、33 skipped；按 CI 规则整体为失败，不报告成通过。1 个失败是上述过时 API 测试，其余为该环境未安装声明的 posture/mujoco 依赖及新主线任务盒 LFS 资产未下载；部分旧视频测试还依赖特定相邻目录布局。上述问题不通过增加通用排除或移除断言来掩盖，最终基础 CI 以标准依赖环境中的 PR Actions 为准。
- 测试报告、依赖安装和暂存备份只保留本地，不纳入提交。新的实时图像 smoke 不在当前运行中的用户 UE 上执行。

- 更新 API fixture 后，针对采集生命周期、协议、LeRobot、Recorder、重置、采样时钟及相关 API 的复验为 **158 passed**，无跳过。
- 任务盒 LFS 资产后续已恢复到工作区；模型选择与任务盒模块测试复验通过。声明的 posture/mujoco 依赖不安装进正在使用的原生动力学环境，完整基础 CI 留给标准远程环境。
