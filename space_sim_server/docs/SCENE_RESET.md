# 当前场景状态重置

## 使用

场景启动且仿真控制连接建立后，“场景实例”区域会出现可用的 **重置状态** 按钮。

- 恢复本次实例启动时的初态，不重新抽样 Seed、轨道相位或目标/机械臂参数。
- 卫星和目标的位置、姿态、线/角速度，以及机械臂、夹爪、反作用轮、目标活动板件的状态一同恢复。
- 控制器、接触求解及星历时钟重新初始化，仿真从 `t=0` 继续推进。
- UE 进程和 Pixel Streaming 不重启；BSK → UE 状态连接会建立新的渲染会话。
- 重置期间按钮和操控暂不可用。完成后须重新点击画面进入操作，不自动恢复按键/摇杆输入。
- 正在采集时按钮禁用，须先结束当前采集。后端也强制校验，不能绕过按钮跨时间线录制。
- 只允许场景创建者或管理员重置；不支持重置的旧仿真进程不会启用按钮。

重置会重新加载并构建本场景的动力学模型，因此不是瞬时操作。界面显示“正在重置…”直到收到新会话观测。
接口等待最多 60 秒；超时不等于重置取消，操控保持禁用。若迟到确认抵达会自动解除；也可停止场景。

## 实现边界

```text
frontend: POST /api/scenes/reset
  → SimulationHub.begin_reset(): 先锁定操控/采集入口，清除缓存指令
  → 撤销操作页面控制权、清理 deadman 指令
  → control TCP: {protocol: "space-arm-control/1", type: "reset", request_id: "..."}
  → SimulationControlClient 接收线程只保存请求
  → run() 在 ExecuteSimulation() 推进段之间消费请求
  → 关闭旧渲染桥、卸载旧 SPICE kernel、释放旧核心
  → _run_session() 使用同一份已加载的初始参数重新构建 BSK/MJScene
  → 新渲染 session_id / manifest / scene_reset / 状态帧
  → 新 observation 携带 reset_generation 与 render_session_id，确认完成
```

`reset_generation` 是控制指令的隔离标识。重建期间拒绝指令，重建后拒绝旧代际指令。
同一重置请求在控制链路重连后可以重发，不会重复重置；正常重连也不重放之前的动作。

UE 以新渲染会话为边界，清除接收队列中的旧帧/事件、显示插值与外推历史、已应用帧缓存及采集时间门限。
新帧必须在对应 manifest 被消费后才应用；首帧直接显示，不能从旧位置插值过去。
采集包已有的 `session_id` 与服务端新观测的 `render_session_id` 比较，旧会话迟到图像不进入新采集。

物理初始化仍复用原 `_build_simulation()`、`_initialize_state()`、`_apply_orbital_initial_state()`；
没有修改模型 XML，也没有在 UE 中创建另一套权威物理状态。

## 两个隔离 worktree 的启动方式

在服务端 worktree 中运行，显式指定配套 UE worktree，避免误用原目录中的适配器：

```powershell
Set-Location C:\Users\LYH\space_sim_server_reset
.\scripts\run_platform.ps1 `
  -AdapterRoot C:\Users\LYH\space_sim_UE_adapter_reset `
  -ModelRoot C:\Users\LYH\space_sim_server_reset\model\SARM\platform `
  -UnrealRoot 'C:\Program Files\Epic Games\UE_5.6'
```

不要在原平台占用同一组端口时同时启动；需要并行运行时，显式传入独立的 API/Control/Capture/Render/Pixel Streaming 端口。

## 回归测试

- `tests/test_scene_reset.py`：权限、采集互斥、请求确认、重复请求、超时锁定、重连、旧会话采集隔离。
- `tests/test_simulation_reset_native.py`：在真实 BSK/MJScene 中连续重建两次；主动扰动卫星、目标、轮速和活动板件后，检查每次 `t=0` 的完整 `qpos/qvel` 与初次启动一致，保留轨道速度及实例 Seed。
- `frontend/tests/scene_reset.test.mjs`：按钮禁用、重复点击、输入清理、错误反馈、不自动重新授权。
- UE `BskUnreal.Presentation.SceneReset` / `BskUnreal.Protocol.ResetReceiveBarrier`：插值/采集时钟清理、跨会话旧帧拒绝、manifest 先于帧的接收边界。

原生测试需使用包含 Basilisk 的 Python 环境，并具备 pytest、pydantic 等测试依赖；不要在同一进程中混用 Python MuJoCo 和 Basilisk MuJoCo。
可通过 `SPACE_SIM_RESET_ADAPTER` 指定测试适配器目录。
