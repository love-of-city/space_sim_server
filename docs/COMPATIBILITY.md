# 环境与配套兼容性

[机器可读配套记录](compatibility.json)固定服务端及适配器完整 SHA，当前状态为候选组合，不代表通过完整系统验收。服务端 CI 使用其中的适配器 SHA；治理 PR 的 CI 验证的是 PR 本身加固定适配器，不会改变记录中的历史验证含义。更新配套提交时另附测试证据。

| 用途 | 要求 |
| --- | --- |
| 基础 CI | Windows、Python 3.11、PowerShell 7；Web 检查使用 Node.js 22 |
| 完整运行 | Windows x64、UE 5.6、VS 2022 C++/Windows SDK、支持渲染和 H.264 编码的 GPU |
| Basilisk | 匹配 Python ABI，具有项目所需模块/API；不限制源码版本、提交或安装来源 |
| 独立 Python MuJoCo | 服务端离线姿态功能使用 posture extra；必须与 Basilisk MuJoCo 分进程 |
| NumPy | >=1.24,<2.4；当前 LeRobot/datasets 录制写入依赖旧标量转换行为，基础 CI 包含真实录制回读测试 |

## Basilisk 功能检查

可以使用包安装，例如对选定解释器执行：

```powershell
uv pip install --python $env:SPACE_SIM_PYTHON "bsk[all]"
& $env:SPACE_SIM_PYTHON scripts/check_basilisk.py
```

也可以自行源码安装，但源码编译不是要求。使用新 Python 版本时，应确认所选安装包提供相应 ABI 的原生模块。

检查脚本从本仓库运行代码中读取 Basilisk 显式导入及模块成员访问，逐项报告缺失模块/API；也检查 MJScene 可用。包括仿真调度、消息、关节 PID/力矩限幅、RKF45 积分器、NBodyGravity/PointMassGravityModel、姿态控制与导航算法。检查只加载 Basilisk 原生模块，不导入独立 Python mujoco。

此命令证明安装可加载所需 API，不证明完整仿真稳定性、GPU 渲染或采集时序；这些仍需在专用环境执行对应测试。缺少某项功能时安装包含该功能的发行包或自行编译补足，不能用普通 Python mujoco 替代 Basilisk.simulation.mujoco。

## 协议与版本

控制协议 space-arm-control/1、渲染协议 bsk-render/2、采集协议 bsk-capture/1。协议版本与 Python 包版本分别管理。破坏性协议改动升级协议标识，并更新双方测试及文档。

适配器 VERSION、根 Python 包、兼容 Python 包及 UE 插件 VersionName 必须一致；基础 CI 执行 scripts/check_versions.py。当前保持 0.2.0，不因治理配置变更升级版本。

基础 CI 的覆盖范围见 CI 下载产物中的 coverage-scope.json 和 JUnit 报告；真实 Basilisk、UE、视频及完整采集链路不在第一阶段验收范围。
