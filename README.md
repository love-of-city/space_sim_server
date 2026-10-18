# 太空机械臂遥操作与数据采集平台

浏览器操作台、权威动力学、UE 渲染和训练数据采集在一个仓库中维护。一个 Git 提交固定整个平台的代码版本。

| 目录 | 功能与文档 |
| --- | --- |
| [space_sim_server/](space_sim_server/README.md) | FastAPI 后端、网页、Pixel Streaming 信令、仿真、模型与数据采集 |
| [space_sim_UE_Adapter/](space_sim_UE_Adapter/README.md) | Basilisk/MJScene 发送端、UE 5.6 渲染与相机采集 |

团队开发请阅读[贡献指南](CONTRIBUTING.md)。环境要求、Python 管理器选择、部署和完整启动参数见[服务端安装指南](space_sim_server/README.md)。

原工作台提供机械臂操作、7 km交会、300 km入轨后三种起始条件。连续任务保留同一任务编号，远距通过后接续近距，再以实际关节终态进入标准操作场景；两阶段共享GNSS配置和导航不确定性。近距仍是参数化视觉与逻辑捕获，未验证真实图像闭环或物理锁定。详细流程见[连续任务说明](archive/2026_10_17_连续任务与共享导航.md)和[远距物理参数](space_sim_server/docs/TEAM_RENDEZVOUS.md)。

交会使用原控制台与原 UE 主视口，可在真实尺度的双星总览、追踪星近景、目标星近景间切换；总览显示身份、质心距离、比例尺与地影状态。团队最新 SARM 实验模型仍在直接遥操作的场景选择器中，保留其独立模型身份和夹爪角度单位，见[新模型说明](space_sim_server/docs/SARM_20261012_PLATFORM.md)。

当前整合以团队main `7f01764`为基础，沿用原导航、自动采集、数据集回放和用户管理页面。此前固定尺度总览等工作保留于[10/13整合记录](archive/2026_10_13_最新主线交会整合与真实尺度显示.md)，其中的版本和验收数字属于当时记录。

动画预览使用连续播放时钟；当前主线启动脚本默认目标90 FPS，网页可选择30/60/90 FPS，实际帧率取决于运行负载。同轨迹对照、权威采集兼容性和此前实测结果见[动画性能优化](archive/2026_10_13_交会动画播放时钟与预览性能优化.md)。

平台碰撞、预览时钟与短录制编码的性能改进及验证方法见[性能优化记录](archive/2026_10_14_平台碰撞与预览采集性能优化.md)。更新后需重新构建 SARM 原生插件与 UE 运行时。

## 首次获取与安装

完整运行需要 Windows x64、PowerShell 7、Node.js 22、UE 5.6、Visual Studio C++ 工具、GPU，以及与 Python 版本匹配且具备所需 API 的 Basilisk/MJScene。

默认 local 动力学还要求 Basilisk 所带 MuJoCo 与项目内置的 3.7.0 头文件匹配。本次完整验收使用 Python 3.13 + bsk 2.11.1；安装该已验证环境时可选择 bsk[all]==2.11.1。使用其他发行版或自编译版本时也需满足这一原生 ABI 要求，单独通过 API 导入检查不足以证明兼容。

只获取一个仓库；不要在两个子目录中再次 clone：

```powershell
git clone https://github.com/love-of-city/space_sim_server.git
Set-Location .\space_sim_server
git lfs install --local
git lfs pull
Set-Location .\space_sim_server
```

最后一步进入**服务端子目录**。下面使用已经选定的仿真解释器；配置方法及 uv、Conda、venv 安装方式见服务端指南：

```powershell
& $env:SPACE_SIM_PYTHON -m pip install -e "../space_sim_UE_Adapter[test]" -e ".[test,simulation]"
npm.cmd --prefix frontend ci --no-audit --no-fund
npm.cmd --prefix signalling ci --no-audit --no-fund
& $env:SPACE_SIM_PYTHON scripts/check_basilisk.py
$env:UE56_ROOT = 'D:\UE\UE_5.6' # 替换为本机路径
pwsh -NoProfile -File ..\space_sim_UE_Adapter\scripts\build.ps1 -UnrealRoot $env:UE56_ROOT
pwsh -NoProfile -File .\scripts\run_platform.ps1 -ApiPort 18000
```

uv 环境可使用指南中的 `uv pip install --python ...`，无需安装 pip。安装项目包不会安装 UE、编译器或 Basilisk。

打开 `http://127.0.0.1:18000`，登录后生成并启动场景。停止命令仍从服务端子目录执行：

```powershell
pwsh -NoProfile -File .\scripts\stop_platform.ps1
```

## 验证与旧工作区迁移

根 CI 运行 `server-python`、`server-web`、`adapter-python`，均使用本次 checkout 中的代码。基础 CI 不替代 Basilisk、UE/GPU、WebRTC 与数据采集验收。

旧工作区迁移时建议全新 clone，重新安装两个 editable 包、重新构建 UE，并按新路径生成资产 catalog。已有 episode、认证数据库与机器配置另行保留，不将旧 `run/`、`Saved/` 或构建输出复制到新 checkout。

合并基线、历史和 LFS 验证、切换步骤见[仓库合并记录](archive/2026_10_09_repository_merge.md)。旧适配器标签 `adapter/v0.2.0` 表示其历史版本，不表示整个平台的发布版本。
