# 独立工作区仿真验收（2026-09-23）

本记录对应 Windows 本机的一次完整部署验证。发布到现有 `zmh_v1` 分支；不改变 main，不改写已有分支历史。

## 配套代码

| 仓库 | 验收的代码提交 |
| --- | --- |
| love-of-city/space_sim_server | `517dfc3a222214de622da19c1d9c82355bb5316f` |
| love-of-city/space_sim_UE_Adapter | `dcd2d945eb04258dddae7d064dce2769b4e12149` |

两者均为验收时 fetch 确认的 main 最新版本。此验收记录之后的本次提交只增加文档，不改变运行代码。本组合通过实机验收，不将此前兼容记录中其他组合的结果视为本次结果。

## 工作区原则

只保留一个固定工作区，两个仓库同级、分别独立克隆，使用 Git 分支管理版本，不按日期复制代码目录。各仓库的 `.git` 与 LFS 对象不依赖其他 worktree、对象 alternates 或历史目录。

服务端使用本仓库 `.venv`。Python 3.12.13、NumPy 2.2.6、MuJoCo 3.7.0、LeRobot 0.4.4；Basilisk 来自已安装且通过必需 API 检查的系统环境。Python MuJoCo 姿态检查与 Basilisk MuJoCo 必须分进程。依赖检查通过。

账号数据库、采集目录、本机部署配置、加密凭据、证书和部署工具保留在固定工作区，不上传 GitHub。复制的 660 个历史采集、压缩包和任务文件逐文件 SHA256 一致。

已有部署配置可在服务端根目录通过 PowerShell 7 调用现有入口：

```powershell
./scripts/remote_visualization.ps1 -Action Start -Python "$PWD/.venv/Scripts/python.exe"
./scripts/remote_visualization.ps1 -Action Stop
```

Start 启动平台，登录网页后创建场景；Stop 前应结束采集。配置迁移需更新 Adapter、模型、Caddy 路径，并按新配置绝对路径重新命名对应 DPAPI 存储，保留证书缓存。不要复用旧 PID 状态。

## 验收结果

| 检查 | 结果 |
| --- | --- |
| 服务端主测试组 | 783 passed，3 skipped |
| Basilisk 原生测试组（独立进程） | 54 passed |
| Adapter Python | 87 passed，19 subtests passed |
| 前端 | 132 passed，生产构建成功 |
| Signalling | 8 passed，语法检查成功 |
| UE 5.6 Editor Development | 26 个构建任务，成功 |
| 环境整理后录制回归 | 29 passed |
| Ruff、CI 失败保护、Adapter 版本检查 | 通过 |

跳过项为可选 DracoPy、OCP，以及需显式启用的旧原生准备测试，不计为通过。日志、截图和机器信息保存在部署本机，不作为源码提交。

实机链路验证：HTTPS 登录入口正常，仿真时间持续推进，writer_ready=true；240 Hz 动力学任务、120 Hz IK、30 Hz 权威采集。指定操作姿态直接启动的目标角度 `[0, -67.6, -86.6, 143.2, -85.5, 0]` 度，初始观测最大误差约 0.000001 度。

8 秒静态双相机验收录制生成 240 条 Parquet 样本和 2 路 MP4，dataset_status=complete。记录标注 workspace-validation，不是成功抓取演示。浏览器 WebRTC 获得 1280×720 实时画面，服务器本机约 2.996 秒新增 245 个解码帧；不代表异地用户网络性能。

## 提交边界

本次同步最新代码并提交验收文档，不提交密钥、账号库、采集文件、日志、虚拟环境、UE 构建缓存或机器绝对路径。UE 准备阶段重新生成的材质、网格和导入资源仍保留在本机，不把自动生成差异当作手工源码改动上传。可通过仓库现有准备流程重建。

本次没有修改控制算法，也没有执行长时间运动稳定性和异地公网压力测试。清理历史工作区后应再次确认 API、仿真进程、采集写入器、Python 导入和独立 Git 对象可用。
