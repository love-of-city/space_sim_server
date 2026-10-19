# main 合并与 Linux 同步

## 范围与版本

在独立 Windows 工作区 `C:/Users/zmh/Desktop/space_sim_linux_migration` 的 `feat/linux-basic-migration` 分支完成集成，不修改原 Windows 工作区。基线为 `2c8637e3c22bbc0eb2b70bff99335d65abfac4ed`，最新 main 为 `a3030bc`（PR #23–#26，8 个提交）。本地原有录制优化、轨道配置、资产和 Linux 移植先保存为检查点 `572f7c4`，再执行保留两侧历史的 merge。

main 包含统一平台导航和用户管理、独立插头采集、策略控制、可配置噪声与随机化，以及 `drawer-cycle/5` 连续抽屉动作。所有 Git 提交仅在本地，未推送。

## 冲突与语义兼容

处理 5 个文本冲突：数据查看器、自动采集文档、旧抽屉批次入口、单次录制入口和动力学脚本。

- 保留 main 的通用 `collect_scripted_tasks.py`，将 compact 验收、可选原始图片、视频完整解码、200 Hz 力校验和录制配置透传迁入通用入口。旧抽屉入口继续导出兼容函数。
- 保留 Linux UE 路径解析、H.264 流式编码、原生几何优化、照片默认关闭及可选调试轨迹；max_frames 继续服从统一录制配置。
- 同时保留 main 的运动速度、独立插头任务、随机扰动，以及原有 J2/大气阻力配置。关闭诊断 trace 不关闭外部扰动更新或观测延迟历史。
- compact 上下文增加噪声启用时的真实关节状态、测量时间/延迟和腕力传感器上下文；200 Hz Parquet 力数据保留 ground_truth_force_n / ground_truth_torque_nm，纳入已有值校验和。历史不含这些可选字段的数据保持可读。
- Windows 基础 CI 排除明确要求 Linux ELF/本机编译插件的专项测试，并保留其单独实机验证要求。

## Windows 验证

证据位于服务端 `reports/main-merge-20261019/`，适配器基础测试证据位于适配器同名目录。

- 合并相关 Python 测试 307 项通过。
- 服务端完整基础测试首轮 1138 项通过、1 项新增测试因 Windows 默认 GBK 读取 UTF-8 文件失败；补充显式 UTF-8 后，compact 专项 5 项通过。该问题发生在测试读文件阶段，生产记录与数值恢复已完成。
- 适配器基础测试 81 项通过、2 项按基础 CI 清单排除，3 项子测试通过。
- 前端 180 项通过，Vite 生产构建成功；信令 8 项通过及语法检查成功。
- 两侧 Ruff、CI 失败/skip 防护、适配器版本一致性检查通过。
- UE 5.6 Windows 编辑器目标及 BskUnrealRuntime 编译成功；BskUnreal 自动化测试 29 项成功、0 失败。自动化使用 NullRHI，不代表真实 GPU 图像验收。
- 历史完整 compact episode `episode-20261019-092605-fc4fb7e8` 通过新通用验收器：6419 帧、12838 个双相机样本；数值、相机配对、200 Hz 力校验与两路视频完整解码通过。
- 新版真实 BSK/MuJoCo drawer_cycle 成功，任务版本 `drawer-cycle/5`，仿真 34.410 秒；内部 wall_seconds 73.964 秒、task_loop_wall_seconds 29.405 秒、ExecuteSimulation 25.719 秒。环境同时在执行其他验证，这些不是独占机器性能基准。本次没有相机或 LeRobot 录制，不能报告为完整数采倍率。

## Linux 同步规则

目标 `/home/spacesim-zmh/src/space_sim_server` 当前是带 `.migration/` SHA256 清单的源快照，没有 `.git`。同步只更新清单明确列出的源码/文档/资产，不传 Windows DLL、UE 构建输出、node_modules、虚拟环境、账号或训练数据。

以原 source-snapshot、linux-overlay 和 ue-preparation 三份清单合成预期版本，逐文件比对增量。应用前检查 Linux 现有文件是否有未纳入的修改；任何冲突都停止。修改文件先保存回滚归档，更新后再次核对 SHA256。main 删除的旧前端页面也须按清单移除，随后重新构建前端，避免留下旧版静态入口。

Linux 上必须重新运行对应 Python、前端与信令检查，并验证新版真实动力学。UE Linux 安装及完整相机/平台启停验收仍属于迁移后续工作；Windows 构建通过不替代 Linux UE 验收。

## Linux 同步与实机结果

2026-10-19 已将合并提交 `94620f4468f550788239742e67ad3447aced0284` 的增量应用到上述 Linux 源快照。更新包含 97 个文件变化（含删除旧前端入口），压缩包 424180 字节；更新前检查无未处理冲突，更新后全部目标 SHA256 一致。Linux 仍为源快照，本次没有在 Linux 初始化 Git 或伪造分支状态。

回滚备份：`/home/spacesim-zmh/migration/before-main-merge-20261019-170344.tar.gz`。源更新凭据：源码根目录 `.migration/main-merge-20261019.json`。增量工具事先通过只读预检、冲突拒绝、注入故障后的回滚、正常更新与幂等检查。

重新生成本地资产 catalog 后，只有 `sarm_20261012.xml` 中一个紧固件位姿的浮点末位及其 manifest 校验和与 Windows 不同；已逐行检查，未覆盖其他本地修改。Linux 模型 SHA256 为 `c750f498971554d9c421bd2c00bb30a1a85b972f25b5522b3d6b211f70e88b0c`。

- 服务端基础检查首轮：961 通过、1 失败、177 跳过。唯一失败来自后端读取旧的 `frontend/dist`；构建新前端后，整个 collection platform 文件复查为 20 通过、1 项 Windows 专用测试跳过。未将首轮报告改写成全绿。
- 适配器检查：76 通过、5 跳过、2 项按基础范围排除，3 项子测试通过。
- 上述 182 个跳过项均为 Windows/PowerShell 专用入口或 Windows 进程归属测试。基础 CI 插件按设计对 skip 返回非零，故这两次 Linux 命令不算完整基础 CI 通过；对应 Windows 入口已在 Windows 验证，不以安装 PowerShell 或放宽 CI 防护掩盖平台边界。
- Linux 原生 stepper、posture、MuJoCo 运行库隔离：32 通过；独立资产插件：1 通过。
- 前端 180 通过，生产构建成功；信令 8 通过，语法检查成功。
- Linux 资产 catalog 更新成功，仍只代表 catalog 就绪，未执行 Linux UE 导入。
- 新版真实 BSK/MuJoCo `drawer-cycle/5` 成功：仿真 34.410 秒；脚本进程耗时 41.232 秒（约 0.83×）；脚本内部 wall_seconds 39.818 秒、task_loop_wall_seconds 20.222 秒、ExecuteSimulation 18.347 秒。使用与 Windows 验证相同的场景配置；未开启相机或 LeRobot 录制，不是完整数采吞吐基准。

Linux 证据：`/home/spacesim-zmh/migration/main-merge-20261019/`。已回收至 Windows 服务端 `reports/main-merge-20261019/linux/`，包含原始失败与复查日志、JUnit、真实任务 summary、源更新凭据和最终文件审计。

## 后续 UE 安装限制

Linux 已存在用户上传的 `Linux_Unreal_Engine_5.6.1.zip`，大小 30868763590 字节（约 28.75 GiB），ZIP 中央目录可读取；尚未完成整个压缩包的 CRC/来源校验。包内解压总大小 67391736152 字节（约 62.76 GiB），当时磁盘剩余约 45.90 GiB，直接完整解压至少差约 16.86 GiB，且还需预留编译和缓存空间。本次保留原上传文件，没有启动解压。Linux UE 编译、图像录制和完整网页进程生命周期仍待后续验收。
