# 主分支合并后仿真验收

> 历史记录：本文对应 2026-09-20 的基线。2026-09-22 合并后的配置与验证见 [MAIN_INTEGRATION_20260922.md](MAIN_INTEGRATION_20260922.md)，在线调度以 240 / 120 / 30 Hz 为准。

日期：2026-09-20。本报告记录实际测试结果，不以编译成功替代仿真运行验收。

## 结论与版本

两个仓库的 PR #1 均已合并到 main。测试前重新 fetch，并在隔离 worktree 中检出远端合并提交；两个合并提交的文件树与合入的 zmh_v1 代码一致。

| 仓库 | 实际测试的 main 提交 |
|---|---|
| space_sim_server | `0ee90c3d82c622d8eb87b341315b94a06571f3d7` |
| space_sim_UE_Adapter | `522891d17b6750fae9e821d6057811e5e88fa2c5` |

在本报告列出的隔离环境中，主线可以运行真实 Basilisk 仿真、接受 TCP 操作指令、驱动 UE 渲染，并完成场景复原及后续推进。不是只运行 mock，也不是只确认端口打开。

仍不能宣称全库零失败：适配器存在两项既有 BOM 读取测试失败；服务端两项可选模型转换测试跳过。公网浏览器 WebRTC、长时间连续操作及全部接触工况不在本次完整验收范围。

## 自动化测试

| 测试范围 | 合并后结果 | 说明 |
|---|---|---|
| 服务端普通 Python 测试 | 608 passed、2 skipped | 包含 API、控制安全、参考保护/卸载、场景管理、真实 LeRobot v3 写入/读取及本地网关测试 |
| 原生仿真独立进程测试 | 52 passed | 复原、历史记录、姿态控制、目标接触模型及轨道初态 |
| 前端 | 118 passed | 生产 Vite 构建成功 |
| UE 适配器 Python | 66 passed、2 failed，另有 16 subtests passed | 失败为两项既有配置文件 BOM 读取问题 |
| UE 5.6 Development Editor | Succeeded | 合并前编译与合并后增量构建均成功 |

服务端跳过项为 `test_glb_to_mjcf.py` 缺少可选 DracoPy，以及 `test_step_to_mjcf.py` 的 OCP 用例缺少可选 OCP。没有把这些跳过项计入通过项。

适配器失败项：

- `test_foil_material.py::test_override_targets_only_one_fixed_panel_by_exact_asset_path`
- `test_sarm_blanket.py::test_config_only_attaches_to_arm_carrying_bus`

两者用 `encoding="utf-8"` 读取带 UTF-8 BOM 的 `Config/DefaultGame.ini`，触发 `MissingSectionHeaderError`。相应配置及测试文件在本次合并中未修改；本次没有删除测试、绕过断言或顺带修改无关上游文件。UE 实际运行正常，不等于这两项测试已经通过。

初轮未将 PowerShell 7 加入测试进程 PATH，导致较多 Windows 脚本测试跳过。最终补齐 PATH，并为网关测试指定已有 Caddy 二进制，仅使用独立 loopback 端口，得到表中最终结果；不沿用初轮 454 passed、156 skipped 作为最终覆盖范围。

## 真实仿真与 UE 联调

使用默认 `sarm-ground-validation-self-collision-grasp` 场景、固定种子 123、1 ms 动力学步长、100 Hz IK，实时场景的数据采集设置为 false。真实数据采集由独立 LeRobot 测试覆盖，不将两者混称为已完成 UE 相机采集全链路验收。

测试链路：独立 `SimulationHub` → loopback TCP → 原生 `teleop_grasp_unreal.py` → 渲染 TCP → UE 5.6 离屏 D3D 渲染。没有使用 NullRHI。操作指令直接进入真实 Hub，未经过浏览器键盘/UI。

| 验证项 | 实测结果 |
|---|---|
| UE 与仿真启动至首个观测 | 20.594 s |
| 运动输入 | 持续发送 -0.5 rad/s 滚转请求，推进到约 2 s 仿真时间 |
| 实际机械臂运动 | 相对初始状态，六轴中最大实际角度变化 0.82735 rad |
| 复原前有效观测 | 61 帧 |
| 场景复原 | completed，12.172 s |
| 初始关节状态恢复 | 八个关节与初始观测最大差为 0 |
| 复原身份与时间 | 新 reset generation、新 render session，确认帧仿真时间为 0 |
| 复原后推进 | step 16、仿真时间 0.5 s，总计 77 个观测 |
| UE 画面 | 960×540 PNG，804468 字节；已人工查看机械臂、目标卫星、地球及两个相机视图 |

UE 日志确认接收场景清单、在 Game Thread 应用首帧，并在复原后接受新渲染会话。关节位置及速度观测均为有限数值。本次位移只是证明输入驱动了真实关节，不是抖动、接触力或速度精度的完整验收指标。

耗时为本机当次测量，不是性能承诺。测试结束仅清理本次启动的 UE/仿真进程；没有替换或重启原公网部署。

## 环境与依赖限制

未向原运行环境安装新依赖。新建两个本地测试虚拟环境：

- 后端/数据采集环境：Python 3.12.13，安装项目 test/simulation 依赖及独立 Python MuJoCo；通过本地 `.pth` 只读复用已有 Basilisk 安装作为后备模块路径。
- 原生环境：基于现有 Python 3.12.14 Basilisk 环境建立 `--system-site-packages` 虚拟环境，仅在新虚拟环境安装 pytest。不向该原生环境安装 Python MuJoCo。

普通测试进程排除四个会构建原生动力学的测试文件，并在原生环境单独运行，避免同一进程混载不同 MuJoCo DLL：

```powershell
$env:PATH = 'C:\tools\powershell-7.4.13;' + $env:PATH
$env:SPACE_SIM_TEST_CADDY = '<已有 Caddy 二进制的绝对路径>'
& .\.venv-main-validation\Scripts\python.exe -m pytest tests --ignore=tests/test_simulation_reset_native.py --ignore=tests/test_attitude_control.py --ignore=tests/test_ground_capture_native.py --ignore=tests/test_orbital_initial_state.py -q -rs --tb=short

$env:SPACE_SIM_RESET_ADAPTER = (Resolve-Path ..\space_sim_UE_Adapter).Path
& .\.venv-native-validation\Scripts\python.exe -m pytest tests/test_simulation_reset_native.py tests/test_attitude_control.py tests/test_ground_capture_native.py tests/test_orbital_initial_state.py -q --tb=short
```

上述命令假设已准备对应测试环境，不是全新机器的一键安装器。

本次后端数据采集验证所用版本：

| 包 | 版本 |
|---|---|
| lerobot | 0.4.4 |
| numpy | 2.2.6 |
| torch / torchvision | 2.10.0 / 0.25.0 |
| datasets / pyarrow | 4.8.5 / 25.0.1 |
| av | 15.1.0 |
| Python MuJoCo，仅后端测试环境 | 3.13.0 |
| fastapi / pytest | 0.141.1 / 9.1.1 |

**重要兼容性发现：**不约束 NumPy 时，本次依赖解析安装 2.5.3，LeRobot/Datasets 录制收尾出现 `TypeError: only 0-dimensional arrays can be converted to Python scalars`。在其他依赖不变的隔离环境中改用 NumPy 2.2.6 后，真实录制、编码和回读测试通过。仍有对应的数组转标量弃用警告，因此不是永久解决上游兼容性。

本次没有修改项目依赖声明或生产录制实现。新部署不能据此认定任意最新依赖都兼容；需使用已验证的 NumPy 版本，例如在隔离后端环境安装时显式加入 `numpy==2.2.6`，并重新运行录制测试。后续依赖升级应单独处理及回归。

## 本地证据与后续

以下文件在执行测试的 server worktree 的忽略目录 `run/` 中，不随 Git 上传：

- `main-postmerge-server-complete.log`
- `main-postmerge-native.log`
- `main-postmerge-frontend.log`、`main-postmerge-frontend-build.log`
- `main-postmerge-adapter-complete.log`、`main-postmerge-unreal-build.log`
- `main-postmerge-scene.log`
- `main-validation/result.json`、`main-validation/observations.json`、`main-validation/environment.json`
- `main-validation/scene-1789888987928223600.png`、对应 UE 日志及 `simulation.log`
- `validate_main_scene.py`：本次临时联调驱动，使用动态 loopback 端口并负责子进程清理。

后续优先项：固定部署依赖、处理两项配置读取测试、单独验收公网 WebRTC 操作与 UE 相机到 LeRobot 的完整采集链路，再开展长时运行和多构型接触回归。合并代码、验证仿真可运行、替换线上部署是三个不同动作；本次未执行最后一个动作。
