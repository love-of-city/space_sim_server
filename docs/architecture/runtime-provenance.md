# 运行版本与构建来源

SpaceSim 将构建来源、批次来源与实际加载模块分别记录。源码提交号提供背景；内容指纹决定某一批次能否继续使用同一实现。

## UE 构建

`ue-adapter/scripts/build_ue_project.py` 在 UnrealBuildTool 成功后调用 `write_ue_build_manifest.py`。Windows `scripts/build.ps1` 同样生成清单。Development Editor清单位于UE工程的 `Binaries/<platform>/ue-build-manifest.json`；其他Editor配置及Game使用独立的 `ue-build-<target>-<configuration>.json`。非Development项目/插件模块按UBT规则读取带平台和配置后缀的 `.modules`，避免借用旧Development产物或覆盖其身份。

清单记录项目与插件 Source、Shader、Config、uproject/uplugin、Engine Build.version/模块 Build ID、构建目标、平台、工具链证据以及模块二进制 SHA-256。普通 README 不参与 UE 源指纹。写清单工具只供成功构建后的装配步骤使用，不能通过手工重新生成清单代替编译。

采集启动时必须通过源码、引擎、模块注册表和二进制核验；不存在清单、修改源码但未重建、替换库或引擎均拒绝启动。既有原生 helper 源指纹检查继续执行。

## 实际加载的模块

每次 UE 启动接收独立的 `-BskRuntimeReport=<absolute-path>`。Runtime 插件写入实际加载路径、PID 和引擎版本：Editor 读取 FModuleManager 模块路径，单体 Game 记录实际可执行文件，采用临时文件完成后替换。Python 在接收第一帧正式采集前核对：

- 报告 PID 与拥有的 UE 进程相同；复用进程由调度器显式传递报告路径和 PID。
- Runtime 与相机 Shader 模块路径和哈希与构建清单相同。
- 引擎版本及 Changelist 相符。
- Linux 的 `/proc/<pid>/maps` 确实映射这些库，且不存在已替换的 `(deleted)` 映射。

Windows 使用模块管理器报告和新进程身份核验，不宣称已执行 Linux 的映射检查。两平台均要求运行期间构建目录保持不变。该机制用于工程一致性，不是针对恶意篡改者的签名认证。

## 采集来源与恢复

`server/collection/runtime_provenance.py` 汇总以下身份：

- server 的 backend/collection/simulation/native/config/contracts 和原生头文件；adapter 的 Python 源码；模型目录及 UE examples 中实际执行的 Python 场景、相关 pyproject 与批采入口。运行范围内未跟踪源码同样计入，日志、测试输出和文档不计入。
- 当前 native 清单与真实库 SHA；解释器 ABI/内容、依赖版本、Basilisk/MuJoCo/NumPy 安装内容指纹及显式运行参数环境变量。
- 已核验的 UE build ID。
- 批次的配置、计划、布局池快照、源场景、录像配置与已解析的整数 seed。

`tools/collect.py` 在实际执行前自动写入 `launch-provenance.json`，`--plan-only` 不要求启动环境已构建。`--resume` 核对原始身份和输入，变化时拒绝并要求新批次。源码 commit/dirty 状态单独记录；只修改 README 不导致运行身份变化。

批次向子进程传递预期运行指纹；每次正式 recorder 启动重新检查，防止批中热改代码。每条录制目录保存 `runtime-provenance.json`，录制 summary 引用来源指纹和实际 UE 模块报告。Dynamics worker 另外检查已加载原生 helper；它不会为了生成报告而加载新库。

旧批次和旧计划保持原样，仍可读取和离线审核。旧人工 provenance 或已存在任务但缺少新版来源的批次不能被补写为已认证，也不能静默在新版实现上续写。

## 诊断

从 `server/` 执行：

```sh
python tools/deployment/check_runtime_provenance.py --adapter ../ue-adapter --unreal /path/to/UE_5.6
```

错误应修复对应构建或输入；不要删除旧批次的来源文件绕过核验。切换实现使用新批次；恢复旧版本需恢复匹配的源码、原生库、UE 产物和环境。Schema 位于仓库 `manifests/schemas/`，运行生成清单保留在批次或构建产物中。

## 验证范围

纯单元覆盖 stale source/Shader/binary、引擎变化、PID/模块路径错误、冻结输入变化及旧批次拒绝。PowerShell 启动器使用内容变化且时间戳倒退的夹具，避免退回 mtime 判定。真实 UE 编译、运行模块报告、双相机以及 Windows 执行结果需分别附验收证据，单元通过不替代这些检查。
