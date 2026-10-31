# UE 与原生分层验证

快速 CI、真实物理、UE Automation、GPU 录制和发布验收是不同证据。新增 workflow 提供执行入口；仓库分支保护、受控 runner 和环境变量需要在托管平台实际配置。仅提交 workflow 不代表远端检查已经运行。

## 执行环境

原生 runner 标签为 `self-hosted, Linux, X64, spacesim-native`；UE/GPU 为 `self-hosted, Linux, X64, spacesim-ue-gpu`。配置 `SPACE_SIM_VALIDATION_PYTHON` 和 `SPACE_SIM_VALIDATION_UE_ROOT`，分别指向独立验收环境和允许使用的引擎。不能指向正式采集的可变环境。通过 GitHub 的 `spacesim-validation` / `spacesim-release-validation` 环境限制可执行分支及审批规则。外部分支 PR 不直接在自托管机器执行；需要维护者审阅后在受控分支验证。

`Native validation required` 与 `UE validation required` 是稳定汇总状态。相关修改若被跳过、缺 runner 或失败，不能算通过。无关文档变更可由路径检测明确跳过。分支保护需要将这些状态设为必需；本机无法代替远端配置。

当前自动 workflow 明确针对 Linux。Windows 工具入口保留，但 Windows 编译、环境重装与 GPU 证据必须在 Windows runner 独立取得，不能引用 Linux 结果。

## 本地命令

从仓库根执行，`SPACE_SIM_PYTHON` 指向本次候选环境，`UE56_ROOT` 指向验收引擎。环境策略核验具体 Engine PatchVersion、Changelist、CompatibleChangelist；Linux 还核对关键插件 descriptor 的内容。

```bash
"$SPACE_SIM_PYTHON" ue-adapter/scripts/build_ue_project.py --engine "$UE56_ROOT"
"$SPACE_SIM_PYTHON" ue-adapter/scripts/build_ue_project.py --engine "$UE56_ROOT" --target BskUnrealRenderer
"$SPACE_SIM_PYTHON" ue-adapter/scripts/run_ue_automation.py --engine "$UE56_ROOT" --report /absolute/empty/automation-report
```

构建使用 UBT 增量依赖判定，并在成功后调用统一 manifest 生成器。记录 UBT 实际输出的工具链信息；没有输出则标记 `not_recorded`，不冒用系统默认编译器。默认禁用 UBA，以免共享内存锁与本机加速服务影响独立构建。Automation 默认 NullRHI；报告必须非空、全部成功，旧报告目录拒绝重用。Linux 使用 NoZen DDC 回退。`--gpu` 只改变 Automation 的 RHI，不等价于完整双相机数据验收。

原生 profile 见 `server/scripts/native-test-profile.json`，从 `server/` 执行：

```bash
PYTHONPATH=.:backend:scripts:../ue-adapter/Adapters "$SPACE_SIM_PYTHON" -m pytest -p ci_pytest --ci-native --junitxml=/absolute/native.xml
```

真实 GPU smoke 从 `server/` 执行，使用独立端口与合成场景：

```bash
SPACE_SIM_UE_CAPTURE_SMOKE=1 SPACE_SIM_UE_STARTUP_TIMEOUT=600 PYTHONPATH=.:backend:../ue-adapter/Adapters "$SPACE_SIM_PYTHON" -m pytest tests/test_capture_on_demand_renderer.py --junitxml=/absolute/gpu-smoke.xml
"$SPACE_SIM_PYTHON" ../ue-adapter/scripts/check_test_report.py /absolute/gpu-smoke.xml
```

烟雾测试、五任务完整录制、长稳测试及整批数据审核分别记录。发布独立包的 Cook/动态资产加载见 [发布重建](release-reproduction.md)。不能将 NullRHI、零用例或 skip 汇报为 GPU 通过。
