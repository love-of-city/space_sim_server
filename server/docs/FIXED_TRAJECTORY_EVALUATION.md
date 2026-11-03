# 固定条件动作重复测试

这是抽屉与三类插头的原生动力学重复性基线。保持相同模型、初态和动作，回答“同样动作重复执行能有多大成功率”。它不测视觉策略、随机场景泛化或 UE 图像质量。代码仅在本地分支开发，运行数据保存到仓库外。

## 两个阶段

1. `canonical` 对每个任务运行一次现有脚本。保存实际施加的 200 Hz 八维关节/夹爪参考、对应任务台坐标下的 TCP 位置和姿态、初始关节、模型及依赖 SHA256、物理子步、插头保持配置和轨道配置。失败动作同样保留；不重试寻找成功示范。
2. `evaluate` 每次启动新原生进程，按时间回放同一份关节参考。既有伺服和载荷保护仍工作，回放不调用示范脚本的 `step()`、在线 IK 或物体位置纠偏。原动作结束后统一保持最后一个参考 2.5 秒，检查最终稳定状态。

这里冻结的是执行器参考，TCP 路径是同一模型正运动学计算出的参考路径；接触和跟踪误差可能使实际末端偏离参考。冻结文件、模型、初态、任务参数及已解析的物理配置不匹配会拒绝执行。旧的缺少物理配置的轨迹需要从原始 trace 重新导出。

## 本地命令

使用同时具备 Basilisk 和原生 MuJoCo 步进器构建依赖的 Python。不要在同一进程导入 Python MuJoCo。首次准备新工作区时：

```powershell
Set-Location D:\mingti\benchmark-asset-library\space_sim_server
$runtimePython = 'D:\mingti\latest-20260923\.venv\Scripts\python.exe'
& $runtimePython tools/assets/build_sarm_plugin.py
& $runtimePython tools/assets/build_native_acceleration.py --component local_mujoco_stepper
& $runtimePython tools/diagnostics/evaluate_fixed_trajectories.py canonical --task all --output D:\mingti\outputs\repeatability\canonical
& $runtimePython tools/diagnostics/evaluate_fixed_trajectories.py evaluate --task all --canonical-dir D:\mingti\outputs\repeatability\canonical --trials 10 --output D:\mingti\outputs\repeatability\evaluation
& $runtimePython tools/diagnostics/evaluate_fixed_trajectories.py rescore D:\mingti\outputs\repeatability\evaluation\drawer_cycle\trial-0000\native
```

输出目录必须不存在；使用新目录保留历次结果。可用 `--task drawer_cycle` 等单任务运行。`--wall-timeout` 控制每个进程的墙钟时间；超时算基础设施错误。此入口不启用场景随机化、UE 渲染或图像采集。

## 当前独立评分：抽屉 v2，插头 v1

抽屉的任务要求由 `DrawerBenchmarkCriteria` 固定并版本化，独立于采集脚本配置。策略观测闭环与固定动作回放可共用这个判据，但后者只是回归基线。完整 benchmark 仍需待测策略执行“观测→动作→仿真推进→新观测”，配置固定的场景、初态、预算和试次数；本入口不代替策略评测入口。

评分只读取物理观测，不读取脚本的 stage、result、open_verified 或 ever_lifted。200 Hz 观测间隔超过 5 ms（含 1 微秒数值容差）、时序倒退、非有限值或零时刻之后力读数无效，均记为不可评分。零时刻尚未产生的全零力读数单独允许；后续读数必须有效。

| 任务 | 按顺序检查的条件 |
| --- | --- |
| 抽屉 | 夹持时打开至少75 mm、速度小于3 mm/s，连续保持2 s；夹爪尚未张开且至少一指近接触时推回至初态2 mm内、速度小于3 mm/s；松爪且双指无接触；末端距实际把手至少90 mm，保持关闭、卡扣误差不超过0.025 rad、松爪和无接触状态连续2 s |
| 三类插头 | 双指接近接触时抬起至少 40 mm；到达指定装配孔并满足插入几何；松爪且双指无接触；末端水平撤离至少 90 mm，插头线速度小于 2 mm/s、角速度小于 0.03 rad/s，稳定 2 s |

插入几何条件：径向误差小于 0.4 mm，轴线误差小于 0.01 rad，深度大于 tip−flange−2 mm，法兰间隙绝对值小于 1 mm；航空/导向插头还要求完整姿态误差小于 0.01 rad。当前接口孔轴沿任务台 Z 轴，此版本评分器限定于这些现有接口。

关节参考速度也直接冻结实际施加值，不通过位置差分重建。失败脚本可能将参考设为实际关节状态，两指受约束柔顺性影响不完全反对称；回放保留原坐标，只校验各执行器范围。抽屉回推可以只有单指接触，返回判定不要求双指同时接触；原有双指接触判定导致的诊断失败仍保留在旧结果目录。

抽屉 v2 不要求固定夹爪朝向、精确抓取点或特定撤离路径。距离按模型把手基准加当前抽屉滑动量计算，任意方向撤离均可；专家的精确悬停误差、朝向及关节速度只作诊断，可缺省。初态须接近关闭（滑动量绝对值不超过5 mm）。旧校准模式仍保留，但不作为 benchmark 的动作约束。评分几何从实际模型读取，不从控制器目标或失败轨迹末点读取。

抽屉腕力/力矩阈值为 80 N / 8 Nm，连续超限 0.1 s 失败；插头为 40 N / 4 Nm，连续超限 0.15 s 失败。距离接触计数来自现有原生接触快照，只代表近接触，不证明夹持载荷。上述为当前仿真器工程阈值，尚需开发集校准；不是 LIBERO 或 RoboDojo 的规定阈值。

## 指标与证据

- 主指标：成功次数 / 全部预定尝试次数。初始化拒绝、基础设施错误和不可评分均保留在分母中，不补跑替换。
- 辅助指标：可评分成功率、五种结果计数、四阶段完成度、最终保持时间、末端位置跟踪 RMSE、峰值腕力/力矩、物体最大位移及最终装配误差。
- 输出 Wilson 95% 区间供描述；完全相同条件下的确定性重复不能被解读为独立随机场景样本，不能据此推断泛化可靠性。
- 每次保留 `result.json`、原生 `summary.json`、日志、`scorer_spec.json` 和 `evaluation_rows.jsonl`；在线与离线评分必须一致。`protocol.json` 保存代码哈希及执行环境，`sources/` 保存对应源码快照。汇总在 `summary.json` 和 `aggregate.json`。
- `evaluate` 返回 0 表示全部尝试均取得可评分结果，任务失败也可能返回 0；出现拒绝、不可评分或基础设施错误返回 2。成功率以 JSON 为准。

借鉴 [LIBERO 的固定初态、固定评估次数和任务成功率](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/lifelong/metric.py)，以及 [RoboDojo 的任务、配置校验与结果制品组织](https://github.com/RoboDojo-Benchmark/RoboDojo)。这里只借鉴评估组织方式，不声称与其分数直接可比。

## 2026-10-29 本地首轮记录（历史 v1 判据）

以下结果保持原样，只对应旧判据；不能当作当前抽屉 v2 成功率。按当前判据重评保留的三次抽屉物理观测，三次均可评分且任务失败：打开后最长连续满足条件约0.385 s，未达到2 s。这是旧观测重评，未新运行策略或原生控制器。

每类预定 3 次，同一原生模型和初态，无随机化；模型 SHA256 为 `b8e77f3a20e7ddf5b27cc00b4df49214d529ac03607b4ef25da5513bc579d84f`。

| 任务 | 完整成功 / 预定次数 | 结果 | 峰值腕力 / 力矩 |
| --- | --- | --- | --- |
| 抽屉完整循环 | 3/3 | 完整循环并稳定撤离 | 25.63 N / 4.84 Nm |
| 航空插头插入 | 0/3 | 载荷保护，抬起阶段未通过 | 38.25 N / 4.73 Nm |
| 导向插头插入 | 0/3 | 载荷保护，抬起阶段未通过 | 56.20 N / 6.70 Nm |
| 圆柱插头插入 | 0/3 | 已抬起，插入未通过，触发载荷保护 | 34.18 N / 7.63 Nm |

12 次均有可评分证据，初始化拒绝、基础设施错误和不可评分均为零。每类任务三次物理观测文件的 SHA256 完全相同。这表明同条件动作的结果可重复；小样本确定性重复不能证明随机场景可靠性。

原生记录位于 `D:\mingti\outputs\repeatability-20261029\final-{task}\`，冻结动作位于 `canonical-final\`。最终评分规则已独立重算全部 12 次，完整报告与原在线评分一致，索引为 `final-scoring-audit\summary.json`。原始评分配置、在线结果和源码均保留；最终审计采用 5 ms 连续证据检查和由任务几何确定的抽屉撤离点。

`sources-used\` 保留首轮原生执行对应的精确源码，修正前诊断记录保留在 `drawer-repeat\`、`repeat-v2\`。相关控制、评分和采集回归共 102 项通过；更大范围 basic CI 在收集阶段因本地环境缺少 `scipy` 阻塞，尚未通过。未执行本次 UE 图像录制或视觉策略评估。

最终代码另外完成一次抽屉原生复验，成功，在线/离线评分一致，结果在 `final-scorer-drawer-verification\`。这次单独复验不混入上述预定 3 次基线统计。

## 保留历史数据并重评

旧抽屉 `scorer_spec` 不会自动套用新标准。用原始测量和匹配的模型显式重评，输出至全新目录：

```powershell
& $runtimePython tools/diagnostics/evaluate_fixed_trajectories.py rescore-drawer-trace --trace D:\mingti\outputs\repeatability-20261029\final-drawer_cycle\drawer_cycle\trial-0000\native\evaluation_rows.jsonl --output D:\mingti\outputs\drawer-reassessment-new
```

入口校验原报告中的模型哈希，保存新判据、规范化测量、评分、源码和来源哈希，不读取专家成功标志作为判定依据。原始数据及历史报告保持原样。新结果可用 `rescore 新输出目录` 再次独立重算；缺失关键证据记为不可评分。
