# 正式 HTTP 7 km 至抓取：完整路线验收

2026-11-07 在 SpaceSim 云端独立冻结工作树完成一次从正式 HTTP 入口启动、远距至实际四爪抓持终点的连续运行。运行源码为 `ca7aa660a147af3cc64995caa9c343f90c63aa83`，任务编号为 `mission-9e652642da9a4f3283c6264a7e60a7c3`，种子为 7。全程没有接管、检查点重试、人工操纵、人为更改物理状态或运行中源码修改。

最终 HTTP 状态为 `completed`、`active=false`、`capture_completed=true`，独立力记录得到连续四爪保持 **20.0875 s**。原连续审计的九项中八项通过；照明审计因读取导航身份的字段层级错误而失败。修复读取器后，仅对同一归档的照明证据进行只读重评，九项联合检查全部通过。原失败报告及全部输入保留，不能将它改记为首次审计通过。

## 入口与运行边界

在已配置原生动力学、完整资产、渲染器及隔离端口的认证后端，提交：

```http
POST /api/mission/start
Content-Type: application/json

{
  "mode": "team-in-orbit",
  "scene": {"template_id": "sarm-dual-capture26"},
  "options": {"seed": 7, "capture26_profile": "near23_vbar_v1"},
  "live": true,
  "continuous": true
}
```

接口返回 202 后仅轮询 `GET /api/mission/state`。正式协调器完成远近距接续，任务编号保持一致。此次观察没有 HTTP 错误；到终态的墙钟时间为 19478.34 s。初始化、端口与资源要求沿用[正式任务说明](CAPTURE26_NEAR23_FORMAL.md)和[工作灯配置](NEAR23_RENDER_LIGHTING.md)，不能用普通 Python 单元测试环境替代。

| 物理记录 | 结果 |
|---|---|
| 起点质心距离 | 6995.421631 m |
| 远距阶段 | 0–5005.0 s，完成 110 m 处连续 5 s 驻留 |
| 近距阶段 | 5005.0–7740.1 s；首历元继承连续，全部近距驻留通过 |
| 终点质心距离 | 1.164283 m；该数值本身不作为抓持判据 |
| 抓持 | 独立 400 Hz 力记录确认四爪连续保持 20.0875 s |
| 实际视觉 | 10012 对双目采集回执、10010 个已消费结果、38 对保存的 RGB 图像 |
| 照明 | 全部采集回执及已存图像哈希通过；右相机固定工作灯符合声明 |

末尾两对曝光的固定 0.2 s 投递时刻晚于任务终点，未被控制消费。独立审计另核验同一连续任务、两个阶段报告、首历元连续性、真实 7 km 起点、全部驻留、四爪保持、停稳后才启用原相机采样，以及实际驻留调姿恢复。控制器阶段名和 HTTP 完成状态不单独授予物理成功。

## 照明审计修复与证据保全

`PoseNavigation.snapshot()` 的实际 schema 是 `space-sim-pose-navigation/1`，身份位于 `navigation.configuration.vision_identity`。旧读取器误读 `navigation.vision_identity`，旧测试夹具也使用了错误层级。因此原报告在检查图像之前返回 `missing_or_mismatched_navigation_vision_identity`。

评分源码 `771505d80f631843d7b1bd400c749467c09752e7` 改为读取真实快照层级，并拒绝错误 schema、仅含平铺字段或包含身份影子字段的输入；没有放松灯光、物理或安全阈值。测试改用真实 `PoseNavigation(...).snapshot()` 构造快照，覆盖错误任务、标定和结构，并逐项检查输入字节与修改时间保持不变。相关云端 **124 项检查及 Ruff 通过，零跳过**。

只读重评绑定原审计 SHA-256，保留其中八项已通过的物理/连续性检查，仅重新执行纯读取 `score_work_lights`。没有重跑动力学、渲染或会写入 `parking-audit.json` 的旧近距评分入口。共 **230 个原始文件、3618036696 字节**的 SHA-256 和修改时间在前后完全一致；运行源码仍固定为 `ca7aa660`，评分源码单独记录为 `771505d`。

下列路径均相对于云端 `C:/Source/yipuwang/SpaceRendezvous/worktrees/`；运行数据不进入源码 Git。

| 工件 | 路径或 SHA-256 |
|---|---|
| 远距完整归档 | `benchmark-formal-v1-20261106/space_sim_server/run/team_rendezvous/6504cb33805b45cb8842cf8fdd36467a/` |
| 近距完整归档 | `benchmark-formal-v1-20261106/space_sim_server/run/team_rendezvous/8796f3785afe45b4aae202bd6b7b829e/` |
| 原 HTTP 验收目录 | `benchmark-formal-v1-20261106/.tmp/benchmark-formal-http-v3/` |
| 原 `acceptance.json`，失败保留 | `947bf1a9664a427bc8a6335a79b1cdf9723c143f026e831d84bf1d30e34a7353` |
| 原 `continuous-audit.json` | `bff69e802a10e20be9cc47a69bb4c36856fc78d0106d0d9899cbfe794ced5fb9` |
| 原 `final-state.json` | `b1f13659693fb6a245ee0658f6fa8b28f560fd5a45948f2b6211dbda88f3a606` |
| 修正重评目录 | `benchmark-ci-v1-20261107/.tmp/benchmark-formal-lighting-identity-rescore-v1/` |
| 修正 `acceptance.json` | `d2e5019c52ffcdf4f4df87b49877261b72d595d5b5a5ac9de23b258e4708073d` |
| 修正 `corrected-audit.json` | `1eab8c56d17bcd196abbbaacaaccb9449ab10f29af7325d1d4db7c99a38bb4c1` |
| `preserved-inputs.json` | `c47f8615ad03e3abdc8588f08d1e676a65cf58d6965f4b1963dcad3a81e33106` |
| 124 项检查报告 | `benchmark-ci-v1-20261107/.tmp/benchmark-lighting-identity-checks-v1/acceptance.json`；`10063108ed91536be60e408ce2ba797f4f32d89f1f8ee20d9ceb97daf5421cf2` |

只读重评入口为评分源码中的 `space_arm_platform.render_lighting_evidence.score_work_lights(near_directory, handoff, vision_config)`。输入分别来自近距归档、其 `handoff.json` 和冻结运行树的 `config/vision/capture26_near23_front_mount_20261029.json`；返回结构写入新的输出目录。任何旧审计结论的复用必须同时核对上述原审计哈希，不能将本次修正结论套用于其他回合。

## 能证明的范围

这是一个种子的 **合作目标 C 工程路线**：目标提供合作导航/陀螺信息并主动调姿，视觉使用已知 23 标志物。实际双目在近距规定阶段开启，不能宣称从 7 km 起完全依靠视觉，更不能代表无标志物非合作抓取。

该运行使用旧正式路线的力归档格式；迟于 300 s 的力记录仍是原派生证据，没有追溯转换成新版[逐接触原始证据](CAPTURE_CONTACT_EVIDENCE.md)。它完成了最终 Goal 中正式 HTTP 不间断全程的工程验证，但不替代新 D54/D55/D62 的统一方法接口、全回合独立评分、原生复位与冻结矩阵，也不授予共享网页部署或多初态成功结论。Benchmark 覆盖仍为 **40 配置 / 37 条已注册来源 / 32 条完整验收 / 85 条未注册**。

此前 v1 基础设施错误、v2 地影失锁和 v3 原照明审计错误均保留，各自具有独立版本和结论。
