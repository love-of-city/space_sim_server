# 仿真扩展与验证范围

本轮选择性整合 `origin/zmh_linux` 的新增能力，以本地已整理工作区为基线。正式采集只有 `tools/collect.py` 与 `collection/` 一条运行链；专家仍在 `simulation/scripted_tasks/`，三插头共用本地 v19 和公共/类型配置。默认固定初态 `[90,-60,60,0,-90,0]°`、400 Hz 物理、200 Hz 控制/腕力、50 Hz 双相机不变。

## 原专家动作回放

`simulation/evaluation/` 冻结当前专家真实应用的 200 Hz、8 执行器位置及速度参考。回放使用同一个动力学入口，仅替换动作产生器，不调用专家步骤或在线 IK。冻结文件采用 `space-arm-fixed-trajectory/2`；不接受旧稀疏轨迹或将 50 Hz 数据插值后冒充原始动作。

从 `server/` 使用包含当前原生库及 Basilisk 的 Python：

```bash
python tools/diagnostics/evaluate_fixed_trajectories.py canonical \
  --task aviation_plug_insert --scene /absolute/attempt/scene.json \
  --model /absolute/attempt/prepared-model.xml --output /absolute/evaluation/canonical
python tools/diagnostics/evaluate_fixed_trajectories.py evaluate \
  --task aviation_plug_insert --canonical-dir /absolute/evaluation/canonical \
  --model /absolute/attempt/prepared-model.xml --trials 2 --output /absolute/evaluation/replay
```

输入必须是当前合法布局及完整初态；模型需要同目录的 `attitude_control.json`。冻结已解析的场景、任务配置、模型/资产依赖、关键代码和实际加载原生库指纹；这些已覆盖依赖改变时拒绝回放。代码清单尚非全部传递导入和 Python 环境的完整闭包。固定次数保留全部成功、任务失败、初始化拒绝和基础设施错误。输出不得覆盖旧目录。

独立评分的 2 秒稳定要求与末尾 2.5 秒观察只属于评估，不替换正式采集判据。它能发现“原专家判成功但释放后仍抖动”的情况；回放通过也不等于视觉策略闭环通过。`rescore <native-directory>` 从原始测量重新评分，并与在线结果核对。

## 场景、飞行和观测

| 能力 | 当前职责与边界 |
|---|---|
| 场景库 | `simulation/scene_library/` 管理目录、装配、坐标、依赖和验收证据。27 项设计目录不代表已实现 27 项。当前运行适配为本星四任务及远距交会；移动任务盒、跨星三插头、杆任务独立评分仍明确拒绝执行。 |
| 双臂 | `components/capture26.py` 处理所选捕获臂的手动参考；`preheld_capture.py` 提供显式预夹持组件。双臂模板单独选择，未成为采集默认。预夹持不等于物理对接或自主抓取验收。 |
| 飞行 | `orbital_rendezvous/`、`near_field/` 和后端 `mission_runtime.py` 管理远/近距执行、导航、交接、取消及任务归属；统一 `mission_api.py` 和现有控制台。完整远近距流程需要独立验收，短运行不能签发成功交接。 |
| 视觉 | `vision_api.py`、双目/腕部服务及独立 worker 消费图像和标定，显示定位/夹持对齐观测；不以仿真真值替代图像估计。需要在视觉 Python 安装 `.[vision]`，通过 `SPACE_SIM_VISION_PYTHON` 指定。 |
| 图像分发 | 原权威录制队列和 ACK/去重保持原逻辑；视觉使用独立有界队列，拥塞丢弃旧视觉帧并计数，不等待估计器。视觉 ACK 仅表示传输已接收，可能按拥塞策略丢弃。 |
| 渲染 | 同一 UE 工程增加视觉采样、相对视图、预览时钟和工作灯；工作灯默认关闭，启用后会影响渲染图像。权威图像使用原始帧和仿真时刻。 |

双臂资产可通过现有 `tools/assets/prepare_sarm_assets.py --template-id sarm-dual-capture26 --use-existing-model` 准备。模型清单保留来源和限用范围；新增外壳视觉 STL 只移除数学上面积为零的 33 个三角形，逐项索引和原始/结果 SHA 写入模型清单，碰撞网格不变。

本轮不引入旧 `scene_plug` / `scene_drawer` 专家、旧初始化路径、历史交会演示、自动抓取/真值编队实验或独立旧控制台。不修改本地 SDF 碰撞裁剪与重力性能实现。Git 保存历史，运行报告留在仓库外。

## 本机验证记录

2026-11-04：三类插头与合并前本地版本各用相同初态运行一次，共 26,307 个 200 Hz 样本；动作参考、关节状态、工具/插头位姿、接触记录逐项相同。航空、圆柱各两次固定回放通过独立评分；导向两次均因释放后角速度未满足 `<0.03 rad/s` 的持续稳定要求而失败。该失败保留，不能宣称三类均通过独立稳定性验收。

UE 自动化 45 项无失败；原有真实采集三项及视觉/录制并发采样测试通过，覆盖空闲/停止边界、帧身份、重置及图像方向。原生保持力与状态回归 52 项通过。完整批次吞吐、全部新场景的物理成功率及完整远近距连续任务不由这些检查证明。

基础服务端 2447 项、原生专项 49 项、最终网页 243 项及构建通过；后续视觉失效/回放/平台/Linux 部署 80 项通过。独立完整三插头批次 3 条录制全部通过完整审核，零跳过，未恢复正式批次。最后删除未接入的杆评分模块及实验页面文案，不改变已验收的采集执行路径。

双臂场景经当前 Linux 启动器实际加载，捕获臂手动参考响应及实测状态有限，主视角、腕部图像与双目传输通过。默认双目朝向未覆盖目标标记，图像近黑，定位返回拒绝；不能把这次传输验证当作双目定位成功。名义标定、完整视觉引导和完整远近距连续任务仍需独立验收。新增双臂 UE 资产由当前模型现场生成，生成目录不纳入 Git。
