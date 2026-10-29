# 航空插头自动采集：卡扣对齐与卸载撤爪

工作分支：`zmh_linux`。实现说明和官方参考源码见
[航空插头采集](../space_sim_server/docs/AVIATION_PLUG_COLLECTION.md)。

## 最终行为

- 将航空插头抓取方向从安装座 X 方向改为插头实测局部 Y 轴，覆盖两侧卡扣，
  支持安装位置/朝向随机化以及物体漂移。
- 在上方移动时同时转腕、张爪；预抓取后减速接近，双指接触和方向正确才拔出。
- 沿初始插座轴反馈回中拔出，插针离座后才按目标孔键向搬运、精对齐和插入。
- 插到位后平滑卸掉伺服预压，恢复实测到位姿态；张爪时不追着插头向下压。
  卸载不受普通路径的 0.06 rad 跟踪等待门限阻塞；路径和张爪均完成后再侧撤。
- 按用户明确确认，插拔任务默认关闭 40 N / 4 N·m 连续超载自动中止及其阈值退让。
  `overload_stop_enabled=false` 写入配置和逐帧指标。20 N 静摩擦上限、20 N
  滑动阻力保持原样；未修改 MJCF、接触几何、关节执行器参数或 UE 资产。
- 更新批量初始化版本、同步张爪碰撞预检和分版本数据验收，兼容历史 v2 数据。

## 真实运行验收

完整证据目录：
`/home/lyh/space_enviroment/runs/aviation-latch-grasp-20261024/`

| 场景 | 证据 | 结果 | 最终到位偏差 |
| --- | --- | --- | --- |
| 原始布局，真实 UE 双相机 | `recording-07-final/` | success，197.32 s | 0.7522 mm |
| 独立位置和朝向随机布局，原生动力学 | `native-randomized-final-v2/` | success | 0.7707 mm |

两次运行均为原有 BSK / MuJoCo 权威动力学，`external_state_writes=0`，
最终手爪释放、插头保持在 `aviation_insert_40p5`。随机布局复用了已生成的
`task-box-yaw-layout-20261023/model-final/layout.xml`，不是重新改写孔位来通过测试。

原始布局在开始拔出时，夹爪与卡扣方向误差约 0.001352 rad（0.0775°）。
卸载撤爪后工具高度保持约 0.0326 m，末端实际侧撤 0.12 m，通过最终保持检查。

录制 episode：`episodes/episode-20261024-222949-870ee1b4/`。
两路相机各 9867 帧、1024×1024、50 Hz；双视频完整解码、时间戳/帧号配对、
7 维状态/动作和 200 Hz 腕力校验均通过，详见 `final-episode-validation.json`。

- `aviation-latch-aligned-dualcam-2x.mp4`：完整并排预览，2 倍速。
- `aviation-latch-aligned-stages.jpg`：真实关键阶段画面。
- `final-manifest.json`：结果、配置、事件和负载峰值。
- `final-source/` / `source-hashes.json`：最终源代码快照及哈希。

## 测试与适用范围

- 定向回归：107 passed，1 skipped。
- 网页测试：185 passed。
- Adapter Python：99 passed，5 skipped，19 subtests passed。
- 修改文件的 Ruff 和 `git diff --check` 通过。
- 原始布局静态碰撞预检：484 个姿态通过。它不替代动态验收。

本次还尝试过全量服务端 Python 测试：1461 passed、82 failed、188 skipped、
5 errors。该环境的测试集合存在 Python MuJoCo 与 Basilisk MuJoCo 同进程
混载，以及缺少 `numba` 等依赖的问题；日志保存在 `server-tests.log`。
未将全量 CI 标记为通过；原生任务和相应接触回归在独立进程中运行。

这次验证覆盖一个原始布局和一个随机布局，不代表任意布局都可成功。失败或
中断的诊断录制保留审计记录，没有标记为成功训练数据。

当前卡扣仍为刚性几何，未模拟独立按压解锁。关闭超载停止后，原始布局实测
腕力/力矩峰值为 125.20 N / 16.63 N·m，随机布局为 326.51 N / 50.34 N·m；
20 N 是插座等效摩擦参数，不是腕部总负载上限。这次成功验证的是几何抓取、
动力学任务流程和录制完整性，不是实物力标定或人类接触力分布。
