# 抽屉与锁杆单向操作采集

原来的六种任务及 `drawer_lift` 诊断入口保留。新增四种独立任务，使用原来的直线优先接近、同步转腕/开爪、原生几何检查、真实接触及双相机 LeRobot 录制链路。

| 任务名 | 初态 | 本条操作与终态 |
|---|---|---|
| `drawer_open` | 抽屉关闭并锁定 | 抓取、解锁、拉起约 8 cm、松爪撤离，保持打开 |
| `drawer_close` | 抽屉打开约 8 cm | 抓取、放回并锁定、松爪撤离，保持关闭 |
| `locking_rod_fold` | 锁杆竖起，紧固件锁紧 | 卡扣抓取并旋松、换抓放倒入卡扣、松爪撤离 |
| `locking_rod_restore` | 锁杆放倒入卡扣，紧固件松开 | 抓取扶起、换抓并旋紧、松爪撤离，保持竖起锁紧 |

初态在每条任务创建物理模型时设置。模型生成保留布局及其他部件，录制开始后由实际动力学和接触驱动，复位任务不需要先执行一遍放倒。每条任务都有自己的名称、阶段、版本、成功判据和20条英文指令；反向操作不会混入该条指令和成功历史。双相机仍为 1024×1024 RGB / 50 Hz，状态动作50 Hz，独立腕力200 Hz。

网页“自动采集”任务下拉框可分别选择四项；抽屉沿用抽屉移动速度设置，锁杆沿用快速采集/原始速度设置。CLI 的 `collection/run_scripted_task.py`（动力学诊断）、`collection/record_scripted_task.py`（单条双相机录制）、`collection/collect_scripted_tasks.py`（批采）与 `collection/collect_parallel.py`（隔离并行）均接受这些 `--task` 名称。

```bash
python collection/run_scripted_task.py --task drawer_open --output /absolute/new-drawer-open
python collection/run_scripted_task.py --task drawer_close --output /absolute/new-drawer-close
python collection/run_scripted_task.py --task locking_rod_fold --threaded-speed-profile fast_collection --output /absolute/new-rod-fold
python collection/run_scripted_task.py --task locking_rod_restore --threaded-speed-profile fast_collection --output /absolute/new-rod-restore
```

诊断命令不生成训练视频。正式录制仍须传入现有场景、UE、模型/catalog等实际运行参数，可参考现有采集文档。按布局采集时，在 `config/collection.json` 的 `tasks` 中启用所需项，再运行 `tools/collect.py`；新增项默认关闭，原五任务批次数量保持不变。

录后验收按方向核对阶段、实际起终位置、抓取/旋拧接触及松爪后的稳定状态。抽屉放回不要求本条先拉起；锁杆复位使用真实初始卡扣状态证明，不伪造“已经旋松/放倒”的历史。缺少必要阶段、错误初态、未完全松爪或反向任务录像均拒收。代码和回归测试通过不等同于任意随机布局均能成功，真实采集仍记录预检或任务失败并保存原因。
