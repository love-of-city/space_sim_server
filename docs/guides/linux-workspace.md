# 本机维护入口

- 源码：/home/spacesim-zmh/src/space-sim
- 服务端：/home/spacesim-zmh/src/space-sim/server
- 解释器：/home/spacesim-zmh/envs/spacesim/bin/python
- UE：/home/spacesim-zmh/engines/UE_5.6
- 保留旧批次：/home/spacesim-zmh/data/collections/layout1000-20261029
- 保留新批次：/home/spacesim-zmh/data/collections/layout1000-20261030-1928
- 必要布局池：/home/spacesim-zmh/data/layout-pools/random150-20261028/pool.json

2026-10-30按用户要求通过cancel.request取消采集，systemd服务已停止。取消时1076次完成尝试、946条pending_audit、130次skipped，另有取消中的任务。cancel.request保留，不能因整理删除它或恢复采集。

原始进度、计划、配置、视频、数据表和失败记录保持原位置和原字节。旧源码路径用兼容软链接解析到新目录，不改写原始数据。布局池、训练权重与引擎保留。

后续只有用户要求继续采集时，才按CLI恢复流程处理取消标记、沿用冻结计划；不重新生成seed，不隐式重试历史中断任务。946条数据没有完成批末审核。

整理前快照和补丁位于 /home/spacesim-zmh/archive/workspace-snapshots/2026-10-30-before-reorganization/。本轮清理与验证记录位于 /home/spacesim-zmh/reports/space-sim-audit-2026-10-30/。

2026-10-31 更新：新批次亦按用户要求停止，130次完成尝试、111条pending_audit、19次skipped。正式服务 MainPID=0；独立新版本验收不恢复任一批次。优化集成报告见 /home/spacesim-zmh/reports/optimized-version-update-20261031/REPORT.md。
