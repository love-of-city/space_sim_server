# 数据保留与来源

2026-10-30 整理保留了 layout1000-20261029。随后用户明确要求保留旧批次并使用时间随机种子新建 layout1000-20261030-1928；2026-10-31 两批均已停止。两批全部原件保留，包括成功、失败、取消、worker记录、配置/计划和审核状态。此前其他历史采集和检测输出已清理。

2026-10-31 后续按用户要求新建 `layout1000-20261031-0042`，原两批继续停止；新批次与源码/二进制来源见其 `launch-provenance.json`，实时状态以 `progress.json` 为准。

版本恢复源码保存在 experiments/recovery-20261030-optimized-runtime，验证输出在 artifacts/verification/optimized-runtime-20261031，报告在 reports/optimized-version-update-20261031。它们是当前版本的来源与验收证据；未合入实验源码不能再按普通检测输出删除。

random150-20261028布局池的模型、catalog、几何审核和哈希清单是功能依赖。旧重编号plan单独保存在池的provenance中，通过原路径兼容链接维持可读性。

后续采集使用 data/collections/<run-id>；固定训练版本用 datasets/<dataset-id>；导入原件用 imports，交付用 exports。目录不同不代表重复，去重需身份、哈希与来源验证。

运行记录包含代码/dirty指纹、任务版本、参数、种子、模型/布局哈希、环境/二进制指纹、输出与验证状态。历史缺失值标unknown，不补写当前HEAD。软链接索引不是备份。
