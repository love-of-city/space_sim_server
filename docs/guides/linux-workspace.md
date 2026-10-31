# 本机维护入口

源码入口为 `/home/spacesim-zmh/src/space-sim`，服务端在 `server/`，UE适配器在 `ue-adapter/`。解释器为 `/home/spacesim-zmh/envs/spacesim/bin/python`，引擎为 `/home/spacesim-zmh/engines/UE_5.6`。

## 当前状态：正式采集停止

| 保留批次 | 停止时完成尝试 | 待完整审核 | 跳过 |
|---|---:|---:|---:|
| `data/collections/layout1000-20261029` | 1076 | 946 | 130 |
| `data/collections/layout1000-20261030-1928` | 130 | 111 | 19 |
| `data/collections/layout1000-20261031-0042` | 296 | 246 | 50 |

取消中的尝试另存；待审核不等于最终训练合格。第三批原计划1000布局轮次/5000尝试，原时间seed为1256867530。该批已通过 `cancel.request` 取消，`spacesim-layout1000-20261031-0042.service` 的 MainPID=0；取消返回码为1，因此systemd显示failed不代表仍在运行。

原始进度、计划、配置、视频、数据表、失败记录和取消标记保留。旧源码路径通过兼容软链接解析到新目录，历史批次不会被补写成新版认证。必要布局池仍为 `data/layout-pools/random150-20261028/pool.json`。

## 新版运行与恢复

本轮工程改进已同步到主目录，匹配的native与UE已重新构建。开始录制前可执行只读核验：

```bash
cd /home/spacesim-zmh/src/space-sim/server
/home/spacesim-zmh/envs/spacesim/bin/python tools/deployment/check_runtime_provenance.py \
  --adapter ../ue-adapter --unreal /home/spacesim-zmh/engines/UE_5.6
```

后续用户要求继续采集时，新实现建立新批次并冻结实际时间seed与来源；不要删除旧取消标记后直接用新版续写旧批次。新机制下同版本批次可通过 `--resume` 核验；运行身份或冻结输入改变时会拒绝。恢复历史实现需使用匹配的源码、环境与二进制。详见[来源契约](../architecture/runtime-provenance.md)。

## 验收与恢复依据

- 本轮实施与清理：`reports/ue-engineering-implementation-20261031/REPORT.md` 和 `cleanup-manifest.json`。
- 独立验收：`artifacts/verification/ue-engineering-20261031/`，不属于正式采集任务。
- 本轮快照：`archive/workspace-snapshots/workspace-before-ue-engineering-20261031/`。
- 初次目录整理快照：`archive/workspace-snapshots/2026-10-30-before-reorganization/`。
- 优化恢复依据：`experiments/recovery-20261030-optimized-runtime/`；此前验证报告为 `reports/optimized-version-update-20261031/REPORT.md`，其结果不自动授予后续代码认证。

源文件职责见[项目地图](../project-map.md)，新增命名见[命名规则](../naming-conventions.md)。正式数据、资产、环境、构建产物与恢复证据按[目录规范](../project-layout.md)分开管理，不在源码中堆放逐次检测输出。
