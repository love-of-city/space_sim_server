# On-demand authoritative capture / 按需严格采集

## 问题与范围

场景配置采集能力后，即使没有开始 episode，旧实现仍持续执行权威 RGB 回读、压缩、发送，并将双相机预览降频。可靠 FIFO 积压后可反向阻塞仿真。此次修改只区分预览与正式采集，不调整动力学、积分器或控制律，不改 RGB 编码格式和正式采样率。

此 PR 与 `love-of-city/space_sim_server` 的 `feature/on-demand-capture-20260924` 配套。服务端负责 episode 生命周期、首帧确认及末帧收尾；本仓库负责帧标记与 UE 消费。建议先合并兼容旧消息的 adapter PR，再合并服务端 PR，最后整体更新部署并重启后端与场景。

## 实现

- Bridge 每个状态帧仅采样一次 capture state，给帧和对应 observation 提供一致的 `capture_episode_id` / `capture_request_id`。钩子不能覆盖权威姿态、帧号或时间戳。
- Python publisher 保留独立的有界严格 FIFO 和 latest-only 预览槽。STOP 后仍先排空此前已接纳的严格帧，预览替换不能删除采集样本。
- UE 帧解析器识别可选 episode 标记。显式空值意味着预览，不执行权威数据产品采集；非空意味着可靠处理，并传递到 RGB 元数据。旧消息不带标记时维持原有行为。
- UE 接收器在严格图像输出有容量时才消费严格帧；空闲预览无需等待采集输出容量。旧的预览不会在严格队列排空后被倒序显示。
- 相机预览降频仅在处理有效严格采集帧时生效，返回预览后恢复配置值。
- 修正文档中“单次回读阻塞”“JPEG 与无损 RGB 像素完全一致”的不准确表述。回读仍同步，JPEG 仍有损。

## 验证证据

2026-09-23 的功能验收：

- Windows UE 5.6 完整编译、链接插件成功；不是 `-NoLink` 检查。
- UE `BskUnreal` 自动化 27 项通过，包含 `Capture.OnDemandBoundaries`，验证预览替换、START/STOP 顺序、停止后严格 FIFO 保留和协议解析。
- adapter Python 相关测试 29 项通过，另有 16 个子测试通过。
- 配套服务端独立端口、合成场景真实 UE/GPU 双相机测试通过：未开始时 0 张权威图像；首次 6 个采样得到 12 张；停止后无新增；第二次 3 个采样得到 6 张；再次停止后无新增。校验 episode、帧号及 JPEG 解码。不是生产场景延迟或吞吐基准。

2026-09-24 合入最新主线规范后会重新运行基础 CI 和上述 Python 回归。基础 CI 不安装 Basilisk，不运行 UE/GPU；新增两个 Bridge 测试依赖真实 SysModel，已按节点明确列入 `scripts/ci-exclusions.json`，其余新增队列/socket 测试仍必须执行。远程 Actions 是最终提交的 CI 证据，不能用跳过测试宣称完整仿真验收。

## 兼容与部署风险

- 旧独立 demo 保持兼容；新平台需后端、仿真 adapter、UE 插件同步部署。
- 正式采集仍保留有界反压，5 Hz 预览限频也暂时保留；此 PR 不保证解决正式采集期间的全部吞吐瓶颈。
- 停止采集并非清空队列，可能需要等待末尾已接纳帧完成。
- 插件需完整链接后重启 UE，后端协议修改需要重启后端。不得用旧 DLL 验证新源码。
- 不提交日志、录制、账号库、构建产物或本机路径；之前存在的 PBR0110 材质资产改动不属于此 PR。


### 2026-09-24 提交前复验

- `ruff check .` 通过（仓库固定版本 0.15.7）。
- `scripts/check_versions.py` 通过，发布版本均为 0.2.0。
- `scripts/verify_ci_guards.py` 通过：断言失败、skip、collection skip、xfail、版本不一致均返回预期失败状态。
- 本地 `--ci-basic` 70 通过、2 个明确记录的 Basilisk 节点排除；无意外 skip。工作区仍保留独立材质改动，最终 PR 实际测试数量以 Actions 为准。
- 本次相关 native Python 回归再次通过：29 项与 16 个子测试。
- 当前存在用户运行中的 UE，本次提交操作不停止、重启或重新编译其正在使用的插件。UE/GPU 证据为上述 2026-09-23 的验收，不能冒充本次生产场景实测。
