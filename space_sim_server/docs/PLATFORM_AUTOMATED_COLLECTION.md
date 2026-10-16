# 平台自动采集与数据集回放

2026-10-16。自动采集与数据查看共用现有 FastAPI 后端、用户会话和正式数据目录，不再需要另起 `data_display/server.py`。

## 使用

1. 按原方式启动平台并登录。在遥操作台顶部打开“自动采集与数据集”。
2. 在 `/collection` 设置成功条数、同时运行数（1–4）、最多尝试数、种子和关节初始化角偏移（±0–10°）。默认两路。
3. 点击开始，后台为各路启动独立的生产仿真/UE 采集进程。页面显示阶段、已记录帧数、每路尝试及验收结果；关闭网页不终止采集。
4. 点击成功记录或进入 `/datasets`，查看两路视频、六轴关节、夹爪连续开合、末端位置、抽屉位移及腕部六维力；支持时间轴、图表缩放和 CSV 导出。
5. “停止该批次”先请求录制正常收尾。超过 180 秒则强制清理该批次进程，已验收记录保留，未完成记录不计入成功数量。

本版开放抽屉完整循环：接近、抓取、抽出、放回、松爪、撤离和最终闭合检查。运动/观测/六维力噪声与光照随机化维持关闭。初始化使用项目默认操作姿态，加六轴独立均匀偏移；每次先通过模型关节限位和碰撞检查。

录制频率沿用 400 Hz 动力学、100 Hz 在线 IK、50 Hz 状态/动作/图像与独立 200 Hz 六维力，两个相机均为 1024×1024。数据仍使用 LeRobot v3、七维 state/action（六轴+夹爪0–1）、原始诊断和力数据 sidecar，不替换采集器、不重采样。

## 结构与扩展

- `backend/space_arm_platform/collection_jobs.py`：持久任务管理、并发预算、随机种子/端口隔离、进度与生命周期；本层不加载 MuJoCo/Basilisk。
- `collection_processes.py`：数据目录独占协调器锁、Windows Job Object 进程树所有权。后端异常退出时 Windows 回收采集子进程；重启将未完成任务标记为 interrupted，不擅自恢复。
- `collection_api.py`：鉴权 API、独立场景配置创建与数据/视频读取。创建自动采集场景不会覆盖遥操作管理器的当前实例。
- `dataset_view.py`：纯数据读取与时间轴构建，保留独立传感器时间轴，缓存签名包含力数据文件。
- `frontend/public/collection/`、`frontend/public/datasets/`：同源前端页面；原操作台增加入口。Vite 构建会复制 public 资源，源码静态挂载也可直接服务这两页。
- `tools/collect_drawer_cycles.py`、`record_drawer_task.py`：复用已验证的真实 BSK/MuJoCo → UE 相机 → CaptureReceiver/SimulationHub → EpisodeRecorder → LeRobot 链路；增加协作停止与运行路径透传。

新增任务在 `TASKS` 注册任务名、显示名称、场景模板和独立采集 CLI。适配 CLI 接受管理器传入的 count/max-attempts/seed/range-deg、场景/输出/数据目录、四个端口、UE/adapter 路径和 cancel-file；原子写入 summary.json，报告 attempts、accepted_episodes 和 status。各任务自行实现成功条件与数据校验，不能只依据进程退出码或录制时长判成功。后续可以扩展插头任务而不改动底层动力学和录制链路。

## 持久化、权限与限制

- 数据：配置的 `data_root`，默认 `data/episodes/`。自动与手动记录使用同一目录及查看接口。
- 批次：`data_root` 同级 `collection_jobs/<id>/`，含 job.json、场景快照、分路报告与日志。运行数据已加入 Git 忽略。
- 操作员仅可查看/停止本人批次，读取本人记录与视频。管理员可查看所有记录；没有用户归属的历史记录只向管理员展示。
- 每个数据目录只运行一个管理服务；同一时刻接受一个批次，批次内 1–4 路并行。此版本不是跨机器队列，也不自动断点续跑。
- 成功条数和尝试上限分配给各路；各路拥有独立种子区间和尝试预算。某一路耗尽预算会令批次失败，不会借用其他路未使用的预算；已经成功的数据保留。
- CPU/UE 并发沿用已验证实现，没有切换到实验 GPU 动力学。4 路长时间完整任务的吞吐与资源上限仍需专项测量。
- 可以保留遥操作实例，但它与自动实例共享本机算力。页面不承诺实时倍率。
- 只读取原数据根下的 mp4，视频响应支持 Range；API、数据和视频请求均复用登录鉴权。

## 接口

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | /api/collections/catalog | 任务注册表与采样配置 |
| POST / GET | /api/collections | 创建 / 列举本人可见批次 |
| GET | /api/collections/{id} | 进度、分路尝试与成功记录 |
| POST | /api/collections/{id}/stop | 协作停止 |
| GET | /api/datasets | 列举本人可见历史/新记录 |
| GET | /api/datasets/{id} | 原生时间轴与曲线数据 |
| GET | /api/datasets/{id}/media/{path} | 鉴权视频读取 |

## 验证

自动化测试覆盖计划边界、四路预算和种子/端口隔离、用户权限、协作取消、持久化恢复、独占管理锁、Windows 子进程树回收、两种采样时间轴、缓存刷新、视频 Range 及路径限制。真实采集与浏览器验收结果记录于统一仓库 archive/2026_10_16_平台自动采集与数据集回放.md。
