# 在训练场景中运行外部策略

`runtime_worker.py` 启动独立的 SpaceSim 后端，供 Enactive 原生 SARM client 控制并录制。每个 worker 使用独立端口、认证数据库、任务目录和录制目录，UE 引擎、Content、模型和已生成的资产 catalog 共享读取。

```bash
cd server
XDG_CACHE_HOME=/home/glx/.cache/spacesim \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
/home/glx/envs/spacesim/bin/python tools/policy/runtime_worker.py \
  --root /home/glx/data/spacesim/policy-runs/example/worker0 \
  --adapter /home/glx/projects/SpaceSim/ue-adapter \
  --unreal /home/assets/engines/UE_5.6 \
  --episodes-root /home/glx/data/spacesim/episodes \
  --port-base 30000
```

端口依次为 API、控制、采集、渲染、预览；其他 worker 使用不同端口段。`credentials.json` 在 worker 根目录以 0600 权限生成，供本机 client 登录。缓存环境变量应指向已有 SPICE 文件的缓存根。

登录后，`POST /api/scenes/start` 可传：

```json
{"template_id":"sarm-task-box-20261012","source_episode_id":"episode-20261104-195706-660040a9"}
```

源 episode 必须是完整的 SARM 漂浮物录制，并保留对应 collection attempt 的 `recording/dynamics/model.xml`、`summary.json` 和 `attitude_control.json`。服务验证录制身份与物理/渲染模型 SHA256，复制模型到本次运行目录，恢复保存的物体类型、颜色、位置、姿态、速度和机械臂初态。它不会重新采样，也不启动原专家抓取控制器。物体自由体初态在每次 reset 中重新应用，插槽保持约束不会重新固定这些物体。

当前此入口接入关闭动态扰动的源场景；启用 action noise、外力或观测噪声等动态扰动的源会明确报错。MJCF 中已有的质量、摩擦以及场景光照仍保持原值。reset 到策略接管存在准备相机和录制的时间；首次策略观测的实际仿真时间应从 client 执行记录读取。

策略端使用项目已有 `/api/policy/sessions` 控制协议，执行六个绝对关节目标和归一化夹爪目标，权威双相机录制 50 Hz LeRobot v3。运行入口位于 Enactive 的 `tools/eval/rollout_sarm_recorded.py`。模型的 action horizon 与客户端执行 horizon 分开配置。

`POST /api/policy/sessions/{session_id}/step` 同时支持 `action_space="eef_pose_absolute"`。
每行仍为 7 维：`[x, y, z, rx, ry, rz, gripper]`，位置单位为米，旋转向量单位为弧度，
坐标系为 `cubesat_bus`，工具点为 `sarm_ee`，夹爪范围为 `[0, 1]`。
请求需携带完整 `kinematics`，服务端核对实际运动链后，以每个 20 ms 时刻的实际关节状态进行有界 IK，
再经过原关节限速与伺服链。回执分别保存请求位姿、IK 关节目标及实际应用关节目标；
LeRobot 的 `action` 仍保存实际应用的 7 维关节目标。
相对关节、chunk 锚点末端增量或 Cartesian OSC 的恢复由策略客户端按 checkpoint 合同完成。

`dataset_status=complete` 表示录制完成。当前浮动物体 policy 运行没有成功率判定，outcome 为 unknown；完整运行还应核对原生 client 的 action_steps 与请求步数相同。录制结果发布到共享浏览目录时使用同一文件系统的硬链接，避免外部目录符号链接被 dataset 文件访问检查拒绝。


三相机单臂源episode沿用相同回放入口。策略会话按场景 `collection_camera_profile` 冻结相机集合：
`single-arm-two-view-v1`为全局＋手眼，`single-arm-three-view-v1`再加入 `teleop/camera/workbench_side`。
RGB/state缓存等待全部相机的session、frame、仿真时间一致；录制与不录制的策略会话均采用同一集合。
返回 `images` 的顺序为 `base/left_wrist/right_wrist`，第三槽位实际是固定侧视，图像各自携带真实 `camera_id`。
相机集合在会话内保持固定，reset清空旧帧；新会话根据当前场景重新选择。双臂14D场景不会进入单臂7D策略接口。
Enactive配套三视角checkpoint按物理camera ID和完整视角集合校验输入。
