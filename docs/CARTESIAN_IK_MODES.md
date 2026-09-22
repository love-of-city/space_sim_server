> 当前默认已恢复 `ik_pose`；定制 QP/纠偏/制动机制已移除。保留的限位与独立速度诊断见 [简化遥操作](teleop_control.md)。

# 末端笛卡尔 IK 模式（ik_pose / strict）

本文说明实时遥操作末端 IK 的两个可选内核、切换方式、可调参数和观测诊断字段。
实现位置：[simulation/serial_chain_kinematics.py](../simulation/serial_chain_kinematics.py)、
调度位置：[simulation/teleop_grasp_unreal.py](../simulation/teleop_grasp_unreal.py)。

## 离线肘部抬高选解（新增）

新场景默认 `teleop-elbow-up-v1` 会在初始化前进行多初值位姿 IK、精度/限位/接触筛选，
以几何肘部高度为软偏好选取并保存关节解。实时 DLS 现在叠加有界在线几何拱形偏好（抬肘＋腕部向下＋J3负角），运行中不会跳解，
已有场景/显式初始角度保持原样。配置、回退和边界见 [肘部抬高优先](ELBOW_PREFERRED_IK.md)。

## 背景

机械臂、坐标框架、`SafetyController`、deadman/超时语义、BSK `MJJointPIDController`
都没有改变：操作员仍然只发送六维末端速度，服务端把它转成关节位置参考，物理跟踪仍由
MJScene 500 Hz 的关节 PID 完成。本文只描述“六维速度 → 关节速度”这一步的内核选择。

## `ik_pose`（默认）

对应 robosuite `IK_POSE`（`robosuite/controllers/parts/arm/ik.py`、
`robosuite/utils/ik_utils.py`）的微分 IK：

```text
dq = J.T @ solve(J @ J.T + λ² I, twist)
dq *= scale   # 关节速度/位置限制的统一缩放
```

- **阻尼最小二乘（DLS）**：`λ` 从 `IK_POSE_BASE_DAMPING = 1e-3` 起，随着最小奇异值
  接近 `IK_POSE_SINGULAR_VALUE_THRESHOLD = 2e-2` 线性升到
  `IK_POSE_MAXIMUM_DAMPING = 5e-2`。均衡位形下残差约为指令模的 `5e-4`，接近奇异位形
  时速度有界、平滑减速而不是直接归零。
- **在线几何构型偏好**：默认在已限幅 DLS 后加入当前肘高、`z4-z6` 高度差与归一化 J3 负角目标共同引导的有界修正，不传
  初始/home 参考。从零位也可主动偏向抬肘。额外速度和累计参考偏移有独立预算；
  `SPACE_SIM_ONLINE_ELBOW_MODE=off` 恢复 task-only，
  `SPACE_SIM_ONLINE_WRIST_MODE=off` 则只关闭腕部项，
  `SPACE_SIM_ONLINE_JOINT3_MODE=off` 只关闭 J3 负角项（默认软目标 −5°）。详见 [预算和限制](ELBOW_PREFERRED_IK.md)。
- **机械臂与夹爪分离**：同一 deadman 仍授权整个数据包，机械臂只由六维末端输入启用，
  夹爪只由开合输入启用。只动夹爪、无输入或超时均不执行机械臂 IK；两者同时输入时
  正常并行执行。转入夹爪操作时清除残留的机械臂平滑指令。
- **统一缩放**：关节速度或位置约束触发时，仍然对整组关节速度乘同一个 `scale`，
  此缩放作用于任务基线；新增抬肘修正在剩余限位/限速余量内缩放，不改变基线。

代价：DLS 在奇异位形给出的是“最小二乘意义下最接近指令”的运动，其方向可以与操作员指令
不一致（例如平移指令在退化方向上被替换成很小的腕部转动）。这正是 robosuite 的默认行为，
也是它相对 `strict` 模式“不冻结”的来源。观测中的 `cartesian_command_residual`、
`ik_minimum_singular_value`、`ik_damping` 用于判断当前处于哪种状态。

## `strict`（回退，不加在线肘部偏好）

原实现，保持方向不变的严格六维最小二乘：

- 先求解完整六维任务，再用一个比例因子缩放全部关节速度，因此末端运动方向与指令一致；
- 若指令落在雅可比列空间之外（投影误差超过容差），保持当前目标不动（冻结）而不是给出
  偏斜的替代运动。

适合需要“宁停不偏”的验证场景，或与历史数据做对比。

## 切换方式

```powershell
# 命令行
python simulation/teleop_grasp_unreal.py --ik-mode strict

# 平台启动脚本尚未转发该参数，可用环境变量（会传递给子进程）
$env:SPACE_SIM_IK_MODE = 'strict'
.\scripts\run_platform.ps1
```

默认值为 `ik_pose`。`--ik-mode` 只接受 `ik_pose`、`strict`。

## 观测诊断字段

| 字段 | 含义 |
| --- | --- |
| `ik_mode` | 当前内核（`ik_pose` / `strict`） |
| `ik_damping` | 本次求解使用的 `λ`（`strict` 恒为 `0`） |
| `ik_velocity_scale` | 关节速度/位置约束触发的统一缩放因子 |
| `ik_minimum_singular_value` | 当前构型雅可比最小奇异值 |
| `ik_condition_number` | 条件数（无界时饱和到 `1e9`，保证 JSON 有限） |
| `ik_nullspace_correction_norm` | 兼容字段，恒为 `0`；几何抬肘并非纯零空间修正 |
| `ik_elbow_preference` | 在线偏好状态、实测/参考高度、额外速度扰动及累计参考偏移 |
| `jacobian_rank` / `cartesian_command_residual` | 原有字段，仍表示雅可比秩与六维指令残差 |

前端“仿真状态 → IK 求解”一行显示 `ik_mode`、`λ`、`scale` 和 `σmin`。

## 调参入口

| 参数 | 位置 | 默认值 |
| --- | --- | --- |
| `IK_POSE_BASE_DAMPING` | `simulation/teleop_grasp_unreal.py` | `1e-3` |
| `IK_POSE_MAXIMUM_DAMPING` | 同上 | `5e-2` |
| `IK_POSE_SINGULAR_VALUE_THRESHOLD` | 同上 | `2e-2` |

`ik_solve_count` 只在实际执行 IK 时增加；空闲/夹爪独立操作时为保持状态，
`ik_solve_time_ms` 为零。此时参考链的雅可比秩/奇异值保留上一次采样值，不表示新求解。
实测运动遥测与 80% 监测仍更新，不会因跳过 IK 而停止采样。
