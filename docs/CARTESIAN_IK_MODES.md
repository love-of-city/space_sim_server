# 末端笛卡尔 IK 模式（ik_pose / strict）

本文说明实时遥操作末端 IK 的两个可选内核、切换方式、可调参数和观测诊断字段。
实现位置：[simulation/serial_chain_kinematics.py](../simulation/serial_chain_kinematics.py)、
调度位置：[simulation/teleop_grasp_unreal.py](../simulation/teleop_grasp_unreal.py)。

## 背景

机械臂、坐标框架、`SafetyController`、deadman/超时语义、BSK `MJJointPIDController`
都没有改变：操作员仍然只发送六维末端速度，服务端把它转成关节位置参考，物理跟踪仍由
MJScene 500 Hz 的关节 PID 完成。本文只描述“六维速度 → 关节速度”这一步的内核选择。

## `ik_pose`（默认）

对应 robosuite `IK_POSE`（`robosuite/controllers/parts/arm/ik.py`、
`robosuite/utils/ik_utils.py`）的微分 IK：

```text
dq = J.T @ solve(J @ J.T + λ² I, twist)
dq += (I - pinv(J) @ J) @ (Kn * (q_posture - q))
dq *= scale   # 关节速度/位置限制的统一缩放
```

- **阻尼最小二乘（DLS）**：`λ` 从 `IK_POSE_BASE_DAMPING = 1e-3` 起，随着最小奇异值
  接近 `IK_POSE_SINGULAR_VALUE_THRESHOLD = 2e-2` 线性升到
  `IK_POSE_MAXIMUM_DAMPING = 5e-2`。均衡位形下残差约为指令模的 `5e-4`，接近奇异位形
  时速度有界、平滑减速而不是直接归零。
- **零空间姿态控制**：把姿态误差 `(q_posture - q)` 投影到雅可比零空间后叠加，参考位姿取
  该实例的初始关节配置（与 robosuite 使用 `initial_joint` 一致）。该分量只作用于雅可比
  无法控制的方向，不会把末端推离指令位姿；deadman 松开时不下发该分量，因此松手后关节
  参考严格保持不动。
- **统一缩放**：关节速度或位置约束触发时，仍然对整组关节速度乘同一个 `scale`，
  这一点与 `strict` 模式一致。

代价：DLS 在奇异位形给出的是“最小二乘意义下最接近指令”的运动，其方向可以与操作员指令
不一致（例如平移指令在退化方向上被替换成很小的腕部转动）。这正是 robosuite 的默认行为，
也是它相对 `strict` 模式“不冻结”的来源。观测中的 `cartesian_command_residual`、
`ik_minimum_singular_value`、`ik_damping` 用于判断当前处于哪种状态。

## `strict`（回退）

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
| `ik_nullspace_correction_norm` | 零空间姿态项的模（缩放后） |
| `jacobian_rank` / `cartesian_command_residual` | 原有字段，仍表示雅可比秩与六维指令残差 |

前端“仿真状态 → IK 求解”一行显示 `ik_mode`、`λ`、`scale` 和 `σmin`。

## 调参入口

| 参数 | 位置 | 默认值 |
| --- | --- | --- |
| `IK_POSE_BASE_DAMPING` | `simulation/teleop_grasp_unreal.py` | `1e-3` |
| `IK_POSE_MAXIMUM_DAMPING` | 同上 | `5e-2` |
| `IK_POSE_SINGULAR_VALUE_THRESHOLD` | 同上 | `2e-2` |
| `IK_POSE_NULLSPACE_GAIN` | 同上 | `0.15`（1/s） |

零空间增益越大，越早从奇异方向被拉回初始姿态，但会产生更多操作员未直接指令的关节运动；
在自由漂浮基座上这会改变基座反力矩，因此默认取较小值。
