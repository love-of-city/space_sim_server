# 构型偏好：在线抬肘 + 腕部向下 + joint3 负角，可选离线初始化

## 在线偏好（默认开启）

**不再仅依赖初始构型。** `ik_pose` 在每次有效机械臂输入时，用当前参考关节角计算
肩/肘中心的相对高度、关节4到关节6的向下高度差、joint3 的负角余量及各自梯度，
先求原有限位/限速 DLS，再加入**一次共同求出的有界构型修正**。
初始角度可全部为 0，也可手动指定/来自旧场景；不会提前替换你指定的角度。
初始状态仍影响局部可达方向，但现在每一步都有显式的几何构型偏好。

实现：`simulation/online_elbow_ik.py`，接口 `apply_online_elbow_preference()`。

```text
min_c ||W J c||² + regularization² ||c||²
      + sum_i weight_i (gradient_i · c - rate_deficit_i)²

目标1：h_elbow = dot(up_axis, p_joint3 - p_joint2)，偏好至少 0.30 m
目标2：h_wrist = dot(up_axis, p_joint4 - p_joint6)，偏好至少 0.05 m
目标3：q3 <= -5°（默认软偏好，不改变机械限位）
       在联合方程中用 h_angle = -s*q3、target_angle = s*5°，s = 0.20 m / 1 rad
```

`c` 是加到原有已限幅关节速度上的小修正，不是全局 IK 关节角。此二次问题通过
一个小型线性方程同时求三个目标的修正方向，无 SciPy/OSQP 依赖。奇异附近自动偏好低任务代价方向；
满秩时也可在明确的额外末端偏差预算内改善构型，而非单纯开启无效的六轴零空间项。
不会引用初始姿态/home，不会在线突然跳到另一组 IK 分支。

### 默认预算（`OnlineElbowPreference`）

- 偏好肘高 0.30 m，本体系 +Z、相对肩关节；腕部偏好 `z4-z6 >= 0.05 m`。
  都是软目标，不是硬限位，也不是严格数学凸性约束。
- `joint3_enabled=True`，默认负角余量 `joint3_negative_margin_rad=5°`。
  在 q3=0 时仍有负向偏好；q3<=-5° 后这一项不再主动推动。使用实际关节坐标，不 wrap 或 clip 到负区。
  `joint3_gain_s=1`、`maximum_joint3_preference_speed_rad_s=0.05`（期望负向调整速率上限，
  不是 J3 总速度硬限制）、`joint3_weight=4`。
- 角度项先按 `joint3_angle_scale_rad=1` 归一化，再乘 `joint3_length_scale_m=0.20`，
  转成长度等效误差/梯度，参与同一二次目标；没有把原始 rad² 和 m² 直接以同数值权重相加。
- `wrist_enabled=True`、`wrist_height_gain_s=1`、`maximum_wrist_drop_speed_m_s=0.02`、
  `wrist_weight=4`（与抬肘权重相同）。满足相应目标后该项不再主动施加驱动；
  不会为了美观无限压低腕部。`joint6` 自转不移动自身关节中心，梯度由真实几何计算。
- 腕部 5 cm 是可调软目标，不是模型真实硬限位。改变本体系 up_axis 会同时改变两个目标的“上/下”。
- 期望抬肘速度最多 0.04 m/s，修正每轴最多 0.15 rad/s，且随指令强度趋零。
- **额外**平移速度扰动最多 0.002 m/s，额外转动最多 0.01 rad/s；还受等效指令幅值
  10% 的更紧预算限制。等效速度 `sqrt(||v||² + (0.54||ω||)²)`。
- 检查雅可比预测和有限步长 FK 两种额外偏差；检查下一步**整体构型代价**优于无偏好基线：
  `C = 4*max(0,0.30-h_elbow)^2 + 4*max(0,0.05-h_wrist)^2
       + 4*(0.20/1.0 * max(0,q3+5°))^2`（代码角度单位为 rad）。
  关闭腕部或 J3 偏好时不计对应项。该验收代价包括所有启用目标，即使某项当前已达标。
  有冲突时允许一项让步、另一项改善更多，不要求两种高度和 J3 角度每步同时改善。
- 保持原关节角度/速度上限，不为了抬肘再次整体缩小原有任务速度。
- 持续保存 `ElbowDeviationState`，让**偏好额外贡献的参考偏移**保持在 2 mm / 1° 内。
  锚点每次沿“当前构型下、不加抬肘修正的有限步长末端增量”推进；不是另一条完整
  反事实轨迹，也不是实际物理臂的测量误差界。原始 DLS 奇异误差、PID 误差仍单独存在。
  三项共用这**一份**预算，不是先抬肘、压腕、再用第三份预算压 J3。达到预算时缩小/取消偏好，不持续漂移。松键/超时不重置此预算，场景重置才清零。
- 松键、超时、无输入、仅夹爪输入仍跳过机械臂 IK，不自行抬肘。
- `strict` 完全保留为无在线偏好的对照内核。`ik_pose` 可用下面的开关恢复原行为：

```powershell
$env:SPACE_SIM_ONLINE_ELBOW_MODE = 'prefer' # 默认，重启场景生效
# $env:SPACE_SIM_ONLINE_ELBOW_MODE = 'off'  # 关闭整个在线构型偏好，恢复 task-only DLS
$env:SPACE_SIM_ONLINE_WRIST_MODE = 'prefer' # 默认：抬肘＋腕部向下
# $env:SPACE_SIM_ONLINE_WRIST_MODE = 'off'  # 只关闭腕部向下
$env:SPACE_SIM_ONLINE_JOINT3_MODE = 'prefer' # 默认：额外偏好 J3 负角
# $env:SPACE_SIM_ONLINE_JOINT3_MODE = 'off' # 恢复前一版抬肘＋腕部向下
```

该开关独立于下文的离线初始化开关。旧场景关节初态仍按存档恢复，但在默认 `prefer`
下的新运动会采用在线偏好；如需复现旧控制算法，需显式设 `off`。
**现在只需重启场景，不必换一个肘上初态或新建离线配置，才能获得在线偏好。**

为兼容已有记录，观测字段名仍是 `ik_elbow_preference`。它包含 enabled/status、
参考/实测肘高、修正量、线/角扰动预算和累计参考偏移，并新增：

- `wrist_enabled` / `elbow_objective_active` / `wrist_objective_active`；
- `reference_wrist_drop_m` / `measured_wrist_drop_m`（正值代表 J6 低于 J4）；
- `preferred_wrist_drop_m` / `predicted_wrist_drop_m`；
- `base_posture_cost` / `predicted_posture_cost`；
- `joint3_enabled` / `joint3_objective_active`；
- `reference_joint3_rad` / `measured_joint3_rad` / `predicted_joint3_rad`；
- `preferred_joint3_upper_rad`（软目标，不是机械上限）、角度与长度归一化尺度。

所有启用的目标都达标时为 `shape_satisfied`；同时关闭腕部和 J3，仅保留抬肘时保留旧 `height_satisfied`。
前端 IK 行显示偏好状态、实测肘高、实测 `J4−J6` 高度差以及启用负角偏好时的实测 J3 度数。
这些字段不把预测高度冒充实测高度，也不把有任务泄漏的修正标成纯零空间运动。

在线偏好是局部、有预算的改善，不保证每步绝对上升、J6 必然低于 J4，或任意目标都肘上：当用户任务要求
下降时，可能只是比无偏好解下降更少；局部驻点、限位、预算耗尽时可取消修正。
J3 从正角起步仍合法：模型的约 ±180° 机械限位不变，旧场景和手动初态不被拒绝或篡改。
紧的末端预算可能阻止 J3 变成负角；负角偏好可能只是让它比无该项时更小，不能承诺跨分支。
三项可能互相竞争，共享预算后抬肘/压腕速度会改变，不以此为由放宽跟踪预算。
**没有新增实时路径碰撞检查**，仍依靠原碰撞模型/动力学响应，不能声称碰撞安全规划。

## 离线初始化生效范围

离线初始化仍使用前一版仅肘高的选解规则，没有增加腕部或 J3 硬性过滤；本轮修改的是在线构型偏好。
2026-09-21 起新建场景默认全零并等待操作姿态准备，见 [操作姿态准备](ARM_PREPARATION.md)。
本节是显式选择 `teleop-elbow-up-v1`（“肘部抬高优先 v1”）时保留的离线初始化行为。
先以原均衡姿态附近采样得到的末端位姿为目标，离线多初值求解，再保存选中的
六轴角度。夹爪、目标随机化、轨道随机流不变。**并不是在运行中的每一步强制抬肘。**

- `teleop-balanced-v1`、`training-v1`、`none` 的初始采样行为不变。
- 用户显式填写初始 J1–J6 时不重选构型。
- 已保存的场景按保存的八自由度状态启动/重置，绝不重新求解。
- 不传场景文件的直接仿真启动现改为六轴全零并等待准备按钮，不再离线替换初态。
- deadman、超时、限速、限位、夹爪独立使能不变；在线 `ik_pose` 偏好见上文。
  没有重新启用零空间回 home，也没有在线全局跳解。

已有场景或已经加载的服务不会热更新：重启平台后**新建**场景并选用上述新配置。
单纯重置旧场景仍会恢复旧关节角，这是刻意保留的复现语义。

## 几何定义与选解

`simulation/serial_chain_kinematics.py::joint_origins()` 返回本体系关节中心。
SARM 的肩/肘采用 `joint2`/`joint3` 中心：

```text
height = dot(unit(up_axis), elbow_position - shoulder_position)
```

默认 `up_axis=(0,0,1)`，在 `cubesat_bus` 本体系定义，而非世界 Z。
不使用 UR5e 的关节符号经验规则；SARM 的抬高分支反而可能是负的第三关节角。

`simulation/pose_ik.py::select_elbow_preferred_pose()`：

1. 默认 19 组初值：当前姿态、6 组肩肘扰动、12 组固定随机流初值。
2. 阻尼迭代、限幅和回溯线搜索；每个初值最多 120 次迭代。
3. 转到离当前姿态最近的**合法** 2π 分支，不能任意 wrap 后越过关节限位。
4. 位置误差不超过 **0.1 mm**，姿态误差不超过 **0.001 rad**，且在模型关节限位内，
   才有资格参与评分。约 π 的旋转误差使用四元数对数，避免角轴除以 sin(π)。
5. 校验端点静态接触、选解用 FK 与 MuJoCo FK 的一致性。
6. 按高度软偏好、实际关节变化、限位裕度和奇异值评分。

`ElbowPreference` 中可调参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `preferred_height_m` | 0.30 | 相对肩部的偏好高度，不是最低硬限位 |
| `length_scale_m` | 0.54 | 高度误差及位姿迭代的尺度 |
| `elbow_weight` | 8.0 | 低于偏好高度的平方惩罚权重 |
| `motion_weight` | 0.05 | 与原始关节角的距离惩罚 |
| `limit_weight` | 0.02 | 距限位不足行程 10% 时的惩罚 |
| `singularity_weight` | 0.05 | 最小奇异值不足 0.02 时的惩罚 |

达到偏好高度后不再额外奖励更高；较低构型仍可成为最优可行解。
这些权重不是碰撞/精度的替代品，也不保证穷举了所有分支。

## 碰撞检查与隔离

`simulation/prepare_elbow_posture.py` 在**独立进程**中使用 MuJoCo 3.7.0，
与当前项目验证过的原生引擎版本相配。不在 Basilisk 物理进程内导入第二套 MuJoCo DLL。
使用所选 MJCF、随机目标位姿、目标铰链和夹爪状态，拒绝涉及机械臂的非正距离接触。
遵循模型原有碰撞体、mask、父子排除等语义；不是原始 CAD 精确碰撞证明。
不将与机械臂无关的目标内部接触误判成机械臂碰撞。

实验 flex 碰撞模型暂不支持这项检查，会保留原始姿态并报告 fallback；不假装已验收。
没有候选、模型缺失、依赖缺失、校验错误或 45 秒超时时，同样保留原始采样姿态。
**回退姿态不因此被标记为安全。**

离线位姿选解只用于初始放置，仿真还没有开始，所以不会执行从旧姿态到新姿态的运动。
如果未来调用位姿 IK API 为运行中的机械臂选目标，必须另行做路径规划/路径碰撞检查，
不可直接把返回的关节角写入控制参考。在线小步偏好与此 API 分开；仍未实现全局换分支路径规划。

## 配置与诊断

以下离线开关只影响初始化选解，不改变已保存场景，也不关闭在线偏好：

```powershell
$env:SPACE_SIM_ELBOW_MODE = 'prefer' # 默认
# $env:SPACE_SIM_ELBOW_MODE = 'off'  # 保留新配置的原始均衡采样，不做选解
```

离线 worker 的 Python 选择顺序：

1. `SPACE_SIM_POSTURE_PYTHON`；
2. 项目本地 `run/ik_posture_worker.json` 中的 `python` 路径；
3. 当前解释器。

本机已配置已有的独立 mesh-collision 虚拟环境，未修改共享 Python 安装。
其他机器可在专用虚拟环境安装 `numpy` 和 `mujoco==3.7.0`，然后设置此路径；
`pyproject.toml` 的 `posture` 可选依赖记录了版本。不要为了本功能在实时进程直接导入 Python MuJoCo。

选解报告保存在场景 JSON 的 `ik_initialization`，并在仿真启动时打印：

- `status`: `selected` / `kept` / `fallback` / `skipped`；
- `original_joint_position_rad` / `joint_position_rad`；
- 原始/选中肘部高度、位置/姿态误差、最小奇异值；
- 候选数量、接触拒绝数量、MuJoCo 版本；
- `reason`：回退或跳过原因。

测试分别运行，避免在同一进程加载两套 MuJoCo：

```powershell
# 项目/Basilisk 环境：数值算法、场景保存、实时控制回归
& $env:SPACE_SIM_PYTHON -m pytest tests/test_elbow_pose_ik.py tests/test_ik_initialization.py tests/test_serial_chain_kinematics.py tests/test_simplified_teleop.py tests/test_teleop_channels.py -q
# 专用 Python MuJoCo 环境：包含真实所选模型的静态接触测试
& $env:SPACE_SIM_POSTURE_PYTHON -m pytest tests/test_elbow_pose_ik.py -q
```


## 历史：仅抬肘在线版本验证记录（2026-09-20，腕部偏好加入前）

实际 SARM 几何、100 Hz 参考积分、固定输入本体系 +X 0.02 m/s、初始六轴全零，运行 5 s：
关闭在线偏好时肘高约 0；开启时参考肘高约 **0.1583 m**。偏好额外参考偏移约
**0.8775 mm / 0.0992°**。这是控制参考验证，不是物理臂实测或任意输入保证。
同输入从旧 balanced 姿态起步时，额外 2 mm 预算耗尽后状态为 `limited`，肘高只是
比无偏好更高，而不是始终上升；这是让位于跟踪预算的预期行为。
报告：本机 `run/elbow-ik-validation/online-zero-start.json`。

相关数值、在线控制、场景和协议回归 197 通过、1 项 Python MuJoCo 测试因环境隔离跳过。
原生 Basilisk 重置测试 4 通过（含全零起步），前端初始角度测试 6 通过。
另在独立 MuJoCo 进程对全零起步的示例参考运动每 5 步检查一次，共 101 个静态样本，
未发现机械臂接触；不是连续路径/实际动力学碰撞保证。
本机示例在线求解 P95 约 7.5–8.0 ms，仍存在调度尖峰，不承诺硬实时。
## 历史：抬肘＋腕部向下联合版本验证（2026-09-20，J3 项加入前）

该轮在线版本默认同时启用两项，离线初始化规则不变。六轴全零、默认输入平滑、100 Hz、
本体系 +X 0.02 m/s，运行 5 s 的**控制参考积分**对照：

| 策略 | 肘部相对肩部高度 | J4−J6 高度差 |
| --- | ---: | ---: |
| 关闭在线偏好 | 约 0 | 约 0 |
| 仅抬肘 | 0.15830 m | 0.02445 m |
| 联合拱形偏好 | 0.09298 m | 0.03982 m |

联合版本的额外参考位置偏移约 1.565 mm、姿态偏移约 0.173°，没有放宽原来的 2 mm / 1° 预算。
这个对照体现了两个软目标的权衡，不代表同时最大化肘高和腕部下落高度，也不是实际物理跟踪保证。
本机报告 `run/elbow-ik-validation/online-arch-zero-start.json`。

216 项相关 Python 回归通过，1 项因 Python MuJoCo 隔离跳过；该独立环境的 14 项测试全部通过。
单独对联合策略的 5 s 示例参考运动抽样 101 个静态构型，未发现机械臂接触，
记录在 `run/elbow-ik-validation/arch-static-samples.json`；并不证明连续轨迹无碰撞。
原生 Basilisk 场景启动/重置回归 4 项通过（包含全零初态）；前端测试 96 项通过，生产构建通过。

## joint3 负角项验证（2026-09-20）

默认新增软目标 q3≤−5°，没有修改 MJCF 机械范围，也不修改手动/保存的初始关节角。
6 轴全零、同一 +X 0.02 m/s 指令、输入平滑和 100 Hz 参考积分，运行 5 s：

| 策略 | 最终参考 J3 | 肘高 | J4−J6 高度差 |
| --- | ---: | ---: | ---: |
| 抬肘＋腕部，无 J3 项 | +2.6161° | 0.09298 m | 0.03982 m |
| 三项联合 | −0.1465° | 0.00659 m | 0.00266 m |

该例的负角项与几何目标竞争，且额外位置偏移达到约 2 mm 后停止修正，因此没有达到 −5°。
不能据此承诺更快抬肘或更好的整体视觉外形。另一个从 J3=+11.4592° 起步的同指令示例中，
无该项最终 +26.0505°，有该项最终 +25.6720°，仍为正角：它仅是偏好，不是“正角禁止”。
这些都是控制参考结果，不是物理跟踪误差保证。报告在本机
`run/elbow-ik-validation/joint3-preference-reference.json`。

234 项相关 Python 测试通过、1 项因 Python MuJoCo 隔离跳过；前端 97 项通过、生产构建通过。
另对示例参考运动抽样 101 个静态构型，未发现机械臂接触；不等于连续路径或实际动力学安全证明。
独立 MuJoCo 环境 14 项测试通过；原生 Basilisk 启动/重置 4 项通过（含全零与正角自定义初态）。
