# 夹爪训练数据：6+1 维

当前 SARM 四连杆夹爪（`four_bar_tendon`）的新录制使用 `space-arm-lerobot/2`。LeRobot 容器仍为 v3.0，RGB 仍为 1024×1024、30 Hz。

## 训练接口

- `observation.state`: `[J1, J2, J3, J4, J5, J6, gripper_opening_normalized]`，前六项为实际关节角（rad），最后一项为实际开合量。
- `action`: 相同的七维排列，使用该观察时刻实际保持的伺服目标，最后一项为目标开合量；不是最近收到但尚未执行的操作命令。
- 0 = 闭合；1 = 当前控制范围内最大张开；中间值连续。
- 当前已用的控制和任务范围为 `ee_link11_joint` 的 0–1 rad，归一化公式为 `clip((q - closed)/(opened - closed), 0, 1)`。因此当前模型数值恰好相同，但归一化开合量没有角度单位。
- 这个范围来自当前遥控限位及抽屉/插头脚本的工作端点；不是把 XML 的被动关节极限 ±π 当作开合范围，也不声称是硬件机械极限标定。
- 开合量不是指尖实际间距的线性比例，更不是夹持力。夹住物体时，实际值和目标值可以不同。

## 原始诊断

`observation.raw_joint_position` 和 `observation.raw_target_joint_position` 保留全部八维原始坐标，包括两个联动关节。`observation.joint_velocity` 仍是八维原始速度，不是七维 state 的导数，名称和语义在元数据中明确标出。软约束/接触造成的越界和联动残差不会因归一化裁剪而从原始数据中丢失。

`steps.jsonl`、`observation.platform_json` 中的旧字段仍保留，并增加 `gripper_opening_normalized`、`target_gripper_opening_normalized`。两个新字段由同一原始观察计算；即使读取携带旧派生值的 JSON，也会重新计算，避免与原始状态不一致。腕部六维力和相机同步规则保持原有含义。

`meta/platform.json` 写入 schema、归一化端点、源坐标、单位、裁剪规则及状态/动作语义。`meta/info.json` 写入七维 shape 和明确的特征名。

## 兼容及扩展

历史八维录制不重写。旧 parallel-slider 模型没有在本次标定归一化端点，继续使用旧版 schema 和布局。训练时应按 `platform.json` 和特征 names/shape 区分，不应直接拼接不同版本。后续夹爪应在 `backend/space_arm_platform/gripper_representation.py` 增加明确的范围/坐标定义，并增加对应数据映射及测试，不能沿用别的模型的角度范围。

独立查看器增加“夹爪开合 · 0 闭合 / 1 张开”曲线，同时保留驱动角诊断。旧记录缺少归一化字段时隐藏该曲线，不猜测端点。

修改的是训练数据表示，不改变 MuJoCo 的关节数量、动力学或遥控接口。运行中的后端需要重启后新建录制；本次无 UE C++ 改动，无需重编译 UE。
