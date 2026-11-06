# D37：真实双目图像闭环接近

`d37-stereo-body-approach-v1` 从约 5 m 的双星 COM 间距出发，仅控制追踪星平移，使目标本体原点在左光学相机系中到达 `[0, 0, 2.2] m`，位置误差不超过 5 cm、该旋转相机系内的相对速度不超过 2 cm/s，并连续保持 5 s。初始相机系位置误差至少为 1 m；已经处于目标位置的初始化不能计为接近成功。最长 120 s，在图像决策边界结束回合。

场景使用原 SARM、10 kg 目标星、被动已知 5 cm 标志物、400 Hz 原生动力学和 50 Hz 独立物理记录。两星初始角速度为零，之后姿态被动演化。目标没有遥测或控制配合，合作标签为 P；此配置不覆盖无标志非合作目标、主动姿态对准或接触。

方法每 0.5 s 接收真实 640×480 双目 PNG，基线 0.12 m；公开输入只有任务、静态标定、质量、标志物几何及图像。动作包含当前图像位姿估计和追踪星本体系推力，模长上限 5 N。推力直接驱动原生本体轴电机，作用于装配体 COM，在相邻图像间保持；不经过真值姿态导航或隐藏姿态保持。实体飞轮保留且命令为零。该模型是理想有界 COM 力，没有推进剂、电气能耗或实际喷口布局模型。

内置 `image-stereo-approach-v1` 在隔离 OpenCV 进程中估计位姿，以最多 2 s 的过去图像拟合相对速度，再计算相机系位置反馈及阻尼，并按静态安装旋转转换成本体系推力。失锁时清空历史并停止推力，重新积累至少三次位姿后才恢复反馈。`no-visual-thrust-v1` 保留图像估计但推力恒为零；`unsafe-visual-close-v1` 持续施加允许幅值的接近推力以验证安全否决；`missing-visual-control-v1` 缺少位姿估计且推力为零。外部示例为 `examples/autonomy_methods/vision-approach.json`。

评分根据实际相机安装与权威本体状态计算目标原点相对位置、相机系旋转项以及两个 COM 至原点的速度杆臂项。COM 相对速度为零不等于图像中的目标静止。50 Hz 连续有效驻留才计时，中途超出位置或速度容差会清零；请求动作必须与每个物理时刻的实际电机回执一致。任意非计划接触、距离/速度/角速度/推力超限、初始化后的状态改写均不能通过。

位姿估计另行检查有效比例至少 95%、位置/姿态 RMSE 不超过 3 cm/0.06 rad、峰值不超过 8 cm/0.16 rad。提前完成时只要求此前实际控制过的图像序列完整，未执行的未来决策时刻不计入有效率；最后额外观测不增加方法估计数。推力积分和恒定质量下的等效 Δv 从物理回执计算，不输出虚构燃料或能耗。

```powershell
python tools/autonomy_benchmark.py spec d37-stereo-body-approach-v1
python tools/autonomy_benchmark.py run d37-stereo-body-approach-v1 --seed 1 --output run/visual-approach-training-1
python tools/autonomy_benchmark.py batch d37-stereo-body-approach-v1 --split test --output run/visual-approach-test
python tools/autonomy_benchmark.py rescore run/visual-approach-training-1
```

当前为隔离开发分支中的实现，真实原生/UE 训练、冻结测试、负对照、复位及网页验收尚未完成。评分器的合成运动用例只验证数学和证据规则，不是物理场景成功证明。

云端首轮相关接口、图像/运动/控制评分、目录、执行、数据集和平台回归共 198 项通过；Ruff 通过。261 项前端检查和生产构建通过。新增用例覆盖提前完成时的实际图像覆盖、位置/速度中断后驻留清零、实际电机回执缺失、安全否决，以及含两侧杆臂和旋转相机的速度公式与独立有限差分的一致性。

实际加载原生环境类后，四个方法均完成公开上下文初始化与关闭；图像方法在独立 OpenCV 进程内运行，父进程没有加载 OpenCV。该检查没有运行物理场景或估计评测图像，报告 `.tmp/benchmark-approach-protocol-v1/acceptance.json` SHA-256 为 `bed50ad8f218f1cf364e03880ab3537b4450ad968c9f6624758562b14ea233eb`。
