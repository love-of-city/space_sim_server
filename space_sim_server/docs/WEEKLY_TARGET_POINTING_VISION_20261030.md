# 本周目标星调姿、视觉与抓取整合

更新：2026-10-30。提交入口为统一仓库 [PR #39](https://github.com/love-of-city/space_sim_server/pull/39)。本页区分历史试验、新机位复验与正式任务接口；本地运行数据和视频不随代码提交。

## 这次上传什么

| 工作 | 源码和说明 | 当前状态 |
| --- | --- | --- |
| 目标星朝向末端双目并持续跟踪 | `simulation/cooperative_pointing.py`、`end_stereo_pointing.py`；[输入与验证](END_STEREO_POINTING_HANDOFF_20261028.md) | 独立原生动力学验证；尚未装入正式V-bar控制链 |
| 到站→调姿→等视觉→续行 | `simulation/pointing_handoff.py`、`original_stereo_sequence.py`；[顺序试验](TARGET_POINTING_SEQUENTIAL_DYNAMICS_20261028.md) | 已用于完整双臂模型诊断，未继承正式110 m交接 |
| 双目PnP、靶标布局、单目/双目对比 | `backend/space_arm_platform/multicamera_pose.py`；[靶标报告](TARGET_MARKER_REPORT_20261029.md) | 23块多尺寸方形ArUco；保留当前较新的双眼质量输出 |
| 图像位置交给导航 | `simulation/image_relative_navigation.py`、`origin_lever_arm.py`；[回放说明](END_STEREO_POINTING_HANDOFF_20261028.md) | 保留曝光时间、相机安装变换与参考点定义；不是自动放行许可 |
| GNSS近距误差和靶标尺寸核算 | [GNSS动力学与覆盖](GNSS_DYNAMICS_AND_VISUAL_OVERLAP_20261028.md)、`tools/diagnostics/validate_gnss_proximity_dynamics.py` | 与同历元真值比较；越过旧边界只在显式诊断中启用 |
| 原双目从17.4 m后的连续诊断 | [原双目连续试验](FROM_17P4M_CONTINUOUS_ORIGINAL_STEREO_20261029.md) | 到达0.5 m驻留并调用抓取；随后建力超时，整段未通过 |
| 小靶标视觉＋陀螺补偿＋原接触力控 | [末端整合与实测](NEAR23_GYRO_CAPTURE_INTEGRATION_20261029.md)、`simulation/stereo_predictive_capture.py` | 新前端机位；独立4 cm起点严格抓持通过，0.5 m记录另列限制 |
| V-bar对接核查 | [V-bar交接约定](VBAR_VISION_CAPTURE_HANDOFF_20261030.md)、`tools/diagnostics/inspect_vbar_near23_handoff.py` | 找到原状态继承接口，自动视觉抓取尚未接通 |

前一版PR主要包含末端整合。本次补入本周调姿、导航转换、顺序协调器、诊断工具、回归测试和报告；重叠的双目实现与相机配置保留新整合版本，没有用旧文件覆盖。

## 固定配置和版本

当前整合使用 `config/vision/capture26_near23_front_mount_20261029.json`：左眼在夹爪坐标 `[0.075, 0.060, 0.031] m`，右眼 `[0.075, -0.060, 0.031] m`。两眼各1920×1920、100°视场，基线0.12 m、10 Hz；光轴沿夹爪−Z。23块靶标和原灯光保留，不添加长焦。

原机位保存为 `capture26_near23_reference_20261028.json`。此前1421组近距图像和17.4 m连续试验属于原机位；不能直接写成新机位的全程验收。新机位末端图像和接触验证见末端整合报告。本次重跑新机位下18个原生调姿/跟踪工况，见下文。

此前3°/6°/12°额外相机试验已撤回，相关历史文档保留经过，结果不计入当前能力。`long_focus_candidate`直接拒绝创建额外相机；旧名工具 `validate_long_focus_images.py`仅允许原wide通道，名字保留用于历史脚本兼容。[结果范围更正](CAMERA_RESULT_SCOPE_20261029.md)列出了受影响的记录。

## 17.4 m以后，已验证到哪

以下顺序是原机位独立连续诊断的实测经过，不是当前网页已经自动执行的流程：

1. 从约17.9 m独立初始化，GNSS引导至17.4 m节点并停稳5 s。
2. 保持两星相对站位，通过目标星受限等效力矩使靶标面朝向末端相机，连续满足姿态/角速度条件后放行。后续继续控制目标姿态，不是转正后撤掉控制。这里假设有合作姿态遥测，尚无目标星实际执行器设计。
3. 因原100°相机在17.4 m看不清编码，显式启用诊断GNSS续行。5.5 m仍是GNSS停泊，不能叫视觉就绪。
4. 本轮首条图像被导航接受发生在光学深度约2.623 m；在近点约1.539 m首次满足视觉就绪，完成视觉与停稳联合检查后继续。
5. 进入工具间隙0.5 m驻留，989.2 s完成平台检查并调用抓取。
6. 1190.55 s报告 `force_not_established`，所以这条17.4 m到末端的完整抓持没有通过。

17.4 m转正后，5 cm黑框仍约2.315 px；正视达到当前12 px尺寸门槛的估算深度约3.36 m。转正解决倾角问题，不会凭空增加图像像素。真实接入点还受遮挡、检测、歧义、误差和连续有效性影响，不能将3.36 m当成已测得的可靠接入距离。

## 各项误差和抓持证据

| 记录 | 结果 | 能说明什么 |
| --- | --- | --- |
| 原机位，12.5→3.36 m GNSS诊断 | 20 s以后相对位置RMSE 3.47 mm、最大8.17 mm；含初始化最大27.90 mm | 当前仿真噪声假设下可越过旧12 m边界；不是最近极限或3 mm全段保证 |
| 原机位，17.4 m后连续诊断 | 17.4 m停泊最大偏差6.143 mm；5.5 m为6.242 mm；20 s以后导航RMSE 3.085 mm、最大8.168 mm | 单轮诊断中的位置分项；不能代替正式全节点、速度、视觉、接触验收 |
| 原近距成功记录，约0.5 m→5 cm | 1421组、1420组有效；图像机构中心位置RMSE 0.294 mm、最大1.024 mm | 原机位的既有近距图像精度；[同帧单目/双目比较](TARGET_MARKER_REPORT_20261029.md)单独列分母 |
| 新机位，4 cm起点严格抓持 | 1050组图像，42040个400 Hz物理步；四爪承载且压盘力4.5–5.5 N连续20.10 s，正常结束 | 末端视觉＋预测＋接触力控确实能完成抓持；范围从4 cm开始 |
| 新机位，0.5 m起点旧脚本记录 | 图像位置RMSE 0.231230 mm；10 Hz记录出现合格抓持 | 收尾异常且400 Hz日志混写，该轮严格评分不通过；不能补成全程通过 |
| 合入main `3562567`后的新机位兼容运行 | 8 s、79组双目全部有效，位置RMSE 0.065784 mm | 新主线兼容性短测；不等于重做完整抓持 |

上述距离分别涉及光学深度和机构轴向间隙；误差分别涉及导航位置、固定节点跟踪和图像机构位置，不能互换。

用户更新的节点要求是110 m交接位置≤2 m、其余节点位置≤5 cm、速度≤1 cm/s、连续5 s。当前主线另有基于仿真真值的运动演示验收，见 [CAPTURE26_MOTION_DEMO.md](CAPTURE26_MOTION_DEMO.md)。不能用该运动演示证明本分支图像导航和自动抓取已经连续接通。

## 新机位调姿复验（2026-10-30）

使用本分支原生Basilisk/MJScene、实际模型惯量、实际夹爪FK与当前双目标定，目标星等效力矩范数限制0.02 N·m，控制20 Hz、积分240 Hz。

- 基础7项全部通过：17.4 m处0°/30°/90°/180°，50 m和5.5 m处90°，以及收拢相机不在共同视场时正确拒绝。
- 17.4 m的30°/90°/180°工况分别在38.85/42.55/58.45 s完成连续5 s对准，结束姿态误差分别0.1563°/0.4183°/0.0517°。
- 跟踪11项全部通过：三轴正负90°与180°，以及服务星转动/目标初始角速度、横向移动两项。后两项在对准后继续保持10 s，结束误差0.02491°/0.02420°。
- 三个机械臂构型下解析FK与原生FK的位置差小于4e-16 m。

这是简化刚体调姿和投影检查，固定机械臂、理想合作姿态遥测，未包括轨道扰动、机械臂展开、图像解码或遮挡验收；不是正式400 Hz全模型任务。

本地证据目录：`run/weekly_upload_20261030/pointing_current_front`、`tracking_current_front`；两份 `summary.json`均为 `passed=true`。绑定的模型SHA256为 `1485987b90df5eff177743fbe6d34985d262f56dfa94b676d6c62627d3867c9c`，相机配置SHA256为 `b1589a44bc55746cfa279622fb9407527ef7e06739f13d4302c048f0fd9579f7`。

本次本地相关回归147项通过，另有300 s以后力反馈连续记录/多试验日志隔离回归1项通过；11个主要诊断入口的 `--help`检查通过，仓库Ruff检查通过。基础CI与原生实际运行分别记录，不能相互替代。

## 复验入口

命令均从统一仓库的 `space_sim_server/` 运行。Python和适配器按子项目README安装；原生试验还需兼容的Basilisk MJScene，不能用普通Python MuJoCo替代。图表/视频工具另需Matplotlib、Pillow、FFmpeg及中文字体；旧UE试验工具依赖已准备的UE 5.6安装和显式 `--source-server` 历史资源，不是全新机器一键复现入口。

```powershell
python -m pytest tests/test_cooperative_pointing.py tests/test_end_stereo_pointing.py tests/test_cooperative_stereo_translation.py tests/test_dual_range_vision.py tests/test_image_epoch_roundoff.py tests/test_origin_lever_arm.py tests/test_original_stereo_sequence.py tests/test_pointing_handoff.py tests/test_stereo_predictive_capture.py tests/test_near23_trial_score.py tests/test_near23_capture_scene.py tests/test_near_field.py -q

python tools/diagnostics/validate_end_stereo_pointing.py --scene-model model/SARM/platform/sarm_dual_capture26.xml --camera-config config/vision/capture26_near23_front_mount_20261029.json --output run/pointing_new
python tools/diagnostics/validate_end_stereo_tracking.py --scene-model model/SARM/platform/sarm_dual_capture26.xml --camera-config config/vision/capture26_near23_front_mount_20261029.json --output run/tracking_new
python tools/diagnostics/inspect_vbar_near23_handoff.py --output run/vbar_contract_new.json
```

GNSS整模工具需要 `--basilisk-root`、`--source-scene`、`--scene-model`、`--camera-config`；先查看 `--help`，使用实际准备的资源路径。`continuous_original`必须显式选择GNSS诊断续行；CLI执行成功不等于 `requested_stage_completed=true`，应检查独立审计和最终建力结果。

本次没有更改正式V-bar放行规则、任务默认相机、默认目标星姿控或网页自动触发点。旧通用V-bar的12 m无视觉保护仍在；当前capture26默认则是另一条使用仿真真值的运动演示，不能把两条控制路线混为“GNSS已经解除12 m限制”。正式接通需要按[交接约定](VBAR_VISION_CAPTURE_HANDOFF_20261030.md)补齐并在同一条实际任务中验收。
