# 双臂 SARM / Capture26 模型导入

## 新工作区启动

使用原平台入口选择 `sarm-dual-capture26` 即可准备机器本地资源目录：启动脚本构建 SARM thread 插件，依次恢复基础 SARM、Capture26 机构和当前 `adjacent_face_v1` 星体网格的 catalog，最后检查并合并映射。它不会重新生成已纳入版本控制的双臂物理模型或基础装配。

网页选择双臂模板时默认使用“指定操作姿态直接启动”。原零位自动展开规划器仍使用旧夹爪关节，因此该模板明确拒绝此模式；固定基准 `none` 继续用于已有实验实例。普通启动仍不自动执行抓取。

当前侧面安装布局使用 `model/dual_arm_capture26/import_trimmed_base.xml` 中的相对源路径，导入到 `/Game/BSK/Generated/Capture26SideV1`；基础与机构资产保留各自的原 UE 路径。以下早期底部安装记录不能作为当前裁切网格的导入配置。`Saved/AssetImport` 为机器本地生成内容，不提交 Git。

双臂 manifest 的历史 `base_commit/base_sha256` 保留；运行依赖 `sources` 对齐当前基础 XML。已核对基础版本差异仅为 `15a771e` 添加的渲染用腕灯，动力学和相机未变，双臂 XML 也未改写。启动继续严格检查模型及依赖摘要。

## 2026-10-22 物理抓取基线的接触保护

显式开启 `randomization.capture_window_trial` 的实验仍按张开、伸出、接近、收拢、软垫力控拉紧运行；普通启动不自动抓取。此基线使用仿真位姿定位，末端相机视觉结果仅供观察，不能作为视觉闭环抓取的验收结果。

自由漂浮机械臂伸展会移动服务星本体，不能把从初态累计的两星原点相对位移全部解释为目标受撞击。该位移继续记录；50 mm 中止判据改为从首次承力接触开始累计，并明确命名为“接触后的相对原点位移”。任意阶段的目标单接触法向力超过 10 N 会中止，自身碰撞保护和原有拉紧力／压缩量保护继续保留。该位移仍包含服务星反作用和轨道运动，并非目标单独的惯性位移。

0.5 m 行程按原有 4 mm/s 速度上限至少需要约 125 s，接近超时由 110 s 改为 240 s。模型、质量、惯量、碰撞过滤及两星自由飞行约束不变。400 Hz 接触诊断保留至 300 s，以覆盖完整接近和保持阶段。

基线：GitHub origin/main 7f01764，保留 sarm_20261012.xml 的原操作臂、任务盒、目标星、原有 36 个刚体的惯性；显示属性除下述底部避让修改外保留。新增 14 个刚体，底部安装视频中的旧六轴抓取臂及四 L 爪。横脚 14 mm，轴向行程 26 mm，四爪收拢行程各 12 mm。

网页场景：双臂 SARM + 四 L 爪（26 mm 行程·导入预览），template_id=sarm-dual-capture26。原机械臂使用现有遥操作；新增机械臂由独立位置伺服保持折叠、缩回与收爪状态。没有把它接入原机械臂的八维操作输入，避免误操作。

重要边界：这是模型导入预览，不是平台抓取成功验收。目标星继续使用上游粗碰撞体；视频的精细接口碰撞与接近轨迹未迁移。软垫仍是刚体。新增臂与服务星之间未启用全自碰撞，因此不能据此判定安装没有干涉。原视频成功工况不自动适用于新星体质量、惯性及控制系统。

验证：原生 Basilisk/MJScene 可加载 50 个刚体；原有 36 个刚体的惯性及非避让部分比较一致；新增模型回归 2 项、现有场景及模型回归 19 项通过。UE 34 个新增资产导入成功，合并目录保留原有 UE 资产路径，运行日志确认 PBR0110 外覆层加载。网页收到实时渲染和仿真状态。模型位于 model/SARM/platform/sarm_dual_capture26.xml。

首次恢复新增 UE 资产可使用适配器 scripts/prepare_mjcf_assets.ps1，将 model/dual_arm_capture26/import_assets.xml 导入 /Game/BSK/Generated/Capture26，并输出 Saved/AssetImport/capture26.catalog.json。基线需要已有 sarm_20261012.catalog.json，随后 tools/verify_capture26_import.py --adapter-root <adapter-path> 验证并生成合并目录。部署仍使用原平台启动入口。

底部安装避让修正：新场景副本中的星体底部突出几何由最低 Z=-690 mm 裁至安装面 Z=-490 mm，涉及 1 个显示网格与 8 个碰撞几何；源模型不覆盖，惯性不改。具体记录见 model/dual_arm_capture26/mount_clearance_changes.json。额外导入 import_trimmed_base.xml 至同一 UE 目录，目录文件为 capture26_trim.catalog.json。该场景的卫星总览相机调整为从底部观察新增折叠臂。此检查不等于全行程自碰撞验收。

底部检查视角附加 15,000 lux 的渲染补光，仅该预览模板启用，不代表真实在轨照明，不影响动力学。

独立 MLI 显示外覆层也需同步避让：保留原 OBJ 的纹理坐标与三类材质，移除触及 Z<-490 mm 区域的三角面，生成 ContentSource/Capture26/SM_Capture26_MLI.obj 与 /Game/BSK/VisualOverlays/Capture26/SM_Capture26_MLI。源 PBR0110 资产不覆盖。
渲染导入修正：新增臂各 body 的自身 geom 排在子 body 前，与 MuJoCo 编译顺序一致；新增 group=3 碰撞几何设为透明，保留接触参数及显示网格。

2026-10-17 显示与工作灯：该预览模板只在渲染桥接阶段清空 rw_x/rw_y/rw_z 的显示几何，完整 MJCF 和 attitude_control.json 保持逐字节不变。底面在 Z=-490 mm 外侧添加与其他面一致的 PBR0110 保温膜，安装座留出避让；生成脚本为适配器 scripts/generate_capture26_bottom.py，导入使用 scripts/import_capture26_mli.py。网页“工作灯”经 Pixel Streaming 的 BskWorkLight 命令控制固定 ID 的预览灯，查询实际可见状态；不修改太阳、物理或姿控。切换相机及页面重连后重新查询，不重置共享灯状态。该开关仅在带该预览灯的场景可用。
# 2026-10-17 中键预览观察

当前本地预览实例将目标星原点设为 `(1.2345, 0.5305, -0.585)` m，姿态保持原值。
新增可选实例字段 `randomization.target_relative_velocity_m_s` 明确以服务星速度为基准；
本实例为 `[0,0,0]`，角速度也为零。旧实例未配置该字段时沿用原初始化语义。
不要把旧 `target_linear_velocity_m_s` 当作两星相对速度：服务星还具有 COMMON_VELOCITY。
目标星最近侧面 X≈0.85 m，抓夹参考原点 X≈0.529 m；这是无接触的观察起点，不是已对准捕获状态。

顶面贴膜：`generate_capture26_bottom.py` 同时生成任务盒所在面的裸露箱体贴膜。
任务盒占位采用 STL 包围盒叠加安装平移：X=0.125..0.575 m，Y=0.065..0.615 m；
机械臂底座另留 X/Y=±0.055 m 的开口，两者周边各留 0.006 m。
仅更新 Capture26 可视覆盖层，不替换任务盒材质、不修改物理模型或相机配置。

卫星总览相机支持中键拖动环绕观察、松开保持、Home 恢复。
原始相机 actor/manifest、FOV 与模型参数不变，没有新增相机。
仅预览 SceneCapture 在渲染时叠加观察变换，并立即恢复；权威采集不使用该偏移。
开始拖动会退出机械臂操作模式，失焦和断流不保留运动速度。
当前只支持卫星总览视频流；主视口原有 C 自由观察和腕部固定相机保持原行为。
该预览变换作用于共享的总览视频流，多位观看者会看到相同角度。



## 2026-10-18 camera and preview controls

- `capture_wrist_cam` is attached to the `gripper` body at local (0.10, 0, 0.06) m, looking along gripper -Z. FOV 85 degrees. It follows the wrist and is independent of radial jaw motion.
- Launch the local four-view platform with `D:/mingti/outputs/gripper_platform_integration/run_camera_platform.ps1`, then the saved instance with `restart_preview.ps1` in the same directory.
- All four streams accept middle-button drag and Home. Offsets are presentation-only, stored per stream. Original body-fixed camera transforms, including upstream overview, remain unchanged. Dataset captures use the original camera pose.
- Each onboard camera has an independently switched spot light attached to it. The main viewport controls the existing inspection fill light. Lamps illuminate a shared scene, so light spill is expected; switching one lamp off does not extinguish sunlight or other lamps.
- Browser tests: 186 passed. UE Development Editor compile passed. Kinematic preflight verifies camera follows wrist while retaining the mount offset. Full live checks are recorded separately.
