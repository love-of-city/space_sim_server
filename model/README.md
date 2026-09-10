# 仿真模型与 CAD 源文件

此目录是服务端仓库内的模型源文件，以这里的文件为准。

## 目录和入口

- **`SARM/platform/sarm_ground_target_self_collision.xml`**：当前默认，保留三个外部抓取粗盒、增加两个板件内部接触粗盒，开启外侧板与固定目标接触。原始模型/质量/惯量不变；铰链邻域间隙为近似，不是真实限位。详见[粗碰撞内部接触](../docs/COARSE_SELF_COLLISION.md)。
- `SARM/platform/sarm_ground_target.xml`：旧无内部接触粗盒组合，保留给旧模板/实例，也作为高精度试验的历史输入，未修改。
- `ground_validation_satellite/mesh_collision_trial/sarm_mesh_collision.xml`：保留的高精度三角网格碰撞实验版，显式选择 `sarm-ground-validation-mesh-grasp` 才加载；原始转换文件未删除。铰链零位接触、显著慢于实时的限制仍在。
- `SARM/platform/sarm_platform.xml`：SARM 基础场景及旧模板/离线回归入口，包含卫星、三轴反作用轮、六轴机械臂、双指夹爪和原小方块目标。
- `ground_validation_satellite/ground_validation_satellite_articulated.xml`：用户指定的原始关节卫星模型，保留用于 CAD/机构预览，不直接替代整个 SARM 场景。
- `SARM/platform/attitude_control.json`：惯性姿态保持的开关/频率/增益（高精度组合目录保留其同内容副本，修改后需重新生成/验证）；硬件质量、惯量、限矩和轮速边界在平台 XML。详见[姿态控制说明](../docs/ATTITUDE_CONTROL.md)。
- `SARM/platform/scenarios/scenario_sarm_grasp.py`：原生 Basilisk/MJScene 控制脚本，路径相对于脚本解析，不依赖特定用户名或盘符。
- `SARM/mjcf/SARM.xml`：独立模型入口；`SARM_scene.xml` 是其独立查看场景，并非平台运行入口。
- `SARM/meshes/`：OBJ/STL 网格源文件。
- `SARM/urdf/`、`config/`、`launch/` 及包描述：原始 URDF/ROS 导出配置。
- `任务盒_v1/`：SolidWorks 装配/零件和 STEP 源文件，仅作为设计源文件保存，不由平台启动脚本直接加载。
- `*_raw.xml`、`*_validated*.xml`、`*_before_*.xml`：保留的转换/验证版本，不是默认运行入口。

SARM 独立模型与四个平台组合 MJCF 入口都包含 `spacecraft_overview` 和 `sarm_wrist_cam` 相机。
轨道参数由服务端场景实例/星历配置控制，不由 CAD 文件决定。

## Git LFS

OBJ、STL、SLDASM、SLDPRT、STP/STEP 通过根目录 `.gitattributes` 使用 Git LFS；
XML、URDF、Python 和配置仍作为普通文本跟踪。克隆/更新后，在服务端和 UE 适配器仓库分别执行：

```powershell
git lfs install
git lfs pull
```

不要把 LFS 指针文本误当成模型文件。UE 的已导入 `.uasset` 在适配器仓库，
`Saved/AssetImport` 中含本机绝对路径的映射由 `scripts/run_platform.ps1` 调用适配器资源准备脚本生成。
直接执行 `run_simulation.ps1` 前，需先通过平台启动脚本完成资源准备。

## 本地兼容目录（2026-09-08 迁移）

原工作区将 `model/` 放在服务端 Git 仓库外。迁移时已逐文件校验 SHA-256，
并保留完整原始备份于工作区的 `run/model-versioning-20260908-*/model-original/`。
外层旧 `model` 路径是指向本仓库 `model/` 的 Windows 目录联接（junction），
因此原 IDE 路径和既有启动参数仍指向同一份文件，不存在两套需要手动同步的模型。
联接和备份仅用于原机器兼容；新克隆只需要本仓库的模型目录。

## 仅本地保留、不提交的内容

- 顶层 ZIP 导出包（已纳入展开后的模型/CAD 源文件）。
- `SARM/export.log`、`SARM/mjcf/SARM.txt` 和 `SARM_scene.txt` 编译转储。
- Python `__pycache__` / `.pyc`。
- `SARM/platform/sarm_platform_diag*.xml` 临时诊断变体。

上述文件没有删除；它们保留在本机且受 `.gitignore` 排除。
