# 地面验证星 STEP → MJCF 预览

这是从 `../地面验证星模型.stp` 本地转换得到的**独立几何预览**，不是已标定的卫星动力学模型。
原始 STEP、SARM 模型及平台启动配置均未修改。转换过程没有上传 CAD 文件到云端。

## 先看结果

- `preview/overview.png`：四视角预览总图（读取 MuJoCo 编译后的网格/位姿，由 CPU 深度缓冲渲染；不是 MuJoCo OpenGL 截图，也不是 AI 生成图）。
- `preview/01_isometric.png` 等：单张 1200 × 900 预览。
- `ground_validation_satellite.xml`：固定、无碰撞的视觉 MJCF。**查看外观请先用这个文件。**
- `satellite_free_preview.xml`：一颗自由刚体的测试版本，**质量人为设为 1 kg，惯量按均匀外接盒估算**。仅用于验证加载和自由运动，不可用于真实任务数据。
- `meshes/*.obj`：103 个按 CAD 面颜色拆分的网格。
- `conversion_manifest.json`：来源 SHA-256、装配层级、实例变换、颜色、原点变换、网格校验和及验证结果。
- `diagnostics/part_009_face_00888.brep`：调试时保存的原始 B-spline 面。最终已换用 Delabella 成功网格化，**不是缺失面**。

## 本次结果

| 项目 | 结果 |
|---|---|
| CAD 来源 | Creo Parametric 导出的 STEP |
| 原文件长度单位 | 毫米；网格和 MJCF 已转换为米 |
| 包围盒尺寸 X/Y/Z | 0.769 × 0.1665 × 0.60015740026 m |
| 读取到的 XCAF 节点 | 226（含装配、引用、叶子） |
| 有可显示表面的实例 | 151 |
| 唯一形状定义 | 74，其中 5 个仅含线/基准，无可网格化的面 |
| 输出网格 / visual geom | 103 / 206 |
| 使用的 RGBA 颜色 | 37 |
| 唯一网格三角形 / 实例展开三角形 | 1,569,791 / 1,694,573 |
| 丢失 CAD 面 | 0 |
| 几何精度设置 | 绝对线性偏差 0.25 mm，角度偏差 0.18 rad |
| 局部补救 | 1 个有效 B-spline 面改用 Delabella 三角化原始面，无人工补洞 |

细节较多，OBJ 约 165 MiB。这一版优先保留外观，尚未针对 UE/批量训练做减面和性能优化。
转换时删除了 918 个数值零面积三角形，并在清单中记录；这不等于删除 CAD 面。
仅含线/基准的五个定义也已明确记录，不会伪造为实体。

## 坐标与树结构

保留源 CAD 的 X/Y/Z 轴方向，只将模型原点平移到整体包围盒中心。它**不是测得的质心**。
源坐标到当前模型坐标：

```text
p_model_m = p_source_m - [-0.1755000000000304, -0.05475000000002002, -0.047078700129989326]
```

目前所有外观件固定在一个 `ground_validation_satellite` body 上。
STEP 装配层级和每个实例的变换完整记录在 manifest 的 `assembly` 中，没有把装配层级误当成可动关节。
**未推断**太阳翼、轮体、铰链、滑轨等关节，也未接入现有机械臂。

## 本机查看

当前工作区：`C:\Users\LYH\space_sim_server`。
专用转换虚拟环境在 `run/step-converter-venv`，没有修改 `mujoco-dev` 或 `space-sim-server` 环境。

若系统 OpenGL 正常，直接运行：

```powershell
.\run\step-converter-venv\Scripts\python.exe -m mujoco.viewer `
  --mjcf .\model\ground_validation_satellite\ground_validation_satellite.xml
```

这台服务器的系统 WGL 不支持所需 OpenGL，因此本次预览使用**独立 CPU 三角形光栅化器**，直接读取 MuJoCo 编译后的顶点、面、法线、颜色和世界变换。支持真实深度遮挡、透视校正和双面 CAD 曲面显示；光照为预览用简化光照，不与 MuJoCo/UE 渲染器完全相同。

```powershell
# 打开已经生成的图片，不需要 OpenGL
.\model\ground_validation_satellite\view_preview.ps1

# 重新生成 CPU 预览
.\model\ground_validation_satellite\render_preview.ps1
```

`view_preview.ps1 -Interactive` 可在具有正常系统 OpenGL 的机器上打开可旋转的 MuJoCo Viewer；当前服务器不保证可用。

环境说明：曾尝试从上游 GitHub 获取 Mesa 软件 OpenGL 包，但被 Windows Defender 隔离。没有解压/执行它，没有恢复隔离或更改防护设置，最终预览也没有使用 Mesa。

## 重新转换（从服务端 Git 仓库根目录执行）

新机器先建立独立环境：

```powershell
python -m venv .venv-step
.\.venv-step\Scripts\python.exe -m pip install -r tools/requirements-step-to-mjcf.txt
.\.venv-step\Scripts\python.exe tools/convert_step_to_mjcf.py `
  "model/地面验证星模型.stp" --output model/ground_validation_satellite --no-render
```

生成不依赖 OpenGL 的 CPU 预览：

```powershell
.\.venv-step\Scripts\python.exe tools/render_step_preview.py model/ground_validation_satellite --backend cpu
```

`--no-render` 不跳过几何/编译验证，只跳过转换器内置的 OpenGL 图片渲染。正常 OpenGL 机器也可以将渲染命令改为 `--backend opengl` 使用原生 MuJoCo 渲染器。
默认遇到无法修复的 CAD 面会报错；`--allow-incomplete-preview` 才允许明确记录缺面。**本次最终结果未使用该选项。**
输出目录只能是空目录，或由此脚本为同一个源 SHA-256 创建的目录；不会覆盖其他现有模型目录。

## 已做验证 / 尚未做验证

已做：

- 两个 MJCF 都通过 MuJoCo 3.12.0 编译。
- 校验全部 206 个外观 geom 的编译后位置和包围盒，补偿 MuJoCo 的网格自动居中/旋转。
- 原 STEP 包围盒、网格包围盒、编译后包围盒一致到相应的浮点精度。
- 自由预览具有 1 个 freejoint、6 个速度自由度，带初始线速度和角速度运行 500 步（1 s），状态有限且无仿真警告。
- 转换器有 30 项回归测试，包括嵌套旋转/平移、重复实例、面颜色、SI 单位、平面色块、三角形细分、MJCF 端到端加载和 CPU 深度缓冲遮挡。

尚未做：

- 真实质量、质心、惯量标定；各部件的运动学/驱动建模。
- 抓取/插接用碰撞体与接触参数标定。自由预览的外接盒会填平孔洞，不适合接触任务。
- 现有平台的 Basilisk/MJScene 集成、UE 资产 catalog 导入和遥操作测试。

请不要用本次预览中的 1 kg 和外接盒惯量替换真实卫星的质量属性。
