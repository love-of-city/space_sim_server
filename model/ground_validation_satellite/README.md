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

## 查看与可选重新转换

现有 `preview/` 图片可直接打开。交互查看需要可用的 OpenGL 驱动。静态预览与转换不属于平台首次部署步骤。

如确需重新转换，从服务端仓库根目录创建**独立**转换环境，避免更换 Basilisk 所用的 MuJoCo。uv 用户：

```powershell
uv venv .venv-step --python 3.13
$converterPython = (Resolve-Path .\.venv-step\Scripts\python.exe).Path
uv pip install --python $converterPython -r tools/requirements-step-to-mjcf.txt
```

Conda（使用同一目录作为环境前缀，无需激活）：

```powershell
conda create --prefix .\.venv-step python=3.13 pip
$converterPython = (Resolve-Path .\.venv-step\python.exe).Path
& $converterPython -m pip install -r tools/requirements-step-to-mjcf.txt
```

传统 venv/pip：

```powershell
py -3.13 -m venv .venv-step
$converterPython = (Resolve-Path .\.venv-step\Scripts\python.exe).Path
& $converterPython -m pip install -r tools/requirements-step-to-mjcf.txt
```

上面三种环境创建/安装方式只选一种；它们都将所选解释器保存为 `$converterPython`。已有环境只需设置该变量并执行对应安装命令，不重复创建。然后运行转换工具：

```powershell
& $converterPython tools/convert_step_to_mjcf.py "model/地面验证星模型.stp" --output run/ground-validation-preview --no-render
& $converterPython tools/render_step_preview.py run/ground-validation-preview --backend cpu
```

仅在该环境/输出目录尚未存在时执行创建步骤。输出放在本机 `run/` 下，不覆盖已经集成的平台模型。CPU 预览读取编译后的几何；不是 UE 画面。`--no-render` 仍执行转换校验，后续单独渲染。

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
