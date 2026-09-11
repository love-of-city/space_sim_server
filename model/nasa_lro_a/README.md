# NASA LRO (A) — 固定结构 MJCF 视觉模型

## 打开什么

- 主文件：`nasa_lro_a.xml`；必须与 `meshes/` 一起保留，不能只复制 XML。
- `preview.html` / `preview/overview.png`：运行预览脚本后生成的离线几何预览。
- `conversion_manifest.json`：源文件校验和、节点/材质映射、逐网格统计和验证结果。

## 本版范围

- 原始 GLB 未修改，SHA-256：`b9547bc7c8c37c44675a80c3c3e27b647a9ff1293cf40030ef36f806b9c616b1`。
- 5 个源网格组、11 个源节点、34 个基础色材质，导出 55 个 OBJ / visual geom。
- 源文件解码得到 76,349 个三角形，其中 2,250 个面积严格为零。仅剔除这些无可见面积的退化面，保留 74,099 个非退化三角形；没有减面或补厚。
- 保留分组层级，但所有 body 都固定。`Layer` / `Pivot` 名称不是机械关节证据。
- 不添加关节、自由基座、执行器、碰撞体或虚构的质量/惯量；关闭几何惯量推断。`inertia="shell"` 仅用于开放/平面网格的编译处理，不代表卫星真实惯量。
- 保留基础色和顶点法线，不保证 glTF 金属/粗糙度 PBR 材质与 MuJoCo 显示完全相同。

## 坐标与尺寸（重要）

- glTF 的 Y 向上转换为 MJCF 的 Z 向上：`(x, y, z) -> (x, -z, y)`，右手旋转，不镜像。
- 所选场景的节点世界变换已烘焙入 OBJ；固定 body 的坐标变换均为单位变换。不能把 body 原点当作转轴位置。
- 原点不居中、不移动；统一缩放倍率为 `1.0`，即每个源坐标单位按 `1.0` 米解释。
- 导出轴向包围盒 X/Y/Z 为 **33.277178 × 32.258003 × 26.293749 m**。
- **上述数字是文件尺度，不是经实测确认的 LRO 尺寸。真实尺寸尚未校准。** 确认一个已知长度后可使用转换器 `--scale` 参数在新目录重新生成，不能直接把此版用于真实动力学计算。

## 验证

- MuJoCo `3.12.0` 加载/编译成功。
- 55 个 visual geom；关节 / 自由度 / 执行器 = 0 / 0 / 0；所有 body 质量和惯量均为 0（固定视觉节点）。
- 编译后逐三角形位置对照已通过，最大顶点误差 `1.03e-06 m`；法线和基础色对照通过。
- 100 步固定场景 smoke test 无警告、无接触；这不是活动机构或真实动力学验证。
- 未改动平台场景/默认配置；尚未测试平台或 UE 导入，也未使用原生 OpenGL 渲染器验证。

## 复现（在仓库根目录执行；建议独立 Python 3.11 环境）

```powershell
python -m pip install --only-binary=:all: -r tools/requirements-glb-to-mjcf.txt
python tools/convert_glb_to_mjcf.py "run/nasa-lro-a/Lunar Reconnaissance Orbiter (A).glb" --output model/nasa_lro_a_new --name nasa_lro_a
python tools/render_glb_preview.py model/nasa_lro_a_new
```

转换器拒绝覆盖已有非空目录。本工具仅支持静态、无纹理、Draco 压缩的三角网格 GLB 子集，不是通用 glTF 场景/动画导入器。
预览使用已有的 CPU z-buffer 绘制 MuJoCo 编译后的几何，非 AI 图片，非平台/UE 截图。

## 格式依据

- Khronos glTF 2.0 specification，Coordinate System and Units。
- Khronos `KHR_draco_mesh_compression` extension，属性按 unique ID 解码。
- MuJoCo XML Reference，mesh / material / compiler。
- DracoPy 官方解码库（转换记录中列出确切版本）。
