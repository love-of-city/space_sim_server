# SARM 运行渲染资产同步

同步已在本地重建的 SARM 网格、Foil002/PBR0110 材质纹理与隔热毯可视覆盖层，使仓库分发的 UE 资产与本地使用的导入结果一致。共更新 134 个已有 `.uasset`，约 105 MB，沿用原包路径，全部通过 Git LFS 存储。

| 资产目录 | 数量 |
| --- | ---: |
| Generated/SARM | 113 |
| Materials/Foil002 | 7 |
| Materials/PBR0110 | 5 |
| VisualOverlays/SarmMLI | 5 |
| VisualOverlays/SarmMLI_PBR0110 | 4 |

导入来源核对中，120 个包的来源 MD5 与原版本一致；link1–link5 的导出 OBJ 来源 MD5 更新；9 个材质或材质实例没有文件导入 MD5。二进制变化包含重新导入和保存产生的包元数据，不将包哈希变化解释为所有源几何都发生改变。源模型、纹理源文件和运行协议不在此提交中修改。

验收使用 UE 5.6 命令行编辑器的 Python 脚本逐一加载改动包，对静态网格检查 LOD0 三角形数量大于零，对材质实例检查有效父材质。脚本不保存资产。UE Development Editor 构建与运行时产物检查通过；真实 UE RGB 冒烟验证在空闲时输出 0 张、两段采集分别输出 12 和 6 张、停止后没有新增。

复核命令模式为 `UnrealEditor-Cmd <project.uproject> -run=pythonscript -script=<validation.py> -nullrhi -unattended -nop4 -nosplash`。验证脚本的核心是 `EditorAssetLibrary.load_asset`、`StaticMesh.get_num_triangles(0)` 和材质实例 `parent` 检查；输入包列表来自本提交的 `.uasset` 差异。

基础 Python CI 无法代替 UE 包加载验证。仓库只更新可分发的 Content 资产，未包含 Binaries、Saved、Intermediate、机器配置或录制。其他协作者需执行 `git lfs pull`；继续在匹配的 UE 5.6 中打开项目。长期运行表现及逐场景视觉效果不由短时冒烟验证覆盖。
