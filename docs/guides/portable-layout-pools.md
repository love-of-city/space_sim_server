# 可搬迁布局包

新建交付使用 `collection-layout-pool/2`；已有 v1 冻结批次保持原文件和原路径。迁移只输出到新目录，不修改原布局、不重新采样、不重跑或伪造历史 native 验收。

在 `server/` 执行：

```bash
python tools/assets/migrate_layout_pool.py \
  --source /absolute/input/pool.json \
  --output /absolute/new-layout-package \
  --asset-root /absolute/space-sim/server/model \
  --ue-content /absolute/space-sim/ue-adapter/Unreal/BskUnrealRenderer/Content
```

输入池目录自动作为允许读取的资产根；其他源资产必须显式列入 `--asset-root`。拒绝越界和软链接逃逸。输出已存在时拒绝覆盖。失败产生 `migration-failed.json`，不能作为发布包使用。

包内目录：

- `pool.json`：版本化入口、根 `.`、各布局相对路径与完整文件哈希。
- `layout-package.json`：包根标识与原池 SHA。
- `layouts/<index>/layout.xml`、`layout.layout.json`、`catalog.json`：迁移后的模型、布局和渲染映射；运行所需路径均可按包根解析。
- `layouts/<index>/original-*`、`native-geometry.json`：原始模型、manifest、catalog 和验收报告，保留原字节。绝对路径在此仅为历史证据。
- `assets/<sha256>/<filename>`：声明的网格、纹理、源模型和其他输入；同内容复用。
- `ue-content/`：catalog 引用的 UE 资源，保持 `/Game/` 路径。基础材质、Shader、插件及资源依赖仍由正式 Cook 清单提供；迁移包不自行充当可执行 UE 发布包。
- `migration-report.json`：源→目标映射、输入/输出哈希及验收范围。

XML 只改资产路径。迁移与加载均比较完整 XML 结构和所引用资产的 SHA；原验收报告继续绑定原 XML 哈希。新模型哈希变化由独立 relocation 证明解释，不能直接把旧报告中的 `model_sha256` 改成新值。包复制到其他根目录后，pool、模型、catalog 和资产字节及哈希保持不变。

运行时 `LayoutPool` 同时读取 v1/v2。v2 完整检查文件 manifest、原报告、迁移证明和布局几何声明。`load_asset_catalog` 将相对源资产映射解析为实际本机文件，用于 UE 资源查找。模型的 mesh/texture 路径同样只指向包内资产。

采集器的 `write_snapshot` 对 v1 保持原快照格式；对 v2 写 `collection-layout-pool-reference/1`，冻结本机包位置和原池 SHA。它是批次来源索引，不是独立资产包；跨机器交付必须复制完整包目录。旧批次不会因为新格式自动改写。
