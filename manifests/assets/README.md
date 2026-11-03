# 资产角色

模型源在server/model，UE交付资源在ue-adapter/Unreal/BskUnrealRenderer/Content。原CAD、网格、源纹理和许可保留。

通用采集资源入口为server/config/collection-base-assets.json。本机当前布局池data/layout-pools/random150-20261028的pool.json固定模型、catalog、几何审核和manifest哈希。

Random150、Collection、ClearanceColor、RuntimeDelivery家族属于本机生成布局，精确忽略；已跟踪通用资产继续LFS交付。禁止整体忽略或删除Content/BSK/Generated。

## 显式 Cook 输入

`ue-cook-base-assets.json` 声明基础 UE 资产文件 SHA、Cook 根和动态加载资源；565 个基础二进制资产从当前已跟踪内容盘点生成。`ue-adapter/scripts/prepare_cook_staging.py` 只复制声明资产，校验哈希、LFS 完整性和路径边界，另记录候选源码身份。布局包按 `ue-cook-inputs/1` 提供额外 `files`、`cook_directories`、`runtime_assets`。资产变更必须更新相应 SHA，并通过真正 Cook 和 packaged 加载验收。清单存在本身不表示依赖闭包已通过验收。

打包仅在独立 staging 使用 `-cookall`，确保清单中的根目录材质也参与 Cook；开发工作区中未声明的布局不会被复制进去。每个 `.uasset` 默认检查同名运行时对象；Blueprint 通过 `runtime_objects` 按资产路径显式指定生成类 `*.BP_Name_C`，因为打包时编辑器用的 `UBlueprint` 对象会被剥离。加载报告必须覆盖全部预期对象，无缺项、重复项或失败项。包内校验使用 `-NullRHI -nosound`，并记录 `.pak`、`.utoc`、`.ucas` 的实际哈希；这不等同于 GPU 采集验收。
