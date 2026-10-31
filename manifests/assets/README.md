# 资产角色

模型源在server/model，UE交付资源在ue-adapter/Unreal/BskUnrealRenderer/Content。原CAD、网格、源纹理和许可保留。

通用采集资源入口为server/config/collection-base-assets.json。本机当前布局池data/layout-pools/random150-20261028的pool.json固定模型、catalog、几何审核和manifest哈希。

Random150、Collection、ClearanceColor、RuntimeDelivery家族属于本机生成布局，精确忽略；已跟踪通用资产继续LFS交付。禁止整体忽略或删除Content/BSK/Generated。

## 显式 Cook 输入

`ue-cook-base-assets.json` 声明基础 UE 资产文件 SHA、Cook 根和动态加载资源；527 个基础二进制资产从当前已跟踪内容盘点生成。`ue-adapter/scripts/prepare_cook_staging.py` 只复制声明资产，校验哈希、LFS 完整性和路径边界，另记录候选源码身份。布局包按 `ue-cook-inputs/1` 提供额外 `files`、`cook_directories`、`runtime_assets`。资产变更必须更新相应 SHA，并通过真正 Cook 和 packaged 加载验收。清单存在本身不表示依赖闭包已通过验收。
