# 资产角色

模型源在server/model，UE交付资源在ue-adapter/Unreal/BskUnrealRenderer/Content。原CAD、网格、源纹理和许可保留。

通用采集资源入口为server/config/collection-base-assets.json。本机当前布局池data/layout-pools/random150-20261028的pool.json固定模型、catalog、几何审核和manifest哈希。

Random150、Collection、ClearanceColor、RuntimeDelivery家族属于本机生成布局，精确忽略；已跟踪通用资产继续LFS交付。禁止整体忽略或删除Content/BSK/Generated。
