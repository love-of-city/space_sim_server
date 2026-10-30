# 清单与来源

- environments/：本轮Python实际依赖；pip清单不能单独重建UE、GPU或原生ABI。
- assets/：资产角色索引；哈希以server/config/collection-base-assets.json和布局池清单为准。

数据文件清单、停止进度在仓库外维护快照。发布manifest记录输入、平台、源码/构建指纹、产物SHA和验收范围。不把机器私有配置或密钥写进共享manifest。
