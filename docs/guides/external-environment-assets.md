# 外部地球环境资产

IncredibleEarth80K 原始资源单独传输，代码仓库只记录版本与逐文件 SHA256 清单。当前包包含 **108 个文件、17,901,852,851 字节（约 16.67 GiB）**。源文件来自迁移提交 `8a3d9dfc95643efad72adf78bc9cb45ee85193e5`，清单版本为 `8a3d9dfc9564`；哈希和长度从该提交的 Git LFS 指针提取。

外部目录是 `ue-adapter/Unreal/BskUnrealRenderer/Content/IncredibleEarth/`。项目关卡 `Content/BSK/Maps/BskEnvironment.umap`、`Content/BSK/Environment/IncredibleEarth/` 中的小型适配材质及代码仍在 Git/LFS 中。不要忽略整个 `Content/` 或所有 `.uasset`，也不要把忽略规则当作删除本地资源的操作。原始资源继续适用其原许可，单独资产包只分发给具备相应使用权的接收者。

`git clone` 和 `git lfs pull` 只会取得仍受 Git/LFS 管理的文件，**不会下载这份外部资产包**。缺少外包时不能正常显示迁移后的默认地球关卡。先安装外包再打开 UE、执行 Cook 或同步部署。

## 检查已安装资源

以下命令从新版仓库根目录执行，Python 3.11+ 即可，无需 UE Python。`python` 应换成当前可用解释器。

```powershell
python tools/release/external_environment_assets.py verify
python tools/release/external_environment_assets.py verify --full
```

默认只核对文件存在、字节数及是否仍是 LFS 指针，适合快速排查缺包；`--full` 读取约 18 GB 数据并校验全部 SHA256，可发现长度未改变的内容修改。缺失、版本不匹配或校验失败会返回非零退出码。

清单位于 `manifests/external-assets/incredible-earth.json`。包版本、完整性校验和视觉验收是不同的检查；哈希通过不代表当前 UE 构建或 GPU 采集已通过验收。

## 从现有本机资源打包

`pack` 只读取清单列出的 108 个源文件，不附带缓存、插件二进制或其他本地资源。输出必须位于仓库外，目标目录须事先存在，且不得覆盖已有包或校验文件。

```powershell
python tools/release/external_environment_assets.py pack --output E:/asset-delivery/incredible-earth80k-8a3d9dfc9564.zip
```

如果资源仍在旧目录结构，显式传入实际资源目录，不需要先复制 18 GB：

```powershell
python tools/release/external_environment_assets.py pack --content-root E:/mujoco_demo/space_arm_data_platform/space_sim_UE_Adapter/Unreal/BskUnrealRenderer/Content/IncredibleEarth --output E:/asset-delivery/incredible-earth80k-8a3d9dfc9564.zip
```

输出为 ZIP64 包和同名 `.zip.sha256` 文件；打包流式校验每个资源，再计算完整 ZIP 的 SHA256。ZIP 使用不压缩模式，避免重复压缩 UE 资源，因此文件约 18 GB，打包目标盘须有相应空闲空间。包内统一使用**新版仓库根目录**下的 `ue-adapter/Unreal/BskUnrealRenderer/Content/IncredibleEarth/...` 路径，并包含同版本的清单。打包时应关闭资源保存/编辑操作，源文件若已修改会被校验拒绝。

工具不会自动下载或上传，也不会因为运行 `verify` 自动生成包。传输 ZIP 和 SHA256 文件后，可用 `Get-FileHash -Algorithm SHA256 <zip>` 或 Linux `sha256sum <zip>` 对照传输前哈希。

## 在另一台电脑恢复

先获取匹配版本代码并运行 `git lfs pull`，随后从新版仓库根执行：

```powershell
python tools/release/external_environment_assets.py restore --archive E:/asset-delivery/incredible-earth80k-8a3d9dfc9564.zip
python tools/release/external_environment_assets.py verify --full
```

也可以通过 `--repository-root <仓库目录>` 指定接收位置。`restore` 以当前代码附带的可信清单核对包内清单和每个文件的 SHA256，拒绝额外文件、重复文件、路径穿越和符号链接。它会保留相同文件；若目标中已有不同内容则报错，**不会覆盖本机编辑成果**。先把需要保留的不同版本另存到仓库外，再用匹配版本恢复。不存在自动强制覆盖参数。

全部 ZIP 内容通过验证后才安装缺失文件。暂存位于目标资源目录所在磁盘，因此首次恢复约需 18 GB 空间；安装完成后临时目录删除。进程中断可能留下临时目录或已成功安装的部分文件，重新运行会校验并复用相同文件；磁盘错误留下的不同文件仍需人工保全和处理。

恢复后，Windows/Linux 各自按适配器构建指南编译 UE 插件，不共享另一平台的 `Binaries/` 或 `Intermediate/`。随后打开项目关卡检查地球、云层、夜景、极光和月球，再进行后端接管与采集验收。

## 同步到部署工程

完整资源先安装到源码 UE 工程，再使用平台现有的 `server/tools/deployment/sync_unreal_runtime.py` 流程同步运行副本。同步读取磁盘上的 `Content/`，资源是否被 Git 忽略不影响文件可用性；运行前应检查该脚本 `--help` 和[环境部署说明](../../server/docs/environment-deployment.md)。不要仅用 `git archive` 或只复制 Git 跟踪文件的方法制作包含地球的部署副本。

对于直接维护的部署工程，也可把资源显式恢复到其 `Content/IncredibleEarth/`：

```bash
python tools/release/external_environment_assets.py restore \
  --archive /path/to/incredible-earth80k-8a3d9dfc9564.zip \
  --content-root /home/lyh/space_enviroment/runtime/ue_adapter/Unreal/BskUnrealRenderer/Content/IncredibleEarth
```

仓库里的资产工具和清单应与部署代码使用同一版本。项目启动检查、Cook 输入清单与真正打包/GPU验证仍按对应部署和发布流程执行；资源包不包含源码编译产物。

## 维护与轻量测试

以后更改外部资源时应创建新包版本、更新逐文件长度/SHA256 和项目依赖，不覆盖旧版包。清单不应包含机器绝对路径或授权凭据。

```powershell
python -m pytest tools/release/tests/test_external_environment_assets.py -q
```

测试使用小型合成文件覆盖打包/恢复、SHA256、拒绝覆盖、损坏包、路径穿越和 ZIP64；不运行 UE，也不重打包真实的 18 GB 资源。
