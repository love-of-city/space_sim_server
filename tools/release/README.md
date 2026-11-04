# 发布工具

发布源码从旧 releases 工作目录纳入版本管理。发布包和构建目录位于仓库外，使用显式输入/输出路径，拒绝覆盖已有正式包。

| 目录 | 内容 |
| --- | --- |
| layout_preparer/ | 布局工具应用、运行环境复制、系统库收集、基础包构建 |
| collector/ | 采集启动应用、公共运行时叠加包构建 |
| tests/ | 启动器、自解压、打包完整性测试；显式GPU验收脚本 |

## 构建基础运行时

`layout_preparer/copy-runtime.py --help` 列出必须提供的环境路径。`--bundle` 必须是空的暂存目录，名称为 space-sim-layout-preparer；需要现成的Python 3.13运行环境、已编译的项目原生库/UE插件、完整LFS资产、rsync和UE。

```bash
python tools/release/layout_preparer/copy-runtime.py \
  --bundle /path/to/artifacts/build/example/space-sim-layout-preparer \
  --repo /path/to/space-sim \
  --python-runtime /path/to/python-runtime \
  --simenv /path/to/simenv --geometryenv /path/to/geometryenv \
  --unreal /path/to/UE_5.6
python tools/release/layout_preparer/collect-system-libs.py \
  --bundle /path/to/artifacts/build/example/space-sim-layout-preparer
python tools/release/layout_preparer/build-release.py \
  --bundle /path/to/artifacts/build/example/space-sim-layout-preparer \
  --output /path/to/releases/layout-preparer/new-version
```

运行时复制不是编译器，也不会自动证明源代码与本机二进制匹配。构建前按子项目指南完成原生/UE编译和验收。当前收集器针对Linux x86_64、Python 3.13和现有ABI；换平台须另行适配。

## Collector 叠加包

先准备一个包含 `app/collection_cli.py`、`space-sim-collect`、所需ffmpeg及系统依赖/许可的overlay目录。应用源位于collector/，原子交付所需的系统二进制需按目标环境准备；不要把旧发布包的验收当成新overlay的验收。

```bash
python tools/release/collector/build-collector.py \
  --base-release /path/to/releases/layout-preparer/base-version \
  --overlay /path/to/artifacts/build/collector/space-sim-layout-preparer \
  --output /path/to/releases/collector/new-version
```

新版应用使用repo/server和repo/ue-adapter布局，因此base-release必须由当前布局的构建工具产生，不能与2026-10-28旧内部布局的包混用。

基础payload与overlay共同写入安装包，验证公共payload的SHA，发布manifest记录输入payload和输出SHA。新包不可覆盖旧包。

## 验证范围

从仓库根运行：

```bash
PYTHONPATH=server:server/backend python -m pytest tools/release/tests -q
```

轻量测试检查启动参数、取消/恢复、目录锁、真实打包/叠加、带空格路径、拒绝覆盖及损坏校验。`test_self_extractor.py` 在收集时执行独立自解压检查。

`verify_release.py`、`verify_artifact.py` 等是历史GPU集成验收程序，须显式提供 `--release-root`；`short_native.py` 须指定 `--bundle`。它们包含原验收场景/断言，运行前按新版本审阅，不在基础测试中自动启动仿真。

本轮纳管并验证了源码和轻量打包流程，没有重新构建几十GiB的真实运行时，也没有新一轮UE/GPU发布验收。保留的两个旧安装包已完整校验SHA，仍标记为2026-10-28历史发布物。

## 当前构建与验收入口

UE 的显式资产 staging、Cook 及 packaged 资源加载使用 `ue-adapter/scripts/package_ue_project.py`；操作流程见 [发布重建](../../docs/guides/release-reproduction.md)。基础Python/native/UE/GPU检查的边界见 [UE分层验证](../../docs/guides/ue-validation.md)。

`runtime_metadata.py` 实际读取Python/Engine/插件与ELF符号依赖；`copy-runtime.py` 在复制前验证解释器ABI，并写入 `RUNTIME-INPUTS.json`，BUILD信息不再硬编码版本。`resolve_environment.py` 只在新目录解析/安装候选锁，不改现有环境。元数据读取、单元测试和哈希通过均不等于新运行包已完成跨机或GPU验收。

外部地球资源由 `external_environment_assets.py` 校验、打包和恢复，使用[外部环境资产指南](../../docs/guides/external-environment-assets.md)。它不自动上传资源。
