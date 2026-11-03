# 独立发布重建与资产验证

发布从声明输入生成新的 staging，不能直接打包开发工程中全部 `Content/BSK`。基础输入清单位于 `manifests/assets/ue-cook-base-assets.json`，包含 565 个基础资产文件及 SHA-256、明确 Cook 根、动态加载资源。新布局必须以同 schema 的额外 manifest 显式声明；未声明的 Random150/Collection 本地资产不会进入 staging。

## Cook 与独立运行验证

从仓库根执行；staging 和 output 都必须为空且相互独立：

```bash
"$SPACE_SIM_PYTHON" ue-adapter/scripts/package_ue_project.py \
  --engine "$UE56_ROOT" \
  --staging /absolute/artifacts/build/build-id/BskUnrealRenderer \
  --output /absolute/artifacts/verification/build-id/package
```

Windows 可使用 `ue-adapter/scripts/package.ps1 -UnrealRoot ... -StagingDirectory ... -OutputDirectory ...`；底层使用同一 Python 工具。自定义 `--manifest` 时必须同时显式传基础清单及布局清单，不能只传增量清单而意外省略基础资产。

工具按哈希校验输入及 Git LFS 内容，复制当前候选源码/配置；Git 忽略的生成目录不复制。所有未忽略的新源码以及工作区删除均反映进 staging 的源码指纹。目录越界、软链接逃逸、非空输出或损坏资产会失败。相同输入重复 staging 的资产集合与文件指纹应一致；并不保证 UE Cook 包逐字节完全相同。

BuildCookRun 在隔离 staging 使用 `-cookall`，完成后生成 Game 构建身份，启动 packaged Development 程序，使用 `-BskValidateCookAssets` 对全部声明 `.uasset` 的运行时对象及 CelestialVault 动态资源逐项 LoadObject；Blueprint 显式映射到打包后保留的生成类。结果在 `runtime-validation/runtime-asset-validation.json`，必须 `passed=true` 且完整覆盖预期对象；完整 package 证据另记录构建清单、实际 Game 二进制、Pak/IoStore 容器和运行报告哈希。此步使用 `-NullRHI -nosound`，仅证明包内资源可加载。GPU 图像、完整任务、跨机启动和长稳结果仍须分别验收。

## Python 与引擎来源

`tools/release/runtime_metadata.py` 从实际输入解释器获取版本、ABI、安装包，并读取 Engine Build.version 和关键插件信息。复制运行环境时生成 `RUNTIME-INPUTS.json`；Linux glibc 最低需求来自交付 ELF 的版本需要表，不是硬编码或构建机 glibc 版本。完整运行包 SHA 证明文件完整性，不能代替干净环境重建或 GPU 验收。

生成完整哈希锁并验证新的隔离环境：

```bash
python tools/release/resolve_environment.py --platform linux \
  --python /absolute/python3.13 \
  --output /absolute/artifacts/verification/environment-id --install
```

输出必须为空。可用 `--constraint` 约束历史版本、`--offline` 使用已完整缓存的依赖，或用 `--index` 指定明确的包来源。私有/本地编译 wheel 应进入受控依赖源并记录 SHA；CPU PyTorch 等特殊分发源也必须显式配置。解析成功但未安装仅为候选 lock；安装、依赖一致性和 Basilisk 能力检查通过后，`environment-validation.json` 才记录 clean_install 通过。该状态仍不等同 native 数值、UE/GPU或整机发布认证。

2026-10-31本次实现时，已有 Linux 环境仅完成来源读取；离线缓存不完整，在线解析未完成。没有把旧 freeze 文本改名成已认证 lock。Windows 未进行重装认证。`manifests/environments/*-runtime.json` 明确保留这些状态，后续取得真实证据后再更新。

## 证据保留

保存源码/dirty 指纹、组件 manifest、环境验证报告、Cook 输入、包运行报告、完整双相机与长稳结果。正式 release 目录不可覆盖。当前两个旧安装包仍是历史发布物；新工具的单元测试或 Editor 构建不能替它们重新授予认证。
