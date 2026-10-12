# 新版 SARM 资源导入记录

主模型：`SARM_convex.xml`。`SARM.xml` 保留为随包的简化版本，二者不等价。

完整保留 2026-10-12 提供资源包的 1,293 个文件，逐文件 SHA-256 验证见 `import-manifest.json`。原 README 和控制脚本原样保留；README 的文件清单与实际交付存在差异，以 manifest 为准。

本阶段只完成资源导入，尚未切换平台活动场景，尚未进行 UE 资产导入、平台控制接口适配及在轨装配合成。

- STL 沿用服务端 .gitattributes 的 Git LFS 规则。
- 随包 Linux .so 在本地保留，但作为构建产物忽略；Windows DLL 需针对平台运行库编译。
- 抓夹、任务盒、机械臂质量和惯量以新版主模型为后续接入来源。
- 新模型为固定基座独立场景，不能直接覆盖当前在轨模型。
- 先前独立 MuJoCo 3.7.0 测试出现插头 QACC 不稳定；完整旋拧和机械臂操作尚未验收。
- 控制脚本可选模式依赖的 bolt_screw_assist / fastener_locking_assist 不在交付包内。
