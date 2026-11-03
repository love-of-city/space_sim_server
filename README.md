# SpaceSim

太空机械臂遥操作、权威动力学、UE 渲染与训练数据采集的统一项目。服务端与渲染适配器在同一个 Git 仓库维护。

| 目录 | 职责 |
| --- | --- |
| [server/](server/README.md) | 网页/API、仿真、原生加速、任务规划与采集 |
| [ue-adapter/](ue-adapter/README.md) | Python 协议适配、UE 工程、Runtime 插件和渲染资产 |
| [docs/](docs/README.md) | 项目架构、目录约定、操作与维护指南 |
| [tools/release/](tools/release/README.md) | 版本管理下的打包工具和应用启动器 |
| [manifests/](manifests/README.md) | 环境和资产的索引规范 |

## 场景与任务

平台支持直接机械臂操作、7 km 交会和 300 km 入轨后连续任务，共用原控制台与任务编号。远距、近距共享 GNSS 配置和在途观测，进入操作阶段时继承真实关节终态。近距视觉当前包含参数化观测与逻辑捕获，不代表真实图像闭环或物理锁定已验收，详见[交会接口](server/docs/NEAR_FIELD_RENDEZVOUS.md)。

双臂捕获场景、便携初始化快照、视觉标记与相机配置见[场景库](server/docs/SCENE_LIBRARY.md)；原操作场景的模拟陀螺观测见[服务星陀螺反馈](server/docs/SERVICE_GYROSCOPE.md)。原有十项自动采集任务（含四个单向任务）继续使用[采集功能基线](server/docs/COLLECTION_BASELINE.md)。动画预览时钟与物理推进分别计时，画面帧率不等同于动力学实时倍率。

## 开发入口

```bash
git clone https://github.com/mingti-org/SpaceSim.git space-sim
cd space-sim
git lfs install --local
git lfs pull
```

服务端命令从 server/ 执行。目录名称与 Python 包名独立，space_arm_platform 和 bsk_render_adapter 导入名称保持不变。

- [贡献与验证规范](CONTRIBUTING.md)
- [项目结构与代码职责](docs/project-map.md)
- [文件与目录命名规范](docs/naming-conventions.md)
- [Linux 部署](server/docs/LINUX_DEPLOYMENT.md)
- [Python/Basilisk 环境](server/docs/PYTHON_RUNTIME.md)
- [采集入口与布局顺序](server/docs/LAYOUT_SEQUENCE_COLLECTION.md)
- [当前采集功能基线](server/docs/COLLECTION_BASELINE.md)
- [仿真与采集全流程、参数和验收对齐](docs/guides/simulation-data-workflow.md)
- [UE 构建与验证](ue-adapter/README.md)

Python 环境必须包含匹配的 Basilisk/MuJoCo API 和原生 ABI；安装项目本身不会安装 UE。使用已有有效解释器安装两个包：

```bash
uv pip install --python "$SPACE_SIM_PYTHON" -e './ue-adapter[test]' -e './server[test,simulation]'
cd server
npm --prefix frontend ci
npm --prefix signalling ci
```

本机源码位于 /home/spacesim-zmh/src/space-sim。2026-10-31 工程改进已集成并在主目录完成独立五任务录制审核；当时三个旧批次已停止并保留。后续新批次单独记录，实时运行状态以仓库外 `WORKSPACE.md` 和各批次 `progress.json` 为准，详见[本机维护指南](docs/guides/linux-workspace.md)。固定布局独立验收不代表全部布局或训练资格已验证。

源码、测试、当前文档和配置模板进入 Git，交付模型与 UE 资产使用 LFS；数据、环境、运行状态与构建输出位于工作区或 Git 忽略目录。见[目录规范](docs/project-layout.md)。基础 CI 不等于完整 UE/GPU、真实动力学或采集验收。
