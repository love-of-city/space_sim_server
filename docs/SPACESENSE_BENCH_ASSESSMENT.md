# SpaceSense-Bench 下载与平台适配检查

检查日期：2026-09-08。本文基于实际下载、完整压缩包清单、抽样数据读取，以及本机服务端/UE 适配器代码与协议检查。

## 结论

**用户给出的 `raw/` 是离线传感器数据，不是可以直接装进仿真场景的航天器三维模型库。**

- 可用于目标检测、部件语义分割、深度/点云、位姿估计的离线训练与评测，但本平台尚未为它新增数据导入器。
- 不能直接替换 `model/SARM/platform/sarm_platform.xml` 内的 `capture_target`，也不能凭这些数据得到可信的质量、惯量、接触/关节模型。
- 另从 NASA 官方资源库下载了两个真正的 GLB 三维模型作为候选。这不是从 Hugging Face 压缩包里解出的模型，也未确认与数据集使用的版本完全一致。
- 本次未改动默认 SARM 场景、运行中的平台、UE Content 或已有用户修改；未执行 UE 导入、MJScene 编译、碰撞或抓取验收。

## 1. 已下载内容与校验

所有大文件和诊断产物保存在已有 Git 忽略目录 `run/spacesense-bench/`，没有把第三方数据加入 Git/LFS。

### Hugging Face

仓库：`Alvin16/SpaceSense-Bench`

固定版本：`90bf1da70fcc5fa2807248f15ac8084b146c663d`

`raw/` API 清单共 **136 个压缩包，73,868,846,508 字节（73.87 GB，约 68.80 GiB）**。
这是该目录当前可下载文件总和，不是历史存储量，也不是每颗卫星的数据量。未下载全部 136 包。

已下载：

| 内容 | 本地位置 | 校验 |
|---|---|---|
| 完整 Mars Express 包 | `run/spacesense-bench/raw/Mars_Express.tar.gz` | 479,855,173 字节；与 HF LFS SHA-256 一致 |
| 数据集说明与尺寸描述 | `run/spacesense-bench/README.md`、`satellite_descriptions.json` | 固定版本下载 |
| 目录和版本信息 | `raw_tree.json`、`root_tree.json`、`dataset_info.json` | API 快照 |
| 全包文件清单 | `archive_inventory.json` | 遍历完整 tar.gz，非仅检查文件头 |
| 抽样文件 | `sample/Mars_Express/` | 每条轨迹每种模态的首个可用文件，以及全部 27 份位姿 CSV |
| 预览 | `sample_contact_sheet.jpg` | 原图缩略图、分割、归一化深度、LiDAR Y-Z 投影 |

压缩包 SHA-256：

```text
c8d1304ba63e8d11b40ea5bd6e75515b3f3ea799929c9fa39a1cb72fd448c138
```

校验记录：`download_verified.json`。下载脚本：`download_sample.py`，支持在保留的 `.part` 文件上续传；它下载清单中最小的一个包，不是全量下载器。

### 完整包实查结果

2,589 个普通文件，展开文件内容总计 495,706,602 字节，27 条轨迹：

| 类型 | 数量 | 内容 |
|---|---:|---|
| PNG RGB | 668 | 1024×1024 图像 |
| PNG segmentation | 668 | 彩色语义掩码，不是可直接读取的类别整数图 |
| NPZ | 668 | `depth` 数组，1024×1024，int32 |
| ASC | 558 | 逗号分隔的 x,y,z 点云 |
| CSV | 27 | 位姿与时间戳 |
| OBJ/STL/FBX/GLB/glTF/Blend/UAsset/URDF/XML/PLY | **0** | 没有网格、材质资产包或动力学描述 |

这里只对 Mars Express 的内部文件做了完整检查；其余 135 包仅检查远端文件清单与统一数据说明，不能声称逐包验证。
官方 GitHub 的递归目录也已保存为 `github_tree.json`，所检查快照主要包含工具、示例和网页，没有发现上述三维源模型。

## 2. 数据兼容性与实际问题

### 可读取，但不能直接视为平台 Episode

- 实测 NPZ 的键为 `depth`，dtype 为 int32；官方定义单位为毫米，需乘 `0.001` 转为平台的米。
- 实测样本最大值 `10,000,000` mm，对应说明中的 10,000 m 背景值。训练前应把背景单独屏蔽，不应作为目标表面深度。
- 实测 ASC 使用逗号分隔，而不是空格。使用 `numpy.loadtxt(path, delimiter=',')`；一份 orbit_xy 抽样点云为 404×3。
- 按完整文件清单的时间戳与全部 CSV 匹配：RGB/深度/分割/位姿各有 668 帧；四种传感器数据加位姿的交集为 **558 帧**。
- **110 帧 RGB 没有同时间戳的 LiDAR 文件**，RGB 没有对应位姿的帧数为 0。这里只验证文件/CSV 时间戳匹配，不代表独立验证了物理时序准确性。
- `alignment_check.json` 保存逐轨迹计数；`sample_metrics.json` 保存读数组和对齐汇总。
- 预览的 approach 行首个 LiDAR 晚于首个图像，图内已标注；不要把该列当成同时刻投影。深度是仅供显示的局部归一化，不是原始灰度值。

### 坐标和协议不能照抄

官方说明使用 AirSim 世界 NED；相机坐标为 X 前、Y 右、Z 下；位姿四元数顺序为 wxyz；LiDAR 位于 SensorLocalFrame。

本平台 `bsk-render/2` 使用 SI 单位、右手局部渲染坐标 L、相对 `origin_N_m` 的位置及主动 parent-from-child 的 wxyz 四元数。
因此，同为 wxyz 不等于可以直接复制姿态；接入前必须核对旋转方向、相机/传感器外参和世界到 L 的变换。
目标在相机中的相对位姿不能当成世界/局部渲染系中的物体状态直接发送。

官方传感器配置是 1024×1024、FOV 50°；平台当前默认两台模型相机是 640×360，SARM XML 中腕部相机 fovy 为 60°。
数据加载或评测需要独立保存内参，不能沿用平台默认相机配置。
七类部件语义标签与平台实例分割也应建立显式映射，不应当作同一套 ID。

本次未转换成平台权威 Episode：该工作还需要帧 ID/时间基准、相机模型、标签定义与来源版本的适配。
这些静态观测也没有机械臂控制动作、接触力或完整状态历史，不能直接成为抓取强化学习交互环境。

### 版本与许可

下载的官方 README 记录：**2026-05-19** 对全部 136 颗卫星重生成位姿 CSV，修复与渲染帧的时序不对齐，并重上传 raw 包。本次固定的版本包含此说明，未另行叠加旧补丁 ZIP。

数据集声明为 **CC-BY-NC-4.0，非商业使用**。不能把它默认当成无使用限制的商业训练资产；保留来源与许可，具体用途需单独确认。
尺寸 JSON 只给出名称、描述和最大直径等信息，不能代替质量、质心、惯量、关节和器件参数。

## 3. 另外下载的真实三维候选

来源：NASA 官方 `nasa/NASA-3D-Resources` 仓库。
目录快照 ID：`11ebb4ee043715aefbba6aeec8a61746fad67fa7`。
每份下载均校验 Git blob SHA-1，并记录本地 SHA-256。记录在 `nasa_originals/download_verified.json`。

| 候选 | GLB 字节数 | 三角面数（按 accessor 统计） | 材质数 | 纹理 |
|---|---:|---:|---:|---|
| Hubble Space Telescope (A) | 1,694,988 | 7,672 | 5 | 5 个 texture 定义，内嵌资源 |
| Voyager Probe (A) | 285,936 | 29,911 | 16 | 无 texture 定义 |

路径：

```text
run/spacesense-bench/nasa_originals/3D Models/
  Hubble Space Telescope (A)/
    Hubble Space Telescope (A).glb
    Hubble Space Telescope (A).png
  Voyager Probe (A)/
    Voyager Probe (A).glb
    Voyager Probe (A).png
```

已验证 GLB 魔数/版本/声明长度、JSON 元数据、节点/网格/材质/纹理引用，并检查没有外部 URI 依赖。没有完成 Draco 几何解码，面数来自 accessor 声明，不是解码后的质量验收。

**两者都要求 `KHR_draco_mesh_compression`；哈勃还声明 `EXT_texture_webp`。**
转换工具需要对应解码能力。哈勃节点还有旋转和平移，转换时不能只拷贝局部顶点而丢弃场景节点变换。
两份文件均没有动画；本次检查不把视觉分组视为物理关节。

NASA 仓库 README 和使用指南入口已保留在 `nasa_originals/README.md`。NASA 资源的使用条件应与 SpaceSense 数据集许可分开核对，不能由其中一个推导另一个。

## 4. 针对当前平台的接入路线

| 目标 | 当前判断 | 需要的工作 |
|---|---|---|
| 用 raw 包训练/评测检测、分割、位姿 | 可作为离线数据来源 | 写数据适配器，处理单位、背景、标签、内外参及缺失 LiDAR |
| 用 raw 包直接加载一个 UE 航天器 Actor | 不可直接做 | 包里没有三维网格；点云重建是另一个任务，不能声称能恢复完整原始模型 |
| 用额外 NASA GLB 显示航天器 | 候选可行，尚未运行验证 | 解码/转换、节点变换、比例、材质和 UE 离线导入 |
| 用航天器作为自由漂浮刚体 | 需要补建 MJCF | 真实或明确标注的假设质量、质心、惯量；free joint；状态桥接 |
| 让 SARM 抓取/碰撞目标 | 不能只换视觉网格 | 简化/分解的碰撞几何、抓取接口、摩擦/接触参数、质量与惯量验证 |

建议从一颗 **被动自由目标** 开始，不替换现有带控制链路的 SARM 母平台：

1. 使用支持 Draco 的工具导出带正确节点变换的 OBJ/MTL 或适用格式，保留纹理和多材质；核对尺度、轴向、原点、法线。
2. 将视觉网格与碰撞网格分开；不能默认以高面数视觉模型直接承受接触求解。
3. 建立独立 MJCF：free joint、显式惯性参数、视觉 geom、碰撞 geom 和抓取 site。缺失物性时明确记录假设，不能从视觉外形自动宣称真实惯量。
4. 通过适配器 `prepare_mjcf_assets.ps1` 离线生成 UE `/Game` 资产和本机 catalog，再由 `bridge.add_mj_scene()` 使用 MJCF 与 catalog 注册。
5. 先做尺寸/坐标/材质检查和自由漂浮动力学验证，再做碰撞/抓取和权威图像对齐验证。

本机适配器文档明确：运行时 UE 只加载 `/Game` 软对象路径，不会直接读取任意磁盘 GLB/OBJ；目录缺失时可能回退为占位几何。
因此，把 GLB 放进 `model/` 并不等于已经接入。

## 5. 证据与复查入口

本地平台：

- `docs/SYSTEM_ARCHITECTURE.md`：唯一动力学、模型资产、UE 离线导入和扩展约定。
- `model/SARM/platform/sarm_platform.xml`：现有平台、目标和相机配置。
- `UE:Unreal/BskUnrealRenderer/docs/MJCF_MESHES.md`：资产准备、catalog、材质和 add_mj_scene。
- `UE:Unreal/BskUnrealRenderer/docs/PROTOCOL.md`：SI、L 坐标、旋转语义和相机协议。

远端入口（完整内容/清单已按上文保存）：

```text
https://huggingface.co/datasets/Alvin16/SpaceSense-Bench/tree/main/raw
https://huggingface.co/datasets/Alvin16/SpaceSense-Bench/blob/90bf1da70fcc5fa2807248f15ac8084b146c663d/README.md
https://github.com/wuaodi/SpaceSense-Bench
https://github.com/nasa/NASA-3D-Resources
```

范围限制：完整下载和遍历了 1/136 个数据包；NASA 模型完成结构检查，未做解码后网格、UE 渲染、MJScene 动力学或抓取验收。结论不把这些未执行步骤称为成功。
