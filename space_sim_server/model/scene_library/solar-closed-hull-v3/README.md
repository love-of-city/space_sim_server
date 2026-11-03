# 完整太阳翼与按种子双任务盒版本

本目录保存 2026-11-03 已运行版本使用的固定星体基线。目标星完整太阳翼层级保留，
随箱体统一放大 4.7 倍；铰链角度和结构不变。外壳封闭，任务盒开口保留。
任务盒安装位姿、两星构型、双臂初态和相机沿用本轮已核验配置。

这里的 `preheld-dual.xml` 是随机生成器的固定输入，不是随机种子 202 的最终模型。
启动时生成器会替换两盒内部布局，并给两侧补齐三个插头和螺栓。
相同种子和输入版本得到相同布局，不使用 A/B 互换。
旧 `original-root-v2` 快照继续保留，供历史复现与装配构建使用。

## 在新机器准备

先在仓库根目录执行 `git lfs pull`，再进入 `space_sim_server`。
需要已构建的 UE 适配器，以及分别包含几何建模依赖和 Basilisk 的 Python 环境。
几何环境需要 NumPy、SciPy、trimesh、manifold3d；基础项目测试环境不替代它。

```text
python tools/prepare_saved_seeded_scene.py --output <新的运行目录> --adapter <适配器根目录> --unreal <UE安装根目录> --geometry-python <几何环境python> --platform-python <Basilisk环境python>
```

入口校验 1,629 个依赖，将模型路径迁移到本机，准备八个任务索引和绑定输入哈希的生成配置。
它不替换当前平台索引，也不执行 UE 导入。`catalog.json` 提供视觉源网格、来源哈希和
UE 资源路径；在相应 UE 工程导入并核验这些基线资产后，备份现有索引，再将输出的
`imported_scene_library.json` 安装到服务端 `run/imported_scene_library.json`。
生成的运行目录应保留，索引会引用它。此索引包含第 2、4 类八个任务；其他任务索引应合并保留。

网页选择相应任务默认开启随机布局；种子留空时生成并保存新种子，指定 `202` 可复现本轮
验证使用的布局采样。第一次需要生成开孔、碰撞网格并导入 UE，已有输入缓存可复用。
“复原初始位置”保持本实例的布局。更换布局需要停止后换种子或清空种子再启动。

仅需要重定位固定输入时，可使用 `tools/prepare_saved_model.py --package
model/scene_library/solar-closed-hull-v3 --output <新目录>`；该入口不登记随机生成器。

## 验证边界

`source-assembly-audit.json` 记录源模型的太阳翼、双臂、相机和封闭外壳检查。
`linux-sampler-comparison.json` 记录与 `zmh_linux` 提交 `a489198` 的六组精确采样对照。
`seed-202-live-verification.json` 记录本轮实际模型运行至 35.02 秒，四路 UE 新帧匹配
当前实例，六个插头均保持于正确星体的初始座。完整作业轨迹与 Linux 原生运行未验收。
本目录资产路径重写会改变 XML 哈希；原始运行模型和源基线哈希均单独保留。
不包含本机环境、认证、运行日志、采集数据或编译后的 UE 资产。
