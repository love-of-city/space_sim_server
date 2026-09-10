# 原始三角网格高精度碰撞转换试验

日期：2026-09-09。**曾试用为默认，现因明显卡顿按用户要求退出默认；模型与工具完整保留，仅供手动实验，不是已验收的工程碰撞模型。**

运行模板为 `sarm-ground-validation-mesh-grasp`，组合入口为本目录 `sarm_mesh_collision.xml`。
当前默认是粗碰撞内部接触版 `sarm-ground-validation-self-collision-grasp` / `model/SARM/platform/sarm_ground_target_self_collision.xml`。
本试验依赖的旧 `sarm_ground_target.xml` 没有改动，因此原始转换及验证指纹保持有效。
本目录版本只有显式选择高精度模板才加载；已保存高精度实例不自动迁移，需新建“粗碰撞体·内部碰撞”实例切换到当前默认。主平台未被自动重启。
启用及回退只改变运行选择，没有更改本目录 XML/网格，也没有删除试验数据。以下转换结果及 `manifest.json` / `validation_report.json`
中 `default_scene_changed: false` 等值是**切换前试验的历史记录**，不是当前启动开关。
当前入口、使用步骤与切换验证见 `docs/GROUND_CAPTURE_TARGET.md`（仓库根目录）。

## 结论

**原始 mesh 的非凸三角面可以在当前 BSK/MJScene 中用于刚性接触，而不必变成三个粗盒子或整件凸包。**
本次采用当前原生 MuJoCo **3.7.0** 支持的 `flexcomp type="mesh" dim="2" rigid="true"`。
所有顶点绑定所属刚体，保持原始几何及位姿；不会将卫星变成软体，也没有逐顶点增加运动自由度。

但转换保真不等于机构物理正确。原模型铰链为预览用表面切分，零位存在接触/干涉；
全分辨率接触也很慢，因此现已退出默认；**只能作为手动实验使用，不能视为正常实时抓取模型**。

## 文件

- `satellite_mesh_collision.xml`：独立待抓取卫星实验，1 个 freejoint + 1 个被动 hinge；无驱动、零重力。
- `sarm_mesh_collision.xml`：包含原 SARM/轮/机械臂/相机的组合实验候选。
- `attitude_control.json`：逐内容复制原 SARM 控制配置，未重新调参，供原生组合测试读取。
- `manifest.json`：每个源 OBJ 的 SHA-256、转换位姿、三角面数量、清理索引和数值误差。
- `cleaned_meshes/`：只有 3 个导入时存在退化面的网格生成了碰撞专用副本，原文件不修改。
- `validation_report.json`：本机真实运行结果（时间数字是局部性能测量，不是跨机器承诺）。
- 生成器：`tools/build_satellite_mesh_collision.py`。
- 验证器：`tools/validate_satellite_mesh_collision.py`。
- 回归测试：`tests/test_mesh_collision_conversion.py`。

## 几何精度：测量了什么

| 指标 | 测量结果 |
| --- | ---: |
| 目标原始可视/碰撞表面实例 | 208 |
| 唯一源网格文件 | 104 |
| 原始三角面实例（含重复装配件） | 1,695,101 |
| 清理后的碰撞三角面实例 | 1,694,919 |
| 数值退化面清理 | 182 个三角面实例 |
| 清理前原面积总损失 | 约 3.70 × 10⁻¹⁶ m² |
| **每个保留三角形顶点**对比原始 OBJ 的最大位置差 | **约 1.034 × 10⁻⁸ m = 0.00001034 mm** |
| 接触表面半径 | 每面 0.00001 m = 0.01 mm；两面合计 0.02 mm |
| 额外减面／凸包化／补洞／人为缩小 | 无 |

这里的微小位置误差是**本次 OBJ → 碰撞三角面转换误差**，不是卫星制造精度，
也不是原始 STEP → 网格误差。此前 CAD 转网格使用的线性偏差设置为 **0.25 mm**，
不能把本次 10⁻⁸ m 量级的复核结果宣传成整个 CAD 模型达到纳米精度。

清理策略：MuJoCo 的 OBJ 导入使用 float32 顶点，原铰链表面裁分包含重复/共线顶点，
某些三角形在此精度下成为零面积，不能作为 flex 单元编译。
只删除这些**导入后严格零面积**的面，不进行一般小面简化；保留所有原坐标，逐面记录清理索引及原面积。
生成器在总面积损失或坐标量化误差过大时拒绝继续。

`dim=2` 表达原始三角形表面与显式接触厚度，不是精确实体布尔运算，
不保证任意大时间步／高速穿透后的体积恢复。切开的铰链网格没有封口，本次没有擅自补洞。

## 碰撞规则

- 原 208 个 visual mesh 仍只渲染，碰撞发生在与其形状对应的刚性三角 flex 上。
- **候选里移除三个粗盒子以及整对目标刚体的碰撞排除规则。**
- 固定本体 + 固定内侧板为同一刚性装配，彼此不建立内部接触；外侧板与固定部分启用接触。
- 固定/移动铰链表面和随动附件**全部纳入**，没有为了让测试通过而删除这些表面或关闭这些接触。
- 与原机械臂碰撞 mask 兼容；原主星控制、8 路机械臂控制数组不改变。
- 质量仍为原候选的合成 8+1+1 kg，惯量仍未标定；不会因使用真实表面而自动成为真实质量参数。
- 真实角度限位未知，仍不增加虚假限位。

## 实际接触与原生验证

1. 编译后的 208 份刚性表面逐三角形与原 OBJ 比较，1,694,919 个保留面全部检查，不是仅检查包围盒或稀疏抽样。
2. 独立模型 `nq=8 / nv=7 / nu=0 / nbody=4（含 world）`，没有引入柔性自由度。
3. 非凸开孔单元测试：小球在开口中心不接触，在实际材料处产生有限非零接触力，验证没有被凸包填孔。
4. 0°、30°、90°、135°、180°、190°、200°、225°、270°、360° 静态角度扫描全部保持有限状态；
   例如 200° 时输出 4,669 条接触，其中 2,619 条属于本体—外侧板，不再是原粗盒场景的零接触。
   **此扫描直接设置姿态，验证重叠时的接触检测，不是翼板从零位连续无穿透旋转的证明。**
5. 独立模型 20 ms 短时推进状态有限、无 MuJoCo warning；存在初始接触干扰，不能据此称其机构正确。
6. **真实 Basilisk/MJScene** 加载组合候选并推进 10 ms：15 个 body、26 项 qpos、8 路机械臂控制、姿态保持启用，状态有限。
   与 Python MuJoCo 测试进程分开运行，未修改用户的 Basilisk 安装。

## 已发现、尚未解决的阻碍

### A. 原预览铰链不是真实可接触机构

零位已输出 **200 条接触约束**，集中在四对表面：

- `instance_0196_hinge_moving_visual` ↔ `instance_0196_hinge_fixed_visual`
- `instance_0197_hinge_moving_visual` ↔ `instance_0197_hinge_fixed_visual`
- `instance_0208_part_070_color_00` ↔ `instance_0196_hinge_fixed_visual`
- `instance_0211_part_070_color_00` ↔ `instance_0197_hinge_fixed_visual`

这是求解器报告的接触数，不应当作所有几何接触点的穷举计数。
原 `wing_articulation/README.md` 已说明：STEP 中铰链是合并实体，固定/随动表面只是沿轴平面做的显示分区，
没有分别还原固定叶片、活动叶片和轴销；原装配中的附件也可能有配合或干涉。

将碰撞表面半径从 10 µm 临时降到 0.1 µm 后，零位仍报告同样四对表面的接触。
短时原生推进中，初始 `+0.1 rad/s` 变成约 `-0.06575 rad/s`，说明这些接触会干扰本应可动的铰链。
这不是把网格导入精度再提高就能解决的；需要核对真实机构分件、配合间隙以及附件固定/移动归属。
本次没有通过 blanket exclude、虚假限位或移动网格来掩盖问题。

### B. 目前不适合实时 500 Hz

- 本机独立全模型零位一次 `mj_forward` 约 0.18 s；严重重叠姿态约 8～10 s。
- 独立模型 20 ms 物理时间耗时约 1.53 s。
- 含原 SARM 控制的实际 BSK 短测，10 ms 物理时间耗时约 4.10 s。

以上是具体测试的墙钟时间，包含当前求解器和原始网格接触开销，不是所有场景的固定性能。
以上转换试验当时没有测试完整浏览器视频、UE flex 显示、长时间稳定性、RGB/depth/segmentation 采集或高速碰撞。
后续切换联调已做真实协议短测、UE 可视几何回放和两路 RGB 首帧输出；RGB 冷启动首帧偏暗，未验收图像质量。
当前仍未完成浏览器、长时或高速碰撞验收；详见仓库 `docs/GROUND_CAPTURE_TARGET.md` 的切换验证范围。

## 复现

从服务端 Git 根目录运行，**单独创建环境，不升级生产 Basilisk 或原 CAD 转换环境**：

```powershell
python -m venv .venv-mesh-collision
.\.venv-mesh-collision\Scripts\python.exe -m pip install -r tools/requirements-mesh-collision.txt
.\.venv-mesh-collision\Scripts\python.exe tools/build_satellite_mesh_collision.py
.\.venv-mesh-collision\Scripts\python.exe tools/build_satellite_mesh_collision.py --check
.\.venv-mesh-collision\Scripts\python.exe -m pytest tests/test_mesh_collision_conversion.py -q
.\.venv-mesh-collision\Scripts\python.exe tools/validate_satellite_mesh_collision.py --native-python <Basilisk环境的python.exe路径>
```

本机独立环境：外层 `run/mesh-collision-venv`。详细过程日志在服务端 `run/mesh-collision-20260909/`。
转换试验及本次入口切换均未覆盖原始 STEP/OBJ/XML 或原组合 XML，也没有重启主服务；没有提交或推送。
当前新建场景默认模板为粗碰撞内部接触版；本目录的高精度实验模型和既有实例身份保留。

**下一阶段**应先确认真实铰链分件/配合，再在可量化几何误差约束下做空间裁剪、碰撞分区或局部精度优化。
任何精度下降或局部接触过滤都应明确列出并验证，不能静默回退为粗盒模型。
