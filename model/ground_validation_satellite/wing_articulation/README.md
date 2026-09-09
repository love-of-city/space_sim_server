# 外侧板单关节运动模型

## 使用哪个文件

- **MJCF：** `../ground_validation_satellite_articulated.xml`
- **离线可调预览：** `motion_preview.html`，用浏览器打开。
- **姿态对比：** `motion_overview.svg`。
- **循环动图：** `preview/motion.gif`。
- **构建和校验记录：** `build_manifest.json`。

模型需同时携带上一级的 `meshes/` 和 `articulated_meshes/` 目录；只拷贝 XML 不够。
原有 `ground_validation_satellite.xml`、`satellite_free_preview.xml`、STEP 与 SARM 场景均未修改。

## 实现范围

```text
world
└─ ground_validation_satellite    星体固定
   └─ satellite_inner_panel       内侧板固定，无关节
      └─ satellite_outer_panel    外侧板组件
         └─ outer_panel_hinge     一个被动旋转关节
```

- 外侧板及其安装件归入活动 body；上下两处同轴铰链共用一个转动自由度。
- 外侧板远端的 **解锁器02** 是随板移动的附件，不增加内部运动；01/03 仍固定于内侧板。
- 自拍相机、分离接口等其他组件不新增自由度。
- 旋转轴来自之前的 STEP 解析圆柱面检查，在当前 MJCF 坐标中约为：
  - 轴上点：`[-0.0145, -0.07702, 0.04707870013] m`
  - 方向：`[0, 0, 1]`
- `q=0` 保持原 CAD 姿态不变，不代表机构收拢零位。
- 没有设置真实角度限位：`limited="false"`，省略 `range`。**这只是因为限位未知，不表示真实硬件能无限旋转。**
- 没有驱动器，模型不会自动展开。可设置关节位置检查姿态，或后续补充驱动与控制。

## 铰链外观的简化

原 STEP 的每个 WXJL180 铰链是一个合并实体，没有独立的固定叶片、活动叶片和销轴装配。
本版保留其原始表面，在测得轴线所在平面上做三角形精确裁分，分成固定/随动两份外观网格。
这是**显示用分区，不是内部机械结构的完整还原**；没有生成切口封盖，也不应用来计算接触、间隙或碰撞。
零姿态表面总面积守恒，整体装配位置保持不变。微观叶片/轴销应在取得更完整 CAD 后进一步建模。

## 动力学边界（重要）

活动 body 使用 **人为设定的 1 kg 质量**；质心和惯量按活动组件外接盒作均匀体估算，仅使带自由度的 MJCF 能编译和做运动测试。
它们不是实际的质量、质心或惯量；不能用于真实卫星控制、载荷或展开动力学结论。

- 所有 visual geom 仍为 `mass="0"`，碰撞关闭。
- 重力为零，未增加阻尼、驱动力、接触或解锁事件。
- 预览图中的绿色仅用于强调随动几何；MJCF 本身保留原 CAD 颜色。
- 预览滑块使用预先计算的 `0°–90°` 姿态，**不是实际机械限位、已验证的安全区间或实时平台控制器**。
- 未接入 Basilisk/UE，未替换当前平台启动场景。

## 设置角度的示例

在外层工作区使用专用转换环境运行以下 Python 逻辑：

```python
import math
import mujoco

model = mujoco.MjModel.from_xml_path(
    r"model\ground_validation_satellite\ground_validation_satellite_articulated.xml"
)
data = mujoco.MjData(model)
jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "outer_panel_hinge")
data.qpos[model.jnt_qposadr[jid]] = math.radians(45)  # 演示值，不是机械限位
mujoco.mj_forward(model, data)
```

当前服务器的原生 OpenGL Viewer 仍受系统驱动限制；可直接查看本目录的 HTML/GIF。
HTML 播放的是 MuJoCo 编译几何和前向运动学生成的 CPU 图像，不是 AI 图片或 UE 截图。

## 重新生成（外层工作区 PowerShell）

```powershell
.\run\step-converter-venv\Scripts\python.exe -X utf8 .\space_sim_server\tools\build_satellite_wing_joint.py .\model\ground_validation_satellite
.\run\step-converter-venv\Scripts\python.exe -X utf8 .\space_sim_server\tools\preview_satellite_wing_joint.py .\model\ground_validation_satellite
```

构建器校验源 STEP 和原网格哈希。若派生 XML/裁分 OBJ 被手动修改，构建器会拒绝覆盖；不会重置现有平台配置。
