# Benchmark 运动夹爪相机

抓取场景的原双目安装在 `gripper` 上。机械臂和夹爪运动后，相机外参随原生关节运动变化；已有 D23/D24/D27/D37 配置的相机固定在收拢服务星本体上，两者不能互换。本组件提供真实挂载体读回、渲染接入和独立曝光核验，为后续 D54 等接触前视觉操作提供接口基础；不增加任务目录覆盖数，也不构成完整抓取验收。

## 标定与公开输入

`capture26_mounted_cameras(repository)` 读取原 `capture26_near23_front_mount_20261029.json`，保持双目 1920×1920、10 Hz、0.12 m 基线、100° 水平视场及原安装位姿。只将渲染命名空间从 `teleop/` 改为当前 Benchmark 的 `team/`。右目保留显式的 `inspection-1500cd-v1` 工作灯。标定为仿真设计值，不是实物标定；相机壳体的质量和碰撞仍未建模。

新 `spacesim-autonomy-mounted-camera/1` 明确 `parent_body_name`。`position_body_m` 指相机原点在该具名刚体**原点坐标系**中的位置，不是总星体或刚体 COM；`orientation_body_from_optical_wxyz` 将右、下、前的光学轴转换到挂载体轴。UE 原始相机使用前、左、上，接入时显式转换。

UE 的 0.002 s 曝光值与 f/4 用于亮度计算，图像仍是权威物理瞬间的针孔渲染，没有实现 2 ms 内运动积分。接口拒绝把这种图像标成带时间积分的曝光。

公开 `spacesim-autonomy-mounted-camera-image/1` 只包含相机/会话/帧号、整数采样时刻、尺寸、PNG 哈希及原始像素。独立 1920 上限和 16 MB PNG 上限允许原设计图像；旧图像格式仍限制 640×480 和 1.35 MB，不改变旧任务、种子、阈值或序列化默认值。PNG 检查包含压缩流长度、像素行、CRC、尺寸和哈希，并拒绝可携带私有位姿的文本/EXIF 块。

## 原生与渲染接入

`NativeCameraMountReader` 从 `PhysicsPorts.body_state(parent_body_name)` 读取挂载体原点和姿态。它同时检查步进器整数时钟、消息写入时刻、模型 XML 哈希和初始化后状态写入次数；不会调用积分、forward 或状态赋值。陈旧、未发布和已关闭运行时均拒绝。绝对采样时刻等于回合纪元加原生相对时刻。

`ArticulatedRenderedCameraSensor` 复用原隔离 UE 进程、捕获接收器、端口、故障和取消管理。调用者须提供完整实际模型，放在共用镜像入口 `episode/model/team_flight.xml`；文件名是历史共用路径，不意味着可传收拢替代模型。该文件的字节哈希必须等于 `LocalMujocoStepper` 实际加载的 XML 哈希，资源引用须满足原镜像的绝对路径规则。

相机实际挂在 `team/gripper`，目标附加原 23 块标志物几何。每次捕获前冻结当前挂载体出版状态，等待真实 UE 图像后再次读取，期间物理状态必须不变。返回 `(images, private_receipts)`：环境只把前者构造为方法观测，后者归档为独立评分证据。现有固定本体传感器继续使用原相机和标志物配置。

## 独立曝光核验

`MountedCameraCaptureEvidence` 保存模型/回合身份、原生步数、物理和绝对时钟、挂载体原点/姿态、实际渲染外参以及图像/标定哈希。`verify_mounted_camera_capture` 纯读取重算：

```text
相机原点_N = 挂载体原点_N + R_NB · 安装偏移_B
R_N,optical = R_NB · R_B,optical
```

实际渲染外参必须为右手正交矩阵，位置误差不超过 10 µm、方向误差不超过 0.1 µrad，与已有视觉任务的对应容差一致。它拒绝错误挂载体、固定本体替代、COM 混用、FLU/光学轴混用、反射、非单位姿态及过期时间。

`MountedCaptureSequence` 检查连续采样网格、渲染会话、连续帧号、原生步数推进、双目同时性和同一挂载体状态的一致性。发生错误后该序列保持无效，下一条看似正确的帧不能抹去此前错误；复位需要新的回合实例。文件重评分不改写输入。

此处只验证相机与物理挂载的对应关系。完整回合仍须绑定源码、原生库、全部模型/网格、UE 构建和运行日志，以及控制动作、完整采样区间和独立任务评分。

## 验证与尚需集成的内容

新增测试覆盖 1920 像素契约、旧格式边界、原双目标定、组合旋转/杆臂、失效帧、会话/模型/时钟混用和私有数据隔离。原生测试使用具有偏置 COM 和偏置铰轴的转动夹具，比较开启/关闭读取及重建回合后的每步物理字节，验证读取实际原点而非 COM；它不生成 UE 图像，也不代表卫星抓取成功。检查结果随云端执行后另行记录。

完整 D54/D55/D62 尚需原 capture26 模型及关节伺服的统一 reset/step 方法接口、编码器观测与受限动作、实际图像闭环、时延队列、全回合接触/安全评分及冻结矩阵。现有外部 JSONL 方法通道仍为 4 MiB 消息上限，不能直接承诺承载任意两张设计分辨率 PNG；接触任务接入前须完成相应版本化传输契约与验证。非合作路线还必须移除合作目标遥测和主动姿态控制。

云端原生组件命令：

```text
python -m pytest tests/test_benchmark_native.py::test_native_articulated_camera_mount_reads_fresh_origins_without_changing_motion -q -ra
```
