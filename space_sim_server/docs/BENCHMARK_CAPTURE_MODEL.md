# Benchmark 完整 capture26 模型准备

抓取环境使用原 `model/SARM/platform/sarm_dual_capture26.xml`。它包含双臂服务星、独立自由部件、目标星及其铰接结构；远距任务的收拢模型删去了部分关节，不能直接用于机械臂任务。

`simulation.benchmark_capture_model.prepare_capture_model(repository, destination)` 只将 1587 项网格声明的文件路径转换为绝对路径。全部刚体、惯性、关节、执行器、接触/等式约束、插件参数和初始布局保持原值，不添加推进器或连接约束。输出仍叫 `team_flight.xml`，以复用已有 UE 镜像入口；这个历史文件名不表示使用收拢模型。

准备器保留六个原根刚体：`cubesat_bus`、`aviation_plug`、`guide_plug`、`cylindrical_plug`、`bolt`、`capture_target`。还原样复制飞轮硬件配置，从 XML 读取十二个捕获关节的类型、量纲、范围和伺服参数，初始参考沿用 `capture_arm_target.INITIAL_JOINTS`。旋转关节使用 rad/N·m，直线关节使用 m/N，不把十二维混合坐标统称为弧度。

`spacesim-autonomy-capture-model/1` 清单绑定原 XML、全部网格、飞轮配置、平台和交付版螺纹插件源码，以及生成模型的字节哈希。准备器拒绝 LFS 指针、缺失/空资源、模型目录外路径、未展开 include、缺失原关节/根刚体和未显式支持的新资源类型。先验证输入，再创建新的回合目录；已有目录或部分写入记录均不会被覆盖。

`verify_capture_model(repository, destination)` 只读核验完整树、资源绑定、模型字节和配置。修改关节、惯量、碰撞、约束或资源后，即使重写清单中的输出哈希，也不能冒充原模型。离线核验不改动任何输入。绝对资源路径绑定本机工作树；迁移后应新建回合，不修改旧回合的模型或证据。

准备器不输出一个恒定的“整星 COM”。机械臂运动会改变组合质心，环境必须使用原生编译后的质量参数和当前关节编码器，区分初始属性、动态质量模型及私有物理真值。原生库和 SDF 插件也必须单独构建、核验并绑定到实际运行；仅有静态清单不足以证明已加载正确模型。

## 验证范围

静态检查覆盖全树保留、原始输入字节和修改时间、十二通道单位/参数、拒绝覆盖，以及重新计算哈希后的关节/惯量/碰撞/约束篡改。原生检查另行比较原文件加载与回合文件内存加载在同一合法初态下的每步 qpos、qvel 和实际执行器力，使用原十二个捕获伺服，并核验运动相机原点读回及重建复位。

原生对比只检查零重力原始布局中的 0.5 s 模型等价性，不属于在轨抓取、真实图像闭环或任务成功。其运行结果须另行记录；当前新增的是完整模型准备组件，不增加任务注册或验收数。

源码 `26b2a20` 已在 SpaceSim 云端完成 **126 项模型/相机/旧场景检查与 Ruff，零跳过**；其中包括 18 项新增完整模型检查。基础 CI 收集范围保留 3569 项，确认纯模型/相机检查仍被收集，两项依赖 Basilisk 的启动清理检查在此次云端套件中显式执行。报告 `.tmp/benchmark-capture-model-checks-v1/acceptance.json` SHA-256 为 `4403c9d0a75c9d6d4e7eacf4822feeeaad4c7841db1bf77c700a7224529ec289`。

同一源码已固定到 `benchmark-capture-model-v1-20261107` 本人隔离工作树，源码字节一致，1588 个原模型/网格文件绑定核对通过。原生 DLL SHA-256 为 `76c840a4039e6e6bc769182a4879cd0b54167a373fff6881294eae3decfcea44`，构建清单为 `124436b36e242b57bd837a34d03ce5c235e548d056d0d5e44e0ac65565f7fc05`；SDF 插件 SHA-256 为 `058fbbfdb2c96ff5cd5a3166ff0056ce2efe244bfcb3d7b40f8a805919c75655`，插件清单为 `b2e2428c13c1c3427e302aeae3edfe71691f8a55d2faa36d08c1f48da50f52f9`。原生组件检查等待 D05 和运动挂载检查释放同一路资源名额后运行，目前尚无原生对比通过结论。

完整环境还需连接轨道/自由部件初始化、原保持和约束组件、受限方法动作与实际关节编码器、运动相机和投递时延、接触日志、独立评分、复位及冻结矩阵。非合作目标路线的遥测和主动协作控制必须另行移除，不能由这个模型转换器自动获得。

云端验证命令：

```text
python -m pytest tests/test_benchmark_capture_model.py -q -ra
python -m pytest tests/test_benchmark_native.py::test_native_complete_capture_model_matches_source_dynamics_and_reset -q -ra
```
