# Orbital 40 HDRI、单反参数和电影级 Lumen 预览

用户要求查看本机 `RenderCrate-HDRI_Orbital_40_UE.hdr` 配合单反相机和电影级 Lumen 的效果。
检查发现它是 16384×8192 的完整轨道环境全景，已包含地球、云、大气、星空和太阳，不能直接当作三维地球的地表漫反射纹理。

在仓库外生成独立 UE 5.6.1 工程：
`/home/lyh/asset_staging/arghanion_planet/orbital40_cinematic/Orbital40Cinematic.uproject`。
地图为 `/Game/Orbital40/Maps/Lv_Orbital40_Cinematic`。
使用官方 HDRI Backdrop 和完整天空球，开启相机投影；CineCameraActor 使用 36 mm 画幅宽度的 16:9 有效区域、25.7067 mm 镜头、f/8、ISO 100、1/125 s，水平视场 70°，俯视 27°。
物理相机曝光开启；将 HDRI 强度从 8192 降至 4096，保留亮一档对照帧。

项目开启 Lumen GI、反射、硬件光追、Hit Lighting、前层半透明反射、Virtual Shadow Maps 和距离场。
MRQ 使用引擎原生 Cinematic（4），4K PNG、8 空间采样、64 引擎预热帧和 32 渲染预热采样，禁用纹理流送。
纹理以线性 RGBA16F TextureCube 导入，无 mipmaps，立方体面分辨率上限 4096。
场景、序列、MRQ 预设、重建脚本与说明均保存在独立工程中。

最终 GPU 渲染成功，UE 进程退出码 0；日志确认硬件光追启用，Lumen 和 Cinematic 参数已读回验证。
4K 图像路径：
`/home/lyh/asset_staging/arghanion_planet/orbital40_cinematic/previews/orbital40-dslr-lumen-cinematic.0000.png`。
本机验证报告为同工程 `verification.json`，原作者 34 个资产 SHA256 校验通过。

这是摄影背景预览，未切换 space_sim 在线场景，也未将全景贴到物理地球球体。
背景已包含的光照不会因 Lumen 开启而重新计算；平移镜头没有真实地表视差，不能表示任意轨道高度和昼夜变化。
原 HDR、Fab 作者资产和在线仿真配置未修改。

依据 UE 5.6 官方文档：

- https://dev.epicgames.com/documentation/unreal-engine/hdri-backdrop-visualization-tool-in-unreal-engine?application_version=5.6
- https://dev.epicgames.com/documentation/en-us/unreal-engine/cinematic-cameras-in-unreal-engine?application_version=5.6
- https://dev.epicgames.com/documentation/en-us/unreal-engine/lumen-global-illumination-and-reflections-in-unreal-engine?application_version=5.6
- https://dev.epicgames.com/documentation/en-us/unreal-engine/cinematic-rendering-image-quality-settings-in-unreal-engine?application_version=5.6

未修改 C++、动力学或服务端；验证采用实际 UE/MRQ GPU 渲染与资产校验，没有重复无关的服务端测试。
