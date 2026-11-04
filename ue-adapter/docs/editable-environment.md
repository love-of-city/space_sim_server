# 可编辑的项目环境关卡

环境关卡是源码工程中的 `/Game/BSK/Maps/BskEnvironment`，磁盘文件为
`Unreal/BskUnrealRenderer/Content/BSK/Maps/BskEnvironment.umap`。
它保存地球、云层、大气、星空、太阳视觉效果、两路太阳灯光、补光、曝光体积和预览相机。
这些对象在编辑器打开关卡后就存在，不依赖后端，也不需要先点击 Play。
卫星、机械臂、操作台和采集相机继续由后端 manifest 动态提供。

## 在本机编辑

1. 在统一仓库执行 `git lfs pull`，再按[外部环境资产恢复](../../docs/guides/external-environment-assets.md)还原 IncredibleEarth 包；按适配器 README 使用 UE 5.6 构建源码插件。
2. 打开源码工程 `ue-adapter/Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject`。
   默认启动地图应为 `BskEnvironment`；也可以从 Content Browser 的 `BSK/Maps` 手动打开。
3. 在 Outliner 展开 **BSK Environment** 文件夹。编辑时先停止 Play，避免修改仅存在于运行世界副本中的对象。
4. 选择 `Earth`、`Stars` 或 `Sun visual` 调整 Transform；选择 Body 的 `Surface`、`Clouds`、
   `Atmosphere` 组件调整层偏移、缩放、可见性及 Materials。地球原始模型轴心不在球心，
   生成时已用组件相对偏移校正，Actor 原点代表天体中心。不要为修正轴心而清零层组件的位置。
5. 使用太阳灯光的 Details 修改颜色、阴影、光照；`Fill` 是默认关闭的美术补光。
   `Exposure` 是无限范围 Post Process Volume，可以保存曝光、色调映射和其他后处理设置。
6. 选择 `Preview camera`，右键 **Pilot**，用编辑器视口调整构图；退出 Pilot 后，选择环境管理对象，
   点击 **Store Editor Preview View** 将相机构图也存为关卡初次打开的编辑器视角，然后保存关卡。
   这台相机的 Transform 和 FOV 是独立运行预览的持久基准。普通编辑器漫游视角不等于相机的保存位置。
7. 使用 **Save All** 保存关卡以及修改的材质实例。关闭并重新打开工程检查保存结果。

`BSK Environment — ownership and bindings` 是 `ABskEnvironment` 管理对象，只保存各 Actor 的明确引用和
接管选项，不会在 Construction Script 中重建或覆盖用户的材质、组件和灯光设置。
可以把这些原生 Actor 另存为蓝图并替换引用；保持管理对象中绑定的类型及必需引用有效。

默认地球半径为 6,371 km，中心在预览原点下方 6,871 km，代表约 500 km 轨道高度；生成几何使用 100 cm/m。
当前保存关卡采用 IncredibleEarth 材质和大气太阳圆盘，原 SpaceFX 效果仅留作旧关卡回退。动态云层和极光需要开启编辑器视口的 **Realtime**。来源、当前保存配置与尺度适配见[迁移说明](../Unreal/BskUnrealRenderer/docs/incredible-earth-migration.md)。
没有额外捕获式 SkyLight，保留 Linux 离线采集的原有直射太阳路径。

## 独立预览和后端接管

管理对象的 **Allow Backend Control** 打开时，独立运行最初使用保存的环境；后端 manifest 和帧到来后接管。
关闭该选项用于纯美术独立预览，运行时拒绝后端场景接入，避免出现动态物体与手工环境使用不同原点的画面。
运行中的 **Is Backend Controlled** 显示接管状态；它是瞬态状态，不会保存到关卡。

| 内容 | 独立预览 | 后端驱动 |
| --- | --- | --- |
| 地球 Actor 的位置、旋转及物理半径 | 保存的默认值 | 仿真局部坐标、姿态及 manifest 半径 |
| 地球各层的材质、相对位置/缩放 | 保存的美术配置 | 继续使用保存的美术配置 |
| 大气中心与底面半径 | 保存的默认值 | 跟随地球中心和物理半径 |
| 太阳方向、角直径、遮挡及照度 | 保存的默认值 | 星历、距离平方规律及日食状态 |
| 太阳视觉材质和特效 | 保存的美术配置 | 复用；可见性、尺寸和亮度随太阳状态变化 |
| 星空的位置、旋转、缩放、材质 | 保存值 | 保存值 |
| 曝光体积、补光及阴影等美术参数 | 保存值 | 保存值，模型照明协议另有明确控制的项除外 |
| 预览相机 | 保存的 Transform/FOV | 保留既有控制台和动态采集相机流程 |

`Earth Radius Meters` 表示**未计 Actor 缩放的各层组件几何**所对应的物理基准半径。
后端接管时，地球 Actor 使用统一缩放 `manifest 半径 / Earth Radius Meters`，覆盖预览时保存的 Actor 缩放，
避免手动放大预览地球后改变仿真中的物理尺寸；组件的层偏移、非均匀缩放和材质美术配置继续保留。
每个 manifest 都从几何基准重新计算，避免重复连接时累计放大。没有地球 manifest 时恢复 Actor 保存的预览缩放。
`Sun Illuminance Scale` 是本机的太阳光美术校准系数；`Sun Visual Distance Meters`、
`Sun Visual Emissive Strength` 和 `Sun Visual Effect Scale` 控制太阳渲染代理，不改变权威天体轨迹。
物理地球位置与姿态依旧通过已有米→厘米、Y 轴镜像和浮动相对原点机制更新。

Play/独立进程中的仿真修改只作用于运行世界，不写回源码关卡。停止 Play 后，编辑器仍显示原保存配置。
断开连接后保留最后收到的后端画面；下一份 manifest 到来时先恢复保存的美术基准再接管，
EndPlay 时也恢复运行世界的基准。停止运行不会把最后一帧状态保存进源码关卡。

## 首次生成及资产维护

仓库中直接保存 `.umap` 和太阳预览材质实例。只有缺少地图或需要从头生成一个新基线时才运行生成脚本。
已有地图时脚本打印 `BSK_ENVIRONMENT_UNCHANGED` 并保留用户编辑；不会因为构建或启动而重置环境。
不要删除已有的手工编辑地图再运行脚本，除非已另存备份并确实希望重新制作默认环境。

Windows（在适配器目录中，替换引擎路径）：

```powershell
$env:UE56_ROOT = 'E:\UE5.6\UE_5.6'
pwsh -NoProfile -File .\scripts\build.ps1 -UnrealRoot $env:UE56_ROOT
& "$env:UE56_ROOT\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" `
  "$PWD\Unreal\BskUnrealRenderer\BskUnrealRenderer.uproject" `
  -run=pythonscript `
  "-script=$PWD\Unreal\BskUnrealRenderer\scripts\create_environment_level.py" `
  -unattended -NullRHI -nosplash
```

Linux（替换引擎根目录）：

```bash
ue_root=/path/to/UnrealEngine
project_root=/home/lyh/server/ue-adapter/Unreal/BskUnrealRenderer
"$ue_root/Engine/Binaries/Linux/UnrealEditor-Cmd" \
  "$project_root/BskUnrealRenderer.uproject" \
  -run=pythonscript "-script=$project_root/scripts/create_environment_level.py" \
  -unattended -NullRHI -nosplash
```

生成使用编辑器 Python 插件；该插件只对 Editor 目标启用。地图保存原生 Runtime Actor 和资源引用，
运行与打包不依赖 Python。关卡和材质资源应按已有 Git LFS 规则提交，不能用 LFS pointer 代替实际资源。
部署副本需要同步源码关卡、其引用资源和 Runtime 插件，并在目标操作系统重新构建插件；
不能把 Windows 的 `Binaries` 拷贝到 Linux 使用。源码与运行副本的具体路径和命令见
[环境部署同步](../../server/docs/environment-deployment.md)。

## 验证

- 编辑器不启动后端、不点 Play，检查地球、星空、太阳、灯光与曝光；移动预览相机、修改一个材质实例、保存重开。
- 无后端单独 Play，检查使用保存视角和环境；停止后确认编辑器配置未被运行时修改。
- 启动完整平台，检查只存在一个地球和一套太阳环境，地球随星历更新，原操作视角和采集相机工作正常。
- 切换任务、重新连接或重复 manifest，检查 Actor 不累计、地球半径不累计缩放，材质和曝光配置保留。
- Linux 使用原有 `-RenderOffscreen` / `-BskOfflineCapture` 运行并检查权威图像，不能以 NullRHI 结果替代视觉验收。

`scripts/validate_environment_level.py` 提供可重复的编辑器保存验收，调用方式与生成脚本一致。
它复制地图到 `/Game/Developers/__BskEnvironmentValidation`，修改地球位置/旋转/缩放、层材质、太阳位置、
星空旋转、灯光强度、曝光和预览相机，保存、卸载并重开后核对，最后删除本次临时副本。
源关卡和共享材质不会保存，脚本还会核对源 `.umap` 的 SHA-256 没有改变。
如果该临时路径原本已存在，脚本直接报错，不覆盖已有文件。成功日志为 `BSK_ENVIRONMENT_PERSISTENCE_OK`。
