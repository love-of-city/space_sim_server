# 精简保存格式

`config/recording.json` 集中控制新录制的保存方式：

```json
{
  "storage_profile": "compact",
  "save_raw_camera_images": false,
  "save_dynamics_trace": false
}
```

自动录制脚本每次启动读取配置。常驻平台后端在启动时读取，需要重载后端才能改变手动录制默认值。配置只影响新录制，不自动修改或删除旧数据。

## 保存内容

主数据保持官方 LeRobot v3，state/action 的七维语义、视频、采样率和数值精度不变。数值特征仍包含原始关节、速度、末端位姿与 50 Hz 力。精简模式不再将完整 observation 和相机 JSON 字符串重复嵌入主表。

| 路径 | 内容 |
|---|---|
| `metadata.json` | 结果、归属、种子、场景配置、采集统计 |
| `lerobot/data/` | 官方训练数值表 |
| `lerobot/videos/` | 原双相机视频 |
| `lerobot/meta/` | 官方特征/统计/索引及平台语义和版本信息 |
| `lerobot/auxiliary/context/` | 与主帧对应的会话、任务、BSK/姿态/轮系、引力配置、环境施力及交会来源，训练数值来源 SHA256 |
| `lerobot/auxiliary/cameras/` | 两路完整相机元数据，包括内外参、浮动原点、时间和源帧号 |
| `lerobot/high_frequency/wrist_wrench/` | 原始 200 Hz 力、力矩、有效标志和整数纳秒时间 |

Sidecar 使用 Zstd Parquet 分块；相机和上下文每 1024 行、力每 4096 行落盘，避免无限内存缓冲或每采样点产生一个文件。分块允许任务结果从 null 变为 success 等可选字段变化，读取时逐文件读取，不直接合并推断所有文件的 Arrow schema。传感器静态身份与坐标约定在 `meta/platform.json` 保存一次；力数值保持 float64。

相机固定字段通过列压缩避免每帧重复字符串的成本；动态相机外参仍完整保留。`space-arm-compact/1` 在平台元数据中标识新 sidecar 格式。数据读取代码不能再假设一定存在 `steps.jsonl`。

## 验收与兼容

实时配对及 ACK 流程保持原逻辑。正式自动采集仍执行连续帧/时间检查、任务成功判据、双相机会话配对、200 Hz 力连续性以及双视频全帧解码；主表数值与写入前摘要比对，力文件与源数据摘要比对。辅助文件尚未完成落盘时不能将 episode 标成完整。

平台查看器从主表和上下文读取新记录，旧记录仍从 JSONL 读取；200 Hz 曲线及 CSV 保持原时间轴，不插值。LeRobot 官方读取器只需要标准主表、视频和元数据，自定义 sidecar 不会改变训练字段。

排障时将 `storage_profile` 改为 `diagnostic`，恢复完整观测/相机 JSONL 和主表 JSON 字段。`save_dynamics_trace=true` 单独开启自动录制的全量物理 trace。启用原始照片保存时，仍写 `cameras/` 和用于定位照片的 `captures.jsonl`。

## 已验证样本

对 `episode-20261016-152729-0c81b376` 创建独立精简副本，原始数据保留。6417 个主帧、12834 个相机记录、25665 条力样本完成源数据比较；两路视频 SHA256 不变、完整解码通过。官方 LeRobot 读取器验证首/中/尾训练帧，平台查看器同步检查通过。

原始目录 195154087 字节（186.1 MiB），精简副本 83343833 字节（79.5 MiB），减少约 57.3%。该副本主表使用 Zstd 重写，未来在线录制主表仍使用安装的 LeRobot 默认 Parquet 压缩，大小会略有不同；sidecar 均为 Zstd。报告和副本位于被 Git 忽略的 `reports/compact-storage-validation-v2/`。

上述精简保存验证未改变视频编码、分辨率或帧率；该样本是旧数据转换后的验收。后续单机提速已单独验证完整实时采集，并启用边录制边编码，详见 [采集性能配置](COLLECTION_PERFORMANCE.md) 和根目录 `archive/2026_10_19_4090单路采集提速.md`。
