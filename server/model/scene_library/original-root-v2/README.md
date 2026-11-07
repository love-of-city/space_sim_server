# 抓捕搬运的保存场景参考

来源为 wenquandan777 的保存场景（`aacce0d963aa0dcf0b3d655b0a4e2260d25f1cfd`）。
`preheld-dual.xml`、`initial-scene.json` 和 `attitude_control.json` 保留交付原件，
其中目标星拥有操作台；它不同于服务星自带操作台的旧双星模型。
模型 XML 的 SHA256 为 `b2d4ca7633d3fc84cd505c583f043a69c0819ba0bdf27c54cb99b578896a6b87`。

这是任务输入，不是本轮录制或训练数据。保存场景中的预持有焊接只供参考；
`prepare_capture_model` 会在运行副本中关闭它并恢复太阳敏感器的原安装结构，
随后通过真实接触抓捕和搬运。抓捕 runner 只从 `initial-scene.json` 读取操作臂保持关节角，
不直接加载该参考文件中的靶标附件。

`ue-catalog.json` 使用既有 portable catalog 协议，以 `server/model/layout-package.json`
标记模型根；切换目录不需要改绝对路径。模型共用 `server/model/` 下的既有 CAD 与碰撞网格。
相邻安装面的源 STL 与已验收 UE 网格保持相同字节（原交付含33个退化三角面）；
未改变其非退化三角面或碰撞分解网格。

使用前执行 Git LFS 拉取，并通过抓捕文档中的资产、初态和完整录制验收。
