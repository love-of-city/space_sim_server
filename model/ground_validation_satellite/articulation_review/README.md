# 地面验证星：疑似活动组件确认

**只完成证据分析和标注，没有添加关节或修改原模型。**

- 打开 `review.html` 查看总图、局部图、依据及待确认项（离线可用）。
- `overview.svg` 为可放大的标注总图；浏览器可直接打开。`overview.png` 是该 SVG 的浏览器渲染快照。
- `candidates.json` 为候选记录，所有 `range_deg` 仍为 `null`。
- `assembly_inventory.json` 保留解码名称、源实例编号、geom 映射及边界。
- `cad_surface_evidence.json` 是原始 STEP 的解析曲面证据，不是自动识别出的关节列表。

| 编号 | 组件 | 初步判断 |
|---|---|---|
| A | 外侧板及板间铰链 | 较强的折叠关节候选 |
| B | 底部自拍相机 / 支杆组件 | 中等：疑似展开连杆机构 |
| C | 解锁器 01 / 02 / 03 | 名称明确；运动形式待确认 |
| D | 正面分离机构 · 星体端 | 接口位置明确；不是已确认的内部关节 |

## 注意

高亮整组不表示所有成员都应活动；R 内侧板暂作固定参考。
A 的上下铰链共轴，但机械限位、运动方向及驱动均未知；不从 WXJL180 型号推断 180° 限位。
B1–B3 是圆柱轴线特征，不能直接等同于三个独立关节。C/D 可能只需事件建模。
本次检查不提供真实质量/惯量，不使用原自由预览的人为 1 kg 参数。

## 复现（外层工作区 PowerShell）

```powershell
.\run\step-converter-venv\Scripts\python.exe -X utf8 .\space_sim_server\tools\extract_step_surface_evidence.py .\model\ground_validation_satellite
.\run\step-converter-venv\Scripts\python.exe -X utf8 .\space_sim_server\tools\review_satellite_components.py .\model\ground_validation_satellite
```

报告使用已有 CPU 几何渲染与原生 SVG/HTML 标注；无 AI 图片、无 CAD 云端上传、无额外二进制下载。
表单需点击导出按钮才会保存确认意见，不会直接修改模型。
