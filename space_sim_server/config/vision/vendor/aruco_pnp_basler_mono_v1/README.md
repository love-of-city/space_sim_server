# 单目 PnP：相机参数更新版

按 Zijie 要求，沿用现有单目 ArUco + PnP，仅将识别示例使用的内参更新为新相机的名义参数。
源码 fiducials.py 与仓库现有版本逐字节一致，参数由 run_pnp.py 读取 camera.json 后传入。

| 参数 | 本包使用 |
|---|---|
| 相机数量 | 1，现有单目流程 |
| 相机 / 镜头 | Basler a2A1920-51gc / Kowa LM6HC 6 mm |
| 图像 | 1920×1200，像元 3.45 µm |
| fx / fy | 1739.1304347826087 px |
| cx / cy | 959.5 / 599.5 px |
| 畸变 | 理想仿真无畸变 |
| 视场 | 水平约 57.80°、垂直约 38.07° |
| 采集要求 | 10 Hz、曝光≤2 ms，需采集端另行配置 |
| 靶标 | 对称四靶，黑框5 cm，DICT_4X4_50，ID0–3，3 mm安装板 |

焦距像素值按6 mm / 3.45 µm计算；主点假设在图像中心。这是名义仿真配置，后续实物需标定。
相机位置/朝向沿用现有安装；新的机械臂外参/爪子中心需要同学接入后更新。

## 文件与运行

- `source/space_arm_platform/fiducials.py`：ArUco检测、平面IPPE候选、LM细化和质量判断。
- `camera.json`：本包实际读取的新相机参数。
- `model/ground_validation_satellite/aruco_rig.json`：当前对称5 cm靶标坐标。
- `run_pnp.py`：读取一张新相机图像，输出JSON；核心调用为estimate(image,K)。
- `legacy_reference.json`、`source/fiducial_center_metrics.py`：旧SARM工具到分离机构中心的坐标换算。
- `SOURCE_PROVENANCE.json`、`FILES.sha256`：源码来源与校验。

解压后在本目录运行（Python 3.11+）：

```text
python -m pip install -r requirements.txt
python run_pnp.py --image your_frame.png --output measurement.json
```

your_frame.png 应为按新相机视场/内参生成的1920×1200原始图像。
旧1024×1024图像不能改尺寸后当作新相机图像。本包未附旧图像或旧测量记录。
程序会拒绝图像尺寸不匹配的输入。

直接接到同学的代码时：

```python
import json, sys
from pathlib import Path
import cv2
import numpy as np

root = Path(__file__).resolve().parent  # 此示例脚本放在包根目录
sys.path.insert(0, str(root / 'source'))
from space_arm_platform.fiducials import FiducialEstimator

camera = json.loads((root / 'camera.json').read_text(encoding='utf-8'))
image = cv2.imread('your_frame.png')
result = FiducialEstimator().estimate(image, np.asarray(camera['K'], float))
```

## 输出定义与配置范围

pose.translation_m：目标卫星本体原点在相机光学系的位置，单位米；X向右、Y向下、Z向前。
pose.rotation_camera_from_target：将目标本体坐标转到相机光学系的旋转矩阵。
pose.rvec_rad：旋转向量，单位弧度，不能当三个欧拉角。
legacy_tool_measurement：旧sarm_ee工具中心至WF10分离机构整体AABB中心的向量和距离。
该参考关系只适用现有安装，换机械臂要改对应外参/工具中心。

navigation_for_control保持无效：单帧PnP没有速度，需要导航坐标变换、时间对齐与滤波后接V-bar。

本包使用run_pnp.py显式传入K；源码中decode_and_estimate/wrist_intrinsics及camera_defaults
是原仓库旧网页/MJCF入口的辅助方法，保留供阅读，本包不调用它们。
接新参数时请走estimate(image,K)，避免从旧MJCF重新推导内参。

这是相机参数更新及代码交接；没有执行新的UE采集、加灯或误差测量。
没有改动运行中的网页相机。完整仿真接入时还需将渲染端视场与输出尺寸同步设为camera.json对应值，
否则即使图像尺寸正确，错误K也会导致错误距离。
