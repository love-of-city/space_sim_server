"""Static wrist calibration shared by the image worker and simulation startup."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .grasp_alignment import load_capture_mount

CAMERA_ID='teleop/camera/capture_wrist_cam'
VISION_CAMERA_ID='teleop/camera/capture_wrist_cam_alignment'


def build_wrist_config(root: Path):
    model=root/'model/SARM/platform/sarm_dual_capture26.xml'
    camera=ET.parse(model).find('.//camera[@name="capture_wrist_cam"]')
    w,h=map(int,camera.get('resolution').split())
    f=h/(2*math.tan(math.radians(float(camera.get('fovy')))/2))
    return dict(camera_id=CAMERA_ID,input_camera_id=VISION_CAMERA_ID,image_size=[w,h],
        K=[[f,0,(w-1)/2],[0,f,(h-1)/2],[0,0,1]],D=[0,0,0,0,0],
        rig=json.loads((root/'config/visual_attachments/target_aruco_flush_5cm_v2/aruco_rig.json').read_text('utf-8')),
        mount=load_capture_mount(model),standoff_m=.5,minimum_markers=2,
        vendor_root=str(root/'config/vision/vendor/aruco_pnp_basler_mono_v1'))
