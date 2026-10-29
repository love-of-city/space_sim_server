"""Static wrist calibration shared by the image worker and simulation startup."""
import json
import math
from pathlib import Path
from .grasp_alignment import load_capture_mount

CAMERA_ID='teleop/camera/capture_wrist_cam'
VISION_CAMERA_ID='teleop/camera/capture_wrist_cam_alignment'


def build_wrist_config(root: Path):
    model=root/'model/SARM/platform/sarm_dual_capture26.xml'
    source=root/'config/vision/capture_wrist_camera.json'
    optics=json.loads(source.read_text('utf-8-sig'))
    dimensions=optics['resolution_wh']
    if len(dimensions)!=2 or any(isinstance(x,bool) or not isinstance(x,int) or x<=0 for x in dimensions):
        raise ValueError('invalid wrist camera resolution')
    w,h=dimensions
    K=optics['K'];fx,fy=float(K[0][0]),float(K[1][1])
    # Independent horizontal/vertical FOV supports an anisotropic raster while
    # preserving both requested angular spans and the centred principal point.
    if w<=0 or h<=0 or not all(math.isfinite(x) and x>0 for x in (fx,fy)):
        raise ValueError('invalid wrist camera resolution/focal length')
    expected=[[fx,0,(w-1)/2],[0,fy,(h-1)/2],[0,0,1]]
    if any(not math.isclose(float(K[i][j]),expected[i][j],abs_tol=1e-9) for i in range(3) for j in range(3)):
        raise ValueError('wrist rendering requires centred zero-skew intrinsics')
    if any(float(x)!=0 for x in optics.get('distortion_coefficients',[])):
        raise ValueError('wrist renderer does not implement lens distortion')
    fovs=[2*math.atan(w/(2*fx)),2*math.atan(h/(2*fy))]
    declared=optics.get('fov_xy_deg',[])
    if len(declared)!=2 or any(not math.isclose(math.radians(float(x)),y,abs_tol=1e-9) for x,y in zip(declared,fovs)):
        raise ValueError('wrist FOV disagrees with intrinsics')
    rate=float(optics['capture_rate_hz'])
    exposure=float(optics['max_exposure_ms_requirement'])/1000
    if not math.isfinite(rate) or rate<=0 or not math.isfinite(exposure) or exposure<=0:
        raise ValueError('invalid wrist camera rate/exposure')
    return dict(camera_id=CAMERA_ID,input_camera_id=VISION_CAMERA_ID,image_size=[w,h],
        K=K,D=[0,0,0,0,0],horizontal_fov_rad=fovs[0],vertical_fov_rad=fovs[1],
        capture_rate_hz=rate,exposure_time_s=exposure,
        calibration_version=optics['intrinsics_version'],
        calibration_source=str(source.relative_to(root)),
        calibration_status=optics['calibration_status'],
        exposure_semantics='ideal instantaneous geometry; no finite-time motion blur',
        rig=json.loads((root/'config/visual_attachments/target_aruco_flush_5cm_v2/aruco_rig.json').read_text('utf-8')),
        mount=load_capture_mount(model),standoff_m=.5,minimum_markers=2,
        vendor_root=str(root/'config/vision/vendor/aruco_pnp_basler_mono_v1'))
