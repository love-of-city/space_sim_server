"""Generate nominal stereo calibration and fixed body mounts, not hardware calibration."""
from pathlib import Path
import argparse
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation


def generate(rig):
    f = .006 / 3.45e-6
    K = [[f, 0, 959.5], [0, f, 599.5], [0, 0, 1]]
    center = np.array([.58, -.16, .12])
    # Fixed provisional mount aimed through the initial target work area. It does
    # not track the target or read truth from a running simulation.
    design_aim = np.array([.30634, -.91079, .48777])
    forward = design_aim-center
    forward /= np.linalg.norm(forward)
    # Keep the physical baseline in the spacecraft's horizontal XY plane;
    # both optical centres have the same height above the top panel.
    left = np.cross(np.array([0., 0, 1.]), forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    R_body_renderer = np.column_stack([forward, left, up])
    R_body_optical = np.column_stack([-left, -up, forward])
    qxyzw = Rotation.from_matrix(R_body_renderer).as_quat()
    quaternion = [float(qxyzw[3]), *qxyzw[:3].tolist()]
    ids = {e: 'teleop/camera/capture_stereo_'+e for e in ('left', 'right')}
    cameras = []
    optical_mounts = {}
    for eye, offset in (('left', .06), ('right', -.06)):
        position = (center+offset*left).tolist()
        cameras.append(dict(camera_id=ids[eye], parent_id='teleop/cubesat_bus',
            position_body_m=position, orientation_body_from_camera_wxyz=quaternion,
            field_of_view_rad=2*math.atan(1920/(2*f)), resolution=[1920, 1200],
            semantic_label='stereo_navigation_camera', display_name='Capture stereo '+eye,
            picture_in_picture=False, capture_rate_hz=10, capture_products=['rgb'],
            capture_consumer='vision', exposure_time_s=.002, aperture_fnumber=4))
        optical_mounts[eye] = dict(position_body_m=position, R_body_from_optical=R_body_optical.tolist())
    markers = {}
    for m in rig['markers']:
        c, u, v = (np.asarray(m[k], float) for k in ('center_m', 'right', 'down'))
        h = m['size_m']/2
        markers[str(m['id'])] = [(c+h*(a*u+b*v)).tolist() for a,b in ((-1,-1),(1,-1),(1,1),(-1,1))]
    return dict(schema='capture26-stereo-vision/1', camera_ids=ids,
        trigger_rate_hz=10, max_skew_ns=100000,
        calibration_status='nominal_simulation_not_measured_hardware_calibration',
        hardware=dict(camera='Basler ace 2 a2A1920-51gc x2 (exact BAS/PRO suffix to be confirmed)',
            lens='Kowa LM6HC 6mm x2', sensor='Sony IMX392', pixel_pitch_m=3.45e-6,
            shutter='global', hardware_max_fps=51, exposure_requirement_s=.002,
            interface='GigE/PoE', focus_range_m=[.1, None], aperture_range=[1.8, 11]),
        mount_status='provisional body top/front edge, static parallel axes; no camera housing added',
        mount_center_body_m=center.tolist(), design_aim_body_m=design_aim.tolist(),
        body_from_optical=optical_mounts, cameras=cameras,
        calibration=dict(K_left=K, K_right=K, D_left=[0]*5, D_right=[0]*5,
            R_right_from_left=np.eye(3).tolist(), t_right_from_left_m=[-.12,0,0],
            image_size=[1920,1200]),
        layout=dict(dictionary=rig['dictionary'], markers=markers),
        marker_rig=rig,
        algorithm_input='left/right RGB PNG + this fixed calibration + known marker corners; no simulator truth',
        output_scope='pose only; no velocity/filter/control and no legacy arm tool conversion',
        exposure_semantics='ideal instantaneous geometry; photometric shutter only, no finite-time motion blur')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--rig', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    config = generate(json.loads(args.rig.read_text('utf-8-sig')))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2)+'\n', encoding='utf-8')
