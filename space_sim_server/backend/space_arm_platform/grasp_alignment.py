"""Read-only camera-to-gripper pose conversion; never imports a control client.

T_AB maps B coordinates into A. G is the existing grip_frame site. The display
frame E has the same origin as G: +X right (-G.Y), +Y down (-G.X), +Z forward
(-G.Z). Rotation errors are SO(3) rotation-vector components in E, not Euler
angles and not three independent sequential turn commands.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np

DISPLAY_FROM_GRIP = np.array([[0., -1, 0], [-1, 0, 0], [0, 0, -1]])
# Existing capture26 convention from simulation/components/capture26.py:
# interface X = target X; interface Y = -target Z; interface Z = target +Y.
TARGET_FROM_INTERFACE = np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])
INTERFACE_CENTER_TARGET_M = np.array([.1755, .05475, .0420787001299893])


def _rotation(value):
    r=np.asarray(value,dtype=float)
    if r.shape != (3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
        raise ValueError('invalid rotation matrix')
    return r


def _quaternion_rotation(text):
    q=np.fromstring(text or '1 0 0 0',sep=' ')
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('invalid camera/site quaternion')
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def _position(value):
    p=np.asarray(value,dtype=float)
    if p.shape!=(3,) or not np.isfinite(p).all():
        raise ValueError('invalid position')
    return p


def load_capture_mount(model_path: Path):
    root=ET.parse(model_path).getroot()
    body=root.find(".//body[@name='gripper']")
    camera=body.find("camera[@name='capture_wrist_cam']") if body is not None else None
    site=body.find("site[@name='grip_frame']") if body is not None else None
    if camera is None or site is None:
        raise ValueError('camera and grip_frame must be on the same fixed gripper body')
    for node in (camera,site):
        if any(k in node.attrib for k in ('xyaxes','euler','axisangle','zaxis')):
            raise ValueError('explicit quaternion camera/site mount required')
    R_BG=_quaternion_rotation(site.get('quat'))
    R_BC=_quaternion_rotation(camera.get('quat')) @ np.diag([1.,-1,-1])
    p_BG=_position(np.fromstring(site.get('pos','0 0 0'),sep=' '))
    p_BC=_position(np.fromstring(camera.get('pos','0 0 0'),sep=' '))
    return {'camera_id':'teleop/camera/capture_wrist_cam','reference_site':'grip_frame',
            'R_grip_from_camera_optical':(R_BG.T@R_BC).tolist(),
            'camera_position_grip_m':(R_BG.T@(p_BC-p_BG)).tolist(),
            'interface_origin_target_m':INTERFACE_CENTER_TARGET_M.tolist(),
            'R_target_from_interface':TARGET_FROM_INTERFACE.tolist()}


def _rotation_vector(r):
    angle=math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
    skew=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])
    if angle<1e-7:
        return .5*skew
    if math.pi-angle<1e-5:
        _,vectors=np.linalg.eigh((r+r.T)/2)
        axis=vectors[:,-1]
        if np.linalg.norm(skew)>1e-10:
            if np.dot(axis,skew)<0: axis=-axis
        elif axis[np.argmax(np.abs(axis))]<0:
            axis=-axis
        return axis*angle
    return skew*(angle/(2*math.sin(angle)))


def alignment_from_pose(observation: dict, mount: dict, standoff_m: float=.5):
    result={'valid':False,'control_enabled':False,
            'visual_status':observation.get('status'),
            'reference_site':mount['reference_site'],
            'display_axes':'X right, Y down, Z forward; origin grip_frame',
            'angle_definition':'SO(3) correction rotation vector in display axes, degrees; not Euler angles',
            'desired_standoff_m':float(standoff_m)}
    if not math.isfinite(standoff_m) or standoff_m<0:
        raise ValueError('invalid desired standoff')
    if observation.get('status')!='tracking' or not observation.get('pose'):
        result['reason']='visual pose unavailable or rejected; no stale error retained'
        return result
    if observation.get('camera_id')!=mount['camera_id']:
        raise ValueError('visual camera does not match mount calibration')
    pose=observation['pose']
    R_CT=_rotation(pose['rotation_camera_from_target'])
    t_CT=_position(pose['translation_m'])
    R_GC=_rotation(mount['R_grip_from_camera_optical'])
    t_GC=_position(mount['camera_position_grip_m'])
    p_TI=_position(mount['interface_origin_target_m'])
    R_TI=_rotation(mount['R_target_from_interface'])
    p_GI=R_GC@(R_CT@p_TI+t_CT)+t_GC
    R_GI=R_GC@R_CT@R_TI
    p_EI=DISPLAY_FROM_GRIP@p_GI
    # Nominal G and I axes coincide; preparation point lies along grip -Z.
    desired=np.array([0.,0.,standoff_m])
    result.update(valid=True,
        interface_position_grip_m=p_GI.tolist(),
        R_grip_from_interface=R_GI.tolist(),
        interface_position_display_m=p_EI.tolist(),
        position_error_display_mm=((p_EI-desired)*1000).tolist(),
        rotation_error_display_deg=np.degrees(DISPLAY_FROM_GRIP@_rotation_vector(R_GI)).tolist(),
        total_angle_error_deg=math.degrees(float(np.linalg.norm(_rotation_vector(R_GI)))),
        interface_origin_target_m=p_TI.tolist(),
        mount_calibration=mount,
        desired_R_grip_from_interface=np.eye(3).tolist(),
        position_error_definition='current interface position minus desired [0,0,standoff] in display frame; geometric errors only, not a planned displacement')
    return result
