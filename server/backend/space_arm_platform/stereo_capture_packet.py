"""Image pose to calibrated gripper interface; no dynamics or renderer state."""
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np


def configuration_id(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def rotation(value):
    r = np.asarray(value, float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or abs(np.linalg.det(r)-1) > 1e-6):
        raise ValueError('invalid rotation')
    return r


def capture_packet(result, config, *, session_id, source_frame_id, sim_time_ns,
                   published_wall_time_ns=None):
    """Export both accepted and rejected images, never reuse an old pose."""
    packet = dict(schema='near23-stereo-capture/1', valid=False, status='lost',
                  session_id=str(session_id), source_frame_id=str(source_frame_id),
                  sim_time_ns=str(int(sim_time_ns)), age_s=0.,
                  published_wall_time_ns=(time.time_ns() if published_wall_time_ns is None
                                          else published_wall_time_ns),
                  configuration_id=configuration_id(config),
                  camera_ids=config['camera_ids'], result={'alignment': {'valid': False}},
                  measurement=result)
    if result.get('status') != 'ok' or result.get('route') != 'joint_multicamera':
        packet['reason'] = result.get('reason', 'joint stereo pose unavailable')
        return packet
    ids = result['used_ids']
    rms = result['reprojection_error_px']
    for eye in ('left', 'right'):
        if not ids.get(eye) or len(ids[eye]) != len(set(ids[eye])):
            raise ValueError('each eye needs complete unique marker inliers')
        if not np.isfinite(rms[eye]) or not 0 <= rms[eye] <= 2.5:
            raise ValueError('invalid stereo reprojection RMS')
        maximum = float(result['maximum_corner_error_px'][eye])
        side = float(result['minimum_marker_side_px'][eye])
        if not np.isfinite(maximum) or not 0 <= maximum <= 5 or not np.isfinite(side) or side < 12:
            raise ValueError('invalid stereo corner/size quality')
    if result.get('positive_depth') is not True:
        raise ValueError('stereo points must have positive depth')
    if config['mount_parent_id'] != 'teleop/gripper':
        raise ValueError('capture requires a gripper-mounted stereo pair')
    mount = config['parent_from_optical']['left']
    rgc = rotation(mount['R_parent_from_optical'])
    rct = rotation(result['pose_target_to_left']['R'])
    t = np.asarray(result['pose_target_to_left']['t_m'], float)
    center = np.asarray(config['marker_rig']['grasp_center_target_m'], float)
    offset = np.asarray(mount['position_parent_m'], float)
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in (t, center, offset)):
        raise ValueError('invalid calibrated interface position')
    # The fixed interface axes are the existing capture arm's axes, not tag axes.
    rti = np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])
    point = rgc @ (rct @ center + t) + offset
    packet.update(valid=True, status='tracking',
                  result={'alignment': dict(valid=True, interface_position_grip_m=point.tolist(),
                                           R_grip_from_interface=(rgc @ rct @ rti).tolist())})
    return packet


def publish_packet(path, packet):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(packet, allow_nan=False)+'\n', encoding='utf-8')
    # Windows readers can briefly hold the mailbox open.
    for attempt in range(30):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 29:
                raise
            time.sleep(.005)
