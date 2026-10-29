"""Model-bound folded TCP / physical interface geometry for the motion demo.

The interface is the outward surface of capture_x_piece_* (+Y), not the legacy
capture_target_grasp site on a different face. No model or physical state edits.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from simulation.capture26_mission import MODEL_PATH, model_files


def load_geometry(root, config):
    root = Path(root).resolve()
    path = (root/config.operating_geometry_file).resolve()
    if not path.is_relative_to(root/'config/missions'):
        raise ValueError('operating geometry must be a versioned mission asset')
    data = path.read_bytes()
    g = json.loads(data)
    if (hashlib.sha256(data).hexdigest() != config.operating_geometry_sha256
            or g.get('schema') != 'space-sim-capture26-operating-geometry/1'
            or g.get('model_xml_sha256') != hashlib.sha256((root/MODEL_PATH).read_bytes()).hexdigest()
            or g.get('model_files_sha256') != model_files(root)
            or g.get('tool_site') != 'grip_frame' or g.get('terminal_gap_m') != .5):
        raise ValueError('capture26 operating geometry binding changed')
    for name in ('folded_tcp_body_m', 'surface_point_target_m', 'surface_normal_target', 'lateral_target_m'):
        if np.shape(g.get(name)) != (3,) or not np.isfinite(g[name]).all():
            raise ValueError('invalid capture26 operating frame: '+name)
    normal = np.array(g['surface_normal_target'])
    if not np.isclose(np.linalg.norm(normal), 1.) or abs(normal@np.array(g['lateral_target_m'])) > 1e-10:
        raise ValueError('operating lateral displacement must lie in the grasp plane')
    from simulation.serial_chain_kinematics import SerialChainKinematics
    kin = SerialChainKinematics.from_mjcf(root/MODEL_PATH, base_body='cubesat_bus',
        joint_names=tuple('bottom_joint'+str(i) for i in range(1, 7)), tool_site='grip_frame')
    if not np.allclose(kin.forward(g['folded_capture_joints_rad'])[0], g['folded_tcp_body_m'], atol=1e-12):
        raise ValueError('folded TCP no longer matches model FK')
    return g


def camera_depth(state, service_com, target_com, vision):
    rs, rt = state['r']
    camera = .5*sum(np.array(vision['body_from_optical'][eye]['position_body_m']) for eye in ('left', 'right'))
    from simulation.capture26_mission import MARKER_REFERENCE_TARGET
    delta = (state['p'][1]-state['p'][0]+rt@(MARKER_REFERENCE_TARGET-target_com)
             -rs@(camera-service_com))
    axis = rs@np.array(vision['body_from_optical']['left']['R_body_from_optical'])[:, 2]
    return float(axis@delta)


def working_reference(state, service_com, target_com, geometry):
    """One immutable folded-tool definition, including after arm deployment."""
    rs, rt = state['r']
    normal = np.array(geometry['surface_normal_target'])
    target = (np.array(geometry['surface_point_target_m'])+geometry['terminal_gap_m']*normal
              +np.array(geometry['lateral_target_m'])-target_com)
    service = np.array(geometry['folded_tcp_body_m'])-service_com
    ws, wt = (r@w for r, w in zip(state['r'], state['rates']-state['b']))
    dc = state.get('com_velocity_body', np.zeros((2, 3)))
    lt, ls = rt@target, rs@service
    offset = lt-ls
    velocity = np.cross(wt, lt)-np.cross(ws, ls)+rs@dc[0]-rt@dc[1]
    acceleration = np.cross(wt, np.cross(wt, lt))-np.cross(ws, np.cross(ws, ls))
    return offset, velocity, acceleration


def axial_gap(state, service_com, target_com, geometry, *, tcp_body_m=None):
    rs, rt = state['r']
    tcp = np.asarray(geometry['folded_tcp_body_m'] if tcp_body_m is None else tcp_body_m)
    delta = state['p'][0]-state['p'][1]+rs@(tcp-service_com)-rt@(np.array(geometry['surface_point_target_m'])-target_com)
    return float(delta@(rt@geometry['surface_normal_target']))
