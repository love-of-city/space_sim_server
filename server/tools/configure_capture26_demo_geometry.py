"""Derive and check the folded-tool parking reference on the unchanged model.

Run with standalone MuJoCo 3.7. Offline FK/collision queries only. This produces
motion-demo geometry, not an image-closed-loop or physical capture certificate.
"""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]
from simulation.capture26_mission import MODEL_PATH, MISSION_PATH, desired_geometry, load_vision, model_files
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.serial_chain_kinematics import SerialChainKinematics, quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz
from simulation.pose_navigation import exp


def generate():
    manifest = json.loads((ROOT/'run/native_acceleration/sarm_thread.json').read_text('utf-8-sig'))
    if mujoco.mj_version() != manifest['runtime_version']:
        raise ValueError('geometry query must use the dynamics MuJoCo version')
    mujoco.mj_loadPluginLibrary(str(ROOT/manifest['library']))
    model = mujoco.MjModel.from_xml_path(str(ROOT/MODEL_PATH))
    data = mujoco.MjData(model)
    tree, vision = ET.parse(ROOT/MODEL_PATH), load_vision(ROOT)
    geoms = [g for g in tree.iter('geom') if g.get('name', '').startswith('capture_x_piece_')]
    if len({(g.get('pos'), g.get('quat')) for g in geoms}) != 1:
        raise ValueError('interface collision pieces no longer share a frame')
    vertices = np.concatenate([np.fromstring(tree.find(f"./asset/mesh[@name='{g.get('mesh')}']").get('vertex'), sep=' ').reshape(-1, 3) for g in geoms])
    origin = np.fromstring(geoms[0].get('pos'), sep=' ')
    rotation = quaternion_wxyz_to_matrix(np.fromstring(geoms[0].get('quat'), sep=' '))
    surface = origin+rotation@np.array([0., 0., vertices.max(axis=0)[2]])
    normal = rotation[:, 2]
    kin = SerialChainKinematics.from_mjcf(ROOT/MODEL_PATH, base_body='cubesat_bus',
        joint_names=tuple('bottom_joint'+str(i) for i in range(1, 7)), tool_site='grip_frame')
    tcp = kin.forward(np.zeros(6))[0]
    _, rs = desired_geometry(1., np.eye(3), np.zeros(3), np.zeros(3), vision)
    service, target = (model.body(n).id for n in ('cubesat_bus', 'capture_target'))
    qa, qt = (model.jnt_qposadr[model.body_jntadr[b]] for b in (service, target))
    for _, name, value in INITIAL_JOINTS:
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
    data.qpos[qt:qt+7] = [0., 0., 0., 1., 0., 0., 0.]
    sg, tg = ([i for i in range(model.ngeom) if model.body_rootid[model.geom_bodyid[i]] == b
               and (model.geom_contype[i] or model.geom_conaffinity[i])] for b in (service, target))

    def place(offset, service_rotation=rs):
        data.qpos[qa:qa+3] = offset
        data.qpos[qa+3:qa+7] = matrix_to_quaternion_wxyz(service_rotation)
        mujoco.mj_forward(model, data)

    def clearance(limit=2.):
        # Spheres only reject pairs farther than the current exact minimum.
        best = limit
        for a in sg:
            bound = np.linalg.norm(data.geom_xpos[tg]-data.geom_xpos[a], axis=1)-model.geom_rbound[tg]-model.geom_rbound[a]
            for b in np.asarray(tg)[bound < best]:
                if not ((model.geom_contype[a] & model.geom_conaffinity[b]) or (model.geom_contype[b] & model.geom_conaffinity[a])):
                    continue
                best = min(best, mujoco.mj_geomDistance(model, data, a, int(b), best, None))
                if best < .10:
                    return float(best)
        return float(best)

    candidates = []
    rotations = [exp(axis*angle)@rs for axis in np.eye(3) for angle in np.linspace(-np.pi, np.pi, 17)]
    limits = np.array([np.fromstring(tree.find(f'.//joint[@name="{name}"]').get('range'), sep=' ')
                       for name in kin.joint_names])
    # The plane has two free transverse coordinates. Check nominal position
    # reachability as well as folded clearance before freezing a work position.
    for values in ((0., 0., 1.), (0., 0., 1.25), (.25, 0., 1.25),
                   (-.25, 0., 1.25), (0., 0., 1.5), (.5, 0., 1.), (.75, 0., .75)):
        lateral = np.array(values)
        body_offset = surface+.5*normal+lateral-rs@tcp
        goal = rs.T@(surface-body_offset)
        solutions = [least_squares(lambda q: kin.forward(q)[0]-goal, start, bounds=limits.T, max_nfev=200)
                     for start in (np.zeros(6), np.random.default_rng(7).uniform(*limits.T))]
        solution = min(solutions, key=lambda s: np.linalg.norm(s.fun))
        reach_error = float(np.linalg.norm(solution.fun))
        if reach_error > .001:
            candidates.append(dict(lateral_target_m=lateral.tolist(), nominal_position_ik_error_m=reach_error,
                                   sampled_clearance_lower_bound_m=None))
            continue
        place(body_offset)
        margin = clearance()
        sampled_margin = .11
        for sampled_rotation in rotations:
            place(surface+.5*normal+lateral-sampled_rotation@tcp, sampled_rotation)
            sampled_margin = min(sampled_margin, clearance(.11))
            if sampled_margin < .10:
                break
        candidates.append(dict(lateral_target_m=lateral.tolist(), nominal_clearance_m=margin,
            sampled_clearance_lower_bound_m=sampled_margin, nominal_position_ik_error_m=reach_error))
        if sampled_margin >= .10:
            break
    if candidates[-1]['sampled_clearance_lower_bound_m'] is None or candidates[-1]['sampled_clearance_lower_bound_m'] < .10:
        raise ValueError('no candidate has 10 cm basic inter-spacecraft clearance')
    place(body_offset)
    cs = rs.T@(data.subtree_com[service]-data.xpos[service])
    ct = data.subtree_com[target]-data.xpos[target]
    com_offset = body_offset+rs@cs-ct
    camera = np.array(vision['mount_center_body_m'])
    from simulation.capture26_mission import MARKER_REFERENCE_TARGET
    optical = rs@np.array(vision['body_from_optical']['left']['R_body_from_optical'])[:, 2]
    depth = float(optical@(MARKER_REFERENCE_TARGET-body_offset-rs@camera))
    result = dict(schema='space-sim-capture26-operating-geometry/1',
        model_xml_sha256=hashlib.sha256((ROOT/MODEL_PATH).read_bytes()).hexdigest(),
        model_files_sha256=model_files(ROOT),
        tool_site='grip_frame', folded_capture_joints_rad=[0.]*6,
        folded_operating_joints_rad=[0.]*6, folded_tcp_body_m=tcp.tolist(),
        grasp_surface='capture_x_piece_* outward mesh bound; reference plane, open centre',
        surface_point_target_m=surface.tolist(), surface_normal_target=normal.tolist(),
        surface_rotation_target=rotation.tolist(), terminal_gap_m=.5,
        lateral_target_m=lateral.tolist(), nominal_service_rotation_target=rs.tolist(),
        nominal_service_body_position_target_m=body_offset.tolist(),
        nominal_service_com_position_target_m=(body_offset+rs@cs).tolist(),
        nominal_target_com_position_target_m=ct.tolist(),
        nominal_relative_com_target_m=com_offset.tolist(),
        nominal_center_distance_m=float(np.linalg.norm(com_offset)), nominal_optical_depth_m=depth,
        minimum_inter_spacecraft_clearance_m=candidates[-1]['nominal_clearance_m'], candidates=candidates,
        nominal_position_ik=dict(joints_rad=solution.x.tolist(), error_m=reach_error,
            scope='position-only offline diagnostic; no orientation, deployment or grasp certification'),
        attitude_clearance_sampling=dict(axes='target X, Y, Z', angles_rad=np.linspace(-np.pi, np.pi, 17).tolist(),
            poses=len(rotations), clearance_lower_bound_m=candidates[-1]['sampled_clearance_lower_bound_m'],
            scope='sampled folded geometry only; not a continuous collision or arm reachability certificate'),
        scope='offline folded geometry; runtime checks actual trajectory, gap and contacts independently')
    path = ROOT/'config/missions/capture26-demo-geometry.json'
    path.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    task = json.loads((ROOT/MISSION_PATH).read_text('utf-8-sig'))
    task['mission'].update(acceptance_profile='simulation_state_demo_v1', feedback_source='authoritative_simulation',
        visual_hold_depth_m=5.5, position_tolerance_m=.05, velocity_tolerance_m_s=.01,
        far_position_tolerance_m=2., terminal_gap_m=.5,
        operating_geometry_file=path.relative_to(ROOT).as_posix(),
        operating_geometry_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), terminal_depth_m=depth)
    task['status'] = 'simulation_state_feedback_motion_demo'
    (ROOT/MISSION_PATH).write_text(json.dumps(task, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    generate()
