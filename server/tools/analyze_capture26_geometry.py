"""Independent compiled geometry and IK candidates; never grants certification.

Run with standalone MuJoCo 3.7 and SciPy, outside the Basilisk process. Uses the
unchanged model and its production SDF plugin. Candidate scratch coordinates
are offline queries, never state writes to an operating simulation.
"""
import argparse
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
from simulation.capture26_mission import MODEL_PATH, desired_geometry, load_vision, MARKER_REFERENCE_TARGET
from simulation.pose_navigation import log
from simulation.serial_chain_kinematics import SerialChainKinematics, matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix
from space_arm_platform.visual_navigation import calibration_id


def plan_joint_path(clear, start, goal, limits, *, seed=7, iterations=4000):
    """Deterministic bidirectional RRT with checked joint-space edges.

    Offline candidate only: the result still needs servo, clearance, camera
    and task checks. No constraints or collision pairs are disabled.
    """
    rng = np.random.default_rng(seed)
    def edge(a, b):
        count = max(2, int(np.ceil(np.max(np.abs(b-a))/.025))+1)
        return all(clear(a+(b-a)*t) for t in np.linspace(0., 1., count))
    if not clear(start) or not clear(goal):
        return None
    if edge(start, goal):
        return [start, goal]
    trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]
    def grow(tree, destination):
        points, parents = tree
        nearest = int(np.argmin(np.linalg.norm(np.array(points)-destination, axis=1)))
        origin = points[nearest]
        direction = destination-origin
        point = origin+direction*min(1., .35/max(np.linalg.norm(direction), 1e-12))
        if not edge(origin, point):
            return None
        points.append(point)
        parents.append(nearest)
        return len(points)-1
    def trace(tree, index):
        points, parents = tree
        result = []
        while index >= 0:
            result.append(points[index])
            index = parents[index]
        return result[::-1]
    for iteration in range(iterations):
        side = iteration % 2
        sample = (trees[1-side][0][0] if rng.random() < .2
                  else rng.uniform(limits[:, 0], limits[:, 1]))
        added = grow(trees[side], sample)
        if added is None:
            continue
        point = trees[side][0][added]
        while True:
            other = grow(trees[1-side], point)
            if other is None:
                break
            if np.linalg.norm(trees[1-side][0][other]-point) < 1e-10:
                left, right = trace(trees[side], added), trace(trees[1-side], other)
                path = left+right[-2::-1] if side == 0 else right+left[-2::-1]
                # Checked shortcutting reduces unnecessary joint travel.
                for _ in range(150):
                    a, b = sorted(rng.choice(len(path), 2, replace=False))
                    if b > a+1 and edge(path[a], path[b]):
                        path[a+1:b] = []
                return path
    return None


def physical_interface_query(model, data, target, origin, rotation, lower, upper):
    """Resolve collision surfaces across the interface, including its open centre."""
    alpha = model.geom_rgba[:, 3].copy()
    samples = []
    try:
        model.geom_rgba[:, 3] = ((model.geom_contype != 0) | (model.geom_conaffinity != 0)).astype(float)
        target_r = data.xmat[target].reshape(3, 3)
        normal = target_r@rotation[:, 2]
        for x in np.linspace(lower[0], upper[0], 21):
            for y in np.linspace(lower[1], upper[1], 21):
                ray_origin = data.xpos[target]+target_r@(origin+rotation@np.array([x, y, upper[2]+1.]))
                hit = np.array([-1], dtype=np.int32)
                distance = mujoco.mj_ray(model, data, ray_origin, -normal,
                    np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8), 1, -1, hit)
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(hit[0])) if hit[0] >= 0 else None
                point = target_r.T@(ray_origin-distance*normal-data.xpos[target]) if distance >= 0 else None
                samples.append(dict(local_xy_m=[float(x), float(y)], hit_geometry=name,
                    point_target_m=None if point is None else point.tolist(),
                    interface_collision_hit=bool(name and name.startswith('capture_x_piece_'))))
    finally:
        model.geom_rgba[:, 3] = alpha
    centre = samples[len(samples)//2]
    return dict(centre_ray=centre, interface_hit_samples=sum(p['interface_collision_hit'] for p in samples),
        sample_count=len(samples), samples=samples,
        scope='physical surface samples; no grasp contact frame or terminal reference certification')


def analyze(output):
    manifest = json.loads((ROOT/'run/native_acceleration/sarm_thread.json').read_text('utf-8-sig'))
    if mujoco.mj_version() != manifest['runtime_version']:
        raise ValueError('independent geometry and native dynamics MuJoCo versions differ')
    mujoco.mj_loadPluginLibrary(str(ROOT/manifest['library']))
    path = ROOT/MODEL_PATH
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    tree, vision = ET.parse(path), load_vision(ROOT)
    geoms = [g for g in tree.iter('geom') if g.get('name', '').startswith('capture_x_piece_')]
    transforms = {(g.get('pos'), g.get('quat')) for g in geoms}
    if len(geoms) != 898 or len(transforms) != 1:
        raise ValueError('capture26 interface mesh/transform changed')
    origin = np.fromstring(geoms[0].get('pos'), sep=' ')
    interface_rotation = quaternion_wxyz_to_matrix(np.fromstring(geoms[0].get('quat'), sep=' '))
    vertices = np.concatenate([np.fromstring(tree.find(f"./asset/mesh[@name='{g.get('mesh')}']").get('vertex'), sep=' ').reshape(-1, 3) for g in geoms])
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    # Explicit candidate surface frame; its location is derived from the actual
    # mesh bounds. This still requires pad/contact and task geometry validation.
    interface_point = origin+interface_rotation@np.array([0., 0., upper[2]])
    kin = SerialChainKinematics.from_mjcf(path, base_body='cubesat_bus',
        joint_names=tuple('bottom_joint'+str(i) for i in range(1, 7)), tool_site='grip_frame')
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in kin.joint_names]
    limits = model.jnt_range[joint_ids]
    service = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cubesat_bus')
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'capture_target')
    roots = model.body_rootid
    baseline_internal = set()
    cases = []
    operating = np.deg2rad([0., -67.6, -86.6, 143.2, -85.5, 0.])
    def assign(depth, q, blend, operating_q=None):
        data.qpos[:] = model.qpos0
        offset, rs = desired_geometry(depth, np.eye(3), np.zeros(3), np.zeros(3), vision)
        for body, position, rotation in ((service, np.zeros(3), rs), (target, -offset, np.eye(3))):
            address = model.jnt_qposadr[model.body_jntadr[body]]
            data.qpos[address:address+3] = position
            data.qpos[address+3:address+7] = matrix_to_quaternion_wxyz(rotation)
        service_address = model.jnt_qposadr[model.body_jntadr[service]]
        for joint in range(model.njnt):
            if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE or model.jnt_bodyid[joint] in (service, target):
                continue
            address = model.jnt_qposadr[joint]
            data.qpos[address:address+3] = rs@(model.qpos0[address:address+3]-model.qpos0[service_address:service_address+3])
            data.qpos[address+3:address+7] = matrix_to_quaternion_wxyz(rs@quaternion_wxyz_to_matrix(model.qpos0[address+3:address+7]))
        for j, angle in zip(joint_ids, q):
            data.qpos[model.jnt_qposadr[j]] = angle*blend
        for i, angle in enumerate(operating*blend if operating_q is None else operating_q, 1):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint'+str(i))
            data.qpos[model.jnt_qposadr[j]] = angle
        for name, angle in [('axial', .02), ('cam_angle', np.pi/3)]+[(f'jaw_slide_{i}', -.012) for i in range(4)]:
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[j]] = angle
        mujoco.mj_forward(model, data)
    def contacts():
        found = set()
        for c in data.contact:
            a, b = (int(model.geom_bodyid[g]) for g in c.geom)
            if c.dist < -1e-5 and (roots[a] == service or roots[b] == service):
                found.add(tuple(sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or str(g) for g in c.geom)))
        return found
    def visibility():
        observations = {}
        for eye in ('left', 'right'):
            camera = data.xpos[service]+data.xmat[service].reshape(3, 3)@vision['body_from_optical'][eye]['position_body_m']
            clear_ids, blocking = [], set()
            for marker_id, corners in vision['layout']['markers'].items():
                points = np.array(corners)
                centre = points.mean(axis=0)
                clear_marker = True
                for point in [centre, *(centre+.98*(points-centre))]:
                    destination = data.xpos[target]+data.xmat[target].reshape(3, 3)@point
                    delta = destination-camera
                    length = np.linalg.norm(delta)
                    hit = np.array([-1], dtype=np.int32)
                    distance = mujoco.mj_ray(model, data, camera, delta/length,
                        np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8), 1, -1, hit)
                    if 0 <= distance < length-.001:
                        clear_marker = False
                        blocking.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(hit[0])))
                if clear_marker:
                    clear_ids.append(marker_id)
            observations[eye] = dict(unoccluded_marker_ids=clear_ids, blocking_geometries=sorted(blocking))
        return observations
    assign(4., np.zeros(6), 0.)
    baseline_internal = contacts()
    # Query the physical interface along its actual surface normal. Transparent
    # collision proxies are temporarily made ray-visible in this offline copy;
    # their shapes, transforms and contact constraints remain unchanged.
    physical_surface = physical_interface_query(model, data, target, origin, interface_rotation, lower, upper)
    preparation_candidates = []
    for yaw in (np.pi/2, -np.pi/2, np.pi, -np.pi):
        waypoints = [np.zeros(6), np.array([yaw, 0., 0., 0., 0., 0.]),
                     operating+np.array([yaw, 0., 0., 0., 0., 0.]), operating]
        collision_pairs, first_collision = set(), None
        for segment, (start, end) in enumerate(zip(waypoints, waypoints[1:])):
            steps = max(2, int(np.ceil(np.max(np.abs(end-start))/.035))+1)
            for blend in np.linspace(0., 1., steps):
                assign(4., np.zeros(6), 0., start+(end-start)*blend)
                current = contacts()-baseline_internal
                collision_pairs |= current
                if current and first_collision is None:
                    first_collision = dict(segment=segment, blend=float(blend))
        preparation_candidates.append(dict(operating_waypoints_rad=[p.tolist() for p in waypoints],
            collision_pairs=sorted(collision_pairs), first_collision=first_collision,
            static_contact_free=not collision_pairs))
    safe_operating = next((p for p in preparation_candidates if p['static_contact_free']), None)
    if safe_operating is None:
        operating_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint'+str(i)) for i in range(1, 7)]
        def clear(q):
            assign(4., np.zeros(6), 0., q)
            return not contacts()
        path_candidate = plan_joint_path(clear, np.zeros(6), operating, model.jnt_range[operating_ids])
        if path_candidate is not None:
            safe_operating = dict(operating_waypoints_rad=[p.tolist() for p in path_candidate],
                collision_pairs=[], first_collision=None, static_contact_free=True, method='RRTConnect seed 7')
            preparation_candidates.append(safe_operating)
    for depth in (1.75, 1.5, 1.25, 1., .75, .5):
        offset, rs = desired_geometry(depth, np.eye(3), np.zeros(3), np.zeros(3), vision)
        goal = rs.T@(interface_point+np.array([0., .5, 0.])-offset)
        goal_rotation = rs.T@interface_rotation
        def residual(q):
            p, r = kin.forward(q)
            return np.r_[p-goal, .25*log(goal_rotation@r.T)]
        solutions = [least_squares(residual, start, bounds=(limits[:, 0]+1e-6, limits[:, 1]-1e-6), max_nfev=250)
                     for start in (np.zeros(6), [0., -1., -1., 1.5, -1., 0.], [0., 1., 1., -1.5, 1., 0.])]
        best = min(solutions, key=lambda s: np.linalg.norm(s.fun))
        transverse = np.zeros(3)
        assign(depth, best.x, 1.)
        initial_visibility = visibility()
        initial_common = set(initial_visibility['left']['unoccluded_marker_ids']) & set(initial_visibility['right']['unoccluded_marker_ids'])
        if depth >= 1. and len(initial_common) < 2:
            alternatives = []
            for dx, dz in ((0., .15), (0., -.15), (.15, 0.), (-.15, 0.),
                           (0., .25), (0., -.25), (.25, 0.), (-.25, 0.), (.25, .25), (-.25, -.25)):
                goal = rs.T@(interface_point+np.array([dx, .5, dz])-offset)
                for start in (best.x, np.zeros(6), [0., -1., -1., 1.5, -1., 0.]):
                    candidate = least_squares(residual, start, bounds=(limits[:, 0]+1e-6, limits[:, 1]-1e-6), max_nfev=250)
                    if np.linalg.norm(candidate.fun) > .001:
                        continue
                    assign(depth, candidate.x, 1.)
                    if contacts():
                        continue
                    seen = visibility()
                    common = set(seen['left']['unoccluded_marker_ids']) & set(seen['right']['unoccluded_marker_ids'])
                    if len(common) >= 2:
                        alternatives.append((len(common), -np.linalg.norm(candidate.x), candidate, np.array([dx, 0., dz])))
            if alternatives:
                _, _, best, transverse = max(alternatives, key=lambda a: a[:2])
        goal = rs.T@(interface_point+np.array([0., .5, 0.])+transverse-offset)
        preparation_contacts = set()
        for blend in np.linspace(0., 1., 121):
            # Sequential preparation: first the operating arm, then capture.
            # The independent operating-arm path is reported separately.
            assign(4., best.x, float(blend), operating)
            preparation_contacts |= contacts()-baseline_internal
        assign(depth, best.x, 1.)
        parked_contacts = contacts()-baseline_internal
        line_of_sight = visibility()
        p, r = kin.forward(best.x)
        cases.append(dict(optical_depth_m=depth, capture_preparation_rad=best.x.tolist(),
            operating_preparation_rad=operating.tolist(), grasp_surface_candidate_target_m=interface_point.tolist(),
            capture_tcp_gap_m=.5, position_residual_m=float(np.linalg.norm(p-goal)),
            capture_transverse_offset_target_m=transverse.tolist(),
            rotation_residual_rad=float(np.linalg.norm(log(goal_rotation@r.T))),
            new_preparation_contacts=sorted(preparation_contacts), new_parked_contacts=sorted(parked_contacts),
            line_of_sight=line_of_sight,
            static_capture_reachable=bool(np.linalg.norm(p-goal) < .001 and np.linalg.norm(log(goal_rotation@r.T)) < np.deg2rad(.3)),
            static_contact_free=bool(safe_operating) and not preparation_contacts and not parked_contacts))
    report = dict(schema='space-sim-capture26-geometry-analysis/1', model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        calibration_id=calibration_id(vision), mujoco_version=mujoco.__version__, interface_pieces=len(geoms),
        marker_reference_target_m=MARKER_REFERENCE_TARGET.tolist(), interface_origin_target_m=origin.tolist(),
        interface_rotation_target=interface_rotation.tolist(), interface_mesh_bounds_m=[lower.tolist(), upper.tolist()],
        tool_site='grip_frame', legacy_target_site_used=False, baseline_contacts=sorted(baseline_internal), cases=cases,
        physical_interface_surface_query=physical_surface,
        operating_preparation_candidates=preparation_candidates,
        certification_issued=False, integration_complete=False,
        remaining=['normal joint control preparation', 'operating task reachability', 'signed clearance and camera occlusion',
                   'image covariance and braking validation', 'continuous missions and faults'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    if safe_operating:
        candidate = next(c for c in cases if c['optical_depth_m'] == 1.)
        prepared_scan = []
        for depth in np.arange(4., .999, -.25):
            assign(float(depth), candidate['capture_preparation_rad'], 1.)
            prepared_scan.append(dict(depth_m=float(depth), line_of_sight=visibility(), contact_pairs=sorted(contacts())))
        report['prepared_arm_visibility_scan'] = prepared_scan
        output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        waypoints = [p+[0.]*6 for p in safe_operating['operating_waypoints_rad']]
        waypoints.append(operating.tolist()+candidate['capture_preparation_rad'])
        preparation = dict(schema='space-sim-capture26-preparation/1', model_sha256=report['model_sha256'],
            waypoints_rad=waypoints, maximum_rate_rad_s=.12, maximum_acceleration_rad_s2=.04,
            optical_terminal_candidate_m=1., grasp_surface_candidate_target_m=interface_point.tolist(),
            capture_transverse_offset_target_m=candidate['capture_transverse_offset_target_m'],
            tool_site='grip_frame', certification_issued=False)
        output.with_name(output.stem+'-preparation.json').write_text(json.dumps(preparation, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('cases', 'baseline_contacts', 'operating_preparation_candidates')}))
    print(json.dumps(dict(operating_candidates=[dict(contact_pairs=len(p['collision_pairs']),
        first_collision=p['first_collision'], static_contact_free=p['static_contact_free']) for p in preparation_candidates])))
    for case in cases:
        print(json.dumps({k: v for k, v in case.items() if k not in ('capture_preparation_rad', 'operating_preparation_rad', 'new_preparation_contacts', 'new_parked_contacts')}
                         | dict(preparation_contact_pairs=len(case['new_preparation_contacts']), parked_contact_pairs=len(case['new_parked_contacts']))))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    analyze(parser.parse_args().output)
