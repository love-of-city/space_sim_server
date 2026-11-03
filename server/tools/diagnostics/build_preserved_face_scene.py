"""Fit two task boxes while keeping the original capture face and real arm trees.

The capture interface and fiducial rig retain their original metric dimensions
and target-body coordinates. Only the housing is enlarged. This builder creates
an initialization candidate; it does not claim task or friction-grasp acceptance.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import CaptureArmTarget, INITIAL_JOINTS
from simulation.scene_library.assembly import _absolutize
from simulation.scene_library.manifest import digest, write_json
from simulation.scene_library.preserved_target import resize_housing
from simulation.scene_library.taskbox_mount import extract_taskbox, detach_taskbox, mount_taskbox
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz as matq, quaternion_wxyz_to_matrix as qmat


def numbers(values):
    return ' '.join(format(float(x), '.16g') for x in values)


def close_taskbox_back(root):
    """A hollow integral enclosure, with 5 mm rear travel clearance, not a stand."""
    parent = root.find('.//body[@name="task_box_base"]')
    shell = ET.SubElement(parent, 'body', name='target_taskbox_enclosure')
    # Original panel back is -25 mm; drawer reaches -90 mm plus 5 mm inward.
    # Rear inner face -100 mm, outer face -102 mm. All units are metres.
    pieces = [
        ('back', [.225, .275, -.101], [.225, .275, .001]),
        ('x_low', [.001, .275, -.0625], [.001, .275, .0375]),
        ('x_high', [.449, .275, -.0625], [.001, .275, .0375]),
        ('y_low', [.225, .001, -.0625], [.223, .001, .0375]),
        ('y_high', [.225, .549, -.0625], [.223, .001, .0375]),
    ]
    for name, pos, size in pieces:
        ET.SubElement(shell, 'geom', name='taskbox_enclosure_' + name,
                      type='box', pos=numbers(pos), size=numbers(size),
                      rgba='.6 .64 .67 1', density='2700',
                      contype='1', conaffinity='1', group='2')
    return dict(kind='hollow_integral_taskbox_enclosure', thickness_m=.002,
                front_to_back_m=.102, minimum_drawer_rear_clearance_m=.005,
                material_density_assumption_kg_m3=2700, support_standoffs=False)


def set_initial(model, data, initial):
    for name, value in zip([*(f'joint{i}' for i in range(1, 7)),
                            'ee_link11_joint', 'ee_link12_joint'], initial['arm_joint_position_rad']):
        data.qpos[model.joint(name).qposadr[0]] = value
    address = int(model.joint('capture_target_free').qposadr[0])
    data.qpos[address:address+3] = initial['target_position_m']
    data.qpos[address+3:address+7] = initial['target_orientation_wxyz']


def build(args):
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    original = ET.parse(args.source).getroot()
    template = extract_taskbox(original)
    root, housing = resize_housing(original, factor=args.housing_scale,
                                    pivot=[.28865, .04975, .04207870013], clear_back_mount=getattr(args, 'clear_back_mount', False),
                                    back_center_xz=getattr(args, 'back_center', None))
    detach_taskbox(root)
    _absolutize(root, args.source)
    target = root.find('./worldbody/body[@name="capture_target"]')
    service = root.find('./worldbody/body[@name="cubesat_bus"]')
    baseline = ET.parse(args.baseline_model).getroot()
    old_service = baseline.find('./worldbody/body[@name="cubesat_bus"]')
    capture_mount = service.find('body[@name="bottom_mount"]')
    requested_mount = getattr(args, 'capture_mount', None)
    if requested_mount is not None:
        if capture_mount is None or not np.all(np.isfinite(requested_mount)) or not np.isclose(np.linalg.norm(requested_mount[3:]), 1., atol=1e-8):
            raise ValueError('Capture mount needs a service-owned root and a finite unit quaternion')
        capture_mount.set('pos', numbers(requested_mount[:3]))
        capture_mount.set('quat', numbers(requested_mount[3:]))
    for name in ('link1', 'operating_arm_support'):
        current = service.find(f'body[@name="{name}"]')
        if current is not None:
            service.remove(current)
        previous = old_service.find(f'body[@name="{name}"]')
        if previous is not None:
            service.append(deepcopy(previous))

    # Source task X becomes target +Z, Y becomes -Y, normal becomes +X.
    mount_r = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])
    plane_local = np.array([.28865 + .102, -.23025, .04707870013])
    reference_plane = np.array([.35, .34, 0.])
    mount_p = plane_local - mount_r @ reference_plane
    # Loose donor parts are world free bodies: their initial poses must be
    # composed with the final owner pose at mounting time.
    r = np.diag([-1., -1., 1.])
    if getattr(args, 'mount_face', 'side') == 'back':
        # The 45 cm short edge follows hull X; the 55 cm long edge follows
        # hull Z. Roll the spacecraft so its long axis lies horizontally.
        bounds = np.asarray(housing['housing_bounds_m'])
        mount_r = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        flip = np.diag([-1., 1., -1.]) if getattr(args, 'back_flip', False) else np.eye(3)
        mount_r = flip @ mount_r
        center = bounds.mean(axis=0)
        center[[0, 2]] = housing['back_mount_center_xz_m']
        center[1] = bounds[0, 1] - (0. if getattr(args, 'recess_taskbox', False) else .102)
        plane_local = center
        mount_p = plane_local - mount_r @ reference_plane
        r = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]) @ flip
    yaw = np.deg2rad(getattr(args, 'task_yaw', 0.))
    r = np.array([[np.cos(yaw), -np.sin(yaw), 0.], [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]]) @ r
    world_plane = np.asarray(args.task_plane)
    p = world_plane - r @ plane_local
    target.set('pos', numbers(p))
    target.set('quat', numbers(matq(r)))
    recess_report = None
    if getattr(args, 'recess_taskbox', False):
        if getattr(args, 'mount_face', 'side') != 'back':
            raise ValueError('Recess installation requires the wide rear wall')
        from simulation.scene_library.taskbox_recess import carve_rear_pocket
        root, recess_report = carve_rear_pocket(root, housing, output/'recess-meshes')
    target_mount = mount_taskbox(root, template, 'capture_target', position=mount_p,
                                rotation=mount_r, move_loose_parts=True)
    enclosure = close_taskbox_back(root)
    if recess_report is not None:
        enclosure['kind'] = 'internal_recess_liner'
        enclosure['work_surface_protrusion_m'] = 0.
        enclosure['recess_contract'] = str((output/'recess-contract.json').resolve())
    service_mount = mount_taskbox(root, template, 'cubesat_bus', position=np.zeros(3),
                                 rotation=np.eye(3), prefix='service_', move_loose_parts=False)

    # Use the original interface calibration, never the v4 relocated adapter.
    grasp = ET.parse(args.grasp_model).getroot().find('./equality/weld[@name="preheld_capture_lock"]')
    if grasp is None:
        raise ValueError('Missing calibrated original-interface lock')
    equality = root.find('equality')
    if equality is None:
        equality = ET.SubElement(root, 'equality')
    prior_lock = equality.find('weld[@name="preheld_capture_lock"]')
    if prior_lock is not None:
        equality.remove(prior_lock)
    equality.append(deepcopy(grasp))
    rel = np.fromstring(grasp.get('relpose'), sep=' ')
    gr_local = qmat(rel[3:]).T
    gp_local = -gr_local @ rel[:3]
    goal_p, goal_r = p + r @ gp_local, r @ gr_local
    model_path = output / 'preheld-dual.xml'
    ET.indent(root)
    ET.ElementTree(root).write(model_path, encoding='utf-8', xml_declaration=True)
    shutil.copy2(args.source.with_name('attitude_control.json'), output / 'attitude_control.json')
    scene = json.loads(args.baseline_scene.read_text(encoding='utf-8-sig'))
    scene['render_profile'] = json.loads((ROOT / 'config/visual_attachments/protected_capture_face.json').read_text(encoding='utf-8'))
    initial = scene['randomization']
    initial['target_position_m'], initial['target_orientation_wxyz'] = p.tolist(), matq(r).tolist()
    assembly = dict(schema='space-arm-dual-arm-preheld/1', model=str(model_path.resolve()),
        model_sha256=digest(model_path), source_model=str(args.source.resolve()), source_sha256=digest(args.source),
        housing=housing, target_free_joint_preserved=True, service_target_rigid_mount=False,
        cross_transfer=True, complete_task_boxes=[target_mount, service_mount], taskbox_enclosure=enclosure,
        capture_interface_unchanged=True, fiducials_fixed_body_coordinates=True,
        target_task_plane_center_body_m=plane_local.tolist(), target_task_plane_center_service_m=world_plane.tolist(),
        target_task_plane_normal_service=(r @ mount_r[:, 2]).tolist(), service_task_plane_center_m=reference_plane.tolist(),
        target_box_mount=('-Y wide back wall; work surface flush, rear mechanism and liner recessed inside housing' if recess_report is not None else
                         '-Y wide back wall; integral hollow back flush to the wall' if getattr(args, 'mount_face', 'side') == 'back' else '+X chassis sidewall; integral hollow back flush to the wall'),
        mount_face=getattr(args, 'mount_face', 'side'),
        recessed_taskbox=recess_report is not None,
        capture_mount_override=requested_mount,
        arm_root_owners={'operating': 'cubesat_bus', 'capture': 'cubesat_bus'},
        capture_pose_found=False, camera_changed=False, operating_arm_pose_changed=False,
        full_task_accepted=False, friction_only_capture_accepted=False)
    if args.geometry_only:
        write_json(output / 'initial-scene.json', scene)
        write_json(output / 'assembly.json', assembly)
        print(json.dumps(dict(model=str(model_path), geometry_only=True)), flush=True)
        return True

    print('Loading geometry for actual-arm IK and contact screening', flush=True)
    capture = CaptureArmTarget(model_path)
    model = load_geometry_model(model_path)
    data = mj.MjData(model)
    set_initial(model, data, initial)
    capture_bodies = {b.get('name') for b in root.find('.//body[@name="bottom_mount"]').iter('body')}
    operating_bodies = {b.get('name') for b in root.find('.//body[@name="link1"]').iter('body')}
    rng = np.random.default_rng(42)
    seed = np.asarray(initial['preheld_capture_joint_position'][:6])
    if getattr(args, 'capture_seed', None) is not None:
        seed = np.asarray(args.capture_seed)
    trials = []
    solution = None
    for attempt in range(args.attempts):
        try:
            q = solve_pose(capture.kinematics, goal_p, goal_r,
                           seed if attempt == 0 else rng.uniform(capture.lower[:6], capture.upper[:6]),
                           capture.lower[:6], capture.upper[:6])
        except ValueError:
            trials.append(dict(attempt=attempt, ik=False))
            continue
        values = np.r_[q, initial['preheld_capture_joint_position'][6:]]
        for (_, name, _), value in zip(INITIAL_JOINTS, values):
            data.qpos[model.joint(name).qposadr[0]] = value
        mj.mj_forward(model, data)
        blocked, interface_contacts = [], []
        for c in data.contact:
            names = [model.geom(int(g)).name for g in c.geom]
            bodies = [model.body(int(model.geom_bodyid[g])).name for g in c.geom]
            if not any(b in capture_bodies or b in operating_bodies for b in bodies):
                continue
            expected = any(n.startswith('capture_x_piece_') for n in names) and any(n.startswith(('collision_ring_convex_', 'collision_jaw_')) for n in names)
            row = dict(geoms=names, bodies=bodies, distance_m=float(c.dist))
            if expected:
                interface_contacts.append(row)
                # Expected contact is still bounded, not an unlimited exemption.
                if c.dist < -.001:
                    blocked.append(row)
            elif c.dist < -1e-5:
                blocked.append(row)
        trials.append(dict(attempt=attempt, ik=True, blocked=blocked[:20],
                           interface_contacts=interface_contacts, q=q.tolist()))
        if attempt % 10 == 0 or not blocked:
            print(json.dumps(dict(attempt=attempt, blocked=len(blocked), interface_contacts=len(interface_contacts))), flush=True)
        if not blocked:
            solution = values.tolist()
            break
    write_json(output / 'capture-preflight.json', dict(goal_position_m=goal_p.tolist(),
        goal_rotation=goal_r.tolist(), trials=trials, pose_found=solution is not None,
        interface_penetration_limit_m=.001, full_task_accepted=False))
    if solution is not None:
        initial['preheld_capture_joint_position'] = solution
        initial['capture_arm_joint_position_rad'] = solution[:6]
    write_json(output / 'initial-scene.json', scene)
    assembly['capture_pose_found'] = solution is not None
    write_json(output / 'assembly.json', assembly)
    print(json.dumps(dict(model=str(model_path), capture_pose_found=solution is not None, trials=len(trials))), flush=True)
    return solution is not None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--baseline-model', type=Path, required=True)
    parser.add_argument('--baseline-scene', type=Path, required=True)
    parser.add_argument('--grasp-model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--housing-scale', type=float, default=5.6)
    parser.add_argument('--mount-face', choices=('side', 'back'), default='side')
    parser.add_argument('--back-flip', action='store_true')
    parser.add_argument('--recess-taskbox', action='store_true', help='Work surface flush with rear hull; carve actual mesh and collision pocket')
    parser.add_argument('--back-center', type=float, nargs=2, metavar=('X', 'Z'), help='Task box center on target-local back wall')
    parser.add_argument('--capture-seed', type=float, nargs=6)
    parser.add_argument('--capture-mount', type=float, nargs=7, metavar=('X', 'Y', 'Z', 'QW', 'QX', 'QY', 'QZ'), help='Explicit service-local capture root pose; only this fixed root transform changes')
    parser.add_argument('--clear-back-mount', action='store_true')
    parser.add_argument('--task-yaw', type=float, default=0., help='Horizontal orientation of the target task plane, degrees')
    parser.add_argument('--task-plane', type=float, nargs=3, default=(.74025, .49, .225))
    parser.add_argument('--attempts', type=int, default=160)
    parser.add_argument('--geometry-only', action='store_true')
    raise SystemExit(0 if build(parser.parse_args()) else 1)
