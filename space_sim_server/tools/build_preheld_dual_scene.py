"""Build an already captured, side-by-side dual-arm operation candidate.

The target keeps its free joint. An explicit ideal locked grasp connects it to
the capture gripper, never to the service bus. No capture maneuver is implied.
Run in the separate geometry Python.
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import CaptureArmTarget, INITIAL_JOINTS
from simulation.scene_library.assembly import _absolutize
from simulation.scene_library.manifest import digest, initial_scene_for_model, write_json
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix


def numbers(values):
    return ' '.join(format(float(v), '.15g') for v in values)


def build(source, reference, output, target_position=(.45, -.55, -.12), cross=False, arm_mount=(0., 0., 0.)):
    source, output = Path(source).resolve(), Path(output).resolve()
    calibration = json.loads(Path(reference).read_text(encoding='utf-8'))
    row = calibration['state']
    gripper, target = [np.asarray(row['bodies'][name]) for name in ('gripper', 'capture_target')]
    rg, rt = [quaternion_wxyz_to_matrix(v[3:7]) for v in (gripper, target)]
    relative_p, relative_r = rg.T @ (target[:3] - gripper[:3]), rg.T @ rt
    target_p = np.asarray(target_position)
    # Put the operation deck on the opposite face from the capture interface.
    target_r = quaternion_wxyz_to_matrix(np.array([2**-.5, -2**-.5, 0, 0]))
    grip_r = target_r @ relative_r.T
    grip_p = target_p - grip_r @ relative_p
    capture = CaptureArmTarget(source)
    rng = np.random.default_rng(42)
    candidates = []
    for i in range(40):
        seed = np.asarray(row['q'][:6]) if i == 0 else rng.uniform(capture.lower[:6], capture.upper[:6])
        try:
            q = solve_pose(capture.kinematics, grip_p, grip_r, seed, capture.lower[:6], capture.upper[:6])
            candidates.append(q)
        except ValueError:
            continue
    if not candidates:
        raise ValueError('No capture-arm IK at this working distance')
    # Select the smallest joint displacement first; collision evidence is
    # reported below and must not be confused with full-path acceptance.
    q = min(candidates, key=lambda v: np.linalg.norm(v - np.asarray(row['q'][:6])))
    capture_q = np.r_[q, row['q'][6:]].tolist()
    root = ET.parse(source).getroot()
    _absolutize(root, source)
    service = root.find('./worldbody/body[@name="cubesat_bus"]')
    arm_offset = np.asarray(arm_mount, dtype=float)
    if arm_offset.shape != (3,) or not np.isfinite(arm_offset).all() or np.any(np.abs(arm_offset) > .4) or arm_offset[2] < 0:
        raise ValueError('Arm mount offset must be finite, within 0.4m and above the platform')
    if np.any(arm_offset):
        arm = service.find('body[@name="link1"]')
        arm.set('pos', numbers(np.fromstring(arm.get('pos', '0 0 0'), sep=' ') + arm_offset))
        mount = ET.SubElement(service, 'body', name='operating_arm_support')
        ET.SubElement(mount, 'geom', name='operating_arm_support_geom', type='capsule',
            fromto=numbers(np.r_[np.zeros(3), arm_offset]), size='.035', mass=numbers([5*np.linalg.norm(arm_offset)]),
            rgba='.35 .42 .5 1', group='2', contype='1', conaffinity='1')
    target_node = root.find('./worldbody/body[@name="capture_target"]')
    target_node.set('pos', numbers(target_p))
    target_node.set('quat', numbers(matrix_to_quaternion_wxyz(target_r)))
    # Reuse original task hardware dimensions. An explicit adapter plate spans
    # the narrower target bus; the capture interface is never scaled.
    shift = np.array([target_p[0] + .20359075 - .35,
                      target_p[1] - .34, target_p[2] + .12 + .125 + .015])
    original_model = load_geometry_model(source)
    original_data = mj.MjData(original_model)
    mj.mj_kinematics(original_model, original_data)
    # Keep the original fixed platform shell on the service spacecraft. Its
    # visual mesh supplies the delivered platform inertia; copying that shell
    # onto the target is an additional physical asset, not a zero-mass bus fix.
    platform = deepcopy(service.find('body[@name="task_box_base"]'))
    platform.set('name', 'service_platform_structure')
    for child in list(platform):
        if child.tag != 'geom':
            platform.remove(child)
        elif child.get('name'):
            child.set('name', 'service_' + child.get('name'))
    service.append(platform)
    receiver = None
    if cross:
        board = root.find('.//body[@name="asm_board1"]')
        for parent in root.iter():
            if board in list(parent):
                parent.remove(board)
                break
        receiver = ET.SubElement(service, 'body', name='service_receiver_mount')
        body = original_data.body('asm_board1')
        board.set('pos', numbers(body.xpos))
        board.set('quat', numbers(body.xquat))
        receiver.append(board)
    for name in ('task_box_base', 'drawer_base'):
        node = service.find(f'body[@name="{name}"]')
        service.remove(node)
        body = original_data.body(name)
        node.set('pos', numbers(target_r.T @ (body.xpos + shift - target_p)))
        node.set('quat', numbers(matrix_to_quaternion_wxyz(target_r.T @ body.xmat.reshape(3, 3))))
        target_node.append(node)
    for name in ('aviation_plug', 'guide_plug', 'cylindrical_plug', 'bolt'):
        node = root.find(f'./worldbody/body[@name="{name}"]')
        node.set('pos', numbers(original_data.body(name).xpos + shift))
    # Plate lies on the target's original broad face. No service-target struts.
    adapter = ET.SubElement(target_node, 'body', name='target_task_adapter')
    ET.SubElement(adapter, 'geom', name='target_task_adapter_plate', type='box',
        pos='.20359075 -.12 0', size='.24 .0075 .30', mass='1.0',
        rgba='.35 .42 .5 1', group='2', contype='1', conaffinity='1')
    # The service task-box frame is retained above, and the receiving board is
    # restored to its original pose inside that frame. Extra vertical posts are
    # unnecessary: a post at (.54, .19) blocks the aviation insertion bore.
    # Preserve the source mounting geometry instead of adding bore obstructions.
    equality = root.find('equality')
    if equality is None:
        equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'weld', name='preheld_capture_lock', body1='gripper', body2='capture_target',
        relpose=numbers(np.r_[relative_p, matrix_to_quaternion_wxyz(relative_r)]),
        solref='.005 1', solimp='.95 .99 .001', torquescale='1')
    output.mkdir(parents=True, exist_ok=False)
    model_path = output / 'preheld-dual.xml'
    ET.indent(root)
    ET.ElementTree(root).write(model_path, encoding='utf-8', xml_declaration=True)
    shutil.copyfile(source.with_name('attitude_control.json'), output / 'attitude_control.json')
    initial = initial_scene_for_model(model_path)
    initial['template_id'] = 'sarm-dual-capture26'
    initial['randomization']['target_linear_velocity_m_s'] = [0., 0., 0.]
    initial['randomization']['target_relative_velocity_m_s'] = [0., 0., 0.]
    initial['randomization']['capture_arm_joint_position_rad'] = capture_q[:6]
    initial['randomization']['preheld_capture_joint_position'] = capture_q
    initial['preheld_capture'] = dict(mode='ideal_locked_grasp', capture_body='gripper',
        target_body='capture_target', equality='preheld_capture_lock',
        target_mount_on_service_bus=False, reference=str(Path(reference).resolve()))
    model = load_geometry_model(model_path)
    data = mj.MjData(model)
    for name, value in zip([*(f'joint{i}' for i in range(1, 7)), 'ee_link11_joint', 'ee_link12_joint'],
                           initial['randomization']['arm_joint_position_rad'], strict=True):
        data.qpos[model.joint(name).qposadr[0]] = value
    capture_bodies = {n.get('name') for n in service.find('body[@name="bottom_mount"]').iter('body')}
    ranked = []
    for solution in candidates:
        values = np.r_[solution, row['q'][6:]]
        for (_, name, _), value in zip(INITIAL_JOINTS, values, strict=True):
            data.qpos[model.joint(name).qposadr[0]] = value
        mj.mj_forward(model, data)
        hits = []
        for c in data.contact:
            if c.dist >= 0:
                continue
            a, b = [model.body(int(model.geom_bodyid[g])).name for g in c.geom]
            ga, gb = [model.geom(int(g)).name for g in c.geom]
            # Only delivered jaw/ring versus X-interface contacts are intended.
            interface = ((ga.startswith('capture_x_piece_') and gb.startswith(('collision_ring_convex_', 'collision_jaw_')))
                         or (gb.startswith('capture_x_piece_') and ga.startswith(('collision_ring_convex_', 'collision_jaw_'))))
            if (a in capture_bodies) != (b in capture_bodies) and not interface:
                hits.append(float(c.dist))
        ranked.append((min(hits, default=0.), -len(hits), solution))
    best = max(ranked, key=lambda v: v[:2])
    capture_q = np.r_[best[2], row['q'][6:]].tolist()
    initial['randomization']['capture_arm_joint_position_rad'] = capture_q[:6]
    initial['randomization']['preheld_capture_joint_position'] = capture_q
    write_json(output / 'initial-scene.json', initial)
    for (_, name, _), value in zip(INITIAL_JOINTS, capture_q, strict=True):
        data.qpos[model.joint(name).qposadr[0]] = value
    mj.mj_forward(model, data)
    contacts = [dict(first=model.geom(int(c.geom[0])).name, second=model.geom(int(c.geom[1])).name,
                     separation_m=float(c.dist)) for c in data.contact]
    report = dict(schema='space-arm-dual-arm-preheld/1', model=str(model_path), model_sha256=digest(model_path),
        source_model=str(source), source_sha256=digest(source), calibration_sha256=digest(reference),
        capture_joints=capture_q, target_position_m=target_p.tolist(), task_shift_m=shift.tolist(),
        capture_interface_scaled=False, target_free_joint_preserved=True, capture_arm_dofs=6,
        service_platform_shell_retained=True,
        additional_target_platform_shell_mass_kg=float(original_model.body('task_box_base').mass[0]),
        operating_arm_mount_offset_m=arm_offset.tolist(), operating_arm_support_mass_kg=float(5*np.linalg.norm(arm_offset)),
        holding_mode='ideal_locked_grasp_at_capture_end_effector', service_target_rigid_mount=False,
        capture_arm_ik_candidates=len(candidates), capture_external_deepest_contact_m=best[0],
        cross_transfer=cross, initial_contacts=contacts, accepted=False,
        receiver_mounting='original_board_pose_in_retained_service_task_box_frame' if cross else None,
        limitation='Candidate initial configuration. No full operation or friction-only grasp acceptance.')
    write_json(output / 'assembly.json', report)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=ROOT / 'model/SARM/platform/sarm_dual_capture26.xml')
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--target-position', type=float, nargs=3, default=(.45, -.55, -.12))
    p.add_argument('--cross', action='store_true')
    p.add_argument('--arm-mount-offset', type=float, nargs=3, default=(0., 0., 0.))
    a = p.parse_args()
    result = build(a.source, a.reference, a.output, a.target_position, a.cross, a.arm_mount_offset)
    print(json.dumps(dict(model=result['model'], contacts=len(result['initial_contacts']),
                          capture_joints=result['capture_joints'])))
