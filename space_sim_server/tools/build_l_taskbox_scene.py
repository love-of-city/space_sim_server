"""Build two complete hull-mounted task boxes: horizontal service, vertical target.

The satellite asset is uniformly scaled, task mechanisms retain their metric
calibration. A separate original-size grapple adapter preserves jaw geometry.
The target remains a free body held by the capture arm's ideal connected joint.
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
from simulation.scene_library.manifest import digest, write_json
from simulation.scene_library.scaling import scale_satellite
from simulation.scene_library.taskbox_mount import extract_taskbox, detach_taskbox, mount_taskbox
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz as matq, quaternion_wxyz_to_matrix as qmat


def numbers(v):
    return ' '.join(format(float(x), '.16g') for x in v)


def build(source, baseline_scene, baseline_model, output, *, scale=3., target_position=(1.09, -.2027, .1)):
    output.mkdir(parents=True, exist_ok=False)
    original = ET.parse(source).getroot()
    template = extract_taskbox(original)
    original_target = original.find('./worldbody/body[@name="capture_target"]')
    original_interface = [deepcopy(g) for g in original_target.findall('geom')
        if g.get('name', '').startswith('capture_x_piece_') or g.get('name') == 'target_instance_0030_part_015_color_00']
    if len(original_interface) != 899:
        raise ValueError('Expected the calibrated WF10 visual and 898 convex pieces')
    root, scaling = scale_satellite(original, 'capture_target', scale)
    detach_taskbox(root)
    _absolutize(root, source)
    service = root.find('./worldbody/body[@name="cubesat_bus"]')
    target = root.find('./worldbody/body[@name="capture_target"]')
    p = np.asarray(target_position)
    r = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    target.set('pos', numbers(p)); target.set('quat', numbers(matq(r)))
    # Keep the original satellite and its enlarged built-in port intact. The
    # separate standard-size accessory is anchored to the -X chassis sidewall.
    for g in target.findall('geom'):
        if g.get('name', '').startswith('capture_x_piece_'):
            g.set('name', 'scaled_' + g.get('name'))
    old_center = np.array([.1755, .073, .04207870013])
    # The source collision envelope extends beyond the CAD side skin. Leave
    # room for the whole ring/jaw assembly rather than disabling that envelope.
    adapter_center = np.array([.0425*scale-.09, 0., .04707870013*scale])
    adapter_r = r.copy()
    adapter_p = adapter_center-adapter_r@old_center
    adapter = ET.SubElement(target, 'body', name='capture_interface_adapter',
                            pos=numbers(adapter_p), quat=numbers(matq(adapter_r)))
    for g in original_interface:
        if not g.get('name', '').startswith('capture_x_piece_'):
            g.set('name', 'capture_adapter_visual')
        adapter.append(g)
    ET.SubElement(adapter, 'inertial', pos=numbers(old_center), mass='.4',
                  diaginertia='.00035 .00068 .00035')
    support = ET.SubElement(target, 'body', name='capture_adapter_support')
    for i,(y,z) in enumerate(((-.045,-.045),(-.045,.045),(.045,-.045),(.045,.045))):
        ET.SubElement(support, 'geom', name=f'capture_adapter_support_geom_{i}', type='capsule',
            fromto=numbers([adapter_center[0]+.01025,y,adapter_center[2]+z,
                            .06235*scale,y,adapter_center[2]+z]), size='.005',
            mass='.04', rgba='.35 .4 .45 1', contype='1', conaffinity='1', group='2')
    # Original task-box X dimension becomes vertical, Y dimension horizontal.
    # The back is 15 mm outside the upper +Y chassis rim, never on a solar wing.
    mount_r = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
    reference_plane = np.array([.35,.34,0.])
    # The drawer reaches 90 mm behind the front, plus 5 mm of inward travel.
    # Preserve 5 mm beyond the chassis collision envelope at the inward limit.
    plane_local = np.array([.1809*scale, .08325*scale+.100, .125])
    mount_p = plane_local-mount_r@reference_plane
    target_mount = mount_taskbox(root, template, 'capture_target', position=mount_p,
                                rotation=mount_r, move_loose_parts=True)
    service_mount = mount_taskbox(root, template, 'cubesat_bus', position=np.zeros(3),
                                 rotation=np.eye(3), prefix='service_', move_loose_parts=False)
    mounts = ET.SubElement(target, 'body', name='target_taskbox_mount_support')
    for i,(x,y) in enumerate(((.14,.08),(.56,.08),(.14,.60),(.56,.60))):
        back = mount_p+mount_r@np.array([x,y,-.025])
        anchor = back.copy(); anchor[1] = .04975*scale+.005
        back[1] -= .005
        ET.SubElement(mounts, 'geom', name=f'target_box_hull_standoff_{i}', type='capsule',
            fromto=numbers(np.r_[anchor,back]), size='.005', mass='.03',
            rgba='.35 .4 .45 1', contype='1', conaffinity='1', group='2')
    # Preserve the current operating-arm chassis attachment, independent of box mounting.
    old = ET.parse(baseline_model).getroot()
    for name in ('link1','operating_arm_support'):
        old_body=old.find(f'.//body[@name="{name}"]')
        if name == 'link1':
            service.find('body[@name="link1"]').set('pos', old_body.get('pos'))
        elif old_body is not None:
            service.append(deepcopy(old_body))
    rel=np.fromstring(old.find('.//weld[@name="preheld_capture_lock"]').get('relpose'),sep=' ')
    gr_old=qmat(rel[3:]).T; gp_old=-gr_old@rel[:3]
    gr_new=adapter_r@gr_old; gp_new=adapter_p+adapter_r@gp_old
    new_rel_p=-gr_new.T@gp_new; new_rel_r=gr_new.T
    equality=root.find('equality')
    if equality is None:equality=ET.SubElement(root,'equality')
    ET.SubElement(equality,'weld',name='preheld_capture_lock',body1='gripper',body2='capture_target',
        relpose=numbers(np.r_[new_rel_p,matq(new_rel_r)]),solref='.005 1',solimp='.95 .99 .001',torquescale='1')
    model_path=output/'preheld-dual.xml'
    ET.indent(root);ET.ElementTree(root).write(model_path,encoding='utf-8',xml_declaration=True)
    shutil.copy2(source.with_name('attitude_control.json'),output/'attitude_control.json')
    scene=json.loads(baseline_scene.read_text(encoding='utf-8-sig'))
    initial=scene['randomization']
    initial['target_position_m']=p.tolist();initial['target_orientation_wxyz']=matq(r).tolist()
    capture=CaptureArmTarget(model_path)
    gp=p+r@gp_new;gr=r@gr_new
    m=load_geometry_model(model_path);d=mj.MjData(m)
    seed=np.asarray(initial['preheld_capture_joint_position'][:6]);rng=np.random.default_rng(42)
    capture_bodies={b.get('name') for b in root.find('.//body[@name="bottom_mount"]').iter('body')}
    operating_bodies={b.get('name') for b in root.find('.//body[@name="link1"]').iter('body')}
    for name,v in zip([f'joint{i}' for i in range(1,7)]+['ee_link11_joint','ee_link12_joint'],initial['arm_joint_position_rad']):
        d.qpos[m.joint(name).qposadr[0]]=v
    trials=[];solutions=[]
    for attempt in range(60):
        try:q=solve_pose(capture.kinematics,gp,gr,seed if attempt==0 else rng.uniform(capture.lower[:6],capture.upper[:6]),capture.lower[:6],capture.upper[:6])
        except ValueError:
            trials.append(dict(attempt=attempt,ik=False));continue
        values=np.r_[q,initial['preheld_capture_joint_position'][6:]]
        for (_,name,_),v in zip(INITIAL_JOINTS,values):d.qpos[m.joint(name).qposadr[0]]=v
        mj.mj_forward(m,d)
        blocked=[]
        for c in d.contact:
            if c.dist>=-1e-5:continue
            names=[m.geom(int(g)).name for g in c.geom]
            bodies=[m.body(int(m.geom_bodyid[g])).name for g in c.geom]
            if any(b in operating_bodies for b in bodies):continue  # solved separately
            interface=any(n.startswith('capture_x_piece_') for n in names) and any(n.startswith(('collision_ring_convex_','collision_jaw_')) for n in names)
            if interface:continue
            if not any(b in capture_bodies for b in bodies):continue
            blocked.append(dict(geoms=names,bodies=bodies,distance_m=float(c.dist)))
        trials.append(dict(attempt=attempt,ik=True,blocked=blocked[:12],q=q.tolist()))
        if not blocked:
            solutions.append(values.tolist());break
    write_json(output/'capture-preflight.json',dict(goal_position_m=gp.tolist(),trials=trials,pose_found=bool(solutions),task_accepted=False))
    if solutions:
        initial['preheld_capture_joint_position']=solutions[0]
        initial['capture_arm_joint_position_rad']=solutions[0][:6]
    write_json(output/'initial-scene.json',scene)
    write_json(output/'assembly.json',dict(schema='space-arm-dual-arm-preheld/1',model=str(model_path.resolve()),
        model_sha256=digest(model_path),source_model=str(source.resolve()),source_sha256=digest(source),
        scaling=scaling,task_hardware_scale=1.,target_free_joint_preserved=True,service_target_rigid_mount=False,
        cross_transfer=True,complete_task_boxes=[target_mount,service_mount],capture_pose_found=bool(solutions),
        standard_capture_adapter=dict(center_target_m=adapter_center.tolist(),mass_kg=.56,original_geometry_count=899,
            old_interface_to_new_position_m=adapter_p.tolist(),old_interface_to_new_rotation=adapter_r.tolist(),
            calibration='original geometry and gripper relative pose preserved at a new rigid chassis attachment'),
        target_task_plane_center_body_m=plane_local.tolist(),target_task_plane_center_service_m=(p+r@plane_local).tolist(),
        target_task_plane_normal_service=(r@mount_r[:,2]).tolist(),service_task_plane_center_m=reference_plane.tolist(),
        target_box_mount='+Y chassis sidewall; drawer back clear of hull even at inward travel limit; original wings remain behind',
        mounting_contact='fixed standoffs meet the panel visual back and hull skin; arm collisions remain enabled',camera_changed=False,
        full_task_accepted=False,friction_only_capture_accepted=False))
    print(json.dumps(dict(model=str(model_path),capture_pose_found=bool(solutions),trials=len(trials))),flush=True)
    return bool(solutions)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--baseline-model',type=Path,required=True)
    parser.add_argument('--baseline-scene',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scale',type=float,default=3.)
    parser.add_argument('--target-position',type=float,nargs=3,default=(1.09,-.2027,.1))
    args=parser.parse_args()
    raise SystemExit(0 if build(args.source,args.baseline_scene,args.baseline_model,args.output,
        scale=args.scale,target_position=args.target_position) else 1)
