"""Fit the authored -Z end face behind the unchanged world task box."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tools')]
from simulation.scene_library.assembly import _absolutize
from simulation.scene_library.manifest import digest, write_json
from simulation.scene_library.preserved_target import resize_housing, DEFAULT_PIVOT, INVENTORY
from simulation.scene_library.scaling import scale_satellite
from simulation.scene_library.taskbox_mount import _pose, _set_pose, _world_pose, extract_taskbox, restore_owner_exclusions
from simulation.scene_library.taskbox_recess import carve_pocket
from simulation.scene_library.hull_closure import add_hull_closure, audit_hull_closure, taskbox_frame_bounds


def build(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    root = ET.parse(args.baseline/'preheld-dual.xml').getroot()
    _absolutize(root, args.baseline/'preheld-dual.xml')
    old = deepcopy(root)
    original = ET.parse(args.source).getroot()
    _absolutize(original, args.source)
    resized, housing = resize_housing(original, args.scale, clear_back_mount=False)
    target = resized.find('.//body[@name="capture_target"]')
    # Previous builder cleared +X for its old mounting design. Restore those
    # parts: only this requested -Z end face may be cleared in this candidate.
    scaled, _ = scale_satellite(original, 'capture_target', args.scale, mass_rule='fixed_mass')
    scaled_target = scaled.find('.//body[@name="capture_target"]')
    shift = (1-args.scale)*np.asarray(DEFAULT_PIVOT)
    for item in scaled_target:
        if item.tag in ('body','geom','site','inertial','camera','light'):
            item.set('pos', ' '.join(map(str,np.fromstring(item.get('pos','0 0 0'),sep=' ')+shift)))
    for removed in housing['removed_geometry']:
        target.append(deepcopy(scaled_target.find(f'geom[@name="{removed["name"]}"]')))
    available = {n.get('name') for n in resized.find('asset')}
    needed = {n.get('mesh') for n in target.iter('geom')}
    for node in scaled.find('asset'):
        if node.get('name') in needed and node.get('name') not in available:
            resized.find('asset').append(deepcopy(node))
    housing['removed_geometry'] = []
    inv = json.loads(INVENTORY.read_text(encoding='utf-8'))
    leaves = {r['id']:r for r in inv if not r.get('assembly') and r.get('bounds_m')}
    end = housing['original_housing_bounds_m'][0][2]
    protected = set(housing['protected_geometry'])
    solar = target.find('body[@name="satellite_inner_panel"]')
    solar_before = ET.tostring(solar)
    solar_names = {g.get('name') for g in solar.iter('geom')}
    removed = []
    # Select actual end-mounted protruding parts by original CAD bounds.
    for record in leaves.values():
        bounds = np.asarray(record['bounds_m'])
        if bounds.mean(axis=0)[2] >= end or bounds[0,2] >= end-1e-4:
            continue
        for name in record['geoms']:
            name = 'target_'+name
            node = target.find(f'geom[@name="{name}"]')
            if node is not None and name not in protected and name not in solar_names:
                removed.append(dict(name=name, original_attributes=dict(node.attrib)))
                target.remove(node)
    hull = np.asarray(housing['housing_bounds_m'])
    # Box 550 mm edge follows hull X; 450 mm edge follows hull Y.
    mount = getattr(args, 'mount', {})
    size = mount.get('panel_size_m', [.45,.55])
    clearance = mount.get('clearance_m', .005)
    half = np.array([size[1]/2+clearance,size[0]/2+clearance])
    center = np.array([hull[1,0]-mount.get('center_from_high_x_m',.280), hull[:,1].mean(), hull[0,2]])
    low = center + [-half[0],-half[1],-clearance]
    high = center + [half[0],half[1],mount.get('recess_depth_m',.107)]
    if np.any(low[:2] < hull[0,:2]) or np.any(high[:2] > hull[1,:2]):
        raise ValueError('Uniform hull scale is too small for the unchanged task box')
    old_box = old.find('.//body[@name="task_box_base"]')
    box_p, box_r = _world_pose(old, old_box)
    work_center = box_p + box_r @ np.array([.225,.275,0.])
    # Interface +Y faces down; original -Y solar face faces up. Whole target
    # rotation changes together; no panel rehinging or task-plane movement.
    mount_basis = np.asarray(mount.get('rotation_columns', [[0,-1,0],[-1,0,0],[0,0,-1]]), dtype=float).T
    if mount_basis.shape != (3,3) or not np.allclose(mount_basis.T@mount_basis, np.eye(3)) or not np.isclose(np.linalg.det(mount_basis),1):
        raise ValueError('Mount basis must be a right-handed orthonormal frame')
    r = box_r@mount_basis
    assert np.isclose(np.linalg.det(r),1.)
    p = work_center-r@center
    _set_pose(target,p,r)
    world = root.find('worldbody')
    world.remove(root.find('./worldbody/body[@name="capture_target"]'))
    world.append(target)
    assets = root.find('asset')
    by_name = {n.get('name'):n for n in assets}
    for node in resized.find('asset'):
        name = node.get('name')
        if name in by_name:
            assets.remove(by_name[name])
        assets.append(deepcopy(node))
    used = {n.get('mesh') for n in root.iter('geom')}
    for node in list(assets):
        if node.tag == 'mesh' and node.get('name','').startswith('recess_target_') and node.get('name') not in used:
            assets.remove(node)
    # Solar components are retained exactly as uniformly scaled, not carved.
    housing['protected_geometry'] = sorted(protected | solar_names)
    root, recess = carve_pocket(root,housing,out/'recess-meshes',low,high,
                               surface_axis=2,surface=float(center[2]))
    # Clearance belongs behind the task panel, not on another exterior face.
    # The CAD payload formerly closed part of this face; removing it requires
    # an explicit continuous backing shell, not an unfilled equipment opening.
    frame_bounds = taskbox_frame_bounds(old)
    corners = np.array([[x,y,z] for x in frame_bounds[:,0] for y in frame_bounds[:,1] for z in frame_bounds[:,2]])
    corners = (box_p+corners@box_r.T-p)@r
    aperture_low,aperture_high = corners.min(0),corners.max(0)
    seam = mount.get('closure_aperture_clearance_m',.0002)
    aperture_low[:2] -= seam;aperture_high[:2] += seam
    closure = add_hull_closure(root,hull,aperture_low,aperture_high,
        thickness=mount.get('closure_thickness_m',.001),inset=mount.get('closure_inset_m',.0001))
    closure.update(taskbox_frame_bounds_m=frame_bounds.tolist(),aperture_clearance_m=seam)
    closure_audit = audit_hull_closure(root,closure)
    if not closure_audit['passed']:
        raise ValueError('Hull closure audit failed: '+str(closure_audit))
    write_json(out/'hull-closure.json',closure)
    write_json(out/'hull-closure-audit.json',closure_audit)
    target = root.find('.//body[@name="capture_target"]')
    assert ET.tostring(target.find('body[@name="satellite_inner_panel"]')) == solar_before
    for name in ('task_box_base','drawer_base'):
        node = deepcopy(old.find(f'.//body[@name="{name}"]'))
        wp, wr = _world_pose(old,old.find(f'.//body[@name="{name}"]'))
        _set_pose(node,r.T@(wp-p),r.T@wr)
        target.append(node)
    template = extract_taskbox(original)
    exclusions = restore_owner_exclusions(root,template,'capture_target')
    exclusions += restore_owner_exclusions(root,template,'cubesat_bus',prefix='service_')
    model = out/'preheld-dual.xml'
    ET.indent(root)
    ET.ElementTree(root).write(model,encoding='utf-8',xml_declaration=True)
    shutil.copyfile(args.baseline/'attitude_control.json',out/'attitude_control.json')
    scene = json.loads((args.baseline/'initial-scene.json').read_text(encoding='utf-8'))
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
    scene['randomization'].update(target_position_m=p.tolist(),target_orientation_wxyz=matrix_to_quaternion_wxyz(r).tolist())
    if getattr(args, 'capture_joint_seed', None) is not None:
        scene['randomization']['preheld_capture_joint_position'] = args.capture_joint_seed
        scene['randomization']['capture_arm_joint_position_rad'] = args.capture_joint_seed[:6]
    scene['preheld_capture']['reference'] = str((args.baseline/'initial-scene.json').resolve())
    write_json(out/'initial-scene.json',scene)
    report = dict(schema='end-face-taskbox-candidate/1',model=str(model),model_sha256=digest(model),
        source_model=str(args.source.resolve()),source_sha256=digest(args.source),
        baseline_model=str((args.baseline/'preheld-dual.xml').resolve()),
        selected_face='target -Z end face with blue round component and black rectangular assembly',
        uniform_housing_and_solar_scale=args.scale,solar_structure_and_hinge_angles_preserved=True,
        original_interface_size_and_target_local_pose_preserved=True,
        task_box_world_pose_unchanged=True,operating_arm_unchanged=True,cameras_unchanged=True,
        housing=housing,removed_end_face_geometry=removed,
        target_task_plane_center_body_m=center.tolist(),target_task_plane_center_service_m=work_center.tolist(),
        target_task_plane_normal_service=box_r[:,2].tolist(),
        pocket_bounds_target_m=[low.tolist(),high.tolist()],recess_contract=str(out/'recess-contract.json'),
        hull_closure=str(out/'hull-closure.json'),hull_closure_verified=closure_audit['passed'],
        original_owner_exclusions=exclusions,
        capture_pose_found=False,full_task_accepted=False,native_hold_verified=False)
    write_json(out/'assembly.json',report)
    print(json.dumps(dict(output=str(out),scale=args.scale,removed_geoms=len(removed),
        task_center=work_center.tolist(),target_position=p.tolist()),ensure_ascii=False),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scale',type=float,default=4.7)
    build(parser.parse_args())
