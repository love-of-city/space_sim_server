"""Read-only XML contracts for preserved arms, cameras and satellite interfaces.

No MuJoCo/native module is imported. This checks authored geometry and mesh
references; it does not prove clearance, collision-free motion or task success.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


def canonical(element):
    if element is None:
        return None
    return (element.tag, {k: semantic(v) for k, v in sorted(element.attrib.items())},
            tuple(canonical(c) for c in element))


def semantic(value):
    try:
        return tuple(round(float(v), 12) for v in value.split())
    except ValueError:
        return value


def mesh_contract(root, path, name):
    node = root.find(f'./asset/mesh[@name="{name}"]')
    if node is None:
        raise ValueError(f'Missing mesh {name}')
    compiler = root.find('compiler')
    settings = {} if compiler is None else compiler.attrib
    directory = settings.get('meshdir', settings.get('assetdir', ''))
    result = {k: semantic(v) for k, v in node.attrib.items() if k not in ('name', 'file', 'scale')}
    result['file'] = str((path.parent/directory/node.get('file')).resolve()) if node.get('file') else None
    result['scale'] = np.fromstring(node.get('scale', '1 1 1'), sep=' ')
    return result


def scaled_attributes(node, factor):
    expected = deepcopy(node)
    lengths = ['pos']
    if node.tag in ('geom', 'site'):
        lengths += ['size', 'fromto', 'margin', 'gap']
    if node.tag == 'joint' and node.get('type') == 'slide':
        lengths += ['range', 'ref', 'springref']
    for key in lengths:
        if key in expected.attrib:
            expected.set(key, ' '.join(str(float(v)*factor) for v in expected.get(key).split()))
    if node.tag in ('geom', 'inertial') and node.get('mass'):
        expected.set('mass', str(float(node.get('mass'))*factor**3))
    if node.tag == 'inertial':
        for key in ('diaginertia', 'fullinertia'):
            if node.get(key):
                expected.set(key, ' '.join(str(float(v)*factor**5) for v in node.get(key).split()))
    return expected


def camera_contract(root):
    parents = {child: parent for parent in root.iter() for child in parent}
    return {c.get('name'): {'parent': parents[c].get('name'), 'attributes': canonical(c)}
            for c in root.iter('camera')}


def verify(source_path, baseline_path, candidate_path, factor, *, baseline_scene=None, candidate_scene=None,
           housing_only=False, clear_back_mount=False, back_center_xz=None, expected_capture_mount=None, recess_contract=None,
           restore_operating_mount=False):
    source_path, baseline_path, candidate_path = map(Path, (source_path, baseline_path, candidate_path))
    source, baseline, candidate = [ET.parse(p).getroot() for p in (source_path, baseline_path, candidate_path)]
    authored_source = source
    housing_report = None
    if housing_only:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from simulation.scene_library.preserved_target import resize_housing
        source, housing_report = resize_housing(source, factor, clear_back_mount=clear_back_mount, back_center_xz=back_center_xz)
        if recess_contract is not None:
            from simulation.scene_library.taskbox_recess import apply_recess_contract
            source, _ = apply_recess_contract(source, recess_contract)
    walk_scale = 1. if housing_only else factor
    differences = []
    protected_names = ['target_instance_0030_part_015_color_00'] + [f'capture_x_piece_{i}' for i in range(898)]
    original_target = source.find('./worldbody/body[@name="capture_target"]')
    current_target = candidate.find('./worldbody/body[@name="capture_target"]')
    if original_target is None or current_target is None:
        raise ValueError('Both models need an independent capture_target body')

    def compare_node(old, new, scale, label):
        if new is None:
            differences.append({'node': label, 'reason': 'missing'})
            return
        expected = scaled_attributes(old, scale)
        expected.attrib.pop('name', None)
        actual = deepcopy(new)
        actual.attrib.pop('name', None)
        if old.get('mesh'):
            a = mesh_contract(source, source_path, old.get('mesh'))
            b = mesh_contract(candidate, candidate_path, new.get('mesh'))
            if not np.allclose(a.pop('scale')*scale, b.pop('scale'), atol=1e-12, rtol=1e-12) or a != b:
                differences.append({'node': label, 'reason': 'mesh reference or scale changed'})
            expected.attrib.pop('mesh', None)
            actual.attrib.pop('mesh', None)
        # Child plugin contents are part of geom identity; body children are
        # compared separately so added task hardware does not obscure deletions.
        if canonical(expected) != canonical(actual):
            differences.append({'node': label, 'reason': 'authored attributes or child configuration changed',
                                'expected': expected.attrib, 'actual': actual.attrib})

    original_geometry_count = 0
    def walk_body(old_body, new_body, root=False):
        nonlocal original_geometry_count
        if new_body is None:
            differences.append({'node': old_body.get('name'), 'reason': 'original satellite body missing'})
            return
        if not root:
            old_shell, new_shell = deepcopy(old_body), deepcopy(new_body)
            for n in list(old_shell): old_shell.remove(n)
            for n in list(new_shell): new_shell.remove(n)
            compare_node(old_shell, new_shell, walk_scale, old_body.get('name'))
        for old in old_body:
            if old.tag == 'body':
                walk_body(old, new_body.find(f'body[@name="{old.get("name")}"]'))
                continue
            name = old.get('name')
            if old.tag == 'geom': original_geometry_count += 1
            wanted = 'scaled_'+name if not housing_only and root and name and name.startswith('capture_x_piece_') else name
            new = (new_body.find(f'{old.tag}[@name="{wanted}"]') if name else new_body.find(old.tag))
            # A model without an added standard-size port can retain original names.
            if new is None and wanted != name:
                new = new_body.find(f'{old.tag}[@name="{name}"]')
            compare_node(old, new, walk_scale, name or old.tag)
    walk_body(original_target, current_target, root=True)
    if housing_only:
        for record in housing_report['removed_geometry']:
            if current_target.find(f'.//geom[@name="{record["name"]}"]') is not None:
                differences.append({'node': record['name'], 'reason': 'removed +X obstruction remains'})
        for name in housing_report['protected_geometry']:
            old = authored_source.find(f'.//geom[@name="{name}"]')
            new = current_target.find(f'geom[@name="{name}"]')
            if new is None:
                differences.append({'node': name, 'reason': 'protected authored geometry missing'})
            elif (old.attrib != new.attrib or
                                   tuple(canonical(c) for c in old) != tuple(canonical(c) for c in new)):
                differences.append({'node': name, 'reason': 'protected authored geometry is not byte-equivalent'})
    protected_errors = [d for d in differences if d['node'] in protected_names]
    original_errors = deepcopy(differences)
    adapter = candidate.find('.//body[@name="capture_interface_adapter"]')
    before = len(differences)
    if adapter is not None:
        for name in protected_names:
            old = original_target.find(f'geom[@name="{name}"]')
            wanted = 'capture_adapter_visual' if name == protected_names[0] else name
            compare_node(old, adapter.find(f'geom[@name="{wanted}"]'), 1., 'adapter/'+name)
    adapter_errors = differences[before:]
    arms = {}
    for name in ('link1', 'bottom_mount', 'operating_arm_support'):
        def body(tree): return tree.find(f'.//body[@name="{name}"]')
        arms[name] = dict(candidate_matches_current_baseline=canonical(body(candidate)) == canonical(body(baseline)),
            current_baseline_matches_source=canonical(body(baseline)) == canonical(body(authored_source)),
            source_mount=None if body(authored_source) is None else body(authored_source).attrib,
            baseline_mount=None if body(baseline) is None else body(baseline).attrib,
            candidate_mount=None if body(candidate) is None else body(candidate).attrib)
    checks = dict(arms_preserved_against_current_baseline=all(v['candidate_matches_current_baseline'] for v in arms.values()),
        cameras_preserved_against_current_baseline=camera_contract(candidate) == camera_contract(baseline),
        original_satellite_uniform_geometry_preserved=not original_errors,
        protected_original_interface_preserved=not protected_errors,
        added_standard_interface_unscaled=not adapter_errors,
        original_defaults_preserved=canonical(source.find('default')) == canonical(candidate.find('default')))
    parents = {child: parent for parent in candidate.iter() for child in parent}
    owners = {name: parents[candidate.find(f'.//body[@name="{name}"]')].get('name')
              if candidate.find(f'.//body[@name="{name}"]') is not None else None for name in ('link1', 'bottom_mount')}
    checks['both_arm_roots_on_service_satellite'] = all(owner == 'cubesat_bus' for owner in owners.values())
    mount_contract = None
    if expected_capture_mount is not None:
        pose = np.asarray(expected_capture_mount, dtype=float)
        if pose.shape != (7,) or not np.isfinite(pose).all() or not np.isclose(np.linalg.norm(pose[3:]), 1., atol=1e-8):
            raise ValueError('Expected capture mount needs seven finite values and a unit quaternion')
        expected = deepcopy(baseline.find('.//body[@name="bottom_mount"]'))
        actual = candidate.find('.//body[@name="bottom_mount"]')
        if expected is None or actual is None:
            raise ValueError('Missing capture mount')
        expected.set('pos', ' '.join(map(str, pose[:3])))
        expected.set('quat', ' '.join(map(str, pose[3:])))
        checks.pop('arms_preserved_against_current_baseline')
        checks['arms_match_authorized_mount_contract'] = (canonical(expected) == canonical(actual)
            and all(arms[name]['candidate_matches_current_baseline'] for name in ('link1', 'operating_arm_support')))
        actual_pose = np.r_[np.fromstring(actual.get('pos', '0 0 0'), sep=' '), np.fromstring(actual.get('quat', '1 0 0 0'), sep=' ')]
        mount_contract = dict(body='bottom_mount', parent=owners['bottom_mount'], expected_pose=pose.tolist(),
                              actual_pose=actual_pose.tolist(), changed_scope='fixed root position/quaternion only',
                              descendant_tree_preserved=tuple(canonical(n) for n in expected) == tuple(canonical(n) for n in actual))
    operating_contract = None
    if restore_operating_mount:
        restored = canonical(candidate.find('.//body[@name="link1"]')) == canonical(authored_source.find('.//body[@name="link1"]'))
        support_absent = candidate.find('.//body[@name="operating_arm_support"]') is None
        capture_unchanged = arms['bottom_mount']['candidate_matches_current_baseline']
        if expected_capture_mount is not None:
            capture_unchanged = canonical(expected) == canonical(actual)
        checks.pop('arms_preserved_against_current_baseline', None)
        checks.pop('arms_match_authorized_mount_contract', None)
        checks['arms_match_original_operating_mount_contract'] = restored and support_absent and capture_unchanged
        operating_contract = dict(body='link1', parent=owners['link1'], source_tree_restored=restored,
            added_support_absent=support_absent, capture_tree_preserved=capture_unchanged,
            source_mount=authored_source.find('.//body[@name="link1"]').attrib,
            actual_mount=candidate.find('.//body[@name="link1"]').attrib)
    if housing_only:
        checks.pop('original_satellite_uniform_geometry_preserved')
        checks['housing_only_geometry_contract_preserved'] = not original_errors
        checks['protected_front_parts_unscaled'] = not any(d['node'] in housing_report['protected_geometry'] for d in differences)
        checks['no_relocated_duplicate_interface'] = adapter is None
    if baseline_scene and candidate_scene:
        initial = [json.loads(Path(p).read_text(encoding='utf-8-sig'))['randomization'] for p in (baseline_scene, candidate_scene)]
        checks['operating_arm_initial_q_preserved'] = initial[0]['arm_joint_position_rad'] == initial[1]['arm_joint_position_rad']
        if restore_operating_mount:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from simulation.serial_chain_kinematics import SerialChainKinematics
            poses = []
            for path, state in zip((baseline_path, candidate_path), initial):
                chain = SerialChainKinematics.from_mjcf(path, base_body='cubesat_bus',
                    joint_names=tuple(f'joint{i}' for i in range(1, 7)), tool_site='sarm_ee')
                poses.append(chain.forward(np.asarray(state['arm_joint_position_rad'][:6])))
            position_error = float(np.linalg.norm(poses[0][0]-poses[1][0]))
            rotation_error = float(np.linalg.norm(poses[0][1]-poses[1][1]))
            q = initial[1]['arm_joint_position_rad']
            limits = [np.fromstring(candidate.find(f'.//joint[@name="joint{i}"]').get('range'), sep=' ') for i in range(1, 7)]
            limits_ok = len(q) == 8 and all(np.isfinite(q)) and all(lo <= v <= hi for v, (lo, hi) in zip(q[:6], limits))
            gripper_ok = initial[0]['arm_joint_position_rad'][6:] == q[6:]
            checks.pop('operating_arm_initial_q_preserved')
            checks['operating_tool_initial_pose_preserved'] = position_error <= 1e-6 and rotation_error <= 1e-6 and limits_ok and gripper_ok
            operating_contract.update(position_error_m=position_error, rotation_matrix_error=rotation_error,
                pose_tolerance=1e-6, joint_limits_respected=bool(limits_ok), gripper_q_preserved=gripper_ok)
    return dict(schema='preserved-scene-geometry/1', scope='authored XML and referenced mesh paths/scales only',
        source=str(source_path.resolve()), current_baseline=str(baseline_path.resolve()), candidate=str(candidate_path.resolve()),
        satellite_scale=factor, scale_mode='housing_only_preserved_face' if housing_only else 'whole_satellite',
        housing_changes=housing_report, checks=checks, passed=all(checks.values()), arms=arms,
        arm_root_owners=owners, authorized_capture_mount=mount_contract, authorized_operating_mount=operating_contract,
        original_satellite_geometry_count=original_geometry_count, protected_interface_geometry_count=len(protected_names),
        standard_adapter_present=adapter is not None, differences=differences,
        task_accepted=False, collision_clearance_checked=False,
        limitation='Does not validate mesh historical bytes, mounting clearance, visibility, dynamics or support strength.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'baseline', 'candidate', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--scale', type=float, required=True)
    parser.add_argument('--baseline-scene', type=Path)
    parser.add_argument('--candidate-scene', type=Path)
    parser.add_argument('--housing-only', action='store_true', help='Keep the original metric WF10/front face while resizing housing')
    parser.add_argument('--clear-back-mount', action='store_true')
    parser.add_argument('--back-center', type=float, nargs=2)
    parser.add_argument('--expected-capture-mount', type=float, nargs=7)
    parser.add_argument('--recess-contract', type=Path)
    parser.add_argument('--restore-operating-mount', action='store_true')
    args = parser.parse_args()
    report = verify(args.source, args.baseline, args.candidate, args.scale,
                    baseline_scene=args.baseline_scene, candidate_scene=args.candidate_scene,
                    housing_only=args.housing_only, clear_back_mount=args.clear_back_mount, back_center_xz=args.back_center,
                    expected_capture_mount=args.expected_capture_mount, recess_contract=args.recess_contract,
                    restore_operating_mount=args.restore_operating_mount)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'passed': report['passed'], 'checks': report['checks'], 'differences': len(report['differences'])}))
    raise SystemExit(0 if report['passed'] else 1)
