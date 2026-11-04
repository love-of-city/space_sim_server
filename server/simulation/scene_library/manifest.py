"""Resolve actual files, not historical acceptance or random seeds alone."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .catalog import recipe


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def model_dependencies(model):
    """Resolve standalone MJCF asset paths using compiler directory semantics."""
    model = Path(model).resolve()
    root = ET.parse(model).getroot()
    if root.find(".//include") is not None:
        raise ValueError("Include-based assets require an explicit dependency resolver")
    compiler = root.find("compiler")
    settings = {} if compiler is None else compiler.attrib
    paths = {model, model.with_name("attitude_control.json")}
    for node in root.findall("./asset/*"):
        directory = settings.get("meshdir" if node.tag == "mesh" else "texturedir",
                                 settings.get("assetdir", ""))
        for key, value in node.attrib.items():
            if key == "file" or key.startswith("file") and key[4:] in ("up", "down", "left", "right", "front", "back"):
                paths.add((model.parent / directory / value).resolve())
    records = []
    for path in sorted(paths):
        with path.open("rb") as stream:
            if stream.read(80).startswith(b"version https://git-lfs.github.com/spec/v1"):
                raise ValueError(f"Unresolved Git LFS asset: {path}")
        records.append(dict(path=str(path), sha256=digest(path)))
    return records


def initial_scene_for_model(model, seed=42):
    target = ET.parse(model).getroot().find('./worldbody/body[@name="capture_target"]')
    if target is None:
        raise ValueError("Source model lacks capture_target")
    return dict(template_id="sarm-task-box-20261012", seed=seed,
                   initialization_enabled=False, randomize_orbit_phase=False,
                   randomize_overview_camera=False,
                   randomization=dict(arm_joint_position_rad=[math.pi/2, -math.pi/3, math.pi/3,
                                      0., -math.pi/2, 0., 0., 0.],
                                      target_position_m=[float(v) for v in target.get("pos", "0 0 0").split()],
                                      target_orientation_wxyz=[float(v) for v in target.get("quat", "1 0 0 0").split()],
                                      target_linear_velocity_m_s=[.01, -.004, .002],
                                      target_angular_velocity_rad_s=[0., 0., 0.]))


def compose(scene_id, repository, *, seed=42):
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32:
        raise ValueError("seed must be an integer in [0, 2**32)")
    item = recipe(scene_id)
    repository = Path(repository).resolve()
    model = repository / item["source_model"]
    records = model_dependencies(model)
    initial = initial_scene_for_model(model, seed)
    return dict(schema="space-arm-scene-instance/1", recipe=item, seed=seed,
                repository=str(repository), model=str(model), dependencies=records,
                model_generated=False, assembly_status="source_model" if item["adapter"] else "not_assembled",
                initial_scene=initial, accepted=False,
                safety=dict(wrist_force_limit_n=40., wrist_torque_limit_nm=4.,
                            limits_and_native_stop_dwell_preserved=True))


def mount_inspection_passed(inspection, model_sha256, *, receiver_required=False):
    """Require complete explicit rigid-group coverage, not a success flag."""
    required = ['target', 'hardware', 'service'] + (['receiver'] if receiver_required else [])
    counts = inspection.get('geometry_counts', {})
    pairs = {('target', 'service'), ('target', 'hardware'), ('target', 'receiver'), ('receiver', 'service')}
    checks = inspection.get('checks', [])
    try:
        complete = (len(checks) == 4 and {tuple(c['groups']) for c in checks} == pairs
                    and all(c['tested_pairs'] == counts[c['groups'][0]] * counts[c['groups'][1]]
                            for c in checks))
        return (inspection.get('schema') == 'space-arm-mount-inspection/2'
                and inspection.get('model_sha256') == model_sha256
                and inspection.get('coverage_complete') is True
                and all(isinstance(counts.get(name), int) and counts[name] > 0 for name in required)
                and complete and not any(c['penetrations'] for c in checks)
                and inspection.get('unsupported_pairs') == [])
    except (KeyError, TypeError):
        return False


def compose_mounted_operation(scene_id, repository, assembly, initial_scene, *, transport_clearance_m=None,
                              mount_inspection=None, part_velocity_damping=None, transfer_offset_xy_m=None,
                              release_speed_rad_s=None):
    """Resolve an operation fixture without claiming the missing docking phase."""
    item = recipe(scene_id)
    if item['family_number'] not in (2, 4):
        raise ValueError('Mounted operation requires a supported family 2 or 4 task')
    if item['family_number'] == 4 and item['task'] == 'drawer_cycle':
        raise ValueError('Detachable drawer transfer is not implemented')
    assembly, initial_scene = Path(assembly).resolve(), Path(initial_scene).resolve()
    fixture = json.loads(assembly.read_text(encoding='utf-8-sig'))
    if (fixture.get('schema') != 'space-arm-mounted-target/1'
            or fixture.get('intended_family') != item['family_number']
            or fixture.get('initial_condition') != 'rigidly_predocked_fixture'):
        raise ValueError('Assembly does not match the requested mounted operation')
    if digest(fixture['model']) != fixture['sha256']:
        raise ValueError('Assembly model changed')
    if mount_inspection is None:
        raise ValueError('Mounted operation requires an explicit mount inspection')
    mount_inspection = Path(mount_inspection).resolve()
    inspection = json.loads(mount_inspection.read_text(encoding='utf-8-sig'))
    if not mount_inspection_passed(inspection, fixture['sha256'], receiver_required=item['family_number'] == 4):
        raise ValueError('Mount inspection is incomplete, mismatched or contains penetrations')
    if transport_clearance_m is not None and (
            not math.isfinite(transport_clearance_m) or transport_clearance_m <= 0):
        raise ValueError('Transport clearance must be finite and positive')
    if part_velocity_damping is not None and (
            not math.isfinite(part_velocity_damping) or not 0 < part_velocity_damping <= 5.
            or not item['task'].endswith('_plug_insert')):
        raise ValueError('Part velocity damping requires a plug task and a gain in (0,5]')
    if transfer_offset_xy_m is not None and (
            len(transfer_offset_xy_m) != 2 or any(not math.isfinite(v) or abs(v)>.25 for v in transfer_offset_xy_m)
            or not item['task'].endswith('_plug_insert')):
        raise ValueError('Transfer offset requires a plug task and two finite values within +/-0.25m')
    initial = json.loads(initial_scene.read_text(encoding='utf-8-sig'))
    if release_speed_rad_s is not None and (
            not math.isfinite(release_speed_rad_s) or not 0 < release_speed_rad_s <= .05
            or not item['task'].endswith('_plug_insert')):
        raise ValueError('Release speed requires a plug task and a value in (0,.05] rad/s')
    instance = dict(schema='space-arm-scene-instance/1', recipe=item,
                repository=str(Path(repository).resolve()), model=fixture['model'],
                dependencies=model_dependencies(fixture['model']), seed=initial.get('seed', 42),
                initial_scene=initial, model_generated=True, assembly_status='candidate', accepted=False,
                operation_fixture=dict(assembly=str(assembly), assembly_sha256=digest(assembly),
                    mount_inspection=str(mount_inspection), mount_inspection_sha256=digest(mount_inspection),
                    initial_scene_source=str(initial_scene), initial_scene_sha256=digest(initial_scene),
                    transport_clearance_m=transport_clearance_m,
                    scope='rigidly_predocked_operation_only', physical_docking_accepted=False))
    if part_velocity_damping is not None:
        instance['operation_fixture']['part_velocity_damping'] = part_velocity_damping
    if transfer_offset_xy_m is not None:
        instance['operation_fixture']['transfer_offset_xy_m'] = list(transfer_offset_xy_m)
    if release_speed_rad_s is not None:
        instance['operation_fixture']['release_speed_rad_s'] = release_speed_rad_s
    return instance


def preheld_inspection_passed(inspection, model_sha256, scene_sha256):
    if inspection.get('schema') == 'preserved-face-initial-audit/1':
        return _preserved_face_inspection_passed(inspection, model_sha256, scene_sha256)
    try:
        counts = inspection['geometry_counts']
        checks = inspection['checks']
        pairs = {('target_hull', 'hardware'), ('target_all', 'service')}
        return (inspection.get('schema') == 'space-arm-preheld-inspection/1'
                and inspection.get('model_sha256') == model_sha256
                and inspection.get('scene_sha256') == scene_sha256
                and inspection.get('coverage_complete') is True
                and all(isinstance(counts.get(k), int) and counts[k] > 0
                        for k in ('target_hull', 'hardware', 'target_all', 'service'))
                and len(checks) == 2 and {tuple(c['groups']) for c in checks} == pairs
                and all(c['tested_pairs'] == counts[c['groups'][0]] * counts[c['groups'][1]]
                        and not c['penetrations'] for c in checks)
                and inspection.get('unsupported_pairs') == [])
    except (KeyError, TypeError):
        return False


def _audit_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _audit_count(value, *, positive=False):
    return type(value) is int and value >= (1 if positive else 0)


def _explicit_pair_coverage(check, left_count, right_count, tolerance):
    """Recompute coverage and depth gates, including pairs rejected by broad phase."""
    try:
        total = check['total_pairs']
        separated, measured, unsupported = (check[k] for k in
            ('aabb_separated_over_5mm', 'distance_pairs', 'unsupported_pairs'))
        limit, minimum = check['penetration_tolerance_m'], check['minimum_distance_capped_5mm']
        near = check['near_zero_pair_count']
        examples = check['near_zero_examples']
        return (all(_audit_count(n) for n in (total, separated, measured, unsupported, near))
                and total == left_count * right_count
                and separated + measured + unsupported == total and unsupported == 0
                and _audit_number(limit) and 0 <= limit <= tolerance
                and _audit_number(minimum) and -limit <= minimum <= .005
                and check['penetrations'] == [] and near <= measured
                and (measured != 0 or minimum == .005)
                and (near > 0) == (minimum < 1e-7)
                and isinstance(examples, list) and len(examples) == min(near, 20)
                and all(_audit_number(row['separation_m'])
                        and minimum <= row['separation_m'] < 1e-7 for row in examples))
    except (KeyError, TypeError, ValueError):
        return False


def _audit_bounds(value):
    return (isinstance(value, list) and len(value) == 2
            and all(isinstance(row, list) and len(row) == 3 and all(_audit_number(v) for v in row) for row in value)
            and all(low <= high for low, high in zip(*value)))


def _preserved_face_inspection_passed(inspection, model_sha256, scene_sha256):
    """Accept only the complete fixed-interface audit, never its passed flag alone."""
    required_gates = {
        'collision_groups_complete', 'structural_vs_hardware_clear', 'target_vs_service_clear',
        'original_interface_penetration_under_1mm', 'no_other_arm_initial_penetration',
        'no_visual_mesh_enclosure_intersection', 'no_structural_visual_intrusion_in_taskbox_rear',
        'no_new_fiducial_primitive_intersection', 'black_frame_grid_front_clear',
        'tag_count_260', 'tag_geometry_hash_unchanged', 'tag_transform_fixed',
        'back_flush_to_hull', 'back_footprint_on_side_wall', 'drawer_inward_clear',
        'drawer_rear_gap_at_least_4p99mm', 'all_target_visual_primitive_types_supported',
        'no_structural_visual_intrusion_in_complete_hardware_envelope',
        'loose_source_slot_relative_poses_preserved', 'preserved_geometry_contract_valid',
    }
    pairs = {('target_structural', 'hardware'), ('target_all', 'service'), ('interface', 'gripper_interface')}
    try:
        if (not _audit_sha256(model_sha256) or not _audit_sha256(scene_sha256)
                or inspection['model_sha256'] != model_sha256 or inspection['scene_sha256'] != scene_sha256
                or inspection['passed'] is not True or inspection['task_accepted'] is not False):
            return False
        gates = inspection['checks']
        recessed = inspection.get('mount', {}).get('mode') == 'recessed_work_surface'
        if recessed:
            required_gates.remove('back_flush_to_hull')
            required_gates.update(('work_surface_flush_to_hull', 'recess_visual_pocket_clear'))
        if (not isinstance(gates, dict) or not required_gates <= set(gates)
                or not all(value is True for value in gates.values())):
            return False
        counts, collisions = inspection['geometry_counts'], inspection['collision_checks']
        if (set(collisions) != {a+'__'+b for a, b in pairs}
                or not all(_audit_count(counts[k], positive=True) for pair in pairs for k in pair)
                or counts['target_all'] != counts['target_structural'] + counts['hardware']
                or counts['interface'] != 898 or counts['interface'] > counts['target_structural']):
            return False
        for a, b in pairs:
            if not _explicit_pair_coverage(collisions[a+'__'+b], counts[a], counts[b], .001 if a == 'interface' else 1e-6):
                return False
        contacts = inspection['interface_contacts']
        if (not isinstance(contacts, list) or not contacts
                or not _audit_count(inspection['interface_contact_count'], positive=True)
                or inspection['interface_contact_count'] != len(contacts)):
            return False
        for row in contacts:
            names = row['geoms']
            if (len(names) != 2 or not _audit_number(row['distance_m']) or row['distance_m'] < -.001
                    or not any(n.startswith('capture_x_piece_') for n in names)
                    or not any(n.startswith(('collision_ring_convex_', 'collision_jaw_')) for n in names)):
                return False
        max_penetration = inspection['maximum_interface_contact_penetration_m']
        if (not _audit_number(max_penetration) or not 0 <= max_penetration <= .001
                or not math.isclose(max_penetration, max(max(0., -c['distance_m']) for c in contacts), abs_tol=1e-12, rel_tol=0)):
            return False
        other_count = inspection['other_arm_contact_count']
        other_limit = inspection['other_arm_penetration_tolerance_m']
        other_minimum = inspection['minimum_other_arm_contact_distance_m']
        other_contacts = inspection['all_other_arm_initial_contacts']
        if (not isinstance(other_contacts, list) or not _audit_count(other_count) or other_count != len(other_contacts)
                or not all(_audit_number(row['distance_m']) for row in other_contacts)
                or not _audit_number(other_limit) or not 0 <= other_limit <= 1e-6
                or (other_count == 0 and other_minimum is not None)
                or (other_count > 0 and (not _audit_number(other_minimum) or other_minimum < -other_limit
                    or not math.isclose(other_minimum, min(row['distance_m'] for row in other_contacts),
                                        abs_tol=1e-12, rel_tol=0)))):
            return False
        empty_evidence = ('other_arm_contacts', 'visual_enclosure_intersections',
            'structural_visual_in_taskbox_rear_volume', 'visual_nonmesh_checked_for_tags',
            'new_tag_primitive_intersections', 'tag_black_frame_occluded_rays',
            'visual_complete_hardware_envelope_intersections')
        if any(inspection[key] != [] for key in empty_evidence):
            return False
        if (not _audit_count(inspection['tag_primitive_count']) or inspection['tag_primitive_count'] != 260
                or not _audit_count(inspection['tag_black_frame_ray_count']) or inspection['tag_black_frame_ray_count'] != 144
                or not isinstance(inspection['tag_primitive_intersections'], list)
                or any(row['protected'] is not True for row in inspection['tag_primitive_intersections'])):
            return False
        mount = inspection['mount']
        face = mount.get('face', '+X')
        if face not in ('+X', '-Y'):
            return False
        back = face == '-Y'
        axis, ri, wi, plane = (1, 1, 0, (0, 2)) if back else (0, 0, 1, (1, 2))
        rear, wall, housing = (mount[k] for k in
            ('rear_back_bounds_target_m', 'actual_back_wall_bounds_target_m' if back else 'actual_X_side_wall_bounds_target_m', 'housing_bounds_target_m'))
        if not all(_audit_bounds(value) for value in (rear, wall, housing)):
            return False
        flush = mount['flush_error_m']
        reference = rear[ri][axis]
        if recessed:
            if not back:
                return False
            reference = mount['work_surface_y_target_m']
            pocket = mount['pocket_bounds_target_m']
            depth = mount['internal_depth_m']
            review = inspection['recess_visual_pocket_checks']
            if (not _audit_number(reference) or not _audit_number(depth) or not _audit_bounds(pocket)
                    or not math.isclose(depth, .102, abs_tol=1e-6, rel_tol=0)
                    or not math.isclose(rear[1][1]-reference, depth, abs_tol=1e-9, rel_tol=0)
                    or not _audit_sha256(mount['recess_contract_sha256'])
                    or pocket[0][1] >= reference or pocket[1][1] < rear[1][1]+.00499
                    or any(pocket[0][a] > rear[0][a]-.00499 or pocket[1][a] < rear[1][a]+.00499 for a in plane)
                    or review['intrusions'] != [] or not _audit_count(review['checked'], positive=True)
                    or review['checked'] != inspection['visual_primitive_count_supported']):
                return False
        if (not _audit_number(flush) or abs(flush) >= 1e-6
                or not math.isclose(flush, reference-housing[wi][axis], abs_tol=1e-9, rel_tol=0)
                or abs(reference-wall[wi][axis]) >= 1e-6 or mount['full_footprint_on_wall'] is not True
                or any(rear[0][a] < wall[0][a]-1e-7 or rear[1][a] > wall[1][a]+1e-7 for a in plane)):
            return False
        edges = mount['back_footprint_X_edge_clearances_m' if back else 'back_footprint_Y_edge_clearances_m']
        edge_axis = plane[0]
        if (not isinstance(edges, list) or len(edges) != 2 or not all(_audit_number(v) for v in edges)
                or not math.isclose(edges[0], rear[0][edge_axis]-wall[0][edge_axis], abs_tol=1e-9, rel_tol=0)
                or not math.isclose(edges[1], wall[1][edge_axis]-rear[1][edge_axis], abs_tol=1e-9, rel_tol=0)):
            return False
        drawer = inspection['drawer_inward']
        if (not all(_audit_number(drawer[k]) for k in ('q_m', 'minimum_source_z_m', 'rear_inner_source_z_m', 'rear_gap_m'))
                or not math.isclose(drawer['q_m'], -.005, abs_tol=1e-12, rel_tol=0)
                or drawer['rear_gap_m'] < .00499
                or not math.isclose(drawer['rear_gap_m'], drawer['minimum_source_z_m']-drawer['rear_inner_source_z_m'], abs_tol=1e-9, rel_tol=0)):
            return False
        # Additional raw coverage and fixed-tag contracts are checked below;
        # they are required even if every diagnostic boolean claims success.
        return _preserved_face_raw_contracts(inspection)
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
        return False


def _preserved_face_raw_contracts(inspection):
    """Recheck fixed fiducials, preserved hardware and independent pair counts."""
    # This schema describes the original four 5 cm markers, with their body-local
    # geometry preserved byte for byte while only the satellite housing grows.
    if (inspection['tag_geometry_sha256'] != 'd3c1a2f39320d3124bba89b85717cd211fdba1d0ae255dd5cdecdf51f5f02d9c'
            or inspection['tag_transform_mode'] != 'fixed_body_geometry'
            or not _audit_vector(inspection['tag_attachment_scale'])
            or inspection['tag_attachment_scale'] != [1., 1., 1.]):
        return False
    centers = {0: [.1, .0578, .15], 1: [.25, .0578, .15],
               2: [.1, .0578, -.13], 3: [.25, .0578, -.13]}
    markers = inspection['tag_markers']
    if (not isinstance(markers, list) or len(markers) != 4
            or any(type(row['id']) is not int for row in markers)
            or {row['id'] for row in markers} != set(centers)):
        return False
    for row in markers:
        if (not all(_audit_vector(row[k]) for k in ('center_m', 'right', 'down'))
                or not _audit_number(row['size_m'])
                or row['center_m'] != centers[row['id']] or row['size_m'] != .05
                or row['right'] != [-1, 0, 0] or row['down'] != [0, 0, -1]):
            return False
    visual_count, shell_count = (inspection[k] for k in ('visual_primitive_count_supported', 'enclosure_geom_count'))
    if (not _audit_count(visual_count, positive=True) or not _audit_count(shell_count, positive=True)
            or not _audit_count(inspection['visual_enclosure_triangle_checks'], positive=True)
            or inspection['visual_enclosure_triangle_checks'] != visual_count * shell_count
            or not _audit_count(inspection['visual_complete_hardware_envelope_check_count'], positive=True)
            or inspection['visual_complete_hardware_envelope_check_count'] != visual_count
            or not _audit_count(inspection['complete_hardware_geom_count'], positive=True)
            or inspection['complete_hardware_geom_count'] < inspection['geometry_counts']['hardware']
            or not _audit_bounds(inspection['complete_hardware_envelope_task_frame_m'])):
        return False
    drawer = inspection['drawer_inward']
    counts = drawer['geometry_counts']
    if (not _audit_count(counts['drawer'], positive=True) or counts['enclosure'] != shell_count
            or not _explicit_pair_coverage(drawer['collisions'], counts['drawer'], shell_count, 1e-6)):
        return False
    original = inspection['preserved_geometry']
    required = {'arms_preserved_against_current_baseline', 'cameras_preserved_against_current_baseline',
        'protected_original_interface_preserved', 'added_standard_interface_unscaled', 'original_defaults_preserved',
        'housing_only_geometry_contract_preserved', 'protected_front_parts_unscaled',
        'no_relocated_duplicate_interface', 'operating_arm_initial_q_preserved'}
    if original.get('authorized_capture_mount') is not None:
        mount = original['authorized_capture_mount']
        expected, actual = mount.get('expected_pose'), mount.get('actual_pose')
        if (mount.get('body') != 'bottom_mount' or mount.get('parent') != 'cubesat_bus'
                or mount.get('changed_scope') != 'fixed root position/quaternion only'
                or mount.get('descendant_tree_preserved') is not True
                or original.get('arm_root_owners') != {'link1': 'cubesat_bus', 'bottom_mount': 'cubesat_bus'}
                or not isinstance(expected, list) or not isinstance(actual, list) or len(expected) != 7 or len(actual) != 7
                or not all(_audit_number(v) for v in expected + actual)
                or any(abs(a-b) > 1e-10 for a, b in zip(expected, actual))
                or abs(sum(v*v for v in expected[3:])-1) > 1e-8):
            return False
        required.remove('arms_preserved_against_current_baseline')
        required.update(('arms_match_authorized_mount_contract', 'both_arm_roots_on_service_satellite'))
    if original.get('authorized_operating_mount') is not None:
        operating = original['authorized_operating_mount']
        if (operating.get('body') != 'link1' or operating.get('parent') != 'cubesat_bus'
                or original.get('arm_root_owners') != {'link1': 'cubesat_bus', 'bottom_mount': 'cubesat_bus'}
                or any(operating.get(k) is not True for k in ('source_tree_restored', 'added_support_absent',
                    'capture_tree_preserved', 'joint_limits_respected', 'gripper_q_preserved'))
                or not isinstance(operating.get('source_mount'), dict)
                or operating['source_mount'] != operating.get('actual_mount')
                or operating.get('pose_tolerance') != 1e-6
                or any(not _audit_number(operating.get(k)) or not 0 <= operating[k] <= 1e-6
                       for k in ('position_error_m', 'rotation_matrix_error'))):
            return False
        required.discard('arms_preserved_against_current_baseline')
        required.discard('arms_match_authorized_mount_contract')
        required.discard('operating_arm_initial_q_preserved')
        required.update(('arms_match_original_operating_mount_contract', 'operating_tool_initial_pose_preserved',
                         'both_arm_roots_on_service_satellite'))
    if (not _audit_sha256(original['report_sha256']) or original['model_sha256'] != inspection['model_sha256']
            or original['scale_mode'] != 'housing_only_preserved_face'
            or original['protected_interface_geometry_count'] != 899
            or original['standard_adapter_present'] is not False or original['differences'] != []
            or not isinstance(original['checks'], dict) or not required <= set(original['checks'])
            or not all(value is True for value in original['checks'].values())):
        return False
    return _loose_source_poses_preserved(inspection['loose_source_pose_contract'])


def _audit_sha256(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _audit_vector(value):
    return isinstance(value, list) and len(value) == 3 and all(_audit_number(v) for v in value)


def _audit_rotation(value):
    if not isinstance(value, list) or len(value) != 3 or not all(_audit_vector(row) for row in value):
        return False
    if any(abs(sum(value[k][i]*value[k][j] for k in range(3)) - float(i == j)) > 1e-9
           for i in range(3) for j in range(3)):
        return False
    a, b, c = value
    determinant = (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0])
                   + a[2]*(b[0]*c[1]-b[1]*c[0]))
    return abs(determinant-1.) <= 1e-9


def _loose_source_poses_preserved(contract):
    """Recompute free-part poses relative to their original pickup slots."""
    expected_anchors = {name+'_plug': (name+'_pickup', 'site') for name in ('aviation', 'guide', 'cylindrical')}
    expected_anchors['bolt'] = ('task_box_base', 'body')
    rows = contract['bodies']
    p_tol, r_tol = contract['position_tolerance_m'], contract['angle_tolerance_rad']
    if (not _audit_sha256(contract['source_model_sha256'])
            or not _audit_number(p_tol) or not 0 <= p_tol <= 1e-6
            or not _audit_number(r_tol) or not 0 <= r_tol <= 1e-6
            or not isinstance(rows, list) or len(rows) != 4
            or {row['body'] for row in rows} != set(expected_anchors)):
        return False
    for row in rows:
        if (row['anchor'], row['anchor_type']) != expected_anchors[row['body']]:
            return False
        ap, rp, ep, cp = (row[k] for k in ('anchor_world_position_m', 'source_relative_position_m',
                                          'expected_world_position_m', 'actual_world_position_m'))
        ar, rr, er, cr = (row[k] for k in ('anchor_world_rotation', 'source_relative_rotation',
                                          'expected_world_rotation', 'actual_world_rotation'))
        if not all(_audit_vector(v) for v in (ap, rp, ep, cp)) or not all(_audit_rotation(r) for r in (ar, rr, er, cr)):
            return False
        expected_p = [ap[i]+sum(ar[i][j]*rp[j] for j in range(3)) for i in range(3)]
        expected_r = [[sum(ar[i][k]*rr[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        if (any(abs(a-b) > 1e-9 for a, b in zip(ep, expected_p))
                or any(abs(er[i][j]-expected_r[i][j]) > 1e-9 for i in range(3) for j in range(3))):
            return False
        position_error = math.dist(ep, cp)
        cosine = (sum(er[i][j]*cr[i][j] for i in range(3) for j in range(3))-1.) / 2.
        angle_error = math.acos(max(-1., min(1., cosine)))
        if (not _audit_number(row['position_error_m']) or not _audit_number(row['angle_error_rad'])
                or not 0 <= row['position_error_m'] <= p_tol or not 0 <= row['angle_error_rad'] <= r_tol
                or position_error > p_tol or angle_error > r_tol
                or not math.isclose(position_error, row['position_error_m'], abs_tol=1e-12, rel_tol=0)
                # acos loses precision close to zero; independently computed
                # rotation error still has to satisfy the unchanged 1 urad gate.
                or not math.isclose(angle_error, row['angle_error_rad'], abs_tol=1e-7, rel_tol=0)):
            return False
    return True


def compose_preheld_operation(scene_id, repository, assembly, initial_scene, *,
                              mount_inspection, transport_clearance_m=None, seating_overtravel_m=None,
                              release_speed_rad_s=None, target_site=None):
    """Bind a candidate dual-arm operation to independently checked geometry."""
    item = recipe(scene_id)
    if item['family_number'] not in (2, 4) or (item['family_number'] == 4 and item['task'] == 'drawer_cycle'):
        raise ValueError('Unsupported preheld operation task')
    assembly, initial_scene, mount_inspection = map(lambda p: Path(p).resolve(),
                                                   (assembly, initial_scene, mount_inspection))
    fixture = json.loads(assembly.read_text(encoding='utf-8-sig'))
    model = Path(fixture['model'])
    initial = json.loads(initial_scene.read_text(encoding='utf-8-sig'))
    inspection = json.loads(mount_inspection.read_text(encoding='utf-8-sig'))
    if (fixture.get('schema') != 'space-arm-dual-arm-preheld/1'
            or fixture.get('cross_transfer') is not (item['family_number'] == 4)
            or fixture.get('target_free_joint_preserved') is not True
            or fixture.get('service_target_rigid_mount') is not False
            or digest(model) != fixture['model_sha256']):
        raise ValueError('Assembly does not match the dual-arm operation')
    from simulation.components.preheld_capture import validate_preheld, hold_gains
    validate_preheld(model, initial['randomization']['preheld_capture_joint_position'])
    hold_gains(initial.get('preheld_capture', {}).get('hold_gains'))
    if not preheld_inspection_passed(inspection, digest(model), digest(initial_scene)):
        raise ValueError('Preheld initial geometry inspection failed or mismatched')
    if transport_clearance_m is not None and (not math.isfinite(transport_clearance_m) or transport_clearance_m <= 0):
        raise ValueError('Transport clearance must be finite and positive')
    if seating_overtravel_m is not None and (item['task'] != 'drawer_cycle'
            or not math.isfinite(seating_overtravel_m) or not 0 < seating_overtravel_m < .002):
        raise ValueError('Drawer seating reference must be in (0, 0.002) metres')
    if release_speed_rad_s is not None and (not item['task'].endswith('_plug_insert')
            or not math.isfinite(release_speed_rad_s) or not 0 < release_speed_rad_s <= .05):
        raise ValueError('Release speed requires a plug task and a value in (0,.05] rad/s')
    if target_site is not None:
        kind = item['task'].removesuffix('_plug_insert')
        allowed = {prefix + kind + '_insert_' + fit for prefix in ('', 'service_')
                   for fit in ('40p1', '40p5')} if item['task'].endswith('_plug_insert') else set()
        if target_site not in allowed or ET.parse(model).find(f'.//site[@name="{target_site}"]') is None:
            raise ValueError('Target site must be an existing compatible insertion socket')
    instance = dict(schema='space-arm-scene-instance/1', recipe=item,
        repository=str(Path(repository).resolve()), model=str(model), dependencies=model_dependencies(model),
        seed=initial.get('seed', 42), initial_scene=initial, model_generated=True,
        assembly_status='candidate', accepted=False,
        operation_fixture=dict(assembly=str(assembly), assembly_sha256=digest(assembly),
            mount_inspection=str(mount_inspection), mount_inspection_sha256=digest(mount_inspection),
            initial_scene_source=str(initial_scene), initial_scene_sha256=digest(initial_scene),
            transport_clearance_m=transport_clearance_m, seating_overtravel_m=seating_overtravel_m,
            scope='dual_arm_preheld_operation',
            physical_docking_accepted=False, friction_only_capture_accepted=False))
    if release_speed_rad_s is not None:
        instance['operation_fixture']['release_speed_rad_s'] = release_speed_rad_s
    if target_site is not None:
        instance['operation_fixture']['target_site'] = target_site
    return instance


def verify_files(manifest):
    if manifest.get("schema") != "space-arm-scene-instance/1":
        raise ValueError("Unsupported instance schema")
    expected = recipe(manifest["recipe"]["id"])
    if manifest["recipe"] != expected:
        raise ValueError("Recipe changed; compose a new instance")
    current = model_dependencies(manifest["model"])
    if current != manifest["dependencies"]:
        raise ValueError("Model dependencies changed; compose a new instance")
    fixture = manifest.get('operation_fixture')
    if fixture is not None:
        if fixture.get('scope') == 'dual_arm_preheld_operation':
            resolved = compose_preheld_operation(expected['id'], manifest['repository'],
                fixture['assembly'], fixture['initial_scene_source'],
                transport_clearance_m=fixture['transport_clearance_m'], mount_inspection=fixture['mount_inspection'],
                seating_overtravel_m=fixture.get('seating_overtravel_m'),
                release_speed_rad_s=fixture.get('release_speed_rad_s'),
                target_site=fixture.get('target_site'))
        else:
            resolved = compose_mounted_operation(expected['id'], manifest['repository'],
            fixture['assembly'], fixture['initial_scene_source'],
            transport_clearance_m=fixture['transport_clearance_m'], mount_inspection=fixture.get('mount_inspection'),
            part_velocity_damping=fixture.get('part_velocity_damping'),
            transfer_offset_xy_m=fixture.get('transfer_offset_xy_m'),
            release_speed_rad_s=fixture.get('release_speed_rad_s'))
        if resolved != manifest:
            raise ValueError('Mounted operation configuration changed; compose a new instance')


def write_json(path, document):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
