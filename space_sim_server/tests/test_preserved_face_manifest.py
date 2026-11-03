"""Registration must reject inconsistent raw audits even with green flags."""
from copy import deepcopy
import math

import pytest


@pytest.mark.parametrize('fault', [None, 'support', 'pose', 'source', 'joint_limit', 'gate'])
def test_restored_operating_mount_requires_original_tree_and_same_tool_pose(audit, fault):
    original = audit['preserved_geometry']
    original['checks'].pop('arms_preserved_against_current_baseline')
    original['checks'].pop('operating_arm_initial_q_preserved')
    original['checks'].update(arms_match_original_operating_mount_contract=True,
        operating_tool_initial_pose_preserved=True, both_arm_roots_on_service_satellite=True)
    original['arm_root_owners'] = {'link1': 'cubesat_bus', 'bottom_mount': 'cubesat_bus'}
    mount = dict(body='link1', parent='cubesat_bus', source_tree_restored=True, added_support_absent=True,
        capture_tree_preserved=True, joint_limits_respected=True, gripper_q_preserved=True,
        source_mount={'pos': '0 0 0'}, actual_mount={'pos': '0 0 0'}, pose_tolerance=1e-6,
        position_error_m=1e-7, rotation_matrix_error=5e-7)
    original['authorized_operating_mount'] = mount
    if fault == 'support': mount['added_support_absent'] = False
    elif fault == 'pose': mount['position_error_m'] = .001
    elif fault == 'source': mount['actual_mount'] = {'pos': '.1 0 0'}
    elif fault == 'joint_limit': mount['joint_limits_respected'] = False
    elif fault == 'gate': original['checks'].pop('operating_tool_initial_pose_preserved')
    assert preheld_inspection_passed(audit, MODEL_SHA, SCENE_SHA) is (fault is None)

from simulation.scene_library.manifest import preheld_inspection_passed


MODEL_SHA = 'a' * 64
SCENE_SHA = 'b' * 64


@pytest.mark.parametrize('fault', [None, 'floating_face', 'no_depth', 'filled_pocket', 'narrow_pocket'])
def test_recessed_panel_requires_flush_work_face_and_empty_internal_pocket(audit, fault):
    audit['checks'].pop('back_flush_to_hull')
    audit['checks'].update(work_surface_flush_to_hull=True, recess_visual_pocket_clear=True)
    audit['mount'] = dict(mode='recessed_work_surface', face='-Y',
        rear_back_bounds_target_m=[[-.1, -.1, -.2], [.1, -.098, .2]],
        actual_back_wall_bounds_target_m=[[-.12, -.2, -.3], [.12, -.19, .3]],
        housing_bounds_target_m=[[-.12, -.2, -.3], [.12, .1, .3]],
        flush_error_m=0., full_footprint_on_wall=True, back_footprint_X_edge_clearances_m=[.02, .02],
        work_surface_y_target_m=-.2, internal_depth_m=.102,
        pocket_bounds_target_m=[[-.105, -.205, -.205], [.105, -.093, .205]], recess_contract_sha256='c'*64)
    audit['recess_visual_pocket_checks'] = dict(checked=audit['visual_primitive_count_supported'], intrusions=[])
    if fault == 'floating_face':
        audit['mount']['work_surface_y_target_m'] -= .01
    elif fault == 'no_depth':
        audit['mount']['internal_depth_m'] = 0.
    elif fault == 'filled_pocket':
        audit['recess_visual_pocket_checks']['intrusions'] = ['uncut_wall']
    elif fault == 'narrow_pocket':
        audit['mount']['pocket_bounds_target_m'][1][0] -= .02
    assert preheld_inspection_passed(audit, MODEL_SHA, SCENE_SHA) is (fault is None)


@pytest.mark.parametrize('fault', [None, 'owner', 'different_pose', 'changed_tree', 'nonunit_rotation', 'missing_gate'])
def test_declared_capture_mount_requires_exact_pose_and_service_ownership(audit, fault):
    original = audit['preserved_geometry']
    original['checks'].pop('arms_preserved_against_current_baseline')
    original['checks'].update(arms_match_authorized_mount_contract=True, both_arm_roots_on_service_satellite=True)
    original['arm_root_owners'] = {'link1': 'cubesat_bus', 'bottom_mount': 'cubesat_bus'}
    pose = [.6, .2, -.35, 0., 2**-.5, 0., 2**-.5]
    mount = dict(body='bottom_mount', parent='cubesat_bus', expected_pose=pose[:], actual_pose=pose[:],
                 changed_scope='fixed root position/quaternion only', descendant_tree_preserved=True)
    original['authorized_capture_mount'] = mount
    if fault == 'owner':
        original['arm_root_owners']['link1'] = 'capture_target'
    elif fault == 'different_pose':
        mount['actual_pose'][0] += .01
    elif fault == 'changed_tree':
        mount['descendant_tree_preserved'] = False
    elif fault == 'nonunit_rotation':
        mount['expected_pose'][3] = mount['actual_pose'][3] = 2.
    elif fault == 'missing_gate':
        original['checks'].pop('arms_match_authorized_mount_contract')
    assert preheld_inspection_passed(audit, MODEL_SHA, SCENE_SHA) is (fault is None)


@pytest.mark.parametrize('fault', [None, 'stand_off', 'overhang', 'wrong_face'])
def test_wide_back_wall_mount_evidence(audit, fault):
    audit['mount'] = dict(face='-Y', rear_back_bounds_target_m=[[-.1, -.202, -.2], [.1, -.2, .2]],
        actual_back_wall_bounds_target_m=[[-.12, -.2, -.3], [.12, -.19, .3]],
        housing_bounds_target_m=[[-.12, -.2, -.3], [.12, .1, .3]],
        flush_error_m=0., full_footprint_on_wall=True,
        back_footprint_X_edge_clearances_m=[.02, .02])
    if fault == 'stand_off':
        audit['mount']['rear_back_bounds_target_m'][1][1] -= .01
    elif fault == 'overhang':
        audit['mount']['rear_back_bounds_target_m'][1][2] = .4
    elif fault == 'wrong_face':
        audit['mount']['face'] = '+Y'
    assert preheld_inspection_passed(audit, MODEL_SHA, SCENE_SHA) is (fault is None)


def _pairs(left, right, *, interface=False):
    total = left * right
    return dict(total_pairs=total, aabb_separated_over_5mm=total-int(interface),
        distance_pairs=int(interface), unsupported_pairs=0,
        minimum_distance_capped_5mm=-.0003 if interface else .005,
        penetrations=[], near_zero_pair_count=int(interface),
        near_zero_examples=[dict(separation_m=-.0003)] if interface else [],
        penetration_tolerance_m=.001 if interface else 1e-6)


@pytest.fixture
def audit():
    gates = ('collision_groups_complete', 'structural_vs_hardware_clear', 'target_vs_service_clear',
        'original_interface_penetration_under_1mm', 'no_other_arm_initial_penetration',
        'no_visual_mesh_enclosure_intersection', 'no_structural_visual_intrusion_in_taskbox_rear',
        'no_new_fiducial_primitive_intersection', 'black_frame_grid_front_clear',
        'tag_count_260', 'tag_geometry_hash_unchanged', 'tag_transform_fixed', 'back_flush_to_hull',
        'back_footprint_on_side_wall', 'drawer_inward_clear', 'drawer_rear_gap_at_least_4p99mm',
        'all_target_visual_primitive_types_supported',
        'no_structural_visual_intrusion_in_complete_hardware_envelope',
        'loose_source_slot_relative_poses_preserved', 'preserved_geometry_contract_valid')
    preserved = ('arms_preserved_against_current_baseline', 'cameras_preserved_against_current_baseline',
        'protected_original_interface_preserved', 'added_standard_interface_unscaled', 'original_defaults_preserved',
        'housing_only_geometry_contract_preserved', 'protected_front_parts_unscaled',
        'no_relocated_duplicate_interface', 'operating_arm_initial_q_preserved')
    identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    loose = []
    for name in ('aviation_plug', 'guide_plug', 'cylindrical_plug', 'bolt'):
        is_bolt = name == 'bolt'
        loose.append(dict(body=name, anchor='task_box_base' if is_bolt else name.removesuffix('_plug')+'_pickup',
            anchor_type='body' if is_bolt else 'site', source_relative_position_m=[0., 0., .027],
            source_relative_rotation=deepcopy(identity), anchor_world_position_m=[.1, .2, .3],
            anchor_world_rotation=deepcopy(identity), expected_world_position_m=[.1, .2, .327],
            actual_world_position_m=[.1, .2, .327], expected_world_rotation=deepcopy(identity),
            actual_world_rotation=deepcopy(identity), position_error_m=0., angle_error_rad=0.))
    return dict(schema='preserved-face-initial-audit/1', model_sha256=MODEL_SHA, scene_sha256=SCENE_SHA,
        passed=True, task_accepted=False, checks={key: True for key in gates},
        geometry_counts=dict(target_structural=900, hardware=10, target_all=910, service=4,
                             interface=898, gripper_interface=2),
        collision_checks={'target_structural__hardware': _pairs(900, 10),
            'target_all__service': _pairs(910, 4), 'interface__gripper_interface': _pairs(898, 2, interface=True)},
        interface_contact_count=1, maximum_interface_contact_penetration_m=.0003,
        interface_contacts=[dict(geoms=['capture_x_piece_1', 'collision_ring_convex_1'],
                                  bodies=['capture_target', 'gripper'], distance_m=-.0003)],
        other_arm_contacts=[], other_arm_contact_count=0, other_arm_penetration_tolerance_m=1e-6,
        minimum_other_arm_contact_distance_m=None, all_other_arm_initial_contacts=[],
        visual_enclosure_triangle_checks=10, visual_enclosure_intersections=[],
        structural_visual_in_taskbox_rear_volume=[], visual_nonmesh_checked_for_tags=[],
        visual_primitive_count_supported=2, enclosure_geom_count=5, complete_hardware_geom_count=20,
        complete_hardware_envelope_task_frame_m=[[0., 0., -.102], [.45, .55, .15]],
        visual_complete_hardware_envelope_check_count=2, visual_complete_hardware_envelope_intersections=[],
        tag_primitive_count=260, tag_primitive_intersections=[dict(primitive='original-white-border', protected=True)],
        new_tag_primitive_intersections=[], tag_black_frame_ray_count=144, tag_black_frame_occluded_rays=[],
        tag_geometry_sha256='d3c1a2f39320d3124bba89b85717cd211fdba1d0ae255dd5cdecdf51f5f02d9c',
        tag_transform_mode='fixed_body_geometry', tag_attachment_scale=[1., 1., 1.],
        tag_markers=[dict(id=i, center_m=center, size_m=.05, right=[-1, 0, 0], down=[0, 0, -1])
                     for i, center in enumerate([[.1, .0578, .15], [.25, .0578, .15],
                                                  [.1, .0578, -.13], [.25, .0578, -.13]])],
        mount=dict(rear_back_bounds_target_m=[[.28865, -.50525, -.1779], [.29065, .04475, .2721]],
            housing_bounds_target_m=[[-.97, -.54, -1.09], [.28865, .04975, 1.23]], flush_error_m=0.,
            full_footprint_on_wall=True, actual_X_side_wall_bounds_target_m=[[.26, -.51025, -1.], [.28865, .04975, 1.]],
            back_footprint_Y_edge_clearances_m=[.005, .005]),
        drawer_inward=dict(q_m=-.005, minimum_source_z_m=-.095, rear_inner_source_z_m=-.1, rear_gap_m=.005,
            collisions=_pairs(4, 5), geometry_counts=dict(drawer=4, enclosure=5)),
        preserved_geometry=dict(report_sha256='c'*64, model_sha256=MODEL_SHA,
            scale_mode='housing_only_preserved_face', protected_interface_geometry_count=899,
            standard_adapter_present=False, differences=[], checks={key: True for key in preserved}),
        loose_source_pose_contract=dict(source_model_sha256='d'*64, position_tolerance_m=1e-6,
                                       angle_tolerance_rad=1e-6, bodies=loose))


def _passed(audit):
    return preheld_inspection_passed(audit, MODEL_SHA, SCENE_SHA)


def _assign(document, path, value):
    keys = path.split('.')
    for key in keys[:-1]:
        document = document[int(key)] if isinstance(document, list) else document[key]
    document[int(keys[-1]) if isinstance(document, list) else keys[-1]] = value


def test_complete_audit_accepts_preserved_historical_white_margin_overlaps(audit):
    assert _passed(audit)
    audit['tag_primitive_intersections'] = []
    assert _passed(audit)


@pytest.mark.parametrize(('path', 'value'), [
    ('passed', False), ('passed', 1), ('task_accepted', True),
    ('model_sha256', 'x'*64), ('scene_sha256', 'x'*64),
    ('checks.back_flush_to_hull', False), ('checks.new_optional_check', False),
    ('checks.loose_source_slot_relative_poses_preserved', False),
    ('geometry_counts.target_all', 911), ('geometry_counts.interface', 897),
    ('collision_checks.target_structural__hardware.total_pairs', 8999),
    ('collision_checks.target_structural__hardware.aabb_separated_over_5mm', 8999),
    ('collision_checks.target_all__service.distance_pairs', 1),
    ('collision_checks.target_all__service.unsupported_pairs', 1),
    ('collision_checks.target_all__service.penetration_tolerance_m', 1e-5),
    ('collision_checks.interface__gripper_interface.penetration_tolerance_m', .0011),
    ('collision_checks.target_all__service.penetrations', [dict(separation_m=-.01)]),
    ('collision_checks.target_all__service.minimum_distance_capped_5mm', -.0001),
    ('collision_checks.target_all__service.total_pairs', float('inf')),
    ('collision_checks.target_all__service.distance_pairs', False),
    ('collision_checks.interface__gripper_interface.near_zero_examples.0.separation_m', -.002),
    ('collision_checks.interface__gripper_interface.near_zero_examples', []),
    ('collision_checks.target_all__service.minimum_distance_capped_5mm', 0.),
    ('interface_contact_count', 2), ('interface_contact_count', True),
    ('interface_contacts.0.distance_m', -.002), ('maximum_interface_contact_penetration_m', 0.),
    ('interface_contacts.0.geoms', ['taskbox_collision', 'collision_ring_convex_1']),
    ('other_arm_contacts', [{'distance_m': -.01}]),
    ('other_arm_penetration_tolerance_m', 1e-5), ('other_arm_contact_count', -1),
    ('other_arm_contact_count', 1), ('minimum_other_arm_contact_distance_m', -.000005),
    ('tag_geometry_sha256', 'e'*64), ('tag_transform_mode', 'mjcf_anchor'),
    ('tag_attachment_scale', [5.6, 5.6, 5.6]), ('tag_markers.0.center_m', [.11, .0578, .15]),
    ('tag_attachment_scale', [True, True, True]),
    ('tag_markers.0.size_m', .28), ('tag_markers.1.id', 0), ('tag_markers.0.id', False),
    ('tag_markers.0.right', [1, 0, 0]), ('tag_primitive_count', 259), ('tag_black_frame_ray_count', 143),
    ('tag_primitive_count', 260.),
    ('tag_primitive_intersections.0.protected', False), ('new_tag_primitive_intersections', [{'protected': True}]),
    ('tag_black_frame_occluded_rays', [{'occluder': 'new-shell'}]),
    ('visual_enclosure_triangle_checks', 9), ('visual_enclosure_intersections', [{}]),
    ('visual_nonmesh_checked_for_tags', ['unsupported_visual_type']),
    ('visual_complete_hardware_envelope_check_count', 1),
    ('visual_complete_hardware_envelope_intersections', [{}]), ('complete_hardware_geom_count', 9),
    ('complete_hardware_envelope_task_frame_m.0.0', .5),
    ('mount.actual_X_side_wall_bounds_target_m.1.1', .04),
    ('mount.rear_back_bounds_target_m.0.0', .3), ('mount.flush_error_m', 1e-4),
    ('mount.back_footprint_Y_edge_clearances_m.0', .1),
    ('drawer_inward.q_m', 0.), ('drawer_inward.rear_gap_m', .0049),
    ('drawer_inward.minimum_source_z_m', -.099), ('drawer_inward.geometry_counts.drawer', 3),
    ('drawer_inward.geometry_counts.enclosure', 4), ('drawer_inward.collisions.distance_pairs', 1),
    ('drawer_inward.collisions.penetration_tolerance_m', 1e-5),
    ('preserved_geometry.model_sha256', 'e'*64), ('preserved_geometry.scale_mode', 'uniform_whole_star'),
    ('preserved_geometry.protected_interface_geometry_count', 898),
    ('preserved_geometry.standard_adapter_present', True), ('preserved_geometry.differences', [{}]),
    ('preserved_geometry.checks.protected_front_parts_unscaled', False),
    ('preserved_geometry.checks.operating_arm_initial_q_preserved', False),
    ('loose_source_pose_contract.position_tolerance_m', .001),
    ('loose_source_pose_contract.angle_tolerance_rad', .01),
    ('loose_source_pose_contract.source_model_sha256', ''),
    ('loose_source_pose_contract.bodies.0.anchor', 'service_aviation_pickup'),
    ('loose_source_pose_contract.bodies.0.actual_world_position_m', [.1, .2, .34]),
    ('loose_source_pose_contract.bodies.0.actual_world_position_m.0', float('nan')),
    ('loose_source_pose_contract.bodies.0.source_relative_position_m', [0., 0., .1]),
    ('loose_source_pose_contract.bodies.0.position_error_m', -.001),
    ('loose_source_pose_contract.bodies.0.actual_world_rotation', [[2, 0, 0], [0, 1, 0], [0, 0, 1]]),
    ('loose_source_pose_contract.bodies.0.actual_world_rotation', [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
])
def test_green_flags_cannot_hide_inconsistent_raw_evidence(audit, path, value):
    assert _passed(audit)
    _assign(audit, path, value)
    assert not _passed(audit), path


@pytest.mark.parametrize('path', [
    'checks.loose_source_slot_relative_poses_preserved',
    'checks.no_structural_visual_intrusion_in_complete_hardware_envelope',
    'collision_checks.interface__gripper_interface', 'collision_checks.target_all__service',
    'collision_checks.target_structural__hardware.distance_pairs',
    'preserved_geometry.checks.cameras_preserved_against_current_baseline',
    'drawer_inward.geometry_counts', 'tag_markers', 'loose_source_pose_contract',
    'loose_source_pose_contract.bodies.0.actual_world_rotation',
    'other_arm_penetration_tolerance_m', 'minimum_other_arm_contact_distance_m',
    'all_other_arm_initial_contacts',
])
def test_incomplete_audit_never_registers(audit, path):
    keys = path.split('.')
    target = audit
    for key in keys[:-1]:
        target = target[int(key)] if isinstance(target, list) else target[key]
    del target[keys[-1]]
    assert not _passed(audit)


def test_pose_angle_is_recomputed_instead_of_trusting_zero_error(audit):
    theta = .001
    row = audit['loose_source_pose_contract']['bodies'][0]
    row['actual_world_rotation'] = [[math.cos(theta), -math.sin(theta), 0.],
                                    [math.sin(theta), math.cos(theta), 0.], [0., 0., 1.]]
    assert row['angle_error_rad'] == 0.
    assert not _passed(audit)


def test_moving_expected_and_actual_together_cannot_hide_wrong_slot(audit):
    row = audit['loose_source_pose_contract']['bodies'][0]
    row['actual_world_position_m'][0] += .1
    row['expected_world_position_m'][0] += .1
    assert not _passed(audit)


def test_loose_pose_check_is_rotation_and_translation_invariant(audit):
    rotation = [[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]
    for row in audit['loose_source_pose_contract']['bodies']:
        row.update(anchor_world_position_m=[10., 20., 30.], anchor_world_rotation=deepcopy(rotation),
                   expected_world_position_m=[10.027, 20., 30.], actual_world_position_m=[10.027, 20., 30.],
                   expected_world_rotation=deepcopy(rotation), actual_world_rotation=deepcopy(rotation))
    assert _passed(audit)


def test_completely_omitted_loose_part_fails_even_if_rest_match(audit):
    audit['loose_source_pose_contract']['bodies'].pop()
    assert not _passed(audit)


def test_permuted_tag_and_loose_records_have_same_meaning(audit):
    audit['tag_markers'].reverse()
    audit['loose_source_pose_contract']['bodies'].reverse()
    assert _passed(audit)


def test_unintended_arm_contact_uses_one_micrometre_tolerance(audit):
    audit['other_arm_contact_count'] = 1
    audit['minimum_other_arm_contact_distance_m'] = -0.9e-6
    audit['all_other_arm_initial_contacts'] = [dict(distance_m=-0.9e-6)]
    assert _passed(audit)
    audit['minimum_other_arm_contact_distance_m'] = -1.1e-6
    audit['all_other_arm_initial_contacts'][0]['distance_m'] = -1.1e-6
    assert not _passed(audit)


def test_zero_reported_penetration_cannot_hide_raw_arm_contact(audit):
    audit['other_arm_contact_count'] = 1
    audit['minimum_other_arm_contact_distance_m'] = 0.
    audit['all_other_arm_initial_contacts'] = [dict(distance_m=-5e-6)]
    assert not _passed(audit)
