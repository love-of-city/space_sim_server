"""Independent predicates must survive rigid task-face mounting changes."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.evaluation.scoring import PhysicalScorer
from simulation.policy_task_success import DrawerCycleSuccess, EVALUATION_PERIOD_NS
from simulation.serial_chain_kinematics import axis_angle_to_matrix
from test_drawer_task_faces import rotated_drawer
from test_unified_drawer_scoring import MODEL, cycle, monitor, policy_sample


ROTATIONS = [np.eye(3), axis_angle_to_matrix(np.array([0., 1., 0.]), np.pi/2),
             axis_angle_to_matrix(np.array([0., 1., 0.]), -np.pi/2),
             axis_angle_to_matrix(np.array([1., 2., 3.])/np.sqrt(14.), .7)]


def plug_spec(donor_rotation=None):
    return dict(task='guide_plug_insert', final_tool_position=[.1, 0., .2],
                hole=[0., 0., 0.], hole_rotation=np.eye(3).tolist(),
                pickup=[-.2, 0., 0.], pickup_rotation=(np.eye(3) if donor_rotation is None else donor_rotation).tolist(),
                tip=.095, flange=.025, keyed=True)


def plug_rows(spec, retreat=None):
    donor=np.asarray(spec['pickup']);donor_r=np.asarray(spec['pickup_rotation'])
    hole=np.asarray(spec['hole']);hole_r=np.asarray(spec['hole_rotation'])
    tool=hole+hole_r@np.array([.1, 0., .2] if retreat is None else retreat)
    for i in range(450):
        grip=i<40;opening=.4 if grip else .65
        pos=(donor+donor_r[:, 2]*(.025 if i<10 else .075)) if i<20 else hole+.025*hole_r[:, 2]
        rot=(donor_r if i<20 else hole_r)@np.diag([1., -1., -1.])
        yield dict(time_s=i*.005, joint_position=[0.]*6+[opening, -opening],
                   tool_position_m=tool.tolist(), contacts=[1, 1] if grip else [0, 0],
                   wrench=[0.]*6, wrench_valid=True, plug_position_m=pos.tolist(),
                   plug_rotation=rot.tolist(), plug_velocity_m_s=[0.]*3,
                   plug_angular_velocity_rad_s=[0.]*3)


def rigid_spec(spec, rotation, shift):
    result=deepcopy(spec)
    for key in ('hole', 'pickup', 'final_tool_position'):
        result[key]=(rotation@np.asarray(spec[key])+shift).tolist()
    for key in ('hole_rotation', 'pickup_rotation'):
        result[key]=(rotation@np.asarray(spec[key])).tolist()
    return result


def rigid_row(row, rotation, shift):
    result=deepcopy(row)
    for key in ('plug_position_m', 'tool_position_m', 'reference_position_m',
                'pickup_position_m', 'hole_position_m'):
        if key in row:result[key]=(rotation@np.asarray(row[key])+shift).tolist()
    for key in ('plug_rotation', 'tool_rotation', 'pickup_rotation', 'hole_rotation'):
        if key in row:result[key]=(rotation@np.asarray(row[key])).tolist()
    for key in ('plug_velocity_m_s', 'plug_angular_velocity_rad_s'):
        if key in row:result[key]=(rotation@np.asarray(row[key])).tolist()
    return result


@pytest.mark.parametrize('rotation', ROTATIONS)
@pytest.mark.parametrize('donor_rotation', [np.eye(3), ROTATIONS[1]])
@pytest.mark.parametrize('profile,tip,keyed', [('aviation', .080, True), ('guide', .095, True),
                                            ('cylindrical', .095, False)])
def test_plug_pass_and_geometric_metrics_are_rigid_transform_invariant(rotation, donor_rotation, profile, tip, keyed):
    spec=plug_spec(donor_rotation);shift=np.array([.3, -.4, .8])
    spec.update(task=profile+'_plug_insert', tip=tip, keyed=keyed)
    original=PhysicalScorer(**spec);transformed=PhysicalScorer(**rigid_spec(spec, rotation, shift))
    for row in plug_rows(spec):
        original.update(row);transformed.update(rigid_row(row, rotation, shift))
    a,b=original.report(),transformed.report()
    assert a['success'] and b['success']
    for key in ('milestones', 'final_hold_s', 'max_wrist_force_n', 'max_wrist_torque_nm'):
        assert a[key]==b[key]
    for key in ('radial_error_m', 'axis_error_rad', 'orientation_error_rad',
                'insertion_depth_m', 'seat_gap_m', 'peak_object_displacement_m'):
        assert b[key]==pytest.approx(a[key], abs=3e-8)


def test_explicit_horizontal_donor_preserves_legacy_results():
    spec=plug_spec();legacy=dict(spec);legacy.pop('pickup');legacy.pop('pickup_rotation')
    a,b=PhysicalScorer(**legacy),PhysicalScorer(**spec)
    for row in plug_rows(spec):a.update(row);b.update(row)
    ar,br=a.report(),b.report();ar.pop('schema');br.pop('schema')
    assert ar==br


@pytest.mark.parametrize('rotation', ROTATIONS)
def test_retreat_along_receiver_normal_does_not_count_as_lateral_clearance(rotation):
    spec=plug_spec();shift=np.array([.1, .2, .3])
    scorer=PhysicalScorer(**rigid_spec(spec, rotation, shift))
    for row in plug_rows(spec, retreat=[0., 0., .2]):scorer.update(rigid_row(row, rotation, shift))
    report=scorer.report()
    assert all(report['milestones'].values()) and not report['success']


@pytest.mark.parametrize('real_extraction', [False, True])
def test_moving_rotating_pickup_cannot_fake_extraction(real_extraction):
    spec=plug_spec(ROTATIONS[1]);spec['moving_pickup']=True
    scorer=PhysicalScorer(**spec)
    for i,row in enumerate(plug_rows(spec)):
        if i==20:break
        rotation=axis_angle_to_matrix(np.array([0., 0., 1.]), .02*i)@ROTATIONS[1]
        origin=np.asarray(spec['pickup'])+np.array([.01*i, .005*i, .003*i])
        clearance=.025+(.05 if real_extraction and i>=10 else 0.)
        row.update(pickup_position_m=origin.tolist(), pickup_rotation=rotation.tolist(),
                   plug_position_m=(origin+clearance*rotation[:, 2]).tolist(),
                   plug_rotation=(rotation@np.diag([1., -1., -1.])).tolist())
        scorer.update(row)
    assert scorer.error is None
    assert scorer.milestones[0] is real_extraction
    assert scorer.peak_displacement==pytest.approx(.05 if real_extraction else 0., abs=1e-12)


@pytest.mark.parametrize('field,value', [('pickup_position_m', None), ('pickup_rotation', np.zeros((3, 3)).tolist())])
def test_missing_or_invalid_moving_donor_evidence_is_unscorable(field, value):
    spec=plug_spec();spec['moving_pickup']=True
    scorer=PhysicalScorer(**spec);row=next(plug_rows(spec))
    row.update(pickup_position_m=spec['pickup'], pickup_rotation=spec['pickup_rotation'])
    if value is None:row.pop(field)
    else:row[field]=value
    scorer.update(row)
    assert scorer.report()['status']=='unscorable'


def test_rotated_legacy_spec_cannot_silently_assume_world_z_donor():
    spec=plug_spec();spec.pop('pickup');spec.pop('pickup_rotation');spec['hole_rotation']=ROTATIONS[1].tolist()
    with pytest.raises(ValueError, match='explicit pickup'):PhysicalScorer(**spec)


@pytest.mark.parametrize('rotation', ROTATIONS)
@pytest.mark.parametrize('mode', ['calibrated', 'benchmark'])
def test_drawer_model_and_frozen_spec_rotate_without_changing_predicates(tmp_path, rotation, mode):
    original=DrawerCycleSuccess(MODEL, sample_period_ns=EVALUATION_PERIOD_NS, mode=mode)
    mounted=DrawerCycleSuccess(rotated_drawer(tmp_path, rotation), sample_period_ns=EVALUATION_PERIOD_NS, mode=mode)
    frozen=DrawerCycleSuccess.from_spec(mounted.specification())
    for row in cycle():
        sample=policy_sample(row);transformed=policy_sample(rigid_row(row, rotation, np.zeros(3)))
        if row['time_s']==0:
            original.reset(sample);mounted.reset(transformed);frozen.reset(transformed)
        else:
            original.update(sample);mounted.update(transformed);frozen.update(transformed)
        for state in (mounted.status(), frozen.status()):
            for key in ('success', 'success_available', 'open_verified', 'seated_before_release_verified',
                        'release_verified', 'verification_stable_s'):
                assert state[key]==original.status()[key]
    assert original.status()['success']


@pytest.mark.parametrize('field,value', [('rotation', np.diag([1., 1., -1.]).tolist()),
                                       ('rotation', (2*np.eye(3)).tolist()), ('axis', [1., 0., 0.])])
def test_drawer_rejects_nonrigid_or_changed_internal_rail(field, value):
    spec=monitor().specification();spec[field]=value
    with pytest.raises(ValueError, match='rigid mounting'):DrawerCycleSuccess.from_spec(spec)


def test_replay_rows_preserve_measured_donor_and_receiver_pose():
    from simulation.evaluation.replay import physical_row
    sample=SimpleNamespace(time_s=0., joints=np.zeros(8), wrench=np.zeros(6), wrench_valid=True,
        plug_position_body_m=np.zeros(3), plug_rotation_body=np.eye(3),
        plug_velocity_body_m_s=np.zeros(3), plug_angular_velocity_body_rad_s=np.zeros(3),
        finger_plug_contacts=(1, 1), site_poses={'pickup': (np.array([1., 2., 3.]), ROTATIONS[1]),
                                               'hole': (np.array([4., 5., 6.]), np.eye(3))})
    task=SimpleNamespace(chain=SimpleNamespace(forward=lambda q: (np.zeros(3), np.eye(3))))
    row=physical_row(sample, task, np.zeros(8))
    assert row['pickup_position_m']==[1., 2., 3.]
    assert row['pickup_rotation']==ROTATIONS[1].tolist()
    assert row['hole_position_m']==[4., 5., 6.]


def moving_drawer_spec():
    original=monitor();spec=original.specification()
    spec['moving_mount']=dict(grip_offset_m=list(original.config.grip_offset_m), axis_local=[0., 0., -1.])
    return spec


def test_frozen_moving_drawer_calibration_has_no_mutable_aliases():
    spec=moving_drawer_spec();frozen=DrawerCycleSuccess.from_spec(spec)
    spec['moving_mount']['axis_local'][0]=1.
    exported=frozen.specification();exported['moving_mount']['grip_offset_m'][0]=10.
    assert frozen.specification()==moving_drawer_spec()


@pytest.mark.parametrize('field,value', [('axis_local', [0., 0., 1.]), ('axis_local', [0., 0., -2.]),
                                       ('grip_offset_m', [float('nan'), 0., 0.]),
                                       ('grip_offset_m', [1., 2., 3.])])
def test_moving_drawer_cannot_weaken_geometry_contract(field, value):
    spec=moving_drawer_spec();spec['moving_mount'][field]=value
    with pytest.raises(ValueError):DrawerCycleSuccess.from_spec(spec)


def test_moving_drawer_pose_replay_preserves_independent_success():
    original=monitor();spec=moving_drawer_spec();mounted=DrawerCycleSuccess.from_spec(spec)
    for row in cycle():
        sample=policy_sample(row)
        rotation=axis_angle_to_matrix(np.array([0., 1., 0.]), .1*row['time_s'])
        shift=np.array([.03*row['time_s'], -.1, .2])
        transformed=policy_sample(rigid_row(row, rotation, shift))
        origin=original.grip_origin-original.rotation@np.asarray(original.config.grip_offset_m)
        transformed.update(drawer_origin_body_m=(rotation@origin+shift).tolist(),
                           drawer_rotation_body=(rotation@original.rotation).tolist())
        if row['time_s']==0:original.reset(sample);mounted.reset(transformed)
        else:original.update(sample);mounted.update(transformed)
        assert mounted.status()['success']==original.status()['success']
        assert mounted.status()['success_available']
    assert mounted.status()['success']


@pytest.mark.parametrize('keyed', [False, True])
def test_keyed_receiver_preserves_full_rotation_predicate(keyed):
    spec=rigid_spec(plug_spec(), ROTATIONS[1], np.array([.2, .3, .4]));spec['keyed']=keyed
    scorer=PhysicalScorer(**spec)
    for i,row in enumerate(plug_rows(spec)):
        if i>=20:
            row['plug_rotation']=(np.asarray(row['plug_rotation'])@axis_angle_to_matrix(np.array([0., 0., 1.]), .05)).tolist()
        scorer.update(row)
    assert scorer.report()['success'] is (not keyed)


@pytest.mark.parametrize('wrench', [[40.1, 0., 0., 0., 0., 0.], [0., 0., 0., 4.01, 0., 0.]])
def test_rotated_plug_overload_is_not_masked_by_completion_or_expert_flags(wrench):
    spec=rigid_spec(plug_spec(), ROTATIONS[1], np.zeros(3));scorer=PhysicalScorer(**spec)
    for row in plug_rows(spec):
        row.update(wrench=wrench, stage='success', result='success', ever_lifted=True)
        scorer.update(row)
    assert scorer.report()['status']=='task_failure'
    assert scorer.report()['reason']=='wrist_overload'


def test_l_shape_moving_donor_spec_json_round_trip_scores_identically():
    import json
    spec=plug_spec(ROTATIONS[1]);spec['moving_pickup']=True
    online,offline=PhysicalScorer(**spec),PhysicalScorer(**json.loads(json.dumps(spec)))
    for row in plug_rows(spec):
        row.update(pickup_position_m=spec['pickup'], pickup_rotation=spec['pickup_rotation'])
        online.update(row);offline.update(json.loads(json.dumps(row)))
    assert online.report()==offline.report()
    assert online.report()['success']
