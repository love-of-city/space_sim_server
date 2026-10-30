import copy
import json

import numpy as np
import pytest

from simulation.end_stereo_pointing import EndStereoGeometry, PrealignmentController


@pytest.fixture
def geometry(tmp_path):
    bodies = ''.join(f'<body name="b{i}" pos="0 0 .1"><joint name="bottom_joint{i}"/>'
                     for i in range(1, 7))
    model = tmp_path/'arm.xml'
    model.write_text('<mujoco><worldbody><body name="cubesat_bus">'+bodies+
        '<body name="gripper"><site name="grip_frame"/></body>'+'</body>'*7+
        '</worldbody></mujoco>')
    optical = [[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]]
    positions = dict(left=[.06,.06,.05], right=[.06,-.06,.05])
    config = dict(mount_parent_id='teleop/gripper', camera_ids=dict(left='left',right='right'),
        cameras=[dict(camera_id=e,parent_id='teleop/gripper',position_body_m=positions[e],
            orientation_body_from_camera_wxyz=[np.sqrt(.5),0,np.sqrt(.5),0],
            field_of_view_rad=np.deg2rad(100),resolution=[1920,1920]) for e in positions],
        parent_from_optical={e:dict(position_parent_m=p,R_parent_from_optical=optical)
                             for e,p in positions.items()},
        calibration=dict(K_left=[[805.5356459301888,0,959.5],[0,805.5356459301888,959.5],[0,0,1]],
            K_right=[[805.5356459301888,0,959.5],[0,805.5356459301888,959.5],[0,0,1]],
            D_left=[0]*5,D_right=[0]*5,image_size=[1920,1920],
            R_right_from_left=np.eye(3).tolist(),t_right_from_left_m=[-.12,0,0]),
        marker_rig=dict(face='+Y',grasp_center_target_m=[0,0,0],markers=[dict(id=0,size_m=.02)]),
        layout=dict(markers={'0':[[.01,0,.01],[-.01,0,.01],[-.01,0,-.01],[.01,0,-.01]]}))
    return EndStereoGeometry(model, config), model, config


def test_actual_render_forward_is_optical_z_not_render_z(geometry):
    g, _, _ = geometry
    left, right = g.cameras_service(np.zeros(6)).values()
    np.testing.assert_allclose(left[1][:,2], [0,0,-1], atol=1e-14)
    np.testing.assert_allclose(left[0], [.06,.06,.65], atol=1e-14)
    assert np.linalg.norm(left[0]-right[0]) == pytest.approx(.12)
    ref = g.reference([0,0,-17.4], np.eye(3), np.zeros(6))
    view = g.view([0,0,-17.4], np.eye(3), ref, np.zeros(6))
    assert view['geometry_ok']
    # Geometric visibility is allowed even when a marker is too tiny to decode.
    assert view['eyes']['left']['largest_marker_edge_px'] < 2


def test_render_calibration_disagreement_is_rejected(geometry):
    _, model, config = geometry
    bad = copy.deepcopy(config)
    bad['cameras'][0]['orientation_body_from_camera_wxyz'] = [1,0,0,0]
    with pytest.raises(ValueError, match='render and optical'):
        EndStereoGeometry(model, bad)


def test_joint_and_inertial_rotation_change_camera_reference_consistently(geometry):
    g, _, _ = geometry
    base = g.reference([0,0,-17.4],np.eye(3),np.zeros(6))
    moved = g.reference([0,0,-17.4],np.eye(3),np.array([.2,0,0,0,0,0]))
    assert np.linalg.norm(base-moved) > .1
    Q = np.diag([-1.,-1.,1.])
    np.testing.assert_allclose(g.reference(Q@np.array([0,0,-17.4]),Q,np.zeros(6)),Q@base,atol=1e-12)


def test_reference_motion_stationary_and_rigidly_rotating_configuration(geometry):
    g = geometry[0]
    relative = np.array([.1, .3, -17.4])
    data = dict(relative_position_n_m=relative, relative_velocity_n_m_s=np.zeros(3),
        service_rotation_n_from_body=np.eye(3), service_angular_rate_body_rad_s=np.zeros(3),
        joint_position_rad=np.zeros(6), joint_velocity_rad_s=np.zeros(6))
    reference, rate = g.reference_motion(**data)
    np.testing.assert_allclose(reference, g.reference(relative,np.eye(3),np.zeros(6)), atol=1e-14)
    np.testing.assert_allclose(rate, np.zeros(3), atol=1e-12)
    # If the entire configuration rotates rigidly, its reference must have
    # exactly that spatial rate, despite the nonzero camera/marker lever arms.
    omega = np.array([.002,-.003,.001])
    _, rate = g.reference_motion(**(data | dict(relative_velocity_n_m_s=np.cross(omega,relative),
                                              service_angular_rate_body_rad_s=omega)))
    np.testing.assert_allclose(rate, omega, atol=1e-10, rtol=0)


def test_reference_motion_responds_to_encoder_rates(geometry):
    g = geometry[0]
    # Six coaxial joints: equal and opposite first/last joint rates leave the
    # camera stationary; a nonzero summed rate must change the roll reference.
    data = dict(relative_position_n_m=[0,0,-17.4], relative_velocity_n_m_s=np.zeros(3),
        service_rotation_n_from_body=np.eye(3), service_angular_rate_body_rad_s=np.zeros(3),
        joint_position_rad=np.zeros(6))
    _, rate = g.reference_motion(**data, joint_velocity_rad_s=[.01,0,0,0,0,-.01])
    np.testing.assert_allclose(rate, np.zeros(3), atol=1e-12)
    _, rate = g.reference_motion(**data, joint_velocity_rad_s=[.01,0,0,0,0,0])
    assert np.linalg.norm(rate) > .0099
    assert rate[2] > 0


@pytest.mark.parametrize('field', ['relative_velocity_n_m_s','service_angular_rate_body_rad_s',
                                  'joint_velocity_rad_s'])
def test_reference_motion_rejects_missing_or_nonfinite_rates(geometry,field):
    data = dict(relative_position_n_m=[0,0,-17.4], relative_velocity_n_m_s=np.zeros(3),
        service_rotation_n_from_body=np.eye(3), service_angular_rate_body_rad_s=np.zeros(3),
        joint_position_rad=np.zeros(6), joint_velocity_rad_s=np.zeros(6))
    data[field] = np.full(6 if field=='joint_velocity_rad_s' else 3,np.nan)
    with pytest.raises(ValueError):
        geometry[0].reference_motion(**data)


def test_one_eye_outside_frame_is_reported(geometry):
    g, _, _ = geometry
    # Keep the main marker frontal; place it at just one eye's horizontal edge.
    target = np.array([[1,0,0],[0,0,-1],[0,1,0.]])
    view = g.view([.06,1.9,-1.],np.eye(3),target,np.zeros(6))
    assert not view['geometry_ok']
    assert sum(e['region_in_frame'] for e in view['eyes'].values()) == 1


def step(controller, t, **overrides):
    relative = np.array([0,0,-17.4])
    desired = controller.geometry.reference(relative,np.eye(3),np.zeros(6))
    data = dict(now_s=t,input_stamps_s=dict(navigation=t,service_attitude=t,
        target_telemetry=t,camera_kinematics=t),relative_position_n_m=relative,
        navigation_position_sigma_m=.03,service_rotation_n_from_body=np.eye(3),
        target_rotation_n_from_body=desired,target_angular_rate_body_rad_s=[0,0,0],
        reference_angular_rate_n_rad_s=[0,0,0],joint_position_rad=np.zeros(6))
    data.update(overrides)
    return controller.update(**data)


def test_alignment_requires_continuous_five_seconds_and_is_not_visual_ready(geometry):
    controller = PrealignmentController(geometry[0])
    for k in range(100):
        assert not step(controller,k*.05).aligned
    result = step(controller,5.)
    assert result.aligned
    assert result.reason == 'prealigned_not_visual_ready'
    assert np.linalg.norm(result.torque_body_nm) < 1e-12


def test_close_orbital_attitude_hold_keeps_view_freshness_and_dwell_checks(geometry, monkeypatch):
    g = geometry[0]
    controller = PrealignmentController(g)
    held = np.array([[1.,0,0],[0,0,-1.],[0,1.,0]])
    relative = np.array([.06,0,.1])  # Only 0.55m from camera midpoint.
    def far_reference_must_not_run(*_):
        raise RuntimeError('far pre-alignment helper called inside close hold')
    monkeypatch.setattr(g, 'reference', far_reference_must_not_run)
    def update(t, **changes):
        data = dict(now_s=t,input_stamps_s=dict(navigation=t,service_attitude=t,
            target_telemetry=t,camera_kinematics=t),relative_position_n_m=relative,
            navigation_position_sigma_m=.003,service_rotation_n_from_body=np.eye(3),
            target_rotation_n_from_body=held,target_angular_rate_body_rad_s=np.zeros(3),
            reference_angular_rate_n_rad_s=np.zeros(3),joint_position_rad=np.zeros(6),
            reference_rotation_n_from_body=held)
        return controller.update(**(data | changes))
    for k in range(100):
        assert not update(k*.05).aligned
    assert update(5.).aligned
    bad = update(5.05,input_stamps_s=dict(navigation=4.,service_attitude=5.05,
        target_telemetry=5.05,camera_kinematics=5.05))
    assert not bad.aligned and bad.reason == 'input_stale'
    assert update(5.1).dwell_s == 0
    bad = update(5.15,relative_position_n_m=[.06,0,2.])  # Behind the camera.
    assert not bad.aligned and bad.reason == 'camera_geometry_not_ready'


@pytest.mark.parametrize('source',['navigation','target_telemetry','camera_kinematics','service_attitude'])
def test_stale_source_cancels_dwell_and_torque(geometry,source):
    controller = PrealignmentController(geometry[0])
    for k in range(99):
        step(controller,k*.05)
    stamps = dict(navigation=5.,target_telemetry=5.,camera_kinematics=5.,service_attitude=5.)
    stamps[source] = 4.
    bad = step(controller,5.,input_stamps_s=stamps)
    assert not bad.aligned and bad.reason == 'input_stale'
    np.testing.assert_array_equal(bad.torque_body_nm,np.zeros(3))
    assert step(controller,5.05).dwell_s == 0


def test_facing_target_does_not_mean_camera_points_at_target(geometry):
    g = geometry[0]
    controller = PrealignmentController(g)
    relative = np.array([0,0,17.4])  # Behind the actual camera optical axis.
    desired = g.reference(relative,np.eye(3),np.zeros(6))
    for k in range(110):
        result = step(controller,k*.05,relative_position_n_m=relative,
                      target_rotation_n_from_body=desired)
    assert result.attitude_error_deg < 1e-5
    assert not result.aligned and result.reason == 'camera_geometry_not_ready'


def test_time_gap_disable_and_bad_joints_reset_alignment(geometry):
    controller = PrealignmentController(geometry[0])
    step(controller,0.)
    assert step(controller,5.).dwell_s == 0
    assert step(controller,5.05,enabled=False).reason == 'prealignment_disabled'
    assert step(controller,5.1,joint_position_rad=[np.nan]*6).reason == 'invalid_prealignment_input'
    assert step(controller,5.2).dwell_s == 0
    assert step(controller,5.2).reason == 'nonincreasing_control_time'


def test_file_loading_and_pair_calibration_are_checked(geometry,tmp_path):
    _, model, config = geometry
    path = tmp_path/'camera.json'
    path.write_text(json.dumps(config))
    EndStereoGeometry.from_files(model,path)
    config['calibration']['t_right_from_left_m'] = [.12,0,0]
    with pytest.raises(ValueError,match='pair calibration'):
        EndStereoGeometry(model,config)


def image_packet(geometry, *, relative=None, q=None, service=None, frame='100', stamp='10000000000'):
    g = geometry
    relative = np.array([1.3,-.4,5.]) if relative is None else np.array(relative)
    q = np.array([.2,-.3,.4,-.1,.2,-.3]) if q is None else q
    service = np.array([[0,-1,0],[1,0,0],[0,0,1.]]) if service is None else service
    p, r = g.cameras_service(q)['left']
    target_r = np.eye(3)
    translation = r.T@(service.T@relative-p)
    rotation = r.T@service.T@target_r
    packet = dict(session_id='test',source_frame_id=frame,sim_time_ns=stamp,
        camera_ids=['left','right'],measurement=dict(status='ok',route='joint_multicamera',
            pose_target_to_left=dict(R=rotation.tolist(),t_m=translation.tolist()),
            used_ids=dict(left=[0],right=[0]),reprojection_error_px=.3))
    arguments = dict(now_ns=int(stamp)+150_000_000,joint_position_rad=q,kinematics_stamp_ns=stamp,
        service_rotation_n_from_body=service,service_attitude_stamp_ns=stamp,
        position_covariance_n_m2=np.eye(3)*1e-6)
    return packet,arguments,relative


def test_image_pose_maps_target_origin_to_navigation_once_and_preserves_epoch(geometry):
    from simulation.image_relative_navigation import ImageRelativeNavigation
    g = geometry[0]
    packet,arguments,expected = image_packet(g)
    output = ImageRelativeNavigation(g,'test').convert(packet,**arguments)
    np.testing.assert_allclose(output['z'],expected,atol=1e-12)
    np.testing.assert_allclose(output['rotation_n_from_target'],np.eye(3),atol=1e-12)
    assert output['stamp'] == 10.  # Never relabel an old image as the control time.
    assert output['reference'] == 'target_body_origin_minus_service_body_origin'
    # Diagnostic target truth and per-frame rendering extrinsics are ignored.
    packet['metadata'] = {'extrinsics': {'position_N_m':[1e9]*3}, 'target_truth':[1e9]*3}
    poison = ImageRelativeNavigation(g,'test').convert(packet,**arguments)
    np.testing.assert_array_equal(poison['z'],output['z'])


@pytest.mark.parametrize('problem', ['wrong_session','wrong_pair','rejected','stale','future',
                                    'wrong_arm_epoch','wrong_attitude_epoch','bad_covariance'])
def test_image_navigation_rejects_unusable_measurements(geometry,problem):
    from simulation.image_relative_navigation import ImageRelativeNavigation
    g = geometry[0]
    packet,args,_ = image_packet(g)
    if problem == 'wrong_session': packet['session_id'] = 'other'
    elif problem == 'wrong_pair': packet['camera_ids'] = ['wrist','right']
    elif problem == 'rejected': packet['measurement']['status'] = 'rejected'
    elif problem == 'stale': args['now_ns'] += 1_000_000_000
    elif problem == 'future': args['now_ns'] = 9_000_000_000
    elif problem == 'wrong_arm_epoch': args['kinematics_stamp_ns'] = args['now_ns']
    elif problem == 'wrong_attitude_epoch': args['service_attitude_stamp_ns'] = args['now_ns']
    elif problem == 'bad_covariance': args['position_covariance_n_m2'] = .3  # pixels are not m^2
    with pytest.raises(ValueError):
        ImageRelativeNavigation(g,'test').convert(packet,**args)


def test_duplicate_images_are_not_repeated_navigation_updates(geometry):
    from simulation.image_relative_navigation import ImageRelativeNavigation
    g = geometry[0]
    adapter = ImageRelativeNavigation(g,'test')
    packet,args,_ = image_packet(g)
    adapter.convert(packet,**args)
    with pytest.raises(ValueError,match='duplicate_or_out_of_order'):
        adapter.convert(packet,**args)
    packet['source_frame_id'] = '101'
    with pytest.raises(ValueError,match='duplicate_or_out_of_order'):
        adapter.convert(packet,**args)


def test_converted_image_packet_updates_existing_relative_ekf(geometry):
    from simulation.image_relative_navigation import ImageRelativeNavigation
    from simulation.near_field.navigation import RelativeEKF
    g = geometry[0]
    packet,args,position = image_packet(g)
    output = ImageRelativeNavigation(g,'test').convert(packet,**args)
    prior = np.r_[position+[.02,-.01,.01],np.zeros(3)]
    nav = RelativeEKF(prior,np.eye(6)*.03**2)
    before = np.linalg.norm(nav.x[:3]-position)
    assert nav.update(output,args['now_ns']*1e-9)
    assert nav.accepted['stereo'] == 1
    assert np.linalg.norm(nav.x[:3]-position) < before/10


def test_cooperative_image_route_requires_explicit_opt_in_and_attitude_epoch(geometry):
    from simulation.image_relative_navigation import ImageRelativeNavigation
    g=geometry[0]
    packet,args,position=image_packet(g)
    packet['measurement'].update(route='cooperative_attitude_stereo_translation',
        attitude_source='same_epoch_cooperative_telemetry',independent_image_attitude=False,
        attitude_stamp_ns=packet['sim_time_ns'])
    with pytest.raises(ValueError,match='image_pose_not_accepted'):
        ImageRelativeNavigation(g,'test').convert(packet,**args)
    converted=ImageRelativeNavigation(g,'test',allow_cooperative_attitude=True).convert(packet,**args)
    np.testing.assert_allclose(converted['z'],position,atol=1e-12)
    assert converted['quality']['route']=='cooperative_attitude_stereo_translation'
    packet['measurement']['attitude_stamp_ns']=int(packet['sim_time_ns'])-1
    with pytest.raises(ValueError,match='image_pose_not_accepted'):
        ImageRelativeNavigation(g,'test',allow_cooperative_attitude=True).convert(packet,**args)
