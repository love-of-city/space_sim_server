from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from simulation.near23_navigation import Near23NavigationBridge, relative_navigation_output
from simulation.pose_navigation import PoseNavigation, exp
from space_arm_platform.multicamera_pose import MultiCameraPoseEstimator
from space_arm_platform.visual_navigation import NEAR23_BODY_PROFILE, VisualReadiness


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def setup(tmp_path):
    model = tmp_path/'arm.xml'
    chain = ''.join(f'<body name="b{i}" pos=".03 0 .1"><joint name="bottom_joint{i}" axis="0 0 1"/>'
                    for i in range(1, 7))
    model.write_text('<mujoco><worldbody><body name="cubesat_bus">'+chain+
        '<body name="gripper"><site name="grip_frame"/></body>'+'</body>'*7+
        '</worldbody></mujoco>')
    config = json.loads((ROOT/'config/vision/capture26_near23_front_mount_20261029.json').read_text())
    bridge = Near23NavigationBridge(model, config, joint_covariance_rad2=np.eye(6)*1e-8,
        mount_covariance=np.diag([1e-8]*3+[1e-6]*3), uncertainty_source='test engineering prior')
    estimator = MultiCameraPoseEstimator(config['calibration'], config['layout'])
    r = exp([np.pi/2+.1, .02, .03])
    t = np.array([-.1755, .04, 1.0])
    observations = []
    for eye, ids in enumerate(((0,), (1,))):
        rr = r if eye == 0 else estimator.geometry.rig_r@r
        tt = t if eye == 0 else estimator.geometry.rig_r@t+estimator.geometry.rig_t
        observations.append({i: estimator.geometry._project(estimator.geometry.markers[i], rr, tt, eye)
                             for i in ids})
    measurement = estimator.estimate_from_detections(*observations)['methods']['joint_multicamera']
    assert measurement['status'] == 'ok', measurement
    q = np.array([.1, -.2, .3, -.1, .2, -.3])
    return bridge, measurement, q, estimator, observations


def packet(setup, epoch=0):
    bridge, measurement, q, _, _ = setup
    return bridge.packet(measurement, mission_id='mission', session_id='session',
        source_frame_id=str(epoch), sim_time_ns=epoch, encoder_time_ns=epoch, joint_position_rad=q)


def navigation(setup):
    bridge = setup[0]
    body = packet(setup)['pose_target_to_service']
    service_com, target_com = np.array([.12, -.03, .08]), np.array([.07, .05, -.02])
    rs = exp([.2, .3, -.1])
    rt = rs@np.array(body['R'])
    ps = np.array([100., 200., 300.])
    pt = ps+rs@(np.array(body['t_m'])-service_com)+rt@target_com
    return PoseNavigation([ps, pt], np.zeros((2, 3)), [rs, rt], np.eye(24)*1e-5,
        camera_rotation=exp([.5, -.4, .3]), camera_position=[.4, -.1, .2],
        service_com=service_com, target_com=target_com,
        visual_profile=NEAR23_BODY_PROFILE,
        vision_identity=('mission', 'session', bridge.calibration_id))


def test_image_joint_covariance_is_nonzero_at_perfect_fit_and_uses_pixel_floor(setup):
    _, measurement, _, estimator, obs = setup
    c = np.array(measurement['pose_covariance']['matrix'])
    assert np.linalg.eigvalsh(c)[0] > 0
    assert np.linalg.norm(c[:3, 3:]) > 0
    estimator.geometry.corner_sigma *= 2
    result = estimator.estimate_from_detections(*obs)['methods']['joint_multicamera']
    np.testing.assert_allclose(result['pose_covariance']['matrix'], c*4, rtol=1e-9, atol=1e-15)


def test_camera_fk_converts_body_origins_and_retains_extra_uncertainty(setup):
    bridge, measurement, q, _, _ = setup
    result = packet(setup)
    assert result['status'] == 'ok'
    p, r = bridge.geometry.cameras_service(q)['left']
    expected = p+r@measurement['pose_target_to_left']['t_m']
    np.testing.assert_allclose(result['pose_target_to_service']['t_m'], expected, atol=1e-14)
    np.testing.assert_allclose(result['pose_target_to_service']['R'],
        r@measurement['pose_target_to_left']['R'], atol=1e-14)
    g = np.zeros((6, 6))
    g[:3, :3] = g[3:, 3:] = r
    image_only = g@np.array(measurement['pose_covariance']['matrix'])@g.T
    added = np.array(result['pose_covariance']['matrix'])-image_only
    assert np.linalg.eigvalsh(added)[0] > 0
    assert result['counts'] == dict(used_markers_left=1, used_markers_right=1, common_markers=0)


def test_exposure_buffer_uses_old_joints_and_rejects_missing_or_changed_samples(setup):
    bridge, measurement, q, _, _ = setup
    original_config = deepcopy(bridge.config)
    result = dict(measurement, mission_id='mission', session_id='session', source_frame_id='0',
                  sim_time_ns='0', calibration_id=bridge.calibration_id)
    bridge.remember_encoders(0, q)
    bridge.remember_encoders(100_000_000, q+.2)
    converted = bridge.image_result(result)
    np.testing.assert_array_equal(converted['exposure_geometry']['joint_position_rad'], q)
    assert converted['status'] == 'ok'
    assert bridge.config == original_config
    with pytest.raises(ValueError, match='same epoch'):
        bridge.remember_encoders(100_000_000, q)
    with pytest.raises(ValueError, match='backwards'):
        bridge.remember_encoders(0, q)
    bridge.remember_encoders(1_100_000_000, q)
    assert bridge.image_result(result)['reason'] == 'no encoders at image exposure'
    result.update(sim_time_ns='1100000000', calibration_id='wrong')
    converted = bridge.image_result(result)
    assert converted['status'] == 'rejected' and 'pose_target_to_service' not in converted


@pytest.mark.parametrize('fault', ['epoch', 'missing_covariance', 'lost', 'bad_side', 'bad_encoder'])
def test_bad_frames_export_invalid_without_reusing_a_pose(setup, fault):
    bridge, measurement, q, _, _ = setup
    measurement = deepcopy(measurement)
    encoder_epoch = 100
    if fault == 'epoch':
        encoder_epoch = 99
    elif fault == 'missing_covariance':
        del measurement['pose_covariance']
    elif fault == 'lost':
        measurement = dict(status='rejected', reason='not detected')
    elif fault == 'bad_side':
        measurement['minimum_marker_side_px']['right'] = 11.9
    else:
        q = [0, 0]
    result = bridge.packet(measurement, mission_id='m', session_id='s', source_frame_id='1',
        sim_time_ns=100, encoder_time_ns=encoder_epoch, joint_position_rad=q)
    assert result['status'] == 'rejected' and result['ambiguous']
    assert 'pose_target_to_service' not in result
    assert 'pose_covariance' not in result


def test_opt_in_fuses_body_measurement_with_com_levers_and_default_rejects(setup):
    nav = navigation(setup)
    result = packet(setup)
    p, r, _ = nav.predict_pose(frame='service_body')
    np.testing.assert_allclose(p, result['pose_target_to_service']['t_m'], atol=1e-13)
    np.testing.assert_allclose(r, result['pose_target_to_service']['R'], atol=1e-13)
    initial = nav.relative_covariances()[0]
    assert nav.visual(result), nav.last_reason
    assert np.trace(nav.relative_covariances()[0]) < np.trace(initial)
    default = PoseNavigation(nav.state['p'], nav.state['v'], nav.state['r'], np.eye(24)*1e-5)
    assert not default.visual(result)
    assert 'insufficient' in default.last_reason


def test_body_measurement_jacobian_includes_both_assembly_com_levers(setup):
    nav = navigation(setup)
    p, r, h = nav.predict_pose(frame='service_body')
    from simulation.pose_navigation import log
    for column in (0, 2, 6, 8, 12, 14, 18, 20):
        state = deepcopy(nav.state)
        body, index = divmod(column, 12)
        delta = np.zeros(3)
        if index < 3:
            state['p'][body, index] += 1e-6
        else:
            delta[index-6] = 1e-6
            state['r'][body] = exp(delta)@state['r'][body]
        pp, rr, _ = nav.predict_pose(state, frame='service_body')
        np.testing.assert_allclose(np.r_[(pp-p)/1e-6, log(rr@r.T)/1e-6], h[:, column], atol=2e-6)


def test_delayed_image_replay_and_restart_keep_exposure_fk_and_com_geometry(setup):
    chronological, delayed = navigation(setup), navigation(setup)
    images = []
    for epoch in (0, 100_000_000, 200_000_000):
        for nav in (chronological, delayed):
            nav.advance(epoch)
            # Different levers at each epoch must survive late image replay.
            offset = nav.camera_com_offset + np.array([epoch*1e-11, 0., 0.])
            nav.encoder_geometry(epoch, offset, nav.target_com, np.zeros((2, 3)))
        image = packet(setup, epoch)
        images.append(image)
        assert chronological.visual(image), chronological.last_reason
    for image in images:
        assert delayed.visual(image), delayed.last_reason
    for key in ('p', 'v', 'r', 'covariance'):
        np.testing.assert_allclose(delayed.state[key], chronological.state[key], atol=1e-12)
    restored = navigation(setup)
    restored.restore(delayed.snapshot())
    assert restored.configuration() == delayed.configuration()
    assert not restored.visual(images[-1])
    np.testing.assert_array_equal(restored.state['covariance'], delayed.state['covariance'])


@pytest.mark.parametrize('fault', ['stale', 'encoder_epoch', 'calibration', 'identity', 'side', 'depth', 'frame'])
def test_nav_rejects_wrong_or_stale_eye_in_hand_data_before_correction(setup, fault):
    nav = navigation(setup)
    image = packet(setup)
    if fault == 'stale':
        nav.advance(400_000_000)
    elif fault == 'encoder_epoch':
        image['exposure_geometry']['sample_time_ns'] = '1'
    elif fault == 'calibration':
        image['exposure_geometry']['calibration_id'] = 'different'
    elif fault == 'identity':
        image['session_id'] = 'different'
    elif fault == 'side':
        image['minimum_marker_side_px']['left'] = 11.9
    elif fault == 'depth':
        image['positive_depth'] = False
    else:
        image['pose_covariance']['frame'] = 'left_optical'
    before = nav.state['covariance'].copy()
    assert not nav.visual(image)
    np.testing.assert_array_equal(nav.state['covariance'], before)


def test_readiness_requires_selected_profile_and_revokes_on_missing_new_image(setup):
    bridge, measurement, q, estimator, _ = setup
    # A broad set of close corners gives enough precision for readiness.
    # Single-marker detection alone does not promise the 3 cm depth gate.
    pose = measurement['pose_target_to_left']
    r, t = np.array(pose['R']), np.array(pose['t_m'])
    t[2] = .5
    observations = []
    for eye in (0, 1):
        rr = r if eye == 0 else estimator.geometry.rig_r@r
        tt = t if eye == 0 else estimator.geometry.rig_r@t+estimator.geometry.rig_t
        observations.append({i: estimator.geometry._project(estimator.geometry.markers[i], rr, tt, eye)
                             for i in (0, 1, 2, 3)})
    measurement = estimator.estimate_from_detections(*observations)['methods']['joint_multicamera']
    assert measurement['status'] == 'ok'
    setup = (bridge, measurement, q, estimator, observations)
    nav = navigation(setup)
    gate = VisualReadiness('mission', 'session', setup[0].calibration_id,
        minimum=2, visual_profile=NEAR23_BODY_PROFILE)
    for epoch in (0, 100_000_000):
        nav.advance(epoch)
        result = packet(setup, epoch)
        assert nav.visual(result), nav.last_reason
        pc, ac = nav.relative_covariances()
        gate.observe(result, epoch, accepted=True, relative_position_covariance=pc,
            relative_attitude_covariance=ac)
    assert gate.ready, gate.reason
    gate.tick(260_000_000)
    assert not gate.ready and gate.count == 0


def test_detected_pose_with_large_depth_uncertainty_cannot_release_visual_gate(setup):
    result = packet(setup)
    assert result['status'] == 'ok'
    # Decoded IDs and a small residual can coexist with poor observability.
    result['optical_depth_sigma_m'] = .031
    gate = VisualReadiness('mission', 'session', setup[0].calibration_id,
        minimum=1, visual_profile=NEAR23_BODY_PROFILE)
    assert not gate.observe(result, 0, accepted=True,
        relative_position_covariance=np.eye(3)*1e-6, relative_attitude_covariance=np.eye(3)*1e-6)
    assert 'uncertainty' in gate.reason


def test_guidance_output_is_com_posterior_with_correlated_covariance(setup):
    nav = navigation(setup)
    common = np.eye(6)*2e-6
    nav.state['covariance'][:6, 12:18] = common
    nav.state['covariance'][12:18, :6] = common
    result = relative_navigation_output(nav)
    assert result['reference'] == 'target_assembly_com_minus_service_assembly_com'
    np.testing.assert_allclose(result['position_m'], nav.state['p'][1]-nav.state['p'][0])
    np.testing.assert_allclose(result['position_velocity_covariance'], np.eye(6)*1.6e-5)
