import copy

import numpy as np
import pytest

from simulation.gyroscope import Gyroscope
from simulation.pose_navigation import PoseNavigation, exp, log
from space_arm_platform.visual_navigation import VisualReadiness, POSE_ERROR_ORDER
from space_arm_platform.vision_channel import VisionResultReceiver, VisionResultSender
from simulation.navigation_gravity import PointMassNavigationGravity, ReplayGravity


def navigation(**kw):
    return PoseNavigation([[0, 0, 0], [0, 0, 2]], np.zeros((2, 3)),
                          [np.eye(3), np.eye(3)], np.eye(24)*.01, **kw)


def image(epoch, frame=None, **kw):
    c = np.diag([.002**2]*3 + [np.deg2rad(.1)**2]*3)
    return dict(status='ok', ambiguous=False, mission_id='m', session_id='s', calibration_id='c',
                source_frame_id=str(epoch if frame is None else frame), sim_time_ns=str(epoch),
                counts=dict(used_markers_left=2, used_markers_right=2, common_markers=2),
                reprojection_error_px=dict(left=.2, right=.3), max_corner_error_px=dict(left=.4, right=.6),
                pose_covariance=dict(matrix=c.tolist(), frame='left_optical', rotation_error='left_multiplicative', order=POSE_ERROR_ORDER),
                pose_target_to_left=dict(t_m=[0, 0, 2], R=np.eye(3).tolist()), **kw)


def rate(epoch, value=(0, 0, 0), valid=True):
    return dict(sample_time_ns=str(epoch), angular_velocity_body_rad_s=list(value),
                valid=valid, saturated_axes=[not valid, False, False])


def test_process_covariance_matches_constant_rate_transition_and_independent_noise():
    dt = .01
    rng = np.random.default_rng(31)
    factor = rng.normal(size=(24, 24))
    prior = factor@factor.T*.0001 + np.eye(24)*.001
    rotations = np.array([exp([.3, -.2, .1]), exp([-.1, .4, .2])])
    for acceleration_sigma, bias_walk in ((.003, 1e-6), (.01, 3e-6), (.003, 1e-6)):
        nav = PoseNavigation(np.zeros((2, 3)), np.zeros((2, 3)), rotations, prior,
                             acceleration_sigma=acceleration_sigma, gyro_bias_walk=bias_walk)
        nav.gyro(0, rate(0))
        nav.gyro(1, rate(0, valid=False))
        transition, noise = np.eye(24), np.zeros((24, 24))
        for body in range(2):
            j = 12*body
            transition[j:j+3, j+3:j+6] = np.eye(3)*dt
            transition[j+6:j+9, j+9:j+12] = -rotations[body]*dt
            for axis in range(3):
                p, v = j+axis, j+3+axis
                noise[p, p] = acceleration_sigma**2*.25*dt**4
                noise[p, v] = noise[v, p] = acceleration_sigma**2*.5*dt**3
                noise[v, v] = acceleration_sigma**2*dt**2
                noise[j+6+axis, j+6+axis] = (1e-4 if body == 0 else .5)**2*.02*dt
                noise[j+9+axis, j+9+axis] = bias_walk**2*dt
        nav.advance(10_000_000)
        np.testing.assert_allclose(nav.state['covariance'], transition@prior@transition.T+noise,
                                   atol=1e-16, rtol=1e-14)
        np.testing.assert_array_equal(nav.state['r'], rotations)


def test_replay_gravity_bounds_field_linearization_in_distance_and_epoch():
    class Counted(PointMassNavigationGravity):
        queries = 0

        def gradient(self, position, time_s):
            self.queries += 1
            return super().gradient(position, time_s)

    field = Counted()
    cached = ReplayGravity(field)
    anchor = np.array([6_878_136.6, 0., 0.])
    initial = cached.gradient(anchor, .1)
    for delta in ([.24, 0, 0], [0, -.24, 0], [0, 0, .24]):
        position = anchor+delta
        assert np.array_equal(cached.gradient(position, .1), initial)
        exact = PointMassNavigationGravity().gradient(position, .1)
        assert np.linalg.norm(initial-exact) < 1e-12
        assert np.linalg.norm(cached.acceleration(position, .1)-field.acceleration(position, .1)) < 1e-11
    assert field.queries == 1
    cached.gradient(anchor+[1., 0, 0], .1)
    cached.gradient(anchor, .100000001)
    assert field.queries == 3  # Distance and exact exposure time both bound reuse.


def test_pose_jacobian_includes_both_com_levers_and_rotations():
    nav = navigation(camera_rotation=exp([.4, -.1, .2]), camera_position=[.5, -.2, .1],
                     service_com=[.05, .01, -.02], target_com=[.12, .07, -.03])
    nav.state['r'] = np.array([exp([.7, -.2, .4]), exp([-.3, .4, -.1])])
    t, r, h = nav.predict_pose()
    for column in range(24):
        state = copy.deepcopy(nav.state)
        body, axis = divmod(column, 12)
        epsilon = 1e-6
        if axis < 3:
            state['p'][body, axis] += epsilon
        elif axis < 6:
            state['v'][body, axis-3] += epsilon
        elif axis < 9:
            delta = np.zeros(3)
            delta[axis-6] = epsilon
            state['r'][body] = exp(delta)@state['r'][body]
        else:
            state['b'][body, axis-9] += epsilon
        tt, rr, _ = nav.predict_pose(state)
        assert np.allclose(np.r_[(tt-t)/epsilon, log(rr@r.T)/epsilon], h[:, column], atol=2e-6)


def test_delayed_gnss_and_pose_replay_equal_chronological_filter():
    chronological, delayed = navigation(), navigation()
    z = np.array([[.01, .02, .03, 0, 0, 0], [.02, 0, 2.01, 0, 0, 0]])
    noise = np.eye(12)*.001
    # Correlated GNSS error must survive filtering and replay.
    noise[:6, 6:] = noise[6:, :6] = np.eye(6)*.0008
    for epoch in range(0, 301_000_000, 20_000_000):
        for nav in (chronological, delayed):
            nav.advance(epoch)
            for body in range(2):
                nav.gyro(body, rate(epoch, (0, .0001*(body+1), 0)))
        if epoch == 100_000_000:
            assert chronological.gnss(z, noise, epoch, 'one')
        if epoch == 200_000_000:
            assert chronological.visual(image(epoch))
    assert delayed.gnss(z, noise, 100_000_000, 'one')
    assert delayed.visual(image(200_000_000))
    for key in ('p', 'v', 'r', 'b', 'covariance'):
        assert np.allclose(delayed.state[key], chronological.state[key], atol=1e-12)
    assert np.linalg.norm(delayed.state['covariance'][:3, 12:15]) > 0
    assert not delayed.visual(image(200_000_000))


def test_bias_is_estimated_but_common_attitude_is_not_artificially_observed():
    nav = navigation()
    initial_common = np.eye(3)*.02
    for epoch in range(0, 8_000_000_001, 20_000_000):
        nav.advance(epoch)
        nav.gyro(0, rate(epoch, (.001, 0, 0)))
        nav.gyro(1, rate(epoch, (0, 0, 0)))
        if epoch % 100_000_000 == 0:
            assert nav.visual(image(epoch))
    # Relative vision observes differential bias, not either absolute bias.
    assert abs(nav.state['b'][0, 0]-nav.state['b'][1, 0]-.001) < .0001
    c = nav.state['covariance']
    common = c[6:9, 6:9]+c[18:21, 18:21]+c[6:9, 18:21]+c[18:21, 6:9]
    assert np.min(np.linalg.eigvalsh(common)) >= np.min(np.linalg.eigvalsh(initial_common))*.99
    assert np.linalg.eigvalsh(c)[0] > 0


def test_restart_keeps_replay_and_gyro_rng_bit_exact():
    original = Gyroscope(seed=7)
    for epoch in range(0, 101_000_000, 20_000_000):
        original.sample(epoch, [0, 0, .001])
    restored = Gyroscope(seed=7)
    restored.restore(original.snapshot())
    assert restored.sample(120_000_000, [0, 0, .001]) == original.sample(120_000_000, [0, 0, .001])
    with pytest.raises(ValueError, match='mismatch'):
        Gyroscope(seed=19).restore(original.snapshot())
    nav = navigation()
    nav.advance(100_000_000)
    nav.visual(image(80_000_000))
    restored_nav = navigation()
    restored_nav.restore(nav.snapshot())
    for n in (nav, restored_nav):
        n.advance(200_000_000)
        n.visual(image(160_000_000))
    assert np.array_equal(nav.state['covariance'], restored_nav.state['covariance'])


def test_replay_snapshots_do_not_alias_live_or_other_snapshots():
    nav = navigation()
    nav.gyro(0, rate(0))
    frozen = copy.deepcopy(nav.snapshots[0])
    nav.advance(20_000_000)
    nav.gyro(0, rate(20_000_000, (.01, 0, 0)))
    nav.state['gyro_valid'][0] = False
    nav.state['p'][0, 0] = 5.
    nav.snapshots[-1]['rates'][0, 0] = 9.
    for key, value in frozen.items():
        assert np.array_equal(nav.snapshots[0][key], value)


def test_new_render_session_after_restart_preserves_posterior_and_rejects_old_frames():
    nav = navigation(vision_identity=('m', 's', 'c'))
    nav.advance(100_000_000)
    assert nav.visual(image(100_000_000, frame='1'))
    resumed = navigation(vision_identity=('m', 's', 'c'))
    resumed.restore(nav.snapshot())
    posterior = resumed.state['covariance'].copy()
    resumed.bind_vision_session('m', 'new-renderer', 'c')
    assert np.array_equal(resumed.state['covariance'], posterior)
    resumed.advance(200_000_000)
    assert not resumed.visual(image(200_000_000, frame='2'))
    packet = image(200_000_000, frame='1')
    packet['session_id'] = 'new-renderer'
    assert resumed.visual(packet)
    with pytest.raises(ValueError, match='changed across handoff'):
        resumed.bind_vision_session('other-mission', 's', 'c')
    with pytest.raises(ValueError, match='changed across handoff'):
        resumed.bind_vision_session('m', 's', 'other-calibration')


def test_saturation_and_missing_gyro_grow_uncertainty_and_revoke_health():
    nav = navigation()
    nav.gyro(0, rate(0))
    nav.gyro(1, rate(0, valid=False))
    assert nav.gyro_health(0) == [True, False]
    before = nav.state['covariance'].copy()
    nav.advance(100_000_000)
    assert nav.gyro_health(nav.time_ns) == [False, False]
    assert np.trace(nav.state['covariance'][18:21, 18:21]) > np.trace(before[18:21, 18:21])


def observe(gate, result, now=None, accepted=True):
    return gate.observe(result, int(result['sim_time_ns']) if now is None else now, accepted=accepted,
                        relative_position_covariance=np.eye(3)*.01**2,
                        relative_attitude_covariance=np.eye(3)*np.deg2rad(.1)**2)


def live_image(epoch, wall_source):
    return image(epoch, capture_timing={eye: dict(source_wall_time_ns=str(wall_source)) for eye in ('left', 'right')})


def test_live_pose_age_cannot_be_hidden_by_slow_physical_time():
    nav = navigation()
    nav.advance(100_000_000)
    assert not nav.visual(live_image(0, 1_000_000_000), received_wall_time_ns=1_360_000_000)
    assert not nav.events
    assert not nav.visual(image(0), received_wall_time_ns=1_200_000_000)  # missing live clock contract


def test_actual_sampling_interval_cannot_be_hidden_by_slow_physical_time():
    gate = VisualReadiness('m', 's', 'c')
    for n in range(12):
        source = 1_000_000_000+n*200_000_000  # real updates are only 5 Hz
        assert not gate.observe(live_image(n*100_000_000, source), n*100_000_000+10_000_000,
            accepted=True, received_wall_time_ns=source+10_000_000,
            relative_position_covariance=np.eye(3)*.01**2,
            relative_attitude_covariance=np.eye(3)*np.deg2rad(.1)**2)
    assert gate.count == 1


@pytest.mark.parametrize('elapsed', [160_000_000, 360_000_000])
def test_live_ready_revokes_when_wall_updates_stop_despite_short_physical_interval(elapsed):
    gate = VisualReadiness('m', 's', 'c')
    for n in range(10):
        ready = gate.observe(live_image(n*100_000_000, 1_000_000_000+n*100_000_000), n*100_000_000,
            accepted=True, received_wall_time_ns=1_010_000_000+n*100_000_000,
            relative_position_covariance=np.eye(3)*.01**2,
            relative_attitude_covariance=np.eye(3)*np.deg2rad(.1)**2)
    assert ready
    assert not gate.tick(910_000_000, 1_910_000_000+elapsed)
    assert gate.count == 0


@pytest.mark.parametrize('fault', ['duplicate', 'disorder', 'age', 'gap', 'rejected', 'ambiguity', 'identity', 'quality'])
def test_readiness_requires_ten_and_revokes_every_fault(fault):
    gate = VisualReadiness('m', 's', 'c')
    for n in range(10):
        assert observe(gate, image(n*100_000_000)) == (n == 9)
    result, now, accepted = image(1_000_000_000), 1_020_000_000, True
    if fault == 'duplicate':
        result['source_frame_id'] = '900000000'
    elif fault == 'disorder':
        result['sim_time_ns'] = '850000000'
    elif fault == 'age':
        now = 1_360_000_000
    elif fault == 'gap':
        now = 1_060_000_000
    elif fault == 'rejected':
        accepted = False
    elif fault == 'ambiguity':
        result['ambiguous'] = True
    elif fault == 'identity':
        result['session_id'] = 'previous'
    else:
        result['max_corner_error_px']['left'] = 5.1
    observe(gate, result, now, accepted)
    assert gate.count <= 1
    assert not gate.ready
    # Ten fresh consecutive accepted exposures are needed after revocation.
    assert not gate.tick(2_000_000_000)


def test_readiness_uses_covariance_and_not_marker_pixel_size():
    gate = VisualReadiness('m', 's', 'c')
    result = image(0)
    result['marker_pixels'] = 8
    assert not observe(gate, result)
    assert gate.count == 1
    result = image(100_000_000)
    result['pose_covariance']['matrix'][2][2] = .03**2
    observe(gate, result)
    assert gate.count == 0


def test_readiness_update_interruption_without_any_result():
    gate = VisualReadiness('m', 's', 'c')
    for n in range(10):
        observe(gate, image(n*100_000_000))
    assert gate.tick(1_050_000_000)
    assert not gate.tick(1_050_000_001)


def test_result_channel_rejects_other_tasks_sessions_and_calibrations():
    receiver = VisionResultReceiver('m', 's', 'c')
    sender = VisionResultSender(receiver.port)
    try:
        for field in ('mission_id', 'session_id', 'calibration_id'):
            packet = image(0)
            packet[field] = 'other'
            sender(packet)
        sender(image(100_000_000))
        results = receiver.poll()
        assert [r['sim_time_ns'] for r in results] == ['100000000']
        assert receiver.rejected == 3
    finally:
        sender.close()
        receiver.close()


def test_future_exposure_does_not_poison_subsequent_epochs():
    gate = VisualReadiness('m', 's', 'c')
    assert not observe(gate, image(1_000_000_000), 0)
    assert not observe(gate, image(100_000_000))
    assert gate.count == 1


def test_reference_geometry_places_image_codes_in_front_on_positive_y_face():
    from simulation.capture26_mission import desired_geometry, load_vision
    from pathlib import Path
    vision = load_vision(Path(__file__).resolve().parents[1])
    offset, rs = desired_geometry(2., np.eye(3), np.zeros(3), np.zeros(3), vision)
    assert offset[1] > 0
    camera = np.array(vision['body_from_optical']['left']['position_body_m'])
    rc = rs@np.array(vision['body_from_optical']['left']['R_body_from_optical'])
    for corners in vision['layout']['markers'].values():
        pixels = (np.array(corners)-offset-rs@camera)@rc
        assert np.all(pixels[:, 2] > 0)
        assert np.allclose(pixels[:, 2], 1.99999)


def test_platform_release_requires_distinct_epochs_and_revokes_on_quality_loss():
    from simulation.capture26_mission import Capture26Mission, desired_geometry, load_vision
    from simulation.capture26_approach import Capture26Approach
    from pathlib import Path
    vision = load_vision(Path(__file__).resolve().parents[1])
    config = Capture26Mission(vision_nodes_validated=True)
    planner = Capture26Approach(np.zeros(3), np.zeros(3), vision, config=config)
    # Isolate the terminal dwell from the preceding independently tested nodes.
    planner.node, planner.depth = 4, config.terminal_depth_m
    offset, r = desired_geometry(planner.depth, np.eye(3), np.zeros(3), np.zeros(3), vision)
    state = dict(p=np.array([offset, np.zeros(3)]), v=np.zeros((2, 3)),
                 r=np.array([r, np.eye(3)]), rates=np.zeros((2, 3)), b=np.zeros((2, 3)))
    def step(epoch, visual=True):
        return planner.step(epoch, state, position_sigma_m=.001, visual_ready=visual, visual_available=True,
                            gyro_healthy=[True, True], gnss_healthy=True, communication_healthy=True,
                            observation_age_s=.1, mass_kg=100.)
    for epoch in range(0, 5_000_000_000, 50_000_000):
        assert not step(epoch)['released']
    assert step(5_000_000_000)['released']
    assert not step(5_050_000_000, False)['released']
    assert planner.phase != 'NearParked'
    with pytest.raises(ValueError, match='distinct'):
        step(5_050_000_000)


def test_unvalidated_camera_nodes_cannot_release_a_mission():
    from simulation.capture26_mission import load_vision
    from simulation.capture26_approach import Capture26Approach
    from pathlib import Path
    planner = Capture26Approach(np.zeros(3), np.zeros(3), load_vision(Path(__file__).resolve().parents[1]))
    planner.node = 2
    state = dict(p=np.array([[0., 4., 0.], np.zeros(3)]), v=np.zeros((2, 3)),
                 r=np.array([np.eye(3), np.eye(3)]), rates=np.zeros((2, 3)), b=np.zeros((2, 3)))
    result = planner.step(0, state, position_sigma_m=.01, visual_ready=True, visual_available=True,
                         gyro_healthy=[True, True], gnss_healthy=True, communication_healthy=True,
                         observation_age_s=.1, mass_kg=100.)
    assert not result['released']
    assert result['phase'] == 'SafeHold'


def test_committed_replay_boundary_cannot_accept_an_old_measurement_twice():
    nav = navigation()
    assert nav.visual(image(0))
    nav.advance(1_020_000_000)
    before = nav.state['covariance'].copy()
    assert not nav.visual(image(0))
    assert np.array_equal(nav.state['covariance'], before)


def test_visual_boundary_rejects_stale_duplicate_epochs_and_other_sessions_before_fusion():
    nav = navigation(vision_identity=('m', 's', 'c'))
    nav.advance(400_000_000)
    before = nav.state['covariance'].copy()
    assert not nav.visual(image(0))
    wrong = image(300_000_000)
    wrong['session_id'] = 'old'
    assert not nav.visual(wrong)
    wrong = image(300_000_000)
    wrong['counts']['common_markers'] = 1
    assert not nav.visual(wrong)
    assert np.array_equal(before, nav.state['covariance'])
    assert nav.visual(image(300_000_000))
    after = nav.state['covariance'].copy()
    assert not nav.visual(image(300_000_000, frame='other-frame-same-exposure'))
    assert np.array_equal(after, nav.state['covariance'])


def test_delayed_replay_can_revoke_visual_readiness():
    gate = VisualReadiness('m', 's', 'c')
    for n in range(10):
        observe(gate, image(n*100_000_000))
    assert gate.ready
    event = PoseNavigation.visual_event_id(image(500_000_000))
    assert not gate.reconcile({event: False})
    assert gate.count == 0


def test_malformed_navigation_restart_is_rejected_without_replacing_current_state():
    nav = navigation()
    saved = nav.snapshot()
    saved['state']['p'] = [[0, 0], [0, 0]]
    before = copy.deepcopy(nav.state)
    with pytest.raises(ValueError, match='restart state'):
        nav.restore(saved)
    assert np.array_equal(nav.state['p'], before['p'])
