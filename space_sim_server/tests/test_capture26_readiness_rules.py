"""Physical-time admission and scheduling regressions; no flight certification."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from simulation.capture26_approach import Capture26Approach
from simulation.capture26_mission import Capture26Mission, desired_geometry, load_vision
from simulation.platform_stability import PlatformStability
from simulation.sensor_delivery import FixedStereoDelivery
from space_arm_platform.visual_navigation import VisualReadiness
from test_pose_navigation import image


def terminal():
    config = Capture26Mission(vision_nodes_validated=True, terminal_depth_m=.75)
    vision = load_vision(Path(__file__).resolve().parents[1])
    approach = Capture26Approach(np.zeros(3), np.zeros(3), vision, config=config)
    approach.node, approach.depth = 4, config.terminal_depth_m
    offset, rotation = desired_geometry(approach.depth, np.eye(3), np.zeros(3), np.zeros(3), vision)
    state = dict(p=np.array([offset, np.zeros(3)]), v=np.zeros((2, 3)),
                 r=np.array([rotation, np.eye(3)]), rates=np.zeros((2, 3)), b=np.zeros((2, 3)))
    return approach, state


def step(approach, state, now, ready=False, **kw):
    options = dict(position_sigma_m=.001, visual_ready=ready, visual_available=ready,
        gyro_healthy=[True, True], gnss_healthy=True, communication_healthy=True,
        observation_age_s=.2, mass_kg=100.)
    return approach.step(now, state, **(options | kw))


@pytest.mark.parametrize('first_exposure,release_ns', [(5_000_000_000, 6_100_000_000), (0, 5_000_000_000)])
def test_release_at_first_control_update_when_both_independent_conditions_hold(first_exposure, release_ns):
    approach, state = terminal()
    gate = VisualReadiness('m', 's', 'c')
    first_release = None
    for now in range(0, release_ns+1, 50_000_000):
        exposure = now-200_000_000
        if exposure >= first_exposure and exposure % 100_000_000 == 0:
            gate.observe(image(exposure), now, accepted=True,
                relative_position_covariance=np.eye(3)*.001**2,
                relative_attitude_covariance=np.eye(3)*np.deg2rad(.1)**2)
        gate.tick(now)
        result = step(approach, state, now, gate.ready)
        if result['released'] and first_release is None:
            first_release = now
    assert first_release == release_ns
    assert result['platform_stable'] and gate.count >= 10


def test_visual_revocation_and_ten_frame_recovery_preserve_platform_dwell():
    approach, state = terminal()
    for now in range(0, 5_000_000_001, 50_000_000):
        result = step(approach, state, now, True)
    assert result['released']
    result = step(approach, state, 5_050_000_000, False)
    assert not result['released'] and result['platform_stable']
    assert result['platform_stable_duration_s'] == pytest.approx(5.05)
    gate = VisualReadiness('m', 's', 'c')
    for now in range(5_100_000_000, 6_000_000_001, 50_000_000):
        if now % 100_000_000 == 0:
            gate.observe(image(now-200_000_000), now, accepted=True,
                relative_position_covariance=np.eye(3)*1e-6,
                relative_attitude_covariance=np.eye(3)*1e-6)
        result = step(approach, state, now, gate.ready)
        assert result['released'] == (now == 6_000_000_000)
    assert result['platform_stable_duration_s'] == pytest.approx(6.)


@pytest.mark.parametrize('fault', ['position', 'velocity', 'uncertainty', 'service_rate', 'gyro', 'target_gyro', 'process'])
def test_platform_faults_clear_dwell_then_require_five_seconds(fault):
    approach, state = terminal()
    for now in range(0, 5_000_000_001, 50_000_000):
        step(approach, state, now, True)
    bad = deepcopy(state)
    options = {}
    if fault == 'position':
        bad['p'][0, 0] += .01
    elif fault == 'velocity':
        bad['v'][0, 0] = .003
    elif fault == 'uncertainty':
        options['position_sigma_m'] = .04
    elif fault == 'service_rate':
        bad['rates'][0, 0] = .001000001
    elif fault == 'gyro':
        options['gyro_healthy'] = [False, True]
    elif fault == 'target_gyro':
        options['gyro_healthy'] = [True, False]
    else:
        options['process_healthy'] = False
    result = step(approach, bad, 5_050_000_000, True, **options)
    assert not result['released'] and result['platform_stable_duration_s'] == 0
    for now in range(5_100_000_000, 10_100_000_001, 50_000_000):
        result = step(approach, state, now, True)
        assert result['released'] == (now == 10_100_000_000)


def test_stability_handoff_inherits_only_matching_reference_and_current_health():
    a = PlatformStability()
    for now in range(0, 5_000_000_001, 50_000_000):
        a.update(now, ['PlatformStandoff', 1.5], {'health': True})
    b = PlatformStability()
    b.restore(a.snapshot())
    assert b.update(5_000_000_000, ['PlatformStandoff', 1.5], {'health': True})['platform_stable']
    assert not b.update(5_050_000_000, ['PlatformStandoff', 1.25], {'health': True})['platform_stable']
    b.restore(a.snapshot())
    assert b.update(5_000_000_000, ['PlatformStandoff', 1.5], {'health': False})['platform_stable_duration_s'] == 0


@pytest.mark.parametrize('age_ns,valid', [(350_000_000, True), (350_000_001, False)])
def test_simulation_age_inclusive_boundary(age_ns, valid):
    gate = VisualReadiness('m', 's', 'c', minimum=1)
    assert gate.observe(image(0), age_ns, accepted=True,
        relative_position_covariance=np.eye(3)*1e-6,
        relative_attitude_covariance=np.eye(3)*1e-6) is valid


@pytest.mark.parametrize('interval', [150_000_000, 150_000_001])
def test_sampling_and_receiving_interval_inclusive_boundary(interval):
    gate = VisualReadiness('m', 's', 'c')
    for exposure in (0, interval):
        gate.observe(image(exposure), exposure+200_000_000, accepted=True,
            relative_position_covariance=np.eye(3)*1e-6,
            relative_attitude_covariance=np.eye(3)*1e-6)
    assert gate.count == (2 if interval == 150_000_000 else 1)


def scheduled_trace(compute_seconds, pacing_seconds):
    from simulation.pose_navigation import PoseNavigation
    from test_pose_navigation import rate
    prior = np.diag(([.002**2]*6+[np.deg2rad(.05)**2]*3+[1e-5**2]*3)*2)
    nav = PoseNavigation([[0, 0, 0], [0, 0, 2]], np.zeros((2, 3)), [np.eye(3)]*2, prior)
    wall = [0.]
    scheduler = FixedStereoDelivery(clock=lambda: wall[0], pause=lambda t: wall.__setitem__(0, wall[0]+t))
    work, trace = [], []
    def poll():
        ready = [packet for done, packet in work if done <= wall[0]]
        work[:] = [(done, packet) for done, packet in work if done > wall[0]]
        return ready
    nav_acceptance = []
    gate = VisualReadiness('m', 's', 'c')
    approach, state = terminal()
    for now in range(0, 6_000_000_001, 10_000_000):
        scheduler.prepare(now, poll)
        nav.advance(now)
        if now % 20_000_000 == 0:
            for body in (0, 1):
                assert nav.gyro(body, rate(now))
        for packet in scheduler.take_due(now):
            accepted = nav.visual(packet)
            nav_acceptance.append((int(packet['sim_time_ns']), now, accepted))
            position, attitude = nav.relative_covariances()
            gate.observe(packet, now, accepted=accepted,
                relative_position_covariance=position, relative_attitude_covariance=attitude)
        gate.reconcile(nav.accepted, now)
        gate.tick(now)
        gate.check_uncertainty(*nav.relative_covariances(), now)
        result = step(approach, state, now, gate.ready)
        trace.append((now, gate.count, gate.ready, result['platform_stable'], result['released']))
        if now % 100_000_000 == 0:
            scheduler.expect(now, str(now))
            work.append((wall[0]+compute_seconds, image(now)))
        wall[0] += pacing_seconds
    return nav_acceptance, trace


def test_fixed_delivery_is_invariant_to_host_speed_and_computation_waits():
    reference = scheduled_trace(0., .05)
    for compute, pacing in ((.1, .025), (.8, .01), (.4, .1)):
        assert scheduled_trace(compute, pacing) == reference
    assert all(received-exposure == 200_000_000 for exposure, received, _ in reference[0])
    assert all(accepted for _, _, accepted in reference[0])
    assert next(now for now, _, ready, _, _ in reference[1] if ready) == 1_100_000_000
    assert next(now for now, _, _, _, released in reference[1] if released) == 5_000_000_000


@pytest.mark.parametrize('exited', [False, True])
def test_watchdog_is_process_health_and_keeps_delivery_epoch(exited):
    wall = [0.]
    scheduler = FixedStereoDelivery(timeout_s=.02, clock=lambda: wall[0], pause=lambda t: wall.__setitem__(0, wall[0]+t))
    scheduler.expect(0, '0')
    scheduler.prepare(200_000_000, lambda: [], alive=lambda: not exited)
    assert not scheduler.healthy and scheduler.take_due(200_000_000) == []
    assert scheduler.events[0]['kind'] == 'process_fault'
    assert scheduler.events[0]['time_ns'] == 200_000_000
    assert 'exited' in scheduler.failure if exited else 'timeout' in scheduler.failure


def test_reset_evidence_preserves_all_failures_and_counts():
    gate = VisualReadiness('m', 's', 'c')
    for n in range(10):
        gate.observe(image(n*100_000_000), n*100_000_000+200_000_000, accepted=True,
            relative_position_covariance=np.eye(3)*1e-6,
            relative_attitude_covariance=np.eye(3)*1e-6)
    bad = image(1_000_000_000)
    bad['ambiguous'] = True
    bad['max_corner_error_px']['left'] = 6.
    gate.observe(bad, 1_200_000_000, accepted=False,
        relative_position_covariance=np.eye(3)*.04**2,
        relative_attitude_covariance=np.eye(3)*1e-6)
    record = gate.revocations[-1]
    assert record['count_before'] == 10 and record['count_after'] == 0
    assert len(record['failures']) == 4


def test_in_flight_images_keep_future_delivery_epochs_across_physical_handoff():
    a = FixedStereoDelivery(clock=lambda: 10.)
    a.expect(0, '0')
    a.expect(100_000_000, '100000000')
    a.prepare(150_000_000, lambda: [image(0), image(100_000_000)])
    b = FixedStereoDelivery(clock=lambda: 999.)
    b.restore(a.snapshot())
    b.prepare(200_000_000, lambda: [])
    assert [p['sim_time_ns'] for p in b.take_due(200_000_000)] == ['0']
    b.prepare(300_000_000, lambda: [])
    assert [p['sim_time_ns'] for p in b.take_due(300_000_000)] == ['100000000']


def test_delayed_image_uses_encoder_geometry_at_exposure_not_at_reception():
    from test_pose_navigation import navigation
    a, b = navigation(), navigation()
    assert a.visual(image(0))
    for nav in (a, b):
        nav.advance(100_000_000)
        assert nav.encoder_geometry(100_000_000, [.5, 0., 0.], [0., .3, 0.], np.zeros((2, 3)))
        nav.advance(200_000_000)
    assert b.visual(image(0))
    for key in ('p', 'v', 'r', 'b', 'covariance', 'camera_com_offset', 'target_com'):
        np.testing.assert_array_equal(a.state[key], b.state[key])


def test_encoder_com_velocity_and_inertia_follow_articulated_mass(tmp_path):
    from simulation.assembly_geometry import AssemblyGeometry
    model = tmp_path/'model.xml'
    model.write_text('''<mujoco><worldbody><body name="bus"><freejoint/>
        <body name="link" pos="1 0 0"><joint name="hinge" axis="0 0 1"/>
        <site name="tool" pos="1 0 0"/></body></body></worldbody></mujoco>''')
    props = {'bus': [2., 0., 0., 0., 1., 0., 0., 0., .1, .1, .1],
             'link': [1., .5, 0., 0., 1., 0., 0., 0., .02, .02, .02]}
    geometry = AssemblyGeometry(model, props.__getitem__, roots=('bus',))
    before = geometry.evaluate(0, {'hinge': [.4, .2]})['bus']
    after = geometry.evaluate(10_000, {'hinge': [.4+.2e-5, .2]})['bus']
    numerical = (np.array(after['com_body_m'])-before['com_body_m'])/1e-5
    np.testing.assert_allclose(numerical, before['com_velocity_body_m_s'], atol=4e-8)
    assert np.linalg.eigvalsh(before['inertia_com_body_kg_m2']).min() > 0
    assert before['inertia_com_body_kg_m2'] != after['inertia_com_body_kg_m2']


def test_uncertified_terminal_and_legacy_missing_extension_cannot_be_silently_certified():
    from simulation.capture26_mission import load_mission, validate_certificate
    root = Path(__file__).resolve().parents[1]
    config = load_mission(root)
    assert config.is_demo and config.terminal_depth_m != .5
    assert config.operating_geometry_sha256
    assert not config.vision_nodes_validated
    assert config.geometry_certificate_sha256 is None
    with pytest.raises(ValueError, match='certificate missing'):
        validate_certificate(root, config, None)


def test_replay_revocation_of_an_earlier_counted_frame_resets_a_long_streak():
    gate = VisualReadiness('m', 's', 'c')
    for n in range(15):
        gate.observe(image(n*100_000_000), n*100_000_000+200_000_000, accepted=True,
            relative_position_covariance=np.eye(3)*1e-6, relative_attitude_covariance=np.eye(3)*1e-6)
    gate.reconcile({gate._accepted_events[1]: False}, 1_600_000_000)
    assert gate.count == 0 and not gate.ready
    assert gate.revocations[-1]['category'] == 'navigation_replay'


def test_acquisition_can_reach_its_reference_without_an_accepted_image():
    approach, state = terminal()
    approach.node, approach.depth = 2, approach.config.acquisition_depth_m+.01
    offset, rotation = desired_geometry(approach.depth, np.eye(3), np.zeros(3), np.zeros(3), approach.vision)
    state['p'][0], state['r'][0] = offset, rotation
    step(approach, state, 0)
    result = step(approach, state, 50_000_000)
    assert approach.depth < approach.config.acquisition_depth_m+.01
    assert result['visual_safety_boundary_m'] == approach.config.readiness_depth_m


def test_independent_platform_audit_recomputes_dwell_and_catches_false_release():
    from tools.score_capture26_platform import audit_rows
    approach, state = terminal()
    rows = []
    for now in range(0, 5_000_000_001, 50_000_000):
        result = step(approach, state, now, True)
        rows.append(dict(time_ns=now, stage='near', depth_m=approach.depth, node='PlatformStandoff',
            position_n_m=state['p'].tolist(), velocity_n_m_s=state['v'].tolist(),
            rotation_n_from_body=state['r'].tolist(), rates_body_rad_s=state['rates'].tolist(),
            com_body_m=np.zeros((2, 3)).tolist(), com_velocity_body_m_s=np.zeros((2, 3)).tolist(),
            gyro_health=[True, True], process_healthy=True, safe_hold=False, last_gnss_ns=now,
            last_cooperative_ns=now, position_covariance_m2=(np.eye(3)*1e-6).tolist(),
            attitude_covariance_rad2=(np.eye(3)*1e-6).tolist(), visual_ready=True, visual_count=10,
            released=result['released'], reported_platform_stable=result['platform_stable']))
    report = audit_rows(rows, approach.config.to_dict(), approach.vision)
    assert report['audit_passed'] and report['first_release_ns'] == 5_000_000_000
    rows[1]['released'] = True
    report = audit_rows(rows, approach.config.to_dict(), approach.vision)
    assert not report['audit_passed'] and report['premature_release_epochs_ns'] == [50_000_000]


def test_preparation_tracking_failure_freezes_reference_without_physical_access(tmp_path):
    import hashlib
    from simulation.capture26_preparation import JointPreparation
    model = tmp_path/'robot.xml'
    model.write_text('<mujoco><worldbody><body>'+''.join(
        f'<joint name="{name}{i}" range="-3 3"/>' for name in ('joint', 'bottom_joint') for i in range(1, 7))
        +'</body></worldbody></mujoco>')
    plan = dict(schema='space-sim-capture26-preparation/1', model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        waypoints_rad=[[0.]*12, [.1]*12], maximum_rate_rad_s=.1, maximum_acceleration_rad_s2=.04)
    preparation = JointPreparation(plan, model)
    preparation.update(0, np.zeros(12), np.zeros(12))
    initial = preparation.reference.copy()
    np.testing.assert_array_equal(preparation.update(10_000_000, np.ones(12), np.zeros(12)), initial)
    assert preparation.failed and not preparation.complete


def test_status_reads_cannot_accumulate_unobserved_platform_stability():
    stable = PlatformStability()
    stable.update(0, ['optical_depth_m', 1.], {'health': True})
    before = stable.snapshot()
    status = stable.status(6_000_000_000)
    assert not status['platform_stable'] and status['platform_stable_duration_s'] == 0.
    assert stable.snapshot() == before


def test_malformed_optional_wall_diagnostics_do_not_reject_physical_observations():
    from test_pose_navigation import navigation
    packet = image(0)
    packet['capture_timing'] = {eye: {'source_wall_time_ns': 'unknown'} for eye in ('left', 'right')}
    nav = navigation()
    nav.advance(200_000_000)
    assert nav.visual(packet, received_wall_time_ns=9_000_000_000)
    gate = VisualReadiness('m', 's', 'c', minimum=1)
    assert gate.observe(packet, 200_000_000, accepted=True,
        relative_position_covariance=np.eye(3)*1e-6, relative_attitude_covariance=np.eye(3)*1e-6,
        received_wall_time_ns=9_000_000_000)


def test_preparation_restart_cannot_claim_completion_or_change_joint_reference():
    import hashlib
    from simulation.capture26_mission import MODEL_PATH
    from simulation.capture26_preparation import JointPreparation
    model = Path(__file__).resolve().parents[1]/MODEL_PATH
    document = dict(schema='space-sim-capture26-preparation/1',
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        waypoints_rad=[[0.]*12, [.1]*12], maximum_rate_rad_s=.1, maximum_acceleration_rad_s2=.04)
    a, b = JointPreparation(document, model), JointPreparation(document, model)
    b.restore(a.snapshot())
    a.update(0, np.zeros(12), np.zeros(12))
    saved = a.snapshot()
    b.restore(saved)
    for field, value in [('complete', True), ('reference_rad', [.1]*12), ('elapsed_s', float('nan'))]:
        broken = deepcopy(saved)
        broken[field] = value
        with pytest.raises(ValueError):
            b.restore(broken)
