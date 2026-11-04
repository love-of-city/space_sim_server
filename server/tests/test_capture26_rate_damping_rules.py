"""Capture26 angular rules and independent rotating-body geometry regressions."""
from copy import deepcopy
import json

import numpy as np
import pytest

from simulation.capture26_mission import Capture26Mission, load_mission, translation_reference
from simulation.capture26_flight import Capture26Flight
from simulation.pose_navigation import exp
from space_arm_platform.visual_navigation import VisualReadiness
from test_capture26_readiness_rules import terminal, step
from test_pose_navigation import image, navigation


def test_large_attitude_error_and_uncertainty_do_not_delay_tenth_frame_release():
    approach, state = terminal()
    state['r'][0] = exp([.8, -.4, .2])
    state['p'][0] = translation_reference(approach.depth, state, approach.service_com,
        approach.target_com, approach.vision)[0]
    gate = VisualReadiness('m', 's', 'c')
    large = np.eye(3)*np.deg2rad(30.)**2
    first_release = None
    for now in range(0, 6_100_000_001, 50_000_000):
        if now >= 5_200_000_000 and now % 100_000_000 == 0:
            packet = image(now-200_000_000)
            measured = np.array(packet['pose_covariance']['matrix'])
            measured[3:, 3:] = large
            packet['pose_covariance']['matrix'] = measured.tolist()
            gate.observe(packet, now, accepted=True, relative_position_covariance=np.eye(3)*1e-6,
                         relative_attitude_covariance=large)
        gate.check_uncertainty(np.eye(3)*1e-6, large, now)
        result = step(approach, state, now, gate.ready, attitude_sigma_rad=np.deg2rad(30.))
        if result['released'] and first_release is None:
            first_release = now
    assert gate.count == 10 and first_release == 6_100_000_000
    assert result['platform_stable_duration_s'] == pytest.approx(6.1)
    nav = navigation()
    packet = image(0)
    packet['pose_covariance']['matrix'] = np.diag([.002**2]*3+[np.deg2rad(30.)**2]*3).tolist()
    assert nav.visual(packet)  # Full 6D fusion remains active.


@pytest.mark.parametrize('invalid', ['rotation', 'singular', 'indefinite', 'nonfinite', 'cross_covariance'])
def test_invalid_pose_and_full_covariance_still_fail_navigation(invalid):
    packet = image(0)
    matrix = np.array(packet['pose_covariance']['matrix'])
    if invalid == 'rotation':
        packet['pose_target_to_left']['R'][0][0] = 2.
    elif invalid == 'singular':
        matrix[3, 3] = 0.
    elif invalid == 'indefinite':
        matrix[3, 3] = -1.
    elif invalid == 'nonfinite':
        matrix[3, 3] = float('nan')
    else:
        matrix[0, 3] = matrix[3, 0] = 1.  # Both diagonal blocks valid; full matrix invalid.
    packet['pose_covariance']['matrix'] = matrix.tolist()
    assert not navigation().visual(packet)


@pytest.mark.parametrize('service_rate,stable', [(.001, True), (.001000001, False)])
def test_only_service_rate_magnitude_limits_stability(service_rate, stable):
    approach, state = terminal()
    state['rates'] = np.array([[service_rate, 0., 0.], [.1, -.2, .3]])
    state['v'][0] = translation_reference(approach.depth, state, approach.service_com,
        approach.target_com, approach.vision)[1]
    for now in range(0, 5_000_000_001, 50_000_000):
        result = step(approach, state, now, True)
    assert result['service_angular_rate_stable'] is stable
    assert result['platform_stable'] is stable and result['released'] is stable
    if not stable:
        assert 'service_angular_rate' in result['platform_failures']


def test_far_service_rate_fault_restarts_original_120_second_dwell():
    flight = Capture26Flight('in-orbit')
    start = flight.profile.start_approach_s+flight.profile.approach_duration_s
    def at(seconds, rate):
        p0, v0, _ = flight.profile.chaser(seconds)
        p1, v1, _ = flight.profile.target(seconds)
        return flight.step(round(seconds*1e9), dict(p=np.array([p0, p1]), v=np.array([v0, v1]),
            rates=np.array([[rate, 0., 0.], [0., .2, 0.]]), b=np.zeros((2, 3))), healthy=True)
    assert not at(start, .001)['released']
    assert at(start+120., .001)['released']
    assert not at(start+120.05, .001000001)['released']
    assert not at(start+120.1, .001)['released']
    assert not at(start+240.05, .001)['released']
    assert at(start+240.1, .001)['released']


@pytest.mark.parametrize('radius', [None, 50.])
def test_independent_body_lever_velocity_matches_numerical_geometry_derivative(radius):
    approach, state = terminal()
    state['r'] = np.array([exp([.7, -.4, .2]), exp([-.3, .2, .5])])
    state['rates'] = np.array([[.0003, -.0005, .0001], [.02, .01, -.03]])
    dc = np.array([[.002, -.001, .0003], [-.001, .003, .002]])
    state['com_velocity_body'] = dc
    cs, ct = np.array([.1, -.2, .3]), np.array([-.2, .03, .01])
    depth, depth_rate, radial_rate, dt = 3., -.01, -.04, 1e-6
    offset, velocity, effective_depth = translation_reference(depth, state, cs, ct, approach.vision,
        depth_rate=depth_rate, com_distance_m=radius, com_distance_rate_m_s=radial_rate)
    moved = deepcopy(state)
    moved['r'] = np.array([r@exp(w*dt) for r, w in zip(state['r'], state['rates'])])
    after = translation_reference(depth+depth_rate*dt, moved, cs+dc[0]*dt, ct+dc[1]*dt, approach.vision,
        com_distance_m=None if radius is None else radius+radial_rate*dt)[0]
    np.testing.assert_allclose((after-offset)/dt, velocity, atol=8e-8)
    if radius:
        assert np.linalg.norm(offset) == pytest.approx(50.)
        assert offset@velocity/50. == pytest.approx(radial_rate)
    marker_n = state['r'][1]@np.array([.1755, .05475, .0420787001299893])
    camera = .5*sum(np.array(approach.vision['body_from_optical'][eye]['position_body_m']) for eye in ('left', 'right'))
    camera_n = offset+state['r'][1]@ct+state['r'][0]@(camera-cs)
    np.testing.assert_allclose(marker_n-camera_n, effective_depth*(-state['r'][1][:, 1]), atol=1e-14)


@pytest.mark.parametrize('node,stage', [(0, 'near'), (4, 'near'), (4, 'operations')])
def test_rotating_references_preserve_dwell_and_independent_scoring(node, stage):
    from tools.score_capture26_platform import audit_rows
    approach, state = terminal()
    approach.node = node
    if node == 0:
        approach.com_distance_m = 50.
    initial = np.array([exp([.7, -.4, .2]), exp([-.3, .2, .5])])
    state['rates'] = np.array([[.0008, 0., 0.], [0., .02, 0.]])
    state['com_velocity_body'] = np.array([[.0001, -.0002, .0003], [-.0002, 0., .0001]])
    rows = []
    for now in range(0, 5_000_000_001, 50_000_000):
        state['r'] = np.array([r@exp(w*now*1e-9) for r, w in zip(initial, state['rates'])])
        cs, ct = state['com_velocity_body']*now*1e-9
        approach.service_com, approach.target_com = cs, ct
        offset, velocity, _ = translation_reference(approach.depth, state, cs, ct, approach.vision,
            com_distance_m=50. if node == 0 else None)
        state['p'][0], state['v'][0] = offset, velocity
        result = step(approach, state, now, True, attitude_sigma_rad=.2)
        rows.append(dict(time_ns=now, stage=stage, depth_m=approach.depth, node=result['node'],
            reference_com_distance_m=result['reference_com_distance_m'],
            position_n_m=state['p'].tolist(), velocity_n_m_s=state['v'].tolist(),
            rotation_n_from_body=state['r'].tolist(), rates_body_rad_s=state['rates'].tolist(),
            com_body_m=[cs.tolist(), ct.tolist()], com_velocity_body_m_s=state['com_velocity_body'].tolist(),
            gyro_health=[True, True], process_healthy=True, safe_hold=False, last_gnss_ns=now,
            last_cooperative_ns=now, position_covariance_m2=(np.eye(3)*1e-6).tolist(),
            attitude_covariance_rad2=(np.eye(3)*.2**2).tolist(), visual_ready=True, visual_count=10,
            released=result['released'], reported_platform_stable=result['platform_stable']))
    report = audit_rows(rows, approach.config.to_dict(), approach.vision)
    assert report['audit_passed'] and report['first_platform_stable_ns'] == 5_000_000_000
    assert result['platform_stable_duration_s'] == pytest.approx(5.)
    if node == 0:
        assert np.linalg.norm(result['position_goal']-state['p'][1]) == pytest.approx(50.)
        assert approach.node == 1
    else:
        assert report['first_release_ns'] == 5_000_000_000
    rows[-1]['rates_body_rad_s'][0][0] = .001000001
    assert not audit_rows(rows, approach.config.to_dict(), approach.vision)['audit_passed']


def test_changed_frozen_attitude_rules_cannot_inherit_old_stability(tmp_path):
    approach, state = terminal()
    step(approach, state, 0)
    old = approach.snapshot()
    del old['configuration']['attitude_control_mode']
    with pytest.raises(ValueError, match='configuration changed'):
        approach.restore(old)
    path = tmp_path/'config/missions/capture26-v2.json'
    path.parent.mkdir(parents=True)
    document = dict(schema='space-sim-capture26-task/2', mission=Capture26Mission().to_dict())
    del document['mission']['service_angular_rate_limit_rad_s']
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='explicit attitude rules'):
        load_mission(tmp_path)
