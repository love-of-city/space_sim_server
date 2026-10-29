"""Parking-clock, frame binding, safety and diagnostic-isolation contracts.

These synthetic state fixtures test decisions; native runs provide trajectory
and actual continuous parking evidence separately.
"""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from simulation.capture26_approach import Capture26DemoApproach
from simulation.capture26_mission import load_mission, load_vision
from simulation.capture26_demo_geometry import load_geometry, axial_gap, working_reference
from simulation.capture26_flight import Capture26Flight
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.reference import FlightProfile
from simulation.sensor_delivery import FixedStereoDelivery

ROOT = Path(__file__).resolve().parents[1]


def fixture(node=0):
    c = load_mission(ROOT)
    g = load_geometry(ROOT, c)
    a = Capture26DemoApproach(np.zeros(3), np.zeros(3), load_vision(ROOT), config=c, geometry=g)
    p, v, _ = FlightProfile().target(5000.)
    f, w = vbar_frame(np.r_[p, v])
    target_rotation = np.column_stack((f[:, 1], -f[:, 0], f[:, 2]))
    # Choose a proper right-handed frame with the surface normal towards -T.
    target_rotation[:, 0] = np.cross(target_rotation[:, 1], target_rotation[:, 2])
    s = dict(p=np.array([p-110*f[:, 0], p]), v=np.array([v, v]),
        r=np.array([target_rotation@np.array(g['nominal_service_rotation_target']), target_rotation]),
        rates=np.zeros((2, 3)), b=np.zeros((2, 3)), com_velocity_body=np.zeros((2, 3)))
    a.initial_local, a.acquired, a.node = [-110., 0., 0.], True, node
    a.leg_start_ns, a.leg_duration_s = -20_000_000_000, 10.
    offset, rate, _ = a._endpoint(s, node, f, w)
    s['p'][0], s['v'][0] = p+offset, v+rate
    return a, s


def step(a, s, t, **kw):
    return a.step(round(t*1e9), s, mass_kg=200., visual_ready=False, visual_available=False,
        gyro_healthy=[False, False], gnss_healthy=False, communication_healthy=False,
        process_healthy=False, position_sigma_m=100., **kw)


def test_frozen_model_geometry_has_a_half_metre_gap_and_distinct_depth():
    a, s = fixture(3)
    assert axial_gap(s, a.service_com, a.target_com, a.geometry) == pytest.approx(.5, abs=1e-9)
    assert a.config.terminal_depth_m == pytest.approx(.5434731996552936)
    assert np.dot(a.geometry['lateral_target_m'], a.geometry['surface_normal_target']) == pytest.approx(0.)
    assert a.geometry['minimum_inter_spacecraft_clearance_m'] >= .1
    assert a.geometry['folded_capture_joints_rad'] == [0.]*6


@pytest.mark.parametrize('node', range(4))
@pytest.mark.parametrize('fault', ['position', 'velocity'])
def test_each_node_requires_a_new_continuous_five_seconds_after_physical_error(node, fault):
    a, s = fixture(node)
    # Earlier nodes are fixture metadata, never acceptance evidence.
    a.audit = [dict(node=n) for n in a.NODE_IDS[:node]]
    for i in range(101):
        step(a, s, i*.01)
    key = 'p' if fault == 'position' else 'v'
    delta = .051 if key == 'p' else .0101
    s[key][0, 0] += delta
    failed = step(a, s, 1.01)
    assert not failed['platform_stable'] and failed['dwell_s'] == 0.
    s[key][0, 0] -= delta
    original = deepcopy(s)
    for i in range(102, 602):
        assert not step(a, s, i*.01)['platform_stable']
    result = step(a, s, 6.02)
    assert result['platform_stable'] and a.audit[-1]['node'] == a.NODE_IDS[node]
    assert a.audit[-1]['dwell_start_ns'] == 1_020_000_000
    for k in s:
        np.testing.assert_array_equal(s[k], original[k])


def test_final_dwell_is_latched_once_and_the_working_reference_ignores_arm_deployment():
    a, s = fixture(3)
    a.audit = [dict(node=n) for n in a.NODE_IDS[:3]]
    for i in range(501):
        result = step(a, s, i*.01)
    assert result['released']
    records = deepcopy(a.audit)
    old = working_reference(s, a.service_com, a.target_com, a.geometry)[0]
    s['tcp_body_m'] = np.array(a.geometry['folded_tcp_body_m'])+[.2, 0., .1]
    for i in range(501, 601):
        result = step(a, s, i*.01)
    assert a.audit == records and result['dwell_s'] == 5.
    np.testing.assert_array_equal(working_reference(s, a.service_com, a.target_com, a.geometry)[0], old)
    step(a, s, 6.01, safety_reason='emergency stop')
    assert a.safe_hold and not a.released and a.audit == records


def test_far_five_second_dwell_is_position_velocity_only_and_rejects_missing_epochs():
    a = Capture26Flight('in-orbit', config=load_mission(ROOT))
    start = a.profile.approach_duration_s
    def at(t):
        p0, v0, _ = a.profile.chaser(t)
        p1, v1, _ = a.profile.target(t)
        s = dict(p=np.array([p0, p1]), v=np.array([v0, v1]), rates=np.full((2, 3), .2), b=np.zeros((2, 3)))
        return a.step(round(t*1e9), s, healthy=True)
    for i in range(100):
        assert not at(start+i*.05)['released']
    assert at(start+5.)['released']
    assert not at(start+5.5)['released']


def test_diagnostic_image_deadline_never_waits_for_a_worker():
    d = FixedStereoDelivery(clock=lambda: 0., pause=lambda _: pytest.fail('diagnostics blocked physics'))
    d.expect(0, '0')
    d.prepare_diagnostic(200_000_000, lambda: [])
    assert not d.pending and not d.take_due(200_000_000)
    assert d.events[-1]['result_available'] is False
    assert d.status()['waiting_wall_s'] == 0.


def test_independent_audit_rejects_fabricated_completion_and_out_of_order_nodes():
    from tools.score_capture26_demo import audit_near
    a, _ = fixture(3)
    g, vision, config = a.geometry, a.vision, a.config.to_dict()
    r0 = np.diag([-1., -1., 1.])
    rs0 = r0@np.array(g['nominal_service_rotation_target'])
    axis0 = rs0@np.array(vision['body_from_optical']['left']['R_body_from_optical'])[:, 2]
    camera = np.array(vision['mount_center_body_m'])
    marker = np.array([.1755, .05475, .0420787001299893])
    correction = axis0@(r0@marker-rs0@camera)
    radius, rate = 7.e6, .0011
    offsets = [np.array([0., -50., 0.]), np.array([0., -(17.4-correction), 0.]),
        np.array([0., -(5.5-correction), 0.]),
        r0@(np.array(g['surface_point_target_m'])+.5*np.array(g['surface_normal_target'])
             +g['lateral_target_m'])-rs0@np.array(g['folded_tcp_body_m'])]
    rows = []
    for node, offset0 in zip(a.NODE_IDS, offsets):
        for _ in range(121):
            t = len(rows)*.05
            angle = rate*t
            r = np.array([[np.cos(angle), -np.sin(angle), 0.],
                [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
            target = r@np.array([radius, 0., 0.])
            offset = r@offset0
            target_velocity = np.cross([0., 0., rate], target)
            rows.append(dict(time_ns=round(t*1e9), node=node, feedback_source='authoritative_simulation',
                position_n_m=[(target+offset).tolist(), target.tolist()],
                velocity_n_m_s=[(target_velocity+np.cross([0., 0., rate], offset)).tolist(), target_velocity.tolist()],
                rotation_n_from_body=[(r@rs0).tolist(), (r@r0).tolist()], com_body_m=np.zeros((2, 3)).tolist(),
                tcp_body_m=g['folded_tcp_body_m'], at_node=True, safe_hold=False,
                released=True, reported_platform_stable=True))
    accepted = audit_near(rows, config, g, vision)
    assert accepted['audit_passed'] and len(accepted['records']) == 4
    bad = deepcopy(rows)
    for row in bad[:121]:
        row['position_n_m'][0][2] += .051
    assert not audit_near(bad, config, g, vision)['audit_passed']
    assert not audit_near(rows[121:242]+rows[:121]+rows[242:], config, g, vision)['audit_passed']
