"""Trajectory, frame and release regressions independent of native dynamics."""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from simulation.near_field.approach_plan import DwellGate
from simulation.near_field.reference import RestLeg, offset_path
from simulation.near_field.center_kinematics import CenterKinematics, center_relative
from simulation.near_field.navigation import VisualReadiness
from simulation.near_field.vbar_approach import VbarApproach
from simulation.near_field.orbital_guidance import vbar_frame, rotating_acceleration
from simulation.orbital_rendezvous.model import DEFAULT_MODEL, assembly_properties


def test_dwell_requires_101_distinct_samples_and_resets_only_continuous_clock():
    gate = DwellGate(timeout_s=60.)
    for i in range(100):
        assert not gate.update(i*.05, holding=True, qualified=True)
    assert gate.dwell_s == pytest.approx(4.95)
    assert not gate.update(4.95, holding=True, qualified=True)
    assert gate.samples == 100
    assert not gate.update(5., holding=True, qualified=False)
    assert gate.waiting_since_s == 0.
    for i in range(100):
        assert not gate.update(5.05+i*.05, holding=True, qualified=True)
    assert gate.update(10.05, holding=True, qualified=True)
    assert gate.update(11., holding=True, qualified=False)
    assert gate.telemetry(100.)['wait_s'] == pytest.approx(10.05)
    assert DwellGate().completed_at_s is None


def test_wait_and_capture_timeouts_do_not_charge_cruise_or_reset():
    gate = DwellGate(timeout_s=60.)
    gate.update(1000., holding=False, braking=False, qualified=False)
    assert gate.failure(1500.) is None
    gate.update(1500., holding=False, braking=True, qualified=False)
    assert gate.failure(1799.) is None
    assert gate.failure(1800.) == 'node_capture_timeout'
    gate.update(1800., holding=True, qualified=False)
    for t in [1801., 1830., 1859.]:
        gate.update(t, holding=True, qualified=False)
        assert gate.failure(t) is None
    assert gate.failure(1860.) == 'node_wait_timeout'


@pytest.mark.parametrize('distance,vmax', [(60., .1), (32.3, .1), (12., .02), (4.5, .0047), (.0001, .02)])
def test_analytic_legs_bound_derivatives_and_stop_without_overshoot(distance, vmax):
    leg = RestLeg(110., 110.-distance, vmax)
    times = np.linspace(0, leg.duration, 2001)
    samples = np.array([leg.sample(t) for t in times])
    np.testing.assert_allclose(samples[0], [110., 0., 0.], atol=1e-10)
    np.testing.assert_allclose(samples[-1], [110.-distance, 0., 0.], atol=1e-10)
    assert abs(samples[:, 1]).max() <= vmax+1e-12
    assert abs(samples[:, 2]).max() <= .002+1e-12
    assert np.all(np.diff(samples[:, 0]) <= 1e-12)
    for duration, _ in leg.phases:
        assert duration >= 0
    h = 1e-4
    for t in times[100:-100:100]:
        numerical = (leg.sample(t+h)-leg.sample(t-h))/(2*h)
        np.testing.assert_allclose(numerical[:2], leg.sample(t)[1:], atol=1e-7)
        assert abs(numerical[2]) <= .002+1e-8


def test_offset_is_c2_and_matches_derivatives_along_same_path():
    transverse = np.array([-.10961, .56096])
    for s in [5.75336, 2.25336]:
        p, v, a, _ = offset_path([s, -.0047, 0.], 5.75336, 2.25336, transverse)
        np.testing.assert_allclose(v[1:], 0., atol=1e-12)
        np.testing.assert_allclose(a[1:], 0., atol=1e-12)
    s, ds, h = 4., -.0047, .001
    p, v, a, _ = offset_path([s, ds, 0.], 5.75336, 2.25336, transverse)
    before = offset_path([s-ds*h, ds, 0.], 5.75336, 2.25336, transverse)
    after = offset_path([s+ds*h, ds, 0.], 5.75336, 2.25336, transverse)
    np.testing.assert_allclose((after[0]-before[0])/(2*h), v, atol=1e-9)
    np.testing.assert_allclose((after[1]-before[1])/(2*h), a, atol=1e-9)


def test_articulated_centers_match_authored_mass_and_velocity_finite_difference():
    root = ET.parse(Path(__file__).resolve().parents[1]/DEFAULT_MODEL).getroot()
    for name in ['capture_target', 'cubesat_bus']:
        body = root.find(f'./worldbody/body[@name="{name}"]')
        fk = CenterKinematics(body)
        np.testing.assert_allclose(fk.evaluate({})[0], assembly_properties(body)['center_body_m'], atol=1e-12)
        q = {'outer_panel_hinge': .2, 'joint2': .3, 'joint_finger1': .02, 'joint_finger2': .03}
        dq = dict.fromkeys(q, .01)
        p, v = fk.evaluate(q, dq)
        h = 1e-5
        before = fk.evaluate({k: x-h*dq[k] for k, x in q.items()})[0]
        after = fk.evaluate({k: x+h*dq[k] for k, x in q.items()})[0]
        np.testing.assert_allclose((after-before)/(2*h), v, atol=1e-9)


def test_center_frame_transforms_rotating_lever_velocity_and_covariance():
    Q = np.array([[0., 0, 1], [1., 0, 0], [0., 1., 0]])
    omega = Q@np.array([0., .001, 0.])
    ct, cb = np.array([.2, .1, 0]), np.array([-.1, .2, .3])
    position = Q@np.array([110., 0., 0.])-Q@(ct-cb)
    relative = np.r_[position, np.cross(omega, position)]
    state, P = center_relative(relative, np.eye(6), Q, omega, Q, Q,
        Q.T@omega, Q.T@omega, (ct, np.zeros(3)), (cb, np.zeros(3)))
    np.testing.assert_allclose(state, [110., 0., 0., 0., 0., 0.], atol=1e-12)
    assert np.linalg.eigvalsh(P).min() > 0
    np.testing.assert_allclose(P[:3, :3], np.eye(3), atol=1e-12)
    assert not np.allclose(P[3:, :3], 0)


def test_circular_vbar_closing_requires_inward_radial_chaser_force():
    mu, radius, mass = 3.986004418e14, 6878136.6, 169.4
    n = np.sqrt(mu/radius**3)
    target = np.array([radius, 0., 0., 0., n*radius, 0.])
    Q, omega = vbar_frame(target)
    delta, velocity = Q@[50., 0., 0.], Q@[-.1, 0., 0.]
    radial = np.array([1., 0., 0.])
    gradient = mu/radius**3*(3*np.outer(radial, radial)-np.eye(3))
    force = mass*(gradient@delta-rotating_acceleration(omega, delta, velocity))
    assert (Q.T@force)[2] == pytest.approx(-.0375, abs=.0001)


def packet(stamp, **geometry):
    return {'stamp': stamp, 'geometry': dict(valid=True, common_fov=True, marker_px=25.3, depth_sigma_m=.029) | geometry}


def test_visual_readiness_is_distinct_current_and_revocable():
    quality = VisualReadiness()
    for i in range(10):
        quality.observe(packet(i*.1), True, .02)
    assert quality.current(.9, .02)
    assert quality.current(1.2, .02)
    assert not quality.current(1.26, .02)
    assert quality.streak == 0

    for i in range(10):
        quality.observe(packet(2.+i*.1), True, .02)
    assert quality.current(2.9, .02)
    assert not quality.current(2.9, .041)
    quality.observe(packet(3.), False, .02)
    assert quality.streak == 0
    for _ in range(10):
        quality.observe(packet(3.1), True, .02)
    assert not quality.current(3.1, .02)
    quality.observe(packet(3.2, marker_px=23.9), True, .02)
    assert quality.streak == 0


def test_visual_updates_must_remain_continuous_before_age_limit():
    quality = VisualReadiness()
    for i in range(10):
        quality.observe(packet(i*.1), True, .02, now=i*.1+.15)
    assert quality.current(1.15, .02)
    assert not quality.current(1.21, .02)  # 0.31 s observation age, 0.16 s without update
    assert quality.streak == 0


def test_safe_stop_setpoint_is_ahead_in_motion_direction_and_latches():
    approach = VbarApproach()
    state = np.array([20., 0., 0., -.1, 0., 0.])
    approach.fail('lost_link', state)
    stopping_point = approach.safe_reference.copy()
    assert stopping_point[0] < 19.
    approach.fail('different', state*2)
    assert approach.failure_reason == 'lost_link'
    np.testing.assert_array_equal(approach.safe_reference, stopping_point)


@pytest.mark.parametrize('vision_enabled', [True, False])
def test_mandatory_nodes_with_early_or_missing_vision(vision_enabled):
    approach = VbarApproach()
    state = np.array([110., 0., 0., 0., 0., 0.])
    covariance = np.diag([1e-6]*3+[1e-10]*3)
    hold = np.array([1.24836, -.10961, .56096])
    for tick in range(72001):
        now = tick*.05
        p, v, a = approach.update(now, state, covariance, common=True, sigma=.001,
            visual_valid=state[0]-.25336 < 17.39,
            visual_ready=vision_enabled, optical_depth=state[0]-.25336,
            optical_s=lambda depth: depth+.25336, hold=hold, offset_end_s=2.25336,
            geometry_ready=True, available_acceleration=.1, observation_age=.1,
            platform_ready=np.linalg.norm(state[:3]-hold) < .001)
        state = np.r_[p+.05*v+.00125*a, v+.05*a]
        if not approach.gates['VisualReady5p5m'].completed_at_s:
            np.testing.assert_allclose(p[1:], 0., atol=1e-12)
        if approach.gates['PlatformStandoff'].completed_at_s or approach.failure_reason:
            break
    assert approach.gates['Vbar50m'].completed_at_s is not None
    assert approach.gates['Optical17p4m'].completed_at_s is not None
    if vision_enabled:
        assert approach.gates['VisualReady5p5m'].completed_at_s is not None
        assert approach.failure_reason is None
        assert approach.gates['PlatformStandoff'].completed_at_s is not None
        assert approach.offset_progress == 1.
    else:
        assert approach.gates['VisualReady5p5m'].completed_at_s is None
        assert approach.failure_reason == 'VisualReady5p5m:node_wait_timeout'
        assert approach.offset_progress == 0.
        assert now-approach.gates['VisualReady5p5m'].waiting_since_s >= 60.


def test_stable_serialization_preserves_relative_covariance_and_same_roe_uncertainty():
    from simulation.near_field.telemetry import near_navigation_solution
    from simulation.relative_navigation import near_navigation_solution as shared
    from simulation.orbital_rendezvous.reference import FlightProfile
    r, v, _ = FlightProfile().target(2700.)
    target = np.r_[r, v]
    relative = np.array([1., .1, .5, .001, 0., 0.])
    Pt = np.diag([.2]*3+[.01]*3)
    Pr = np.diag([1e-6]*3+[1e-8]*3)
    old = shared(target, relative, Pt, Pr, 2700.)
    new = near_navigation_solution(target, relative, Pt, Pr, 2700.)
    np.testing.assert_allclose(new['roe']['covariance'], old['roe']['covariance'], rtol=1e-5, atol=1e-20)
    enormous = near_navigation_solution(target, relative, Pt*1e12, Pr, 2700.)
    np.testing.assert_array_equal(enormous['covariance'], Pr)
    assert np.isfinite(enormous['roe']['covariance']).all()
    assert np.linalg.eigvalsh(enormous['roe']['covariance']).min() >= -1e-14


@pytest.mark.parametrize('along, transverse, speed', [(100., 0., 0.), (125., 4.99, .1999)])
def test_unchanged_handoff_envelope_admits_extremes_and_rejects_outside(along, transverse, speed):
    from simulation.near_field.navigation import admit_handoff
    from simulation.orbital_rendezvous.reference import FlightProfile
    r, v, _ = FlightProfile().target(1000.)
    target = np.r_[r, v]
    Q, omega = vbar_frame(target)
    p = Q@np.array([along, transverse, 0.])
    dv = np.cross(omega, p)+Q@np.array([-speed, 0., 0.])
    # Avoid floating-point boundary ambiguity in the physical envelope itself.
    if along == 100.:
        p += 1e-8*Q[:, 0]
    if along == 125.:
        p -= 1e-8*Q[:, 0]
    bodies = [dict(body_id='rendezvous/target', position_m=r, velocity_m_s=v,
                   attitude_wxyz=[1, 0, 0, 0], angular_velocity_rad_s=[0, 0, 0]),
              dict(body_id='rendezvous/chaser', position_m=r-p, velocity_m_s=v-dv,
                   attitude_wxyz=[1, 0, 0, 0], angular_velocity_rad_s=[0, 0, 0])]
    handoff = dict(guidance_frame='orbital-vbar', source_kind='synthetic envelope test',
                   gnss_estimates_eci=[target, target-np.r_[p, dv]], covariances=[np.eye(6), np.eye(6)],
                   state=dict(bodies=bodies), stowed_joints=[])
    admit_handoff(handoff)
    bodies[1]['position_m'] = r-Q@[99., 0., 0.]
    with pytest.raises(ValueError, match='corridor'):
        admit_handoff(handoff)


@pytest.mark.parametrize('along, transverse, speed', [(100., 0., 0.), (125., 4.99, -.1999)])
def test_entry_controller_captures_legal_extremes_before_enabling_strict_corridor(along, transverse, speed):
    approach = VbarApproach()
    state = np.array([along, transverse, 0., speed, 0., 0.])
    for tick in range(12001):
        now = tick*.05
        p, v, a = approach.update(now, state, np.eye(6)*1e-8, common=True, sigma=.001,
            visual_valid=False, visual_ready=False, optical_depth=state[0]-.25336,
            optical_s=lambda depth: depth+.25336, hold=np.array([1.24836, -.10961, .56096]),
            offset_end_s=2.25336, geometry_ready=True, available_acceleration=.1, observation_age=.1)
        assert approach.failure_reason is None
        if approach.acquire.completed_at_s is not None:
            break
        acceleration = a-.06*(state[:3]-p)-.6*(state[3:]-v)
        state[:3] += .05*state[3:]+.00125*acceleration
        state[3:] += .05*acceleration
    assert approach.acquire.completed_at_s is not None
    assert np.linalg.norm(state[1:3]) < .10
    assert np.linalg.norm(state[3:]) < .002
    assert abs(p[0]-along) < 1e-12


@pytest.mark.parametrize('fault, expected', [('corridor', 'transverse_corridor'),
    ('braking', 'braking_authority_unverified'), ('boundary', 'optical_braking_margin')])
def test_new_safety_faults_brake_and_latch_even_after_conditions_recover(fault, expected):
    approach = VbarApproach()
    state = np.array([110., 0., 0., 0., 0., 0.])
    kwargs = dict(common=True, sigma=.001, visual_valid=False, visual_ready=False,
        optical_depth=110., optical_s=lambda depth: depth+.25336,
        hold=np.array([1.24836, -.10961, .56096]), offset_end_s=2.25336,
        geometry_ready=True, available_acceleration=.1, observation_age=.1)
    for tick in range(101):
        approach.update(tick*.05, state, np.eye(6)*1e-8, **kwargs)
    assert approach.acquire.completed_at_s is not None
    if fault == 'corridor':
        state[1] = 2.
    elif fault == 'braking':
        kwargs['available_acceleration'] = 0.
    else:
        kwargs['optical_depth'] = 12.04
        state[3] = -.02
    approach.update(5.05, state, np.eye(6)*1e-8, **kwargs)
    assert approach.failure_reason == expected
    endpoint = approach.safe_reference.copy()
    state[1] = 0.
    kwargs.update(available_acceleration=.1, optical_depth=110., visual_valid=True, visual_ready=True)
    approach.update(6., state, np.eye(6)*1e-8, **kwargs)
    assert approach.failure_reason == expected
    np.testing.assert_array_equal(approach.safe_reference, endpoint)
    assert approach.telemetry['subsegment'].startswith('SafeHold')


def test_physical_audit_rejects_fabricated_controller_completion_and_readiness():
    import json
    from simulation.near_field.acceptance import audit_vbar
    from simulation.near_field.approach_plan import approach_plan
    from simulation.serial_chain_kinematics import SerialChainKinematics
    from simulation.near_field.model import JOINTS, GRASP_TARGET
    source = Path(__file__).resolve().parents[1]/DEFAULT_MODEL
    chain = SerialChainKinematics.from_mjcf(source, base_body='cubesat_bus', joint_names=JOINTS[:6], tool_site='sarm_ee')
    plan = approach_plan(chain)
    target = np.array([6878136.6, 0., 0., 0., 7612., 0.])
    Q, omega = vbar_frame(target)
    delta = Q@np.array([110., 0., 0.])
    states = np.array([target, target-np.r_[delta, np.cross(omega, delta)]])
    depth = (Q@np.asarray(plan['grasp_axis_bus']))@(delta+Q@GRASP_TARGET)+.10
    root = ET.parse(source).getroot()
    centers = [CenterKinematics(root.find(f'./worldbody/body[@name="{name}"]')).evaluate({})[0]
               for name in ['capture_target', 'cubesat_bus']]
    center_distance = np.linalg.norm(delta+Q@(centers[0]-centers[1]))
    fake = dict(nodes={name:dict(completed_at_s=5.) for name in
        ['Vbar50m', 'Optical17p4m', 'VisualReady5p5m', 'PlatformStandoff']},
        acquire=dict(completed_at_s=0.), subsegment='PlatformStandoffHold',
        reference_m=[50., 0., 0.], reference_velocity_m_s=[0.]*3,
        reference_acceleration_m_s2=[0.]*3, offset_progress=0.)
    data = dict(truth=np.tile(states, (101, 1, 1)), truth_rotations=np.tile([Q, Q], (101, 1, 1, 1)),
        truth_angular_rates=np.tile([Q.T@omega, Q.T@omega], (101, 1, 1)),
        joints=np.zeros((101, 8)), joint_rates=np.zeros((101, 8)), elapsed_s=np.arange(101)*.05,
        target_hinge_angle_rad=np.zeros(101), target_hinge_rate_rad_s=np.zeros(101),
        covariance=np.tile(np.eye(6)*1e-6, (101, 1, 1)), position_sigma_m=np.full(101, .001),
        last_gnss_stamp_s=np.arange(101)*.05, near_guidance_json=[json.dumps(fake)]*101,
        vision_updates_json=['[]']*101, visual_ready=np.ones(101, dtype=bool),
        truth_optical_depth_m=np.full(101, depth), truth_tcp_gap_m=np.full(101, .5))
    data['center_distance_m'] = np.full(101, center_distance)
    result = audit_vbar(data, source, plan)
    assert not result['passed']
    assert not result['checks']['readiness_recomputed']
    assert all(not result['checks'][name+'_physical_joint_dwell'] for name in fake['nodes'])
