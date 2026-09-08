"""Native MJScene and drive-level regressions (never import Python MuJoCo)."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk

from simulation.attitude_control import _inertia, limit_wheel_torque, load_hardware, load_settings

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'model/SARM/platform/sarm_platform.xml'


@pytest.fixture
def native(monkeypatch):
    # The native scenario's optional offline renderer imports a different MuJoCo
    # DLL. Defer only that optional API; all dynamics below use real Basilisk.
    if 'mujoco' in sys.modules and hasattr(sys.modules['mujoco'], 'MjModel'):
        pytest.fail('Do not load Python MuJoCo and Basilisk MuJoCo in one process')
    monkeypatch.setitem(sys.modules, 'mujoco', ModuleType('mujoco'))
    spec = importlib.util.spec_from_file_location('sarm_rw_native_test', MODEL.parent / 'scenarios/scenario_sarm_grasp.py')
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def advance(native, sim, seconds):
    sim.ConfigureStopTime(native.macros.sec2nano(seconds))
    sim.ExecuteSimulation()


def build(native, *, enabled=True, move_arm=False):
    q0 = native.PREGRASP.copy()
    q1 = q0.copy()
    q1[1] += 0.3  # [rad]
    native.JointTrajectoryPublisher.reference = classmethod(
        lambda cls, t: cls._segment(t, 1, 4, q0, q1) if move_arm else (q0.copy(), np.zeros(8)))
    sim, scene, models, logs = native._build_simulation(attitude_control_enabled=enabled)
    native._initialize_state(sim, scene)
    scene.getBody('capture_target').setPosition([5, 0, 0])  # [m], avoid contact
    return sim, scene, models, logs


def momenta(control):
    """Evaluate spacecraft P and angular momentum about its instantaneous COM."""
    parts = []
    for name, mass, com, inertia in control.parts:
        state = control.scene.getBody(name).getOrigin().stateOutMsg.read()
        rotation = rbk.MRP2C(state.sigma_BN).T
        omega = rotation @ np.asarray(state.omega_BN_B)
        offset = rotation @ com
        position = np.asarray(state.r_BN_N) + offset
        velocity = np.asarray(state.v_BN_N) + np.cross(omega, offset)
        parts.append((mass, position, velocity, rotation @ inertia @ rotation.T @ omega))
    total_mass = sum(p[0] for p in parts)
    center = sum(m * r for m, r, _, _ in parts) / total_mass
    momentum = sum(m * v for m, _, v, _ in parts)
    angular = sum(h + np.cross(r - center, m * v) for m, r, v, h in parts)
    return momentum, angular


def test_rotor_allocation_preserves_original_bus_mass_com_and_inertia():
    root = ET.parse(MODEL).getroot()
    bus = root.find('./worldbody/body[@name="cubesat_bus"]')
    bodies = [(bus, np.zeros(3))] + [(bus.find(f'./body[@name="rw_{a}"]'), None) for a in 'xyz']
    parts = []
    for body, position in bodies:
        origin = np.fromstring(body.get('pos', '0 0 0'), sep=' ') if position is None else position
        inertial = body.find('inertial')
        parts.append((float(inertial.get('mass')), origin + np.fromstring(inertial.get('pos'), sep=' '), _inertia(inertial)))
    mass = sum(m for m, _, _ in parts)
    com = sum(m * r for m, r, _ in parts) / mass
    tensor = sum(I + m * (np.dot(r-com, r-com)*np.eye(3)-np.outer(r-com, r-com)) for m,r,I in parts)
    assert mass == pytest.approx(162.76)
    np.testing.assert_allclose(com, [.18986, .27487, -.31454], atol=1e-12)
    np.testing.assert_allclose(tensor, [[4.8392,.095847,.01857],[.095847,5.532,-.006219],[.01857,-.006219,8.6291]], atol=1e-12)


@pytest.mark.parametrize('sign', [-1, 1])
def test_torque_speed_limits_braking_disabled_and_nonfinite(sign):
    wheel = load_hardware(MODEL)[0][0]
    assert limit_wheel_torque(sign, 0, wheel, True, .98) == (sign*.2, True, False)
    assert limit_wheel_torque(sign*.1, sign*wheel.max_speed, wheel, True, .98) == (0., False, True)
    assert limit_wheel_torque(-sign*.1, sign*wheel.max_speed, wheel, True, .98) == (-sign*.1, False, False)
    assert limit_wheel_torque(sign, sign*wheel.max_speed, wheel, False, .98) == (0., False, False)
    with pytest.raises(FloatingPointError):
        limit_wheel_torque(float('nan'), 0, wheel, True, .98)


def test_invalid_control_rate_rejected(tmp_path):
    import json
    config = load_settings(MODEL.with_name('attitude_control.json'))
    config['control_rate_hz'] = 101.0
    path = tmp_path / 'invalid.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='divide'):
        load_settings(path)


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_native_internal_torque_reaction_sign_and_momentum(native, axis):
    sim, scene, models, logs = build(native, enabled=False)
    advance(native, sim, .02)
    before = momenta(sim.attitude_control)
    wheel = sim.attitude_control.wheels[axis]
    payload = messaging.SingleActuatorMsgPayload(input=.05)  # [N*m]
    command = messaging.SingleActuatorMsg().write(payload)
    scene.getSingleActuator(wheel.motor).actuatorInMsg.subscribeTo(command)
    advance(native, sim, .22)
    after = momenta(sim.attitude_control)
    assert sim.attitude_control.joints[axis].stateDotOutMsg.read().state > .8  # [rad/s]
    assert scene.getBody('cubesat_bus').getOrigin().stateOutMsg.read().omega_BN_B[axis] < 0
    np.testing.assert_allclose(after[0], before[0], atol=2e-7)
    np.testing.assert_allclose(after[1], before[1], atol=2e-7)
    assert np.all(np.isfinite(np.asarray(logs[0].qpos)))


def test_initial_nonidentity_reference_is_latched_not_zeroed(native):
    sim, scene, models, logs = build(native)
    initial = [.05, -.025, .01]  # [MRP]
    scene.getBody('cubesat_bus').setAttitude(initial)
    advance(native, sim, .1)
    np.testing.assert_allclose(sim.attitude_control.reference_mrp, initial, atol=1e-9)
    assert sim.attitude_control.telemetry()['attitude_control']['attitude_error_angle_rad'] < 1e-8
    scene.getBody('cubesat_bus').setAttitude([.025, -.0125, .02])
    advance(native, sim, .2)
    np.testing.assert_allclose(sim.attitude_control.reference_mrp, initial, atol=1e-9)


def test_native_arm_disturbance_rejection_enabled_vs_disabled(native):
    results = []
    for enabled in (False, True):
        sim, scene, models, logs = build(native, enabled=enabled, move_arm=True)
        advance(native, sim, 20)
        results.append(sim.attitude_control.telemetry())
        assert np.all(np.isfinite(np.asarray(logs[0].qpos)))
    off, on = results
    assert off['attitude_control']['attitude_error_angle_rad'] > .005
    assert on['attitude_control']['attitude_error_angle_rad'] < .1 * off['attitude_control']['attitude_error_angle_rad']
    assert on['attitude_control']['attitude_error_angle_rad'] < np.deg2rad(.02)
    assert off['reaction_wheels']['applied_motor_torque_nm'] == [0, 0, 0]


def test_native_attitude_impulse_recovers_and_reports_torque_saturation(native):
    sim, scene, models, logs = build(native)
    advance(native, sim, .1)
    scene.getBody('cubesat_bus').setAttitude([.05, -.03, .04])
    advance(native, sim, .2)
    first = sim.attitude_control.telemetry()
    assert first['attitude_control']['attitude_error_angle_rad'] > .1
    assert first['attitude_control']['saturated']
    assert max(abs(v) for v in first['reaction_wheels']['applied_motor_torque_nm']) <= .2
    advance(native, sim, 30)
    last = sim.attitude_control.telemetry()
    assert last['attitude_control']['attitude_error_angle_rad'] < .003
    assert np.linalg.norm(last['attitude_control']['angular_velocity_body_rad_s']) < .001


def test_native_speed_guard_allows_braking_and_disabled_drive(native):
    sim, scene, models, logs = build(native)
    advance(native, sim, .02)
    control = sim.attitude_control
    joint, drive, wheel = control.joints[0], control.drives[0], control.wheels[0]
    initial_speed = .99 * wheel.max_speed  # [rad/s]
    joint.setVelocity(initial_speed)
    command = messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=.1))
    drive.torque_message = command
    advance(native, sim, .04)
    assert drive.speed_limited
    assert drive.applied == 0.0
    assert joint.stateDotOutMsg.read().state == pytest.approx(initial_speed, abs=.01)
    command.write(messaging.SingleActuatorMsgPayload(input=-.1))
    advance(native, sim, .14)
    assert not drive.speed_limited
    assert drive.applied == -.1
    assert joint.stateDotOutMsg.read().state < initial_speed - .5
    control.enabled = False
    advance(native, sim, .16)
    assert all(d.applied == 0 for d in control.drives)


def test_native_rate_disturbance_is_damped_without_losing_reference(native):
    sim, scene, models, logs = build(native)
    advance(native, sim, .1)
    scene.getBody('cubesat_bus').setAttitudeRate([.015, -.01, .02])  # [rad/s]
    advance(native, sim, 25)
    result = sim.attitude_control.telemetry()
    assert np.linalg.norm(result['attitude_control']['angular_velocity_body_rad_s']) < .0002
    assert result['attitude_control']['attitude_error_angle_rad'] < .002
    np.testing.assert_array_equal(sim.attitude_control.reference_mrp, np.zeros(3))
    assert max(abs(v) for v in result['reaction_wheels']['relative_momentum_nms']) > .05
