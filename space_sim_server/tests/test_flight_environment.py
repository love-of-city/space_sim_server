"""Physical direction, eclipse, reproducibility and force superposition checks."""
from dataclasses import replace
import numpy as np
import pytest

pytest.importorskip("Basilisk")
from Basilisk.architecture import messaging
from simulation.components.flight_environment import FlightEnvironment, solar_force_n, AU_M, SOLAR_PRESSURE_N_M2
from space_arm_platform.rendezvous_config import FlightOptions


def test_flight_entry_uses_native_substep_gravity_despite_teleop_stride(tmp_path, monkeypatch):
    """Exercise the real assembly boundary; a mocked stepper misses API wiring errors."""
    import json
    from pathlib import Path
    import subprocess
    import sys

    monkeypatch.setenv('SPACE_SIM_ORBITAL_FORCE_STRIDE', '8')
    folder = tmp_path / 'short-flight'
    result = subprocess.run(
        [sys.executable, '-m', 'simulation.orbital_rendezvous.run', '--scenario', 'in-orbit',
         '--seed', '19', '--duration', '1', '--output-dir', str(folder)],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=180)
    # A deliberately shortened flight must remain unaccepted for handoff.
    assert result.returncode == 2, result.stdout + result.stderr
    report = json.loads((folder / 'mission_report.json').read_text(encoding='utf-8'))
    gravity = report['local_dynamics']['orbital_environment']
    assert report['final_time_s'] == 1 and not report['basic_nominal_passed']
    assert gravity['force_stride_steps'] == 1 and gravity['evaluations'] == 240
    assert report['checks']['gravity_at_every_substep'] and report['checks']['finite_states']
    assert not json.loads((folder / 'handoff.json').read_text(encoding='utf-8'))['accepted']


def make_model(options, seed=13, illumination=1.0):
    state = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload(
        r_BN_N=[6.8e6, 0, 0], v_BN_N=[0, 7600, 0]))
    sun = messaging.SpicePlanetStateMsg().write(messaging.SpicePlanetStateMsgPayload(PositionVector=[AU_M, 0, 0]))
    density = messaging.AtmoPropsMsg().write(messaging.AtmoPropsMsgPayload(neutralDensity=2e-11))
    shadow = messaging.EclipseMsg().write(messaging.EclipseMsgPayload(shadowFactor=illumination))
    model = FlightEnvironment(state, sun, density, shadow, mass=160, area=1, options=options, seed=seed)
    model.keep_alive = state, sun, density, shadow
    return model


def test_srp_inverse_square_direction_and_eclipse():
    force = solar_force_n([0, 0, 0], [AU_M, 0, 0], 2, 1.5, 1)
    np.testing.assert_allclose(force, [-3 * SOLAR_PRESSURE_N_M2, 0, 0])
    np.testing.assert_allclose(solar_force_n([0, 0, 0], [2*AU_M, 0, 0], 2, 1.5, 1), force/4)
    assert not np.any(solar_force_n([0, 0, 0], [AU_M, 0, 0], 2, 1.5, 0))


def test_environment_switches_and_force_superposition():
    options = FlightOptions(orbit_acceleration_sigma_m_s2=0)
    both = make_model(options)
    drag = make_model(replace(options, srp_enabled=False))
    srp = make_model(replace(options, drag_enabled=False))
    eclipsed = make_model(replace(options, drag_enabled=False), illumination=0)
    for model in (both, drag, srp, eclipsed):
        model.UpdateState(0)
    total = np.array(both.forceOutMsg.read().forceRequestInertial)
    np.testing.assert_allclose(total, np.array(drag.forceOutMsg.read().forceRequestInertial) + srp.forceOutMsg.read().forceRequestInertial)
    assert drag.last["drag_n"][1] < 0
    assert srp.last["srp_n"][0] < 0
    assert not np.any(eclipsed.forceOutMsg.read().forceRequestInertial)


def test_disturbance_is_seeded_correlated_and_not_measurement_noise():
    options = FlightOptions(drag_enabled=False, srp_enabled=False)
    models = [make_model(options, seed) for seed in (13, 13, 17)]
    records = []
    for model in models:
        values = []
        for tick in range(1200):
            model.UpdateState(tick * 50000000)
            values.append(model.forceOutMsg.read().forceRequestInertial)
        records.append(np.asarray(values))
    np.testing.assert_array_equal(records[0], records[1])
    assert not np.allclose(records[0], records[2])
    assert np.corrcoef(records[0][:-1, 0], records[0][1:, 0])[0, 1] > .95
    assert np.max(np.abs(records[0])) > 0


def test_handoff_preserves_disturbance_and_its_next_sample():
    options = FlightOptions(drag_enabled=False, srp_enabled=False)
    source, restored = make_model(options), make_model(options, 99)
    source.UpdateState(0)
    source.UpdateState(50000000)
    restored.restore(source.snapshot())
    restored.UpdateState(0)
    np.testing.assert_array_equal(source.forceOutMsg.read().forceRequestInertial, restored.forceOutMsg.read().forceRequestInertial)
    source.UpdateState(100000000)
    restored.UpdateState(50000000)
    np.testing.assert_array_equal(source.forceOutMsg.read().forceRequestInertial, restored.forceOutMsg.read().forceRequestInertial)


def test_handoff_accepts_both_json_quaternion_signs():
    from types import SimpleNamespace
    from Basilisk.utilities import RigidBodyKinematics as rbk
    from simulation.orbital_rendezvous.handoff import apply_handoff

    orientations = []
    body = SimpleNamespace(setPosition=lambda value: None, setVelocity=lambda value: None,
                           setAttitudeRate=lambda value: None,
                           setAttitude=lambda value: orientations.append(rbk.MRP2C(value)))
    scene = SimpleNamespace(getBody=lambda name: body)
    for sign in (1, -1):
        quaternion = (sign * np.array([0.8, 0.6, 0., 0.])).tolist()
        data = {"stowed_joints": [], "reaction_wheels": [], "state": {"sim_time_s": 1., "bodies": [{
            "body_id": "rendezvous/chaser", "position_m": [7e6, 0, 0], "velocity_m_s": [0, 7500, 0],
            "angular_velocity_rad_s": [0, 0, 0], "attitude_wxyz": quaternion}]}}
        apply_handoff(scene, data)
        np.testing.assert_allclose(orientations[-1], rbk.EP2C(quaternion), atol=1e-15)
    np.testing.assert_allclose(orientations[0], orientations[1], atol=1e-15)
