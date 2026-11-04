from copy import deepcopy
import json
import random

from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.randomization import prepare_episode


def source():
    return dict(randomize_orbit_phase=True,
                randomization=dict(arm_joint_position_rad=[0.1] * 6 + [0.0, 0.0]),
                environment=dict(orbit=dict(altitude_m=500000, eccentricity=0.0,
                    inclination_deg=51.6, raan_deg=0.0, argument_of_periapsis_deg=0.0,
                    true_anomaly_deg=180.0)))


def test_each_episode_samples_phase_independently_without_mutating_source():
    original = source()
    before = deepcopy(original)
    first = randomized_scene(original, 10)
    second = randomized_scene(original, 11)
    replay = randomized_scene(original, 10)
    assert original == before
    assert first == replay
    a = first['environment']['orbit']['true_anomaly_deg']
    b = second['environment']['orbit']['true_anomaly_deg']
    assert 0 <= a < 360 and 0 <= b < 360 and a != b
    assert a == random.Random('space-arm-orbit-phase-v1:10').random() * 360
    expected = deepcopy(before['environment'])
    expected['orbit']['true_anomaly_deg'] = a
    assert first['environment'] == expected
    assert first['collection_config']['orbital_phase_randomization']['true_anomaly_deg'] == a


def test_orbital_toggle_does_not_change_arm_or_noise_random_streams():
    enabled = source()
    enabled['collection_randomization'] = dict(joint_bias=dict(enabled=True))
    disabled = deepcopy(enabled)
    disabled['randomize_orbit_phase'] = False
    a, b = [randomized_scene(scene, 42) for scene in (enabled, disabled)]
    assert a['randomization'] == b['randomization']
    assert a['collection_config']['realized_randomization'] == b['collection_config']['realized_randomization']
    assert b['environment'] == disabled['environment']
    # Arm initialization can be disabled without disabling orbital variation.
    enabled['initialization_enabled'] = False
    c = randomized_scene(enabled, 42)
    assert c['randomization']['arm_joint_position_rad'] == enabled['randomization']['arm_joint_position_rad']
    assert c['environment'] == a['environment']


def test_saved_episode_keeps_phase_when_recorder_prepares_it_again():
    sampled = randomized_scene(source(), 100)
    reloaded = json.loads(json.dumps(sampled))
    prepare_episode(reloaded)
    assert reloaded == sampled


def test_legacy_scene_does_not_gain_orbital_randomization():
    scene = source()
    del scene['randomize_orbit_phase']
    sample = randomized_scene(scene, 42)
    assert sample['environment'] == scene['environment']
    assert sample['collection_config']['orbital_phase_randomization'] == {'enabled': False}
