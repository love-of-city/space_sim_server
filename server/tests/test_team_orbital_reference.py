"""Independent kinematics and navigation tests for the team flight profile."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from simulation.orbital_rendezvous.model import (
    DEFAULT_MODEL,
    assembly_properties,
    prepare_model,
)
from simulation.orbital_rendezvous.navigation import OrbitFilter
from simulation.orbital_rendezvous.reference import FlightProfile, gravity

ROOT = Path(__file__).resolve().parents[1]


def test_initial_orbit_phase_matches_spacesim_default():
    from space_arm_platform.scene_runtime import _DEFAULT_ORBIT
    profile = FlightProfile()
    assert profile.initial_true_anomaly_deg == _DEFAULT_ORBIT['true_anomaly_deg']
    assert profile.target(0)[0][0] < 0
    assert np.linalg.norm(profile.chaser(0)[0] - profile.target(0)[0]) == pytest.approx(7000, abs=.01)


def test_flight_camera_and_lighting_use_platform_defaults():
    from simulation.orbital_rendezvous.rendering import flight_scene_settings
    from space_arm_platform.lighting import DEFAULT_SUNLIGHT_INTENSITY_SCALE
    settings = flight_scene_settings()
    assert settings.origin_object_id == settings.default_camera_target == 'team/cubesat_bus'
    assert settings.default_camera_distance_m == 2.8
    assert settings.sunlight_intensity_scale == DEFAULT_SUNLIGHT_INTENSITY_SCALE
    assert settings.headlight_enabled is False


@pytest.mark.parametrize("scenario", ["in-orbit", "orbit-raise"])
def test_reference_derivatives_and_finite_thrust(scenario):
    profile = FlightProfile(scenario=scenario)
    for t in np.linspace(1, profile.duration_s - 1, 31):
        r, v, a = profile.chaser(t)
        before = profile.chaser(t - 0.05)
        after = profile.chaser(t + 0.05)
        np.testing.assert_allclose((after[0] - before[0]) / 0.1, v, atol=2e-5)
        np.testing.assert_allclose((after[1] - before[1]) / 0.1, a, atol=1e-7)
        assert np.linalg.norm(a - gravity(r)) * 169.398546 < 25
    for t in (
        profile.start_approach_s,
        profile.start_approach_s + profile.approach_duration_s,
    ):
        if t:
            left, right = profile.chaser(t - 1e-6), profile.chaser(t + 1e-6)
            np.testing.assert_allclose(left[0], right[0], atol=0.02)
            np.testing.assert_allclose(left[1], right[1], atol=2e-5)
            np.testing.assert_allclose(left[2], right[2], atol=1e-6)
    t = profile.duration_s
    target, chaser = profile.target(t), profile.chaser(t)
    assert abs(np.linalg.norm(target[0] - chaser[0]) - profile.handoff_distance_m) < 1e-6
    nvec = np.cross(target[0], target[1]) / np.linalg.norm(target[0]) ** 2
    np.testing.assert_allclose(
        chaser[1] - target[1] - np.cross(nvec, chaser[0] - target[0]), 0, atol=1e-9
    )


def test_model_preserves_team_assets_and_mass(tmp_path):
    output, files, manifest = prepare_model(ROOT, tmp_path)
    source = ET.parse(ROOT / DEFAULT_MODEL).getroot()
    derived = ET.parse(output).getroot()
    assert manifest["bodies"]["cubesat_bus"]["mass_kg"] == pytest.approx(169.398546)
    assert manifest["bodies"]["capture_target"]["mass_kg"] == 10
    for tag in ("inertial", "geom"):
        old = [dict(item.attrib) for item in source.findall(f".//{tag}")]
        new = [dict(item.attrib) for item in derived.findall(f".//{tag}")]
        assert old == new
    assert len(files) == 113
    assert len(manifest["stowed_joints"]) == 9
    assert derived.find("keyframe") is None
    assert len(derived.findall('.//joint[@type="free"]')) == 2
    assert len(derived.findall('.//joint[@type="hinge"]')) == 3


def test_mass_properties_parallel_axis_independent_example():
    body = ET.fromstring(
        '<body><inertial mass="2" pos="0 0 0" diaginertia="1 1 1"/>'
        '<body pos="2 0 0"><inertial mass="2" pos="0 0 0" diaginertia="1 1 1"/></body></body>'
    )
    result = assembly_properties(body)
    np.testing.assert_allclose(result["center_body_m"], [1, 0, 0])
    np.testing.assert_allclose(result["inertia_com_body_kg_m2"], np.diag([2, 6, 6]))


def test_navigation_initialization_requires_observation_and_filters_noise():
    profile = FlightProfile()
    nav = OrbitFilter()
    assert nav.state is None
    rng = np.random.default_rng(93)
    raw, filtered = [], []
    for t in np.arange(0, 100, 0.5):
        r, v, _ = profile.target(t)
        truth = np.r_[r, v]
        observation = truth + rng.normal(size=6) * [0.2, 0.2, 0.2, 0.002, 0.002, 0.002]
        state = nav.update(observation, 0.5, np.zeros(3))
        raw.append(np.linalg.norm(observation[:3] - r))
        filtered.append(np.linalg.norm(state[:3] - r))
    assert np.mean(np.square(filtered[20:])) < 0.4 * np.mean(np.square(raw[20:]))
    assert np.linalg.eigvalsh(nav.covariance).min() > 0
    with pytest.raises(ValueError):
        nav.update([float("nan")] * 6, 0.5, np.zeros(3))
