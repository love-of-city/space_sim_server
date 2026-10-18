"""Numerical trajectory contracts; contact acceptance is tested with the real scene."""

import numpy as np
import pytest
from simulation.scripted_tasks.motion_timing import time_joint_path
from simulation.scripted_tasks.motion_options import motion_arguments
from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
from simulation.scripted_tasks.drawer_lift import DrawerLiftConfig


def timing(path=None, **overrides):
    values = dict(
        distance=0.2,
        angle=0.0,
        joint_speed=0.35,
        linear_speed=0.1,
        angular_speed=0.2,
        linear_acceleration=0.2,
        angular_acceleration=0.4,
        joint_acceleration=0.7,
    )
    values.update(overrides)
    return time_joint_path(
        np.linspace([0.0], [0.3], 81) if path is None else path, **values
    )


def samples(profile):
    t = np.linspace(0, profile.duration, 10001)
    u = np.array([profile.at(x) for x in t])
    return u, np.diff(u) / np.diff(t)


def test_cruise_reaches_requested_speed_and_ramps_at_both_ends():
    profile = timing()
    u, speed = samples(profile)
    assert u[0] == 0 and u[-1] == 1
    assert np.all(np.diff(u) >= 0)
    assert np.max(speed * 0.2) == pytest.approx(0.1, abs=1e-8)
    assert speed[0] * 0.2 < 0.001 and speed[-1] * 0.2 < 0.001
    assert profile.at(-1) == 0 and profile.at(profile.duration + 1) == 1


def test_terminal_zone_is_slow_even_for_interval_crossing_boundary():
    profile = timing(slow_distance=0.02, slow_speed=0.012)
    u, speed = samples(profile)
    assert np.max(speed[u[:-1] >= 0.9] * 0.2) <= 0.012 + 1e-9
    assert np.max(speed * 0.2) == pytest.approx(0.1, abs=1e-8)


def test_joint_limit_can_prevent_cartesian_cruise_and_rotation_has_own_units():
    profile = timing(np.linspace([0.0], [2.0], 81))
    _, speed = samples(profile)
    assert np.max(speed * 2) <= 0.35 + 1e-9
    profile = timing(np.linspace([0.0], [np.pi], 81), distance=0.0, angle=np.pi)
    _, speed = samples(profile)
    assert np.max(speed * np.pi) == pytest.approx(0.2, abs=1e-8)


def test_short_and_stationary_paths_remain_finite():
    for path in (np.array([[0.0], [0.001]]), np.zeros((81, 6))):
        profile = timing(path, distance=0.0)
        u, speed = samples(profile)
        assert profile.duration >= 1 - 1e-12 and np.isfinite(speed).all()
        assert u[-1] == 1


def test_drawer_override_does_not_relax_legacy_task_or_other_collectors():
    assert DrawerCycleConfig().arm_speed_rad_s == 0.35
    with pytest.raises(ValueError):
        DrawerLiftConfig(arm_speed_rad_s=0.35)
    with pytest.raises(ValueError):
        DrawerCycleConfig(arm_speed_rad_s=0.4)
    for invalid in (0.0, -0.1, 0.2, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            motion_arguments("drawer_cycle", invalid)
    with pytest.raises(ValueError):
        motion_arguments("aviation_plug_insert", 0.1)
    assert motion_arguments("drawer_cycle", 0.05) == ["--linear-speed", "0.05"]
    assert motion_arguments("aviation_plug_insert", None) == []


def test_acceleration_is_continuous_at_cruise_and_slow_zone_transitions():
    profile = timing(slow_distance=0.02, slow_speed=0.012)
    # Old piecewise-constant acceleration had jumps here. Check both sides,
    # including entering and leaving the held endpoint references.
    for stamp in profile.times:
        left = profile.joint_state(stamp - 1e-8)
        right = profile.joint_state(stamp + 1e-8)
        np.testing.assert_allclose(left[1], right[1], atol=1e-7)
        np.testing.assert_allclose(left[2], right[2], atol=1e-6)
    for stamp in (0, profile.duration):
        for derivative in profile.joint_state(stamp)[1:]:
            np.testing.assert_allclose(derivative, 0, atol=1e-12)


def test_curved_joint_path_respects_joint_acceleration_and_jerk_too():
    u = np.linspace(0, 1, 21)
    path = np.column_stack((0.8 * np.sin(2 * u), u**3 - u))
    profile = timing(path, slow_distance=0.02, slow_speed=0.012, joint_jerk=2.0)
    states = [profile.joint_state(t) for t in np.linspace(0, profile.duration, 8001)]
    for derivative, bound in ((1, 0.35), (2, 0.7), (3, 2.0)):
        assert np.max(np.abs([row[derivative] for row in states])) <= bound + 1e-9
    # Limits include q''(u) and q'''(u), which scalar-progress limits miss.
    assert profile.duration > 0


def test_spline_preserves_geometry_and_checks_between_waypoint_position_limits():
    from simulation.scripted_tasks.joint_path import JointPath

    u = np.linspace(0, 1, 9)
    curve = JointPath.from_waypoints(np.column_stack((u**3 - u, 2 * u**2)))
    for x in np.linspace(0, 1, 101):
        state = curve.state(x)
        for actual, expected in zip(
            state, ([x**3 - x, 2 * x**2], [3 * x**2 - 1, 4 * x], [6 * x, 4], [6, 0])
        ):
            np.testing.assert_allclose(actual, expected, atol=1e-10)
    for knot in u[1:-1]:
        for order in (0, 1, 2):
            np.testing.assert_allclose(
                curve.state(knot - 1e-9)[order],
                curve.state(knot + 1e-9)[order],
                atol=1e-7,
            )
    overshoot = JointPath.from_waypoints(np.array([[0], [1], [1], [0]]))
    assert overshoot.position_bounds()[1][0] == pytest.approx(1.125)
    with pytest.raises(ValueError, match="position limit"):
        overshoot.validate_limits([0], [1])


def test_short_terminal_zone_reduces_speed_early_and_all_slow_path_is_supported():
    for slow_distance in (1e-5, 0.2, 0.5):
        profile = timing(slow_distance=slow_distance, slow_speed=0.012)
        for stamp in np.linspace(0, profile.duration, 10001):
            u, speed, _, _ = profile.state(stamp)
            if (1 - u) * 0.2 <= slow_distance:
                assert speed * 0.2 <= 0.012 + 1e-9


@pytest.mark.parametrize("field", ["linear_jerk", "angular_jerk", "joint_jerk"])
def test_invalid_jerk_limits_are_rejected(field):
    for bad in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            timing(**{field: bad})


def test_joint_only_terminal_zone_limits_contact_approach_speed():
    profile = timing(
        np.array([[1.0], [0.0]]),
        distance=0,
        joint_speed=0.2,
        slow_fraction=0.2,
        slow_joint_speed=0.04,
    )
    max_speed = 0.0
    for stamp in np.linspace(0, profile.duration, 5001):
        q, v, _, _ = profile.joint_state(stamp)
        max_speed = max(max_speed, abs(v[0]))
        if q[0] <= 0.2:
            assert abs(v[0]) <= 0.04 + 1e-9
    assert max_speed == pytest.approx(0.2, abs=1e-9)
    for invalid in (
        dict(slow_fraction=-1),
        dict(slow_fraction=2),
        dict(slow_fraction=0.2),
        dict(slow_fraction=0.2, slow_joint_speed=0.04, slow_distance=0.01),
    ):
        with pytest.raises(ValueError):
            timing(**invalid)
