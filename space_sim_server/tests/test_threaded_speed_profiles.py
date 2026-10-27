"""Collection speed profiles preserve the default and remain explicit."""

from pathlib import Path

import numpy as np
import pytest

from simulation.scripted_tasks.approach_path import ApproachPosePath
from simulation.scripted_tasks.motion_options import motion_arguments
from simulation.scripted_tasks.threaded_motion import threaded_speed_profile_options
from simulation.scripted_tasks.threaded_tasks import create_threaded_task


MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def test_fast_profile_is_explicit_and_keeps_default_unchanged():
    default = create_threaded_task("aviation_bolt_insert", MODEL).config
    fast = create_threaded_task(
        "aviation_bolt_insert", MODEL, speed_profile="fast_collection"
    ).config
    assert default.speed_profile == "default"
    assert default.linear_speed_m_s == pytest.approx(.05)
    assert default.rotation_speed_rad_s == pytest.approx(.20)
    assert default.rotation_acceleration_rad_s2 == pytest.approx(.40)
    assert default.approach_open_rad == pytest.approx(1.)
    assert default.release_open_rad == pytest.approx(.75)
    assert fast.speed_profile == "fast_collection"
    assert fast.linear_speed_m_s == pytest.approx(.08)
    assert fast.grasp_approach_speed_m_s == pytest.approx(.04)
    assert fast.rotation_speed_rad_s == pytest.approx(.30)
    assert fast.gripper_speed_rad_s == pytest.approx(.30)
    assert fast.rotation_acceleration_rad_s2 == pytest.approx(.60)
    assert fast.rotation_jerk_rad_s3 == pytest.approx(2.40)
    assert fast.approach_open_rad == pytest.approx(.45)
    assert fast.release_open_rad == pytest.approx(.45)


def test_profile_overrides_are_copy_safe_and_explicit_options_win():
    first = threaded_speed_profile_options("fast_collection")
    first["rotation_speed_rad_s"] = 99.
    second = threaded_speed_profile_options("fast_collection")
    assert second["rotation_speed_rad_s"] == pytest.approx(.30)
    task = create_threaded_task(
        "aviation_bolt_insert", MODEL,
        speed_profile="fast_collection", rotation_speed_rad_s=.25,
    )
    assert task.config.rotation_speed_rad_s == pytest.approx(.25)


def test_cli_profile_is_only_forwarded_for_threaded_tasks():
    assert motion_arguments("aviation_bolt_insert", None, "fast_collection") == [
        "--threaded-speed-profile", "fast_collection"
    ]
    assert motion_arguments("locking_rod_cycle", None, "default") == []
    with pytest.raises(ValueError):
        motion_arguments("drawer_cycle", None, "fast_collection")
    with pytest.raises(ValueError):
        motion_arguments("aviation_bolt_insert", None, "unknown")


@pytest.mark.parametrize("start", [0., .4, 1.])
def test_approach_reaches_clearance_opening_before_contact_without_full_open(start):
    path = ApproachPosePath(
        [0., .1, .2], np.eye(3), np.zeros(3), np.eye(3),
        [0., 0., 1.], start, .045, goal_opening=.45,
    )
    openings = np.array([path.opening(u) for u in np.linspace(0, 1, 301)])
    assert openings[0] == pytest.approx(start)
    assert path.opening(path.blend.finish) == pytest.approx(.45)
    assert path.opening(1.) == pytest.approx(.45)
    assert np.min(openings) >= min(start, .45)-1e-12
    assert np.max(openings) <= max(start, .45)+1e-12
    assert np.all(path.derivative_bounds()[2] >= 0.)
    # Jaw motion ends with the orientation blend while axial entry continues.
    assert path.pose(path.blend.finish)[0][2] == pytest.approx(.045)
    assert path.pose(1.)[0][2] == pytest.approx(0.)


@pytest.mark.parametrize("opening", [0., 1.01, float("nan")])
def test_collection_profile_rejects_invalid_gripper_openings(opening):
    with pytest.raises(ValueError):
        create_threaded_task("aviation_bolt_insert", MODEL, approach_open_rad=opening)
