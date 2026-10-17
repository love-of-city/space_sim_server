"""Policy failure gates and real-model kinematics; native acceptance is separate."""

from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from simulation.scripted_tasks.drawer_lift import (
    DrawerLiftConfig,
    DrawerLiftTask,
    DrawerSample,
    solve_pose,
)
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def setup_task():
    task = DrawerLiftTask(MODEL)
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    sample = DrawerSample(0.0, q, np.zeros(8), 0.0, 0.0, 0.0, np.zeros(6), False)
    task.step(sample)
    return task, sample


def test_invalid_limits_and_initial_drawer_are_rejected():
    with pytest.raises(ValueError):
        DrawerLiftConfig(arm_speed_rad_s=0.1)
    with pytest.raises(ValueError):
        DrawerLiftConfig(lift_m=float("nan"))
    task, sample = setup_task()
    task = DrawerLiftTask(MODEL)
    task.step(replace(sample, drawer_m=0.08))
    assert task.result == "failed"
    assert task.reason == "drawer_not_initially_closed"


def test_simulation_time_controls_progress_and_rejects_reset_without_new_task():
    task, sample = setup_task()
    before = task.step(sample)
    after = task.step(sample)
    np.testing.assert_array_equal(before[0], after[0])
    assert task.stage == "settle"
    with pytest.raises(ValueError, match="monotonic"):
        task.step(replace(sample, time_s=-0.1))


def test_overload_halts_commands_at_measured_configuration():
    task, sample = setup_task()
    for tick in range(1, 5):
        q, velocity = task.step(
            replace(
                sample,
                time_s=tick / 30,
                wrench_valid=True,
                wrench=np.array([100.0, 0, 0, 0, 0, 0]),
            )
        )
    assert task.result == "failed" and task.reason == "wrist_load_limit"
    np.testing.assert_allclose(q, sample.joints)
    np.testing.assert_array_equal(velocity, np.zeros(8))


def test_missing_sensor_fails_instead_of_treating_zero_as_safe():
    task, sample = setup_task()
    for tick in range(1, 9):
        task.step(replace(sample, time_s=tick / 30))
    assert task.result == "failed" and task.reason == "wrist_sensor_unavailable"


def test_hold_cannot_report_success_when_only_the_arm_moved():
    task, sample = setup_task()
    goal = task.grasp + task.axis * task.config.lift_m
    q = sample.joints.copy()
    # Find the continuous approach branch, then the lifted pose.
    for phase in ("raise", "above_drawer", "align", "descend", "lift"):
        task._enter(phase, replace(sample, joints=q))
        q[:6] = task._path[-1]
        task.reference[:] = q
    task._enter("hold", replace(sample, joints=q))
    task.step(replace(sample, time_s=1 / 30, joints=q, wrench_valid=True))
    assert task.result == "failed"
    assert task.reason == "drawer_slipped_or_not_lifted"
    np.testing.assert_allclose(task.chain.forward(q[:6])[0], goal, atol=2e-6)


def test_planned_path_respects_arm_speed_and_reaches_cartesian_target():
    task, sample = setup_task()
    task._enter("above_drawer", sample)
    q = sample.joints.copy()
    t = 0.0
    # Simulated perfect tracking only tests the command planner, not contact physics.
    while task.stage == "above_drawer":
        t += 1 / 30
        q, v = task.step(replace(sample, time_s=t, joints=q, wrench_valid=True))
        assert np.max(np.abs(v[:6])) <= task.config.arm_speed_rad_s + 1e-6
        assert np.all(q[:6] >= task.lower) and np.all(q[:6] <= task.upper)
        assert t < 60
    np.testing.assert_allclose(task.chain.forward(q[:6])[0], task.hover, atol=0.003)


def test_unreachable_pose_stops_planning():
    task, sample = setup_task()
    with pytest.raises(ValueError, match="No continuous IK"):
        solve_pose(
            task.chain,
            np.array([5.0, 5.0, 5.0]),
            np.eye(3),
            sample.joints[:6],
            task.lower,
            task.upper,
        )


def test_loaded_stage_transition_preserves_reference_and_speed_bound():
    task, sample = setup_task()
    for phase in ("raise", "above_drawer", "align", "descend"):
        task._enter(phase, sample)
        task.reference[:6] = task._path[-1]
    held = task.reference.copy()
    measured = held.copy()
    measured[0] += 0.01  # finite servo deflection under a grasp contact load
    sample = replace(sample, joints=measured, wrench_valid=True)
    task._enter("lift", sample)
    np.testing.assert_array_equal(task._path[0], held[:6])
    q, v = task.step(replace(sample, time_s=1 / 30))
    assert np.max(np.abs(v[:6])) <= task.config.arm_speed_rad_s
    assert np.max(np.abs(q[:6] - held[:6])) <= task.config.arm_speed_rad_s / 30
