from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from simulation.scripted_tasks.plug_insert import PlugInsertTask, PlugSample
from simulation.scripted_tasks.plug_motion_config import PlugInsertConfig
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def task_sample():
    task = PlugInsertTask(MODEL)
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0]
    sample = PlugSample(
        0.0,
        q,
        np.zeros(8),
        np.array([0.3695, 0.33003, 0.032]),
        np.diag([1.0, -1.0, -1.0]),
        np.zeros(3),
        np.zeros(3),
        np.zeros(6),
        True,
    )
    task.step(sample)
    return task, sample


def test_assembly_target_is_forward_hole_and_not_pickup():
    task, _ = task_sample()
    np.testing.assert_allclose(task.pickup, [0.3695, 0.33003, 0.005])
    np.testing.assert_allclose(task.hole, [0.535, 0.355, 0.005])
    assert np.linalg.norm(task.hole - task.pickup) > 0.15
    with pytest.raises(ValueError):
        PlugInsertConfig(target_site="cylindrical_pickup")


def test_success_requires_actual_part_inserted_after_release():
    task, sample = task_sample()
    task._ever_lifted = True
    task._enter("verify", sample)
    for i in range(1, 65):
        task.step(replace(sample, time_s=i / 30))
    assert task.result is None
    released = sample.joints.copy()
    released[6:] = [task.config.release_open_rad, -task.config.release_open_rad]
    placed = replace(
        sample, joints=released, plug_position_body_m=np.array([0.535, 0.355, 0.030])
    )
    for i in range(65, 130):
        task.step(replace(placed, time_s=i / 30))
    assert task.result == "success"
    assert task.metrics(placed)["insertion_depth_m"] == pytest.approx(0.07)
    json.dumps(task.events)


def test_current_seated_completion_confirms_geometry_after_release():
    task, sample = task_sample()
    task._ever_lifted = True
    task._enter("verify", sample)
    released = sample.joints.copy()
    released[6:] = [task.config.release_open_rad, -task.config.release_open_rad]
    placed = replace(
        sample, joints=released, plug_position_body_m=np.array([.535, .355, .030]),
        plug_velocity_body_m_s=np.array([.003, 0., 0.]),
        plug_angular_velocity_body_rad_s=np.array([0., 0., .04]),
    )
    for tick in range(1, 26):
        task.step(replace(placed, time_s=tick / 100))
    assert task.result == "success"


def test_insertion_depth_without_alignment_is_not_success():
    task, sample = task_sample()
    task._enter("verify", sample)
    misaligned = replace(sample, plug_position_body_m=np.array([0.54, 0.355, 0.030]))
    for i in range(1, 90):
        task.step(replace(misaligned, time_s=i / 30))
    assert task.result is None


def test_overload_and_missing_sensor_are_explicit_failures():
    task, sample = task_sample()
    task.config = replace(task.config, overload_stop_enabled=True)
    for i in range(1, 7):
        task.step(
            replace(sample, time_s=i / 30, wrench=np.array([50.0, 0, 0, 0, 0, 0]))
        )
    assert task.reason == "wrist_load_limit"
    task, sample = task_sample()
    task.step(replace(sample, time_s=0.3, wrench_valid=False))
    assert task.reason == "wrist_sensor_unavailable"


def test_default_plug_task_records_load_without_stopping_on_threshold():
    task, sample = task_sample()
    assert not task.config.overload_stop_enabled
    for i in range(1, 7):
        task.step(replace(sample, time_s=i / 30,
                          wrench=np.array([50., 0., 0., 5., 0., 0.])))
    assert task.result is None
    assert task.max_force_n == 50.
    assert task.max_torque_nm == 5.
    assert task.metrics(sample)["overload_stop_enabled"] is False


@pytest.mark.parametrize("body,keyed", [
    ("cylindrical_plug", False), ("aviation_plug", True), ("guide_plug", True),
])
def test_insertion_alignment_uses_connector_axis_or_key(body, keyed):
    task = PlugInsertTask(MODEL, PlugInsertConfig(plug_body=body))
    metrics = {"radial_error_m": .0002, "orientation_error_rad": .018, "axis_error_rad": .005}
    assert task.config.insert_orientation_tolerance_rad == (.004 if keyed else .008)
    assert task._insertion_alignment_scale(metrics, .01) == (0. if keyed else 1.)
    metrics["axis_error_rad"] = .012
    assert task._insertion_alignment_scale(metrics, .01) == 0.


def test_mild_insert_misalignment_switches_to_load_limited_creep():
    task, _ = task_sample()
    metrics = {"radial_error_m": .0005, "orientation_error_rad": .005, "axis_error_rad": .005}
    scales = [task._insertion_alignment_scale(metrics, .01, 0., 0.)
              for _ in range(40)]
    assert scales[0] == 0.
    assert scales[-1] == pytest.approx(task.config.insert_creep_speed_scale)
    assert task._insert_alignment_creep_active
    assert task._insert_alignment_creep_count == 1


def test_insert_creep_still_holds_for_large_error_or_contact_load():
    task, _ = task_sample()
    large_error = {"radial_error_m": .0008, "orientation_error_rad": .008, "axis_error_rad": .008}
    for _ in range(40):
        assert task._insertion_alignment_scale(large_error, .01, 0., 0.) == 0.
    assert not task._insert_alignment_creep_active
    task, _ = task_sample()
    mild_error = {"radial_error_m": .0005, "orientation_error_rad": .005, "axis_error_rad": .005}
    for _ in range(40):
        scale = task._insertion_alignment_scale(
            mild_error, .01, task.config.force_limit_n, 0.
        )
    assert scale == 0.
    assert not task._insert_alignment_creep_active


def test_insert_creep_delay_restarts_after_alignment_recovers():
    task, _ = task_sample()
    mild_error = {"radial_error_m": .0005, "orientation_error_rad": .005, "axis_error_rad": .005}
    aligned = {"radial_error_m": .0002, "orientation_error_rad": .003, "axis_error_rad": .003}
    for _ in range(20):
        assert task._insertion_alignment_scale(mild_error, .01, 0., 0.) == 0.
    # A recovered alignment ends the current pause window.
    task._insertion_alignment_scale(aligned, .01, 0., 0.)
    for _ in range(10):
        assert task._insertion_alignment_scale(mild_error, .01, 0., 0.) == 0.
    for _ in range(16):
        scale = task._insertion_alignment_scale(mild_error, .01, 0., 0.)
    assert scale == pytest.approx(task.config.insert_creep_speed_scale)


def test_insert_alignment_duration_metrics_separate_hold_and_creep_across_recovery():
    task, sample = task_sample()
    mild_error = {"radial_error_m": .0005, "orientation_error_rad": .005, "axis_error_rad": .005}
    aligned = {"radial_error_m": .0002, "orientation_error_rad": .003, "axis_error_rad": .003}
    for _ in range(2):
        task._insertion_alignment_scale(mild_error, .125, 0., 0.)
    task._insertion_alignment_scale(aligned, .125, 0., 0.)
    for _ in range(2):
        task._insertion_alignment_scale(mild_error, .125, 0., 0.)
    recorded = task.metrics(sample)
    assert recorded["insertion_alignment_pause_s"] == pytest.approx(.5)
    assert recorded["insertion_alignment_hold_s"] == pytest.approx(.25)
    assert recorded["insertion_alignment_creep_s"] == pytest.approx(.25)
    assert recorded["insertion_alignment_creep_count"] == 2
    assert recorded["insertion_alignment_transitions"] == 3


@pytest.mark.parametrize("force_n,torque_nm", [
    (float("nan"), 0.), (float("inf"), 0.),
    (0., float("nan")), (0., float("inf")),
])
def test_insert_creep_holds_when_contact_load_is_not_finite(force_n, torque_nm):
    task, _ = task_sample()
    metrics = {"radial_error_m": .0005, "orientation_error_rad": .005, "axis_error_rad": .005}
    for _ in range(40):
        assert task._insertion_alignment_scale(metrics, .01, force_n, torque_nm) == 0.
    assert not task._insert_alignment_creep_active
    assert task._insert_alignment_creep_count == 0


def test_overload_switch_requires_boolean_without_weakening_numeric_validation():
    with pytest.raises(ValueError, match="overload_stop_enabled"):
        PlugInsertConfig(overload_stop_enabled="false")
    with pytest.raises(ValueError, match="arm_speed_rad_s"):
        PlugInsertConfig(arm_speed_rad_s=False)


def test_native_boolean_metrics_remain_json_serializable():
    task, sample = task_sample()
    task._ever_lifted = np.bool_(True)
    assert json.loads(json.dumps(task.metrics(sample)))["ever_lifted"] is True


def test_time_reset_requires_new_task():
    task, sample = task_sample()
    task.step(replace(sample, time_s=0.1))
    with pytest.raises(ValueError, match="monotonic"):
        task.step(sample)


def test_relative_part_state_cancels_common_translation_and_rigid_rotation():
    from types import SimpleNamespace
    from simulation.scripted_tasks.native_state import PlugState
    from simulation.serial_chain_kinematics import (
        matrix_to_quaternion_wxyz,
        axis_angle_to_matrix,
    )

    task, _ = task_sample()
    rb = axis_angle_to_matrix(np.array([0.0, 0.0, 1.0]), 0.7)
    rp = np.diag([1.0, -1.0, -1.0])
    position = np.array([0.535, 0.355, 0.03])
    origin = np.array([100.0, 200.0, 300.0])
    common = np.array([7000.0, -500.0, 200.0])
    omega = np.array([0.03, -0.02, 0.01])
    state = np.r_[
        origin,
        matrix_to_quaternion_wxyz(rb),
        origin + rb @ position,
        matrix_to_quaternion_wxyz(rb @ rp),
    ]
    rate = np.r_[common, omega, common + rb @ np.cross(omega, position), rp.T @ omega]
    reader = PlugState.__new__(PlugState)
    reader.task = task
    reader.stepper = SimpleNamespace(qpos=state, qvel=rate)
    reader.base = SimpleNamespace(getQposAdr=lambda: 0, getQvelAdr=lambda: 0)
    reader.plug = SimpleNamespace(getQposAdr=lambda: 7, getQvelAdr=lambda: 6)
    sample = reader.sample(
        0.0,
        np.zeros(8),
        np.zeros(8),
        dict(force_n=[0] * 3, torque_nm=[0] * 3, valid=True),
    )
    np.testing.assert_allclose(sample.plug_position_body_m, position, atol=1e-12)
    np.testing.assert_allclose(sample.plug_rotation_body, rp, atol=1e-12)
    np.testing.assert_allclose(sample.plug_velocity_body_m_s, 0.0, atol=1e-11)
    np.testing.assert_allclose(sample.plug_angular_velocity_body_rad_s, 0.0, atol=1e-12)


def test_seated_part_is_not_success_while_gripper_stays_closed():
    task, sample = task_sample()
    task._ever_lifted = True
    task._enter("verify", sample)
    seated = replace(sample, plug_position_body_m=np.array([0.535, 0.355, 0.03]))
    for tick in range(1, 91):
        task.step(replace(seated, time_s=tick / 30))
    assert task.result is None


def test_side_withdrawal_is_tangent_to_finger_closing_axis():
    task, sample = task_sample()
    task._enter("retract", sample)
    p, r = task.chain.forward(task.reference[:6])
    delta = task._goal - p
    assert delta[2] == pytest.approx(0.0)
    assert np.linalg.norm(delta) == pytest.approx(0.12)
    # The projection of tool-Y sets the chosen exit; no arbitrary world reset.
    expected = r[:, 1].copy()
    expected[2] = 0.0
    expected /= np.linalg.norm(expected)
    assert abs(np.dot(delta / 0.12, expected)) == pytest.approx(1.0)


def test_verify_rejects_unlifted_part_already_at_hole():
    task, sample = task_sample()
    released = sample.joints.copy()
    released[6:] = [0.65, -0.65]
    task._enter("verify", sample)
    sample = replace(
        sample, joints=released, plug_position_body_m=np.array([0.535, 0.355, 0.03])
    )
    for tick in range(1, 91):
        task.step(replace(sample, time_s=tick / 30))
    assert task.result is None
