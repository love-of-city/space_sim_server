"""Measured-state success witnesses must not depend on scripted controller phases."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from simulation.policy_task_success import (
    DrawerCycleSuccess, PolicyTaskSuccessProvider, RENDER_PERIOD_NS,
)
from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"
ROTATION = [[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]]


def physical(monitor, stamp=0, *, drawer=0., opening=.05, contacts=(3, 2), hover=False, **updates):
    position = (monitor.grip_origin + monitor.axis * drawer).tolist()
    if hover:
        position = monitor.hover.tolist()
    return dict(sim_time_ns=stamp, drawer_position_m=drawer, drawer_velocity_m_s=0., buckle_rad=0.,
                gripper_drive_position_rad=opening, finger_drawer_contacts=list(contacts),
                end_effector_position_body_m=position, end_effector_rotation_body=ROTATION,
                arm_joint_velocity_rad_s=[0.] * 6, **updates)


def initialized():
    monitor = DrawerCycleSuccess(MODEL)
    monitor.reset(physical(monitor))
    assert monitor.status()["success_available"]
    return monitor


def feed(monitor, count, **values):
    for _ in range(count):
        sample = physical(monitor, monitor.last_time_ns + RENDER_PERIOD_NS,
                          **{key: value for key, value in values.items() if key not in
                             ("drawer_velocity_m_s", "buckle_rad", "end_effector_position_body_m",
                              "end_effector_rotation_body", "arm_joint_velocity_rad_s")})
        sample.update({key: values[key] for key in ("drawer_velocity_m_s", "buckle_rad", "end_effector_position_body_m",
                                                  "end_effector_rotation_body", "arm_joint_velocity_rad_s") if key in values})
        result = monitor.update(sample)
    return result


def grasp_and_lift(monitor):
    assert feed(monitor, 51)["grasp_verified"]
    result = feed(monitor, 101, drawer=.08)
    assert result["open_verified"] and not result["success"]


def seat_and_release(monitor):
    assert feed(monitor, 101)["seated_before_release_verified"]
    assert feed(monitor, 1, opening=.96, contacts=(0, 0))["release_verified"]


def test_model_geometry_uses_delivered_calibration_and_config():
    monitor = initialized()
    config = DrawerCycleConfig()
    np.testing.assert_allclose(monitor.grip_origin, [.455, .105, .035])
    np.testing.assert_allclose(monitor.hover, monitor.grip_origin + [0., 0., config.clearance_m])
    assert monitor.status()["lift_threshold_m"] == config.lift_m - config.loaded_position_tolerance_m


def test_full_physical_cycle_requires_release_retreat_and_two_seconds_of_simulation():
    monitor = initialized()
    assert not feed(monitor, 101, opening=1., contacts=(0, 0), hover=True)["success"]
    grasp_and_lift(monitor)
    seat_and_release(monitor)
    assert not feed(monitor, 102, opening=1., contacts=(0, 0))["success"], "closed drawer at handle is not retreat"
    result = feed(monitor, 100, opening=1., contacts=(0, 0), hover=True)
    assert result["verification_stable_s"] == pytest.approx(1.98) and not result["success"]
    result = feed(monitor, 1, opening=1., contacts=(0, 0), hover=True)
    assert result["success_available"] and result["success"]
    assert result["verification_stable_s"] == pytest.approx(2.)
    assert not {"phase", "stage"}.intersection(result)


@pytest.mark.parametrize("bad", [
    {"contacts": (2, 0)}, {"contacts": (0, 2)}, {"opening": .025}, {"opening": 1.},
    {"arm_joint_velocity_rad_s": [.02] + [0.] * 5},
    {"end_effector_rotation_body": np.eye(3).tolist()},
    {"end_effector_position_body_m": [0., 0., 0.]},
])
def test_grasp_requires_bilateral_contact_and_actual_pose(bad):
    monitor = initialized()
    result = feed(monitor, 120, **bad)
    assert not result["grasp_verified"] and not result["success"]


def test_lift_requires_displacement_and_continued_physical_grasp():
    monitor = initialized()
    feed(monitor, 51)
    for bad in ({"drawer": .07}, {"drawer": .08, "contacts": (0, 0)},
                {"drawer": .08, "drawer_velocity_m_s": .004}):
        assert not feed(monitor, 101, **bad)["open_verified"]
    assert feed(monitor, 101, drawer=.08)["open_verified"]


def test_opening_before_seating_cannot_complete_cycle():
    monitor = initialized()
    grasp_and_lift(monitor)
    result = feed(monitor, 201, opening=1., contacts=(0, 0), hover=True)
    assert not result["seated_before_release_verified"]
    assert not result["release_verified"] and not result["success"]


@pytest.mark.parametrize("bad", [
    {"drawer": .003}, {"drawer_velocity_m_s": .003}, {"buckle_rad": .026},
    {"opening": .95}, {"end_effector_position_body_m": [0., 0., 0.]},
    {"arm_joint_velocity_rad_s": [.02] + [0.] * 5},
    {"end_effector_rotation_body": np.eye(3).tolist()},
])
def test_each_final_predicate_is_required_and_breaks_continuous_stability(bad):
    monitor = initialized()
    grasp_and_lift(monitor)
    seat_and_release(monitor)
    feed(monitor, 100, opening=1., contacts=(0, 0), hover=True)
    values = dict(opening=1., contacts=(0, 0), hover=True)
    values.update(bad)
    assert not feed(monitor, 1, **values)["success"]
    assert not feed(monitor, 100, opening=1., contacts=(0, 0), hover=True)["success"]
    assert feed(monitor, 1, opening=1., contacts=(0, 0), hover=True)["success"]


def test_buckle_may_be_pressed_while_seating_but_must_return_after_release():
    monitor = initialized()
    grasp_and_lift(monitor)
    assert feed(monitor, 101, buckle_rad=-.137)["seated_before_release_verified"]
    feed(monitor, 1, opening=1., contacts=(0, 0), buckle_rad=-.137)
    assert not feed(monitor, 101, opening=1., contacts=(0, 0), hover=True, buckle_rad=-.137)["success"]
    assert feed(monitor, 101, opening=1., contacts=(0, 0), hover=True)["success"]


def test_duplicate_samples_cannot_credit_wall_time_and_missing_frames_disable_success():
    monitor = initialized()
    grasp_and_lift(monitor)
    seat_and_release(monitor)
    feed(monitor, 50, opening=1., contacts=(0, 0), hover=True)
    same = physical(monitor, monitor.last_time_ns, opening=1., contacts=(0, 0), hover=True)
    for _ in range(200):
        result = monitor.update(same)
    assert result["verification_stable_s"] == pytest.approx(.98) and not result["success"]
    result = monitor.update({**same, "sim_time_ns": same["sim_time_ns"] + 1_000_000_000})
    assert not result["success_available"] and "consecutive" in result["success_unavailable_reason"]


@pytest.mark.parametrize("field,value", [("finger_drawer_contacts", None),
    ("drawer_position_m", float("nan")), ("end_effector_rotation_body", None),
    ("arm_joint_velocity_rad_s", [float("inf")] * 6)])
def test_missing_or_invalid_physical_evidence_never_claims_success(field, value):
    monitor = initialized()
    bad = physical(monitor, RENDER_PERIOD_NS)
    bad[field] = value
    result = monitor.update(bad)
    assert not result["success_available"] and not result["success"]


def test_reacquire_resets_witnesses_and_does_not_mutate_old_snapshots():
    monitor = initialized()
    grasp_and_lift(monitor)
    seat_and_release(monitor)
    result = feed(monitor, 101, opening=1., contacts=(0, 0), hover=True)
    assert result["success"]
    previous = deepcopy(result)
    monitor.reset(physical(monitor, monitor.last_time_ns, opening=1., contacts=(0, 0), hover=True))
    status = monitor.status()
    assert status["success_available"] and not status["success"]
    assert not status["grasp_verified"] and not status["open_verified"] and not status["seated_before_release_verified"]
    assert result == previous
    monitor.reset(None)
    assert not monitor.status()["success_available"]


def test_provider_has_no_success_for_generic_or_uncalibrated_geometry(tmp_path):
    provider = PolicyTaskSuccessProvider(MODEL)
    provider.reset("", None)
    assert provider.update({})["task_name"] == ""
    assert not provider.update({})["success_available"]
    provider.reset("unknown", None)
    assert not provider.update({})["success"]
    path = tmp_path / "unsupported.xml"
    path.write_text('<mujoco><worldbody><body name="cubesat_bus" /></worldbody></mujoco>')
    provider = PolicyTaskSuccessProvider(path)
    provider.reset("drawer_cycle", {})
    result = provider.update({})
    assert not result["success_available"] and "calibrated drawer" in result["success_unavailable_reason"]


def test_drawer_must_start_closed_for_monitor_to_be_available():
    monitor = DrawerCycleSuccess(MODEL)
    monitor.reset(physical(monitor, drawer=.02))
    assert not monitor.status()["success_available"]
    assert "initially closed" in monitor.status()["success_unavailable_reason"]


def test_runtime_provider_anchors_monitor_to_acquire_snapshot_and_generic_disables_it():
    from types import SimpleNamespace
    from simulation.policy_control import DrawerPolicyState

    class Message:
        def __init__(self, value):
            self.value = value
        def addSubscriber(self):
            return lambda: SimpleNamespace(state=self.value)
    drawer = SimpleNamespace(stateOutMsg=Message(.001), stateDotOutMsg=Message(0.))
    buckle = SimpleNamespace(stateOutMsg=Message(0.))
    bodies = {"drawer_base": drawer, "drawer_buckle": buckle}
    scene = SimpleNamespace(getBody=lambda name: SimpleNamespace(getScalarJoint=lambda _: bodies[name]))
    names = ["drawer_base", "drawer_buckle", "ee_finger1", "ee_finger2"]
    stepper = SimpleNamespace(nbody=4, _name=lambda _, i: names[i],
        contact_snapshot=lambda: [(0, 0, 0, 2, -.001), (0, 0, 1, 3, -.001)])
    provider = DrawerPolicyState(scene, stepper, model_path=MODEL)
    kwargs = dict(sim_time_ns=100_000_000, joint_velocities=[0.] * 8, ee_rotation=ROTATION)
    initial = provider.snapshot([0.] * 6 + [.05, -.05], [.455, .105, .036], **kwargs)
    preserved = deepcopy(initial)
    assert not initial["success_available"]
    provider.reset("drawer_cycle")
    drawer.stateOutMsg.value = .002
    kwargs["sim_time_ns"] += RENDER_PERIOD_NS
    following = provider.snapshot([0.] * 6 + [.05, -.05], [.455, .105, .037], **kwargs)
    assert following["success_available"] and not following["success"]
    assert following["initial_drawer_position_m"] == .001
    assert following["drawer_displacement_m"] == pytest.approx(.001)
    np.testing.assert_allclose(following["expected_hover_position_body_m"], [.455, .105, .241])
    assert initial == preserved, "acquire must not mutate the already emitted authoritative snapshot"
    provider.reset(None)
    kwargs["sim_time_ns"] += RENDER_PERIOD_NS
    generic = provider.snapshot([0.] * 8, [.455, .105, .037], **kwargs)
    assert generic["task_name"] == "" and not generic["success_available"] and not generic["success"]


@pytest.mark.parametrize("initial_yaw", [-1e-10, 0., 1e-10])
@pytest.mark.parametrize("swap_jaws", [False, True])
def test_jaw_sign_is_independent_of_initial_perpendicular_pose_drift(initial_yaw, swap_jaws):
    from simulation.serial_chain_kinematics import axis_angle_to_matrix

    monitor = DrawerCycleSuccess(MODEL)
    initial_rotation = axis_angle_to_matrix(np.array([0., 0., 1.]), initial_yaw) @ np.diag([1., -1., -1.])
    initial = physical(monitor)
    initial["end_effector_rotation_body"] = initial_rotation.tolist()
    monitor.reset(initial)
    grasp_rotation = np.asarray(ROTATION) @ np.diag([-1., -1., 1.]) if swap_jaws else np.asarray(ROTATION)
    orientation = {"end_effector_rotation_body": grasp_rotation.tolist()}
    assert feed(monitor, 51, **orientation)["grasp_verified"]
    assert feed(monitor, 101, drawer=.08, **orientation)["open_verified"]
    assert feed(monitor, 101, **orientation)["seated_before_release_verified"]
    assert feed(monitor, 1, opening=.96, contacts=(0, 0), **orientation)["release_verified"]
    assert feed(monitor, 101, opening=1., contacts=(0, 0), hover=True, **orientation)["success"]


def test_recorded_acquire_tie_and_opposite_measured_grasp_are_axis_equivalent():
    from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix

    # Physical quaternion samples at 0.24 s (acquire) and 81.24 s (grasp) from
    # episode-20261018-142932-433b6864. No recorded script/monitor flags are used.
    initial_rotation = quaternion_wxyz_to_matrix([
        1.8366403738854346e-06, -.9999999999915616, 2.9236832689859357e-10, 3.6747409711268515e-06])
    measured_rotation = quaternion_wxyz_to_matrix([
        .000623757723977796, -.7072689804950985, -.7069442487485982, -.0001712220962880423])
    monitor = DrawerCycleSuccess(MODEL)
    initial = physical(monitor)
    initial["end_effector_rotation_body"] = initial_rotation.tolist()
    monitor.reset(initial)
    assert -1e-9 < float(np.dot(monitor.rotation[:, 1], initial_rotation[:, 0])) < 0
    result = feed(monitor, 51, end_effector_rotation_body=measured_rotation.tolist())
    assert result["grasp_verified"]
    assert result["jaw_alignment_error_rad"] < .002


@pytest.mark.parametrize("axis,angle", [([0., 0., 1.], np.pi / 2),
    ([1., 0., 0.], np.pi / 2), ([1., 0., 0.], np.pi), ([0., 1., 0.], np.pi)])
def test_jaw_symmetry_still_rejects_sideways_upward_and_crosswise_tools(axis, angle):
    from simulation.serial_chain_kinematics import axis_angle_to_matrix

    wrong = np.asarray(ROTATION) @ axis_angle_to_matrix(np.asarray(axis), angle)
    monitor = initialized()
    result = feed(monitor, 101, end_effector_rotation_body=wrong.tolist())
    assert not result["grasp_verified"] and result["jaw_alignment_error_rad"] > 1.
    grasp_and_lift(monitor)
    seat_and_release(monitor)
    result = feed(monitor, 101, opening=1., contacts=(0, 0), hover=True,
                  end_effector_rotation_body=wrong.tolist())
    assert not result["success"] and result["verification_stable_s"] == 0.


@pytest.mark.parametrize("yaw,error_ok", [(.024, True), (.026, False),
                                         (np.pi + .024, True), (np.pi + .026, False)])
def test_both_jaw_branches_keep_the_original_orientation_tolerance(yaw, error_ok):
    from simulation.serial_chain_kinematics import axis_angle_to_matrix

    measured = np.asarray(ROTATION) @ axis_angle_to_matrix(np.array([0., 0., 1.]), yaw)
    monitor = initialized()
    result = feed(monitor, 51, end_effector_rotation_body=measured.tolist())
    assert result["grasp_verified"] is error_ok
