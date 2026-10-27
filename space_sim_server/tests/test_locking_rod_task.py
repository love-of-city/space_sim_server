"""Geometry and evidence regressions for the captive-rod collection controller."""

from dataclasses import replace
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.locking_rod import LockingRodConfig, LockingRodTask
from simulation.scripted_tasks.threaded_motion import ThreadedSample, model_body_pose
from simulation.serial_chain_kinematics import axis_angle_to_matrix
from tools.threaded_task_initialization import apply_locking_cycle_initial_state
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "model/SARM/platform/sarm_20261012.xml"


@pytest.fixture
def model(tmp_path):
    root = ET.parse(SOURCE).getroot()
    apply_locking_cycle_initial_state(root)
    path = tmp_path / "locking.xml"
    ET.ElementTree(root).write(path)
    return path


def sample_for(task, *, hinge=0., slide=None, spin=None, hook_force=5.,
               hook_contacts=1., clip_contacts=0., time=0., fingers=(0, 0)):
    slide = task.slide_ref if slide is None else slide
    spin = task.spin_ref if spin is None else spin
    p, r = task.mechanism_pose(hinge, slide, spin)
    return ThreadedSample(
        time, np.r_[np.deg2rad([0, -67.6, -86.6, 143.2, -85.5, 0]), .5, -.5],
        np.zeros(8), p, r, np.zeros(3), np.zeros(3), np.zeros(6), True, fingers,
        {"locking_rod_hinge": hinge, "fastener_slide": slide, "fastener_spin": spin,
         "locking_rod_hinge_velocity": 0., "fastener_slide_velocity": 0., "fastener_spin_velocity": 0.,
         "fastener_hook_contacts": hook_contacts, "fastener_hook_normal_force_n": hook_force,
         "locking_rod_clip_contacts": clip_contacts, "fastener_clip_contacts": 0.},
    )


def test_short_original_hinge_cannot_claim_to_reach_clip():
    with pytest.raises(ValueError, match="cannot reach"):
        LockingRodTask(SOURCE)


def test_locked_refs_preserve_exact_reference_pose_and_inverse_loose_pose(model):
    task = LockingRodTask(model)
    p, r = task.mechanism_pose(task.hinge_ref, task.slide_ref, task.spin_ref)
    expected_p, expected_r = model_body_pose(ET.parse(model).getroot(), "fastener")
    assert np.allclose(p, expected_p, atol=1e-12)
    assert np.allclose(r, expected_r, atol=1e-12)
    old_p, old_r = model_body_pose(ET.parse(SOURCE).getroot(), "fastener")
    loose_p, loose_r = task.mechanism_pose(0., 0., 0.)
    assert np.allclose(loose_p, old_p, atol=1e-12)
    assert np.allclose(loose_r, old_r, atol=1e-12)
    assert math.isclose(task.lead_m, .016)


def test_swing_uses_bottom_pivot_and_captive_axis_at_random_installation_yaw(model, tmp_path):
    root = ET.parse(model).getroot()
    base = root.find('.//body[@name="task_box_base"]')
    yaw = .91
    base.set("quat", f"{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}")
    base.set("pos", ".19 .08 0")
    path = tmp_path / "yaw.xml"
    ET.ElementTree(root).write(path)
    task = LockingRodTask(path)
    loose, rotation = task.mechanism_pose(0., 0., 0.)
    for hinge in np.linspace(task.config.folded_hinge_rad, 0., 7):
        p, r = task.mechanism_pose(hinge, 0., 0.)
        swing = axis_angle_to_matrix(task.hinge_axis, hinge)
        assert np.allclose(p, task.pivot+swing@(loose-task.pivot), atol=1e-12)
        assert np.allclose(r, swing@rotation, atol=1e-12)
        assert math.isclose(np.linalg.norm(p-task.pivot), np.linalg.norm(loose-task.pivot), abs_tol=1e-12)


def test_turn_grasp_uses_original_tab_axis_and_coaxial_wrist(model):
    task = LockingRodTask(model)
    sample = sample_for(task, hinge=-1.53, slide=0., spin=0.)
    position, rotation = task._grasp_pose(sample)
    closing = sample.part_rotation_body.T @ rotation[:, 0]
    assert np.isclose(abs(closing @ np.array([0., 1., 0.])), 1.)
    assert np.isclose(closing[2], 0.)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(rotation), 1.)
    approach = sample.part_rotation_body.T @ rotation[:, 2]
    assert np.isclose(approach[2], 1.)
    assert np.allclose(position, sample.part_position_body_m
                       + sample.part_rotation_body @ np.array(task.config.grasp_offset_m))


@pytest.mark.parametrize("hinge,slide,spin", [(0., 0., 0.), (-.2, .00479, 1.88)])
def test_initial_locked_variant_still_requires_actual_closed_coordinates(model, hinge, slide, spin):
    task = LockingRodTask(model)
    task.step(sample_for(task, hinge=hinge, slide=slide, spin=spin))
    task.step(sample_for(task, hinge=hinge, slide=slide, spin=spin, time=.3))
    assert task.result == "failed"
    assert task.reason == "initial_fastener_not_physically_tightened"


def test_unloaded_initial_latch_does_not_require_a_fake_permanent_preload(model):
    task = LockingRodTask(model)
    sample = sample_for(task, hook_force=0., hook_contacts=0.)
    assert task._initially_locked(sample)
    assert not task._at_hook(sample)


def test_missing_force_is_not_fabricated_from_contact_count(model):
    task = LockingRodTask(model)
    sample = sample_for(task)
    sample.scalar_joints.pop("fastener_hook_normal_force_n")
    assert not task._at_hook(sample)
    assert task.metrics(sample)["fastener_hook_normal_force_n"] is None


@pytest.mark.parametrize("field", ["rotation", "scalar"])
def test_nonfinite_native_rotation_or_scalar_cannot_drive_the_arm(model, field):
    task = LockingRodTask(model)
    sample = sample_for(task)
    if field == "rotation":
        sample = replace(sample, part_rotation_body=np.full((3, 3), np.nan))
    else:
        sample.scalar_joints["fastener_slide"] = math.nan
    reference, velocity = task.step(sample)
    assert task.result == "failed" and task.reason == "nonfinite_native_sample"
    assert np.isfinite(reference).all() and np.array_equal(velocity, np.zeros(8))


def test_clip_evidence_needs_actual_contact_and_folded_angle(model):
    task = LockingRodTask(model)
    assert not task._at_clip(sample_for(task, clip_contacts=3.))
    assert not task._at_clip(sample_for(task, hinge=task.config.folded_hinge_rad))
    assert task._at_clip(sample_for(task, hinge=task.config.folded_hinge_rad, clip_contacts=2.))


def test_clip_contact_cannot_cut_short_a_moving_lowering_reference(model, monkeypatch):
    task = LockingRodTask(model)
    task.stage, task._entered, task._last_time = "lower", 0., 0.
    task._stable = task.config.terminal_confirm_s
    sample = sample_for(task, hinge=task.config.folded_hinge_rad, slide=0., spin=0.,
                        clip_contacts=2., time=.005)
    task.reference[:] = sample.joints
    monkeypatch.setattr(task, "_path_step", lambda *args: False)
    task.step(sample)
    assert task.stage == "lower" and not task.folded_verified


def test_commanded_rotation_cannot_substitute_for_measured_loosen(model, monkeypatch):
    task = LockingRodTask(model)
    initial = sample_for(task, fingers=(2, 2))
    task._initial_slide, task._initial_spin = task.slide_ref, task.spin_ref
    task.reference[:] = initial.joints
    task._enter("loosen", initial)
    task._spin_command = -10.
    monkeypatch.setattr(task, "_track_part", lambda *args, **kwargs: None)
    monkeypatch.setattr(task, "_plan_swing", lambda *args, **kwargs: None)
    task.step(replace(initial, time_s=.01))
    assert task.stage == "loosen"
    assert not task.loosen_verified
    actual_loose = sample_for(task, slide=0., spin=0., hook_force=0., hook_contacts=0., time=.02, fingers=(2, 2))
    task.step(actual_loose)
    assert task.stage == "fold_release"
    assert task.loosen_verified


def test_final_acceptance_requires_full_cycle_and_released_stability(model):
    task = LockingRodTask(model)
    task.stage, task._entered = "verify", 0.
    for time in np.arange(0., 1.5, .05):
        task.step(sample_for(task, time=float(time)))
    assert task.result is None
    task.loosen_verified = task.folded_verified = task.returned_verified = task.tightened_verified = True
    task._tighten_seated_coordinates = (task.slide_ref, task.spin_ref)
    task._tighten_peak_hook_force_n = 5.
    for time in np.arange(1.5, 2.7, .05):
        task.step(sample_for(task, time=float(time)))
    assert task.result == "success"
    assert task.metrics(sample_for(task))["released_stable_seconds"] >= task.config.verify_hold_s


def test_unloaded_release_needs_real_tightening_history_and_retains_thread_seat(model):
    task = LockingRodTask(model)
    unloaded = sample_for(task, hook_force=0., hook_contacts=0.)
    task.tightened_verified = True
    assert not task._released_locked(unloaded)
    task._tighten_seated_coordinates = (task.slide_ref, task.spin_ref)
    assert not task._released_locked(unloaded)  # Coordinate history cannot invent force.
    task._tighten_peak_hook_force_n = 5.
    assert task._released_locked(unloaded)
    task.stage, task._entered = "verify", 0.
    task.loosen_verified = task.folded_verified = task.returned_verified = True
    for time in np.arange(0., 1.1, .05):
        task.step(replace(unloaded, time_s=float(time)))
    assert task.result == "success"


def test_release_rejects_too_much_backoff_even_inside_absolute_closed_interval(model):
    task = LockingRodTask(model)
    task.tightened_verified = True
    task._tighten_peak_hook_force_n = 5.
    task._tighten_seated_coordinates = (.0051, .0051*2*math.pi/task.lead_m)
    slide = .0046
    backed_off = sample_for(task, slide=slide, spin=slide*2*math.pi/task.lead_m,
                            hook_force=0., hook_contacts=0.)
    assert task._closed_coordinates(backed_off)
    assert not task._released_locked(backed_off)


def test_rotation_holds_the_hook_alignment_instead_of_following_a_deflected_rod(model, monkeypatch):
    task = LockingRodTask(model)
    sample = sample_for(task, hinge=.05)
    task._spin_command = sample.scalar_joints["fastener_spin"]
    captured = []
    monkeypatch.setattr(task, "_track_part", lambda p, r, *args: captured.append((p, r)))
    task._rotate_fastener(sample, .005, task.spin_ref)
    expected = task.mechanism_pose(0., task.slide_ref, task.spin_ref)
    assert np.allclose(captured[0][0], expected[0])
    assert np.allclose(captured[0][1], expected[1])


def test_only_sustained_stationary_hook_seating_stops_further_winding_preload(model, monkeypatch):
    task = LockingRodTask(model)
    task.stage, task._entered = "tighten", 0.
    task._initial_spin = task.spin_ref
    task._spin_command = task.spin_ref+.02
    held = []
    winding = []
    monkeypatch.setattr(task, "_track_part", lambda p, r, *args: held.append((p.copy(), r.copy())))
    monkeypatch.setattr(task, "_rotate_fastener", lambda *args: winding.append(True))
    monkeypatch.setattr(task, "_motion_stopped", lambda *args: False)
    task._last_time = 0.
    task.step(sample_for(task, hook_force=5., time=.01))
    assert task._seating_reference is None
    task.step(sample_for(task, hook_force=0., hook_contacts=0., time=.02))
    assert task._seating_reference is None and task._stable == 0.
    for time in np.arange(.03, .251, .01):
        task.step(sample_for(task, hook_force=7., time=float(time)))
    assert task._seating_reference is not None and task.tightened_verified
    assert task._tighten_seated_coordinates == (task.slide_ref, task.spin_ref)
    held.clear()
    def forbidden_winding(*args):
        raise AssertionError("Seated contact must hold instead of adding another turn")
    monkeypatch.setattr(task, "_rotate_fastener", forbidden_winding)
    task.step(sample_for(task, hook_force=5., time=.26))
    task.step(sample_for(task, hook_force=7., time=.27))
    assert len(held) == 2
    assert np.array_equal(held[0][0], held[1][0])
    assert np.array_equal(held[0][1], held[1][1])
    assert task.metrics(sample_for(task))["tighten_peak_hook_force_n"] == 7.


def test_grip_confirmation_brakes_before_starting_a_swing_path(model, monkeypatch):
    task = LockingRodTask(model)
    sample = sample_for(task, time=0., fingers=(2, 2))
    task.reference[:] = sample.joints
    task._enter("grip", sample)
    task._last_time = 0.
    task._stable = task.config.grip_confirm_s
    monkeypatch.setattr(task, "_latch_engaged", lambda *args: True)
    monkeypatch.setattr(task, "_follow_pose", lambda *args: None)
    monkeypatch.setattr(task, "_motion_stopped", lambda *args: False)
    task.step(replace(sample, time_s=.005))
    assert task._grip_hold is not None and task.stage == "grip"
    opening = task.reference[6]
    task.step(replace(sample, time_s=.010))
    assert task.stage == "grip" and task.reference[6] == opening


def test_stage_validation_rejects_skipped_fold_release_and_restore():
    expected = LockingRodTask.recorded_stages()
    assert LockingRodTask.validate_recorded_stages([stage for stage in expected for _ in range(2)])
    assert not LockingRodTask.validate_recorded_stages([stage for stage in expected if stage != "release_down"])
    assert not LockingRodTask.validate_recorded_stages(expected[:4]+expected[10:])
    with pytest.raises(ValueError, match="Unsupported"):
        LockingRodTask.recorded_stages("imaginary-v2")


def test_invalid_hinge_endpoint_and_overload_abort_are_rejected():
    with pytest.raises(ValueError, match="physical limit clip"):
        LockingRodConfig(folded_hinge_rad=-.5)
    with pytest.raises(ValueError, match="overload"):
        LockingRodConfig(overload_stop_enabled=True)


def test_preflight_covers_both_grasps_and_replays_real_mechanism_coordinates(model, monkeypatch):
    # Isolate path sampling from the separate native collision probe. Actual
    # finger contact is exercised by the authoritative task runs.
    monkeypatch.setattr(LockingRodTask, "_contact_opening", lambda *args: .33)
    task = LockingRodTask(model)
    initial = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.]
    paths = task.preflight_paths(initial)
    assert [path[0] for path in paths] == [
        "approach", "grip", "loosen", "fold_release", "fold_clear", "fold_approach", "fold_grip",
        "lower", "release_down", "clear_down", "reapproach", "regrip", "raise", "turn_release",
        "turn_clear", "turn_approach", "turn_grip", "tighten", "release", "retract",
    ]
    assert task.stage == "pending"  # Preflight has no effect on the controller.
    for _, references, allowed, overrides in paths:
        assert references.shape == (161, 8)
        assert allowed == ("fastener",)
        assert np.isfinite(references).all()
        assert all(np.shape(coordinates) == (161,) for coordinates in overrides.values())
        assert np.allclose(overrides["fastener_spin"],
                           overrides["fastener_slide"]*2*math.pi/task.lead_m)
    assert paths[7][3]["locking_rod_hinge"][-1] == task.config.folded_hinge_rad
    assert paths[12][3]["locking_rod_hinge"][-1] == 0.
