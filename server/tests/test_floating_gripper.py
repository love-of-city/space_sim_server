"""Contact feedback must avoid squeezing a blocked or misaligned target."""
from dataclasses import replace
import numpy as np
import pytest
from test_floating_capture import realized, sample_at_grasp
from simulation.scripted_tasks.floating_capture import create_floating_task


def gripping(tmp_path):
    scene, model = realized(tmp_path)
    task = create_floating_task('floating_bolt_capture', model)
    sample = sample_at_grasp(task, scene, contacts=(1, 1))
    q = sample.joints.copy()
    q[6:] = [.3, -.3]
    sample = replace(sample, joints=q)
    task.stage = 'grip'
    task.reference[:] = q
    return task, sample


def test_blocked_misaligned_jaw_does_not_accumulate_full_close_target(tmp_path):
    task, sample = gripping(tmp_path)
    # Observed failure: measured q~=0.29, command kept falling to zero while
    # the target was >8 mm off-frame and contact force rose above 800 N.
    sample = replace(sample, finger_normal_force_n=(850., 780.),
                     plug_position_body_m=sample.plug_position_body_m+[.012, 0, 0])
    task.reference[6:] = [0., 0.]
    for _ in range(200):
        task._drive_gripper(sample, .005, task.metrics(sample))
        assert task.reference[6] >= sample.joints[6]-.002-1e-12
        assert task.reference[7] == -task.reference[6]
    assert task.reference[6] > sample.joints[6]
    assert task._hold_joints is None
    assert task.result is None


def test_one_sided_contact_waits_for_alignment_before_closing(tmp_path):
    task, sample = gripping(tmp_path)
    sample = replace(sample, finger_plug_contacts=(1, 0), finger_normal_force_n=(1., 0.))
    for _ in range(200):
        task._drive_gripper(sample, .005, {'grasp_error_m': .01})
    assert task.reference[6] == pytest.approx(sample.joints[6])
    task._drive_gripper(sample, .005, {'grasp_error_m': .0001})
    assert task.reference[6] < sample.joints[6]


@pytest.mark.parametrize('stage', ['hold', 'retract', 'settle'])
def test_retained_grasp_adjusts_preload_without_changing_measurements(tmp_path, stage):
    task, sample = gripping(tmp_path)
    task.stage = stage
    task._hold_joints = task.reference[:6].copy()
    task._hold_opening = .3
    measured = sample.joints.copy()
    weak = replace(sample, finger_normal_force_n=(.2, 1.))
    for _ in range(200):
        task._drive_gripper(weak, .005, task.metrics(weak))
    assert .298-1e-12 <= task.reference[6] < .3
    before = task.reference[6]
    high = replace(sample, finger_normal_force_n=(13., .2))
    task._drive_gripper(high, .005, task.metrics(high))
    assert task.reference[6] > before
    assert task.reference[6]-before <= .02*.005+1e-12
    np.testing.assert_array_equal(sample.joints, measured)
    assert task.result is None


def test_free_closing_limits_first_contact_impact(tmp_path):
    task, sample = gripping(tmp_path)
    sample = replace(sample, finger_plug_contacts=(0, 0), finger_normal_force_n=(0., 0.))
    before = task.reference[6]
    task._drive_gripper(sample, .005, task.metrics(sample))
    assert before-task.reference[6] == pytest.approx(.06*.005)
