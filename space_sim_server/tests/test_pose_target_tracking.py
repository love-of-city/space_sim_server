"""Do not send socket vibration straight into the arm servo."""
import numpy as np
import pytest
from simulation.scripted_tasks.pose_tracking import PoseTargetFilter
from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector


def test_socket_vibration_is_rejected_but_true_drift_is_followed():
    initial = axis_angle_to_matrix(np.array([0., 0., 1.]), 3.13)
    p0 = np.array([.37, .22, .035])
    tracker = PoseTargetFilter(p0, initial)
    errors = []
    for t in np.arange(.005, 4., .005):
        wave = np.sin(2*np.pi*21.5*t)
        p = p0 + np.array([.00027*wave, .001*min(t, 1.), 0.])
        r = axis_angle_to_matrix(np.array([0., 1., 0.]), .008*wave) @ initial
        p_copy, r_copy = p.copy(), r.copy()
        out_p, out_r = tracker.update(p, r, .005)
        np.testing.assert_array_equal(p, p_copy)
        np.testing.assert_array_equal(r, r_copy)
        if t > 2.:
            errors.append([out_p[0]-p0[0], np.linalg.norm(rotation_matrix_to_vector(out_r@initial.T))])
    errors = np.asarray(errors)
    assert np.max(abs(errors[:, 0])) < .00027*.01
    assert np.max(abs(errors[:, 1])) < .008*.01
    assert out_p[1]-p0[1] == pytest.approx(.001, abs=1e-7)


def test_tracking_held_measurement_is_independent_of_control_step_partition():
    a = PoseTargetFilter(np.zeros(3), np.eye(3))
    b = PoseTargetFilter(np.zeros(3), np.eye(3))
    p, r = np.array([.003, .002, 0.]), axis_angle_to_matrix(np.array([0.,0.,1.]), .04)
    ap, ar = a.update(p, r, .1)
    for _ in range(20):
        bp, br = b.update(p, r, .005)
    np.testing.assert_allclose(ap, bp, atol=1e-15)
    np.testing.assert_allclose(ar, br, atol=1e-15)
