"""Current-pixel and loss/reset guarantees for the causal approach tracker."""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2', reason='optional vision environment required')
from space_arm_platform.benchmark_stereo_tracking import ApproachStereoTracker
from space_arm_platform.stereo_pose import StereoPoseEstimator
from test_stereo_pose import calibration, layout, projected, render, assert_pose


def tracker():
    return ApproachStereoTracker(StereoPoseEstimator(calibration(), layout()))


def test_tracking_uses_new_stereo_pixels_and_follows_translation_and_rotation():
    method = tracker()
    for frame in range(12):
        detections, r, t = projected(layout(), calibration(),
            rvec=[.12 + frame*.001, -.17, .05], tvec=[.03 + frame*.002, -.01, .9 - frame*.006])
        result = method.estimate_from_detections(*detections)
        assert_pose(result, r, t, translation_tolerance=.001, rotation_tolerance_deg=.1)
    assert np.linalg.norm(np.asarray(result['pose_target_to_left']['t_m']) - [.03, -.01, .9]) > .06


def test_lost_current_pair_emits_no_pose_and_two_losses_discard_old_state():
    method = tracker()
    detections, _, _ = projected(layout(), calibration())
    assert method.estimate_from_detections(*detections)['status'] == 'ok'
    for missing in (1, 2):
        result = method.estimate_from_detections(detections[0], {})
        assert result['status'] == 'rejected' and result['pose_target_to_left'] is None
        assert (method.previous is None) == (missing == 2)
    # A reacquired image must bootstrap again and may represent a new pose.
    detections, r, t = projected(layout(), calibration(), rvec=[.3, -.2, -.1], tvec=[-.04, .02, 1.2])
    assert_pose(method.estimate_from_detections(*detections), r, t)


def test_history_cannot_validate_an_epipolar_inconsistent_current_pair():
    method = tracker()
    detections, _, _ = projected(layout(), calibration())
    assert method.estimate_from_detections(*detections)['status'] == 'ok'
    corrupted = {i: corners + [0., 20.] for i, corners in detections[1].items()}
    result = method.estimate_from_detections(detections[0], corrupted)
    assert result['status'] == 'rejected' and result['pose_target_to_left'] is None


def test_actual_png_decoding_does_not_turn_blank_or_wrong_size_images_into_history_estimates():
    method = tracker()
    detections, r, t = projected(layout(), calibration())
    assert_pose(method.estimate(*(render(eye) for eye in detections)), r, t)
    blank = cv2.imencode('.png', np.zeros((1200, 1920), np.uint8))[1].tobytes()
    result = method.estimate(blank, blank)
    assert result['status'] == 'rejected' and result['pose_target_to_left'] is None
    wrong = cv2.imencode('.png', np.zeros((480, 640), np.uint8))[1].tobytes()
    result = method.estimate(wrong, wrong)
    assert result['status'] == 'rejected' and method.previous is None
