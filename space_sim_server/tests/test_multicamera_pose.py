import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.multicamera_pose import MultiCameraPoseEstimator


def scene():
    k = [[1739.13, 0, 959.5], [0, 1739.13, 599.5], [0, 0, 1]]
    cal = dict(K_left=k, K_right=k, D_left=[0]*5, D_right=[0]*5,
               R_right_from_left=np.eye(3).tolist(), t_right_from_left_m=[-.12, 0, 0], image_size=[1920, 1200])
    markers = {}
    for i, (x, y) in enumerate([(-.1, -.08), (.1, -.08), (-.1, .08), (.1, .08)]):
        markers[str(i)] = [[x+dx, y+dy, 0] for dx, dy in [(-.02,-.02),(.02,-.02),(.02,.02),(-.02,.02)]]
    e = MultiCameraPoseEstimator(cal, dict(dictionary='DICT_4X4_50', markers=markers))
    r = Rotation.from_euler('xyz', [12, 20, 5], degrees=True).as_matrix(); t = np.array([.02,-.01,.7])
    obs = []
    for eye in (0, 1):
        obs.append({int(i): e.geometry._project(np.array(xyz), r, t + ([-.12,0,0] if eye else np.zeros(3)), eye)
                    for i, xyz in markers.items()})
    return e, obs, r, t


def test_joint_pose_with_no_common_markers_and_no_truth_input():
    e, obs, r, t = scene()
    result = e.estimate_from_detections({i: obs[0][i] for i in (0, 2)}, {i: obs[1][i] for i in (1, 3)})
    assert result['methods']['triangulation_registration']['status'] == 'rejected'
    for route in ('mono_left', 'mono_right', 'pnp_fusion', 'joint_multicamera'):
        pose = result['methods'][route]
        assert pose['status'] == 'ok', pose
        np.testing.assert_allclose(pose['pose_target_to_left']['t_m'], t, atol=1e-7)
        np.testing.assert_allclose(pose['pose_target_to_left']['R'], r, atol=1e-7)


def test_same_points_all_routes_and_right_eye_reference():
    e, obs, r, t = scene()
    result = e.estimate_from_detections(*obs)
    for pose in result['methods'].values():
        assert pose['status'] == 'ok', pose
        np.testing.assert_allclose(pose['pose_target_to_left']['t_m'], t, atol=1e-7)


def test_missing_eye_never_turns_joint_estimate_into_mono_fallback():
    e, obs, _, _ = scene()
    result = e.estimate_from_detections(obs[0], {})
    assert result['methods']['mono_left']['status'] == 'ok'
    for name in ('pnp_fusion', 'triangulation_registration', 'joint_multicamera'):
        assert result['methods'][name]['status'] == 'rejected'
        assert result['methods'][name]['pose_target_to_left'] is None


@pytest.mark.parametrize('left_ids,right_ids', [((0,), (0,)), ((0,), (1,)), ((0, 2), (1,)), ((0,), (1, 3))])
def test_joint_single_complete_marker_keeps_both_eyes_and_recovers_pose(left_ids, right_ids):
    e, obs, r, t = scene()
    result = e.estimate_from_detections({i: obs[0][i] for i in left_ids}, {i: obs[1][i] for i in right_ids})
    joint = result['methods']['joint_multicamera']
    assert joint['status'] == 'ok', joint
    assert joint['reduced_redundancy'] is True
    assert joint['used_ids'] == dict(left=list(left_ids), right=list(right_ids))
    np.testing.assert_allclose(joint['pose_target_to_left']['R'], r, atol=1e-7)
    np.testing.assert_allclose(joint['pose_target_to_left']['t_m'], t, atol=1e-7)


def test_two_marker_reference_policy_remains_available():
    e, obs, _, _ = scene()
    e = MultiCameraPoseEstimator(dict(K_left=e.geometry.k[0], K_right=e.geometry.k[1],
        D_left=e.geometry.d[0], D_right=e.geometry.d[1], R_right_from_left=e.geometry.rig_r,
        t_right_from_left_m=e.geometry.rig_t, image_size=e.geometry.image_size),
        dict(markers=e.geometry.markers), min_markers_per_eye=2)
    assert e.estimate_from_detections(obs[0], {0: obs[1][0]})['methods']['joint_multicamera']['status'] == 'rejected'


def test_single_marker_incompatible_stereo_pixels_are_rejected():
    e, obs, _, _ = scene()
    result = e.estimate_from_detections({0: obs[0][0]}, {0: obs[1][0] + [0, 100]})
    joint = result['methods']['joint_multicamera']
    assert joint['status'] == 'rejected'
    assert joint['pose_target_to_left'] is None


def test_single_marker_too_small_still_rejected():
    e, obs, _, _ = scene()
    corners = obs[1][0]
    tiny = corners.mean(axis=0) + (corners - corners.mean(axis=0)) * .001
    result = e.estimate_from_detections(obs[0], {0: tiny})
    assert result['methods']['joint_multicamera']['status'] == 'rejected'


def test_full_resolution_low_contrast_marker_decodes_without_brightening_input():
    e, _, _, _ = scene()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = np.full((1200, 1920), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(dictionary, 19, 120)
    image[500:620, 800:920] = 240 + np.rint(marker.astype(float) * 15 / 255).astype(np.uint8)
    image = cv2.GaussianBlur(image, (0, 0), 2)
    original_pixels = image.copy()
    _, encoded = cv2.imencode('.png', image)
    detected = e.detect(encoded.tobytes())
    assert 19 in detected
    np.testing.assert_allclose(detected[19], [[800, 500], [919, 500], [919, 619], [800, 619]], atol=2)
    np.testing.assert_array_equal(image, original_pixels)


@pytest.mark.parametrize('bad_marker', [False, True])
def test_nonminimal_seed_recovers_from_ransac_failure_with_same_inlier_gates(monkeypatch, bad_marker):
    e, obs, r, t = scene()
    if bad_marker:
        obs[0][3] = obs[0][3] + [120, -85]
        obs[1][3] = obs[1][3] + [120, -85]
    def failed_seed(*args):
        raise ValueError('minimal-sample initialization failed')
    monkeypatch.setattr(e.geometry, '_pnp', failed_seed)
    result = e.estimate_from_detections(*obs)
    joint = result['methods']['joint_multicamera']
    assert joint['status'] == 'ok', joint
    np.testing.assert_allclose(joint['pose_target_to_left']['t_m'], t, atol=1e-7)
    np.testing.assert_allclose(joint['pose_target_to_left']['R'], r, atol=1e-7)
    if bad_marker:
        assert 3 not in joint['used_ids']['left']
        assert 3 not in joint['used_ids']['right']
