"""Independent projection/render checks for the image-only pose baseline."""
import copy
import json

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2", reason="optional vision environment required")
from space_arm_platform.stereo_pose import StereoPoseEstimator


def calibration():
    f = 6000 / 3.45
    k = [[f, 0, 959.5], [0, f, 599.5], [0, 0, 1]]
    return dict(K_left=k, K_right=k, D_left=[0] * 5, D_right=[0] * 5,
                R_right_from_left=np.eye(3).tolist(),
                t_right_from_left_m=[-.12, 0, 0], image_size=[1920, 1200])


def layout(nonplanar=False):
    markers = {}
    for marker_id, (x, y) in enumerate([(-.12, -.10), (.12, -.10),
                                        (-.12, .10), (.12, .10)]):
        z = marker_id * .015 if nonplanar else 0
        markers[str(marker_id)] = [[x-.04, y-.04, z], [x+.04, y-.04, z],
                                   [x+.04, y+.04, z], [x-.04, y+.04, z]]
    return dict(dictionary="DICT_4X4_50", markers=markers)


def projected(markers, cal, rvec=None, tvec=None):
    """Direct OpenCV forward projection, independent of estimator helpers."""
    rvec = np.array([.12, -.17, .05]) if rvec is None else np.array(rvec, float)
    tvec = np.array([.03, -.01, .9]) if tvec is None else np.array(tvec, float)
    rmat = cv2.Rodrigues(rvec)[0]
    rig_r = np.array(cal["R_right_from_left"])
    rig_t = np.array(cal["t_right_from_left_m"])
    result = []
    for eye, r, t in [("left", rmat, tvec), ("right", rig_r @ rmat, rig_r @ tvec + rig_t)]:
        result.append({int(i): cv2.projectPoints(np.array(c, float), cv2.Rodrigues(r)[0], t,
                                                np.array(cal["K_" + eye]),
                                                np.array(cal["D_" + eye], float))[0].reshape(4, 2)
                       for i, c in markers["markers"].items()})
    return result, rmat, tvec


def render(corners):
    image = np.full((1200, 1920), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    source = np.array([[0, 0], [239, 0], [239, 239], [0, 239]], np.float32)
    for marker_id, points in corners.items():
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 240)
        homography = cv2.getPerspectiveTransform(source, points.astype(np.float32))
        warped = cv2.warpPerspective(marker, homography, (1920, 1200),
                                     flags=cv2.INTER_NEAREST, borderValue=255)
        image = np.minimum(image, warped)
    return cv2.imencode(".png", image)[1].tobytes()


def assert_pose(result, rmat, tvec, translation_tolerance=.002, rotation_tolerance_deg=1):
    assert result["status"] == "ok", result
    pose = result["pose_target_to_left"]
    assert np.linalg.norm(np.array(pose["t_m"]) - tvec) < translation_tolerance
    angle = np.linalg.norm(cv2.Rodrigues(np.array(pose["R"]) @ rmat.T)[0])
    assert np.rad2deg(angle) < rotation_tolerance_deg


def test_actual_rendered_pngs_detect_and_estimate_known_pose():
    cal, board = calibration(), layout()
    observations, rmat, tvec = projected(board, cal)
    result = StereoPoseEstimator(cal, board).estimate(*(render(p) for p in observations))
    assert_pose(result, rmat, tvec)
    assert result["detected_ids"] == {"left": [0, 1, 2, 3], "right": [0, 1, 2, 3]}
    assert np.array(result["detected_corners_px"]["left"]["0"]).shape == (4, 2)
    assert max(result["reprojection_error_px"].values()) < 1


def test_tracking_regions_use_current_pixels_in_full_coordinates_and_reset_after_loss():
    cal, board = calibration(), layout()
    observations, rmat, tvec = projected(board, cal)
    pngs = [render(p) for p in observations]
    estimator = StereoPoseEstimator(cal, board, detection_padding_px=96)
    calls = []
    detector = estimator.detector

    class CountDetector:
        def detectMarkers(self, image):
            calls.append(image.shape)
            return detector.detectMarkers(image)

    estimator.detector = CountDetector()
    initial = estimator.estimate(*pngs)
    tracked = estimator.estimate(*pngs)
    json.dumps(initial, allow_nan=False)
    json.dumps(tracked, allow_nan=False)
    assert_pose(tracked, rmat, tvec)
    assert calls[:2] == [(1200, 1920)]*2
    assert all(h*w < 1200*1920/2 for h, w in calls[2:])
    for eye in ('left', 'right'):
        for key, corners in initial['detected_corners_px'][eye].items():
            assert np.max(np.abs(np.array(tracked['detected_corners_px'][eye][key])-corners)) < .03
    assert len(calls) == 4
    lost = estimator.estimate(render({}), render({}))
    assert lost['status'] == 'rejected' and lost['pose_target_to_left'] is None
    reacquired = estimator.estimate(*pngs)
    assert_pose(reacquired, rmat, tvec)
    assert calls[-2:] == [(1200, 1920)]*2
    assert len(calls) == 8  # A failed ROI never causes a second detector pass.


@pytest.mark.parametrize("nonplanar", [False, True])
def test_noisy_corners_joint_refine_retains_accuracy(nonplanar):
    cal, board = calibration(), layout(nonplanar)
    observations, rmat, tvec = projected(board, cal)
    rng = np.random.default_rng(216)
    noisy = [{i: p + rng.normal(0, .25, p.shape) for i, p in eye.items()} for eye in observations]
    result = StereoPoseEstimator(cal, board)._estimate_from_detections(*noisy)
    assert_pose(result, rmat, tvec)


def test_single_bad_corner_rejects_whole_marker_and_uses_remaining_markers():
    cal, board = calibration(), layout()
    observations, rmat, tvec = projected(board, cal)
    observations[0][3][0] += [60, -45]
    result = StereoPoseEstimator(cal, board)._estimate_from_detections(*observations)
    assert_pose(result, rmat, tvec)
    assert 3 not in result["used_ids"]["left"]
    assert result["counts"]["used_corners_left"] == 12


@pytest.mark.parametrize("invalid", ["stereo_mismatch", "negative_depth", "tiny_parallax"])
def test_unphysical_or_unobservable_stereo_is_rejected(invalid):
    cal, board = calibration(), layout()
    if invalid == "tiny_parallax":
        # Keep angular board extent observable while independently reducing
        # stereo parallax, so this exercises the stereo conditioning gate.
        board["markers"] = {i: (20*np.array(c)).tolist() for i, c in board["markers"].items()}
    observations, _, _ = projected(board, cal, tvec=[0, 0, 200] if invalid == "tiny_parallax" else None)
    if invalid == "stereo_mismatch":
        observations[1] = {i: p + [0, 35] for i, p in observations[1].items()}
    if invalid == "negative_depth":
        observations[1] = {i: p + [140, 0] for i, p in observations[0].items()}
    result = StereoPoseEstimator(cal, board)._estimate_from_detections(*observations)
    assert result["status"] == "rejected", result
    assert result["pose_target_to_left"] is None
    assert result["reason"]
    if invalid == "tiny_parallax":
        assert "parallax" in result["reason"]


def test_one_marker_requires_explicit_degraded_opt_in():
    cal, board = calibration(), layout()
    observations, _, _ = projected(board, cal)
    one = [{0: eye[0]} for eye in observations]
    assert StereoPoseEstimator(cal, board)._estimate_from_detections(*one)["status"] == "rejected"
    result = StereoPoseEstimator(cal, board, allow_single_marker=True)._estimate_from_detections(*one)
    assert result["status"] == "degraded", result
    assert result["counts"]["common_markers"] == 1


def test_empty_invalid_and_wrong_size_images_never_return_pose():
    estimator = StereoPoseEstimator(calibration(), layout())
    blank = render({})
    for left, right in [(b"not a png", blank), (blank, blank),
                        (cv2.imencode('.png', np.full((20, 20), 255, np.uint8))[1].tobytes(), blank)]:
        result = estimator.estimate(left, right)
        assert result["status"] == "rejected"
        assert result["pose_target_to_left"] is None


def test_unknown_only_ids_are_rejected_but_pixel_detections_are_retained():
    estimator = StereoPoseEstimator(calibration(), layout())
    points = np.array([[1, 1], [2, 1], [2, 2], [1, 2]], float)
    result = estimator._estimate_from_detections({49: points}, {49: points})
    assert result["status"] == "rejected"
    assert result["detected_corners_px"]["left"]["49"] == points.tolist()


def test_actual_duplicate_marker_images_are_rejected_but_corners_are_retained():
    image = np.full((1200, 1920), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 240)
    image[200:440, 200:440] = marker
    image[600:840, 1100:1340] = marker
    png = cv2.imencode(".png", image)[1].tobytes()
    result = StereoPoseEstimator(calibration(), layout()).estimate(png, png)
    assert result["status"] == "rejected"
    assert "duplicate" in result["reason"]
    assert result["detected_ids"]["left"] == [0, 0]
    assert result["detected_corners_px"]["left"]["0"]


def test_general_calibrated_stereo_rotation_and_distortion():
    cal, board = calibration(), layout(True)
    cal["R_right_from_left"] = cv2.Rodrigues(np.array([0, .025, .005]))[0].tolist()
    cal["t_right_from_left_m"] = [-.12, .002, .001]
    cal["D_left"] = [.015, -.01, .0001, .0002, 0]
    cal["D_right"] = [.025, -.015, -.0001, .0002, 0]
    observations, rmat, tvec = projected(board, cal)
    result = StereoPoseEstimator(cal, board)._estimate_from_detections(*observations)
    assert_pose(result, rmat, tvec, translation_tolerance=1e-5, rotation_tolerance_deg=.001)


def test_corrupted_second_marker_cannot_silently_become_single_marker_success():
    cal, board = calibration(), layout()
    observations, _, _ = projected(board, cal)
    two = [{i: eye[i] for i in (0, 1)} for eye in observations]
    two[0][1][0] += [60, -45]
    result = StereoPoseEstimator(cal, board)._estimate_from_detections(*two)
    assert result["status"] == "rejected"
    assert result["pose_target_to_left"] is None


@pytest.mark.parametrize("field,value", [("t_right_from_left_m", [0, 0, 0]),
                                          ("R_right_from_left", np.zeros((3, 3)).tolist()),
                                          ("K_left", [[0, 0, 0]] * 3)])
def test_invalid_calibration_rejected_before_processing(field, value):
    cal = copy.deepcopy(calibration())
    cal[field] = value
    with pytest.raises(ValueError):
        StereoPoseEstimator(cal, layout())


# Frozen detector outputs from actual UE PNG pair, session 9a93c857, frame 0.
# Only observed pixel corners and known layout are retained; no capture truth.
LIVE_LAYOUT = {'dictionary': 'DICT_4X4_50',
 'markers': {'0': [[0.14049999999999999, 0.05476, 0.21707870012998928],
                   [0.0905, 0.05476, 0.21707870012998928],
                   [0.0905, 0.05476, 0.1670787001299893],
                   [0.14049999999999999, 0.05476, 0.1670787001299893]],
             '1': [[0.2605, 0.05476, 0.21707870012998928],
                   [0.2105, 0.05476, 0.21707870012998928],
                   [0.2105, 0.05476, 0.1670787001299893],
                   [0.2605, 0.05476, 0.1670787001299893]],
             '2': [[0.14049999999999999, 0.05476, -0.0829212998700107],
                   [0.0905, 0.05476, -0.0829212998700107],
                   [0.0905, 0.05476, -0.1329212998700107],
                   [0.14049999999999999, 0.05476, -0.1329212998700107]],
             '3': [[0.2605, 0.05476, -0.0829212998700107],
                   [0.2105, 0.05476, -0.0829212998700107],
                   [0.2105, 0.05476, -0.1329212998700107],
                   [0.2605, 0.05476, -0.1329212998700107]]}}
LIVE_CORNERS = [{0: [[1126.5855712890625, 267.12109375],
      [1213.091552734375, 271.96588134765625],
      [1218.3629150390625, 364.2420959472656],
      [1131.2738037109375, 359.76251220703125]],
  1: [[907.0054931640625, 252.93057250976562],
      [999.7545166015625, 258.25567626953125],
      [1002.7949829101562, 354.3929138183594],
      [909.8336181640625, 349.1268005371094]],
  2: [[1152.9703369140625, 845.9613037109375],
      [1241.0535888671875, 841.956787109375],
      [1246.26220703125, 939.9954833984375],
      [1157.1280517578125, 946.5277099609375]],
  3: [[924.499755859375, 857.25439453125],
      [1021.9812622070312, 852.9903564453125],
      [1024.9993896484375, 955.0780029296875],
      [928.8380126953125, 960.9865112304688]]},
 {0: [[898.0509643554688, 266.89654541015625],
      [987.0799560546875, 271.9844055175781],
      [989.6925048828125, 364.5149230957031],
      [900.0262451171875, 360.0279846191406]],
  1: [[667.0350952148438, 252.13995361328125],
      [765.0510864257812, 258.6271057128906],
      [767.1168212890625, 354.1914978027344],
      [668.8802490234375, 349.1696472167969]],
  2: [[914.9733276367188, 845.9652709960938],
      [1007.1632690429688, 841.8548583984375],
      [1011.064208984375, 940.028564453125],
      [918.1658325195312, 946.5328979492188]],
  3: [[677.0634155273438, 856.9419555664062],
      [777.931640625, 852.1431274414062],
      [779.9281616210938, 955.0579223632812],
      [679.0332641601562, 961.066650390625]]}]


def test_live_y_plane_corner_regression_has_full_stereo_consensus():
    observations = [{i: np.array(c, float) for i, c in eye.items()} for eye in LIVE_CORNERS]
    result = StereoPoseEstimator(calibration(), LIVE_LAYOUT)._estimate_from_detections(*observations)
    assert result["status"] == "ok", result
    assert result["used_ids"] == {"left": [0, 1, 2, 3], "right": [0, 1, 2, 3]}
    pose = result["pose_target_to_left"]
    for index, eye in enumerate(("left", "right")):
        xyz = np.concatenate([LIVE_LAYOUT["markers"][str(i)] for i in range(4)])
        uv = np.concatenate([observations[index][i] for i in range(4)])
        rotation = np.array(pose["R"])
        translation = np.array(pose["t_m"]) + (np.array([-.12, 0, 0]) if index else 0)
        prediction = cv2.projectPoints(xyz, cv2.Rodrigues(rotation)[0], translation,
                                       np.array(calibration()["K_"+eye]), None)[0].reshape(-1, 2)
        rms = np.sqrt(np.mean(np.sum((prediction-uv)**2, axis=1)))
        assert rms < 1, result
        assert np.isclose(result["reprojection_error_px"][eye], rms)
        assert np.all((xyz @ rotation.T + translation)[:, 2] > 0)


@pytest.mark.parametrize("frame_rvec", [[np.pi/2, 0, 0], [0, np.pi/2, 0], [np.pi, 0, 0]])
@pytest.mark.parametrize("outlier", [False, True])
def test_pose_covariance_under_rotated_and_flipped_target_planes(frame_rvec, outlier):
    # Re-express one physical scene in different target frames. Camera pixels
    # stay identical; the returned target->camera transform must change exactly
    # by the inverse object-frame transform, including near-pi orientations.
    cal, board = calibration(), layout()
    frame_rotation = cv2.Rodrigues(np.array(frame_rvec, float))[0]
    offset = np.array([.1755, .05476, .0420787])
    board["markers"] = {i: (np.array(c) @ frame_rotation.T + offset).tolist()
                         for i, c in board["markers"].items()}
    optical_rotation = cv2.Rodrigues(np.array([.12, -.17, .05]))[0]
    expected_rotation = optical_rotation @ frame_rotation.T
    expected_translation = np.array([.03, -.01, .9]) - expected_rotation @ offset
    observations, _, _ = projected(board, cal, rvec=cv2.Rodrigues(expected_rotation)[0].reshape(3),
                                    tvec=expected_translation)
    if outlier:
        observations[0][3][0] += [60, -45]
    result = StereoPoseEstimator(cal, board)._estimate_from_detections(*observations)
    assert_pose(result, expected_rotation, expected_translation, translation_tolerance=1e-5,
                rotation_tolerance_deg=.001)
    assert len(result["used_ids"]["left"]) == (3 if outlier else 4)
    covariance = np.array(result['pose_covariance']['matrix'])
    assert covariance.shape == (6, 6)
    assert np.allclose(covariance, covariance.T)
    assert np.linalg.eigvalsh(covariance)[0] > 0
    assert np.linalg.norm(covariance[:3, 3:]) > 1e-12


def test_pose_covariance_covers_independent_pixel_noise_and_has_a_nonzero_floor():
    cal, board = calibration(), layout()
    observations, r, t = projected(board, cal, tvec=[.03, -.01, 1.5])
    estimator = StereoPoseEstimator(cal, board)
    perfect = estimator.estimate_from_detections(*observations)
    assert perfect['status'] == 'ok'
    assert np.linalg.eigvalsh(perfect['pose_covariance']['matrix'])[0] > 0
    rng = np.random.default_rng(31)
    nees = []
    for _ in range(30):
        noisy = [{i: corners+rng.normal(0, .5, corners.shape) for i, corners in eye.items()} for eye in observations]
        result = estimator.estimate_from_detections(*noisy)
        assert result['status'] == 'ok', result['reason']
        pose = result['pose_target_to_left']
        # Independent forward projections and known pose score the full
        # six-dimensional residual, including position/attitude correlation.
        delta = np.r_[np.array(pose['t_m'])-t, cv2.Rodrigues(np.array(pose['R'])@r.T)[0].reshape(3)]
        nees.append(float(delta@np.linalg.solve(result['pose_covariance']['matrix'], delta)))
    assert .2 < np.mean(nees) < 8
    assert np.mean(np.array(nees) < 16.812) >= .95
