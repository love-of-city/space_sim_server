"""User-provided monocular source stays byte-original under the adapter."""
import hashlib
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2", reason="optional vision environment required")
from space_arm_platform.provided_mono import ProvidedMonoEstimator
from test_stereo_pose import calibration, layout, projected, render

VENDOR = Path(__file__).parents[1] / "config/vision/vendor/aruco_pnp_basler_mono_v1"
SOURCE_SHA = "80a5bf3b97bf3b360307593894166124d87667e70bc944e70807262568478dac"


def test_supplied_source_processes_actual_pixels_using_current_8cm_layout():
    cal, board = calibration(), layout()
    observations, rmat, tvec = projected(board, cal)
    result = ProvidedMonoEstimator(cal, board, VENDOR).estimate(*(render(p) for p in observations))
    assert result["scope"] == "independent_monocular_no_stereo_fusion"
    assert result["navigation_for_control"]["valid"] is False
    assert SOURCE_SHA in result["source"]
    for eye, expected_t in [("left", tvec), ("right", tvec + np.array([-.12, 0, 0]))]:
        output = result[eye]
        assert output["status"] == "tracking", output
        assert output["marker_count"] == 4
        assert output["reprojection_error_px"] < 1
        assert np.linalg.norm(np.array(output["pose"]["translation_m"]) - expected_t) < .002
        assert np.rad2deg(np.linalg.norm(cv2.Rodrigues(np.array(output["pose"]["rotation_camera_from_target"]) @ rmat.T)[0])) < 1
    assert "legacy_tool_measurement" not in result["left"]


def test_no_rig_extrinsics_or_old_intrinsics_helper_are_used(monkeypatch):
    cal, board = calibration(), layout()
    observations, _, _ = projected(board, cal)
    del cal["R_right_from_left"]
    del cal["t_right_from_left_m"]
    estimator = ProvidedMonoEstimator(cal, board, VENDOR)
    module = sys.modules[estimator.estimators[0].__class__.__module__]
    assert module.__name__.startswith("_provided_aruco_")
    monkeypatch.setattr(module, "wrist_intrinsics", lambda *args: pytest.fail("old helper must not run"))
    result = estimator.estimate(*(render(p) for p in observations))
    assert result["left"]["status"] == "tracking"
    assert result["right"]["status"] == "tracking"


def test_one_bad_eye_is_rejected_without_contaminating_other_eye():
    cal, board = calibration(), layout()
    observations, _, _ = projected(board, cal)
    estimator = ProvidedMonoEstimator(cal, board, VENDOR)
    wrong_size = cv2.imencode('.png', np.zeros((1200, 1919), np.uint8))[1].tobytes()
    result = estimator.estimate(wrong_size, render(observations[1]))
    assert result["left"]["status"] == "adapter_rejected"
    assert result["left"]["pose"] is None
    assert result["right"]["status"] == "tracking"


def test_original_lost_status_is_preserved_for_blank_image():
    png = render({})
    result = ProvidedMonoEstimator(calibration(), layout(), VENDOR).estimate(png, png)
    assert result["left"]["status"] == "lost"
    assert result["left"]["pose"] is None
    assert result["right"]["status"] == "lost"


def test_arbitrary_target_frame_plane_and_marker_size_are_preserved():
    cal, board = calibration(), layout()
    plane = cv2.Rodrigues(np.array([np.pi/2, 0, 0]))[0]
    origin = np.array([.1755, .0578, .0420787])
    board["markers"] = {i: (np.array(c) @ plane.T + origin).tolist() for i, c in board["markers"].items()}
    optical_rotation = cv2.Rodrigues(np.array([.12, -.17, .05]))[0]
    expected_rotation = optical_rotation @ plane.T
    expected_translation = np.array([.03, -.01, .9]) - expected_rotation @ origin
    observations, _, _ = projected(board, cal, rvec=cv2.Rodrigues(expected_rotation)[0].reshape(3),
                                    tvec=expected_translation)
    result = ProvidedMonoEstimator(cal, board, VENDOR).estimate(*(render(p) for p in observations))
    assert result["left"]["status"] == "tracking", result
    assert np.linalg.norm(np.array(result["left"]["pose"]["translation_m"]) - expected_translation) < .002


def test_nonplanar_layout_is_honestly_unsupported_by_provided_planar_ippe():
    cal, board = calibration(), layout(True)
    observations, _, _ = projected(board, cal)
    result = ProvidedMonoEstimator(cal, board, VENDOR).estimate(*(render(p) for p in observations))
    for eye in ("left", "right"):
        assert result[eye]["status"] == "unsupported_layout"
        assert result[eye]["pose"] is None


def test_nonzero_distortion_is_rejected_instead_of_ignored():
    cal = calibration()
    cal["D_left"][0] = .01
    with pytest.raises(ValueError, match="distortion"):
        ProvidedMonoEstimator(cal, layout(), VENDOR)


def test_vendor_bytes_match_manifest_and_tampering_is_rejected(tmp_path):
    assert len([p for p in VENDOR.rglob("*") if p.is_file()]) == 12
    assert hashlib.sha256((VENDOR / "source/space_arm_platform/fiducials.py").read_bytes()).hexdigest() == SOURCE_SHA
    for line in (VENDOR / "FILES.sha256").read_text().splitlines():
        expected, relative = line.split(None, 1)
        assert hashlib.sha256((VENDOR / relative.strip()).read_bytes()).hexdigest() == expected
    altered = tmp_path / "package"
    shutil.copytree(VENDOR, altered)
    path = altered / "source/space_arm_platform/fiducials.py"
    path.write_bytes(path.read_bytes() + b"\n# changed\n")
    with pytest.raises(ValueError, match="SHA256"):
        ProvidedMonoEstimator(calibration(), layout(), altered)
