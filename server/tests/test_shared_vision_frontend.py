from pathlib import Path

import pytest
pytest.importorskip('cv2')

from space_arm_platform.provided_mono import ProvidedMonoEstimator
from space_arm_platform.stereo_pose import StereoPoseEstimator
from space_arm_platform.vision_worker import fuse_provided_detections
from test_stereo_pose import calibration,layout,projected,render,assert_pose


def test_shared_image_corners_produce_pose_without_second_decode_or_detection():
    cal, board = calibration(),layout()
    points,R,t=projected(board,cal)
    root=Path(__file__).resolve().parents[1]/'config/vision/vendor/aruco_pnp_basler_mono_v1'
    provided=ProvidedMonoEstimator(cal,board,root).estimate(*(render(p) for p in points))
    estimator=StereoPoseEstimator(cal,board)
    estimator.detector=None  # This path must not run another detector.
    result=fuse_provided_detections(estimator,provided)
    assert_pose(result,R,t)
    assert result['detected_ids']=={'left':[0,1,2,3],'right':[0,1,2,3]}


def test_shared_frontend_does_not_overwrite_duplicate_ids_or_make_up_missing_pixels():
    estimator=StereoPoseEstimator(calibration(),layout())
    duplicate={'left':{'status':'duplicate_ids'},'right':{'status':'lost'}}
    assert fuse_provided_detections(estimator,duplicate)['pose_target_to_left'] is None
    missing={'left':{'status':'adapter_rejected'},'right':{'status':'lost'}}
    result=fuse_provided_detections(estimator,missing)
    assert result['status']=='rejected' and result['pose_target_to_left'] is None
