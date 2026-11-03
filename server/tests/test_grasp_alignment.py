import math
from pathlib import Path
import numpy as np
import pytest

from space_arm_platform.grasp_alignment import load_capture_mount, alignment_from_pose

ROOT = Path(__file__).resolve().parents[1]
CENTER = np.array([.1755, .05475, .0420787001299893])
NOMINAL_R_CT = np.array([[0., 0, 1], [-1, 0, 0], [0, -1, 0]])


def observation(position=(0, .075, .531), rotation=NOMINAL_R_CT):
    return {'status': 'tracking', 'camera_id': 'teleop/camera/capture_wrist_cam',
            'pose': {'translation_m': (np.asarray(position)-rotation@CENTER).tolist(),
                     'rotation_camera_from_target': rotation.tolist()}}


@pytest.fixture
def mount():
    return load_capture_mount(ROOT / 'model/SARM/platform/sarm_dual_capture26.xml')


def test_camera_offset_is_removed_at_nominal_half_metre_pose(mount):
    result = alignment_from_pose(observation(), mount, standoff_m=.5)
    assert result['valid']
    assert np.allclose(result['interface_position_grip_m'], [0, 0, -.5], atol=1e-10)
    assert np.allclose(result['position_error_display_mm'], [0, 0, 0], atol=1e-7)
    assert np.allclose(result['rotation_error_display_deg'], [0, 0, 0], atol=1e-7)
    assert result['control_enabled'] is False


def test_signed_lateral_and_axial_error_in_right_down_forward_axes(mount):
    result = alignment_from_pose(observation((.002, .072, .511)), mount)
    assert np.allclose(result['interface_position_display_m'], [.002, -.003, .480], atol=1e-10)
    assert np.allclose(result['position_error_display_mm'], [2, -3, -20], atol=1e-7)


@pytest.mark.parametrize('axis, degrees', [(0,10), (1,-7), (2,10), (2,179.9999)])
def test_signed_rotation_is_correction_about_grip_display_axes(mount,axis,degrees):
    a=math.radians(degrees)
    v=np.eye(3)[axis]
    k=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    delta=np.eye(3)+math.sin(a)*k+(1-math.cos(a))*(k@k)
    result=alignment_from_pose(observation(rotation=delta@NOMINAL_R_CT), mount)
    assert np.allclose(result['rotation_error_display_deg'], v*degrees, atol=1e-5)


def test_nonzero_camera_tilt_is_transformed_instead_of_treated_as_grip_error(mount):
    a=math.radians(30)
    tilt=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
    R_GC=tilt@np.array(mount['R_grip_from_camera_optical'])
    changed=dict(mount,R_grip_from_camera_optical=R_GC.tolist())
    R_TI=np.array(mount['R_target_from_interface'])
    rotation=R_GC.T@R_TI.T
    p_CI=R_GC.T@(np.array([0.,0.,-.5])-np.array(mount['camera_position_grip_m']))
    result=alignment_from_pose(observation(p_CI,rotation),changed)
    assert np.allclose(result['position_error_display_mm'],0,atol=1e-7)
    assert np.allclose(result['rotation_error_display_deg'],0,atol=1e-7)


def test_rejected_visual_pose_does_not_produce_control_error(mount):
    result=alignment_from_pose({'status':'ambiguous','pose':None}, mount)
    assert not result['valid'] and 'position_error_display_mm' not in result


def test_corrupt_rotation_and_wrong_camera_are_rejected(mount):
    bad=observation(); bad['pose']['rotation_camera_from_target'][0][0]=1
    with pytest.raises(ValueError, match='rotation'):
        alignment_from_pose(bad,mount)
    bad=observation(); bad['camera_id']='teleop/camera/capture_stereo_left'
    with pytest.raises(ValueError, match='camera'):
        alignment_from_pose(bad,mount)
