from copy import deepcopy
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from space_arm_platform.collection_cameras import (
    apply_camera_profile, camera_ids, freeze_camera_profile,
)
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix


def test_fixed_camera_looks_at_workspace_and_preserves_original_cameras():
    root = ET.fromstring('''<mujoco><worldbody><body name="cubesat_bus">
        <camera name="spacecraft_overview" pos="0 1 2" quat="1 0 0 0"/>
        <body name="wrist"><camera name="sarm_wrist_cam" pos="0 0 1"/></body>
        </body></worldbody></mujoco>''')
    original = [ET.tostring(c) for c in root.findall('.//camera')]
    scene = {'collection_camera_profile': 'single-arm-three-view-v1'}
    profile = apply_camera_profile(root, scene)
    assert [ET.tostring(c) for c in root.findall('.//camera')[:2]] == original
    camera = root.find('.//camera[@name="workbench_side"]')
    rotation = quaternion_wxyz_to_matrix(np.fromstring(camera.get('quat'), sep=' '))
    mount = profile['mounts'][0]
    direction = np.array(mount['look_at_m']) - np.array(mount['position_m'])
    assert np.allclose(-rotation[:, 2], direction / np.linalg.norm(direction))
    assert np.linalg.det(rotation) == pytest.approx(1)
    apply_camera_profile(root, scene)
    assert len(root.findall('.//camera')) == 3
    camera.set('pos', '0 0 0')
    with pytest.raises(ValueError, match='Existing fixed camera'):
        apply_camera_profile(root, scene)


def test_frozen_profile_detects_drift_and_requires_actual_model_cameras():
    scene = {'collection_camera_profile': 'single-arm-three-view-v1'}
    freeze_camera_profile(scene)
    changed = deepcopy(scene)
    changed['collection_config']['camera_profile']['mounts'][0]['fovy_deg'] = 20
    with pytest.raises(ValueError, match='Frozen camera profile'):
        camera_ids(changed)
    with pytest.raises(ValueError, match='missing or duplicated'):
        apply_camera_profile(ET.fromstring('<mujoco><worldbody><body name="cubesat_bus"/></worldbody></mujoco>'), scene)


def test_three_view_recording_rejects_two_view_settings(tmp_path):
    from collection.record_scripted_task import recording_settings
    scene = {'collection_camera_profile': 'single-arm-three-view-v1'}
    path = tmp_path / 'recording.json'
    path.write_text('{}')
    with pytest.raises(ValueError, match='selected scene camera set'):
        recording_settings(path, scene)
    path.write_text(json.dumps({'camera_ids': camera_ids(scene)}))
    assert len(recording_settings(path, scene).camera_ids) == 3

@pytest.mark.parametrize('corruption', ['fov', 'position', 'rotation'])
def test_third_camera_actual_calibration_is_checked(corruption):
    from space_arm_platform.collection_cameras import camera_profile, mount_rotation, validate_profile_capture
    scene = {'collection_camera_profile': 'single-arm-three-view-v1'}
    mount = camera_profile(scene)['mounts'][0]
    metadata = dict(camera_id='teleop/camera/workbench_side',
        intrinsics=dict(fov_x_rad=np.deg2rad(mount["fovy_deg"]), fov_y_rad=np.deg2rad(mount["fovy_deg"])),
        floating_origin=dict(c_LN=np.eye(3).reshape(-1).tolist()),
        extrinsics=dict(position_L_m=list(mount['position_m']),
            c_NC=(mount_rotation(mount) @ np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])).reshape(-1).tolist()))
    observation = dict(attitude_control=dict(orientation_inertial_wxyz=[1., 0., 0., 0.]))
    validate_profile_capture(metadata, observation, scene)
    if corruption == 'fov':
        metadata['intrinsics']['fov_y_rad'] += .1
    elif corruption == 'position':
        metadata['extrinsics']['position_L_m'][0] += .1
    else:
        metadata['extrinsics']['c_NC'] = np.eye(3).reshape(-1).tolist()
    with pytest.raises(AssertionError):
        validate_profile_capture(metadata, observation, scene)
