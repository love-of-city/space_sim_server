"""Additional fixed onboard cameras, separate from the original stereo rig."""
from dataclasses import replace
import numpy as np
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix

WORKSPACE_ID = 'teleop/camera/capture_workspace_cam'
GLOBAL_ID = 'teleop/camera/capture_global_cam'
APPROACH_ID = 'teleop/camera/capture_approach_cam'
JAWS_ID = 'teleop/camera/capture_jaw_cam'
CAMERAS = [GLOBAL_ID, APPROACH_ID, JAWS_ID]
WORK_LIGHT_INTENSITY = 1500.0  # Existing adapter units, not calibrated candelas.
GLOBAL_OFFSET = (1.8, -2., 1.15)
GLOBAL_AIM = (.95, 0., -.08)
# Keep the approved illumination fixed when adjusting global-camera framing.
GLOBAL_LIGHT_OFFSET = (2.4, -2.4, 1.5)
GLOBAL_LIGHT_AIM = (.9, 0., .1)
APPROACH_OFFSET = np.array([.20, 0., .10])


def look_rotation(position, aim, up=(0., 1., 0.)):
    forward = np.asarray(aim)-np.asarray(position)
    forward = forward/np.linalg.norm(forward)
    left = np.cross(up, forward); left /= np.linalg.norm(left)
    return np.column_stack((forward, left, np.cross(forward, left)))


APPROACH_ROTATION = look_rotation(APPROACH_OFFSET, [0., 0., -.25])
# Optical frame: right, down, forward. Render camera: forward, left, up.
GRIP_FROM_OPTICAL = APPROACH_ROTATION @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def task_cameras(grip_position):
    from bsk_render_adapter import CameraVisual
    result = []
    for cid, parent, pos, aim, up, fov, label in [
        (GLOBAL_ID, 'teleop/cubesat_bus', GLOBAL_OFFSET, GLOBAL_AIM, [0.,0.,1.], 65., 'Capture global view (both spacecraft)'),
        (APPROACH_ID, 'teleop/gripper', APPROACH_OFFSET, [0.,0.,-.25], [0.,1.,0.], 100., 'Capture approach (single camera)'),
        (JAWS_ID, 'teleop/gripper', [.14,-.14,.025], [0.,0.,-.025], [0.,0.,1.], 45., 'Capture jaws side view'),
    ]:
        result.append(CameraVisual(camera_id=cid, parent_id=parent, position_body_m=tuple(pos),
            orientation_body_from_camera_wxyz=tuple(matrix_to_quaternion_wxyz(look_rotation(pos,aim,up))),
            field_of_view_rad=np.deg2rad(fov), resolution=(1024,1024), display_name=label,
            aperture_fnumber=8. if cid == GLOBAL_ID else 0.,
            picture_in_picture=True, picture_in_picture_slot=len(result)+1,
            capture_rate_hz=50., capture_products=('rgb',)))
    return result


def configure_capture_cameras(bridge, grip_position):
    original = [camera.to_payload() for camera in bridge._cameras]
    # Keep every original camera's mount/optics. The unrequested vision route
    # leaves these available for preview without extra dataset readbacks.
    for camera in bridge._cameras:
        camera.capture_consumer = 'vision'
    bridge.vision_capture_camera_ids = ()
    from simulation.camera_lighting import overview_camera_lamp
    lights = []
    for camera in task_cameras(grip_position):
        bridge.add_camera(camera)
        # Scene lights illuminate preview and authoritative RGB alike. Unlike
        # the WebRTC work-light button, this also works in offline capture.
        lamp = overview_camera_lamp(camera.camera_id)
        global_view = camera.camera_id == GLOBAL_ID
        lamp_position = GLOBAL_LIGHT_OFFSET if global_view else camera.position_body_m
        lamp_normal = (look_rotation(GLOBAL_LIGHT_OFFSET, GLOBAL_LIGHT_AIM, [0.,0.,1.])[:,0]
                       if global_view else quaternion_wxyz_to_matrix(
                           camera.orientation_body_from_camera_wxyz)[:,0])
        lamp = replace(lamp, visual_id='teleop/light/'+camera.camera_id.rsplit('/', 1)[-1]+'_lamp',
                       parent_id=camera.parent_id, position_body_m=lamp_position,
                       normal_body=tuple(lamp_normal),
                       range_m=12. if global_view else 2.,
                       properties={**lamp.properties, 'intensity': 160000. if global_view else WORK_LIGHT_INTENSITY})
        bridge.add_lights(lamp)
        lights.append(lamp.to_payload())
    import json
    print(json.dumps(dict(type='capture_work_lighting_configuration', lights=lights,
                          scope='shared_scene_preview_and_dataset',
                          intensity_units='adapter_legacy_units',
                          exposure='unchanged_platform_manual_exposure')), flush=True)
    return original
