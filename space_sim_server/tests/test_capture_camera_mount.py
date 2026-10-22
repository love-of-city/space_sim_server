import json
from pathlib import Path
import xml.etree.ElementTree as ET
import math
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def _rotation(wxyz):
    w, x, y, z = wxyz / np.linalg.norm(wxyz)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def test_camera_optical_center_is_on_outer_end_face_and_just_outside_shell():
    model = ET.parse(ROOT / 'model/SARM/platform/sarm_dual_capture26.xml').getroot()
    camera = model.find(".//body[@name='gripper']/camera[@name='capture_wrist_cam']")
    position = np.fromstring(camera.get('pos'), sep=' ')
    # Front surface of D138 pressure plate; shell has the same 69 mm radius.
    mesh = ROOT / 'model/dual_arm_capture26/meshes/capture26_10_round_pressure_plate_D138.stl'
    raw = mesh.read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    vertices = np.frombuffer(raw, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3)
    assert abs(position[2] - vertices[:, 2].min()) < 1e-7
    radius = np.linalg.norm(vertices[:, :2], axis=1).max()
    clearance = np.linalg.norm(position[:2]) - radius
    assert 0.003 < clearance < 0.010
    assert camera.get('quat') is not None  # Renderer reads quat, not xyaxes.
    assert camera.get('xyaxes') is None

def test_capture_camera_is_mounted_on_new_gripper_and_matches_startup_profile(tmp_path):
    from space_arm_platform.models import SceneInstanceCreate
    from space_arm_platform.scene_runtime import SceneRuntimeManager
    model = ET.parse(ROOT / 'model/SARM/platform/sarm_dual_capture26.xml').getroot()
    profile = json.loads((ROOT / 'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    created = manager.create_instance(SceneInstanceCreate(
        template_id='sarm-dual-capture26', seed=1, randomization_profile='none'))
    scene = json.loads(Path(created['config_path']).read_text(encoding='utf-8'))
    assert scene['render_profile']['camera_mounts'] == profile['camera_mounts']
    for name, mount in profile['camera_mounts'].items():
        camera = model.find(f".//body[@name='{mount['body']}']/camera[@name='{name}']")
        assert camera is not None
        for key, value in mount.items():
            if key != 'body':
                assert camera.get(key) == value


def test_grasp_axis_stays_in_view_before_final_contact():
    profile = json.loads((ROOT / 'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
    mount = profile['camera_mounts']['capture_wrist_cam']
    position = np.fromstring(mount['pos'], sep=' ')
    wxyz = np.fromstring(mount['quat'], sep=' ')
    rotation = _rotation(wxyz)
    limit = math.tan(math.radians(float(mount['fovy']) / 2))
    for distance in (0.50, 0.25, 0.10, 0.05):
        point = rotation.T @ (np.array([0., 0., -distance]) - position)
        assert point[2] < 0
        assert max(abs(point[0]), abs(point[1])) < -point[2] * limit


def test_renderer_uses_the_same_mount_and_optical_direction(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT.parent / 'space_sim_UE_Adapter/Adapters'))
    from bsk_render_adapter.mjcf_assets import parse_mjcf_scene_metadata
    metadata = parse_mjcf_scene_metadata(ROOT / 'model/SARM/platform/sarm_dual_capture26.xml', namespace='teleop')
    camera = next(c for c in metadata.cameras if c['camera_id'] == 'teleop/camera/capture_wrist_cam')
    assert camera['parent_id'] == 'teleop/gripper'
    assert np.allclose(camera['position_body_m'], [.075, 0, .031])
    rotation = _rotation(np.array(camera['orientation_body_from_camera_wxyz']))
    # Renderer +X must be perpendicular to the end face, parallel to grip -Z.
    assert np.allclose(rotation[:, 0], [0, 0, -1], atol=1e-9)
    # Keep image roll: camera right is gripper -Y and camera up is gripper +X.
    assert np.allclose(rotation[:, 2], [1, 0, 0], atol=1e-9)
