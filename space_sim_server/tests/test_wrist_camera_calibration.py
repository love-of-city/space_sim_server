"""The native UE image and PnP must share the requested wrist optics."""
import math
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from space_arm_platform.wrist_vision_config import build_wrist_config
from simulation.stereo_cameras import configure_wrist_capture

ROOT = Path(__file__).resolve().parents[1]


def test_live_wrist_config_uses_100_degree_axes_with_existing_gripper_mount():
    config = build_wrist_config(ROOT)
    assert config['image_size'] == [1920, 1200]
    tangent=math.tan(math.radians(50))
    np.testing.assert_allclose(config['K'], [[960/tangent, 0, 959.5],
                                           [0, 600/tangent, 599.5], [0, 0, 1]])
    assert math.isclose(config['horizontal_fov_rad'],math.radians(100))
    assert math.isclose(config['vertical_fov_rad'],math.radians(100))
    assert config['capture_rate_hz'] == 10
    assert config['exposure_time_s'] == .002
    np.testing.assert_allclose(config['mount']['camera_position_grip_m'], [.075, 0, .031])
    assert config['camera_id'] == 'teleop/camera/capture_wrist_cam'


def test_native_capture_and_visible_wrist_share_optics_without_moving_any_camera():
    @dataclass
    class Camera:
        camera_id: str
        parent_id: str = 'teleop/gripper'
        position_body_m: tuple = (.075, 0, .031)
        orientation_body_from_camera_wxyz: tuple = (.5, .5, .5, .5)
        resolution: tuple = (1024, 1024)
        field_of_view_rad: float = math.radians(100)
        vertical_field_of_view_rad: float = 0.
        capture_consumer: str = 'dataset'
        capture_products: tuple = ('rgb',)
        display_name: str = 'wrist'
        picture_in_picture: bool = True
        capture_rate_hz: float = 50.
        exposure_time_s: float = 0.

    wrist = Camera('teleop/camera/capture_wrist_cam')
    overview = Camera('teleop/camera/spacecraft_overview', parent_id='teleop/cubesat_bus')
    before_overview = dict(overview.__dict__)
    before_mount = (wrist.parent_id, wrist.position_body_m, wrist.orientation_body_from_camera_wxyz)
    added = []
    config = build_wrist_config(ROOT)
    configure_wrist_capture(SimpleNamespace(_cameras=[overview, wrist], add_camera=added.append),
                            True, calibration=config)
    assert overview.__dict__ == before_overview
    assert len(added) == 1
    for camera in (wrist, added[0]):
        assert (camera.parent_id, camera.position_body_m, camera.orientation_body_from_camera_wxyz) == before_mount
        assert tuple(camera.resolution) == (1920, 1200)
        assert math.isclose(1920/(2*math.tan(camera.field_of_view_rad/2)), config['K'][0][0])
        assert math.isclose(1200/(2*math.tan(camera.vertical_field_of_view_rad/2)), config['K'][1][1])
        assert math.isclose(camera.vertical_field_of_view_rad,math.radians(100))
        assert camera.exposure_time_s == .002
    assert added[0].capture_rate_hz == 10
    assert added[0].capture_consumer == 'vision'
    assert not added[0].picture_in_picture


def test_optional_vertical_fov_survives_render_protocol_and_defaults_to_legacy():
    from bsk_render_adapter.descriptors import CameraVisual
    legacy=CameraVisual('overview').to_payload()
    assert legacy['vertical_field_of_view_rad']==0
    custom=CameraVisual('wrist',field_of_view_rad=math.radians(100),
                        vertical_field_of_view_rad=math.radians(100),resolution=(1920,1200))
    assert math.isclose(custom.to_payload()['vertical_field_of_view_rad'],math.radians(100))


@pytest.mark.parametrize('mode', ['rgb', 'monochrome'])
def test_independent_projection_preserves_main_camera_modes(mode):
    from bsk_render_adapter.descriptors import CameraVisual
    payload = CameraVisual('wrist', camera_mode=mode,
        vertical_field_of_view_rad=math.radians(100)).to_payload()
    assert payload['camera_mode'] == mode


@pytest.mark.parametrize('value', [-1, math.pi, math.inf, math.nan, True, '100'])
def test_invalid_vertical_fov_is_rejected(value):
    from bsk_render_adapter.descriptors import CameraVisual
    with pytest.raises(ValueError, match='vertical_field_of_view_rad'):
        CameraVisual('wrist', vertical_field_of_view_rad=value).to_payload()


@pytest.mark.parametrize('mode', ['fisheye', 'monochrome_fisheye'])
def test_fisheye_keeps_legacy_model_and_rejects_independent_pinhole_axes(mode):
    from bsk_render_adapter.descriptors import CameraVisual
    assert CameraVisual('eye', camera_mode=mode).to_payload()['camera_mode'] == mode
    with pytest.raises(ValueError, match='fisheye'):
        CameraVisual('eye', camera_mode=mode, vertical_field_of_view_rad=1).to_payload()


@pytest.mark.parametrize('field,value', [
    ('resolution_wh', [1920.5, 1200]), ('K', [[1,0,0],[0,1,0],[0,0,1]]),
    ('distortion_coefficients', [0.1]), ('capture_rate_hz', float('nan')),
    ('max_exposure_ms_requirement', -1), ('fov_xy_deg', [58,38]),
])
def test_bad_calibration_fails_before_rendering(tmp_path, field, value):
    import json
    source = ROOT / 'config/vision/capture_wrist_camera.json'
    optics = json.loads(source.read_text())
    optics[field] = value
    destination = tmp_path / 'config/vision/capture_wrist_camera.json'
    destination.parent.mkdir(parents=True)
    destination.write_text(json.dumps(optics))
    with pytest.raises(ValueError, match='wrist'):
        build_wrist_config(tmp_path)


@pytest.mark.parametrize('mode', ['rgb', 'fisheye', 'monochrome'])
def test_startup_applies_capture_calibration_and_preserves_operation_arm_modes(monkeypatch, mode):
    import copy
    import importlib.util
    import sys
    from bsk_render_adapter import CameraVisual, VisualElement
    monkeypatch.setitem(sys.modules, 'simulation.assembly', SimpleNamespace(
        AssemblyContext=object, Slot=SimpleNamespace(OUTPUT=0)))
    spec = importlib.util.spec_from_file_location('capture_optics_render_probe',
        ROOT / 'simulation/components/render.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    capture = CameraVisual('teleop/camera/capture_wrist_cam', parent_id='teleop/gripper',
        position_body_m=(.075,0,.031), resolution=(1024,1024), picture_in_picture=True)
    operation = CameraVisual('teleop/camera/sarm_wrist_cam', display_name='Operation')
    overview = CameraVisual('teleop/camera/spacecraft_overview')
    before_overview = copy.deepcopy(overview)
    bridge = SimpleNamespace(_cameras=[capture,operation,overview], _objects=[],
        add_mj_scene=lambda *a, **kw: None, set_scene_settings=lambda *a: None,
        add_lights=lambda *a: None)
    bridge.add_camera=bridge._cameras.append
    monkeypatch.setitem(sys.modules, 'bsk_render_adapter', SimpleNamespace(
        BasiliskRenderBridge=lambda **kw: bridge, SceneSettings=SimpleNamespace,
        VisualElement=VisualElement))
    monkeypatch.setitem(sys.modules, 'simulation.teleop_grasp_unreal', SimpleNamespace(
        _register_celestial_bodies=lambda *a: None))
    ctx=SimpleNamespace(scene=object(),extra={'template_id':'sarm-dual-capture26'},
        add=lambda *a, **kw: None,keep_alive=lambda *a: None)
    module.RenderComponent(model_path='unused',catalog={},host='localhost',port=5558,
        render_rate_hz=50,capture_rate_hz=30,capture_state_provider=None,
        wrist_camera_mode=mode, vision_capture={'enabled':True,'stereo_enabled':False,
            'wrist_alignment_enabled':True}).install(ctx)
    assert overview == before_overview
    assert operation.camera_mode == mode
    for camera in (capture,bridge._cameras[-1]):
        assert camera.resolution == (1920,1200)
        assert camera.field_of_view_rad == pytest.approx(math.radians(100))
        assert camera.vertical_field_of_view_rad == pytest.approx(math.radians(100))
        assert camera.camera_mode == 'rgb'
        assert camera.parent_id == 'teleop/gripper'
    assert capture.capture_consumer == 'dataset'
    assert bridge._cameras[-1].capture_consumer == 'vision'


@pytest.mark.parametrize('size,accepted', [([63,1200],False),([4097,1200],False),
    ([1920,63],False),([1920,4097],False),([64,64],True),([4096,4096],True)])
def test_calibration_respects_ue_raster_limits(tmp_path, monkeypatch, size, accepted):
    import json
    import space_arm_platform.wrist_vision_config as config_module
    optics=json.loads((ROOT/'config/vision/capture_wrist_camera.json').read_text())
    w,h=size
    optics['resolution_wh']=size
    optics['K']=[[w/(2*math.tan(math.radians(50))),0,(w-1)/2],
                 [0,h/(2*math.tan(math.radians(50))),(h-1)/2],[0,0,1]]
    source=tmp_path/'config/vision/capture_wrist_camera.json'
    source.parent.mkdir(parents=True)
    source.write_text(json.dumps(optics))
    rig=tmp_path/'config/visual_attachments/target_aruco_flush_5cm_v2/aruco_rig.json'
    rig.parent.mkdir(parents=True);rig.write_text('{}')
    monkeypatch.setattr(config_module,'load_capture_mount',lambda _: {})
    if accepted:
        assert build_wrist_config(tmp_path)['image_size']==size
    else:
        with pytest.raises(ValueError,match='resolution'):
            build_wrist_config(tmp_path)
