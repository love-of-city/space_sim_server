import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from simulation.stereo_cameras import load_stereo_setup, register_stereo_cameras
from simulation.stereo_cameras import configure_wrist_capture


def test_stereo_off_does_not_read_or_register_anything(tmp_path):
    assert load_stereo_setup(tmp_path, None) is None
    register_stereo_cameras(None, None)
    assert load_stereo_setup(tmp_path,{'enabled':True,'stereo_enabled':False,'config_file':'not-used.json'}) is None


def test_wrist_vision_changes_only_capture_routing_not_camera_geometry():
    import copy
    from dataclasses import dataclass
    @dataclass
    class Camera:
        camera_id:str
        parent_id:str='teleop/gripper'
        position_body_m:tuple=(.075,0,.031)
        resolution:tuple=(1024,1024)
        field_of_view_rad:float=math.radians(100)
        orientation_body_from_camera_wxyz:tuple=(.5,.5,.5,.5)
        capture_consumer:str='dataset'
        capture_products:tuple=('rgb',)
        display_name:str='wrist'
        picture_in_picture:bool=True
        capture_rate_hz:float=50.
    wrist=Camera(camera_id='teleop/camera/capture_wrist_cam',
        parent_id='teleop/gripper',position_body_m=(.075,0,.031),resolution=(1024,1024),
        field_of_view_rad=math.radians(100),orientation_body_from_camera_wxyz=(.5,.5,.5,.5),
        capture_consumer='dataset',capture_products=('rgb',))
    old=SimpleNamespace(camera_id='overview',capture_consumer='dataset')
    before=copy.deepcopy(wrist.__dict__)
    added=[]
    configure_wrist_capture(SimpleNamespace(_cameras=[old,wrist],add_camera=added.append),True)
    assert wrist.__dict__==before and old.capture_consumer=='dataset'
    assert len(added)==1 and added[0].capture_consumer=='vision' and not added[0].picture_in_picture
    for key,value in before.items():
        if key in ('parent_id','position_body_m','orientation_body_from_camera_wxyz','resolution','field_of_view_rad'):
            assert getattr(added[0],key)==value


def test_full_resolution_cameras_are_added_without_changing_old_camera(tmp_path, monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'bsk_render_adapter', SimpleNamespace(CameraVisual=SimpleNamespace))
    old = SimpleNamespace(camera_id='overview', position_body_m=(1, 2, 3))
    cameras = [old]
    rows = [dict(camera_id=eye, parent_id='teleop/cubesat_bus', capture_consumer='vision',
                 resolution=[1920, 1200], capture_rate_hz=10, picture_in_picture=False)
            for eye in ('left', 'right')]
    path = tmp_path / 'stereo.json'
    path.write_text(json.dumps({'camera_ids': {'left': 'left', 'right': 'right'}, 'cameras': rows}))
    config = load_stereo_setup(tmp_path, {'enabled': True, 'config_file': 'stereo.json'})
    register_stereo_cameras(SimpleNamespace(add_camera=cameras.append), config)
    assert cameras[0] is old and old.position_body_m == (1, 2, 3)
    assert len(cameras) == 3
    assert cameras[1].resolution == [1920, 1200]
    assert not cameras[1].picture_in_picture


def test_stereo_config_cannot_escape_root(tmp_path):
    with pytest.raises(ValueError, match='inside'):
        load_stereo_setup(tmp_path, {'enabled': True, 'config_file': '../secret.json'})


def test_active_optics_match_calibration_and_horizontal_12cm_baseline():
    root=Path(__file__).resolve().parents[1]
    config=load_stereo_setup(root, {'enabled': True, 'config_file': 'config/vision/capture26_stereo.json'})
    left,right=config['cameras']
    delta=[a-b for a,b in zip(left['position_body_m'],right['position_body_m'])]
    assert math.isclose(math.sqrt(sum(v*v for v in delta)), .12, abs_tol=1e-12)
    assert delta[2]==0
    assert left['orientation_body_from_camera_wxyz']==right['orientation_body_from_camera_wxyz']
    for camera in config['cameras']:
        fx=camera['resolution'][0]/(2*math.tan(camera['field_of_view_rad']/2))
        assert math.isclose(fx,config['calibration']['K_left'][0][0],abs_tol=1e-9)
        assert camera['capture_rate_hz']==10 and camera['exposure_time_s']<=.002
        assert camera['capture_consumer']=='vision' and not camera['picture_in_picture']
