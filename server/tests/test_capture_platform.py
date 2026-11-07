"""Ensure original environment and optics survive the capture task adapter."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest


def test_capture_defaults_match_platform_except_marker_and_solver():
    from collection.capture_initial_pool import source_scene
    from space_arm_platform.models import SceneInstanceCreate
    from space_arm_platform.scene_runtime import _sample_instance, SCENE_RENDER_PROFILES
    from simulation.stereo_cameras import load_stereo_setup, vision_trigger_ids
    from pathlib import Path
    original = _sample_instance(SceneInstanceCreate(template_id='sarm-dual-capture26'), 7)
    scene = source_scene(7)
    assert scene['environment'] == original['environment']
    assert scene['environment']['physics']['earth_gravity'] == 'j2'
    assert scene['environment']['physics']['drag'] is None
    profile = scene['render_profile']
    assert profile['camera_mounts'] == SCENE_RENDER_PROFILES['sarm-dual-capture26']['camera_mounts']
    assert profile['visual_attachments'] == []
    stereo = load_stereo_setup(Path(__file__).resolve().parents[1], profile['vision_capture'])
    assert len(stereo['cameras']) == 2
    assert vision_trigger_ids(stereo, False, profile['vision_capture']) == []


def test_extra_cameras_keep_original_mounts_and_record_only_new_three():
    from bsk_render_adapter import CameraVisual
    from simulation.scripted_tasks.capture_cameras import configure_capture_cameras, CAMERAS
    cameras = [CameraVisual(camera_id='original', position_body_m=(.1,.2,.3),
                            resolution=(1920,1200), capture_products=('rgb',))]
    before = deepcopy(cameras[0].to_payload())
    lights = []
    bridge = SimpleNamespace(_cameras=cameras, add_camera=cameras.append, add_lights=lights.append)
    configure_capture_cameras(bridge, [.3,-.44,.32])
    assert len(cameras) == 4
    after = cameras[0].to_payload()
    assert after == {**before, 'capture_consumer': 'vision'}
    assert [c.camera_id for c in cameras if c.capture_consumer=='dataset'] == CAMERAS
    assert cameras[-1].position_body_m == (.14,-.14,.025)
    assert not bridge.vision_capture_camera_ids
    assert [light.parent_id for light in lights] == [c.parent_id for c in cameras[1:]]
    # Moving the global view must preserve the previously reviewed scene lighting.
    assert cameras[1].position_body_m == (1.8,-2.,1.15)
    assert lights[0].position_body_m == (2.4,-2.4,1.5)
    old_lamp_forward = np.array([.9,0.,.1])-np.array([2.4,-2.4,1.5])
    np.testing.assert_allclose(lights[0].normal_body,
        old_lamp_forward/np.linalg.norm(old_lamp_forward))
    for light,camera in zip(lights[1:],cameras[2:]):
        assert light.position_body_m == camera.position_body_m
        from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
        np.testing.assert_allclose(light.normal_body,
            quaternion_wxyz_to_matrix(camera.orientation_body_from_camera_wxyz)[:,0])
    assert len({light.visual_id for light in lights}) == 3
    assert all(light.kind == 'light' for light in lights)
    assert [light.range_m for light in lights] == [12.,2.,2.]
    assert all(camera.exposure_time_s == 0 for camera in cameras[1:])
    assert [camera.aperture_fnumber for camera in cameras[1:]] == [8.,0.,0.]


def test_capture_scene_keeps_non_marker_attachments_and_saved_sensors(monkeypatch):
    from collection.capture_initial_pool import source_scene
    from space_arm_platform import scene_runtime
    sample = scene_runtime._sample_instance
    def sample_with_sensors(*args, **kwargs):
        scene = sample(*args, **kwargs)
        scene['sensors'] = {'service_gyro': {'enabled': False}, 'other_sensor': {'enabled': True}}
        return scene
    monkeypatch.setattr(scene_runtime, '_sample_instance', sample_with_sensors)
    profile = deepcopy(scene_runtime.SCENE_RENDER_PROFILES['sarm-dual-capture26'])
    retained = {'geometry_file': 'model/thermal_blanket.xml', 'body': 'capture_target'}
    marker = {'geometry_file': 'model/target_aruco_markers.xml', 'body': 'capture_target'}
    profile['visual_attachments'] = [retained, marker]
    monkeypatch.setitem(scene_runtime.SCENE_RENDER_PROFILES, 'sarm-dual-capture26', profile)
    scene = source_scene(7)
    assert scene['render_profile']['visual_attachments'] == [retained]
    assert scene['sensors'] == {'service_gyro': {'enabled': False}, 'other_sensor': {'enabled': True}}
    assert profile['visual_attachments'] == [retained, marker]


def test_main_renderer_requires_original_assets_without_saved_map(tmp_path, monkeypatch):
    from space_arm_platform import collection_runtime as runtime
    from simulation.scripted_tasks import capture_platform
    args = SimpleNamespace(adapter=tmp_path, unreal=tmp_path, render_port=25002,
        capture_port=25001, offline=True, render_environment='platform_main')
    monkeypatch.setattr(runtime, 'ensure_environment_level', lambda *a: pytest.fail('wrong branch map'))
    monkeypatch.setattr(runtime, 'unreal_editor', lambda _: tmp_path/'UnrealEditor')
    with pytest.raises(FileNotFoundError, match='Original main'):
        runtime.renderer_command(args,tmp_path)
    monkeypatch.setattr(capture_platform, 'verify_main_assets', lambda _: [])
    command=runtime.renderer_command(args,tmp_path)
    assert '/Engine/Maps/Entry' in command
    assert f'-BskConfig={capture_platform.RENDER_CONFIG}' in command


def test_capture_render_catalog_rejects_missing_gripper_or_pointer(tmp_path):
    import json
    from simulation.scripted_tasks.capture_platform import verify_capture_render_assets
    catalog = tmp_path/'catalog.json'
    catalog.write_text(json.dumps({'assets': {'jaw.stl': {
        'asset_path': '/Game/BSK/Generated/Capture26/jaw.jaw'}}}))
    asset = tmp_path/'Unreal/BskUnrealRenderer/Content/BSK/Generated/Capture26/jaw.uasset'
    with pytest.raises(FileNotFoundError, match='jaw.uasset'):
        verify_capture_render_assets(tmp_path, catalog)
    asset.parent.mkdir(parents=True)
    asset.write_bytes(b'version https://git-lfs.github.com/spec/v1\n')
    with pytest.raises(FileNotFoundError, match='jaw.uasset'):
        verify_capture_render_assets(tmp_path, catalog)
    asset.write_bytes(b'\xc1\x83\x2a\x9e')
    assert len(verify_capture_render_assets(tmp_path, catalog)) == 1


def test_sampling_cones_are_joint_constraints_not_separate_axis_boxes():
    from collection.capture_initial_domain import contains_initial
    bounds=dict(gap_range_m=[.3,.7],bearing_cone_deg=10.,tilt_cone_deg=10.,roll_range_deg=[-5.,5.])
    assert contains_initial(dict(gap_m=.7,bearing_x_deg=10.,tilt_y_deg=-10.),bounds)
    assert not contains_initial(dict(gap_m=.7,bearing_x_deg=10.,bearing_y_deg=10.),bounds)
    assert not contains_initial(dict(gap_m=.3,tilt_x_deg=10.,tilt_y_deg=10.),bounds)
    assert not contains_initial(dict(gap_m=.29),bounds)


def test_recording_boundary_checks_all_selected_camera_counts():
    from collection.record_scripted_task import validate_episode
    result=dict(dataset_status='complete', dataset_frame_count=4, step_count=4,
        dataset_camera_ids=['workspace','approach','jaws'],capture_count=12,
        incomplete_sample_count=0,rejected_capture_count=0)
    validate_episode(result)
    with pytest.raises(RuntimeError,match='frame counts'):
        validate_episode({**result,'capture_count':8})


def test_random_arm_postures_expand_spatial_coverage_and_obey_bounds():
    from tools.prepare_capture_initial_pool import sample_candidates
    from simulation.scripted_tasks.capture_grasp import CaptureGraspTask, PREPARED_ARM
    from collection.capture_initial_domain import contains_initial
    from pathlib import Path
    config=dict(schema='capture-initial-sampling/1',seed=27,sample_count=20,
        gap_range_m=[.3,.5],bearing_cone_deg=5.,tilt_cone_deg=5.,roll_range_deg=[-5.,5.],
        arm_joint_baseline_rad=PREPARED_ARM.tolist(),
        arm_joint_offset_range_deg=[[-90.,90.],[-40.,40.],[-20.,20.],[0.,0.],[0.,0.],[0.,0.]])
    states=sample_candidates(config)
    assert states==sample_candidates(config)
    assert all(contains_initial(state,config) for state in states)
    q=np.array([state['arm_joint_position_rad'] for state in states])
    assert np.ptp(q[:,0])>2. and np.ptp(q[:,1])>1.
    task=CaptureGraspTask(Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml',
                          initial_joints=q[0])
    np.testing.assert_allclose(task.reference[:6],q[0])
    assert not contains_initial({**states[0],'arm_joint_position_rad':None},config)
    with pytest.raises(ValueError,match='joint limits'):
        CaptureGraspTask(task.chain_path if hasattr(task,'chain_path') else
            Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml',initial_joints=[100.]*6)
