import json
from pathlib import Path

import pytest

from space_arm_platform.imported_scene_library import digest, model_for_instance
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager


def install_fixture(root):
    model = root/'fixture.xml'
    model.write_text('<mujoco/>', encoding='utf-8')
    receipt = root/'receipt.json'
    receipt.write_text('{}', encoding='utf-8')
    catalog = root/'catalog.json'
    catalog.write_text('{}', encoding='utf-8')
    initial = dict(seed=42, randomization=dict(
        arm_joint_position_rad=[0., -.4, .6, 0., -.2, 0., 0., 0.],
        target_position_m=[.45,-.55,-.12], target_orientation_wxyz=[1.,0.,0.,0.],
        target_linear_velocity_m_s=[0.,0.,0.], target_angular_velocity_rad_s=[0.,0.,0.],
        preheld_capture_joint_position=list(range(12))),
        preheld_capture={'hold_gains':{'kp':2500.,'kd':80.,'torque_limit_nm':80.}})
    source = root/'source.json'
    source.write_text(json.dumps(dict(model=str(model), initial_scene=initial,
        dependencies=[dict(path=str(model),sha256=digest(model))])), encoding='utf-8')
    entry = dict(id='library/02_docked_operation/example',label='Imported fixture',
        description='Fixture',kind='operation',base_template_id='sarm-dual-capture26',
        source_instance=str(source),source_instance_sha256=digest(source),
        receipt=str(receipt),receipt_sha256=digest(receipt),
        catalog=str(catalog),catalog_sha256=digest(catalog))
    (root/'run').mkdir()
    (root/'run/imported_scene_library.json').write_text(json.dumps(
        dict(schema='space-arm-platform-library/1',entries=[entry])),encoding='utf-8')
    return entry, initial, model


def test_imported_scene_uses_normal_platform_instance_and_saved_dual_initial_state(tmp_path):
    entry, initial, model = install_fixture(tmp_path)
    manager = SceneRuntimeManager(None,project_root=tmp_path)
    assert any(t['id']==entry['id'] for t in manager.catalog()['templates'])
    instance = manager.create_instance(SceneInstanceCreate(template_id=entry['id'],dataset_capture=False))
    assert instance['template_id']=='sarm-dual-capture26'
    assert instance['randomization']==initial['randomization']
    assert instance['preheld_capture']==initial['preheld_capture']
    assert instance['randomize_overview_camera'] is False
    assert instance['render_profile']['camera_ids']==[
        'teleop/camera/spacecraft_overview','teleop/camera/sarm_wrist_cam','teleop/camera/capture_wrist_cam']
    assert model_for_instance(instance)==model
    assert Path(instance['config_path']).is_file()


def test_imported_scene_rejects_model_changed_after_registration(tmp_path):
    entry, _, model = install_fixture(tmp_path)
    manager = SceneRuntimeManager(None,project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(template_id=entry['id'],dataset_capture=False))
    model.write_text('<mujoco model="changed"/>',encoding='utf-8')
    with pytest.raises(ValueError,match='dependency changed'):
        model_for_instance(instance)


def test_imported_scene_rejects_changed_camera_baseline(tmp_path):
    from space_arm_platform.imported_scene_library import verified_source
    entry, _, _ = install_fixture(tmp_path)
    baseline = tmp_path/'baseline.xml'
    baseline.write_text('<mujoco><camera pos="0 0 1"/></mujoco>',encoding='utf-8')
    entry.update(camera_baseline=str(baseline),camera_baseline_sha256=digest(baseline))
    verified_source(entry)
    baseline.write_text('<mujoco><camera pos="0 0 2"/></mujoco>',encoding='utf-8')
    with pytest.raises(ValueError,match='camera_baseline'):
        verified_source(entry)


def test_imported_protected_face_profile_preserves_camera_settings(tmp_path):
    from space_arm_platform.imported_scene_library import apply_instance
    from copy import deepcopy
    entry, _, _ = install_fixture(tmp_path)
    path=Path(entry['source_instance']);source=json.loads(path.read_text())
    root=Path(__file__).resolve().parents[1]
    override=json.loads((root/'config/visual_attachments/protected_capture_face.json').read_text())
    source['initial_scene']['render_profile']=override
    path.write_text(json.dumps(source));entry['source_instance_sha256']=digest(path)
    profile=json.loads((root/'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
    original=deepcopy(profile)
    instance=dict(render_profile=profile,capture_target={})
    apply_instance(instance,entry)
    result=instance['render_profile']
    assert result['visual_attachments']==override['visual_attachments']
    assert result['vision_capture']=={**original['vision_capture'],'fiducial_calibration_valid':False}
    assert result['camera_ids']==original['camera_ids']
    assert result['camera_mounts']==original['camera_mounts']
    assert profile==original


def test_imported_attachment_override_cannot_change_cameras(tmp_path):
    from space_arm_platform.imported_scene_library import apply_instance
    entry, _, _=install_fixture(tmp_path)
    path=Path(entry['source_instance']);source=json.loads(path.read_text())
    source['initial_scene']['render_profile']={'camera_mounts':{}}
    path.write_text(json.dumps(source));entry['source_instance_sha256']=digest(path)
    with pytest.raises(ValueError,match='only select visual attachments'):
        apply_instance(dict(capture_target={}),entry)


def test_imported_rendezvous_uses_existing_live_flight_executor(tmp_path, monkeypatch):
    import asyncio
    from types import SimpleNamespace
    from unittest.mock import AsyncMock
    from space_arm_platform.mission_runtime import MissionRuntime
    entry, _, _ = install_fixture(tmp_path)
    entry.update(id='library/03_rendezvous/rendezvous',kind='rendezvous',flight_options={'seed':7})
    (tmp_path/'run/imported_scene_library.json').write_text(json.dumps(
        dict(schema='space-arm-platform-library/1',entries=[entry])),encoding='utf-8')
    manager = object.__new__(MissionRuntime)
    manager.scenes = SimpleNamespace(project_root=tmp_path)
    manager.start_flight = AsyncMock(return_value={'stage':'flight'})
    result=asyncio.run(manager.start_scene(SceneInstanceCreate(template_id=entry['id']),{'user_id':'local'}))
    assert result=={'stage':'flight'}
    manager.start_flight.assert_awaited_once_with({'user_id':'local'},scenario='team-in-orbit',
        options={'seed':7},live=True,mode='simulation',background=False,continuous=False)
