"""Configuration rejection and static kinematics regression checks."""
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]


def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def profile(tmp_path):
    source=tmp_path/'source.xml';source.write_text('<mujoco/>')
    data=dict(schema='target-taskbox-profile/1',source_model=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),scale=dict(value=1,max_value=2),constraints=dict(taskbox_world_pose='fixed'))
    path=tmp_path/'profile.json';path.write_text(json.dumps(data));return source,path,data


def test_profile_rejects_changed_source(tmp_path):
    source,path,_=profile(tmp_path);source.write_text('<mujoco model="changed"/>')
    with pytest.raises(ValueError,match='Source asset changed'):module('build_target_asset').load_profile(path)


def test_profile_rejects_unreviewed_scale(tmp_path):
    _,path,data=profile(tmp_path);data['scale']['value']=3;path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='Scale exceeds'):module('build_target_asset').load_profile(path)


def test_profile_rejects_moving_taskbox(tmp_path):
    _,path,data=profile(tmp_path);data['constraints']['taskbox_world_pose']='movable';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='fixed task'):module('build_target_asset').load_profile(path)


def test_hinge_rotates_about_authored_anchor():
    root=ET.fromstring('<mujoco><worldbody><body name="arm"><joint name="joint1" axis="0 0 1" pos="1 0 0"/><body name="tip" pos="2 0 0"/></body></worldbody></mujoco>')
    poses=module('export_asset_xml_preview').kinematics(root,dict(arm_joint_position_rad=[np.pi/2]))
    assert np.allclose(poses['tip'][0],[1,1,0])


def test_slide_ref_and_free_target_pose():
    root=ET.fromstring('<mujoco><worldbody><body name="arm"><joint name="joint1" type="slide" axis="1 0 0" ref="0.2"/></body><body name="capture_target" pos="9 9 9"><freejoint/></body></worldbody></mujoco>')
    poses=module('export_asset_xml_preview').kinematics(root,dict(arm_joint_position_rad=[.3],target_position_m=[1,2,3],target_orientation_wxyz=[1,0,0,0]))
    assert np.allclose(poses['arm'][0],[.1,0,0]);assert np.allclose(poses['capture_target'][0],[1,2,3])


def test_geometry_attributes_follow_body_order_not_xml_interleaving():
    root=ET.fromstring('<mujoco><worldbody><body name="parent"><body name="child"><geom name="small" mesh="small" size="0.1"/></body><geom name="large" mesh="large" size="2"/></body></worldbody></mujoco>')
    metadata=[SimpleNamespace(body_name='parent',mesh_name='large'),SimpleNamespace(body_name='child',mesh_name='small')]
    geoms=module('export_asset_xml_preview').ordered_geometries(root,metadata)
    assert [g.get('size') for g in geoms]==['2','0.1']
    with pytest.raises(ValueError,match='identity mismatch'):
        module('export_asset_xml_preview').ordered_geometries(root,list(reversed(metadata)))
