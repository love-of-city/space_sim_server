"""The protected four plates retain original dimensions and body-local coordinates."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.serial_chain_kinematics import axis_angle_to_matrix, matrix_to_quaternion_wxyz
from simulation.visual_attachments import attach_visual_geometries, attachment_transform

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'config/visual_attachments/protected_capture_face.json'


def spec():
    result=json.loads(CONFIG.read_text(encoding='utf-8'))['visual_attachments'][0]
    # Optional generic anchoring remains available to unrelated attachments;
    # it must never be selected by the protected four-marker configuration.
    result['transform_mode']='mjcf_anchor'
    result['mjcf_anchor']=dict(geom='target_instance_0006_part_001_color_00',
        mesh_sha256='7e5aefc36b541fbdd21c1d3c25c6d74a88e9d2fb5b136bc350dadfb66463ba3c',
        reference_position_body_m=[.1755,.05625,.04707870013],
        reference_orientation_wxyz=[1.,0.,0.,0.],reference_mesh_scale=[1.,1.,1.])
    return result


def model(tmp_path, factor, rotation=np.eye(3), shift=np.zeros(3)):
    anchor=spec()['mjcf_anchor']
    root=ET.Element('mujoco');asset=ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='hull',
        file=str(ROOT/'model/ground_validation_satellite/meshes/part_001_color_00.obj'),
        scale=' '.join(map(str, np.broadcast_to(factor, (3,)))))
    body=ET.SubElement(ET.SubElement(root, 'worldbody'), 'body', name='capture_target')
    ET.SubElement(body, 'geom', name=anchor['geom'], type='mesh', mesh='hull',
        pos=' '.join(map(str, shift+rotation@(np.asarray(anchor['reference_position_body_m'])*factor))),
        quat=' '.join(map(str, matrix_to_quaternion_wxyz(rotation))))
    path=tmp_path/'model.xml';ET.ElementTree(root).write(path)
    return path


@pytest.mark.parametrize('factor', [1., 2.7, 3., 4.])
@pytest.mark.parametrize('angle', [0., .6])
def test_all_original_boards_and_pixels_follow_one_rigid_uniform_transform(tmp_path, factor, angle):
    rotation=axis_angle_to_matrix(np.array([0., 0., 1.]), angle);shift=np.array([.2, -.1, .05])
    path=model(tmp_path, factor, rotation, shift);original_model=path.read_bytes()
    attachment=spec();original_spec=deepcopy(attachment)
    source=json.loads((ROOT/attachment['geometry_file']).read_text())
    target=SimpleNamespace(object_id=attachment['parent_id'], geometries=[]);contracts=[]
    count=attach_visual_geometries([target], [attachment], ROOT, model_path=path, contracts=contracts)
    assert count==260
    assert len([g for g in target.geometries if g.geometry_id.endswith('/mount')])==4
    for row, geometry in zip(source, target.geometries, strict=True):
        np.testing.assert_allclose(geometry.position_body_m, shift+factor*(rotation@row['position_body_m']), atol=1e-12)
        np.testing.assert_allclose(geometry.dimensions_m, factor*np.asarray(row['dimensions_m']), atol=1e-12)
        assert geometry.color_rgba==row['color_rgba']
        assert geometry.material_texture==row['material_texture']
        assert geometry.geometry_id==row['geometry_id']
        assert geometry.render_role=='visual'
    assert contracts[0]['uniform_scale']==factor
    assert contracts[0]['model_sha256']==hashlib.sha256(original_model).hexdigest()
    assert path.read_bytes()==original_model and attachment==original_spec


def test_protected_four_marker_centers_and_sizes_ignore_hull_scale_and_pose(tmp_path):
    attachment=json.loads(CONFIG.read_text())['visual_attachments'][0]
    assert attachment['transform_mode']=='fixed_body_geometry' and 'mjcf_anchor' not in attachment
    offset,rotation,factor=attachment_transform(attachment, model(tmp_path, 3.,
        axis_angle_to_matrix(np.array([0.,0.,1.]),.7),np.array([1.,2.,3.])))
    rig=json.loads((ROOT/'config/visual_attachments/target_aruco_5cm/aruco_rig.json').read_text())
    centers=[(offset+factor*(rotation@np.asarray(marker['center_m']))).tolist() for marker in rig['markers']]
    np.testing.assert_array_equal(centers, [[.1,.0578,.15],[.25,.0578,.15],[.1,.0578,-.13],[.25,.0578,-.13]])
    assert factor==1. and all(factor*marker['size_m']==.05 for marker in rig['markers'])


@pytest.mark.parametrize('factor', [[3.,2.,4.], 1., 3., 4.])
def test_fixed_protected_geometry_is_exactly_the_original_260_records(tmp_path, factor):
    attachment=json.loads(CONFIG.read_text())['visual_attachments'][0]
    path=model(tmp_path,factor,axis_angle_to_matrix(np.array([1.,0.,0.]),.6),np.ones(3))
    original=json.loads((ROOT/attachment['geometry_file']).read_text())
    target=SimpleNamespace(object_id=attachment['parent_id'],geometries=[]);contracts=[]
    attach_visual_geometries([target],[attachment],ROOT,model_path=path,contracts=contracts)
    assert [geometry.to_payload() for geometry in target.geometries]==original
    assert contracts[0]['transform_mode']=='fixed_body_geometry'
    assert contracts[0]['uniform_scale']==1. and contracts[0]['anchor_geom'] is None
    assert len([g for g in target.geometries if g.geometry_id.endswith('/mount')])==4


def test_fixed_geometry_forbids_accidental_hull_anchoring():
    attachment=spec();attachment['transform_mode']='fixed_body_geometry'
    with pytest.raises(ValueError,match='forbids hull scale or pose'):
        attachment_transform(attachment)


def test_fixed_geometry_does_not_require_or_read_a_satellite_scale():
    attachment=json.loads(CONFIG.read_text())['visual_attachments'][0]
    offset,rotation,factor=attachment_transform(attachment,Path('missing-nonuniform-pivot-scaled-model.xml'))
    np.testing.assert_array_equal(offset,np.zeros(3))
    np.testing.assert_array_equal(rotation,np.eye(3))
    assert factor==1.


@pytest.mark.parametrize('factor', [[3., 2., 3.], [-1., -1., -1.], [0., 0., 0.]])
def test_nonuniform_reflected_or_zero_scale_is_rejected(tmp_path, factor):
    with pytest.raises(ValueError):attachment_transform(spec(), model(tmp_path, factor))


def test_anchor_requires_runtime_model_and_matching_protected_hull(tmp_path):
    with pytest.raises(ValueError, match='runtime model'):attachment_transform(spec())
    path=model(tmp_path, 3.);changed=spec();changed['mjcf_anchor']['mesh_sha256']='0'*64
    with pytest.raises(ValueError, match='protected face'):attachment_transform(changed, path)
    changed=spec();changed['parent_id']='teleop/cubesat_bus'
    with pytest.raises(ValueError, match='parent body'):attachment_transform(changed, path)


def test_unscaled_old_profiles_keep_their_existing_coordinates():
    profile=json.loads((ROOT/'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
    assert 'mjcf_anchor' not in profile['visual_attachments'][0]
    offset,rotation,factor=attachment_transform(profile['visual_attachments'][0])
    np.testing.assert_array_equal(offset, np.zeros(3))
    np.testing.assert_array_equal(rotation, np.eye(3))
    assert factor==1.


def test_invalid_rig_suppresses_only_algorithm_triggers_without_mutating_cameras():
    from simulation.stereo_cameras import load_stereo_setup, vision_trigger_ids
    profile=json.loads((ROOT/'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
    options=deepcopy(profile['vision_capture']);before=deepcopy(options)
    stereo=load_stereo_setup(ROOT, options);cameras=deepcopy(stereo['cameras'])
    original=vision_trigger_ids(stereo, True, options)
    assert len(original)==3
    options['fiducial_calibration_valid']=False
    assert vision_trigger_ids(stereo, True, options)==[]
    assert load_stereo_setup(ROOT, options)['cameras']==cameras
    options.pop('fiducial_calibration_valid')
    assert options==before and vision_trigger_ids(stereo, True, options)==original
