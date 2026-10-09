"""Surface partition and one-DOF wing model regression checks."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
np = pytest.importorskip('numpy')
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wing_builder', ROOT/'tools/build_satellite_wing_joint.py')
wing = importlib.util.module_from_spec(spec); spec.loader.exec_module(wing)
FOLDER = ROOT/'model/ground_validation_satellite'
OUTPUT = FOLDER/wing.OUTPUT_XML


def triangle(points):
    return np.column_stack((np.asarray(points,float),np.tile([0.,0,1],(3,1))))


@pytest.mark.parametrize('offset',[-.7,0,.25,.8])
def test_split_preserves_area_winding_and_plane(offset):
    original=np.array([triangle([[-1,0,0],[1,0,0],[.5,2,0]])])
    fixed,moving,info=wing.split_surface(original,[1,0,0],offset)
    assert abs(wing.area(fixed)+wing.area(moving)-wing.area(original))<1e-13
    assert fixed[:,:,:3][:,:,0].max()<=offset+1e-13
    assert moving[:,:,:3][:,:,0].min()>=offset-1e-13
    assert not info['generated_cut_caps'] and not info['mechanical_internals_reconstructed']
    for t in np.concatenate([fixed,moving]):
        assert np.cross(t[1,:3]-t[0,:3],t[2,:3]-t[0,:3])[2]>0


def test_coplanar_faces_not_duplicated():
    original=np.array([triangle([[-1,0,0],[1,0,0],[0,1,0]]),triangle([[0,0,0],[0,1,0],[0,0,1]])])
    fixed,moving,_=wing.split_surface(original,[1,0,0],0)
    assert wing.area(fixed)+wing.area(moving)==pytest.approx(wing.area(original))
    assert sum(np.all(abs(t[:,:3][:,0])<1e-13) for t in fixed)==1
    assert not any(np.all(abs(t[:,:3][:,0])<1e-13) for t in moving)


def test_nonunit_plane_equivalent():
    original=np.array([triangle([[-1,0,0],[1,0,0],[.5,2,0]])])
    a,b,_=wing.split_surface(original,[1,0,0],.2)
    c,d,_=wing.split_surface(original,[2,0,0],.4)
    np.testing.assert_allclose(a,c);np.testing.assert_allclose(b,d)


def test_obj_partition_roundtrip(tmp_path):
    original=np.array([triangle([[-1,0,0],[1,0,0],[.5,2,0]])])
    _,moving,_=wing.split_surface(original,[1,0,0],.23)
    path=tmp_path/'part.obj';path.write_text(wing.obj_text(moving),encoding='utf-8')
    np.testing.assert_allclose(wing.read_obj_triangles(path),moving,atol=1e-13)


def test_quaternion_and_axis_rotation_consistent():
    angle=.63
    q=[np.cos(angle/2),0,0,np.sin(angle/2)]
    np.testing.assert_allclose(wing.quat_matrix(q),wing.axis_rotation([0,0,1],angle),atol=1e-14)
    np.testing.assert_allclose(wing.quat_matrix([-x for x in q]),wing.quat_matrix(q))


@pytest.mark.skipif(not OUTPUT.exists(),reason='Generated wing model not present')
def test_model_single_joint_no_invented_limits_or_actuation():
    root=ET.parse(OUTPUT).getroot()
    joints=root.findall('.//joint');assert len(joints)==1
    assert joints[0].get('name')==wing.JOINT and joints[0].get('type')=='hinge'
    assert joints[0].get('limited')=='false' and joints[0].get('range') is None
    assert not root.findall('.//freejoint') and root.find('actuator') is None
    base=root.find('worldbody/body');inner=base.find(f"body[@name='{wing.INNER_BODY}']")
    outer=inner.find(f"body[@name='{wing.OUTER_BODY}']")
    assert inner.find('joint') is None and outer.find('joint') is not None
    assert outer.find('inertial').get('mass')=='1'
    assert 'NOT engineering data' in OUTPUT.read_text(encoding='utf-8')


@pytest.mark.skipif(not OUTPUT.exists(),reason='Generated wing model not present')
def test_attachments_and_visual_groups():
    root=ET.parse(OUTPUT).getroot()
    inner=root.find(f".//body[@name='{wing.INNER_BODY}']")
    outer=root.find(f".//body[@name='{wing.OUTER_BODY}']")
    moving={g.get('name') for g in outer.findall('geom')}
    static={g.get('name') for g in inner.findall('geom')}
    record=json.loads((FOLDER/'wing_articulation/build_manifest.json').read_text(encoding='utf-8'))
    assert any(n.startswith('instance_0214_') for n in moving)
    assert not any(n.startswith(('instance_0194_','instance_0195_','instance_0113_','instance_0114_','instance_0030_')) for n in moving)
    assert {'instance_0196_hinge_fixed_visual','instance_0197_hinge_fixed_visual'}<=static
    assert {'instance_0196_hinge_moving_visual','instance_0197_hinge_moving_visual'}<=moving
    assert len(moving)==sum(r['moving'] for r in record['geometry_mapping'])
    assert len(record['moving_leaf_instances'])==12
    assert record['joint']['actual_range_deg'] is None


@pytest.mark.skipif(not OUTPUT.exists(),reason='Generated wing model not present')
def test_colors_assets_and_original_model_unchanged():
    root=ET.parse(OUTPUT).getroot()
    old=ET.parse(FOLDER/'ground_validation_satellite.xml').getroot()
    assert not old.findall('.//joint') and len(old.findall('.//body'))==1
    source={g.get('name'):g for g in old.findall('.//geom')}
    target={g.get('name'):g for g in root.findall('.//geom')}
    assert len(target)==len(root.findall('.//geom'))==208
    record=json.loads((FOLDER/'wing_articulation/build_manifest.json').read_text(encoding='utf-8'))
    for row in record['geometry_mapping']:
        g=target[row['output_geom']];s=source[row['source_geom']]
        for attr in ('rgba','contype','conaffinity','mass','group'):
            assert g.get(attr)==s.get(attr)
    for mesh in root.findall('asset/mesh'):
        assert (FOLDER/root.find('compiler').get('meshdir')/mesh.get('file')).is_file()
    assert record['hinge_visual_partition']['area_error_m2']<1e-14
    assert record['validation']['rigid_motion_max_vertex_error_m']<1e-9

@pytest.mark.skipif(not OUTPUT.exists(),reason='Generated wing model not present')
@pytest.mark.parametrize('angle',[0,45,90])
def test_preview_vertices_equal_mujoco_forward_kinematics(monkeypatch,angle):
    mujoco=pytest.importorskip('mujoco')
    monkeypatch.syspath_prepend(str(ROOT/'tools'))
    from preview_satellite_wing_joint import geometry,pose
    model=mujoco.MjModel.from_xml_path(str(OUTPUT));data=mujoco.MjData(model)
    mujoco.mj_forward(model,data);template=geometry(model,data)
    vertices,_=pose(template,model,data,angle)
    start=0
    for gid in np.where(model.geom_group==2)[0]:
        expected=wing.world_vertices(model,data,gid)
        np.testing.assert_allclose(vertices[start:start+len(expected)],expected,atol=2e-12,rtol=0)
        start+=len(expected)
    assert start==len(vertices)
