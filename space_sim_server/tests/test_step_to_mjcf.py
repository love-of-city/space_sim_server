"""Geometry/pose regression tests; CAD integration skips without optional OCP."""
from __future__ import annotations

import importlib.util
from argparse import Namespace
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

np = pytest.importorskip('numpy')
MODULE = Path(__file__).resolve().parents[1] / 'tools' / 'convert_step_to_mjcf.py'
spec = importlib.util.spec_from_file_location('step_mjcf_converter', MODULE)
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


@pytest.mark.parametrize('axis', [(1,0,0),(0,1,0),(0,0,1),(1,2,3)])
@pytest.mark.parametrize('angle', [0, 0.3, -1.4, np.pi, np.pi-1e-9])
def test_quaternion_preserves_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x,y,z = axis
    cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    r=np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*(cross@cross)
    w,x,y,z=converter.quaternion(r)
    rebuilt=np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    np.testing.assert_allclose(rebuilt,r,atol=1e-10)


@pytest.mark.parametrize('invalid', [np.diag([-1,1,1]), np.eye(3)*2, np.full((3,3),np.nan)])
def test_quaternion_rejects_nonrigid_pose(invalid):
    with pytest.raises(ValueError):
        converter.quaternion(invalid)


def test_box_inertia_units_and_triangle_inequality():
    inertia=converter.box_inertia(12,[1,2,3])
    np.testing.assert_allclose(inertia,[13,10,5])
    assert max(inertia) <= sum(inertia)-max(inertia)
    with pytest.raises(ValueError): converter.box_inertia(-1,[1,2,3])
    with pytest.raises(ValueError): converter.box_inertia(1,[1,0,3])


def test_obj_preserves_local_origin_and_winding(tmp_path):
    vertices=np.array([[1.,2,3],[2,2,3],[2,3,3],[1,3,3]])
    triangles=np.array([[0,1,2],[0,2,3]])
    out=tmp_path/'quad.obj'
    n,t=converter.write_obj(out,[(vertices,triangles)],np.array([1.,2,3]))
    assert (n,t)==(4,2)
    text=out.read_text()
    assert 'v 0 0 0\n' in text
    assert 'f 1//1 2//2 3//3' in text
    np.testing.assert_allclose(converter.mesh_normals(vertices,triangles), [[0,0,1]]*4)


def test_visual_and_synthetic_physics_are_separate():
    visual=converter.create_mjcf([],[],[1,2,3])
    assert visual.find('.//freejoint') is None
    assert visual.find('.//inertial') is None
    assert visual.find('.//geom') is None
    free=converter.create_mjcf([],[],[1,2,3],1.0)
    assert free.find('.//freejoint') is not None
    assert free.find('.//inertial').get('mass')=='1'
    assert free.find('.//geom').get('name')=='preview_collision_bbox'
    assert free.find('.//geom').get('group')=='3'


def test_nested_step_instances_faces_and_colors(tmp_path):
    pytest.importorskip('OCP')
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf
    from OCP.TDataStd import TDataStd_Name
    from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCP.TopLoc import TopLoc_Location
    from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE

    doc=TDocStd_Document(TCollection_ExtendedString('XCAF'))
    st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    part=st.AddShape(BRepPrimAPI_MakeBox(100,200,300).Shape(), False)
    TDataStd_Name.Set_s(part,TCollection_ExtendedString('colored_box'))
    ct.SetColor(part,Quantity_Color(0.7,0.2,0.1,Quantity_TOC_RGB),XCAFDoc_ColorSurf)
    shape=st.GetShape_s(part)
    explorer=TopExp_Explorer(shape,TopAbs_FACE)
    face_label=st.AddSubShape(part,explorer.Current())
    ct.SetColor(face_label,Quantity_Color(0.1,0.2,0.7,Quantity_TOC_RGB),XCAFDoc_ColorSurf)
    sub=st.NewShape()
    TDataStd_Name.Set_s(sub,TCollection_ExtendedString('rotated_subassembly'))
    rotation=gp_Trsf()
    rotation.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),np.pi/2)
    rotation.SetTranslationPart(gp_Vec(10,20,30))
    st.AddComponent(sub,part,TopLoc_Location(rotation))
    root=st.NewShape()
    TDataStd_Name.Set_s(root,TCollection_ExtendedString('top'))
    translated=gp_Trsf(); translated.SetTranslation(gp_Vec(1000,2000,3000))
    st.AddComponent(root,sub,TopLoc_Location(translated))
    other=gp_Trsf(); other.SetTranslation(gp_Vec(-1000,-2000,-3000))
    st.AddComponent(root,part,TopLoc_Location(other))
    st.UpdateAssemblies()
    writer=STEPCAFControl_Writer()
    writer.SetColorMode(True); writer.SetNameMode(True)
    assert writer.Transfer(doc,STEPControl_AsIs)
    step=tmp_path/'nested.step'
    assert writer.Write(str(step))==IFSelect_RetDone
    parts,nodes,occurrences,warnings,exact=converter.load_cad(step,0.25,0.18)
    assert len(parts)==1
    assert len(occurrences)==2
    assert len(parts[0]['chunks'])==2
    assert sum(len(t) for group in parts[0]['chunks'].values() for _,t in group)==12
    vertices=[]
    for instance in occurrences:
        m=instance['transform']
        vertices.append(instance['part']['vertices']@m[:3,:3].T+m[:3,3])
    vertices=np.concatenate(vertices)
    np.testing.assert_allclose([vertices.min(0),vertices.max(0)], exact, atol=1e-8)
    assert exact[1,2]>3
    assert exact[0,2]<-2.9
    pytest.importorskip('mujoco')
    report=converter.convert(Namespace(source=step,output=tmp_path/'output',deflection_mm=0.25,
                                      angular_deflection_rad=0.18,preview_mass_kg=1.0,
                                      allow_incomplete_preview=False,no_render=True))
    assert report['status']=='complete'
    assert report['counts']['rendered_leaf_instances']==2
    assert report['counts']['omitted_cad_faces']==0
    assert report['validation']['satellite_free_preview.xml']['dofs']==6
    assert report['validation']['ground_validation_satellite.xml']['max_component_placement_error_m']<2e-5

def test_mujoco_compiles_planar_color_patches(tmp_path):
    mujoco=pytest.importorskip('mujoco')
    (tmp_path/'meshes').mkdir()
    v=np.array([[0.,0,0],[1,0,0],[1,1,0],[0,1,0]])
    t=np.array([[0,1,2],[0,2,3]])
    converter.write_obj(tmp_path/'meshes'/'plane.obj',[(v,t)],np.zeros(3))
    meshes=[dict(name='plane',file='plane.obj')]
    geoms=[dict(name='visual',mesh='plane',position_m=[0,0,0],quaternion_wxyz=[1,0,0,0],rgba=[0.6,0.7,0.9,1])]
    converter.save_xml(converter.create_mjcf(meshes,geoms,[1,1,1]),tmp_path/'test.xml')
    model=mujoco.MjModel.from_xml_path(str(tmp_path/'test.xml'))
    assert model.nmesh==1
    assert model.ngeom==1
    assert model.njnt==0
    assert np.all(model.geom_contype==0)


def test_single_triangle_patch_subdivision_preserves_area(tmp_path):
    vertices=np.array([[0.,0,0],[1,0,0],[0,1,0]])
    triangle=np.array([[0,1,2]])
    path=tmp_path/'triangle.obj'
    assert converter.write_obj(path,[(vertices,triangle)],np.zeros(3))==(4,2)
    lines=path.read_text().splitlines()
    v=np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith('v ')])
    t=np.array([[int(x.split('//')[0])-1 for x in line.split()[1:]] for line in lines if line.startswith('f ')])
    areas=np.linalg.norm(np.cross(v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]]),axis=1)/2
    assert sum(areas)==pytest.approx(0.5)
    np.testing.assert_allclose(v.min(0),vertices.min(0))
    np.testing.assert_allclose(v.max(0),vertices.max(0))

def test_cpu_preview_z_buffer_order_independent(monkeypatch):
    pytest.importorskip('mujoco')
    pytest.importorskip('numba')
    monkeypatch.syspath_prepend(str(MODULE.parent))
    from cpu_mjcf_preview import rasterize
    projected=np.array([[2.,2,0.5],[14,2,0.5],[2,14,0.5],
                        [2,2,1.0],[14,2,1.0],[2,14,1.0]])
    normals=np.tile(np.array([[0.,0,1.]],dtype=np.float32),(6,1))
    faces=np.array([[0,1,2],[3,4,5]],dtype=np.int32)
    colors=np.array([[0.,0,0.8],[0.8,0,0]],dtype=np.float32)
    light=np.array([0.,0,1.])
    image,depth=rasterize(projected,normals,faces,faces,colors,16,16,light,light,light)
    reverse,_=rasterize(projected,normals,faces[::-1].copy(),faces[::-1].copy(),colors[::-1].copy(),16,16,light,light,light)
    np.testing.assert_allclose(image,reverse)
    assert depth[4,4]==1.0
    assert image[4,4,0]>image[4,4,2]
    assert depth[15,15]==0
    assert np.all(np.isfinite(image))
