"""Geometry and runtime wiring regressions for a visual-only SARM MLI shell."""
from collections import Counter
import configparser
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ContentSource/SarmMLI'
spec=importlib.util.spec_from_file_location('generate_sarm_blanket',ROOT/'scripts/generate_sarm_blanket.py')
generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)

@pytest.fixture(scope='module')
def mesh():
    recipe=json.loads((SOURCE/'recipe.json').read_text(encoding='utf-8'))
    return recipe,generator.generate(recipe)


def test_generated_source_hash_and_counts_are_recorded(mesh):
    recipe,(v,uv,f,m,p)=mesh
    manifest=json.loads((SOURCE/'mesh_manifest.json').read_text(encoding='utf-8'))
    assert hashlib.sha256((SOURCE/'SM_SarmMLI.obj').read_bytes()).hexdigest()==manifest['mesh_sha256']
    assert (len(v),len(f),len(p))==(manifest['vertices'],manifest['triangles'],20)
    assert manifest['source_sha256']==recipe['source_sha256']
    assert manifest['physics']=='none'
    assert set(m)=={'MliFace','MliHem','MliBacking'}


def test_blanket_is_closed_and_consistently_wound(mesh):
    _,(v,uv,faces,m,p)=mesh
    edges=Counter(tuple(sorted((a,b))) for face in faces for a,b in zip(face,(face[1],face[2],face[0])))
    assert set(edges.values())=={2}
    directed=Counter((a,b) for face in faces for a,b in zip(face,(face[1],face[2],face[0])))
    assert all(directed[a,b]==directed[b,a] for a,b in directed)
    t=v[faces];area=np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1)*.5
    assert np.all(area>1e-14)
    assert np.isfinite(v).all() and np.isfinite(uv).all()


def test_twenty_nonoverlapping_front_uv_islands_with_uniform_density(mesh):
    recipe,(v,uv,faces,m,pieces)=mesh
    for piece in pieces:
        start=piece['vertex_start'];count=piece['front_vertex_count'];slot=piece['uv_island']
        mapped=uv[start:start+count]
        assert mapped[:,0].min()>=.25*(slot%4)
        assert mapped[:,0].max()<.25*(slot%4+1)
        assert mapped[:,1].min()>=.5*(slot//4)
        assert mapped[:,1].max()<.5*(slot//4+1)
        face=next(f for f in recipe['faces'] if f['name']==piece['face'])
        positions=v[start:start+count]
        expected=(positions[:,[face['uaxis'],face['vaxis']]]-positions[0,[face['uaxis'],face['vaxis']]])*recipe['atlas_scale_per_m']
        np.testing.assert_allclose(mapped-mapped[0],expected,atol=1e-9)


def test_skin_is_outside_body_covers_bottom_and_leaves_top_open(mesh):
    recipe,(v,uv,faces,m,pieces)=mesh
    assert np.max(v[:,2])<=-.018+1e-8
    assert -.697 < np.min(v[:,2]) < -.693
    for piece in pieces:
        face=next(f for f in recipe['faces'] if f['name']==piece['face'])
        start=piece['vertex_start'];count=piece['front_vertex_count'];a=face['axis']
        front=v[start:start+count];back=v[start+count:start+count*2]
        signed=(front[:,a]-face['plane'])*face['sign']
        assert signed.min()>=recipe['standoff']-1e-9
        assert signed.max()<.007
        np.testing.assert_allclose((front[:,a]-back[:,a])*face['sign'],recipe['visual_thickness'],atol=1e-9)
        assert np.ptp(signed)>.001  # Real macro folds, not only shader normals.


def test_hardware_cutouts_and_seams_are_not_filled(mesh):
    recipe,(v,uv,faces,m,pieces)=mesh
    for piece in pieces:
        face=next(f for f in recipe['faces'] if f['name']==piece['face'])
        start=piece['face_start'];count=piece['front_triangle_count']
        centers=v[faces[start:start+count]].mean(1)
        for x,y in centers[:,[face['uaxis'],face['vaxis']]]:
            assert not any(generator.in_box(x,y,box) for box in piece['keepouts_m'])
        mid=sum(face['urange'])*.5
        assert np.all(np.abs(centers[:,face['uaxis']]-mid)>=recipe['seam_gap']/2-1e-8)


def test_runtime_attaches_child_without_replacing_base_or_enabling_physics():
    source=(ROOT/'Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/Private/BskSceneController.cpp').read_text(encoding='utf-8')
    helper=source.split('void AttachConfiguredVisualOverlay(',1)[1].split('\n}',1)[0]
    for text in ['TEXT("BskDisableVisualOverlays")','*Geometry.AssetPath, MeshPath, GGameIni','Overlay->SetupAttachment(Parent)','Overlay->SetRelativeTransform(FTransform::Identity)','Overlay->SetCollisionEnabled(ECollisionEnabled::NoCollision)','Overlay->SetGenerateOverlapEvents(false)','Overlay->SetCanEverAffectNavigation(false)','original mesh retained']:
        assert text in helper
    assert 'Parent->SetStaticMesh' not in helper and 'SetSimulatePhysics(true)' not in helper
    assert 'ApplyGeometryMaterial(Overlay' not in helper
    assert source.count('AttachConfiguredVisualOverlay(Component, Geometry);')==1


def test_config_only_attaches_to_arm_carrying_bus():
    config=configparser.ConfigParser(strict=False);config.optionxform=str
    config.read(ROOT/'Config/DefaultGame.ini',encoding='utf-8')
    entries=dict(config['Bsk.VisualOverlays']);assert entries.pop('Enabled')=='True'
    assert entries=={'/Game/BSK/Generated/SARM/base_link.base_link':'/Game/BSK/VisualOverlays/SarmMLI_PBR0110/SM_SarmMLI_PBR0110.SM_SarmMLI_PBR0110'}
    assert 'part_001_color_00' in next(k for k in config['Bsk.MaterialOverrides'] if k.startswith('/Game/'))


def test_uv_material_and_mesh_import_preserve_explicit_uvs_and_native_slots():
    source=(ROOT/'scripts/create_sarm_blanket_assets.py').read_text(encoding='utf-8-sig')
    assert 'MaterialExpressionTextureCoordinate' in source
    assert "'tangent_space_normal',True" in source
    assert "'recompute_tangents',True" in source
    assert "'generate_lightmap_u_vs',False" in source
    assert 'subsystem.get_num_uv_channels' in source
    assert 'Incomplete MLI material slots' in source
    assert 'Blanket triangle loss on import' in source
    assert "'/Game/BSK/VisualOverlays/SarmMLI'" in source


def test_exported_obj_has_non_degenerate_uvs_including_closing_rims():
    uv=[];faces=[];vertices=[]
    for line in (SOURCE/'SM_SarmMLI.obj').read_text().splitlines():
        if line.startswith('v '): vertices.append([float(x) for x in line.split()[1:]])
        elif line.startswith('vt '): uv.append([float(x) for x in line.split()[1:]])
        elif line.startswith('f '):faces.append([int(x.split('/')[1])-1 for x in line.split()[1:]])
    t=np.asarray(uv)[np.asarray(faces)]
    a=t[:,1]-t[:,0];b=t[:,2]-t[:,0]
    determinant=a[:,0]*b[:,1]-a[:,1]*b[:,0]
    assert np.all(np.abs(determinant)>1e-14)
    manifest=json.loads((SOURCE/'mesh_manifest.json').read_text())
    assert manifest['obj_length_unit']=='cm'
    np.testing.assert_allclose(np.asarray(vertices).min(0),np.asarray(manifest['bounds_m'][0])*100,atol=1e-7)
    np.testing.assert_allclose(np.asarray(vertices).max(0),np.asarray(manifest['bounds_m'][1])*100,atol=1e-7)


def test_startup_prepares_blanket_before_generic_cache_return():
    prepare=(ROOT/'scripts/prepare_runtime_materials.ps1').read_text(encoding='utf-8')
    assert prepare.index('prepare_sarm_blanket.ps1') < prepare.index('if (!$Force')
    script=(ROOT/'scripts/prepare_sarm_blanket.ps1').read_text(encoding='utf-8-sig')
    for fragment in ['Get-FileHash','SARM_MLI_BUILD_OK','SARM_MLI_GRAPH_READY','-WindowStyle Hidden','-ExecutePythonScript']:
        assert fragment in script
    assert script.index("$content -notmatch 'SARM_MLI_BUILD_OK'") < script.index('Set-Content -LiteralPath $marker')


def test_bottom_cap_and_housing_surfaces_are_present_without_covering_mount(mesh):
    recipe,(v,uv,faces,m,pieces)=mesh
    expected={'bus_bottom','lower_minus_x','lower_plus_x','lower_minus_y','lower_plus_y','lower_bottom'}
    assert expected.issubset({p['face'] for p in pieces})
    assert len(pieces)==20
    assert not any(f['axis']==2 and f['sign']==1 for f in recipe['faces'])
    cap=next(f for f in recipe['faces'] if f['name']=='lower_bottom')
    assert cap['plane']==-.69 and cap['sign']==-1
    for p in pieces:
        if p['face']!='bus_bottom':continue
        start=p['face_start'];count=p['front_triangle_count']
        centers=v[faces[start:start+count]].mean(1)
        assert not any(-.158<x<.458 and .057<y<.473 for x,y,z in centers)


def test_existing_side_positions_uvs_and_faces_are_unchanged(mesh):
    recipe,(v,uv,faces,m,pieces)=mesh
    previous=dict(recipe,faces=recipe['faces'][:4])
    old_v,old_uv,old_f,old_m,old_p=generator.generate(previous)
    np.testing.assert_array_equal(v[:len(old_v)],old_v)
    np.testing.assert_array_equal(uv[:len(old_uv)],old_uv)
    np.testing.assert_array_equal(faces[:len(old_f)],old_f)
    assert m[:len(old_m)]==old_m
    assert pieces[:len(old_p)]==old_p
