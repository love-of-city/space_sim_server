"""Continuous PBR0110 shell: no mid-face gaps, retained hardware/top keepouts."""
from collections import Counter
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ContentSource/PBR0110/Continuous'
spec=importlib.util.spec_from_file_location('continuous_blanket',ROOT/'scripts/generate_pbr0110_blanket.py')
generator=importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)

@pytest.fixture(scope='module')
def mesh():
    recipe=json.loads((SOURCE/'recipe.json').read_text())
    return recipe,generator.generate(recipe)


def test_one_continuous_sheet_per_face_instead_of_twenty_halves(mesh):
    r,(v,uv,f,m,pieces)=mesh
    assert len(pieces)==len(r['faces'])==10
    assert len({p['face'] for p in pieces})==10
    assert r['seam_gap']==0 and r['outer_inset']==0
    for p in pieces:
        face=next(face for face in r['faces'] if face['name']==p['face'])
        assert p['bounds_uv_m']==[*face['urange'],*face['vrange']]


def test_no_artificial_interior_boundary_or_exposed_midpoint_gap(mesh):
    r,(v,uv,f,m,pieces)=mesh
    for p in pieces:
        face=next(face for face in r['faces'] if face['name']==p['face'])
        front=f[p['face_start']:p['face_start']+p['front_triangle_count']]
        edges=Counter(tuple(sorted(edge)) for tri in front for edge in zip(tri,(tri[1],tri[2],tri[0])))
        a,b,c,d=p['bounds_uv_m']
        for (i,j),count in edges.items():
            if count!=1:continue
            u,w=(v[[i,j]].mean(0))[[face['uaxis'],face['vaxis']]]
            outer=any(abs(x-y)<1e-7 for x,y in [(u,a),(u,b),(w,c),(w,d)])
            hardware=any(((abs(u-x0)<1e-7 or abs(u-x1)<1e-7) and y0-1e-7<=w<=y1+1e-7) or
                         ((abs(w-y0)<1e-7 or abs(w-y1)<1e-7) and x0-1e-7<=u<=x1+1e-7)
                         for x0,x1,y0,y1 in p['keepouts_m'])
            assert outer or hardware,(p['name'],u,w)


def test_each_sheet_closed_and_no_degenerate_triangles(mesh):
    r,(v,uv,f,m,pieces)=mesh
    edges=Counter(tuple(sorted(edge)) for tri in f for edge in zip(tri,(tri[1],tri[2],tri[0])))
    assert set(edges.values())=={2}
    t=v[f]
    assert np.all(np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1)>1e-14)
    assert np.isfinite(v).all() and np.isfinite(uv).all()


def test_corner_ranges_cover_old_exposed_frame_and_top_stays_open(mesh):
    r,(v,uv,f,m,pieces)=mesh
    by_name={face['name']:face for face in r['faces']}
    s=r['standoff']
    assert v[:,2].max()<=1e-8
    assert not any(face['axis']==2 and face['sign']==1 for face in r['faces'])
    np.testing.assert_allclose(by_name['minus_y']['urange'],[-.2-s,.6+s])
    np.testing.assert_allclose(by_name['plus_x']['urange'],[-.11-s,.64+s])
    np.testing.assert_allclose(by_name['minus_y']['vrange'],[-.49-s,0])
    assert by_name['bus_bottom']['keepout_clearance']<s
    assert by_name['lower_plus_x']['vrange'][1]>-.49-s
    np.testing.assert_allclose(by_name['lower_bottom']['urange'],[-.15-s,.45+s])


def test_hardware_openings_remain_uncovered(mesh):
    r,(v,uv,f,m,pieces)=mesh
    for p in pieces:
        face=next(face for face in r['faces'] if face['name']==p['face'])
        centers=v[f[p['face_start']:p['face_start']+p['front_triangle_count']]].mean(1)
        for u,w in centers[:,[face['uaxis'],face['vaxis']]]:
            assert not any(generator.in_box(u,w,box) for box in p['keepouts_m'])


def test_side_uvs_continue_across_corners(mesh):
    r,(v,uv,f,m,pieces)=mesh
    faces={face['name']:face for face in r['faces']}
    for prefix in ['', 'lower_']:
        for before,after in [('minus_y','plus_x'),('plus_x','plus_y'),('plus_y','minus_x')]:
            a,b=faces[prefix+before],faces[prefix+after]
            def endpoints(face):
                w=face['uv_wrap']
                return sorted(w['offset_m']+w['u_sign']*(x-w['u_origin_m']) for x in face['urange'])
            assert abs(endpoints(a)[1]-endpoints(b)[0])<1e-12
            assert a['uv_wrap']['v_origin_m']==b['uv_wrap']['v_origin_m']


def test_no_dark_hem_bands_added_to_continuous_surface():
    config=json.loads((ROOT/'ContentSource/PBR0110/source.json').read_text())
    assert config['instances']['MliHem']==config['instances']['MliFace']
