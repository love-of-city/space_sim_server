import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scene_library.taskbox_recess import apply_recess_contract, clip_surface_pocket, subtract_box


@pytest.mark.parametrize('cut', [([.2, -.1, .2], [.8, .5, .8]), ([-1, -1, -1], [2, 2, 2]), ([2, 2, 2], [3, 3, 3])])
def test_collision_pocket_has_exact_volume_without_overlapping_solids(cut):
    parts = subtract_box([0, 0, 0], [1, 1, 1], *cut)
    volume = sum(np.prod(np.array(b)-a) for a, b in parts)
    clipped = np.maximum(0, np.minimum(1, cut[1])-np.maximum(0, cut[0]))
    assert volume == pytest.approx(1-np.prod(clipped))
    for i, (a, b) in enumerate(parts):
        for c, d in parts[i+1:]:
            assert np.any(np.minimum(b, d)-np.maximum(a, c) <= 0)


def test_open_surface_cut_removes_only_aperture_and_preserves_winding():
    trimesh = pytest.importorskip('trimesh')
    mesh = trimesh.Trimesh(vertices=[[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]], faces=[[0,1,2],[0,2,3]], process=False)
    cut = clip_surface_pocket(mesh, np.array([-.25,-.25,-.1]), np.array([.25,.25,.1]))
    assert cut.area == pytest.approx(3.75)
    assert np.all(cut.face_normals[:,2] > .99)
    assert np.allclose(cut.bounds, mesh.bounds)


def test_recess_replay_rejects_changed_source_geometry(tmp_path):
    root = ET.fromstring('<mujoco><asset/><worldbody><body name="capture_target"><geom name="wall" type="box" size="1 1 1"/></body></worldbody></mujoco>')
    recipe = tmp_path/'recess.json'
    recipe.write_text(json.dumps(dict(schema='taskbox-recess/1', assets=[], edits=[dict(parent='capture_target', original=dict(name='wall',type='box',size='2 2 2'), replacement=[])])))
    with pytest.raises(ValueError, match='source geometry mismatch'):
        apply_recess_contract(root, recipe)
