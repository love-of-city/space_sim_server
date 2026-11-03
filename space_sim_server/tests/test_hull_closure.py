import xml.etree.ElementTree as ET

import pytest

from simulation.scene_library.hull_closure import add_hull_closure, audit_hull_closure, taskbox_frame_bounds
import numpy as np


def model():
    root=ET.fromstring('<mujoco><worldbody><body name="capture_target"/></worldbody></mujoco>')
    contract=add_hull_closure(root,[[0,0,0],[1,1,2]],[.2,.2,0],[.8,.8,0])
    return root,contract


def test_all_six_faces_closed_and_only_task_aperture_open():
    root,contract=model()
    result=audit_hull_closure(root,contract)
    assert result['passed'] and len(result['faces'])==6
    assert result['uncovered_area_m2']==0
    assert result['blocked_aperture_area_m2']==0


def test_detects_missing_panel_even_if_contract_is_unchanged():
    root,contract=model();body=root.find('.//body')
    body.remove(body.find('geom[@name="hull_closure_0_0_0"]'))
    result=audit_hull_closure(root,contract)
    assert not result['passed'] and result['uncovered_area_m2']>1


def test_detects_geometry_blocking_task_aperture():
    root,contract=model();body=root.find('.//body')
    ET.SubElement(body,'geom',name='hull_closure_block',type='box',pos='.5 .5 .0012',size='.1 .1 .001')
    result=audit_hull_closure(root,contract)
    assert not result['passed'] and result['blocked_aperture_area_m2']>0


def test_detects_visual_collision_divergence():
    root,contract=model()
    root.find('.//geom[@name="hull_closure_0_0_0_collision"]').set('size','.001 .1 .1')
    result=audit_hull_closure(root,contract)
    assert not result['passed'] and any('collision_mismatch' in e for e in result['errors'])


def test_rejects_aperture_that_breaks_adjacent_wall():
    root=ET.fromstring('<mujoco><worldbody><body name="capture_target"/></worldbody></mujoco>')
    with pytest.raises(ValueError,match='continuous rim'):
        add_hull_closure(root,[[0,0,0],[1,1,2]],[.2,.2,0],[1,.8,0])


def test_aperture_uses_fixed_frame_geometry_including_collision_proxy(monkeypatch):
    import sys
    # Primitive frames must work without the optional mesh tooling installed.
    monkeypatch.setitem(sys.modules, 'trimesh', None)
    root=ET.fromstring('''<mujoco><worldbody><body name="task_box_base">
    <geom type="box" pos=".225 .275 0" size=".225 .275 .01"/>
    <geom type="box" pos=".225 .275 0" size=".228 .278 .012" group="3"/>
    <body name="moving_drawer" pos="100 0 0"><geom type="box" size="1 1 1"/></body>
    </body></worldbody></mujoco>''')
    assert np.allclose(taskbox_frame_bounds(root),[[-.003,-.003,-.012],[.453,.553,.012]])
