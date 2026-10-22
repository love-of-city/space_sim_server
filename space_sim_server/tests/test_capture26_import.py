"""Regression checks for the additive capture-arm import."""
import json
import xml.etree.ElementTree as E
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_original_bodies_unchanged():
    base=E.parse(ROOT/'model/SARM/platform/sarm_20261012.xml')
    new=E.parse(ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    manifest=json.loads((ROOT/'model/SARM/platform/sarm_dual_capture26.manifest.json').read_text())
    changed={x['geom'] for x in manifest.get('mount_clearance_changes',[])}
    assert all(n.startswith('base_link_geom') for n in changed)
    target=new.find(".//body[@name='capture_target']")
    original_target=base.find(".//body[@name='capture_target']")
    old_names={g.get('name') for g in original_target.findall('geom')}
    extras=[g for g in target.findall('geom') if g.get('name') not in old_names]
    assert {g.get('name') for g in extras} == {
        *(f'capture_x_piece_{i}' for i in range(898)),
        *(f'capture_bus_{side}' for side in ('core','left','right','lower','upper')),
    }
    assert all(g.get('mass')=='0' for g in extras)
    old_box=original_target.find("geom[@name='capture_target_collision']")
    new_box=target.find("geom[@name='capture_target_collision']")
    assert new_box.attrib == dict(old_box.attrib,contype='0',conaffinity='0')
    for b in base.iter('body'):
        match=new.find(f".//body[@name='{b.get('name')}']")
        assert match is not None and match.attrib==b.attrib
        for tag in ['geom','inertial','joint','freejoint','site','camera']:
            # Only the reviewed target collision replacement and mounting
            # clearance geoms may differ; inertia and original visuals remain exact.
            ignored=changed | ({'capture_target_collision',*(g.get('name') for g in extras)} if b.get('name')=='capture_target' and tag=='geom' else set())
            canon=lambda els:[E.canonicalize(E.tostring(x,encoding='unicode'),strip_text=True) for x in els if x.get('name') not in ignored]
            assert canon(b.findall(tag))==canon(match.findall(tag))

def test_capture_joints_and_scene_registration():
    from space_arm_platform.scene_targets import capture_target,SARM_TEMPLATES
    from space_arm_platform.scene_runtime import SCENE_TEMPLATES
    assert 'sarm-dual-capture26' in SARM_TEMPLATES
    assert any(x['id']=='sarm-dual-capture26' for x in SCENE_TEMPLATES)
    p=capture_target('sarm-dual-capture26').resolve_model(ROOT/'model/SARM/platform')
    r=E.parse(p)
    assert r.find(".//joint[@name='axial']").get('range')=='-.006 .02'
    assert len([a for a in r.findall('./actuator/position') if a.get('name','').startswith('bottom_')])==6
    assert not any('bottom_' in (e.get('body1','')+e.get('body2','')) for e in r.findall('./equality/weld'))
