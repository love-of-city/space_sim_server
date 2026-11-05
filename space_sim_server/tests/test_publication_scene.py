"""Publication simplification must preserve inertia and renderer geom indexing."""

import xml.etree.ElementTree as ET

import pytest

from simulation.publication_scene import prepare_publication_scene


def document(geoms, *, extra="", defaults="", assets=""):
    return ET.fromstring(f'''<mujoco><compiler fusestatic="false"/>{defaults}
        <asset><mesh name="collision" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>{assets}</asset>
        <worldbody><body name="part"><freejoint/>
        <geom name="visual" type="box" size=".1 .2 .3" mass="2"/>{geoms}
        <site name="tip" pos=".1 0 0"/></body></worldbody>{extra}</mujoco>''')


COLLISION = '<geom name="collider" type="mesh" mesh="collision" group="3" density="0" rgba="0 0 0 0"/>'


def test_proxy_keeps_order_names_body_and_pose_and_prunes_only_unused_mesh():
    root = document(COLLISION.replace('/>', ' pos="1 2 3" quat="1 0 0 0"/>'))
    original = ET.tostring(root)
    geoms = root.findall("./worldbody//geom")
    stats = prepare_publication_scene(root)
    assert stats["collision_mesh_proxies"] == stats["discarded_mesh_assets"] == 1
    assert root.findall("./worldbody//geom") == geoms
    assert [g.get("name") for g in geoms] == ["visual", "collider"]
    assert geoms[1].get("type") == "sphere" and geoms[1].get("mass") == "0"
    assert geoms[1].get("pos") == "1 2 3" and geoms[1].get("quat") == "1 0 0 0"
    assert root.find("option/flag").attrib == {"contact": "disable", "constraint": "disable"}
    assert ET.tostring(geoms[0]) in original
    assert prepare_publication_scene(root)["collision_mesh_proxies"] == 0


@pytest.mark.parametrize("change", [
    {"mass": "1"}, {"density": "1000"}, {"rgba": "0 0 0 1"},
    {"rgba": "0 0 0 .1"}, {"rgba": None}, {"group": "2"}, {"material": "paint"},
])
def test_preserve_mass_contributors_visible_or_uncertain_geometry(change):
    root = document(COLLISION)
    geom = root.find("./worldbody/body/geom[@name='collider']")
    for key, value in change.items():
        if value is None:
            geom.attrib.pop(key)
        else:
            geom.set(key, value)
    before = ET.tostring(geom)
    prepare_publication_scene(root)
    assert ET.tostring(geom) == before


@pytest.mark.parametrize("extra", [
    '<contact><pair geom1="collider" geom2="visual"/></contact>',
    '<sensor><framepos name="position" objtype="geom" objname="collider"/></sensor>',
    '<extension><plugin plugin="test"><instance name="p"><config key="geom" value="collider"/></instance></plugin></extension>',
    '<include file="unknown.xml"/>',
])
def test_preserve_referenced_geometry_and_unexpanded_includes(extra):
    root = document(COLLISION, extra=extra)
    assert prepare_publication_scene(root)["collision_mesh_proxies"] == 0


@pytest.mark.parametrize("default", [
    '<geom mass="1"/>', '<geom material="paint"/>', '<geom mesh="collision"/>',
    '<geom><plugin plugin="test"/></geom>',
])
def test_default_attributes_are_handled_conservatively(default):
    root = document(COLLISION, defaults=f'<default><default class="c">{default}</default></default>')
    assert prepare_publication_scene(root)["collision_mesh_proxies"] == 0


def test_explicit_zero_mass_overrides_density_and_preserves_shared_mesh():
    root = document(COLLISION.replace('density="0"', 'mass="0" density="1000"')
                    + '<geom name="shared_visual" mesh="collision" type="mesh" mass="1"/>')
    stats = prepare_publication_scene(root)
    assert stats["collision_mesh_proxies"] == 1
    assert stats["discarded_mesh_assets"] == 0


def test_plugin_geometry_and_mesh_assets_stay_intact():
    root = document(COLLISION.replace('mesh="collision"', 'mesh="sdf"'),
                    assets='<mesh name="sdf"><plugin plugin="test"/></mesh>')
    assert prepare_publication_scene(root)["collision_mesh_proxies"] == 0
    root = document(COLLISION.replace('/>', '><plugin plugin="test"/></geom>'))
    assert prepare_publication_scene(root)["collision_mesh_proxies"] == 0
