"""Publication simplification must preserve inertia and renderer geom indexing."""

import xml.etree.ElementTree as ET
import ast
from pathlib import Path
from types import SimpleNamespace

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


def test_local_loader_builds_vfs_after_simplification_and_preserves_source(tmp_path):
    path = tmp_path / "scene.xml"
    root = document(COLLISION)
    root.find("asset/mesh").set("file", "collision.obj")
    path.write_bytes(ET.tostring(root))
    original = path.read_bytes()
    # A publication-only mesh must no longer be opened or passed to the VFS.
    # The real stepper still loads (and validates) the original source separately.
    scenario = Path(__file__).resolve().parents[1] / "model/SARM/platform/scenarios/scenario_sarm_grasp.py"
    loader = next(node for node in ast.parse(scenario.read_text()).body
                  if isinstance(node, ast.FunctionDef) and node.name == "_load_scene")
    calls = []
    def fake_scene(xml, files):
        calls.append((xml, files))
        return SimpleNamespace()

    namespace = {"mujoco": SimpleNamespace(MJScene=fake_scene),
                 "MODEL_PATH": path, "ET": ET, "Path": Path}
    exec(compile(ast.Module(body=[loader], type_ignores=[]), str(scenario), "exec"), namespace)
    scene = namespace["_load_scene"](kinematics_only=True)
    assert scene.publication_preparation["discarded_mesh_assets"] == 1
    assert scene.publication_preparation["retained_mesh_assets"] == 0
    assert calls[0][1] == []
    assert ET.fromstring(calls[0][0]).find("asset/mesh") is None
    assert path.read_bytes() == original


def test_native_publication_keeps_mass_states_and_visible_geom_indices():
    pytest.importorskip("Basilisk")
    import numpy as np
    from Basilisk.simulation import mujoco
    from Basilisk.utilities import SimulationBaseClass

    snapshots = []
    for simplified in (False, True):
        root = document(COLLISION)
        if simplified:
            prepare_publication_scene(root)
        scene = mujoco.MJScene(ET.tostring(root, encoding="unicode"), [])
        simulation = SimulationBaseClass.SimBaseClass()
        process = simulation.CreateNewProcess("p")
        process.addTask(simulation.CreateNewTask("t", 10**9))
        simulation.AddModelToTask("t", scene)
        simulation.InitializeSimulation()
        # Keep SWIG owners alive while reading their vector members.
        geoms = scene.getGeomInfos()
        geometry = []
        for i in range(len(geoms)):
            geom = geoms[i]
            geometry.append((geom.bodyName, list(geom.rgba)))
        snapshots.append((list(scene.getBodyNames()), scene.getBody("part").getMass(),
                          np.asarray(scene.getMassState().getState()).copy(), geometry))
    assert snapshots[0][0:2] == snapshots[1][0:2]
    np.testing.assert_array_equal(snapshots[0][2], snapshots[1][2])
    assert snapshots[0][3] == snapshots[1][3]
