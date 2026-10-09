"""Composition, scene identity, original CAD preservation and grasp contact."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager
from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, GROUND_TARGET_TEMPLATE, SELF_COLLISION_TEMPLATE, LEGACY_TEMPLATE, capture_target

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "model/SARM/platform/sarm_platform.xml"
SOURCE = ROOT / "model/ground_validation_satellite/ground_validation_satellite_articulated.xml"
MODEL = BASE.with_name("sarm_ground_target.xml")


def test_composition_is_current_and_does_not_change_sarm_or_source_cad():
    spec = importlib.util.spec_from_file_location("ground_target_builder", ROOT / "tools/build_sarm_ground_target.py")
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    xml, manifest = builder.composed_bytes()
    assert MODEL.read_bytes().replace(b"\r\n", b"\n") == xml
    assert MODEL.with_suffix(".manifest.json").read_bytes().replace(b"\r\n", b"\n") == manifest
    base, combined = ET.parse(BASE).getroot(), ET.parse(MODEL).getroot()
    for path in ('.//body[@name="cubesat_bus"]', 'actuator', 'custom'):
        assert ET.tostring(base.find(path)) == ET.tostring(combined.find(path))
    assert ET.parse(SOURCE).find('.//body[@name="ground_validation_satellite"]/joint') is None
    assert ET.parse(SOURCE).find('.//geom[@contype="1"]') is None


def test_target_keeps_all_source_visuals_with_relocatable_assets():
    source, combined = ET.parse(SOURCE).getroot(), ET.parse(MODEL).getroot()
    target = combined.find('.//body[@name="capture_target"]')
    originals = {g.get("name"): g for g in source.findall('.//geom')}
    visuals = target.findall('.//geom[@group="2"]')
    assert len(visuals) == len(originals) == 208
    for geom in visuals:
        original = originals[geom.get("name").removeprefix("target_")]
        for field in ("pos", "quat", "rgba"):
            assert geom.get(field) == original.get(field)
        assert geom.get("mesh") == "target_" + original.get("mesh")
        assert geom.get("contype") == geom.get("conaffinity") == geom.get("mass") == "0"
    compiler = combined.find("compiler")
    for mesh in combined.findall('./asset/mesh[@file]'):
        assert not Path(mesh.get("file")).is_absolute()
        asset = (MODEL.parent / compiler.get("meshdir") / mesh.get("file")).resolve()
        assert asset.is_relative_to((ROOT / "model").resolve()) and asset.is_file()
        assert not asset.read_bytes()[:128].startswith(b"version https://git-lfs.github.com/spec/v1")


def test_target_has_one_free_root_and_passive_hinge_and_explicit_synthetic_inertia():
    root = ET.parse(MODEL).getroot()
    target = root.find('.//body[@name="capture_target"]')
    assert len(target.findall('.//joint')) == 2
    free = target.find('./joint')
    assert free.get("type") == "free"
    hinge = target.find('.//joint[@name="outer_panel_hinge"]')
    assert hinge.get("limited") == "false" and hinge.get("range") is None
    for joint in (free, hinge):
        for field in ("damping", "frictionloss", "stiffness", "armature"):
            assert float(joint.get(field)) == 0
    assert sum(float(i.get("mass")) for i in target.findall('.//inertial')) == 10
    assert len(target.findall('.//geom[@group="3"]')) == 3
    assert all(a.get("joint") != "outer_panel_hinge" for a in root.find("actuator"))
    assert target.find('.//geom[@name="capture_target_handle"]') is None  # No invented CAD handle.
    assert target.find('.//site[@name="capture_target_grasp"]') is not None
    assert target.find('.//body[@name="satellite_outer_panel"]') is not None


def test_new_default_and_legacy_template_keep_distinct_reproducible_layouts(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    assert manager.catalog()["defaults"]["template_id"] == SELF_COLLISION_TEMPLATE == DEFAULT_TEMPLATE
    new = manager.create_instance(SceneInstanceCreate(seed=7))
    repeat = manager.create_instance(SceneInstanceCreate(seed=7))
    old = manager.create_instance(SceneInstanceCreate(seed=7, template_id=LEGACY_TEMPLATE))
    assert new["randomization"] == repeat["randomization"]
    assert new["capture_target"]["synthetic_mass_kg"] == 10
    assert new["randomization"]["target_hinge_position_rad"] == 0
    assert "target_hinge_position_rad" not in old["randomization"]
    assert old["capture_target"]["runtime_model"].endswith("sarm_platform.xml")
    for instance in (new, old):
        expected = capture_target(instance["template_id"]).position_m
        assert np.max(np.abs(np.asarray(instance["randomization"]["target_position_m"]) - expected)) <= .020
    assert new["randomization"]["arm_joint_position_rad"] == old["randomization"]["arm_joint_position_rad"]
    assert new["environment"]["orbit"] == old["environment"]["orbit"]


@pytest.fixture(scope="module")
def mujoco_model():
    m = pytest.importorskip("mujoco")
    return m, m.MjModel.from_xml_path(str(MODEL))


def set_instance(m, model, data, instance):
    m.mj_resetData(model, data)
    state = instance["randomization"]
    jid = m.mj_name2id(model, m.mjtObj.mjOBJ_JOINT, "capture_target_free")
    adr = model.jnt_qposadr[jid]
    data.qpos[adr:adr+3] = state["target_position_m"]
    data.qpos[adr+3:adr+7] = state["target_orientation_wxyz"]
    for name, value in zip([*(f"joint{i}" for i in range(1,7)), "joint_finger1", "joint_finger2"], state["arm_joint_position_rad"]):
        joint = m.mj_name2id(model, m.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    m.mj_forward(model, data)


def test_randomized_initial_target_has_clearance_from_the_robot(tmp_path, mujoco_model):
    m, model = mujoco_model
    data = m.MjData(model)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    target = m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, "capture_target")
    minimum = 10.
    for seed in range(128):
        instance = manager.create_instance(SceneInstanceCreate(seed=seed, template_id=GROUND_TARGET_TEMPLATE, randomize_orbit_phase=True))
        set_instance(m, model, data, instance)
        for contact in data.contact:
            a, b = model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]
            assert (a >= target) == (b >= target), (seed, contact.dist)
        for a in range(model.ngeom):
            if model.geom_bodyid[a] < target and model.geom_contype[a]:
                for b in range(model.ngeom):
                    if model.geom_bodyid[b] >= target and model.geom_contype[b]:
                        # This is a >10 cm clearance check, not an exact global
                        # minimum-distance measurement. Bound the query at 11 cm;
                        # MuJoCo 3.7 can return spurious zeros for distant convex
                        # pairs with a large, unnecessary distance window (1 m).
                        minimum = min(minimum, m.mj_geomDistance(model, data, a, b, .11, None))
    assert minimum > .10


def test_wing_motion_is_preserved_and_can_move_in_the_combined_model(mujoco_model):
    m, model = mujoco_model
    data = m.MjData(model)
    jid = m.mj_name2id(model, m.mjtObj.mjOBJ_JOINT, "outer_panel_hinge")
    outer = m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, "satellite_outer_panel")
    inner = m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, "satellite_inner_panel")
    m.mj_forward(model, data)
    fixed = data.xmat[inner].copy()
    before = data.xmat[outer].copy()
    data.qpos[model.jnt_qposadr[jid]] = .5
    m.mj_forward(model, data)
    assert not np.allclose(before, data.xmat[outer])
    np.testing.assert_allclose(fixed, data.xmat[inner])
    assert model.nv == 24 and model.nu == 11  # Two free bodies + 8 arm/fingers + 3 RW + one passive hinge.


def test_xml_geometry_order_matches_mjscene_render_adapter(mujoco_model):
    # The adapter joins parsed XML metadata to compiled geoms by index. Parent
    # collision proxies must precede child bodies, even though MJCF permits
    # arbitrary interleaving. Check names, not just counts/body sets.
    m, model = mujoco_model
    source_names = [g.get("name") for g in ET.parse(MODEL).findall("./worldbody/.//geom")]
    compiled_names = [m.mj_id2name(model, m.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)]
    assert source_names == compiled_names


def test_both_fingers_can_make_finite_contact_with_outer_panel(tmp_path, mujoco_model):
    # A deliberately penetrating contact fixture, NOT an autonomous approach
    # or transport demonstration. The production initialization stays clear.
    m, model = mujoco_model
    data = m.MjData(model)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=0, template_id=GROUND_TARGET_TEMPLATE, randomization_profile="none"))
    instance["randomization"]["target_position_m"] = [.74, .039086, .36]
    instance["randomization"]["arm_joint_position_rad"][-2:] = [.008, .008]
    set_instance(m, model, data, instance)
    panel = m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, "satellite_outer_panel")
    fingers = {m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, name) for name in ("finger1", "finger2")}
    touching = set()
    for i, contact in enumerate(data.contact):
        bodies = {int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])}
        if panel in bodies and bodies & fingers:
            force = np.zeros(6)
            m.mj_contactForce(model, data, i, force)
            assert np.isfinite(force).all() and force[0] > 0
            touching.update(bodies & fingers)
    assert touching == fingers
    for _ in range(10):
        m.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
