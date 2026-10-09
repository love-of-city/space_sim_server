"""Coarse self-contact on the runtime-matched MuJoCo; separate process from BSK.

No original CAD/visuals/inertia are changed. Static interpenetration checks and
continuous finite-speed/torque checks are distinct; neither proves arbitrary-speed
collision accuracy or real mechanical hinge travel.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager
from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, SELF_COLLISION_TEMPLATE

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'model/SARM/platform/sarm_ground_target.xml'
NEW = OLD.with_name('sarm_ground_target_self_collision.xml')
spec = importlib.util.spec_from_file_location('coarse_contact_builder', ROOT / 'tools/build_sarm_ground_target.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)
PAIRS = {
    frozenset(('capture_target_collision', 'satellite_outer_panel_internal_collision')),
    frozenset(('satellite_inner_panel_internal_collision', 'satellite_outer_panel_internal_collision')),
}


def standalone_xml():
    full = ET.parse(NEW).getroot()
    root = ET.Element('mujoco', model='target_self_contact_test')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false', fusestatic='false')
    root.append(copy.deepcopy(full.find('option')))
    target = copy.deepcopy(full.find('.//body[@name="capture_target"]'))
    target.set('pos', '0 0 0')
    target.set('quat', '1 0 0 0')
    for body in target.iter('body'):
        for geom in list(body.findall('geom')):
            if geom.get('group') != '3':
                body.remove(geom)
    ET.SubElement(root, 'worldbody').append(target)
    root.append(copy.deepcopy(full.find('contact')))
    return root


@pytest.fixture(scope='module')
def isolated_model():
    m = pytest.importorskip('mujoco')
    return m, m.MjModel.from_xml_string(ET.tostring(standalone_xml(), encoding='unicode'))


@pytest.fixture(scope='module')
def full_model():
    m = pytest.importorskip('mujoco')
    return m, m.MjModel.from_xml_path(str(NEW))


def contact_names(m, model, contact):
    return frozenset(model.geom(int(i)).name for i in (contact.geom1, contact.geom2))


def test_generator_preserves_legacy_artifacts_and_reproduces_self_contact_variant():
    for internal, path in ((False, OLD), (True, NEW)):
        xml, manifest = builder.composed_bytes(target_self_collision=internal)
        assert xml == path.read_bytes().replace(b'\r\n', b'\n')
        assert manifest == path.with_suffix('.manifest.json').read_bytes().replace(b'\r\n', b'\n')
    metadata = json.loads(NEW.with_suffix('.manifest.json').read_text())
    assert metadata['target_internal_collision_enabled']
    assert metadata['internal_contact']['engineering_hinge_clearance_known'] is False
    assert metadata['internal_contact']['hinge_keepout_each_side_m'] == pytest.approx(.01595001)
    assert metadata['internal_contact']['zero_pose_internal_panel_gap_m'] == pytest.approx(.03190002)


def test_visuals_external_shapes_mass_arm_wheels_cameras_and_joint_limits_are_unchanged():
    old, new = ET.parse(OLD).getroot(), ET.parse(NEW).getroot()
    for selector in ('.//body[@name="cubesat_bus"]', './actuator', './custom'):
        assert ET.tostring(old.find(selector)) == ET.tostring(new.find(selector))
    old_target = old.find('.//body[@name="capture_target"]')
    new_target = new.find('.//body[@name="capture_target"]')
    assert [g.attrib for g in old_target.findall('.//geom[@group="2"]')] == [g.attrib for g in new_target.findall('.//geom[@group="2"]')]
    assert len(new_target.findall('.//geom[@group="2"]')) == 208
    assert [e.attrib for e in old_target.findall('.//inertial')] == [e.attrib for e in new_target.findall('.//inertial')]
    assert [e.attrib for e in old_target.findall('.//joint')] == [e.attrib for e in new_target.findall('.//joint')]
    for geom in old_target.findall('.//geom[@group="3"]'):
        expected = dict(geom.attrib, contype='2')
        assert new_target.find(f'.//geom[@name="{geom.get("name")}"]').attrib == expected
    assert len(new_target.findall('.//geom[@group="3"]')) == 5
    assert new.findall('.//flexcomp') == []
    assert not new.findall('./contact/exclude')
    assert len(new.findall('./contact/pair')) == 2
    assert {frozenset((p.get('geom1'), p.get('geom2'))) for p in new.findall('./contact/pair')} == PAIRS
    assert new_target.find('.//joint[@name="outer_panel_hinge"]').get('limited') == 'false'


@pytest.mark.parametrize('angle_deg', [-90, -45, -1, 0, 1, 45, 90])
def test_initial_and_near_hinge_motion_has_no_false_contact(isolated_model, angle_deg):
    m, model = isolated_model
    data = m.MjData(model)
    data.qpos[model.joint('outer_panel_hinge').qposadr[0]] = math.radians(angle_deg)
    m.mj_forward(model, data)
    assert data.ncon == 0
    assert model.nq == 8 and model.nv == 7 and model.nu == 0 and model.ngeom == 5 and model.nflex == 0
    assert not model.opt.disableflags & m.mjtDisableBit.mjDSBL_FILTERPARENT


def test_both_explicit_pairs_detect_and_react_to_static_overlap(isolated_model):
    m, model = isolated_model
    forces = {pair: 0. for pair in PAIRS}
    for angle in (180, 190, 200):
        data = m.MjData(model)
        data.qpos[model.joint('outer_panel_hinge').qposadr[0]] = math.radians(angle)
        m.mj_forward(model, data)
        for index, contact in enumerate(data.contact):
            pair = contact_names(m, model, contact)
            assert pair in PAIRS
            force = np.zeros(6)
            m.mj_contactForce(model, data, index, force)
            assert np.isfinite(force).all()
            forces[pair] = max(forces[pair], float(force[0]))
    assert all(value > 0 for value in forces.values()), forces


def test_no_internal_contact_without_pairs_even_when_excludes_are_removed():
    m = pytest.importorskip('mujoco')
    root = standalone_xml()
    root.remove(root.find('contact'))
    # Reproduce "just remove excludes" with the original compatible masks:
    # MuJoCo still filters this parent/weld group by default.
    for geom in root.findall('.//geom'):
        geom.set('contype', '1')
        geom.set('conaffinity', '1')
    model = m.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = m.MjData(model)
    data.qpos[model.joint('outer_panel_hinge').qposadr[0]] = math.pi
    m.mj_forward(model, data)
    assert data.ncon == 0


def test_rest_does_not_generate_a_startup_impulse(isolated_model):
    m, model = isolated_model
    data = m.MjData(model)
    initial = data.qpos.copy()
    for _ in range(500):
        m.mj_step(model, data)
        assert data.ncon == 0
    np.testing.assert_allclose(data.qpos, initial, atol=1e-12, rtol=0)
    np.testing.assert_allclose(data.qvel, 0., atol=1e-12, rtol=0)


@pytest.mark.parametrize('speed,torque', [(1., 0.), (-1., 0.), (5., 0.), (-5., 0.), (0., .1), (0., -.1)])
def test_continuous_motion_and_sustained_torque_are_blocked_by_contact(isolated_model, speed, torque):
    m, model = isolated_model
    data = m.MjData(model)
    joint = model.joint('outer_panel_hinge')
    qa, va = int(joint.qposadr[0]), int(joint.dofadr[0])
    data.qvel[va] = speed
    minimum = maximum = 0.
    minimum_distance = 0.
    max_force = 0.
    for _ in range(5000):  # 10 s at the unchanged 2 ms dynamics step.
        data.qfrc_applied[va] = torque
        m.mj_step(model, data)
        minimum = min(minimum, float(data.qpos[qa]))
        maximum = max(maximum, float(data.qpos[qa]))
        for index, contact in enumerate(data.contact):
            assert contact_names(m, model, contact) in PAIRS
            minimum_distance = min(minimum_distance, float(contact.dist))
            force = np.zeros(6)
            m.mj_contactForce(model, data, index, force)
            assert np.isfinite(force).all()
            max_force = max(max_force, float(force[0]))
    assert max_force > 0
    assert -math.pi < minimum <= maximum < math.pi  # Cannot pass through for a full revolution.
    assert minimum_distance > -.001  # Bound measured solver penetration, not CAD accuracy.
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert sum(int(w.number) for w in data.warning) == 0


def set_instance(m, model, data, instance):
    m.mj_resetData(model, data)
    state = instance['randomization']
    adr = model.joint('capture_target_free').qposadr[0]
    data.qpos[adr:adr+3] = state['target_position_m']
    data.qpos[adr+3:adr+7] = state['target_orientation_wxyz']
    for name, value in zip([*(f'joint{i}' for i in range(1, 7)), 'joint_finger1', 'joint_finger2'], state['arm_joint_position_rad']):
        data.qpos[model.joint(name).qposadr[0]] = value
    m.mj_forward(model, data)


def test_full_scene_has_valid_compiled_render_order_and_clear_random_initialization(full_model, tmp_path):
    m, model = full_model
    assert DEFAULT_TEMPLATE == SELF_COLLISION_TEMPLATE
    source_names = [g.get('name') for g in ET.parse(NEW).findall('./worldbody/.//geom')]
    assert source_names == [m.mj_id2name(model, m.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)]
    assert model.nq == 26 and model.nflex == 0 and model.npair == 2
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    data = m.MjData(model)
    target = model.body('capture_target').id
    for seed in range(128):
        set_instance(m, model, data, manager.create_instance(SceneInstanceCreate(seed=seed)))
        for contact in data.contact:
            assert all(model.geom_bodyid[i] < target for i in (contact.geom1, contact.geom2))
        assert np.isfinite(data.qpos).all()


def test_full_scene_keeps_both_finger_contacts(full_model, tmp_path):
    m, model = full_model
    data = m.MjData(model)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=0, randomization_profile='none'))
    instance['randomization']['target_position_m'] = [.74, .039086, .36]
    instance['randomization']['arm_joint_position_rad'][-2:] = [.008, .008]
    set_instance(m, model, data, instance)
    panel = model.body('satellite_outer_panel').id
    fingers = {model.body(name).id for name in ('finger1', 'finger2')}
    touching = set()
    for i, contact in enumerate(data.contact):
        bodies = {int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])}
        if panel in bodies and bodies & fingers:
            force = np.zeros(6)
            m.mj_contactForce(model, data, i, force)
            assert np.isfinite(force).all() and force[0] > 0
            touching.update(bodies & fingers)
    assert touching == fingers
