"""Episode rest transforms must preserve the original loosened mechanism pose."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.randomization import write_model_variant
from simulation.model_preparation.captive_fastener import body_frames, quaternion_product, rotation
from simulation.model_preparation.threaded_task_initialization import LOCKED_TRAVEL_M, apply_locking_cycle_initial_state

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT/'model/SARM/platform/sarm_20261012.xml'


def test_joint_refs_and_body_rest_pose_represent_same_screw_motion():
    root = ET.parse(MODEL).getroot()
    before = body_frames(root)
    report = apply_locking_cycle_initial_state(root)
    after = body_frames(root)
    old_p, old_q = before['fastener']
    new_p, new_q = after['fastener']
    travel = LOCKED_TRAVEL_M
    spin = 2*math.pi/.016*travel
    np.testing.assert_allclose(new_p, old_p+rotation(old_q)[:, 2]*travel, atol=1e-12)
    c, s = math.cos(spin), math.sin(spin)
    np.testing.assert_allclose(rotation(new_q), rotation(old_q)@np.array([[c,-s,0],[s,c,0],[0,0,1]]), atol=1e-12)
    assert float(root.find('.//joint[@name="fastener_slide"]').get('ref')) == travel
    assert float(root.find('.//joint[@name="fastener_spin"]').get('ref')) == pytest.approx(spin)
    assert root.find('.//joint[@name="locking_rod_hinge"]').get('range').split()[0] == '-1.75'
    for name in before.keys()-{'fastener'}:
        for a, b in zip(before[name], after[name], strict=True):
            np.testing.assert_allclose(a, b, atol=1e-12)
    assert report['runtime_state_writes'] is False
    state = ET.tostring(root)
    apply_locking_cycle_initial_state(root)
    assert ET.tostring(root) == state


def test_only_locking_task_gets_locked_initial_pose(tmp_path):
    original = MODEL.read_bytes()
    assert write_model_variant(MODEL, tmp_path/'drawer.xml', {'scripted_task_name':'drawer_cycle'}) == MODEL
    scene = {'scripted_task_name':'locking_rod_cycle'}
    output = write_model_variant(MODEL, tmp_path/'locked.xml', scene)
    tree = ET.parse(output)
    assert float(tree.find('.//joint[@name="fastener_slide"]').get('ref')) == LOCKED_TRAVEL_M
    assert MODEL.read_bytes() == original
    assert scene['collection_config']['model_randomization']['task_initialization']['locking_rod']['runtime_state_writes'] is False


def test_old_bolt_model_gets_storage_thread_before_preflight(tmp_path):
    # Existing randomized-layout pool files predate the new source-bore SDF.
    scene = {'scripted_task_name':'aviation_bolt_insert'}
    output = write_model_variant(MODEL, tmp_path/'bolt.xml', scene)
    tree = ET.parse(output)
    assert tree.find('.//geom[@name="bolt_storage_thread_sdf"]') is not None
    assert tree.find('.//joint[@name="fastener_slide"]').get('ref') is None


def test_old_marker_still_repairs_contacts_without_duplicate_meshes_or_pairs():
    import copy
    root = ET.parse(MODEL).getroot()
    coarse = copy.deepcopy(root.find('.//geom[@name="task_box_base_geom_collision_016"]'))
    apply_locking_cycle_initial_state(root)
    base = root.find('.//body[@name="task_box_base"]')
    for geom in list(base.findall('geom')):
        if geom.get('name', '').startswith('locking_precision_base_'):
            base.remove(geom)
    base.insert(0, coarse)
    contacts = root.find('contact')
    for pair in list(contacts):
        if pair.get('name', '').startswith('locking_clip_contact_'):
            contacts.remove(pair)
    report = apply_locking_cycle_initial_state(root)
    assert report['physical_contacts']['clip_explicit_contact_pairs'] == 960
    assert report['physical_contacts']['base_false_hull_replacement_count'] > 0
    assert root.find('.//geom[@name="task_box_base_geom_collision_016"]') is None
    state = ET.tostring(root)
    report = apply_locking_cycle_initial_state(root)
    assert ET.tostring(root) == state
    for tag in ('./asset/mesh', './contact/pair'):
        names = [node.get('name') for node in root.findall(tag)]
        assert len(names) == len(set(names))


def test_collision_assets_have_complete_reproducible_provenance():
    import hashlib
    import json
    folder = ROOT/'model/SARM/platform/mechanism_collision'
    manifest = json.loads((folder/'locking_sweep_base.json').read_text())
    assert manifest['schema'] == 'locking-sweep-collision/1'
    assert manifest['relative_volume_error'] < 2e-5
    for name in manifest['files']:
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest() == manifest['sha256'][name]


def test_older_initial_state_migrates_without_changing_original_loose_pose():
    root = ET.parse(MODEL).getroot()
    apply_locking_cycle_initial_state(root)
    expected = body_frames(root)['fastener']
    nut = root.find('.//body[@name="locking_rod"]/body[@name="fastener"]')
    old_travel = .0051
    old_spin = old_travel*2*math.pi/.016
    current_spin = LOCKED_TRAVEL_M*2*math.pi/.016
    quat = np.fromstring(nut.get('quat'), sep=' ')
    position = np.fromstring(nut.get('pos'), sep=' ')
    position += rotation(quat)[:, 2]*(old_travel-LOCKED_TRAVEL_M)
    delta = old_spin-current_spin
    quat = quaternion_product(quat, [math.cos(delta/2), 0, 0, math.sin(delta/2)])
    nut.set('pos', ' '.join(format(float(v), '.17g') for v in position))
    nut.set('quat', ' '.join(format(float(v), '.17g') for v in quat))
    nut.find('joint[@name="fastener_slide"]').set('ref', str(old_travel))
    nut.find('joint[@name="fastener_spin"]').set('ref', str(old_spin))
    root.find('./custom/numeric[@name="scripted_locking_cycle_initial_state"]').set(
        'data', f'{old_travel} {old_spin} -1.75 .15')
    report = apply_locking_cycle_initial_state(root)
    assert report['source'] == 'migrated_episode_initial_state'
    actual = body_frames(root)['fastener']
    for first, second in zip(expected, actual, strict=True):
        np.testing.assert_allclose(first, second, atol=1e-12)
    state = ET.tostring(root)
    apply_locking_cycle_initial_state(root)
    assert ET.tostring(root) == state
