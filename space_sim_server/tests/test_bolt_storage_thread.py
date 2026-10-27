"""Independent helicoidal phase and provenance checks for the new storage seat."""
import math
from pathlib import Path
import xml.etree.ElementTree as E

import numpy as np
import pytest

from tools.bolt_storage_thread import LEAD_M, apply_bolt_storage_thread
from tools.build_sarm_20261012 import compose_platform_model
from tools.captive_fastener import body_frames, quaternion_product, rotation

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'model/SARM_mjcf_20261012/SARM_convex.xml'
REFERENCE = ROOT/'model/SARM/platform/sarm_task_box_plugs.xml'


def generated(tmp_path):
    path = tmp_path/'storage.xml'
    report = compose_platform_model(SOURCE, REFERENCE, path)
    return E.parse(path).getroot(), report


def test_storage_is_contact_only_and_preserves_target_and_bolt(tmp_path):
    root, report = generated(tmp_path)
    source = E.parse(SOURCE).getroot()
    a = root.find('.//geom[@name="board_thread_sdf"]')
    assert E.tostring(a) == E.tostring(source.find('.//geom[@name="board_thread_sdf"]'))
    assert root.find('.//body[@name="bolt"]').attrib == source.find('.//body[@name="bolt"]').attrib
    assert root.find('.//body[@name="bolt"]/freejoint') is not None
    storage = root.find('.//geom[@name="bolt_storage_thread_sdf"]')
    assert storage.get('mesh') == a.get('mesh')
    assert storage.find('plugin').attrib == a.find('plugin').attrib
    assert storage.get('density') == '0'
    pair = root.find('./contact/pair[@name="physical_bolt_storage_thread"]')
    original_pair = root.find('./contact/pair[@name="physical_two_start_thread"]')
    assert {k: v for k, v in pair.attrib.items() if k not in ('name', 'geom2')} == {
        k: v for k, v in original_pair.attrib.items() if k not in ('name', 'geom2')}
    assert report['bolt_storage_thread']['initial_overlap_m'] == pytest.approx(.020)
    assert report['bolt_storage_thread']['turns_to_disengage'] == pytest.approx(3.625)


def test_storage_helical_profile_aligns_at_every_angle_and_depth(tmp_path):
    root, _ = generated(tmp_path)
    frames = body_frames(root)
    def frame(body, geom):
        bp, bq = frames[body]
        node = root.find(f'.//geom[@name="{geom}"]')
        return (bp + rotation(bq) @ np.fromstring(node.get('pos'), sep=' '),
                rotation(bq) @ rotation(np.fromstring(node.get('quat', '1 0 0 0'), sep=' ')))
    mp, mr = frame('bolt', 'bolt_thread_sdf')
    fp, fr = frame('asm_board1', 'bolt_storage_thread_sdf')
    # Evaluate the delivered trigonometric surface independently of the phase
    # helper. Male/female root/crest radii must match (clearance added by SDF).
    for z in np.linspace(-.014, .004, 7):
        for angle in np.linspace(-math.pi, math.pi, 17):
            point = fp + np.array([.014*math.cos(angle), .014*math.sin(angle), z-.005])
            profiles = []
            for p, r in ((mp, mr), (fp, fr)):
                x, y, axial = r.T @ (point-p)
                u = axial-LEAD_M*math.atan2(y, x)/(2*math.pi)+.004
                profiles.append(.0135+.0015*.5*(1-math.cos(2*math.pi*u/.008)))
            assert profiles[0] == pytest.approx(profiles[1], abs=1e-14)


def test_duplicate_installation_rejected(tmp_path):
    root, _ = generated(tmp_path)
    with pytest.raises(ValueError, match='already installed'):
        apply_bolt_storage_thread(root)


def test_old_independent_layout_installs_in_storage_module_frame(tmp_path):
    root, _ = generated(tmp_path)
    board = root.find('.//body[@name="asm_board1"]')
    board.remove(board.find('geom[@name="bolt_storage_thread_sdf"]'))
    root.find('contact').remove(root.find('./contact/pair[@name="physical_bolt_storage_thread"]'))
    site = board.find('site[@name="bolt_storage_hole"]')
    board.remove(site)
    yaw = 1.13
    q = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
    module = E.SubElement(board, 'body', name='layout_bolt_storage_hole',
                          pos='.350 .290 .005', quat=' '.join(map(str, q)))
    site.set('pos', '0 0 0')
    module.append(site)
    bolt = root.find('.//body[@name="bolt"]')
    bolt.set('pos', '.475 .355 .032')
    bolt.set('quat', ' '.join(map(str, quaternion_product(q, [0, 1, 0, 0]))))
    report = apply_bolt_storage_thread(root)
    assert module.find('geom[@name="bolt_storage_thread_sdf"]') is not None
    assert report['female_installation_yaw_rad'] == pytest.approx(-math.pi/4)
    assert report['initial_overlap_m'] == pytest.approx(.020)
