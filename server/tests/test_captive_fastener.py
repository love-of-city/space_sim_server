from pathlib import Path
from types import SimpleNamespace
import math
import xml.etree.ElementTree as E
import pytest
import numpy as np

from tools.assets.build_sarm_20261012 import compose_platform_model
from space_arm_platform.task_box import initialize_free_plugs

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'model/SARM_mjcf_20261012/SARM_convex.xml'
REFERENCE = ROOT / 'model/SARM/platform/sarm_task_box_plugs.xml'


def test_captive_model_has_screw_motion_and_retention(tmp_path):
    out = tmp_path / 'captive.xml'
    record = compose_platform_model(SOURCE, REFERENCE, out, fastener_model='captive')
    root = E.parse(out)
    body = root.find('.//body[@name="locking_rod"]/body[@name="fastener"]')
    assert body is not None
    assert body.find('freejoint') is None
    slide = body.find('joint[@name="fastener_slide"]')
    low, high = map(float, slide.get('range').split())
    assert low == 0 and .05 < high < .06
    hinge = body.find('joint[@name="fastener_spin"]')
    assert hinge.get('limited') == 'false'
    assert float(hinge.get('frictionloss')) > 0
    coupling = root.find('./equality/joint[@name="fastener_helix"]')
    assert coupling.get('joint1') == 'fastener_spin'
    assert coupling.get('joint2') == 'fastener_slide'
    assert math.isclose(float(coupling.get('polycoef').split()[1]), 2*math.pi/.016)
    assert root.find('./contact/pair[@name="physical_locking_thread"]') is None
    assert root.find('./contact/pair[@name="locking_rod_fastener_radial_guide"]') is None
    assert root.find('./contact/pair[@name="physical_two_start_thread"]') is not None
    assert record['fastener']['model'] == 'captive'
    assert record['fastener']['geometry']['retainer_radius_m'] > .012


def test_sdf_comparison_keeps_original_free_joint_and_pairs(tmp_path):
    before = SOURCE.read_bytes()
    out = tmp_path / 'sdf.xml'
    compose_platform_model(SOURCE, REFERENCE, out, fastener_model='sdf')
    root = E.parse(out)
    assert root.find('./worldbody/body[@name="fastener"]/freejoint') is not None
    assert root.find('./contact/pair[@name="physical_locking_thread"]') is not None
    assert SOURCE.read_bytes() == before


def test_reparenting_preserves_all_world_poses_and_grasp_geometries(tmp_path):
    from simulation.model_preparation.captive_fastener import body_frames, rotation
    roots = {}
    for mode in ('sdf', 'captive'):
        out = tmp_path / (mode+'.xml')
        compose_platform_model(SOURCE, REFERENCE, out, fastener_model=mode)
        roots[mode] = E.parse(out).getroot()
    a, b = (body_frames(roots[mode]) for mode in ('sdf', 'captive'))
    assert set(a) == set(b)
    for name in a:
        assert np.allclose(a[name][0], b[name][0], atol=1e-12, rtol=0)
        assert np.allclose(rotation(a[name][1]), rotation(b[name][1]), atol=1e-12, rtol=0)
    for name in ('fastener', 'locking_rod'):
        geoms = [[E.tostring(g) for g in roots[mode].find(f'.//body[@name="{name}"]').findall('geom')]
                 for mode in ('sdf', 'captive')]
        assert geoms[0] == geoms[1]


def test_unknown_variant_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='fastener_model'):
        compose_platform_model(SOURCE, REFERENCE, tmp_path/'bad.xml', fastener_model='unknown')


def test_unrecognized_attached_part_is_not_silently_initialized(tmp_path):
    out = tmp_path/'bad.xml'
    out.write_text('<mujoco><worldbody><body name="fastener"/></worldbody></mujoco>')
    scene = SimpleNamespace(getBodyNames=lambda: ['fastener'])
    with pytest.raises(ValueError, match='independent world body'):
        initialize_free_plugs(scene, out, [0, 0, 0], [0, 0, 0])


def test_attached_fastener_inherits_orbit_without_world_pose_write(tmp_path):
    out = tmp_path / 'captive.xml'
    compose_platform_model(SOURCE, REFERENCE, out, fastener_model='captive')
    writes = []
    scene = SimpleNamespace(getBodyNames=lambda: ['fastener', 'bolt'], getBody=lambda name:
        SimpleNamespace(setPosition=lambda x: writes.append((name, 'pos')),
                        setVelocity=lambda x: writes.append((name, 'vel'))))
    initialize_free_plugs(scene, out, [100, 200, 300], [1, 2, 3])
    assert writes == [('bolt', 'pos'), ('bolt', 'vel')]
