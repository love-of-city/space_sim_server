from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scene_library.taskbox_mount import (
    ROOT_BODIES, LOOSE_BODIES, detach_taskbox, extract_taskbox, mount_taskbox,
    _world_pose,
    restore_owner_exclusions,
)


def fixture():
    return ET.fromstring('''<mujoco><asset><mesh name="shared_mesh"/></asset>
    <extension><plugin plugin="test.thread"><instance name="thread"><config key="mode" value="3"/></instance></plugin></extension>
    <worldbody><body name="cubesat_bus" pos=".1 .2 .3">
      <geom name="finger"/>
      <body name="task_box_base" pos=".125 .065 0"><geom name="shell" mesh="shared_mesh"/>
        <body name="board"><geom name="bore"/><site name="insert" pos=".2 .3 .1"/>
          <body name="fastener"><joint name="spin"/><joint name="slide" type="slide"/>
            <geom name="thread_geom"><plugin instance="thread"/></geom></body></body></body>
      <body name="drawer_base" pos=".455 .105 .03" quat="0 0 1 0"><joint name="drawer_slide" type="slide"/><geom name="drawer_geom"/></body>
    </body><body name="capture_target" pos="2 3 4" quat=".7071067811865476 0 .7071067811865476 0"><freejoint name="target_free"/></body>
    <body name="aviation_plug" pos=".47 .42 .332"><freejoint name="aviation_free"/><geom name="aviation_geom"/></body>
    <body name="guide_plug" pos=".38 .475 .332"><freejoint name="guide_free"/></body>
    <body name="cylindrical_plug" pos=".47 .53 .332"><freejoint name="cyl_free"/></body>
    <body name="bolt" pos=".535 .635 .332"><freejoint name="bolt_free"/><geom name="bolt_thread"/></body>
    </worldbody><equality><joint name="helix" joint1="spin" joint2="slide" polycoef="0 10 0 0 0"/></equality>
    <contact><exclude name="old_hull_drawer" body1="cubesat_bus" body2="drawer_base"/>
      <exclude name="internal" body1="drawer_base" body2="task_box_base"/>
      <exclude name="plug_board" body1="aviation_plug" body2="board"/>
      <pair name="thread_pair" geom1="bolt_thread" geom2="bore"/>
      <pair name="finger_pair" geom1="finger" geom2="bolt_thread"/>
    </contact></mujoco>''')


def test_relocates_free_parts_and_internal_sites_as_one_rigid_assembly():
    source = fixture()
    template = extract_taskbox(source)
    old = deepcopy(source)
    detach_taskbox(source)
    r = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    p = np.array([.4, -.2, .7])
    result = mount_taskbox(source, template, 'capture_target', position=p, rotation=r,
                          move_loose_parts=True)
    target = source.find('.//body[@name="capture_target"]')
    tp, tr = _world_pose(source, target)
    source_owner = old.find('.//body[@name="cubesat_bus"]')
    op, or_ = _world_pose(old, source_owner)
    for name in ROOT_BODIES + LOOSE_BODIES:
        old_body = old.find(f'.//body[@name="{name}"]')
        original_p, original_r = _world_pose(old, old_body)
        actual_p, actual_r = _world_pose(source, source.find(f'.//body[@name="{name}"]'))
        assert np.allclose(actual_p, tp+tr @ (p+r @ or_.T @ (original_p-op)))
        assert np.allclose(actual_r, tr @ r @ or_.T @ original_r)
    assert result['loose_parts'] == list(LOOSE_BODIES)
    assert source.find('./contact/exclude[@name="old_hull_drawer"]').get('body1') == 'capture_target'
    assert source.find('./contact/pair[@name="finger_pair"]').get('geom1') == 'finger'
    assert source.find('./equality/joint[@name="helix"]').get('polycoef') == '0 10 0 0 0'


def test_plugin_whitespace_is_not_a_configuration_conflict():
    root = fixture()
    template = extract_taskbox(root)
    ET.indent(root)
    detach_taskbox(root)
    mount_taskbox(root, template, 'capture_target', position=[0, 0, 0], rotation=np.eye(3), move_loose_parts=True)


def test_second_box_has_independent_mechanisms_and_plugin_instances_but_shared_assets():
    root = fixture()
    template = extract_taskbox(root)
    detach_taskbox(root)
    mount_taskbox(root, template, 'capture_target', position=[0, 0, 0], rotation=np.eye(3), move_loose_parts=True)
    before_meshes = len(root.findall('./asset/mesh'))
    result = mount_taskbox(root, template, 'cubesat_bus', position=[0, 0, 0], rotation=np.eye(3), prefix='service_')
    assert len(root.findall('./asset/mesh')) == before_meshes
    assert root.find('.//geom[@name="service_shell"]').get('mesh') == 'shared_mesh'
    assert root.find('.//body[@name="service_aviation_plug"]') is None
    assert root.find('./equality/joint[@name="service_helix"]').get('joint1') == 'service_spin'
    assert root.find('.//geom[@name="service_thread_geom"]/plugin').get('instance') == 'service_thread'
    assert root.find('./extension/plugin/instance[@name="service_thread"]/config').attrib == {'key': 'mode', 'value': '3'}
    exclusion = root.find('./contact/exclude[@name="service_old_hull_drawer"]')
    assert (exclusion.get('body1'), exclusion.get('body2')) == ('cubesat_bus', 'service_drawer_base')
    pair = root.find('./contact/pair[@name="service_thread_pair"]')
    assert pair.get('geom1') == 'bolt_thread'
    assert pair.get('geom2') == 'service_bore'
    assert result['loose_parts'] == []


def test_rejects_repeated_install_and_nonrigid_transform():
    root = fixture()
    template = extract_taskbox(root)
    with pytest.raises(ValueError, match='already installed'):
        mount_taskbox(root, template, 'capture_target', position=[0, 0, 0], rotation=np.eye(3))
    with pytest.raises(ValueError, match='rigid transform'):
        mount_taskbox(root, template, 'capture_target', position=[0, 0, 0], rotation=np.eye(3)*2, prefix='new_')


def test_delivered_source_copies_complete_hardware_and_resolves_references():
    path = Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml'
    root = ET.parse(path).getroot()
    template = extract_taskbox(root)
    original_body_count = sum(len(list(body.iter('body'))) for body in template.roots)
    detach_taskbox(root)
    mount_taskbox(root, template, 'capture_target', position=[.2, .3, .4], rotation=np.eye(3), move_loose_parts=True)
    result = mount_taskbox(root, template, 'cubesat_bus', position=[0, 0, .1], rotation=np.eye(3), prefix='service_')
    assert sum(len(list(root.find(f'.//body[@name="service_{name}"]').iter('body'))) for name in ROOT_BODIES) == original_body_count
    assert root.find('./equality/joint[@name="service_fastener_helix"]').get('joint2') == 'service_fastener_slide'
    assert root.find('.//joint[@name="service_drawer_base_slide"]') is not None
    assert root.find('.//site[@name="service_aviation_insert_40p1"]') is not None
    assert root.find('./contact/pair[@name="service_physical_two_start_thread"]').get('geom1') == 'bolt_thread_sdf'
    for suffix in ('drawer_base', 'drawer_buckle'):
        exclusion = root.find(f'./contact/exclude[@name="service_base_link_{suffix}_exclude"]')
        assert (exclusion.get('body1'), exclusion.get('body2')) == ('cubesat_bus', 'service_'+suffix)
        target_exclusion = root.find(f'./contact/exclude[@name="base_link_{suffix}_exclude"]')
        assert (target_exclusion.get('body1'), target_exclusion.get('body2')) == ('capture_target', suffix)
    assert result['plugin_instances']['fastener_female_sdf'] == 'service_fastener_female_sdf'
    for name in LOOSE_BODIES:
        assert len(root.findall(f'.//body[@name="{name}"]')) == 1
    # The baseline-reuse path must recover exactly the same mapped rules.
    rule=root.find('./contact/exclude[@name="service_base_link_drawer_buckle_exclude"]')
    expected=dict(rule.attrib);root.find('contact').remove(rule)
    restore_owner_exclusions(root,template,'cubesat_bus',prefix='service_')
    restore_owner_exclusions(root,template,'cubesat_bus',prefix='service_')
    matches=root.findall('./contact/exclude[@name="service_base_link_drawer_buckle_exclude"]')
    assert len(matches)==1 and matches[0].attrib==expected
