from copy import deepcopy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from verify_preserved_scene_geometry import verify


def files(tmp_path, mutation=None):
    root = ET.fromstring('''<mujoco><default><geom friction="1 .1 .01"/></default>
      <asset><mesh name="port" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 3 0 2 3 1 2 3"/></asset>
      <worldbody><body name="cubesat_bus"><camera name="spacecraft_overview" pos="-.18 .27 .5"/>
        <body name="link1" pos=".2 -.2 .25"><joint name="joint1"/></body>
        <body name="bottom_mount" pos=".4 -.1 -.5"><body name="bottom_link1"><joint name="bottom_joint1"/></body></body>
      </body><body name="capture_target" pos="1 0 0"><freejoint name="target_free"/>
        <geom name="target_instance_0030_part_015_color_00" type="mesh" mesh="port"/>
      </body></worldbody></mujoco>''')
    target = root.find('.//body[@name="capture_target"]')
    for i in range(898):
        ET.SubElement(target, 'geom', name=f'capture_x_piece_{i}', type='box', size='.01 .02 .03',
                      pos='.175 .073 .042', contype='2', conaffinity='1')
    original = tmp_path/'source.xml'
    ET.ElementTree(root).write(original)
    candidate_root = deepcopy(root)
    if mutation:
        mutation(candidate_root)
    candidate = tmp_path/'candidate.xml'
    ET.ElementTree(candidate_root).write(candidate)
    return original, candidate


@pytest.mark.parametrize(('mutation', 'failed_check'), [
    (lambda r: r.find('.//body[@name="link1"]').set('pos', '0 0 0'), 'arms_preserved_against_current_baseline'),
    (lambda r: r.find('.//geom[@name="capture_x_piece_0"]').set('contype', '0'), 'protected_original_interface_preserved'),
    (lambda r: r.find('./asset/mesh').set('scale', '2 2 2'), 'original_satellite_uniform_geometry_preserved'),
    (lambda r: r.find('.//camera').set('pos', '0 0 1'), 'cameras_preserved_against_current_baseline'),
])
def test_detects_changes_without_loading_a_simulator(tmp_path, mutation, failed_check):
    source, candidate = files(tmp_path, mutation)
    report = verify(source, source, candidate, 1.)
    assert report['checks'][failed_check] is False
    assert report['passed'] is False


def test_preserved_model_passes_but_changed_initial_q_does_not(tmp_path):
    source, candidate = files(tmp_path)
    initial = tmp_path/'initial.json'
    changed = tmp_path/'changed.json'
    initial.write_text(json.dumps({'randomization': {'arm_joint_position_rad': [0]*8}}))
    changed.write_text(json.dumps({'randomization': {'arm_joint_position_rad': [1]*8}}))
    assert verify(source, source, candidate, 1., baseline_scene=initial, candidate_scene=initial)['passed']
    report = verify(source, source, candidate, 1., baseline_scene=initial, candidate_scene=changed)
    assert report['checks']['operating_arm_initial_q_preserved'] is False


@pytest.mark.parametrize('fault', [None, 'support', 'joint', 'owner'])
def test_original_operating_mount_restoration_is_exact(tmp_path, fault):
    source, candidate = files(tmp_path)
    baseline_root = ET.parse(source).getroot()
    baseline_root.find('.//body[@name="link1"]').set('pos', '.3 -.3 .4')
    ET.SubElement(baseline_root.find('.//body[@name="cubesat_bus"]'), 'body', name='operating_arm_support')
    baseline = tmp_path/'baseline.xml'
    ET.ElementTree(baseline_root).write(baseline)
    root = ET.parse(candidate).getroot()
    if fault == 'support':
        ET.SubElement(root.find('.//body[@name="cubesat_bus"]'), 'body', name='operating_arm_support')
    elif fault == 'joint':
        root.find('.//body[@name="link1"]/joint').set('axis', '1 0 0')
    elif fault == 'owner':
        arm = root.find('.//body[@name="link1"]')
        root.find('.//body[@name="cubesat_bus"]').remove(arm)
        root.find('.//body[@name="capture_target"]').append(arm)
    ET.ElementTree(root).write(candidate)
    report = verify(source, baseline, candidate, 1., restore_operating_mount=True)
    assert report['passed'] is (fault is None)


@pytest.mark.parametrize('fault', [None, 'wrong_mount', 'changed_joint', 'operating_on_target'])
def test_capture_mount_override_preserves_arm_and_service_ownership(tmp_path, fault):
    pose = [.6, .2, -.35, 0., 2**-.5, 0., 2**-.5]
    def mutate(root):
        mount = root.find('.//body[@name="bottom_mount"]')
        mount.set('pos', '.6 .2 -.35' if fault != 'wrong_mount' else '.7 .2 -.35')
        mount.set('quat', ' '.join(map(str, pose[3:])))
        if fault == 'changed_joint':
            mount.find('.//joint').set('axis', '1 0 0')
        if fault == 'operating_on_target':
            service = root.find('.//body[@name="cubesat_bus"]')
            arm = service.find('body[@name="link1"]')
            service.remove(arm)
            root.find('.//body[@name="capture_target"]').append(arm)
    source, candidate = files(tmp_path, mutate)
    assert not verify(source, source, candidate, 1.)['passed']
    report = verify(source, source, candidate, 1., expected_capture_mount=pose)
    assert report['passed'] is (fault is None)
    assert report['arms']['bottom_mount']['candidate_matches_current_baseline'] is False


def test_real_housing_contract_keeps_original_port_and_current_arm_baseline(tmp_path):
    from simulation.scene_library.preserved_target import resize_housing
    source = Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml'
    authored = ET.parse(source).getroot()
    baseline_root = deepcopy(authored)
    baseline_root.find('.//body[@name="link1"]').set('pos', '.2 -.2 .25')
    baseline = tmp_path/'baseline.xml'
    ET.ElementTree(baseline_root).write(baseline)
    candidate_root, _ = resize_housing(baseline_root)
    candidate_root.find('compiler').set('meshdir', str(source.parent))
    candidate = tmp_path/'housing.xml'
    ET.ElementTree(candidate_root).write(candidate)
    report = verify(source, baseline, candidate, 5.6, housing_only=True)
    assert report['passed'], report['differences'][:3]
    assert report['arms']['link1']['current_baseline_matches_source'] is False
    candidate_root.find('.//geom[@name="capture_x_piece_0"]').set('pos', '0 0 0')
    ET.ElementTree(candidate_root).write(candidate)
    report = verify(source, baseline, candidate, 5.6, housing_only=True)
    assert report['checks']['protected_original_interface_preserved'] is False
