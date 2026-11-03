"""Version a preheld layout by rigidly moving the target and its loose task parts.

This changes initialization, not cameras or grasp calibration. Output is a
candidate; previous task receipts do not validate the changed layout.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulation.serial_chain_kinematics import (
    matrix_to_quaternion_wxyz as matq,
    quaternion_wxyz_to_matrix as qmat,
)
from simulation.scene_library.manifest import digest, write_json


def build(model_path, scene_path, candidate, output):
    output.mkdir(parents=True, exist_ok=False)
    tree = ET.parse(model_path)
    root = tree.getroot()
    cameras_before = [dict(n.attrib) for n in root.iter('camera')]
    target = root.find('.//body[@name="capture_target"]')
    old_p = np.fromstring(target.get('pos'), sep=' ')
    old_r = qmat(np.fromstring(target.get('quat'), sep=' '))
    new_p = np.asarray(candidate['position'])
    new_r = qmat(candidate['quaternion'])
    rotation = new_r @ old_r.T
    # These free bodies start seated in the target's task box; receiver hardware
    # and the service satellite remain in their original frames.
    names = ['capture_target', 'aviation_plug', 'guide_plug', 'cylindrical_plug', 'bolt']
    for name in names:
        body = root.find(f'.//body[@name="{name}"]')
        if body is None:
            raise ValueError(f'Missing target-owned body: {name}')
        p = np.fromstring(body.get('pos', '0 0 0'), sep=' ')
        r = qmat(np.fromstring(body.get('quat', '1 0 0 0'), sep=' '))
        body.set('pos', ' '.join(format(v, '.16g') for v in new_p + rotation @ (p-old_p)))
        body.set('quat', ' '.join(format(v, '.16g') for v in matq(rotation @ r)))
    # Existing generated models use absolute assets. Refuse ambiguous relocation.
    for node in root.iter():
        if node.get('file') and not Path(node.get('file')).is_absolute():
            raise ValueError('Resolve relative model dependencies before relocation')
    if cameras_before != [dict(n.attrib) for n in root.iter('camera')]:
        raise AssertionError('Camera attributes changed')
    model_out = output/'preheld-dual.xml'
    tree.write(model_out, encoding='utf-8', xml_declaration=True)
    shutil.copy2(model_path.with_name('attitude_control.json'), output/'attitude_control.json')
    scene = json.loads(scene_path.read_text(encoding='utf-8-sig'))
    initial = scene['randomization']
    initial['target_position_m'] = candidate['position']
    initial['target_orientation_wxyz'] = candidate['quaternion']
    initial['capture_arm_joint_position_rad'] = candidate['capture_q']
    initial['preheld_capture_joint_position'] = candidate['capture_q'] + candidate['capture_tool_tail']
    write_json(output/'initial-scene.json', scene)
    write_json(output/'layout-provenance.json', dict(
        source_model=str(model_path.resolve()), source_model_sha256=digest(model_path),
        model_sha256=digest(model_out), candidate=candidate,
        camera_attributes_unchanged=True, moved_bodies=names,
        full_task_accepted=False, docking_stereo_required=False,
    ))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--search', type=Path, required=True)
    parser.add_argument('--candidate', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    candidate = json.loads(args.search.read_text(encoding='utf-8'))['candidates'][args.candidate]
    build(args.model, args.scene, candidate, args.output)
