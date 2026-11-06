"""Normalize the delivered V2 left/right names once, preserving physical geometry."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = 'afaf8a14de98c1be7044751bfa59a08de938c02e'


def prepare(source, output, workspace_pose):
    source, output = Path(source).resolve(), Path(output).resolve()
    design = json.loads((source.parent/'design.json').read_text())
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if source_hash != design['model_sha256']:
        raise ValueError('Delivered dual-arm source hash mismatch')
    tree = ET.parse(source)
    root = tree.getroot()
    def rename(token):
        if token.startswith('left_'): return 'right_' + token[5:]
        if token.startswith('right_'): return 'left_' + token[6:]
        return token
    for node in root.iter():
        for key, value in list(node.attrib.items()):
            if key != 'file':
                node.set(key, ' '.join(rename(t) for t in value.split()))
    compiler = root.find('compiler')
    meshdir = source.parent / compiler.get('meshdir', '.')
    for mesh in root.findall('./asset/mesh[@file]'):
        mesh.set('file', os.path.relpath(meshdir/mesh.get('file'), output.parent))
    compiler.set('meshdir', '.')
    root.set('model', 'dual_arm_capture_v2')
    # Renaming does not reorder the native qpos layout or move any body.
    pose = json.loads(Path(workspace_pose).read_text())['qpos']
    cursor, joints = 0, {}
    for body in root.iter('body'):
        for node in body:
            if node.tag not in ('joint', 'freejoint'): continue
            width = 7 if node.tag == 'freejoint' or node.get('type') == 'free' else 4 if node.get('type') == 'ball' else 1
            joints[node.get('name')] = pose[cursor:cursor+width]
            cursor += width
    if cursor != len(pose): raise ValueError('Workspace pose has wrong qpos dimension')
    for side, sign in [('left', -1), ('right', 1)]:
        position = list(map(float, root.find(f'.//body[@name="{side}_link1"]').get('pos').split()))
        if sign*position[0] <= 0: raise ValueError('Arm side disagrees with fixed overview orientation')
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    identity = dict(schema='dual-arm-collection-model/1', source_commit=SOURCE_COMMIT,
        source_sha256=source_hash, model_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        arm_order=['left','right'], left_body_axis='-X', right_body_axis='+X',
        side_definition='fixed overview camera facing the workbench',
        workspace_joint_positions=joints, base_mass_kg=design['mass_assumption_kg'],
        attitude_control='passive free spacecraft; source model has no reaction wheels')
    output.with_suffix('.json').write_text(json.dumps(identity,indent=2)+'\n')
    return identity


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'model/dual_stowed_service/camera_version_2/dual_stowed_service.xml')
    parser.add_argument('--output',type=Path,default=ROOT/'model/dual_stowed_service/collection_v2/dual_arm_capture.xml')
    parser.add_argument('--workspace-pose',type=Path,default=ROOT/'model/dual_stowed_service/workspace_pose.json')
    args=parser.parse_args();print(json.dumps(prepare(args.source,args.output,args.workspace_pose),indent=2))
