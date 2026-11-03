"""Register verified task instances in the existing platform, preserving its cameras."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'backend'))
from space_arm_platform.imported_scene_library import digest, read, verified_source
from space_arm_platform.scene_targets import capture_target


def cameras(path):
    result = {}
    def visit(node, parent=''):
        if node.tag == 'body':
            parent = node.get('name', '')
        if node.tag == 'camera':
            result[node.get('name')] = {'parent':parent, 'attributes':dict(node.attrib)}
        for child in node:
            visit(child, parent)
    visit(ET.parse(path).getroot())
    return result


def install(library_path, replay_root, output):
    library = read(library_path)
    if not library.get('phase_one_accepted') or len(library['entries']) != 14:
        raise ValueError('Import requires the complete accepted first-stage library')
    entries = []
    for item in library['entries']:
        source_path = library_path.parent/item['instance']
        if digest(source_path) != item['instance_sha256']:
            raise ValueError('Accepted instance changed')
        source = read(source_path)
        family = source['recipe']['family_number']
        base = ('sarm-ground-validation-self-collision-grasp' if family == 3 else
                'sarm-dual-capture26' if family in (2,4) else 'sarm-task-box-20261012')
        baseline = ROOT/capture_target(base).runtime_model
        expected = cameras(baseline)
        actual = cameras(source['model'])
        if actual != expected:
            raise ValueError(f'Model cameras differ from platform baseline: {item["scene_id"]}')
        entry = dict(id='library/'+item['scene_id'], label='场景库 · '+item['display_name'],
            description='已验收任务的完整场景；沿用系统相机，加载后实时求解。双臂任务从已抓持初态开始。',
            kind='rendezvous' if family==3 else 'operation', base_template_id=base,
            source_instance=str(source_path.resolve()), source_instance_sha256=digest(source_path),
            receipt=item['receipt'], receipt_sha256=item['receipt_sha256'],
            camera_baseline=str(baseline), camera_baseline_sha256=digest(baseline),
            camera_contract=expected, scene_id=item['scene_id'])
        if family == 3:
            from space_arm_platform.rendezvous_config import FlightOptions
            report = read(item['native_report'])
            entry['flight_options'] = FlightOptions.from_dict(report.get('mission_options', {'seed':7})).to_dict()
        else:
            meta = read(replay_root/(item['scene_id'].replace('/','--')+'.json'))
            entry.update(catalog=meta['catalog'], catalog_sha256=digest(meta['catalog']))
        verified_source(entry)
        entries.append(entry)
    output.parent.mkdir(parents=True, exist_ok=True)
    index = dict(schema='space-arm-platform-library/1', source_library=str(library_path.resolve()),
                 source_library_sha256=digest(library_path), entries=entries)
    output.write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return {'imported':len(entries), 'output':str(output), 'camera_baselines_unchanged':True}


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--replay-root',type=Path,required=True,help='Existing hash-matched UE asset catalogs; no replay is launched')
    parser.add_argument('--output',type=Path,default=ROOT/'run/imported_scene_library.json')
    args=parser.parse_args()
    print(json.dumps(install(args.library,args.replay_root,args.output)))
