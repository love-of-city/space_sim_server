"""Verify and relocate the saved model into a new runtime directory.

Does not import UE assets or modify the currently installed platform index.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'model/scene_library/original-root-v2'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n',
                    encoding='utf-8', newline='\n')


def prepare(output):
    manifest = read(PACKAGE/'manifest.json')
    dependencies = []
    for item in manifest['dependencies']:
        path = (ROOT/item['path']).resolve()
        if not path.is_relative_to(ROOT) or digest(path) != item['sha256']:
            raise ValueError(f'Missing or changed snapshot asset: {item["path"]}; run git lfs pull')
        dependencies.append(dict(path=str(path), sha256=item['sha256']))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    tree = ET.parse(PACKAGE/'preheld-dual.xml')
    for node in tree.iter():
        if 'file' in node.attrib:
            node.set('file', str((PACKAGE/node.get('file')).resolve()))
    model = output/'preheld-dual.xml'
    tree.write(model, encoding='utf-8', xml_declaration=True)
    shutil.copyfile(PACKAGE/'attitude_control.json', output/'attitude_control.json')
    shutil.copyfile(PACKAGE/'initial-scene.json', output/'initial-scene.json')
    dependencies.append(dict(path=str(model), sha256=digest(model)))
    catalog = read(PACKAGE/'ue-catalog.json')
    catalog['source_mjcf'] = str(model)
    catalog['assets'] = {str((ROOT/k).resolve()):v for k,v in catalog['assets'].items()}
    write(output/'catalog.json', catalog)
    receipt = dict(schema='saved-model-registration/1', full_task_accepted=False,
        manifest_sha256=digest(PACKAGE/'manifest.json'), model_sha256=digest(model),
        linux_runtime_verified=False, ue_asset_import_verified=False)
    write(output/'receipt.json', receipt)
    entries = []
    for task in read(PACKAGE/'tasks.json'):
        initial = deepcopy(task['initial_scene'])
        instance = dict(schema='space-arm-scene-instance/1', recipe=task['recipe'],
            model=str(model), dependencies=dependencies, seed=initial.get('seed',42),
            initial_scene=initial, model_generated=True, accepted=False,
            assembly_status='saved_local_candidate', operation_fixture=task['operation_fixture'])
        filename = task['scene_id'].replace('/', '--')+'.json'
        write(output/filename, instance)
        entry = dict(id=task['id'], scene_id=task['scene_id'], label=task['label'],
            kind='operation', base_template_id='sarm-dual-capture26',
            layout_revision='original-root-v2', full_task_accepted=False,
            description='保存的双臂已抓持初态；原系统相机；完整作业未通过验收。')
        for key, path in [('source_instance',output/filename), ('receipt',output/'receipt.json'),
                          ('catalog',output/'catalog.json')]:
            entry[key] = str(path)
            entry[key+'_sha256'] = digest(path)
        entries.append(entry)
    write(output/'imported_scene_library.json', dict(schema='space-arm-platform-library/1',
        layout_revision='original-root-v2', entries=entries))
    return dict(output=str(output), verified_dependencies=len(manifest['dependencies']),
                initializations=len(entries), installed=False, ue_assets_imported=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(json.dumps(prepare(parser.parse_args().output), ensure_ascii=False))
