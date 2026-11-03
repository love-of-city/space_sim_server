"""Publish checked initial-layout candidates for local family 2/4 teleoperation."""
from copy import deepcopy
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend'), str(ROOT/'tools')]
from simulation.scene_library.manifest import digest, model_dependencies, write_json
from space_arm_platform.imported_scene_library import verified_source
from import_platform_scene_library import cameras


def register(index_path, cases, donors, output, *, publish=True):
    index = json.loads(index_path.read_text(encoding='utf-8'))
    entries = [e for e in index['entries'] if e['id'].startswith(('library/02_', 'library/04_'))]
    if not entries or len(cases) < (2 if publish else 1):
        raise ValueError('Require existing family 2/4 entries and at least two variants')
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'previous-index.json', index)
    assets, donor_hashes = {}, set()
    for donor in donors:
        donor_hashes.add(digest(donor/'layout.xml'))
        for filename in ('native-geometry.json', 'inset-geometry.json'):
            proof = json.loads((donor/filename).read_text(encoding='utf-8'))
            if not proof['passed'] or proof['model_sha256'] != digest(donor/'layout.xml'):
                raise ValueError('Donor geometry failed or changed')
        catalog = json.loads((donor/'ue-assets/layout.catalog.json').read_text(encoding='utf-8'))
        for path, asset in catalog['assets'].items():
            if digest(path) != asset['source_sha256']:
                raise ValueError('Catalog source changed '+path)
            assets[path] = asset
    variants = []
    model_hashes = set()
    for case in cases:
        report = json.loads((case/'initial-geometry.json').read_text(encoding='utf-8'))
        layout = json.loads((case/'dual-layout.json').read_text(encoding='utf-8'))
        model = case/'preheld-dual.xml'
        model_hash = digest(model)
        if not report['passed'] or report['model_sha256'] != model_hash or layout['model_sha256'] != model_hash:
            raise ValueError('Combined geometry failed or changed')
        if report['scene_sha256'] != digest(case/'initial-scene.json'):
            raise ValueError('Initial scene changed')
        native = json.loads((case/'native-load.json').read_text(encoding='utf-8'))
        if not native['passed'] or native['model_sha256'] != model_hash:
            raise ValueError('Basilisk model load failed or changed')
        if any(layout[role]['model_sha256'] not in donor_hashes for role in ('target', 'service')):
            raise ValueError('Combined layout uses unchecked donor')
        model_hashes.add(model_hash)
        case_assets = deepcopy(assets)
        for alias in layout.get('vfs_asset_aliases', []):
            if digest(alias['path']) != alias['sha256'] or digest(alias['source']) != alias['sha256']:
                raise ValueError('VFS asset alias changed')
            if alias['source'] in assets:
                case_assets[alias['path']] = deepcopy(assets[alias['source']])
        catalog_path = case/'platform.catalog.json'
        catalog = dict(schema='bsk-render-asset-catalog/1', source_mjcf=str(model), assets=case_assets)
        if catalog_path.exists():
            if json.loads(catalog_path.read_text(encoding='utf-8')) != catalog:
                raise ValueError('Existing case catalog differs')
        else:
            write_json(catalog_path, catalog)
        dependencies = model_dependencies(model) + [dict(path=str(case/name), sha256=digest(case/name))
            for name in ('initial-geometry.json', 'native-load.json', 'dual-layout.json', 'initial-scene.json')]
        installed = {}
        for entry in entries:
            source = deepcopy(verified_source(entry))
            if digest(source['model']) != layout['base_model_sha256']:
                raise ValueError('Layout was built from another installed base model')
            if cameras(model) != entry['camera_contract']:
                raise ValueError('Original installed cameras changed')
            source.update(model=str(model), dependencies=dependencies, accepted=False,
                          initial_scene=json.loads((case/'initial-scene.json').read_text(encoding='utf-8')),
                          task_box_layout=layout, assembly_status='random_layout_initial_geometry_candidate')
            for key, name in (('assembly', 'dual-layout.json'), ('mount_inspection', 'initial-geometry.json'),
                              ('initial_scene_source', 'initial-scene.json')):
                source['operation_fixture'][key] = str(case/name)
                source['operation_fixture'][key+'_sha256'] = digest(case/name)
            directory = output/case.name/entry['scene_id'].replace('/', '--')
            directory.mkdir(parents=True)
            write_json(directory/'instance.json', source)
            write_json(directory/'receipt.json', dict(model_sha256=model_hash, full_task_accepted=False,
                dataset_capture_allowed=False, initial_geometry=report, base_source_sha256=entry['source_instance_sha256']))
            selected = deepcopy(entry)
            for key in ('task_box_layout_pool', 'task_box_layout_pool_sha256'):
                selected.pop(key, None)
            for key, path in (('source_instance', directory/'instance.json'), ('receipt', directory/'receipt.json'), ('catalog', catalog_path)):
                selected[key], selected[key+'_sha256'] = str(path), digest(path)
            selected.update(full_task_accepted=False, layout_revision='dual-random-'+model_hash[:12])
            verified_source(selected)
            installed[entry['id']] = selected
        variants.append(dict(id=case.name, model_sha256=model_hash, entries=installed))
    if len(model_hashes) != len(cases):
        raise ValueError('Variants must contain distinct models')
    pool = output/'pool.json'
    write_json(pool, dict(schema='dual-taskbox-layout-pool/1',
        base_sources={e['id']:e['source_instance_sha256'] for e in entries}, variants=variants,
        acceptance_scope='initial_geometry', full_task_accepted=False, dataset_capture_allowed=False))
    if publish:
        for entry in entries:
            entry.update(task_box_layout_pool=str(pool), task_box_layout_pool_sha256=digest(pool))
        temporary = index_path.with_suffix('.json.tmp')
        write_json(temporary, index)
        os.replace(temporary, index_path)
    write_json(output/'installed-index.json', index)
    return dict(pool=str(pool), variants=len(variants), tasks=len(entries))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path, required=True)
    parser.add_argument('--case', type=Path, action='append', required=True)
    parser.add_argument('--donor', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(register(args.index.resolve(), [p.resolve() for p in args.case],
                              [p.resolve() for p in args.donor], args.output.resolve())))
