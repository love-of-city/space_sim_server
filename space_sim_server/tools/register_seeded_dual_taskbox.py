"""Install a content-pinned generator on existing family 2/4 scene entries."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]
from simulation.scene_library.manifest import model_dependencies, digest, write_json
from space_arm_platform.imported_scene_library import verified_source


def register(config, output):
    index_path = Path(config['index'])
    index = json.loads(index_path.read_text(encoding='utf-8'))
    entries = [e for e in index['entries'] if e['id'].startswith(('library/02_', 'library/04_'))]
    if not entries:
        raise ValueError('No installed family 2/4 scenes')
    base = Path(config['base'])
    base_hash = digest(base/'preheld-dual.xml')
    for entry in entries:
        if digest(verified_source(entry)['model']) != base_hash:
            raise ValueError('Generator base differs from the installed model')
    deps = model_dependencies(Path(config['source'])) + model_dependencies(base/'preheld-dual.xml')
    paths = [base/n for n in ('initial-scene.json', 'hull-closure.json')]
    paths += [Path(config['base_catalog'])]
    paths += [ROOT/p for p in (
        'simulation/independent_task_box_layout.py', 'simulation/scene_library/taskbox_mount.py',
        'simulation/scene_library/hull_closure.py', 'simulation/plug_retention.py',
        'tools/build_independent_task_box_layout.py', 'tools/build_dual_taskbox_layout.py',
        'tools/prepare_seeded_dual_taskbox.py', 'tools/audit_dual_taskbox_layout.py',
        'tools/check_independent_task_box_layout.py', 'tools/check_inset_target_geometry.py',
        'tools/check_dual_taskbox_native_load.py', 'tools/prepare_independent_layout_catalog.py',
        'tools/register_dual_taskbox_pool.py')]
    deps += [dict(path=str(p.resolve()), sha256=digest(p)) for p in paths]
    config = dict(config, base_model_sha256=base_hash,
                  dependencies=list({d['path']:d for d in deps}.values()))
    output.mkdir(parents=True, exist_ok=False)
    path = output/'generator.json'
    write_json(path, config)
    write_json(output/'previous-index.json', index)
    for entry in entries:
        entry.pop('task_box_layout_pool', None)
        entry.pop('task_box_layout_pool_sha256', None)
        entry.update(task_box_layout_generator=str(path.resolve()), task_box_layout_generator_sha256=digest(path))
    temporary = index_path.with_suffix('.tmp')
    write_json(temporary, index)
    os.replace(temporary, index_path)
    return dict(generator=str(path), tasks=len(entries), sha256=digest(path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(register(json.loads(args.config.read_text(encoding='utf-8')), args.output.resolve())))
