"""Relocate the solar-wing baseline and prepare its seeded layout registration.

Does not import UE assets or replace the active platform index. Machine-specific
paths are written only to the requested runtime output, never to the snapshot.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from prepare_saved_model import prepare, write
from register_seeded_dual_taskbox import register
from import_platform_scene_library import cameras


def prepare_seeded(output, adapter, unreal, geometry_python, platform_python):
    output = output.resolve()
    result = prepare(output, ROOT/'model/scene_library/solar-closed-hull-v3')
    index_path = output/'imported_scene_library.json'
    index = json.loads(index_path.read_text(encoding='utf-8'))
    contract = cameras(output/'preheld-dual.xml')
    for entry in index['entries']:
        entry['camera_contract'] = contract
    write(index_path, index)
    config = dict(schema='seeded-dual-taskbox-config/1',
                  source=str(ROOT/'model/SARM/platform/sarm_20261012.xml'),
                  base=str(output), base_catalog=str(output/'catalog.json'),
                  index=str(output/'imported_scene_library.json'),
                  adapter=str(adapter.resolve()), unreal=str(unreal.resolve()),
                  geometry_python=str(geometry_python.resolve()),
                  platform_python=str(platform_python.resolve()),
                  cache_root=str(output/'generated'), max_attempts=12,
                  upstream_revision='a48919802cc2d5cb326015a45149d46630832a1c')
    write(output/'generator-input.json', config)
    result['generator'] = register(config, output/'generator')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'adapter', 'unreal', 'geometry-python', 'platform-python'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare_seeded(args.output, args.adapter, args.unreal,
                                  args.geometry_python, args.platform_python)))
