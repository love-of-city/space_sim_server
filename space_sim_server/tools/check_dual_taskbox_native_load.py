"""Load the assembled model through the actual Basilisk VFS, without stepping."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.mujoco_plugins import ensure_model_plugins
from simulation.scene_library.manifest import digest, write_json


def check(case):
    from Basilisk.simulation import mujoco

    path = case/'preheld-dual.xml'
    root = ET.parse(path).getroot()
    compiler = root.find('compiler')
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    files = sorted({str((path.parent/meshdir/node.get('file')).resolve())
        for selector in ('./asset/mesh[@file]', './/flexcomp[@file]') for node in root.findall(selector)})
    if len({Path(p).name.casefold() for p in files}) != len(files):
        raise ValueError('Basilisk VFS requires unique mesh basenames')
    ensure_model_plugins(path)
    print('Loading actual Basilisk MJScene', flush=True)
    scene = mujoco.MJScene.fromFile(str(path), files=files)
    report = dict(passed=True, model_sha256=digest(path), mesh_file_count=len(files),
                  runtime='Basilisk.MJScene.fromFile', scope='model_load_only', full_task_accepted=False)
    write_json(case/'native-load.json', report)
    del scene
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    print(check(parser.parse_args().case.resolve()), flush=True)
