"""Prepare a small immutable dual-task-box preview pool, then register it.

Run with the geometry Python. Full-task/collection acceptance stays false.
Failed generation, checks or UE import stop publication and retain evidence.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend'), str(ROOT/'tools')]
from build_dual_taskbox_layout import compose
from register_dual_taskbox_pool import register
from space_arm_platform.collection_processes import ManagerLease


def prepare(args):
    if args.count < 2 or args.seed < 0:
        raise ValueError('Require at least two layouts and a nonnegative seed')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    state = dict(stages={}, full_task_accepted=False, dataset_capture_allowed=False)
    def save_state():
        # Unlike immutable evidence files, this is the live stage journal.
        temporary = output/'pipeline.json.tmp'
        temporary.write_text(json.dumps(state, indent=2)+'\n', encoding='utf-8')
        os.replace(temporary, output/'pipeline.json')
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='4', PYTHONUTF8='1',
               PYTHONPATH=str(ROOT.parent/'space_sim_UE_Adapter/Adapters'))
    def run(name, interpreter, script, *arguments):
        state['stages'][name] = 'running'
        save_state()
        try:
            with (output/(name+'.log')).open('w', encoding='utf-8') as log:
                subprocess.run([str(interpreter), str(ROOT/'tools'/script), *map(str, arguments)],
                               cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        except Exception:
            state['stages'][name] = 'failed'
            save_state()
            raise
        state['stages'][name] = 'passed'
        save_state()
    donors = [p.resolve() for p in args.existing_donor]
    if donors and len(donors) < 2:
        raise ValueError('Require two existing donor layouts')
    with_lease = ManagerLease(args.adapter/'Unreal/BskUnrealRenderer/Saved/collection-layout-import.lock')
    try:
        if not donors:
            for i in range(args.count):
                donor = output/f'donor-{i:03d}'
                model = donor/'layout.xml'
                run(f'build-{i}', sys.executable, 'build_independent_task_box_layout.py',
                    '--source', args.source.resolve(), '--output', model, '--seed', args.seed+i,
                    '--target-mounts', 'inset', '--pickup-mounts', 'inset', '--yaw-range-deg', 180,
                    '--clearance-mode', 'native-3d-candidate', '--inset-deck-clearance-m', .0009)
                run(f'native-{i}', sys.executable, 'check_independent_task_box_layout.py',
                    '--source', args.source.resolve(), '--layout', model, '--report', donor/'native-geometry.json', '--preview-candidate')
                run(f'inset-{i}', sys.executable, 'check_inset_target_geometry.py',
                    '--layout', model, '--report', donor/'inset-geometry.json', '--preview-candidate')
                run(f'import-{i}', args.platform_python, 'prepare_independent_layout_catalog.py',
                    '--layout', model, '--base-catalog', args.base_catalog.resolve(), '--adapter', args.adapter.resolve(),
                    '--unreal', args.unreal.resolve(), '--output', donor/'ue-assets', '--preview-candidate',
                    '--destination', '/Game/BSK/Generated/Dual_'+output.name.replace('-', '_')+'_'+str(i))
                donors.append(donor)
        cases = []
        for i, donor in enumerate(donors):
            case = output/f'pair-{i:03d}'
            compose(args.base.resolve(), donor/'layout.xml', donors[(i+1) % len(donors)]/'layout.xml', case)
            run(f'combined-{i}', sys.executable, 'audit_dual_taskbox_layout.py', '--case', case)
            run(f'native-load-{i}', args.platform_python, 'check_dual_taskbox_native_load.py', '--case', case)
            cases.append(case)
        result = register(args.index.resolve(), cases, donors, output/'registration')
        state.update(registered=result, completed=True)
        save_state()
        return result
    finally:
        with_lease.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('base', 'base-catalog', 'adapter', 'unreal', 'platform-python', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--index', type=Path, default=ROOT/'run/imported_scene_library.json')
    parser.add_argument('--source', type=Path, default=ROOT/'model/SARM/platform/sarm_20261012.xml')
    parser.add_argument('--count', type=int, default=2)
    parser.add_argument('--seed', type=int, default=2026110300)
    parser.add_argument('--existing-donor', type=Path, action='append', default=[])
    print(json.dumps(prepare(parser.parse_args())))
