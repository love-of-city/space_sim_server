"""Generate two independent, populated task boxes for one requested seed.

Uses the Linux branch's continuous XY/yaw sampler, never finite-pool selection.
Geometry rejection is deterministic; failed candidates and logs are retained.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend'), str(ROOT/'tools')]
from build_dual_taskbox_layout import compose
from register_dual_taskbox_pool import register
from simulation.scene_library.manifest import digest
from space_arm_platform.collection_processes import ManagerLease

SCHEMA = 'seeded-dual-taskbox/1'


def role_seed(seed, role, attempt=0):
    if type(seed) is not int or seed < 0 or role not in ('target', 'service'):
        raise ValueError('Require a nonnegative integer seed and a spacecraft role')
    return int.from_bytes(hashlib.sha256(f'{SCHEMA}:{seed}:{role}:{attempt}'.encode()).digest()[:8], 'big')


def prepare(config, seed, output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    state = dict(schema=SCHEMA, seed=seed, stages={}, rejected=[], completed=False,
                 full_task_accepted=False, dataset_capture_allowed=False)
    def save():
        temp = output/'pipeline.json.tmp'
        temp.write_text(json.dumps(state, indent=2)+'\n', encoding='utf-8')
        os.replace(temp, output/'pipeline.json')
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='4', PYTHONUTF8='1')
    env['PYTHONPATH'] = os.pathsep.join([str(ROOT), str(ROOT/'backend'),
        str(ROOT.parent/'space_sim_UE_Adapter/Adapters')])
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    def run(name, interpreter, script, *args):
        state['stages'][name] = 'running'
        save()
        with (output/(name+'.log')).open('w', encoding='utf-8') as log:
            result = subprocess.run([str(interpreter), str(ROOT/'tools'/script), *map(str, args)],
                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, creationflags=flags)
        state['stages'][name] = 'passed' if result.returncode == 0 else 'failed'
        save()
        return result.returncode == 0
    geo, native = config['geometry_python'], config['platform_python']
    lock = Path(config['adapter'])/'Unreal/BskUnrealRenderer/Saved/collection-layout-import.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    lease = ManagerLease(lock)
    try:
        for attempt in range(config.get('max_attempts', 12)):
            folder = output/f'attempt-{attempt:03d}'
            folder.mkdir()
            donors = []
            valid = True
            for role in ('target', 'service'):
                donor = folder/role
                model = donor/'layout.xml'
                stage = f'{attempt}-{role}'
                stages = [
                    ('build', 'build_independent_task_box_layout.py', ('--source', config['source'],
                        '--output', model, '--seed', role_seed(seed, role, attempt),
                        '--target-mounts', 'inset', '--pickup-mounts', 'inset', '--yaw-range-deg', 180,
                        '--clearance-mode', 'native-3d-candidate', '--inset-deck-clearance-m', .0009)),
                    ('native', 'check_independent_task_box_layout.py', ('--source', config['source'],
                        '--layout', model, '--report', donor/'native-geometry.json', '--preview-candidate')),
                    ('inset', 'check_inset_target_geometry.py', ('--layout', model,
                        '--report', donor/'inset-geometry.json', '--preview-candidate')),
                ]
                for suffix, script, args in stages:
                    if not run(stage+'-'+suffix, geo, script, *args):
                        # Only failed geometry checks are sampling rejections.
                        # Build/import/runtime errors stop rather than changing seed.
                        proof = donor/('native-geometry.json' if suffix == 'native' else 'inset-geometry.json')
                        if suffix == 'build' or not proof.exists():
                            raise RuntimeError('Layout preparation failed: '+stage+'-'+suffix)
                        valid = False
                        state['rejected'].append(dict(attempt=attempt, stage=stage+'-'+suffix))
                        break
                if not valid:
                    break
                donors.append(donor)
            if not valid:
                save()
                continue
            case = folder/'pair'
            compose(Path(config['base']), donors[0]/'layout.xml', donors[1]/'layout.xml', case)
            if not run(f'{attempt}-combined', geo, 'audit_dual_taskbox_layout.py', '--case', case):
                if not (case/'initial-geometry.json').exists():
                    raise RuntimeError('Combined model checker failed before producing evidence')
                state['rejected'].append(dict(attempt=attempt, stage='combined'))
                save()
                continue
            if not run(f'{attempt}-native-load', native, 'check_dual_taskbox_native_load.py', '--case', case):
                raise RuntimeError('Basilisk failed to load generated layout')
            for role, donor in zip(('target', 'service'), donors):
                namespace = 'Seed_'+hashlib.sha256(str(output).encode()).hexdigest()[:16]+'_'+role
                if not run(f'{attempt}-{role}-import', native, 'prepare_independent_layout_catalog.py',
                           '--layout', donor/'layout.xml', '--base-catalog', config['base_catalog'],
                           '--adapter', config['adapter'], '--unreal', config['unreal'],
                           '--output', donor/'ue-assets', '--preview-candidate', '--reuse-identical-assets',
                           '--destination', '/Game/BSK/Generated/'+namespace):
                    raise RuntimeError('UE asset import failed for '+role)
            result = register(Path(config['index']), [case], donors, output/'registration', publish=False)
            state.update(completed=True, generation_attempt=attempt, registered=result,
                         role_seeds={role: role_seed(seed, role, attempt) for role in ('target', 'service')},
                         result_sha256=digest(result['pool']))
            save()
            return state
        raise RuntimeError('No collision-free dual layout within deterministic sampling budget; see '+str(output))
    except Exception as error:
        state['error'] = str(error)
        save()
        raise
    finally:
        lease.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(json.loads(args.config.read_text(encoding='utf-8')), args.seed, args.output)))
