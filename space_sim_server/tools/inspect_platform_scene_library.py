"""Sequential real-platform imports and original-camera screenshot evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import requests


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--only', nargs='*')
    parser.add_argument('--reuse-current', action='store_true', help='Capture the matching live scene without restarting it')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    entries = json.loads((root/'run/imported_scene_library.json').read_text(encoding='utf-8-sig'))['entries']
    session = requests.Session()
    def api(route, body=None):
        response = session.request('GET' if body is None else 'POST', 'http://127.0.0.1:8000'+route, json=body, timeout=900)
        if not response.ok:
            raise RuntimeError(f'{route} HTTP {response.status_code}: {response.text[:1000]}')
        return response.json()
    api('/api/auth/login', {'username':os.environ.get('SPACE_SIM_ADMIN_USERNAME','admin'), 'password':os.environ.get('SPACE_SIM_ADMIN_PASSWORD','ChangeMe123!')})
    results = []
    for entry in entries:
        if args.only and entry['id'] not in args.only:
            continue
        dest = args.output/entry['id'].removeprefix('library/')
        dest.mkdir(parents=True, exist_ok=True)
        if (dest/'complete.json').exists():
            results.append(json.loads((dest/'complete.json').read_text(encoding='utf-8')))
            continue
        print('START', entry['id'], flush=True)
        record = {'id':entry['id'], 'started_at':time.time(), 'camera_transforms_changed':False}
        try:
            current = api('/api/state') if args.reuse_current else {}
            active = (current.get('scene_runtime', {}).get('instance') or {}).get('library_scene', {})
            if args.reuse_current and active.get('id') == entry['id'] and active.get('source_instance_sha256') == entry['source_instance_sha256']:
                instance = current
            else:
                api('/api/scenes/stop', {})
                time.sleep(3)
                instance = api('/api/scenes/start', {'template_id':entry['id'], 'dataset_capture':False, 'capture_rate_hz':2.0, 'ik_rate_hz':100.0})
            write(dest/'launch.json', instance)
            for tick in range(480):
                state = api('/api/state')
                runtime = state.get('scene_runtime', {})
                observation = state.get('simulation', {}).get('latest_observation') or {}
                ready = (runtime.get('phase')=='running' and state['simulation']['connected'] and observation.get('scene_instance_id')==runtime.get('instance',{}).get('instance_id') and int(observation.get('sim_time_ns',0))>1000000000)
                ready = ready or (entry['kind']=='rendezvous' and state.get('mission',{}).get('stage')=='flight' and state['mission'].get('telemetry',{}).get('connected'))
                if ready:
                    break
                if runtime.get('phase')=='failed':
                    raise RuntimeError(str(runtime.get('error')))
                if tick%15==0:
                    print('WAIT', entry['id'], runtime.get('phase'), tick*2, flush=True)
                time.sleep(2)
            else:
                raise RuntimeError('Native startup timeout')
            write(dest/'native-ready.json', state)
            env = dict(os.environ, SPACE_SIM_INSPECTION_OUTPUT=str(dest))
            with (dest/'browser.log').open('w',encoding='utf-8') as log:
                subprocess.run(['node',str(root/'scripts/inspect_platform_cameras.mjs')],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900,check=True)
            inspection = json.loads((dest/'inspection.json').read_text(encoding='utf-8'))
            expected = 3 if entry['base_template_id']!='sarm-dual-capture26' else 4
            available = [s for s in inspection['streams'] if s.get('available')]
            if len(available)<expected:
                raise RuntimeError(f'Missing original streams: {len(available)}/{expected}')
            runtime = api('/api/state').get('scene_runtime',{})
            instance_id = (runtime.get('instance') or {}).get('instance_id')
            if entry['kind']!='rendezvous' and instance_id:
                for suffix in ('simulation.out.log','simulation.err.log','launcher.out.log','launcher.err.log'):
                    source = root/'logs'/f'{instance_id}.{suffix}'
                    if source.exists():
                        shutil.copy2(source,dest/suffix)
                lines = (dest/'simulation.out.log').read_text(encoding='utf-8',errors='replace').splitlines()
                cameras = [json.loads(line) for line in lines if line.startswith('{') and '"platform_camera_configuration"' in line]
                write(dest/'camera-configuration.json',cameras)
            record.update(success=True, streams=[s['id'] for s in available], instance_id=instance_id, images={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.glob('*.png')})
            write(dest/'complete.json', record)
            print('DONE',entry['id'],len(available),flush=True)
        except Exception as error:
            record.update(success=False,error=str(error))
            write(dest/'failure.json',record)
            print('FAILED',entry['id'],str(error),flush=True)
        results.append(record)
        write(args.output/'batch.json',results)
    print('FINISHED',sum(r.get('success',False) for r in results),'/',len(results),flush=True)


if __name__=='__main__':
    main()
