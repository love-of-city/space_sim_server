"""Open a verified manipulation or flight run in UE (recorded replay only)."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from export_scene_ue_replay import digest, export


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--scene-id')
    parser.add_argument('--rate', type=float, help='Replay rate; defaults to 20 for rendezvous and 1 for manipulation')
    args = parser.parse_args()
    if args.rate is not None and not 0.1 <= args.rate <= 20:
        parser.error('rate must be in [0.1, 20]')
    config = read(args.config)
    library = read(config['library'])
    entries = [e for e in library['entries']
               if (Path(e['native_report']).parent / 'states-200hz.jsonl.gz').is_file() or e.get('ue_recording')]
    if not entries:
        raise ValueError('No saved UE-compatible trajectories in this library')
    if args.scene_id:
        selected = next(e for e in entries if e['scene_id'] == args.scene_id)
    else:
        print('UE 场景库：已验收轨迹回放\n')
        for index, entry in enumerate(entries, 1):
            print(f'{index:2d}. {entry.get("display_name", entry["scene_id"])} [{entry["scene_id"]}]')
        print('\n打开下一个场景前，请先关闭上一个 UE 窗口。')
        number = int(input('选择场景编号：'))
        if not 1 <= number <= len(entries):
            raise ValueError('Scene number is out of range')
        selected = entries[number - 1]
    rate = args.rate if args.rate is not None else (20. if selected['scene_id'].startswith('03_') else 1.)
    for key in ('receipt', 'native_report', 'launch'):
        if digest(selected[key]) != selected[key + '_sha256']:
            raise ValueError(f'Accepted artifact changed: {key}')
    if selected['scene_id'].startswith(('02_', '04_')) and read(selected['receipt']).get('evidence_scope') != 'dual_arm_preheld_operation':
        raise ValueError('This legacy deck-mounted run is superseded: families 2/4 require a target held by the capture arm.')
    native = Path(selected['native_report']).parent
    if selected.get('ue_recording'):
        output = Path(selected['ue_recording'])
        for key in ('ue_recording', 'ue_recording_validation', 'ue_telemetry'):
            if digest(selected[key]) != selected[key + '_sha256']:
                raise ValueError(f'Flight recording provenance changed: {key}')
        if read(selected['ue_recording_validation']).get('passed') is not True:
            raise ValueError('Flight recording has not passed its independent pose audit')
        launch_ue(config, selected, output, rate)
        return
    trace = native / 'states-200hz.jsonl.gz'
    model = Path(read(native / 'run/summary.json')['model'])
    output = Path(config['output']) / (selected['scene_id'].replace('/', '--') + '.bskrec')
    if output.exists():
        report = read(output.with_suffix('.json'))
        for path, expected in ((output, report['recording_sha256']),
                               (trace, report['trace_sha256']), (model, report['model_sha256'])):
            if digest(path) != expected:
                raise ValueError(f'Cached replay provenance changed: {path}')
    else:
        print('Preparing UE recording from the saved trajectory...', flush=True)
        export(model, trace, [Path(p) for p in config['catalogs']], Path(config['project']),
               output, selected['scene_id'])
    launch_ue(config, selected, output, rate)


def launch_ue(config, selected, output, rate):
    scene_config = config.get('scene_configs', {}).get(selected['scene_id'], config['scene_config'])
    command = [config['editor'], str(Path(config['project']) / 'BskUnrealRenderer.uproject'),
        '-game', '-windowed', '-ResX=1440', '-ResY=900', '-nosplash',
        '-DDC=InstalledDerivedDataBackendGraph', f'-BskReplay={output}', f'-BskReplayRate={rate}',
        f'-BskConfig={scene_config}', f'-BskScreenshot={output.with_suffix(".png")}']
    environment = dict(os.environ)
    environment['UE-LocalDataCachePath'] = config['derived_data_cache']
    process = subprocess.Popen(command, env=environment)
    launch = dict(mode='UE_RECORDED_REPLAY', scene_id=selected['scene_id'], pid=process.pid,
                  command=command, recording_sha256=digest(output))
    output.with_suffix('.launch.json').write_text(json.dumps(launch, indent=2), encoding='utf-8')
    print(f'已打开 UE：{selected.get("display_name", selected["scene_id"])}（{rate:g} 倍回放）\nC：自由相机；W/S、A/D、Q/E：移动；鼠标：转向；Esc/Home：总览。')


if __name__ == '__main__':
    main()
