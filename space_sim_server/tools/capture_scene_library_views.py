"""Capture actual UE overview and all model cameras from accepted replay segments.

Use the separate MuJoCo geometry Python. Original recordings/receipts are immutable.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import time

from export_scene_ue_replay import compiled_cameras, digest, load_geometry_model
from bsk_render_adapter.recording import BskRecordingReader, BskRecordingWriter


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


def prepare(config, entry, root):
    sid = entry['scene_id']
    folder = root / sid.replace('/', '--')
    folder.mkdir(parents=True, exist_ok=True)
    if (folder/'prepared.json').exists():
        saved = read(folder/'prepared.json')
        for key in ('recording', 'inspection_recording', 'config', 'original_recording'):
            if digest(saved[key]) != saved[key+'_sha256']:
                raise ValueError(f'Prepared capture artifact changed: {key}')
        return folder, saved
    for key in ('receipt', 'native_report', 'launch'):
        if digest(entry[key]) != entry[key+'_sha256']:
            raise ValueError(f'Accepted evidence changed: {sid}/{key}')
    flight = bool(entry.get('ue_recording'))
    original = Path(entry['ue_recording']) if flight else Path(config['output'])/(sid.replace('/', '--')+'.bskrec')
    model_path = None
    if flight:
        if digest(original) != entry['ue_recording_sha256']:
            raise ValueError('Flight recording changed')
    else:
        meta = read(original.with_suffix('.json'))
        for key in ('recording', 'trace', 'model'):
            if digest(meta[key]) != meta[key+'_sha256']:
                raise ValueError(f'Replay provenance changed: {key}')
        model_path = Path(meta['model'])
    packets = list(BskRecordingReader(original).messages())
    manifest = next(m for m in packets if m['type']=='scene_manifest')
    cameras = copy.deepcopy(manifest['cameras']) if flight else compiled_cameras(load_geometry_model(model_path))
    parents = {o['object_id'] for o in manifest['objects']}
    for i, camera in enumerate(cameras):
        assert camera['parent_id'] in parents
        camera.update(capture_products=['rgb'], capture_rate_hz=2,
                      picture_in_picture=False, picture_in_picture_slot=i+1,
                      capture_consumer='dataset')
    frames = [m for m in packets if m['type']=='frame']
    duration = int(frames[-1]['sim_time_ns'])/1e9
    # A recorded working pose, not a staged reconstruction. Flight uses near hold.
    start = max(0, duration-18) if flight else min(70, duration*.35)
    grid = [int(m['sim_time_ns'])/1e9 for m in frames
            if int(m['sim_time_ns']) % 500000000 == 0]
    if sum(start <= t <= start+10 for t in grid) < 3:
        candidates = [(max(0,t-2), sum(max(0,t-2) <= x <= max(0,t-2)+10 for x in grid)) for t in grid]
        if not candidates:
            raise ValueError('Recording contains no exact 2 Hz camera sample timestamps')
        dense = [t for t,n in candidates if n >= 3]
        start = min(dense, key=lambda t: abs(t-start)) if dense else max(candidates, key=lambda pair: pair[1])[0]
    segment = [m for m in frames if start <= int(m['sim_time_ns'])/1e9 <= start+10]
    if not segment:
        raise ValueError('No recorded frames in inspection interval')
    manifest['cameras'] = cameras
    manifest['settings']['ui'] = {}
    full = folder/'with-cameras.bskrec'
    if not full.exists():
        with BskRecordingWriter(full) as writer:
            for packet in packets:
                writer.write(packet)
    clip = folder/'inspection.bskrec'
    if not clip.exists():
        with BskRecordingWriter(clip) as writer:
            writer.write(next(m for m in packets if m['type']=='hello'))
            writer.write(manifest)
            for frame in segment:
                writer.write(frame)
    scene_config = read(config.get('scene_configs', {}).get(sid, config['scene_config']))
    scene_config['capture'].update(output_directory=str(folder/'raw-cameras'),
                                   rate_override_hz=2, products_override=['rgb'])
    cp = folder/'config.json'
    write(cp, scene_config)
    report = dict(scene_id=sid, display_name=entry['display_name'], original_recording=str(original),
        original_recording_sha256=digest(original), recording=str(full), recording_sha256=digest(full),
        inspection_recording=str(clip), inspection_recording_sha256=digest(clip),
        model=str(model_path) if model_path else None, cameras=cameras,
        frame_ids=[m['frame_id'] for m in segment],
        start_time_s=int(segment[0]['sim_time_ns'])/1e9,
        end_time_s=int(segment[-1]['sim_time_ns'])/1e9,
        objects=len(manifest['objects']), config=str(cp), config_sha256=digest(cp),
        limitation='Actual UE images of unchanged accepted poses; selected 10-second recording segment, no new dynamics. Camera poses/FOV preserved; manipulation uses existing readability lighting.')
    write(folder/'prepared.json', report)
    return folder, report


def capture(config, folder, report):
    result = folder/'capture.json'
    if result.exists() and read(result).get('captured'):
        return
    png, log = folder/'overview.png', folder/'ue.log'
    command = [config['editor'], str(Path(config['project'])/'BskUnrealRenderer.uproject'),
        '-game', '-windowed', '-ResX=1440', '-ResY=900', '-nosplash',
        '-DDC=InstalledDerivedDataBackendGraph', f'-BskReplay={report["inspection_recording"]}',
        '-BskReplayRate=1', f'-BskConfig={report["config"]}',
        f'-BskScreenshot={png}', f'-abslog={log}']
    env = dict(os.environ)
    env['UE-LocalDataCachePath'] = config['derived_data_cache']
    process = subprocess.Popen(command, env=env)
    started = time.monotonic()
    try:
        while time.monotonic()-started < 300 and process.poll() is None:
            if png.exists():
                time.sleep(12)
                break
            time.sleep(.5)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
    metadata = []
    for path in (folder/'raw-cameras').rglob('*.json'):
        item = read(path)
        if item.get('camera_id'):
            metadata.append((path, item))
    ids = {c['camera_id'] for c in report['cameras']}
    per_camera = {cid: [(p,m) for p,m in metadata if m['camera_id']==cid] for cid in ids}
    # Select one common actual source frame across every camera, after warmup.
    common = set.intersection(*[{str(m['source_frame_id']) for _,m in rows} for rows in per_camera.values()]) if ids else set()
    if not common:
        write(result, dict(captured=False, command=command, cameras_found={k:len(v) for k,v in per_camera.items()}))
        raise RuntimeError(f'No synchronized camera products: {folder}')
    chosen = max(common, key=lambda value: int(value))
    selected = []
    for cid in sorted(ids):
        p, m = next((p,m) for p,m in per_camera[cid] if str(m['source_frame_id'])==chosen)
        selected.append(dict(camera_id=cid, metadata=str(p), metadata_sha256=digest(p)))
    text = log.read_text(encoding='utf-8', errors='replace')
    assert png.is_file() and f'with {report["objects"]} objects' in text
    write(result, dict(captured=True, command=command, overview=str(png), overview_sha256=digest(png),
        selected_frame_id=chosen, cameras=selected, visual_review='pending', log=str(log)))
    print(json.dumps(dict(scene=report['scene_id'], cameras=len(ids), captured=True)), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scene-id')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    config = read(args.config)
    library = read(config['library'])
    for entry in library['entries']:
        if args.scene_id and args.scene_id != entry['scene_id']:
            continue
        folder, report = prepare(config, entry, args.output)
        if not args.prepare_only:
            capture(config, folder, report)


if __name__ == '__main__':
    main()
