"""Export matching UE RGB frames and provenance from a dual-service scene."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT.parent / 'space_sim_UE_Adapter/Adapters'))
from bsk_render_adapter.recording import BskRecordingReader


def export(instance_id, output):
    source = ROOT / 'run/dual_service' / instance_id
    runtime = json.loads((source / 'runtime.json').read_text())
    version = runtime['version']
    output.mkdir(parents=True, exist_ok=True)
    camera_names = [('spacecraft_overview', 'global', '全局相机'),
                    ('right_sarm_wrist_cam', 'left_wrist', '左臂手眼相机'),
                    ('left_sarm_wrist_cam', 'right_wrist', '右臂手眼相机')]
    frames = {}
    for camera, _, _ in camera_names:
        records = {}
        for path in (source / 'ue_preview').glob(f'*/teleop_camera_{camera}/*.json'):
            record = json.loads(path.read_text())
            records[int(record['sim_time_ns'])] = (path, record)
        frames[camera] = records
    shared = set.intersection(*(set(records) for records in frames.values()))
    if not shared:
        raise ValueError('No common authoritative UE capture timestamp')
    timestamp = max(shared)
    paths = []
    for camera, stem, _ in camera_names:
        metadata_path, metadata = frames[camera][timestamp]
        rgb = next(item for item in metadata['products'] if item['name'] == 'rgb')
        image_path = metadata_path.parent / rgb['file_name']
        destination = output / (stem + image_path.suffix)
        shutil.copy2(image_path, destination)
        shutil.copy2(metadata_path, output / f'{stem}.json')
        paths.append(destination)
    overview=max((source/'ue_preview').glob(f'full_system_v{version}*.png'), key=lambda path:path.stat().st_mtime_ns)
    shutil.copy2(overview, output / 'full_system.png')
    shutil.copy2(source / 'runtime.json', output / 'runtime.json')
    messages = [json.loads((source/'live_manifest.json').read_text())] if (source/'live_manifest.json').exists() else BskRecordingReader(source / 'scene.bskrec').messages()
    for message in messages:
        if message['type'] == 'scene_manifest':
            (output / 'manifest.json').write_text(json.dumps(message, indent=2), encoding='utf-8')
            assert len(message['cameras']) == 3 and len(message['celestial_bodies']) == 3
            bus=next(item for item in message['objects'] if item['object_id']=='teleop/cubesat_bus')
            mli=next(item for item in bus['geometries'] if item['geometry_id']=='teleop/cubesat_bus/visual/mli')
            assert mli['scale']==[2.,2.,2.] and mli['position_body_m']==[-.4,-.53,0.]
            assert mli['use_asset_materials'] is True
            break
    panel = Image.new('RGB', (3072, 1120), '#101927')
    draw = ImageDraw.Draw(panel)
    font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 32)
    for index, ((_, _, label), path) in enumerate(zip(camera_names, paths)):
        draw.text((index * 1024 + 24, 28), f'版本 {version} · {label} · UE 实渲', font=font, fill='white')
        with Image.open(path) as image:
            assert image.size == (1024, 1024)
            panel.paste(image, (index * 1024, 96))
    panel.save(output / 'three_views.png')
    panel.resize((1536, 560), Image.Resampling.LANCZOS).save(output / 'three_views_preview.jpg', quality=94)
    receipt = {'instance_id': instance_id, 'version': version,
               'source_directory': str(source), 'common_sim_time_ns': str(timestamp),
               'model_sha256': runtime['model_sha256'],
               'mli_visual':mli,
               'scope': 'UE visual preview with live BSK/MuJoCo orbit and saved pose hold; not task or dataset acceptance',
               'sha256': {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}}
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--instance', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export(args.instance, args.output)
