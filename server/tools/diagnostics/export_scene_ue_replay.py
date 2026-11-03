"""Convert saved native poses to UE packets, without executing dynamics.

Run in the separate geometry Python, never in a Basilisk process. Reuse an
installed UE mesh only when its catalog source hash matches the actual source.
"""
import argparse
from collections import defaultdict, deque
from dataclasses import replace
import math
import gzip
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT.parent / 'ue-adapter/Adapters'))
from bsk_render_adapter.bridge import _mj_geometry, _mj_mesh_geometry
from bsk_render_adapter.descriptors import CameraVisual, SceneSettings
from bsk_render_adapter.mjcf_assets import (
    _quat_multiply, _rotate, load_asset_catalog, parse_mjcf_geometry_metadata,
)
from bsk_render_adapter.recording import BskRecordingWriter
from preview_scene_model import load_geometry_model


def compiled_cameras(model):
    """Export actual compiled MJCF cameras, including xyaxes/frame rotations."""
    cameras = []
    for cid in range(model.ncam):
        if int(model.cam_mode[cid]) != int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED):
            raise ValueError('Replay camera export currently requires fixed body cameras')
        bid = int(model.cam_bodyid[cid])
        if bid == 0:
            raise ValueError('World cameras need an explicit replay-origin transform')
        width, height = (int(x) for x in model.cam_resolution[cid])
        if width <= 1 or height <= 1:
            width, height = 1920, 1080
        fovy = math.radians(float(model.cam_fovy[cid]))
        name = model.camera(cid).name or f'camera_{cid}'
        cameras.append(CameraVisual(camera_id=f'camera/{name}', display_name=name,
            parent_id=model.body(bid).name, position_body_m=model.cam_pos[cid].tolist(),
            orientation_body_from_camera_wxyz=_quat_multiply(
                model.cam_quat[cid], (.5, -.5, .5, .5)),
            field_of_view_rad=2*math.atan(math.tan(fovy/2)*width/height),
            resolution=(width, height), semantic_label='compiled_mjcf_camera',
            picture_in_picture=True, picture_in_picture_slot=cid+1,
            capture_rate_hz=2, capture_products=('rgb',)).to_payload())
    return cameras


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def export(model_path, trace, catalogs, project, output, label, fps=25):
    summary = json.loads((trace.parent / 'run/summary.json').read_text(encoding='utf-8-sig'))
    model_hash = digest(model_path)
    if summary['model_sha256'] != model_hash:
        raise ValueError('Native trace/model hash mismatch')
    if output.exists():
        raise FileExistsError(output)
    model = load_geometry_model(model_path)
    data = mujoco.MjData(model)
    raw_metadata = parse_mjcf_geometry_metadata(model_path, compile_mesh_poses=False)
    gids = [i for i in range(model.ngeom) if model.geom_bodyid[i] != 0]
    if len(raw_metadata) != len(gids):
        raise ValueError('XML/compiled geom ordering mismatch')
    # MuJoCo groups each body's own geoms before its descendants, including
    # mount geoms appended after a child body in the generated XML.
    by_body = defaultdict(deque)
    for meta in raw_metadata:
        by_body[meta.body_name].append(meta)
    metadata = []
    for gid in gids:
        meta = by_body[model.body(int(model.geom_bodyid[gid])).name].popleft()
        if meta.mesh_name:
            mid = int(model.geom_dataid[gid])
            if model.mesh(mid).name != meta.mesh_name:
                raise ValueError(f'Compiled mesh identity mismatch: {gid}')
            mesh_q = model.mesh_quat[mid]
            q = _quat_multiply(model.geom_quat[gid], (mesh_q[0], *(-mesh_q[1:])))
            p = model.geom_pos[gid] - np.asarray(_rotate(q, model.mesh_pos[mid]))
            meta = replace(meta, position_body_m=tuple(p), orientation_body_from_geometry_wxyz=q)
        metadata.append(meta)
    by_hash = {}
    for path in catalogs:
        catalog = json.loads(path.read_text(encoding='utf-8-sig'))
        for item in catalog['assets'].values():
            asset = item['asset_path'].split('.')[0]
            if asset.startswith('/Game/') and (project / 'Content' / (asset[6:] + '.uasset')).is_file():
                by_hash.setdefault(item.get('source_sha256'), item)
    rebound = {}
    for meta in metadata:
        if not meta.mesh_name or meta.role == 'collision' or not meta.visible:
            continue
        source_hash = digest(meta.source_path)
        if source_hash not in by_hash:
            raise ValueError(f'No installed, hash-matched UE mesh: {meta.source_path} ({source_hash})')
        rebound[meta.source_path] = by_hash[source_hash]
    output.parent.mkdir(parents=True, exist_ok=True)
    catalog_path = output.with_suffix('.catalog.json')
    catalog_path.write_text(json.dumps(dict(schema='bsk-render-asset-catalog/1',
        source_mjcf=str(model_path), assets=rebound), indent=2), encoding='utf-8')
    resolved = load_asset_catalog(catalog_path)
    objects = {bid: dict(object_id=model.body(bid).name, display_name=model.body(bid).name,
        parent_id='', transform_space='world', semantic_label='spacecraft', asset_path='', geometries=[])
        for bid in range(1, model.nbody)}
    for gid, meta in zip(gids, metadata):
        bid = int(model.geom_bodyid[gid])
        if meta.body_name != model.body(bid).name:
            raise ValueError(f'Geometry owner mismatch: {gid}')
        if meta.role == 'collision' or not meta.visible or model.geom_rgba[gid, 3] <= 0:
            continue
        geom = SimpleNamespace(size=model.geom_size[gid], type=model.geom_type[gid],
            pos=model.geom_pos[gid], quat=model.geom_quat[gid], rgba=model.geom_rgba[gid])
        visual = (_mj_mesh_geometry(geom, f'geom/{gid}', meta, resolved, {}) if meta.mesh_name
                  else _mj_geometry(geom, f'geom/{gid}'))
        if visual:
            objects[bid]['geometries'].append(visual.to_payload())
    common = dict(protocol='bsk-render/2', session_id='saved-native-' + model_hash[:12])
    # Keep the launch-configured overview. The legacy orbit camera starts below
    # the task-box plane and hides the mounting surface.
    settings = SceneSettings(origin_object_id='', default_camera_target='',
        default_camera_distance_m=3.5, orbit_lines=False, trajectory_history=False,
        fill_light_intensity_lux=100000., headlight_enabled=True,
        headlight_diffuse_rgb=(10000., 10000., 10000.)).to_payload()
    count, next_ns, last_time, stage = 0, 0, -1, None
    def frame(row):
        nonlocal count, last_time
        qpos = np.asarray(row['qpos'], dtype=float)
        if qpos.shape != (model.nq,) or not np.isfinite(qpos).all():
            raise ValueError('Invalid recorded qpos')
        data.qpos[:] = qpos
        mujoco.mj_kinematics(model, data)  # Forward kinematics only; no mj_step.
        origin = data.body('cubesat_bus').xpos.copy()
        result = dict(common, type='frame', manifest_revision='1', frame_id=str(count),
            sim_time_ns=str(row['sim_time_ns']), wall_time_ns='0', origin_N_m=origin.tolist(),
            c_LN=np.eye(3).reshape(-1).tolist(), celestial_bodies=[], visual_states=[],
            objects=[dict(object_id=obj['object_id'], position_m=(data.xpos[bid]-origin).tolist(),
                orientation_wxyz=data.xquat[bid].tolist()) for bid, obj in objects.items()])
        count += 1
        last_time = row['sim_time_ns']
        return result
    with BskRecordingWriter(output) as writer:
        writer.write(dict(common, type='hello', capabilities=['scene_manifest', 'recording', 'events'],
            required_capabilities=['scene_manifest'], coordinates=dict(length_unit='m', handedness='right',
                position_frame='L', quaternion_order='wxyz', rotation_semantics='active_parent_from_child')))
        writer.write(dict(common, type='scene_manifest', revision='1', objects=list(objects.values()),
            celestial_bodies=[], visuals=[], cameras=compiled_cameras(model), settings=settings))
        writer.write(dict(common, type='event', sequence='1', event_kind='replay_notice',
            payload=dict(severity='info', message='RECORDED NATIVE REPLAY | ' + label)))
        last = None
        with gzip.open(trace, 'rt', encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if last and row['sim_time_ns'] <= last['sim_time_ns']:
                    raise ValueError('Trace timestamps are not strictly increasing')
                if row['sim_time_ns'] >= next_ns or row.get('stage') != stage:
                    writer.write(frame(row))
                    next_ns = row['sim_time_ns'] + round(1e9 / fps)
                stage, last = row.get('stage'), row
        if last is None:
            raise ValueError('Empty native trace')
        if last['sim_time_ns'] != last_time:
            writer.write(frame(last))
    report = dict(mode='recorded_native_poses_only', dynamics_executed=False, label=label,
        model=str(model_path), model_sha256=model_hash, trace=str(trace), trace_sha256=digest(trace),
        recording=str(output), recording_sha256=digest(output), frame_count=count,
        duration_s=last_time / 1e9, source_meshes=len(rebound), bodies=len(objects),
        project=str(project), catalog=str(catalog_path),
        limitation='UE replay and readability lighting; not live physics or a new task acceptance')
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--trace', type=Path)
    parser.add_argument('--library', type=Path)
    parser.add_argument('--scene-id')
    parser.add_argument('--catalog', type=Path, action='append', required=True)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', default='Scene library')
    args = parser.parse_args()
    if args.library:
        if args.model or args.trace or not args.scene_id:
            parser.error('--library requires --scene-id and excludes --model/--trace')
        entries = json.loads(args.library.read_text(encoding='utf-8-sig'))['entries']
        entry = next(e for e in entries if e['scene_id'] == args.scene_id)
        for key in ('receipt', 'native_report', 'launch'):
            if digest(entry[key]) != entry[key + '_sha256']:
                raise ValueError(f'Accepted run artifact changed: {key}')
        native = Path(entry['native_report']).parent
        args.trace = native / 'states-200hz.jsonl.gz'
        if not args.trace.is_file():
            parser.error('This entry has no joint-state trace; orbital telemetry needs a separate viewer')
        args.model = Path(json.loads((native / 'run/summary.json').read_text(encoding='utf-8-sig'))['model'])
        args.label = args.scene_id
    if not args.model or not args.trace:
        parser.error('Supply --library/--scene-id or both --model/--trace')
    print(json.dumps(export(args.model.resolve(), args.trace.resolve(), args.catalog,
                            args.project.resolve(), args.output.resolve(), args.label)))
