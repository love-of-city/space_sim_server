"""Replace one installed initialization with an explicitly unaccepted layout.

Keeps the four family menus and stable task IDs. The previous installed index
is retained, and prior task receipts are not assigned to the new geometry.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]
from simulation.scene_library.manifest import compose_preheld_operation, digest, write_json
from space_arm_platform.imported_scene_library import read, verified_source
from import_platform_scene_library import cameras


def register(scene_id, model, scene, audit, output, index_path, *, target_site=None,
             layout_revision='overview-direction-v2', description=None, supplemental_catalog=None):
    index = read(index_path)
    entry = next(e for e in index['entries'] if e['scene_id'] == scene_id)
    original = deepcopy(entry)
    prior = verified_source(original)
    if cameras(model) != entry['camera_contract']:
        raise ValueError('Candidate changes original cameras')
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'previous-index.json', index)
    initial = read(scene)
    assembly = dict(schema='space-arm-dual-arm-preheld/1', model=str(model.resolve()),
        model_sha256=digest(model), cross_transfer=scene_id.startswith('04_'),
        target_free_joint_preserved=True, service_target_rigid_mount=False,
        source_model=prior['model'], source_model_sha256=digest(prior['model']),
        target_position_m=initial['randomization']['target_position_m'],
        target_orientation_wxyz=initial['randomization']['target_orientation_wxyz'],
        capture_joints=initial['randomization']['preheld_capture_joint_position'],
        camera_unchanged=True, full_task_accepted=False)
    builder_assembly = model.with_name('assembly.json')
    if builder_assembly.exists():
        assembly.update(layout_builder_assembly=str(builder_assembly.resolve()),
                        layout_builder_assembly_sha256=digest(builder_assembly))
    write_json(output/'assembly.json', assembly)
    fixture = prior.get('operation_fixture', {})
    source = compose_preheld_operation(scene_id, ROOT, output/'assembly.json', scene,
        mount_inspection=audit, transport_clearance_m=fixture.get('transport_clearance_m'),
        seating_overtravel_m=fixture.get('seating_overtravel_m'),
        release_speed_rad_s=fixture.get('release_speed_rad_s'), target_site=target_site)
    write_json(output/'instance.json', source)
    catalog = read(Path(entry['catalog']))
    if supplemental_catalog is not None:
        added = read(supplemental_catalog)
        for path, asset in added['assets'].items():
            if digest(path) != asset['source_sha256']:
                raise ValueError('Supplemental UE source mesh bytes changed')
        catalog['assets'].update(added['assets'])
    catalog['source_mjcf'] = str(model.resolve())
    write_json(output/'catalog.json', catalog)
    receipt = dict(schema='layout-candidate-registration/1', full_task_accepted=False,
        geometry_audit=str(audit.resolve()), geometry_audit_sha256=digest(audit),
        model_sha256=digest(model), scene_sha256=digest(scene),
        docking_stereo_required=False, camera_unchanged=True, previous_entry=original)
    write_json(output/'candidate-receipt.json', receipt)
    for key, file in [('source_instance','instance.json'), ('receipt','candidate-receipt.json'), ('catalog','catalog.json')]:
        entry[key] = str((output/file).resolve())
        entry[key+'_sha256'] = digest(output/file)
    entry.update(layout_revision=layout_revision, full_task_accepted=False,
        description=description or '已对接初态；调整两星相对方位以匹配原总览相机。新布局完成静态检查，完整任务待复验；不评审对接双目。')
    verified_source(entry)
    index['layout_revision'] = layout_revision + '-candidate'
    index_path.write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return {'id':entry['id'], 'model':str(model), 'full_task_accepted':False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-id', required=True)
    for name in ['model','scene','audit','output']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--index', type=Path, default=ROOT/'run/imported_scene_library.json')
    parser.add_argument('--target-site')
    parser.add_argument('--layout-revision', default='overview-direction-v2')
    parser.add_argument('--description')
    parser.add_argument('--supplemental-catalog', type=Path)
    args = parser.parse_args()
    print(json.dumps(register(args.scene_id,args.model,args.scene,args.audit,args.output,args.index,
        target_site=args.target_site,layout_revision=args.layout_revision,description=args.description,supplemental_catalog=args.supplemental_catalog)))
