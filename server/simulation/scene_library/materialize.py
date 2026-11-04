"""Create current-controller scene candidates without historical expert presets."""
import json
from pathlib import Path
from .catalog import catalog, phase_one_scope
from .manifest import compose, digest, write_json


def phase_one_presets():
    return [dict(scene_id=item['id'], variant=None, options={})
            for item in catalog()['scenes'] if item['adapter'] is not None]


def materialize(repository, output, *, geometry_python=None, native_python=None, scene_ids=None):
    repository, output=Path(repository).resolve(), Path(output).resolve()
    entries=phase_one_presets()
    if scene_ids is not None:
        requested=set(scene_ids)
        unknown=requested-{entry['scene_id'] for entry in entries}
        if unknown or not requested:
            raise ValueError(f'No validated current-controller preparation adapter for: {sorted(unknown)}')
        entries=[entry for entry in entries if entry['scene_id'] in requested]
    output.mkdir(parents=True,exist_ok=False);(output/'instances').mkdir()
    records=[]
    for entry in entries:
        instance=compose(entry['scene_id'],repository,seed=7 if entry['scene_id'].startswith('03_') else 42)
        relative=Path('instances')/(entry['scene_id'].replace('/','--')+'.json')
        write_json(output/relative,instance)
        records.append(dict(scene_id=entry['scene_id'],instance=relative.as_posix(),
                            instance_sha256=digest(output/relative),accepted=False))
    result=dict(schema='space-arm-materialized-library/1',repository=str(repository),scope=phase_one_scope(),
                entries=records,accepted=False,status='ready_to_run',validation='manifest_only')
    write_json(output/'library.json',result)
    return result


def library_instances(library):
    """Only consume the exact generated manifests, including their dependency checks."""
    from .manifest import verify_files
    path = Path(library).resolve()
    record = json.loads(path.read_text(encoding='utf-8-sig'))
    if record.get('schema') != 'space-arm-materialized-library/1' or record.get('status') != 'ready_to_run':
        raise ValueError('Library is not ready to run')
    instances = []
    for entry in record['entries']:
        source = (path.parent / entry['instance']).resolve()
        if not source.is_relative_to(path.parent) or digest(source) != entry['instance_sha256']:
            raise ValueError('Library instance path or hash mismatch')
        instance = json.loads(source.read_text(encoding='utf-8-sig'))
        if instance['recipe']['id'] != entry['scene_id']:
            raise ValueError('Library scene identity mismatch')
        verify_files(instance)
        instances.append(instance)
    if not instances:
        raise ValueError('Empty library')
    return instances


def export_verified(output_root, output):
    """Publish a local selection of exact successful instances, never rerate traces."""
    from .manifest import verify_files
    from .status import summarize
    status = summarize(output_root)
    if not status['phase_one']['complete']:
        raise ValueError('Current phase-one scope is not complete; rejected/deferred tasks cannot be substituted')
    selected = []
    for scene in status['scenes']:
        if not scene['phase_one_required']:
            continue
        candidates = [Path(run['receipt']) for run in scene['runs'] if run['phase_one_accepted']
                      and Path(run['receipt']).with_name('instance.json').is_file()]
        if not candidates:
            raise ValueError(f'No reproducible successful instance for {scene["id"]}')
        receipt_path = candidates[-1]
        receipt = json.loads(receipt_path.read_text(encoding='utf-8-sig'))
        source = receipt_path.with_name('instance.json')
        instance = json.loads(source.read_text(encoding='utf-8-sig'))
        verify_files(instance)
        selected.append((scene['id'], source, receipt_path, Path(receipt['native_report'])))
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'instances').mkdir()
    entries = []
    for scene_id, source, receipt_path, native_path in selected:
        relative = Path('instances') / (scene_id.replace('/', '--') + '.json')
        (output / relative).write_bytes(source.read_bytes())
        item = json.loads(source.read_text(encoding='utf-8-sig'))['recipe']
        entries.append(dict(scene_id=scene_id, display_name=item['label'] + ' / ' + item['task_label'],
            instance=relative.as_posix(), instance_sha256=digest(source),
            receipt=str(receipt_path), receipt_sha256=digest(receipt_path),
            native_report=str(native_path), native_report_sha256=digest(native_path),
            launch=str(receipt_path.with_name('launch.json')),
            launch_sha256=digest(receipt_path.with_name('launch.json')),
            stdout_sha256=digest(receipt_path.with_name('stdout.log')),
            phase_one_accepted=True))
        flight_recording = native_path.parent / 'team_rendezvous.bskrec'
        flight_validation = native_path.parent / 'recording_validation.json'
        if scene_id.startswith('03_') and flight_recording.is_file() and flight_validation.is_file():
            if json.loads(flight_validation.read_text(encoding='utf-8')).get('passed') is not True:
                raise ValueError('Flight recording failed its independent pose audit')
            for key, path in [('ue_recording', flight_recording), ('ue_recording_validation', flight_validation),
                              ('ue_telemetry', native_path.parent / 'team_telemetry.npz')]:
                entries[-1][key], entries[-1][key + '_sha256'] = str(path), digest(path)
    record = dict(schema='space-arm-materialized-library/1', scope=phase_one_scope(),
        entries=entries, status='ready_to_run', accepted=False, phase_one_accepted=True,
        validation='verified_previous_native_runs_under_declared_scope_not_a_fresh_execution',
        physical_docking_accepted=False, ue_visual_verified=False, linux_verified=False)
    write_json(output / 'coverage.json', status)
    write_json(output / 'library.json', record)
    return record
