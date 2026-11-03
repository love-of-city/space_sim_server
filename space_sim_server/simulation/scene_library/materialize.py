"""Materialize portable presets into locally fingerprinted runnable instances."""
import json
from pathlib import Path
import subprocess
import sys

from .catalog import catalog, phase_one_scope
from .manifest import compose, compose_preheld_operation, digest, write_json


def phase_one_presets():
    """Configurations are explicit candidates; runtime receipts prove acceptance."""
    entries = []
    for item in catalog()['scenes']:
        if item['phase'] != 1 or item['id'] in phase_one_scope()['deferred_scene_ids']:
            continue
        family, task = item['family_number'], item['task']
        variant = 'target-preheld' if family == 2 else ('cross-preheld' if family == 4 else None)
        options = {}
        if family in (2, 4) and task.endswith('_plug_insert'):
            options['transport_clearance_m'] = .16
        if family in (2, 4) and task == 'cylindrical_plug_insert':
            options['release_speed_rad_s'] = .02
        if family == 2 and task == 'drawer_cycle':
            options['seating_overtravel_m'] = .0016
        entry = dict(scene_id=item['id'], variant=variant, options=options)
        if family == 2 and task in ('aviation_plug_insert', 'guide_plug_insert'):
            entry['hold_gains'] = dict(kp=2500., kd=80., torque_limit_nm=80.)
        entries.append(entry)
    return entries


def materialize(repository, output, *, geometry_python, native_python=None, scene_ids=None,
                capture_reference=None, arm_mount_offset=(.2, -.2, .25)):
    """No network, package installation, shell syntax, or mutation of source assets."""
    repository, output = Path(repository).resolve(), Path(output).resolve()
    entries = phase_one_presets()
    if scene_ids is not None:
        selected = set(scene_ids)
        unknown = selected - {e['scene_id'] for e in entries}
        if unknown:
            raise ValueError(f'Scene outside current phase-one scope: {sorted(unknown)}')
        entries = [e for e in entries if e['scene_id'] in selected]
        if not entries:
            raise ValueError('At least one scene is required')
    if any(e['variant'] for e in entries) and capture_reference is None:
        raise ValueError('A capture-arm-held target requires an explicit capture reference calibration.')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'instances').mkdir()
    record = dict(schema='space-arm-materialized-library/1', scope=phase_one_scope(),
                  repository=str(repository), entries=[], commands=[], accepted=False,
                  status='preparing', builder_sha256=digest(__file__),
                  presets=entries, validation='build_and_initial_pose_only')

    def prepare(argv, log_name):
        record['commands'].append(argv)
        with (output / log_name).open('x', encoding='utf-8') as log:
            result = subprocess.run(argv, cwd=repository, stdout=log, stderr=subprocess.STDOUT, check=False)
        if result.returncode:
            raise RuntimeError(f'Preparation failed ({result.returncode}); see {log_name}')

    native_python = str(native_python or sys.executable)
    geometry_python = str(geometry_python)
    try:
        for variant in sorted({e['variant'] for e in entries if e['variant']}):
            folder = output / variant
            command = [geometry_python, str(repository / 'tools/build_preheld_dual_scene.py'),
                '--source', str(repository / 'model/SARM/platform/sarm_dual_capture26.xml'),
                '--reference', str(Path(capture_reference).resolve()), '--output', str(folder),
                '--arm-mount-offset', *map(str, arm_mount_offset)]
            if variant == 'cross-preheld':
                command.append('--cross')
            prepare(command, f'{variant}-build.log')
        for entry in entries:
            variant = entry['variant']
            if variant:
                folder = output / variant
                initial = folder / 'initial-scene.json'
                if entry.get('hold_gains'):
                    initial_data = json.loads(initial.read_text(encoding='utf-8-sig'))
                    initial_data.setdefault('preheld_capture', {})['hold_gains'] = entry['hold_gains']
                    initial = folder / (entry['scene_id'].split('/')[1] + '-initial-scene.json')
                    write_json(initial, initial_data)
                if entry['scene_id'].endswith('/drawer_cycle'):
                    prepare([native_python, str(repository / 'tools/prepare_scene_initial_pose.py'),
                        '--model', str(folder / 'preheld-dual.xml'), '--scene', str(initial),
                        '--output', str(folder / 'drawer-initial-pose')], f'{variant}-drawer-initial-pose.log')
                    initial = folder / 'drawer-initial-pose/initial-scene.json'
                inspection = folder / (entry['scene_id'].split('/')[1] + '-inspection.json')
                prepare([geometry_python, str(repository / 'tools/audit_preheld_scene.py'),
                    '--model', str(folder / 'preheld-dual.xml'), '--scene', str(initial),
                    '--output', str(inspection)], variant + '-' + inspection.stem + '.log')
                instance = compose_preheld_operation(entry['scene_id'], repository,
                    folder / 'assembly.json', initial,
                    mount_inspection=inspection, **entry['options'])
            else:
                instance = compose(entry['scene_id'], repository, seed=7 if entry['scene_id'].startswith('03_') else 42)
            relative = Path('instances') / (entry['scene_id'].replace('/', '--') + '.json')
            write_json(output / relative, instance)
            record['entries'].append(dict(scene_id=entry['scene_id'], instance=relative.as_posix(),
                                          instance_sha256=digest(output / relative), accepted=False))
        record['status'] = 'ready_to_run'
    except Exception as exc:
        record.update(status='preparation_failed', error=f'{type(exc).__name__}: {exc}')
        write_json(output / 'library.json', record)
        raise
    write_json(output / 'library.json', record)
    return record


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
