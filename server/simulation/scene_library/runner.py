"""Validate catalog instances with the single current expert and frozen replay."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys

from .manifest import digest, verify_files, write_json

ROOT = Path(__file__).resolve().parents[2]
REPLAY_TASKS = {'drawer_cycle', 'aviation_plug_insert', 'guide_plug_insert', 'cylindrical_plug_insert'}


def runtime_fingerprints():
    paths = {*ROOT.glob('config/**/*.json'), *ROOT.glob('run/native_acceleration/*.json')}
    override = os.environ.get('SPACE_SIM_PLUG_RETENTION_CONFIG')
    if override:
        paths.add(Path(override).resolve())
    for manifest in ROOT.glob('run/native_acceleration/*.json'):
        record = json.loads(manifest.read_text(encoding='utf-8-sig'))
        if record.get('library'):
            paths.add(ROOT / record['library'])
    return {str(path): digest(path) for path in sorted(paths)}


def run(instance, output, *, python=None, geometry_python=None):
    verify_files(instance)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'instance.json', instance)
    recipe = instance['recipe']
    receipt = dict(schema='space-arm-scene-run/1', scene_id=recipe['id'], accepted=False,
                   started_utc=datetime.now(timezone.utc).isoformat(), ue_visual_verified=False, physical_docking_accepted=False,
                   status='preparing', operation_accepted=False, missing_capabilities=recipe['missing_capabilities'], controller_family='current_local_expert')
    if instance.get('operation_fixture') or (recipe['family_number'] == 1 and recipe['task'] not in REPLAY_TASKS) or recipe['family_number'] not in (1, 3):
        receipt.update(status='unsupported', reason='current_expert_scene_adapter_or_independent_scoring_not_validated')
        write_json(output / 'receipt.json', receipt)
        return receipt
    interpreter = str(python or sys.executable)
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join([str(ROOT), str(ROOT/'backend'),
           str(ROOT.parent/'ue-adapter/Adapters'), str(ROOT.parent/'ue-adapter/Unreal/BskUnrealRenderer/examples')])}
    commands = []
    try:
        if recipe['family_number'] == 3:
            command = [interpreter, '-m', 'simulation.orbital_rendezvous.run', '--scenario', 'in-orbit',
                       '--seed', str(instance['seed']), '--output-dir', str(output/'native')]
            commands.append(command)
            with (output/'native.log').open('w') as log:
                process = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            report = json.loads((output/'native/mission_report.json').read_text())
            checks = report.get('checks', {})
            receipt.update(accepted=process.returncode == 0 and bool(checks) and all(v is True for v in checks.values()),
                           native_report=str(output/'native/mission_report.json'),
                           evidence_scope='far_field_rendezvous_and_hold_only')
        else:
            from tools.diagnostics.evaluate_fixed_trajectories import canonical, evaluate
            from types import SimpleNamespace
            scene = output/'initial-scene.json'
            write_json(scene, instance['initial_scene'])
            args = SimpleNamespace(task=recipe['task'], model=Path(instance['model']), scene=scene,
                                   output=output/'canonical', max_sim_seconds=420., wall_timeout=3600.)
            if Path(interpreter).resolve() != Path(sys.executable).resolve():
                raise ValueError('Scene validation must run in the explicitly selected current Python')
            if canonical(args) != 0:
                raise RuntimeError('Current expert did not produce a complete reference trace')
            args.canonical_dir=args.output; args.output=output/'replay'; args.trials=1
            evaluate(args)
            report=json.loads((args.output/'aggregate.json').read_text())
            receipt.update(accepted=report.get('counts', {}).get('success') == 1, native_report=str(args.output/'aggregate.json'),
                           evidence_scope='current_expert_frozen_actuator_replay')
        receipt['status'] = 'passed' if receipt['accepted'] else 'failed'
    except Exception as error:
        receipt.update(status='infrastructure_error', error=f'{type(error).__name__}: {error}')
    receipt.update(finished_utc=datetime.now(timezone.utc).isoformat())
    write_json(output/'launch.json', dict(argv=commands, runtime_configuration_and_binary_sha256=runtime_fingerprints()))
    write_json(output/'receipt.json', receipt)
    return receipt


def run_batch(instances, output, *, python=None, geometry_python=None):
    if not instances:
        raise ValueError('Batch requires at least one instance')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    runs=[]
    for index, instance in enumerate(instances):
        folder=output/f'{index:03d}'
        try:
            result=run(instance,folder,python=python,geometry_python=geometry_python)
        except Exception as error:
            result=dict(accepted=False,status='infrastructure_error',error=str(error))
        runs.append(dict(output=str(folder),**result))
    result=dict(schema='space-arm-scene-batch/1',runs=runs,
                requested_scopes_passed=all(r.get('accepted') for r in runs),
                complete_scenes_passed=all(r.get('accepted') for r in runs))
    write_json(output/'batch-result.json',result)
    return result
