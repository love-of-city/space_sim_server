"""Collect one attempt per enabled task, grouped by a shuffled layout pass."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import time
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from collection.collect_scripted_tasks import CollectionSession, ROOT, main as collect_task
from space_arm_platform.collection_layouts import LayoutPool
from space_arm_platform.collection_processes import ManagerLease
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.randomization import generator
from simulation.scripted_tasks.threaded_tasks import THREADED_TASKS

TaskName = Literal['drawer_cycle', 'aviation_plug_insert', 'guide_plug_insert',
                   'cylindrical_plug_insert', 'locking_rod_cycle', 'aviation_bolt_insert']
TASK_NAMES = tuple(TaskName.__args__)
ORDER_STREAM = 'layout-sequence-without-replacement-v1'
TERMINAL = {'accepted', 'skipped', 'interrupted'}


class TaskConfig(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    name: TaskName
    enabled: bool = True


class SequenceConfig(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    schema_version: Literal['layout-sequence/1'] = Field(default='layout-sequence/1', alias='schema')
    layout_pool: str
    scene: str | None = None
    tasks: list[TaskConfig] = Field(default_factory=lambda: [
        TaskConfig(name=name, enabled=name != 'aviation_bolt_insert') for name in TASK_NAMES])
    passes: int = Field(default=1, ge=1, le=1000)
    layouts_per_pass: int | None = Field(default=None, ge=1, le=1000)
    seed: int = Field(default=2026102800, ge=0, le=2**31-1)
    range_deg: float = Field(default=30., ge=0, le=50, allow_inf_nan=False)
    baseline_joint_deg: list[float] = Field(default_factory=lambda: [90., -60., 60., 0., -90., 0.], min_length=6, max_length=6)
    randomize_orbit_phase: bool = False
    recording_config: str = 'recording-veryfast.json'
    adapter: str | None = None
    unreal: str | None = None
    port_base: int = Field(default=24000, ge=1024, le=65532)
    gpu_index: int = Field(default=0, ge=0, le=255)
    warmup_seconds: float = Field(default=5., ge=0, le=60, allow_inf_nan=False)
    reuse_warmup_seconds: float = Field(default=5., ge=0, le=60, allow_inf_nan=False)
    simulation_startup_timeout_seconds: float = Field(default=600., gt=0, le=7200, allow_inf_nan=False)
    dynamics_max_episodes: int = Field(default=50, ge=1, le=1000)
    dynamics_max_rss_mb: int = Field(default=8192, ge=1)
    threaded_speed_profile: Literal['default', 'fast_collection'] = 'fast_collection'

    @model_validator(mode='after')
    def check_tasks(self):
        import math
        if not self.tasks or not any(t.enabled for t in self.tasks):
            raise ValueError('Enable at least one task')
        if len({t.name for t in self.tasks}) != len(self.tasks):
            raise ValueError('Duplicate tasks are not allowed')
        if not all(math.isfinite(v) for v in self.baseline_joint_deg):
            raise ValueError('Baseline joints must be finite')
        return self


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def load_config(path):
    path = Path(path).resolve()
    config = SequenceConfig.model_validate_json(path.read_bytes())
    for name in ('layout_pool', 'scene', 'recording_config', 'adapter', 'unreal'):
        value = getattr(config, name)
        if value is not None:
            candidate = Path(value).expanduser()
            setattr(config, name, str((path.parent / candidate).resolve()))
    return config


def make_plan(config, pool):
    entries = pool.document['layouts']
    if config.layouts_per_pass is not None and config.layouts_per_pass > len(entries):
        raise ValueError('layouts_per_pass exceeds the pool size')
    enabled = [t.name for t in config.tasks if t.enabled]
    groups = []
    for pass_index in range(config.passes):
        indices = generator(config.seed, ORDER_STREAM, pass_index).permutation(len(entries))
        if config.layouts_per_pass is not None:
            indices = indices[:config.layouts_per_pass]
        for position, index in enumerate(indices):
            index = int(index)
            layout_id = entries[index]['id']
            # Include the pool index: arbitrary ids cannot escape or collide on disk.
            folder = f'{index:03d}-' + re.sub(r'[^A-Za-z0-9_-]', '_', layout_id)
            group_id = f'pass-{pass_index+1:03d}/{folder}'
            layout_seed = config.seed + (pass_index * len(entries) + position) * 7
            groups.append(dict(id=group_id, pass_number=pass_index+1, pool_index=index,
                               layout_id=layout_id, layout_seed=layout_seed,
                               jobs=[dict(id=group_id+'/'+task, task=task,
                                          seed=layout_seed+TASK_NAMES.index(task)+1,
                                          relative_output=task+'/'+group_id)
                                     for task in enabled]))
    return dict(schema='layout-sequence-plan/1', pool_sha256=pool.sha256,
                pool_size=len(entries), order_stream=ORDER_STREAM,
                task_order=enabled, groups=groups,
                total_attempts=sum(len(g['jobs']) for g in groups))


def source_scene(config):
    if config.scene is not None:
        scene = json.loads(Path(config.scene).read_text(encoding='utf-8-sig'))
    else:
        from space_arm_platform.models import SceneInstanceCreate
        from space_arm_platform.scene_runtime import _sample_instance
        scene = _sample_instance(SceneInstanceCreate(
            template_id='sarm-task-box-20261012',
            randomization_profile='teleop-zero-prepare-v1',
            operating_arm_joint_position_deg=config.baseline_joint_deg), config.seed)
    if scene.get('template_id') != 'sarm-task-box-20261012':
        raise ValueError('Layout sequence requires the SARM workbench scene')
    if scene.get('created_by') is None:
        scene.pop('created_by', None)
    # A supplied scene is a baseline, not a previous task's mutable runtime state.
    for key in ('task_box_layout', 'collection_fixed_layout', 'collection_config',
                'scripted_task_name', 'target_site', 'overview_camera_seed', 'overview_camera'):
        scene.pop(key, None)
    scene['randomize_overview_camera'] = True
    scene['overview_camera_sampling'] = 'presets'
    scene['randomize_orbit_phase'] = config.randomize_orbit_phase
    scene['initialization_enabled'] = True
    return scene


def prepare_layout(source, group, pool):
    model, catalog, placement, selection = pool.select_index(
        group['pool_index'], group['layout_seed'], stream=ORDER_STREAM)
    common = randomized_scene(source, group['layout_seed'], 0., 'drawer_cycle')
    return dict(model=str(model), catalog=str(catalog), placement=placement,
                selection=selection, color_assignment=common['collection_config']['episode_color_assignment'],
                overview_camera=common['overview_camera'], overview_camera_seed=group['layout_seed'])


def prepare_task_scene(source, config, group, job, layout):
    baseline = deepcopy(source)
    baseline['overview_camera_seed'] = layout['overview_camera_seed']
    scene = randomized_scene(baseline, job['seed'], config.range_deg, job['task'],
                             color_assignment=layout['color_assignment'])
    scene['scripted_task_name'] = job['task']
    scene['task_box_layout'] = deepcopy(layout['placement'])
    scene['collection_layout_randomization'] = True
    cfg = scene['collection_config']
    cfg['layout_randomization'] = deepcopy(layout['selection'])
    cfg['layout_sequence'] = dict(group_id=group['id'], job_id=job['id'],
                                 pass_number=group['pass_number'], layout_id=group['layout_id'],
                                 layout_seed=group['layout_seed'], arm_seed=job['seed'])
    return scene


def quarantine_episode(output, job):
    """Move unaccepted captures out of the training-data folder, without deleting evidence."""
    destination = Path(output) / job['relative_output']
    if (destination/'episodes').exists():
        (destination/'episodes').rename(destination/'rejected')
        return str(destination/'rejected')
    return None


def run_attempt(config, source_path, output, group, job, layout, scene, session):
    destination = output / job['relative_output']
    args = ['--task', job['task'], '--scene', str(source_path),
            '--model', layout['model'], '--catalog', layout['catalog'],
            '--output', str(output/'attempts'/job['relative_output']),
            '--episodes-root', str(destination/'episodes'),
            '--cancel-file', str(output/'cancel.request'),
            '--count', '1', '--max-attempts', '1', '--seed', str(job['seed']),
            '--range-deg', str(config.range_deg), '--reuse-renderer', '--reuse-dynamics', '--offline',
            '--recording-config', config.recording_config, '--gpu-index', str(config.gpu_index),
            '--dynamics-max-episodes', str(config.dynamics_max_episodes),
            '--dynamics-max-rss-mb', str(config.dynamics_max_rss_mb),
            '--warmup-seconds', str(config.warmup_seconds),
            '--reuse-warmup-seconds', str(config.reuse_warmup_seconds),
            '--simulation-startup-timeout-seconds', str(config.simulation_startup_timeout_seconds)]
    for offset, name in enumerate(('control', 'capture', 'render', 'signalling')):
        args += [f'--{name}-port', str(config.port_base + offset)]
    for name in ('adapter', 'unreal'):
        if getattr(config, name) is not None:
            args += ['--'+name, getattr(config, name)]
    if job['task'] in THREADED_TASKS:
        args += ['--threaded-speed-profile', config.threaded_speed_profile]
    collect_task(argv=args, session=session, prepared_scene=scene)
    summary = json.loads((output/'attempts'/job['relative_output']/'summary.json').read_text())
    if summary['status'] == 'cancelled':
        return dict(status='cancelled', reason='operator_cancelled',
                    rejected_directory=quarantine_episode(output, job))
    attempt = summary['attempts'][0]
    accepted = attempt['status'] == 'accepted'
    result = dict(status='accepted' if accepted else 'skipped', category=attempt['status'],
                  reason=(attempt.get('preflight', {}).get('reason')
                          or (attempt.get('task_result') or {}).get('reason')
                          or attempt['status']),
                  error=attempt.get('error'),
                  attempt_summary=str(output/'attempts'/job['relative_output']/'summary.json'),
                  timing={key: value for key, value in attempt.items()
                          if key.endswith('_wall_seconds') or key == 'recorded_sim_seconds'},
                  accepted_episodes=summary['accepted_episodes'])
    # Failed/incomplete recordings are audit evidence, never mixed into episodes/.
    if not accepted:
        result['rejected_directory'] = quarantine_episode(output, job)
    return result


def execute_plan(config, pool, plan, source, output, *, resume=False, runner=run_attempt,
                 session_factory=CollectionSession):
    output = Path(output)
    progress_path = output/'progress.json'
    progress = (json.loads(progress_path.read_text()) if resume else
                dict(schema='layout-sequence-progress/1', jobs={}, wall_seconds=0.))
    started, previous = time.monotonic(), progress['wall_seconds']
    progress.update(status='running', total_attempts=plan['total_attempts'])
    progress.pop('error', None)
    session = session_factory(output)

    def save():
        progress['wall_seconds'] = previous + time.monotonic() - started
        progress['accepted'] = sum(r['status'] == 'accepted' for r in progress['jobs'].values())
        progress['skipped'] = sum(r['status'] in {'skipped', 'interrupted'} for r in progress['jobs'].values())
        atomic_json(progress_path, progress)

    try:
        # A crash or infrastructure fault never causes an implicit replay on resume.
        jobs_by_id = {job['id']: job for group in plan['groups'] for job in group['jobs']}
        for identifier, result in progress['jobs'].items():
            if result['status'] in {'running', 'infrastructure_failed', 'cancelled', 'interrupted'}:
                result['rejected_directory'] = quarantine_episode(output, jobs_by_id[identifier])
                result.update(status='interrupted', reason='previous_attempt_interrupted_no_retry')
        save()
        for group in plan['groups']:
            pending = [j for j in group['jobs'] if progress['jobs'].get(j['id'], {}).get('status') not in TERMINAL]
            if not pending:
                continue
            if (output/'cancel.request').exists():
                progress['status'] = 'cancelled'
                return progress
            preparation_started = time.monotonic()
            layout = prepare_layout(source, group, pool)
            atomic_json(output/'layouts'/group['id']/'initialization.json', layout)
            progress.setdefault('layouts', {})[group['id']] = dict(
                preparation_wall_seconds=time.monotonic()-preparation_started)
            for job in pending:
                if (output/'cancel.request').exists():
                    progress['status'] = 'cancelled'
                    return progress
                t0 = time.monotonic()
                progress['current_job'] = job['id']
                progress['jobs'][job['id']] = dict(status='running', started_at_ns=str(time.time_ns()))
                save()
                scene = prepare_task_scene(source, config, group, job, layout)
                result = runner(config, output/'source-scene.json', output, group, job, layout, scene, session)
                result.update(started_at_ns=progress['jobs'][job['id']]['started_at_ns'],
                              finished_at_ns=str(time.time_ns()), wall_seconds=time.monotonic()-t0)
                progress['jobs'][job['id']] = result
                save()
                print(json.dumps(dict(job=job['id'], status=result['status'],
                                      wall_seconds=result['wall_seconds']), ensure_ascii=False), flush=True)
                if result['status'] == 'cancelled':
                    progress['status'] = 'cancelled'
                    return progress
        progress['status'] = 'complete'
        progress.pop('current_job', None)
        return progress
    except BaseException as error:
        progress['status'] = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'infrastructure_failed'
        progress['error'] = f'{type(error).__name__}: {error}'
        current = progress['jobs'].get(progress.get('current_job'))
        if current is not None and current['status'] == 'running':
            current.update(status=progress['status'], error=progress['error'])
            try:
                current['rejected_directory'] = quarantine_episode(output, jobs_by_id[progress['current_job']])
            except OSError as cleanup_error:
                current['quarantine_error'] = str(cleanup_error)
        raise
    finally:
        try:
            session.close()
        finally:
            save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='Defaults to config/collection.json; resume uses its saved snapshot')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--layouts', type=int, help='Limit layouts per pass, e.g. 2 for a short run')
    parser.add_argument('--plan-only', action='store_true', help='Write the plan without starting UE or dynamics')
    parser.add_argument('--resume', action='store_true', help='Continue pending jobs without retrying interrupted/failed jobs')
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    if args.resume and (args.layouts is not None or args.config is not None):
        parser.error('Resume uses frozen inputs; do not pass --layouts or --config')
    if args.resume:
        config = load_config(output/'config.snapshot.json')
    else:
        config = load_config(args.config or ROOT/'config/collection.json')
        if args.layouts is not None:
            config = SequenceConfig.model_validate({**config.model_dump(by_alias=True), 'layouts_per_pass': args.layouts})
    pool = LayoutPool(config.layout_pool)  # Geometry/hash checks only; no task certification.
    plan = make_plan(config, pool)
    if args.resume:
        if json.loads((output/'plan.json').read_text()) != plan:
            raise ValueError('Pool or frozen plan changed; use a new output directory')
        source = json.loads((output/'source-scene.json').read_text())
        inputs = json.loads((output/'inputs.json').read_text())
        for name, path in [('source_scene', output/'source-scene.json'),
                           ('recording_config', Path(config.recording_config)),
                           ('config', output/'config.snapshot.json')]:
            if hashlib.sha256(path.read_bytes()).hexdigest() != inputs[name+'_sha256']:
                raise ValueError('Frozen input changed: '+name)
    else:
        from collection.record_scripted_task import recording_settings
        recording_settings(Path(config.recording_config))
        source = source_scene(config)
        output.mkdir(parents=True, exist_ok=False)
        atomic_json(output/'config.snapshot.json', config.model_dump(by_alias=True))
        atomic_json(output/'plan.json', plan)
        atomic_json(output/'pool.snapshot.json', pool.document)
        atomic_json(output/'source-scene.json', source)
        atomic_json(output/'inputs.json', {name+'_sha256': hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in [('source_scene', output/'source-scene.json'),
                               ('recording_config', Path(config.recording_config)),
                               ('config', output/'config.snapshot.json')]})
        atomic_json(output/'progress.json', dict(schema='layout-sequence-progress/1', status='planned', jobs={}, wall_seconds=0.))
    print(json.dumps(dict(output=str(output), layouts=len(plan['groups']), attempts=plan['total_attempts'],
                          tasks=plan['task_order'], plan_only=args.plan_only)), flush=True)
    if args.plan_only:
        return 0
    lease = ManagerLease(output/'collector.lock')
    try:
        result = execute_plan(config, pool, plan, source, output, resume=True)
    finally:
        lease.close()
    return 0 if result['status'] == 'complete' else 1
