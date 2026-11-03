"""Seeded generation of complete, independent two-spacecraft task-box layouts.

Content-pinned results are cached for replay; legacy entries can use a pool.
Geometry acceptance is not trajectory or full-task acceptance.
"""
from copy import deepcopy
import json
from pathlib import Path
import hashlib
import os
import subprocess
import threading
import uuid

from .imported_scene_library import digest, verified_source

STREAM = 'task-box-layout-selection-v1'
GENERATION_STREAM = 'seeded-dual-taskbox/1'
_generation_lock = threading.Lock()


def generation_config(entry):
    path = entry.get('task_box_layout_generator')
    if not path or digest(path) != entry.get('task_box_layout_generator_sha256'):
        raise ValueError('任务盒生成配置缺失或已改变，请重新登记')
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if config.get('schema') != 'seeded-dual-taskbox-config/1':
        raise ValueError('Unsupported task-box generation configuration')
    for item in config['dependencies']:
        if digest(item['path']) != item['sha256']:
            raise ValueError('Task-box generation input changed: '+item['path'])
    return config


def generate_layout(entry, seed):
    """On-demand continuous generation; cache is keyed by seed and all inputs."""
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError('Seed must be an integer in [0, 2**32)')
    source = verified_source(entry)
    config = generation_config(entry)
    if digest(source['model']) != config['base_model_sha256']:
        raise ValueError('Layout generator belongs to another installed base model')
    fingerprint = entry['task_box_layout_generator_sha256']
    key = hashlib.sha256(f'{GENERATION_STREAM}:{fingerprint}:{seed}'.encode()).hexdigest()
    cache = Path(config['cache_root'])/key
    cache.mkdir(parents=True, exist_ok=True)
    receipt = cache/'ready.json'
    with _generation_lock:
        if not receipt.exists():
            output = cache/('build-'+uuid.uuid4().hex[:12])
            script = Path(__file__).resolve().parents[2]/'tools/prepare_seeded_dual_taskbox.py'
            env = dict(os.environ, PYTHONUTF8='1')
            with (cache/'prepare.log').open('a', encoding='utf-8') as log:
                result = subprocess.run([config['geometry_python'], str(script), '--config',
                    entry['task_box_layout_generator'], '--seed', str(seed), '--output', str(output)],
                    cwd=script.parents[1], env=env, stdout=log, stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            if result.returncode:
                raise ValueError('任务盒随机生成未通过，已保留失败记录：'+str(output))
            journal = output/'pipeline.json'
            record = json.loads(journal.read_text(encoding='utf-8'))
            if not record['completed'] or record['seed'] != seed:
                raise ValueError('Incomplete generated task-box result')
            temporary = receipt.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(pipeline=str(journal), sha256=digest(journal))), encoding='utf-8')
            os.replace(temporary, receipt)
        ref = json.loads(receipt.read_text(encoding='utf-8'))
        if digest(ref['pipeline']) != ref['sha256']:
            raise ValueError('Generated layout receipt changed')
        result = json.loads(Path(ref['pipeline']).read_text(encoding='utf-8'))
        pool_path = result['registered']['pool']
        if digest(pool_path) != result['result_sha256']:
            raise ValueError('Generated layout evidence changed')
        pool = json.loads(Path(pool_path).read_text(encoding='utf-8'))
        if pool['base_sources'][entry['id']] != entry['source_instance_sha256']:
            raise ValueError('Generated layout belongs to another task revision')
        selected = deepcopy(pool['variants'][0]['entries'][entry['id']])
        source = verified_source(selected)
        record = source['task_box_layout']
        if not record.get('complete_parts_on_both_spacecraft'):
            raise ValueError('Generated layout omitted task-box parts')
        selection = dict(enabled=True, stream=GENERATION_STREAM, selection_seed=seed,
            generation_seed=seed, generation_attempt=result['generation_attempt'],
            role_seeds=result['role_seeds'], generator_sha256=fingerprint,
            layout_id=key, model_sha256=record['model_sha256'], catalog_sha256=selected['catalog_sha256'],
            acceptance_scope='initial_geometry', full_task_accepted=False,
            target=deepcopy(record['target']), service=deepcopy(record['service']))
        return selected, selection, record


def select_layout(entry, seed):
    if entry.get('task_box_layout_generator'):
        return generate_layout(entry, seed)
    from simulation.scripted_tasks.randomization import generator

    verified_source(entry)
    path = entry.get('task_box_layout_pool')
    if not path:
        raise ValueError('当前场景没有已准备的任务盒随机布局池')
    if digest(path) != entry.get('task_box_layout_pool_sha256'):
        raise ValueError('Task-box layout pool changed; prepare and register it again')
    pool = json.loads(Path(path).read_text(encoding='utf-8'))
    if pool.get('schema') != 'dual-taskbox-layout-pool/1':
        raise ValueError('Unsupported two-spacecraft layout pool')
    if pool['base_sources'].get(entry['id']) != entry['source_instance_sha256']:
        raise ValueError('Layout pool belongs to another installed scene revision')
    variants = pool['variants']
    if len(variants) < 2 or len({v['id'] for v in variants}) != len(variants):
        raise ValueError('Layout pool requires distinct variants')
    index = int(generator(seed, STREAM).integers(len(variants)))
    variant = variants[index]
    selected = deepcopy(variant['entries'][entry['id']])
    if selected['id'] != entry['id'] or selected['base_template_id'] != entry['base_template_id']:
        raise ValueError('Layout variant changed the task or robot template')
    source = verified_source(selected)
    record = source['task_box_layout']
    if record['model_sha256'] != digest(source['model']):
        raise ValueError('Layout manifest does not match the selected model')
    selection = dict(enabled=True, stream=STREAM, selection_seed=seed,
                     pool_sha256=entry['task_box_layout_pool_sha256'], pool_size=len(variants),
                     pool_index=index, layout_id=variant['id'],
                     model_sha256=record['model_sha256'], catalog_sha256=selected['catalog_sha256'],
                     acceptance_scope='initial_geometry', full_task_accepted=False,
                     target=deepcopy(record['target']), service=deepcopy(record['service']))
    return selected, selection, record
