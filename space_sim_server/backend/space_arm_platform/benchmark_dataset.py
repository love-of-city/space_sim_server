"""Portable, typed method-input/action transitions from verified native episodes.

This is an imitation/estimation interchange, not a fabricated reward function
or a LeRobot robot-action mapping. Private physical logs are verified and used
for replay checks only; none are copied into policy features.
"""
from collections import Counter
import json
from pathlib import Path

from .benchmark_contract import load_task, parse_task
from .benchmark_execution import rescore_episode, sha256, verify_episode_evidence, write_json
from .benchmark_paths import evidence_path
from .benchmark_registry import method_input_types, parse_action

SCHEMA = 'spacesim-autonomy-policy-dataset/1'
TRANSITION_SCHEMA = 'spacesim-autonomy-policy-transition/1'
TERMINATED = {'success', 'safety_violation', 'task_failure'}
TRUNCATED = {'time_limit', 'method_error', 'infrastructure_error', 'cancelled'}
EPISODE_FIELDS = {'episode_id', 'configuration_id', 'task_fingerprint', 'action_type', 'seed', 'split',
                  'method', 'outcome', 'transitions', 'unpaired_action_requests', 'source_result_sha256',
                  'source_manifest_sha256', 'runtime_fingerprint', 'context_path', 'transitions_path'}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def read_rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def validated_inputs(task, context_raw, observations_raw, actions_raw, episode_id):
    context_type, observation_type = method_input_types(task)
    context = context_type.model_validate(context_raw)
    if context.task.fingerprint() != task.fingerprint():
        raise ValueError('method context differs from the frozen task')
    observations = [observation_type.model_validate(row) for row in observations_raw]
    if len(observations) < 2:
        raise ValueError('at least one complete observation/action transition is required')
    for index, observation in enumerate(observations):
        if (observation.episode_id != episode_id or observation.sequence != index
                or observation.sample_time_ns != index*task.decision_period_ns):
            raise ValueError('method observations have mixed episodes, gaps or changed sample times')
        if index and observation.delivery_time_ns <= observations[index-1].delivery_time_ns:
            raise ValueError('method observation delivery time must advance')
    if len(actions_raw) not in (len(observations)-1, len(observations)):
        raise ValueError('requested actions do not align with recorded observations')
    actions = [parse_action(task, row, observations[index]) for index, row in enumerate(actions_raw)]
    return context, observations, actions


def inspect_episode(directory, split):
    directory = evidence_path(directory)
    result, manifest = verify_episode_evidence(directory)
    required = {'task.json', 'context.json', 'observations.jsonl', 'actions.jsonl'}
    if not required.issubset(result['evidence_sha256']):
        raise ValueError('export requires hashed context, observations and actions; legacy contexts are not reconstructed')
    task = load_task(directory/'task.json')
    actual_split = task.seeds.split_for(result['seed'])
    if actual_split != split or result['split'] != split or manifest['split'] != split:
        raise ValueError('dataset split differs from the recorded frozen seed split; relabeling is forbidden')
    if (result['seed'] != manifest['seed'] or result['method'] != manifest['method']
            or result['configuration_id'] != task.configuration_id or manifest['configuration_id'] != task.configuration_id):
        raise ValueError('episode identity differs from its manifest or task')
    if result['outcome'] not in TERMINATED | TRUNCATED:
        raise ValueError('invalid or unfinished evidence cannot become policy training data')
    replay = rescore_episode(directory)
    if not replay['matches_recorded_physics_score'] or replay['outcome'] != result['outcome']:
        raise ValueError('independent physical replay differs from the archived result')
    context, observations, actions = validated_inputs(task, read_json(directory/'context.json'),
        read_rows(directory/'observations.jsonl'), read_rows(directory/'actions.jsonl'), manifest['episode_id'])
    unpaired = len(actions)-len(observations)+1
    if unpaired and result['outcome'] in TERMINATED | {'time_limit'}:
        raise ValueError('a complete physical episode cannot end with an unpaired action request')
    metadata = dict(episode_id=manifest['episode_id'], configuration_id=task.configuration_id,
        task_fingerprint=task.fingerprint(), action_type=task.action_type,
        seed=result['seed'], split=actual_split, method=result['method'], outcome=result['outcome'],
        transitions=len(observations)-1, unpaired_action_requests=unpaired,
        source_result_sha256=sha256(directory/'result.json'), source_manifest_sha256=sha256(directory/'manifest.json'),
        runtime_fingerprint=manifest.get('runtime_identity', {}).get('fingerprint'))
    return metadata, context, observations, actions


def export_dataset(episodes, output, *, split='train'):
    """Export all supplied episodes, preserving failures, split and action type."""
    if split not in {'train', 'validation', 'test'} or not episodes:
        raise ValueError('a declared split and at least one episode are required')
    directories = [evidence_path(path) for path in episodes]
    output = evidence_path(output)
    if output.exists() or len(set(directories)) != len(directories):
        raise ValueError('dataset output must be new and source episodes must be unique')
    if any(output.is_relative_to(path) or path.is_relative_to(output) for path in directories):
        raise ValueError('dataset output and source archives must be separate directories')
    # Validate the entire selection before publishing any dataset files. The
    # second pass also detects changes to sources while a long export runs.
    expected = [inspect_episode(path, split)[0] for path in directories]
    if len({item['episode_id'] for item in expected}) != len(expected):
        raise ValueError('duplicate episode identity would duplicate training samples')
    output.mkdir(parents=True, exist_ok=False)
    exported, hashes = [], {}
    for index, (directory, first) in enumerate(zip(directories, expected, strict=True)):
        metadata, context, observations, actions = inspect_episode(directory, split)
        if metadata != first:
            raise ValueError('source episode changed during dataset export')
        folder = output/'episodes'/f'{index:04d}'
        folder.mkdir(parents=True)
        context_path, data_path = folder/'context.json', folder/'transitions.jsonl'
        write_json(context_path, context.model_dump(mode='json'))
        with data_path.open('w', encoding='utf-8', newline='\n') as stream:
            for sequence, (observation, next_observation) in enumerate(zip(observations, observations[1:])):
                last = sequence == len(observations)-2
                row = dict(schema_version=TRANSITION_SCHEMA, episode_id=metadata['episode_id'], sequence=sequence,
                    observation=observation.model_dump(mode='json'), action=actions[sequence].model_dump(mode='json'),
                    next_observation=next_observation.model_dump(mode='json'),
                    terminated=last and metadata['outcome'] in TERMINATED,
                    truncated=last and metadata['outcome'] in TRUNCATED)
                stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n')
        paths = {name:str(path.relative_to(output).as_posix()) for name,path in
                 (('context_path', context_path), ('transitions_path', data_path))}
        exported.append(dict(metadata, **paths))
        for path in (context_path, data_path):
            hashes[path.relative_to(output).as_posix()] = sha256(path)
    manifest = dict(schema_version=SCHEMA, split=split,
        purpose='training' if split == 'train' else 'validation' if split == 'validation' else 'held_out_evaluation',
        feature_policy='only typed public method context, observations and requested actions; no private truth or invented rewards',
        incomplete_action_policy='unpaired requests are counted but never represented as executed transitions',
        episode_count=len(exported), transition_count=sum(item['transitions'] for item in exported),
        outcomes=dict(sorted(Counter(item['outcome'] for item in exported).items())),
        episodes=exported, files_sha256=hashes)
    write_json(output/'manifest.json', manifest)
    return manifest


def verify_dataset(directory):
    """Validate hashes, public schemas, temporal alignment and terminal labels."""
    directory = evidence_path(directory)
    manifest = read_json(directory/'manifest.json')
    if manifest.get('schema_version') != SCHEMA or manifest.get('split') not in {'train', 'validation', 'test'}:
        raise ValueError('unknown policy dataset schema or split')
    expected_purpose = {'train':'training', 'validation':'validation', 'test':'held_out_evaluation'}[manifest['split']]
    if manifest.get('purpose') != expected_purpose:
        raise ValueError('dataset purpose contradicts its frozen split')
    expected_paths, seen, transitions, outcomes = set(), set(), 0, Counter()
    for episode in manifest['episodes']:
        if (set(episode) != EPISODE_FIELDS or type(episode['unpaired_action_requests']) is not int
                or episode['unpaired_action_requests'] not in (0, 1)
                or episode['unpaired_action_requests'] and episode['outcome'] not in TRUNCATED-{'time_limit'}):
            raise ValueError('dataset episode fields or incomplete request count changed')
        if episode['episode_id'] in seen or episode['split'] != manifest['split']:
            raise ValueError('duplicate episode or mixed dataset splits')
        seen.add(episode['episode_id'])
        paths = []
        for name in ('context_path', 'transitions_path'):
            relative = episode[name]
            path = (directory/relative).resolve()
            if not path.is_relative_to(directory) or relative in expected_paths:
                raise ValueError('dataset artifact escapes its directory or is reused')
            expected_paths.add(relative)
            if manifest['files_sha256'].get(relative) != sha256(path):
                raise ValueError('dataset artifact hash changed')
            paths.append(path)
        context_raw, rows = read_json(paths[0]), read_rows(paths[1])
        task = parse_task(context_raw['task'])
        if (task.fingerprint() != episode['task_fingerprint'] or task.configuration_id != episode['configuration_id']
                or task.action_type != episode['action_type'] or task.seeds.split_for(episode['seed']) != manifest['split']):
            raise ValueError('dataset task identity or seed split changed')
        if not rows:
            raise ValueError('dataset episode contains no complete transition')
        observations = [row['observation'] for row in rows]+[rows[-1]['next_observation']]
        actions = [row['action'] for row in rows]
        validated_inputs(task, context_raw, observations, actions, episode['episode_id'])
        for index, row in enumerate(rows):
            fields = {'schema_version', 'episode_id', 'sequence', 'observation', 'action', 'next_observation', 'terminated', 'truncated'}
            if (set(row) != fields or row['schema_version'] != TRANSITION_SCHEMA
                    or row['episode_id'] != episode['episode_id'] or type(row['sequence']) is not int or row['sequence'] != index
                    or row['next_observation'] != observations[index+1]
                    or type(row['terminated']) is not bool or type(row['truncated']) is not bool
                    or row['terminated'] != (index == len(rows)-1 and episode['outcome'] in TERMINATED)
                    or row['truncated'] != (index == len(rows)-1 and episode['outcome'] in TRUNCATED)):
                raise ValueError('dataset transition continuity, public fields or terminal label changed')
        if episode['outcome'] not in TERMINATED | TRUNCATED or episode['transitions'] != len(rows):
            raise ValueError('dataset episode outcome or length changed')
        outcomes[episode['outcome']] += 1
        transitions += len(rows)
    if (set(manifest['files_sha256']) != expected_paths or manifest['episode_count'] != len(seen)
            or manifest['transition_count'] != transitions or manifest['outcomes'] != dict(sorted(outcomes.items()))):
        raise ValueError('dataset manifest does not match its episodes')
    return dict(schema_version=SCHEMA, verified=True, split=manifest['split'],
                episodes=len(seen), transitions=transitions, outcomes=dict(outcomes))


def iter_transitions(directory):
    """Model-independent consumer; call after acquiring exclusive archive access."""
    directory = evidence_path(directory)
    verify_dataset(directory)
    manifest = read_json(directory/'manifest.json')
    for episode in manifest['episodes']:
        context = read_json(directory/episode['context_path'])
        for row in read_rows(directory/episode['transitions_path']):
            yield dict(context=context, transition=row, episode=episode)
