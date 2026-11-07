"""Split leakage, private-feature leakage, stale actions and archive integrity."""
import json
import shutil

import pytest

from space_arm_platform import benchmark_dataset as data
from space_arm_platform.benchmark_contract import SPEC_DIRECTORY, ForceAction, MethodContext, NavigationObservation, load_task
from space_arm_platform.benchmark_execution import sha256, write_json


@pytest.fixture
def archive_factory(tmp_path, monkeypatch):
    # Physical grading has separate adversarial tests and real native acceptance.
    # Here only the replay boundary is substituted; archive hashes remain real.
    monkeypatch.setattr(data, 'rescore_episode', lambda path: dict(matches_recorded_physics_score=True,
        outcome=json.loads((path/'result.json').read_text())['outcome']))
    def create(name, *, seed=1, outcome='success'):
        path=tmp_path/name
        path.mkdir()
        task=load_task(SPEC_DIRECTORY/'d32-cooperative-gnss-v1.json')
        split=task.seeds.split_for(seed)
        context=MethodContext(task=task,chaser_mass_kg=100.)
        write_json(path/'task.json',task.model_dump(mode='json'))
        write_json(path/'context.json',context.model_dump(mode='json'))
        write_json(path/'manifest.json',dict(episode_id=name,configuration_id=task.configuration_id,
            task_fingerprint=task.fingerprint(),seed=seed,split=split,method='gnss-pd-v1',physics_path='physics.jsonl'))
        observations=[NavigationObservation(episode_id=name,sequence=k,sample_time_ns=k*500_000_000,
            delivery_time_ns=k*500_000_000,target_estimate_j2000=(7e6,0.,0.,0.,7500.,0.),
            chaser_estimate_j2000=(7e6,-160.,0.,0.,7500.,0.)).model_dump(mode='json') for k in range(3)]
        actions=[ForceAction(episode_id=name,observation_sequence=k,force_j2000_n=(0.,1.,0.)).model_dump(mode='json') for k in range(2)]
        rows(path/'observations.jsonl',observations)
        rows(path/'actions.jsonl',actions)
        (path/'physics.jsonl').write_text('PRIVATE_TRUTH_SHOULD_NEVER_BE_A_POLICY_FEATURE\n')
        result=dict(configuration_id=task.configuration_id,seed=seed,split=split,method='gnss-pd-v1',outcome=outcome,
                    evidence_sha256={p.name:sha256(p) for p in path.iterdir()})
        write_json(path/'result.json',result)
        return path
    return create


def rows(path, values):
    path.write_text(''.join(json.dumps(value)+'\n' for value in values),encoding='utf-8')


def rehash_source(path, name):
    result=data.read_json(path/'result.json')
    result['evidence_sha256'][name]=sha256(path/name)
    write_json(path/'result.json',result)


def rewrite_dataset_row(output, change):
    manifest=data.read_json(output/'manifest.json')
    path=output/manifest['episodes'][0]['transitions_path']
    values=data.read_rows(path)
    change(values)
    rows(path,values)
    manifest['files_sha256'][path.relative_to(output).as_posix()]=sha256(path)
    write_json(output/'manifest.json',manifest)


def test_portable_samples_keep_failed_episodes_and_no_private_truth(archive_factory,tmp_path):
    good,bad=archive_factory('good'),archive_factory('bad',outcome='time_limit')
    out=tmp_path/'dataset'
    manifest=data.export_dataset([good,bad],out)
    assert manifest['outcomes']=={'success':1,'time_limit':1}
    assert manifest['transition_count']==4 and manifest['purpose']=='training'
    moved=tmp_path/'portable'
    shutil.copytree(out,moved)
    assert data.verify_dataset(moved)['verified']
    samples=list(data.iter_transitions(moved))
    assert len(samples)==4 and samples[0]['context']['chaser_mass_kg']==100.
    assert samples[1]['transition']['terminated'] and samples[-1]['transition']['truncated']
    for path in moved.rglob('*'):
        if path.is_file():
            assert 'PRIVATE_TRUTH' not in path.read_text('utf-8')
    assert not any(path.name=='physics.jsonl' for path in moved.rglob('*'))


def test_test_seeds_cannot_be_exported_as_training(archive_factory,tmp_path):
    source=archive_factory('test',seed=201)
    out=tmp_path/'dataset'
    with pytest.raises(ValueError,match='relabeling'):
        data.export_dataset([source],out)
    assert not out.exists()
    manifest=data.export_dataset([source],out,split='test')
    assert manifest['purpose']=='held_out_evaluation'
    manifest['purpose']='training'
    write_json(out/'manifest.json',manifest)
    with pytest.raises(ValueError,match='purpose'):
        data.verify_dataset(out)


@pytest.mark.parametrize('defect',['truth_observation','truth_context','episode','sequence','time'])
def test_rehashed_source_still_obeys_public_input_and_time_contract(archive_factory,tmp_path,defect):
    source=archive_factory('source')
    name='context.json' if defect=='truth_context' else 'observations.jsonl'
    if defect=='truth_context':
        context=data.read_json(source/name);context['private_truth']=[1,2,3]
        write_json(source/name,context)
    else:
        observations=data.read_rows(source/name)
        key,value={'truth_observation':('private_truth',[1,2,3]),'episode':('episode_id','other'),
                   'sequence':('sequence',0),'time':('sample_time_ns',499_000_000)}[defect]
        observations[1][key]=value
        rows(source/name,observations)
    rehash_source(source,name)
    with pytest.raises(ValueError):
        data.export_dataset([source],tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_stale_action_cannot_be_paired_with_a_different_observation(archive_factory,tmp_path):
    source=archive_factory('source')
    actions=data.read_rows(source/'actions.jsonl');actions[0]['observation_sequence']=1
    rows(source/'actions.jsonl',actions);rehash_source(source,'actions.jsonl')
    with pytest.raises(ValueError,match='stale action'):
        data.export_dataset([source],tmp_path/'out')


def test_unpaired_request_is_counted_without_inventing_execution(archive_factory,tmp_path):
    source=archive_factory('failed',outcome='infrastructure_error')
    actions=data.read_rows(source/'actions.jsonl')
    actions.append(dict(actions[-1],observation_sequence=2))
    rows(source/'actions.jsonl',actions);rehash_source(source,'actions.jsonl')
    manifest=data.export_dataset([source],tmp_path/'out')
    assert manifest['episodes'][0]['unpaired_action_requests']==1
    assert data.verify_dataset(tmp_path/'out')['transitions']==2
    result=data.read_json(source/'result.json');result['outcome']='success';write_json(source/'result.json',result)
    with pytest.raises(ValueError,match='unpaired action'):
        data.export_dataset([source],tmp_path/'invalid')


def test_missing_legacy_context_is_not_reconstructed(archive_factory,tmp_path):
    source=archive_factory('source')
    result=data.read_json(source/'result.json');result['evidence_sha256'].pop('context.json')
    write_json(source/'result.json',result)
    with pytest.raises(ValueError,match='legacy contexts'):
        data.export_dataset([source],tmp_path/'out')


def test_modified_private_evidence_still_invalidates_export(archive_factory,tmp_path):
    source=archive_factory('source')
    (source/'physics.jsonl').write_text('changed private evidence')
    with pytest.raises(ValueError,match='changed or invalid evidence'):
        data.export_dataset([source],tmp_path/'out')


@pytest.mark.parametrize('defect',['truth','continuity','terminal','bool_sequence'])
def test_dataset_verifier_rechecks_semantics_after_rehash(archive_factory,tmp_path,defect):
    out=tmp_path/'out'
    data.export_dataset([archive_factory('source')],out)
    def change(values):
        if defect=='truth':values[0]['truth']=[1,2,3]
        if defect=='continuity':values[0]['next_observation']['chaser_estimate_j2000'][0]+=1
        if defect=='terminal':values[0]['terminated']=True
        if defect=='bool_sequence':values[0]['sequence']=False
    rewrite_dataset_row(out,change)
    with pytest.raises(ValueError,match='transition'):
        data.verify_dataset(out)


def test_duplicate_episode_or_output_inside_source_is_rejected(archive_factory,tmp_path):
    source=archive_factory('source')
    with pytest.raises(ValueError,match='unique'):
        data.export_dataset([source,source],tmp_path/'out')
    with pytest.raises(ValueError,match='separate'):
        data.export_dataset([source],source/'dataset')
