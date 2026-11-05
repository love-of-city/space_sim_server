import hashlib
import json
from pathlib import Path
import pytest
from space_arm_platform.recorded_policy_scene import restore_scene, copy_recorded_model


def fixture(root):
    episodes=root/'episodes'; episode='episode-test'
    recording=root/'collections'/'batch'/'attempts'/'cell'/'0000'/'attempt-1'/'recording'
    model=recording/'dynamics'/'model.xml'; model.parent.mkdir(parents=True)
    model.write_text('<mujoco/>'); model.with_name('attitude_control.json').write_text('{}')
    digest=hashlib.sha256(model.read_bytes()).hexdigest()
    model.with_name('summary.json').write_text(json.dumps(dict(model_sha256=digest,render_model_sha256=digest)))
    (recording/'summary.json').write_text(json.dumps(dict(episode_id=episode)))
    (recording.parent.parent/'summary.json').write_text(json.dumps({'attempts':[dict(status='accepted',episode_id=episode,directory=str(recording.parent))]}))
    scene=dict(template_id='sarm-task-box-20261012',instance_id='original',collection_batch=dict(batch_id='batch',attempt_id='cell/0000'),collection_config=dict(task='floating_bolt_capture',floating_objects=dict(instances=[dict(instance_id='bolt',initial_state={'position_body_m':[1,2,3]})])),scripted_task_name='floating_bolt_capture')
    (episodes/episode).mkdir(parents=True)
    (episodes/episode/'metadata.json').write_text(json.dumps(dict(scene_instance=scene,dataset_status='complete',instruction='Catch the bolt.')))
    return episodes,episode,model


def test_restore_preserves_frozen_state_and_copies_exact_physics(tmp_path):
    root,episode,model=fixture(tmp_path)
    before=(root/episode/'metadata.json').read_bytes()
    scene=restore_scene(root,episode,{'username':'glx'})
    assert scene['collection_config']['floating_objects']['instances'][0]['initial_state']['position_body_m']==[1,2,3]
    assert scene['source_instance_id']=='original' and scene['instance_id']!='original'
    assert 'scripted_task_name' not in scene
    dest=tmp_path/'worker';dest.mkdir()
    copied=copy_recorded_model(scene,dest)
    assert copied.read_bytes()==model.read_bytes()
    assert (dest/'attitude_control.json').is_file()
    assert (root/episode/'metadata.json').read_bytes()==before


def test_rejects_corrupt_model_and_path_escape(tmp_path):
    root,episode,model=fixture(tmp_path)
    with pytest.raises(ValueError):restore_scene(root,'../episode-test',{})
    model.write_text('<changed/>')
    with pytest.raises(ValueError,match='digest'):restore_scene(root,episode,{})
