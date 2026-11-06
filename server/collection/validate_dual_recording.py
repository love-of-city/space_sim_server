"""Acceptance checks for sequential two-arm retention and the native dataset."""
from itertools import groupby
import hashlib
import json
import numpy as np
from space_arm_platform.compact_storage import read_observations,read_wrench,wrench_bytes,is_compact
from simulation.scripted_tasks.dual_floating_grasps import selection_digest
from simulation.scripted_tasks.dual_floating_model import DUAL_TASK


def validate(episode):
    import pyarrow.parquet as pq
    from collection.collect_scripted_tasks import validate_floating_recording,validate_capture_manifest,validate_videos
    from collection.record_scripted_task import task_instruction
    meta=json.loads((episode/'metadata.json').read_text())
    info=json.loads((episode/'lerobot/meta/info.json').read_text())
    platform=json.loads((episode/'lerobot/meta/platform.json').read_text())
    scene=meta['scene_instance'];record=scene['collection_config']['dual_floating']
    assert meta['outcome']=='success' and meta['dataset_status']=='complete'
    assert platform['schema']=='space-arm-dual-lerobot/1' and platform['arm_order']==['left','right']
    assert record['preflight']['accepted'] and record['selection_sha256']==selection_digest(scene)
    assert meta['instruction']==task_instruction(scene,DUAL_TASK,None)
    names=[]
    for side in ('left','right'):names.extend([f'{side}_joint_{i+1}_rad' for i in range(6)]+[side+'_gripper_opening_normalized'])
    for key in ('observation.state','action'):
        assert info['features'][key]['shape']==[14] and info['features'][key]['names']==names
    steps=list(read_observations(episode));rows=[]
    for path in sorted((episode/'lerobot/data').rglob('*.parquet')):rows.extend(pq.read_table(path).to_pylist())
    assert len(rows)==len(steps)==info['total_frames']==meta['dataset_frame_count']
    assert meta['dataset_fps']==50 and np.all(np.diff([int(o['sim_time_ns']) for o in steps])==20_000_000)
    assert np.all(np.diff([int(o['render_frame_id']) for o in steps])==1)
    assert all(o['model_sha256']==record['model_sha256'] for o in steps)
    assert steps[-1]['scripted_task']['result']=='success'
    assert [stage for stage,_ in groupby(o['scripted_task']['stage'] for o in steps)]==['left','right']
    right_rows=[o for o in steps if o['scripted_task']['stage']=='right']
    assert right_rows and right_rows[0]['scripted_task']['arms']['left']['settled_seconds']>=1.
    assert all(o['scripted_task']['arms']['left']['stage']=='settle' for o in right_rows)
    assert all(o['scripted_task']['arms']['right']['stage']=='pending' for o in steps if o['scripted_task']['stage']=='left')
    assert steps[-1]['scripted_task']['right_recheck']['accepted']
    for side,index in [('left',0),('right',1)]:
        target=next(i for i in record['instances'] if i['arm']==side)
        projected=[dict(o['arms'][side],sim_time_ns=o['sim_time_ns'],scripted_task=o['scripted_task']['arms'][side])
                   for o in steps if o['scripted_task']['arms'][side]['stage']!='pending']
        stages=[stage for stage,_ in groupby(o['scripted_task']['stage'] for o in projected)]
        validate_floating_recording(projected,stages,target['type_task'],target_body=target['instance_id'],require_result=side=='right')
        selected=(record['arms'][side]['grasp']['candidate']['id'] if side=='left'
                  else steps[-1]['scripted_task']['right_recheck']['selected']['id'])
        assert all(o['scripted_task']['grasp_candidate_id']==selected for o in projected)
        for row,obs in zip(rows,steps):
            arm=obs['arms'][side];prefix='observation.'+side+'.'
            np.testing.assert_allclose(row['observation.state'][index*7:(index+1)*7],arm['joint_position_rad'][:6]+[arm['gripper_opening_normalized']],atol=1e-7)
            np.testing.assert_allclose(row['action'][index*7:(index+1)*7],arm['target_joint_position_rad'][:6]+[arm['target_gripper_opening_normalized']],atol=1e-7)
            np.testing.assert_allclose(row['observation.raw_joint_position'][index*8:(index+1)*8],arm['joint_position_rad'],atol=1e-7)
            np.testing.assert_allclose(row['observation.raw_target_joint_position'][index*8:(index+1)*8],arm['target_joint_position_rad'],atol=1e-7)
            np.testing.assert_allclose(row[prefix+'wrist_wrench'],arm['wrist_wrench']['force_n']+arm['wrist_wrench']['torque_nm'],atol=1e-5)
            assert arm['wrist_wrench']['valid']
        high=platform['high_frequency_wrist_wrench'][side];forces=list(read_wrench(episode,side))
        assert high['rate_hz']==200 and len(forces)==high['samples']==4*(len(steps)-1)+1
        stamps=[int(r['sim_time_ns']) for r in forces]
        assert stamps[0]==int(steps[0]['sim_time_ns']) and stamps[-1]==int(steps[-1]['sim_time_ns'])
        assert np.all(np.diff(stamps)==5_000_000)
        if is_compact(episode):
            digest=hashlib.sha256()
            for sample in forces:digest.update(wrench_bytes(sample))
            assert digest.hexdigest()==high['values_sha256']
        by_time={int(r['sim_time_ns']):r for r in forces}
        for obs in steps:
            sample=by_time[int(obs['sim_time_ns'])];reading=obs['arms'][side]['wrist_wrench']
            for field in ('force_n','torque_nm'):np.testing.assert_allclose(sample[field],reading[field],rtol=1e-6,atol=1e-5)
            for field in ('sample_time_ns','interval_end_time_ns'):assert int(sample[field])==int(reading[field])
    count=validate_capture_manifest(episode,meta,steps)
    validate_videos(episode,meta,info,len(steps))
    return dict(passed=True,episode_id=episode.name,frames=len(steps),images=count,state_action_dimensions=14,
        fps=50,stages=['left','right'],final_task_state=steps[-1]['scripted_task'],videos_fully_decoded=True,
        source_validation='numeric_sha256_and_camera_pairing' if is_compact(episode) else 'full_observation_comparison')
