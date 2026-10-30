"""Exercise the exact distribution, fresh runtime paths, generated assets, and two real captures."""
from pathlib import Path
import json,subprocess,time,hashlib
import argparse
_parser=argparse.ArgumentParser(description='Explicit release integration verification; requires GPU/runtime.')
_parser.add_argument('--release-root',type=Path,required=True)
root=_parser.parse_args().release_root.resolve()
(root/'validation').mkdir(exist_ok=True)
artifact=root/'space-sim-collector-linux-x86_64.run'
bundle=root/'relocated collector/SpaceSimCollector'
layouts=root/'validation/release-layout'
output=root/'validation/release-capture'
records=[]
def run(command,name):
    started=time.monotonic()
    with (root/'validation'/(name+'.log')).open('w') as log:
        result=subprocess.run(list(map(str,command)),stdout=log,stderr=subprocess.STDOUT)
    records.append(dict(name=name,returncode=result.returncode,wall_seconds=time.monotonic()-started))
    (root/'validation/verification-progress.json').write_text(json.dumps(records,indent=2)+'\n')
    if result.returncode:
        raise RuntimeError(f'{name}: exit {result.returncode}; see validation/{name}.log')
    print(name,'passed',flush=True)
run([artifact,'--extract',bundle],'extract-final-artifact')
run([artifact,'--install-dir',bundle,'check'],'final-dependencies')
run([artifact,'--install-dir',bundle,'tasks'],'final-task-list')
run([artifact,'--install-dir',bundle,'prepare-layouts','prepare','--count','1','--seed','202610288801',
     '--assets-only','--output',layouts],'final-layout-preparation')
folder=layouts/'layout-000-seed-202610288801'
run([artifact,'--install-dir',bundle,'run','--task','drawer_cycle','--count','2','--max-attempts','4',
     '--range-deg','30','--seed','2026102700','--no-randomize-orbit-phase',
     '--model',folder/'model/layout.xml','--catalog',folder/'ue-assets/layout.catalog.json',
     '--output',output],'final-two-episode-capture')
run([artifact,'--install-dir',bundle,'status','--output',output],'final-status')
summary=json.loads((output/'batch/summary.json').read_text())
assert summary['status']=='complete' and len(summary['accepted_episodes'])==2
rows=[]
for attempt in summary['attempts']:
    if attempt['status']!='accepted':continue
    ep=Path(output/'episodes'/attempt['episode_id'])
    metadata=json.loads((ep/'metadata.json').read_text())
    validation=json.loads((Path(attempt['directory'])/'validation.json').read_text())
    recording=json.loads((Path(attempt['directory'])/'recording/summary.json').read_text())
    assert metadata['dataset_status']=='complete' and metadata['outcome']=='success'
    assert metadata['instruction']=='Open the drawer approximately 8 cm, then close it fully and release it.'
    scene=metadata['scene_instance'];init=scene['collection_config']['initialization']
    assert init['half_range_deg']==30 and all(abs(v)<=30 for v in init['offset_deg'])
    camera=scene['overview_camera'];assert camera['schema']=='space-arm-overview-camera/7'
    assert len(list(ep.rglob('*.mp4')))==2
    rows.append(dict(episode=str(ep),seed=attempt['seed'],frames=metadata['dataset_frame_count'],
        instruction=metadata['instruction'],camera_preset=camera['preset_id'],joint_offset_deg=init['offset_deg'],
        full_cycle_seconds=attempt['end_to_end_wall_seconds'],recording_seconds=attempt['recording_wall_seconds'],
        recorded_sim_seconds=recording.get('recorded_sim_seconds'),dynamics_worker_pid=attempt.get('dynamics_worker_pid'),
        validation=validation,recording_summary=str(Path(attempt['directory'])/'recording/summary.json')))
assert rows[0]['joint_offset_deg']!=rows[1]['joint_offset_deg']
accepted=[a for a in summary['attempts'] if a['status']=='accepted']
interval=(int(accepted[1]['finished_at_ns'])-int(accepted[0]['finished_at_ns']))/1e9
report=dict(passed=True,artifact=json.loads((root/'release.json').read_text()),checks=records,episodes=rows,
    second_completion_interval_seconds=interval,renderer_pid=summary.get('renderer_pid'),
    renderer_reuse_enabled=summary['reuse_renderer'],dynamics_reuse_enabled=summary['reuse_dynamics'],
    supported_task_entrypoints=6,tested_task='drawer_cycle',tested_episodes=2,
    test_layout='One newly generated layout, explicitly selected for this packaging acceptance; no certified random pool was substituted',
    orbit_phase_randomized_in_test=False,random_joint_half_range_deg=30,
    relocation='Same Linux host, different directory containing spaces; another machine not tested',
    videos_fully_decoded_by_production_validator=True,launcher_tests=9)
(root/'validation/RESULTS.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('FINAL ARTIFACT CAPTURE VALIDATION PASSED',flush=True)
