"""Independent acceptance of recorded capture12 state and 400 Hz force logs."""
import json
from pathlib import Path
import numpy as np

HANDOFF_VERSION = 'capture-target-grasp/5-preferred-handoff-branch'
HANDOFF_VERSIONS = ('capture-target-grasp/4-physical-handoff', HANDOFF_VERSION)


def validate_handoff(final, stages):
    assert final['stage'] == 'handoff_hold' and final['grasp_hold_s'] >= 20.
    assert 'pull' in stages and 'handoff_hold' in stages
    assert final['carry_plan']['accepted']
    assert final['handoff']['target_position_error_m'] <= .005
    assert final['handoff']['target_angle_error_rad'] <= np.deg2rad(1.)


def validate_task_rows(steps,stages):
    final=steps[-1]['scripted_task']
    assert final['task_version'] in ('capture-target-grasp/2', 'capture-target-grasp/3-relative-motion', *HANDOFF_VERSIONS)
    assert final['result']=='success'
    if final['task_version'] in HANDOFF_VERSIONS:
        validate_handoff(final, stages)
    else:
        assert final['stage']=='hold'
    assert all(stage in stages for stage in ('settle','align','approach','close','tighten','hold'))
    assert final['stable_hold_s']>=20.
    assert not final['self_contact'] and final['peak_contact_force_n']<=10.
    end=int(steps[-1]['sim_time_ns'])
    stable=[obs['scripted_task']['feedback'] for obs in steps if 0.<=(end-int(obs['sim_time_ns']))*1e-9<19.98]
    assert len(stable)>=998
    assert all(row['jaw_support_count']==4 and 4.5<=row['jaw_force_n']<=5.5 for row in stable)
    assert all(obs['gripper_type']=='capture26_four_jaw' and len(obs['joint_position_rad'])==12 for obs in steps)


def validate_physical_evidence(meta):
    directory=Path(meta['scene_instance']['collection_config']['physical_evidence_directory'])
    report=json.loads((directory/'summary.json').read_text())
    assert report['result']=='success' and report['preflight']['accepted']
    rows=[json.loads(line) for line in (directory/'contacts_400hz_force.jsonl').read_text().splitlines()]
    end=rows[-1]['time_s'];stable=[r for r in rows if 0.<=end-r['time_s']<19.995]
    assert len(stable)>=7997
    assert all(r['feedback']['jaw_support_count']==4 and 4.5<=r['feedback']['jaw_force_n']<=5.5 for r in stable)
    assert all(r['self_contact_count']==0 and r['feedback']['peak_target_contact_force_n']<=10. for r in rows)
    assert all(abs((b['time_s']-a['time_s'])-.0025)<1e-7 for a,b in zip(rows,rows[1:]))
    if report['metrics']['task_version'] in HANDOFF_VERSIONS:
        from simulation.scripted_tasks.capture_carry import handoff_error
        trace = [json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()]
        validate_handoff(report['metrics'], {row['stage'] for row in trace})
        final_stamp = int(trace[-1]['sim_time_ns'])
        final_hold = [row for row in trace if 0. <= (final_stamp-int(row['sim_time_ns']))*1e-9 < 19.98]
        assert len(final_hold) >= 998
        for row in final_hold:
            position, angle = handoff_error(dict(
                interface_position=np.asarray(row['interface_position_body_m']),
                interface_rotation=np.asarray(row['interface_rotation_body'])))
            assert position <= .005 and angle <= np.deg2rad(1.)
        carrying = [row for row in rows if row['phase'] in ('pull','handoff_hold')]
        assert carrying and all(row['feedback']['jaw_support_count']==4 for row in carrying)
