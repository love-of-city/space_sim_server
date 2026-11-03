from pathlib import Path
import numpy as np
import pytest
from simulation.policy_task_success import DrawerCycleSuccess, EVALUATION_PERIOD_NS
from simulation.evaluation.scoring import PhysicalScorer

MODEL=Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'
ROTATION=[[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]

def monitor():
    return DrawerCycleSuccess(MODEL,sample_period_ns=EVALUATION_PERIOD_NS,mode='benchmark')

def scorer():
    m=monitor()
    return PhysicalScorer('drawer_cycle',[0,0,0],drawer_spec=m.specification())


def test_drawer_independent_score_also_enforces_40N_4Nm():
    s=scorer()
    for i,sample in enumerate(cycle()):
        sample['wrench']=[45.,0.,0.,0.,0.,0.]
        s.update(sample)
        if i>=30:break
    assert s.safety_failure=='wrist_overload'

def cycle(period_ns=EVALUATION_PERIOD_NS,open_s=2.2):
    m=monitor();end_open=1.1+open_s;end_seat=end_open+2.2;end_release=end_seat+.1;end=end_release+2.2
    for stamp in range(0,round(end*1e9)+1,period_ns):
        t=stamp*1e-9;opened=1.1<t<=end_open;released=t>end_seat;hover=t>end_release
        drawer=.08 if opened else 0.;opening=1. if released else .05
        position=m.grip_origin+m.axis*(m.config.clearance_m if hover else drawer)
        yield dict(time_s=t,drawer_m=drawer,drawer_velocity_m_s=0.,buckle_rad=0.,
                   tool_position_m=position.tolist(),tool_rotation=ROTATION,joint_velocity=[0.]*8,
                   joint_position=[0.]*6+[opening,-opening],contacts=[0,0] if released else [2,2],
                   wrench=[0.]*6,wrench_valid=True,reference_position_m=position.tolist())

def policy_sample(row):
    return dict(sim_time_ns=round(row['time_s']*1e9),drawer_position_m=row['drawer_m'],
                drawer_velocity_m_s=row['drawer_velocity_m_s'],buckle_rad=row['buckle_rad'],
                end_effector_position_body_m=row['tool_position_m'],end_effector_rotation_body=row['tool_rotation'],
                arm_joint_velocity_rad_s=row['joint_velocity'][:6],finger_drawer_contacts=row['contacts'],
                gripper_drive_position_rad=row['joint_position'][6])

def test_full_detailed_cycle_passes_and_records_dwell():
    s=scorer()
    for row in cycle():s.update(row)
    r=s.report()
    assert r['success'] and r['progress']==1
    assert r['grasp_verified'] and r['max_open_hold_s']>=2 and r['max_seat_hold_s']>=2
    assert r['schema']=='space-arm-physical-score/2'

def test_short_opening_cannot_be_forged_by_script_success_flags():
    s=scorer()
    for row in cycle(open_s=.1):s.update(dict(row,stage='success',result='success',open_verified=True))
    assert not s.report()['success'] and not s.report()['milestones']['opened_or_lifted']

@pytest.mark.parametrize('field',['tool_rotation','joint_velocity'])
def test_optional_diagnostics_do_not_define_task_success(field):
    s=scorer()
    for row in cycle():
        del row[field];s.update(row)
    assert s.report()['success']

@pytest.mark.parametrize('field',['drawer_m','drawer_velocity_m_s','contacts','wrench_valid'])
def test_missing_task_evidence_is_unscorable(field):
    s=scorer();row=next(cycle());del row[field];s.update(row)
    assert s.report()['status']=='unscorable'

def test_old_drawer_spec_never_falls_back_to_instantaneous_rule():
    with pytest.raises(ValueError,match='drawer_spec'):
        PhysicalScorer('drawer_cycle',[0,0,.2])

def test_200hz_evaluation_agrees_with_50hz_policy_at_shared_timestamps():
    s=scorer();p=DrawerCycleSuccess(MODEL,mode='benchmark')
    for row in cycle():
        s.update(row);stamp=round(row['time_s']*1e9)
        if stamp==0:p.reset(policy_sample(row))
        elif stamp%20_000_000==0:p.update(policy_sample(row))
        else:continue
        assert s.report()['success']==p.status()['success']
        assert s.report()['milestones']['opened_or_lifted']==p.status()['open_verified']

def test_interruption_resets_open_hold():
    s=scorer()
    for row in cycle():
        if round(row['time_s']*1e9)==2_000_000_000:row['contacts']=[0,0]
        s.update(row)
    assert not s.report()['success'] and s.report()['max_open_hold_s']<2

def test_benchmark_does_not_require_expert_grasp_pose_or_exact_hover():
    s=scorer();m=monitor()
    for row in cycle():
        row['tool_rotation']=np.eye(3).tolist()
        row['joint_velocity']=[.02]*6+[0.,0.]
        if row['time_s']>5.6:
            row['tool_position_m']=(m.grip_origin+np.array([.12,0,0])).tolist()
        s.update(row)
    assert s.report()['success']

def test_benchmark_rejects_hand_still_near_closed_handle():
    s=scorer();m=monitor()
    for row in cycle():
        if row['time_s']>5.6:row['tool_position_m']=(m.grip_origin+np.array([.04,0,0])).tolist()
        s.update(row)
    assert not s.report()['success']

def test_expert_controller_configuration_cannot_weaken_benchmark():
    from dataclasses import replace
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
    expert=replace(DrawerCycleConfig(),hold_s=.01,verify_hold_s=.01)
    m=DrawerCycleSuccess(MODEL,expert,sample_period_ns=EVALUATION_PERIOD_NS,mode='benchmark')
    assert m.config.hold_s==2 and m.config.verify_hold_s==2
    spec=m.specification();spec['config']['hold_s']=.01
    with pytest.raises(ValueError,match='benchmark'):DrawerCycleSuccess.from_spec(spec)

def test_physical_completion_cannot_override_sustained_overload():
    s=scorer()
    for row in cycle():
        if 1.5 <= row['time_s'] <= 1.7:row['wrench']=[81.,0.,0.,0.,0.,0.]
        s.update(row)
    assert s.drawer_state['success']
    assert s.report()['status']=='task_failure' and s.report()['reason']=='wrist_overload'

@pytest.mark.parametrize('field,value',[
    ('tool_rotation',[[1.,0.,0.],[0.,1.,0.],[0.,0.,2.]]),
    ('joint_velocity',[0.]*7),
])
def test_supplied_malformed_diagnostics_are_not_ignored(field,value):
    s=scorer()
    for row in cycle():
        row[field]=value;s.update(row)
    assert s.report()['status']=='unscorable'

def test_benchmark_retreat_threshold_cannot_be_weakened():
    spec=monitor().specification();spec['retreat_distance_m']=.01
    with pytest.raises(ValueError,match='benchmark'):DrawerCycleSuccess.from_spec(spec)

@pytest.mark.parametrize('layout',['native','evaluation-v1'])
def test_raw_trace_reassessment_preserves_original_and_uses_new_hold(tmp_path,layout):
    import json
    from dataclasses import asdict
    from types import SimpleNamespace
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
    from simulation.evaluation.fixed_trajectory import sha256
    from tools.evaluate_fixed_trajectories import rescore_drawer_trace,score_recording
    source=tmp_path/'source';source.mkdir()
    report=source/'summary.json'
    report.write_text(json.dumps(dict(task_name='drawer_cycle',result='success',model_sha256=sha256(MODEL),config=asdict(DrawerCycleConfig()))))
    trace=source/'trace.jsonl'
    with trace.open('w') as stream:
        for row in cycle(open_s=.1):
            raw=dict(sim_time_ns=str(round(row['time_s']*1e9)),joint_position=row['joint_position'],
                     joint_velocity=row['joint_velocity'],tool_position_body_m=row['tool_position_m'],
                     tool_rotation_body=row['tool_rotation'],drawer_m=row['drawer_m'],drawer_velocity_m_s=0.,buckle_rad=0.,
                     finger_drawer_contacts=row['contacts'],wrist_wrench=dict(force_n=row['wrench'][:3],torque_nm=row['wrench'][3:],valid=True),
                     result='success',open_verified=True)
            if layout=='evaluation-v1':
                raw={key:value for key,value in row.items() if key not in ('tool_rotation','joint_velocity')}
            stream.write(json.dumps(raw)+'\n')
    original=(report.read_bytes(),sha256(trace))
    output=tmp_path/'revised'
    assert rescore_drawer_trace(SimpleNamespace(trace=trace,model=MODEL,output=output))==0
    score=json.loads((output/'score.json').read_text())
    assert score['status']=='task_failure' and score['max_open_hold_s']<2
    assert score_recording(output)==score
    assert (report.read_bytes(),sha256(trace))==original

@pytest.mark.parametrize('tail',['not json','[]',''])
def test_reassessment_ingestion_errors_remain_unscorable_offline(tmp_path,tail):
    import json
    from types import SimpleNamespace
    from simulation.evaluation.fixed_trajectory import sha256
    from tools.evaluate_fixed_trajectories import rescore_drawer_trace,score_recording
    source=tmp_path/'source';source.mkdir()
    (source/'summary.json').write_text(json.dumps(dict(task_name='drawer_cycle',model_sha256=sha256(MODEL))))
    trace=source/'trace.jsonl'
    # An otherwise successful prefix must not mask a malformed observation.
    trace.write_text(''.join(json.dumps(row)+'\n' for row in cycle())+tail+'\n' if tail else '')
    output=tmp_path/'revised'
    assert rescore_drawer_trace(SimpleNamespace(trace=trace,model=MODEL,output=output))==2
    score=json.loads((output/'score.json').read_text())
    assert score['status']=='unscorable'
    assert score_recording(output)==score
