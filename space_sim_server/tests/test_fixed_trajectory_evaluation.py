"""Fixed-count evaluation must not trust the demonstration controller's flags."""
import copy
import numpy as np
import pytest
from simulation.evaluation.fixed_trajectory import FrozenTrajectory
from simulation.evaluation.scoring import PhysicalScorer, aggregate
from test_unified_drawer_scoring import scorer as drawer_scorer, cycle, ROTATION


def document():
    return dict(schema='space-arm-fixed-trajectory/1',task='drawer_cycle',frame='cubesat_bus',
                model_sha256='a'*64,initial_joints=[0.0]*8,
                samples=[dict(time_s=0.,joint_reference=[0.]*8,joint_reference_velocity=[0.]*8,position_m=[0.,0.,0.],rotation=np.eye(3).tolist()),
                         dict(time_s=1.,joint_reference=[.1]*6+[.7,-.7],joint_reference_velocity=[0.]*8,position_m=[.1,0.,0.],rotation=np.eye(3).tolist())])


def row(t, **kw):
    return dict(time_s=t,tool_position_m=[0.,0.,.2],joint_position=[0.]*6+[1.,-1.],
                wrench=[0.]*6,wrench_valid=True,contacts=[0,0],drawer_m=0.,
                tool_rotation=ROTATION,joint_velocity=[0.]*8,
                drawer_velocity_m_s=0.,buckle_rad=0.,**kw)


def test_frozen_reference_does_not_depend_on_observed_objects():
    d=document();t=FrozenTrajectory(d);d['samples'][0]['joint_reference'][0]=999
    assert t.reference(.5)[0][0]==0
    assert t.reference(1.0)[0][0]==.1
    assert t.reference(8.)[1]==pytest.approx([0.]*8)


@pytest.mark.parametrize('edit',[
    lambda d:d['samples'][1].update(time_s=0.),
    lambda d:d['samples'][0].update(joint_reference=[float('nan')]*8),
    lambda d:d.update(frame='world'),
    lambda d:d['samples'][0].update(rotation=np.zeros((3,3)).tolist()),
])
def test_trajectory_rejects_invalid_input(edit):
    d=copy.deepcopy(document());edit(d)
    with pytest.raises(ValueError):FrozenTrajectory(d)


def test_script_success_without_physical_opening_cannot_pass():
    s=drawer_scorer()
    for i in range(601):s.update(row(i*.005,result='success',open_verified=True,stage='verify'))
    assert not s.report()['success'];assert s.report()['progress']==0


def test_ordered_drawer_cycle_and_final_hold():
    s=drawer_scorer()
    rows=list(cycle())
    for sample in rows:s.update(sample)
    assert s.report()['success'];assert s.report()['progress']==1
    x=row(rows[-1]['time_s']+.005);x['drawer_m']=.02;s.update(x)
    assert s.report()['status']=='task_failure'



def test_missing_or_gapped_measurements_are_unscorable():
    s=drawer_scorer();s.update(row(0))
    s.update(row(.1))
    assert s.report()['status']=='unscorable'
    with pytest.raises(ValueError):PhysicalScorer('unknown',final_tool_position=[0,0,0])


def test_fixed_count_denominators_keep_failures_and_errors():
    a=aggregate([{'status':'success'},{'status':'task_failure'}, {'status':'infrastructure_error'}, {'status':'initialization_rejected'}, {'status':'unscorable'}])
    assert a['attempts']==5;assert a['success_rate_all_attempts']==.2
    assert a['success_rate_scorable']==.5
    assert len(a['wilson95_all_attempts'])==2


@pytest.mark.parametrize('task,tip,keyed',[
    ('aviation_plug_insert',.080,True),('guide_plug_insert',.095,True),('cylindrical_plug_insert',.095,False)])
def test_plug_requires_lift_insertion_release_and_stable_retreat(task,tip,keyed):
    s=PhysicalScorer(task,[.1,0,.2],hole=[0,0,0],tip=tip,keyed=keyed)
    def sample(i,z,contacts,opening=.4,rotation=None):
        x=row(i*.005);x.update(tool_position_m=[.1,0,.2],contacts=contacts,joint_position=[0.]*6+[opening,-opening],
            plug_position_m=[0,0,z],plug_rotation=(np.diag([1,-1,-1]) if rotation is None else rotation).tolist(),
            plug_velocity_m_s=[0,0,0],plug_angular_velocity_rad_s=[0,0,0]);return x
    for i in range(20):s.update(sample(i,.15 if i<10 else .20,[1,1]))
    for i in range(20,40):s.update(sample(i,.025,[1,1]))
    for i in range(40,450):s.update(sample(i,.025,[0,0],.65))
    assert s.report()['success']
    # Losing seating after previously reaching success invalidates the final state.
    s.update(sample(450,.05,[0,0],.65))
    assert not s.report()['success']


def test_infrastructure_progress_is_not_scorable():
    result=aggregate([dict(status='infrastructure_error',progress=1.),dict(status='task_failure',progress=.25)])
    assert result['mean_progress_scorable']==.25


def test_batch_keeps_exact_count_when_every_initialization_rejects(tmp_path):
    from types import SimpleNamespace
    from tools.evaluate_fixed_trajectories import evaluate
    model=tmp_path/'model.xml';model.write_text('model')
    args=SimpleNamespace(output=tmp_path/'results',canonical_dir=tmp_path/'missing',model=model,
                         task='drawer_cycle',trials=3,wall_timeout=1.)
    assert evaluate(args)==2
    import json
    result=json.loads((args.output/'aggregate.json').read_text())
    assert result['attempts']==3
    assert result['counts']['initialization_rejected']==3
    assert result['tasks']['drawer_cycle']['donor_result'] is None


def test_offline_rescore_agrees(tmp_path):
    import json
    from tools.evaluate_fixed_trajectories import score_recording
    spec=dict(task='drawer_cycle',final_tool_position=[0,0,.2],drawer_spec=drawer_scorer().drawer_monitor.specification())
    (tmp_path/'scorer_spec.json').write_text(json.dumps(spec))
    rows=[row(i*.005) for i in range(10)]
    (tmp_path/'evaluation_rows.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    online=PhysicalScorer(**spec)
    for r in rows:online.update(r)
    assert score_recording(tmp_path)==online.report()


def test_only_zero_time_wrench_bootstrap_may_be_unavailable():
    s=drawer_scorer()
    x=row(0);x['wrench_valid']=False;s.update(x)
    s.update(row(.005))
    assert s.report()['status']=='task_failure'
    x=row(.010);x['wrench_valid']=False;s.update(x)
    assert s.report()['status']=='unscorable'


@pytest.mark.parametrize('value',[None,[],42])
def test_non_object_trajectory_rejected(value):
    with pytest.raises(ValueError):FrozenTrajectory(value)


def test_resolved_physics_must_match_donor():
    d=document();d['physics_identity']=dict(substeps=4,plug_retention={'enabled':True},orbital_physics={})
    t=FrozenTrajectory(d)
    t.validate_physics(d['physics_identity'])
    with pytest.raises(ValueError):t.validate_physics(dict(d['physics_identity'],substeps=8))
    with pytest.raises(ValueError):FrozenTrajectory(document()).validate_physics(d['physics_identity'])


def test_applied_velocity_is_frozen_instead_of_reconstructed():
    d=document()
    for sample in d['samples']:sample['joint_reference_velocity']=[.03]*8
    t=FrozenTrajectory(d)
    assert t.reference(.5)[1]==pytest.approx([.03]*8)
    assert t.reference(1.1)[1]==pytest.approx([0.]*8)


def test_dependency_list_is_rejected():
    d=document();d['dependencies']=[]
    with pytest.raises(ValueError):FrozenTrajectory(d)


def test_replay_step_does_not_call_adaptive_controller():
    from types import SimpleNamespace
    from simulation.evaluation.replay import FixedReplayTask
    def forbidden(*args):raise AssertionError('Adaptive controller was called')
    donor=SimpleNamespace(step=forbidden,chain=SimpleNamespace(forward=lambda q:(np.array([0.,0.,.2]),np.eye(3))))
    replay=FixedReplayTask.__new__(FixedReplayTask)
    replay.donor=donor;replay.trajectory=FrozenTrajectory(document());replay.scorer=drawer_scorer()
    replay.result=None;replay.events=[];replay.reason=''
    sample=SimpleNamespace(time_s=0.,joints=np.zeros(8),joint_velocities=np.zeros(8),wrench=np.zeros(6),wrench_valid=False,
                           drawer_m=0.,drawer_velocity_m_s=0.,buckle_rad=0.,finger_drawer_contacts=[0,0])
    command,_=replay.step(sample)
    assert command==pytest.approx([0.]*8)
    assert replay.result is None


def test_overload_blocks_otherwise_successful_drawer():
    s=drawer_scorer()
    for i in range(30):
        x=row(i*.005);x['wrench']=[81.,0,0,0,0,0];s.update(x)
    assert s.report()['reason']=='wrist_overload'
    assert not s.report()['success']


def test_initialization_hash_mismatch_is_rejected(tmp_path):
    d=document();t=FrozenTrajectory(d);model=tmp_path/'model.xml';model.write_text('changed')
    with pytest.raises(ValueError,match='model hash'):t.validate_runtime(model,[0.]*8,tmp_path)


def test_batch_keeps_malformed_canonical_in_denominator(tmp_path):
    import json
    from types import SimpleNamespace
    from tools.evaluate_fixed_trajectories import evaluate
    model=tmp_path/'model.xml';model.write_text('model')
    canonical=tmp_path/'canonical/drawer_cycle';canonical.mkdir(parents=True)
    (canonical/'trajectory.json').write_text('[]')
    args=SimpleNamespace(output=tmp_path/'results',canonical_dir=canonical.parent,model=model,
                         task='drawer_cycle',trials=3,wall_timeout=1.)
    assert evaluate(args)==2
    result=json.loads((args.output/'aggregate.json').read_text())
    assert result['attempts']==3 and result['counts']['initialization_rejected']==3


def test_timeout_kills_interpreter_children(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from tools import evaluate_fixed_trajectories as cli
    process=Mock(pid=123)
    process.wait.side_effect=[cli.subprocess.TimeoutExpired('test',1),0]
    child=Mock()
    monkeypatch.setattr(cli.subprocess,'Popen',Mock(return_value=process))
    monkeypatch.setattr(cli.psutil,'Process',Mock(return_value=SimpleNamespace(children=lambda recursive:[child])))
    result=cli.execute('drawer_cycle',tmp_path/'model.xml',tmp_path/'native',wall_timeout=1)
    assert result['timed_out']
    child.kill.assert_called_once();process.kill.assert_called_once()


def test_single_missing_200hz_sample_is_unscorable():
    s=drawer_scorer();s.update(row(0));s.update(row(.01))
    assert s.report()['status']=='unscorable'


def test_missing_reference_velocity_is_not_silently_replaced():
    d=document();del d['samples'][0]['joint_reference_velocity']
    with pytest.raises(ValueError):FrozenTrajectory(d)


def test_drawer_retreat_goal_cannot_be_weakened_by_failed_donor_endpoint():
    from types import SimpleNamespace
    from simulation.evaluation.replay import scorer_for
    from test_unified_drawer_scoring import MODEL
    donor=SimpleNamespace(grip_origin=np.array([9.,9.,9.]),axis=np.array([1.,0.,0.]),
                          config=SimpleNamespace(clearance_m=0.))
    s=scorer_for(donor,FrozenTrajectory(document()),model=MODEL)
    assert s.final==pytest.approx([.455,.105,.240])
