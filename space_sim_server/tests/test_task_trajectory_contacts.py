"""Acceptance must preserve task success and distinguish legal physical contact."""

from pathlib import Path
import gzip
import json
from types import SimpleNamespace

import pytest

from tools.validate_task_trajectory_contacts import (
    ContactAccumulator, ContactPolicy, TASKS, REQUIRED_OPERATION_STAGES, analyze_existing,
)


MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def test_required_paths_match_the_current_production_stage_enumerations():
    from simulation.scripted_tasks.plug_insert import PlugInsertTask
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
    from simulation.scripted_tasks.drawer_lift import DrawerLiftTask

    for task in ("aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"):
        assert REQUIRED_OPERATION_STAGES[task] == PlugInsertTask.stages
    assert REQUIRED_OPERATION_STAGES["drawer_cycle"] == DrawerCycleTask.stages
    assert REQUIRED_OPERATION_STAGES["drawer_lift"] == DrawerLiftTask.stages


@pytest.mark.parametrize("task", TASKS)
def test_only_fingers_may_touch_the_active_task_object(task):
    policy = ContactPolicy(MODEL, task)
    active = sorted(policy.active_bodies)[0]
    assert policy.classify("ee_finger1", active, "finger", "part", -.001) == "legal_grasp"
    assert policy.classify(active, "ee_finger2", "part", "finger", -.001) == "legal_grasp"
    assert policy.classify("ee_base", active, "palm", "part", -.00001) == "forbidden_robot_contact"
    assert policy.classify("ee_finger1", "task_box_base", "finger", "deck", -.000001) == "forbidden_robot_contact"
    inactive = "guide_plug" if active != "guide_plug" else "aviation_plug"
    assert policy.classify("ee_finger2", inactive, "finger", "part", -.00001) == "forbidden_robot_contact"


def test_mating_uses_selected_hole_ownership_and_never_an_entire_shared_board():
    policy = ContactPolicy(MODEL, "aviation_plug_insert")
    classify = lambda body, geom: policy.classify("aviation_plug", body, "plug_shaft", geom, -.00001)
    assert classify("board1_plug_bore_colliders", "aviation_bore_40p5_000") == "legal_mating"
    assert classify("board1_plug_bore_colliders", "aviation_pin_floor_40p5_015_3") == "legal_mating"
    assert classify("board1_plug_bore_colliders", "aviation_bore_40p1_000") == "forbidden_active_object_contact"
    assert classify("board1_plug_bore_colliders", "guide_bore_40p5_000") == "forbidden_active_object_contact"
    assert classify("asm_board1", "unassigned_shared_plate") == "forbidden_active_object_contact"
    assert classify("task_box_base", "task_box_base_geom_collision_000") == "forbidden_active_object_contact"


def test_service_hole_contact_policy_keeps_recipient_and_hole_identity(tmp_path):
    model=tmp_path/'two_boxes.xml'
    model.write_text('''<mujoco><worldbody><body name="cubesat_bus">
        <body name="link1"><body name="ee_finger1"/><body name="ee_finger2"/></body>
        <body name="service_board1_plug_bore_colliders">
          <geom name="service_aviation_bore_40p5_000"/>
          <geom name="service_aviation_bore_40p1_000"/>
          <geom name="service_guide_bore_40p5_000"/>
        </body><body name="board1_plug_bore_colliders"><geom name="aviation_bore_40p5_000"/></body>
        </body></worldbody></mujoco>''',encoding='utf-8')
    policy=ContactPolicy(model,'aviation_plug_insert','service_aviation_insert_40p5')
    def classify(geom):
        return policy.classify('aviation_plug','service_board1_plug_bore_colliders','shaft',geom,-.0001)
    assert classify('service_aviation_bore_40p5_000')=='legal_mating'
    for geom in ('service_aviation_bore_40p1_000','service_guide_bore_40p5_000',
                 'aviation_bore_40p5_000','unknown'):
        assert classify(geom)=='forbidden_active_object_contact'
    with pytest.raises(ValueError):
        ContactPolicy(model,'aviation_plug_insert','service_guide_insert_40p5')


def test_controller_snapshot_labels_nonvertical_seat_coordinate():
    import numpy as np
    from tools.validate_task_trajectory_contacts import controller_snapshot
    task=SimpleNamespace(reference=np.zeros(8),_seat_z=.25,
                         hole_rotation=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]]))
    report=controller_snapshot(task)
    assert report['seat_target_z_m'] is None
    assert report['seat_target_axis_coordinate_m']==.25
    assert report['seat_axis_body']==[1.,0.,0.]
    task.hole_rotation=np.eye(3)
    assert controller_snapshot(task)['seat_target_z_m']==.25


def test_original_shared_pickup_colliders_are_owned_by_vertices_not_a_whole_board():
    policy = ContactPolicy(MODEL, "aviation_plug_insert")
    assert policy.module("asm_board2", "asm_board2_geom_collision_006") == "aviation_pickup"
    assert policy.module("asm_board2", "asm_board2_geom_collision_088") == "cylindrical_pickup"
    assert policy.classify("aviation_plug", "asm_board2", "shaft",
                           "asm_board2_geom_collision_006", -.0001) == "legal_mating"
    assert policy.classify("aviation_plug", "asm_board2", "shaft",
                           "asm_board2_geom_collision_088", -.0001) == "forbidden_active_object_contact"


def test_legal_grasp_does_not_hide_self_contact_or_other_mechanism_collision():
    policy = ContactPolicy(MODEL, "drawer_cycle")
    assert policy.classify("ee_finger1", "drawer_buckle", "f", "b", -.001) == "legal_grasp"
    assert policy.classify("drawer_buckle", "asm_board3", "b", "mount", -.001) == "legal_mating"
    assert policy.classify("ee_finger1", "limit_clip", "f", "clip", -.000001) == "forbidden_robot_contact"
    assert policy.classify("link2", "ee_finger1", "arm", "f", -.001) == "robot_self_contact"
    assert policy.classify("guide_plug", "asm_board2", "plug", "mount", -.001) == "background_contact"
    assert policy.classify("ee_finger1", "limit_clip", "f", "clip", .000001) == "positive_candidate"


def accumulator(stamps=(0, 2_500_000, 5_000_000), contacts=(), substeps=1):
    result = ContactAccumulator(ContactPolicy(MODEL, "aviation_plug_insert"))
    result.substeps.add(substeps)
    names = {"bodies": {"1": "ee_finger1", "2": "limit_clip"},
             "geoms": {"3": "finger", "4": "clip"}}
    for stamp in stamps:
        result.observe(stamp, "release_retract", contacts, names=names)
    return result


def test_success_and_complete_clear_observation_are_both_required():
    state = accumulator(stamps=())
    for index, stage in enumerate(REQUIRED_OPERATION_STAGES["aviation_plug_insert"]):
        state.observe(index * 2_500_000, stage, [])
    seconds = state.last_ns / 1e9
    failed = state.report({"result": "failed", "reason": "original_spin_threshold", "simulated_seconds": seconds}, 1)
    assert failed["trajectory_clear"] and failed["coverage_complete"]
    assert failed["operational_path_complete"]
    assert not failed["task_success"] and not failed["accepted"]
    passed = state.report({"result": "success", "simulated_seconds": seconds}, 0)
    assert passed["task_success"] and passed["trajectory_clear"] and passed["accepted"]


def test_release_collision_rejects_success_and_counts_ticks_separately_from_points():
    state = accumulator(contacts=[[3, 4, 1, 2, -.00002], [3, 4, 1, 2, -.00003]])
    report = state.report({"result": "success", "simulated_seconds": .005}, 0)
    assert report["task_success"] and report["coverage_complete"]
    assert not report["trajectory_clear"] and not report["accepted"]
    record, = report["contact_pairs"]
    assert record["observed_ticks"] == 3
    assert record["contact_point_samples"] == 6
    assert record["minimum_separation_m"] == -.00003
    assert record["stages"] == {"release_retract": 3}


@pytest.mark.parametrize("stamps,substeps", [((0, 5_000_000), 1), ((0, 2_500_000), 1),
                                              ((0, 2_500_000, 5_000_000), 2),
                                              ((0, 2_500_000, 2_500_000, 5_000_000), 1)])
def test_missing_ticks_unobserved_substeps_and_duplicate_ticks_cannot_pass(stamps, substeps):
    report = accumulator(stamps=stamps, substeps=substeps).report(
        {"result": "success", "simulated_seconds": .005}, 0)
    assert report["trajectory_clear"]
    assert not report["coverage_complete"] and not report["accepted"]


def test_historical_200hz_and_invalid_contacts_cannot_claim_full_coverage():
    legacy = accumulator(stamps=(0, 5_000_000)).report(
        {"result": "success", "simulated_seconds": .005}, 0, expected_hz=200)
    assert not legacy["coverage_complete"] and not legacy["accepted"]
    broken = accumulator(contacts=[[3, 4, 1, 2, float("nan")]]).report(
        {"result": "success", "simulated_seconds": .005}, 0)
    assert not broken["trajectory_clear"] and not broken["coverage_complete"]
    empty = accumulator(stamps=()).report({"result": "success", "simulated_seconds": 0}, 0)
    assert not empty["trajectory_clear"] and not empty["accepted"]


def test_explicit_bolt_deferral_preserves_other_forbidden_contacts_and_inventory():
    strict = ContactPolicy(MODEL, "guide_plug_insert")
    deferred = ContactPolicy(MODEL, "guide_plug_insert", ignore_free_bolt=True)
    assert strict.classify("guide_plug", "bolt", "flange", "bolt", -.0001) == "forbidden_active_object_contact"
    assert deferred.classify("guide_plug", "bolt", "flange", "bolt", -.0001) == "deferred_bolt_contact"
    assert deferred.classify("bolt", "link2", "bolt", "arm", -.0001) == "deferred_bolt_contact"
    assert deferred.classify("ee_finger1", "cylindrical_plug", "f", "p", -.0001) == "forbidden_robot_contact"
    assert deferred.classify("guide_plug", "aviation_plug", "p", "p", -.0001) == "forbidden_active_object_contact"
    assert deferred.classify("guide_plug", "bolt", "p", "bolt", .0001) == "positive_candidate"
    assert deferred.metadata()["ignore_free_bolt"] is True
    assert strict.metadata()["ignore_free_bolt"] is False


def test_reanalysis_changes_only_classification_and_keeps_native_launch_and_errors(tmp_path):
    source, output = tmp_path / "source", tmp_path / "analysis"
    (source / "run").mkdir(parents=True)
    output.mkdir()
    names = {"bodies": {"1": "guide_plug", "2": "bolt"},
             "geoms": {"3": "flange", "4": "bolt"}}
    (source / "names.json").write_text(json.dumps(names))
    stages = REQUIRED_OPERATION_STAGES["guide_plug_insert"]
    (source / "run/summary.json").write_text(json.dumps(
        dict(result="success", simulated_seconds=(len(stages) - 1) * .0025, substeps=1)))
    launch = b'{"native_returncode": 0, "original_launch_marker": true}\n'
    (source / "launch.json").write_bytes(launch)
    with gzip.open(source / "contacts-400hz.jsonl.gz", "wt") as stream:
        for index, stage in enumerate(stages):
            stream.write(json.dumps(dict(interval_end_ns=index * 2_500_000, stage=stage,
                                         contacts=[[3, 4, 1, 2, -.0001]])) + "\n")
    report = analyze_existing(SimpleNamespace(analyze_existing=source),
                              ContactPolicy(MODEL, "guide_plug_insert", ignore_free_bolt=True), output)
    assert report["task_success"] and report["trajectory_clear"] and report["coverage_complete"]
    assert report["operational_path_complete"] and not report["accepted"]
    assert report['independent_evaluation']['status']=='unscorable'
    assert report["categories"]["deferred_bolt_contact"] == len(stages)
    assert report["contact_pairs"][0]["minimum_separation_m"] == -.0001
    assert (output / "launch.json").read_bytes() == launch
    assert report["original_launch"]["path"] == str(source / "launch.json")
    assert report["analysis_source_sha256"]
    # A native cleanup/execution error must survive reclassification even if
    # the task wrote a success summary before that error happened.
    (source / "acceptance.json").write_text(json.dumps(dict(execution_error="native cleanup failed")))
    failed_output = tmp_path / "analysis-error"
    failed_output.mkdir()
    failed = analyze_existing(SimpleNamespace(analyze_existing=source),
                              ContactPolicy(MODEL, "guide_plug_insert", ignore_free_bolt=True), failed_output)
    assert failed["task_success"] and not failed["coverage_complete"] and not failed["accepted"]


def test_complete_sampling_of_a_failed_plan_is_not_a_complete_operational_path():
    state = accumulator()
    report = state.report(dict(result="failed", reason="planning_failed: Boundary velocity", simulated_seconds=.005), 1)
    assert report["trajectory_clear"] and report["coverage_complete"]
    assert not report["operational_path_complete"]
    assert "verify" in report["missing_operation_stages"]
    complete = accumulator(stamps=())
    stages = REQUIRED_OPERATION_STAGES["aviation_plug_insert"]
    for index, stage in enumerate(stages):
        complete.observe(index * 2_500_000, stage, [])
    planning_failure = complete.report(dict(result="failed", reason="planning_failed: Boundary velocity",
                                            simulated_seconds=complete.last_ns / 1e9), 1)
    assert not planning_failure["operational_path_complete"]


def test_physical_gate_recomputes_rows_and_requires_online_agreement(tmp_path):
    from tools.validate_task_trajectory_contacts import apply_physical_gate
    from simulation.evaluation.scoring import PhysicalScorer
    import numpy as np
    from test_fixed_trajectory_evaluation import row
    spec=dict(task='guide_plug_insert',final_tool_position=[.1,0,.2],hole=[0,0,0],tip=.095,keyed=True)
    (tmp_path/'independent-scorer-spec.json').write_text(json.dumps(spec))
    scorer=PhysicalScorer(**spec)
    with gzip.open(tmp_path/'states-200hz.jsonl.gz','wt') as stream:
        for i in range(450):
            sample=row(i*.005)
            opening=.4 if i<40 else .65
            sample.update(tool_position_m=[.1,0,.2],contacts=[1,1] if i<40 else [0,0],
                joint_position=[0.]*6+[opening,-opening],
                plug_position_m=[0,0,(.15 if i<10 else .20) if i<20 else .025],
                plug_rotation=np.diag([1,-1,-1]).tolist(),plug_velocity_m_s=[0,0,0],
                plug_angular_velocity_rad_s=[0,0,0])
            scorer.update(sample)
            stream.write(json.dumps(dict(evaluation=sample))+'\n')
    online=scorer.report()
    assert online['success']
    matched=dict(accepted=True)
    apply_physical_gate(matched,tmp_path,online)
    assert matched['accepted'] and matched['online_offline_agree']
    changed=dict(accepted=True)
    apply_physical_gate(changed,tmp_path,dict(online,success=False))
    assert changed['independent_evaluation']['success'] and not changed['accepted']


def test_unsupported_old_drawer_spec_is_unscorable_on_reanalysis(tmp_path):
    from tools.validate_task_trajectory_contacts import apply_physical_gate
    (tmp_path/'independent-scorer-spec.json').write_text(json.dumps(dict(task='drawer_cycle',final_tool_position=[0,0,.2])))
    with gzip.open(tmp_path/'states-200hz.jsonl.gz','wt') as stream:stream.write('')
    report=dict(accepted=True)
    apply_physical_gate(report,tmp_path,dict(success=True))
    assert not report['accepted'] and report['independent_evaluation']['status']=='unscorable'


def test_native_drawer_spec_includes_full_current_policy_geometry():
    from tools.validate_task_trajectory_contacts import physical_scorer
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
    task=DrawerCycleTask(MODEL)
    scorer,spec=physical_scorer(task)
    assert spec['drawer_spec']['sample_period_ns']==5_000_000
    assert scorer.drawer_monitor is not None


def test_utf8_drawer_instruction_can_be_rescored_on_windows(tmp_path):
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
    from tools.validate_task_trajectory_contacts import physical_scorer,apply_physical_gate,write_json
    from test_fixed_trajectory_evaluation import row
    scorer,spec=physical_scorer(DrawerCycleTask(MODEL))
    write_json(tmp_path/'independent-scorer-spec.json',spec)
    with gzip.open(tmp_path/'states-200hz.jsonl.gz','wt') as stream:
        for i in range(2):
            sample=row(i*.005);scorer.update(sample)
            stream.write(json.dumps(dict(evaluation=sample))+'\n')
    report=dict(accepted=True)
    apply_physical_gate(report,tmp_path,scorer.report())
    assert report['online_offline_agree']
    assert report['independent_evaluation']['status']=='task_failure'


def test_controller_diagnostics_are_serializable_copies_not_control_inputs():
    import numpy as np
    from tools.validate_task_trajectory_contacts import controller_snapshot
    task=SimpleNamespace(reference=np.zeros(8),_plug_goal=np.array([.5,.2,.03075]),
        _seat_z=.03075,_seat_recovery=True,_seat_unloaded=False)
    row=controller_snapshot(task)
    task.reference[0]=1.;task._plug_goal[2]=.03
    assert row['last_joint_reference'][0]==0.
    assert row['plug_goal_body_m'][2]==.03075
    assert row['seat_recovery'] and not row['seat_unloaded']
    json.dumps(row)
