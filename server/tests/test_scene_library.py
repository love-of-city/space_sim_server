import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from simulation.scene_library.catalog import catalog
from simulation.scene_library.manifest import compose, compose_mounted_operation, verify_files, write_json
from simulation.scene_library.runner import run
from simulation.scene_library.scaling import scale_satellite
from simulation.scene_library.frames import mounted_body_pose
from simulation.scene_library.assembly import mounted_target_fixture
import numpy as np


@pytest.fixture
def repository(tmp_path):
    model = tmp_path / "model/SARM/platform/sarm_20261012.xml"
    model.parent.mkdir(parents=True)
    model.write_text('<mujoco><asset><mesh name="shell" file="shell.stl"/></asset><worldbody><body name="capture_target" pos="1 2 3"/></worldbody></mujoco>')
    model.with_name("shell.stl").write_bytes(b"test asset dependency")
    model.with_name("attitude_control.json").write_text('{}')
    model.with_name('sarm_dual_capture26.xml').write_bytes(model.read_bytes())
    return tmp_path


def test_catalog_preserves_diagram_and_first_phase():
    scenes = catalog()["scenes"]
    assert len(scenes) == 27
    assert len({s["id"] for s in scenes}) == 27
    assert [sum(s["family_number"] == n for s in scenes) for n in range(1, 9)] == [5, 5, 1, 4, 6, 4, 1, 1]
    assert all(not s["accepted"] for s in scenes)
    assert sum(s["phase"] == 1 for s in scenes) == 15


def test_materialized_library_preserves_scope_and_rejects_changed_manifest(repository):
    from simulation.scene_library.materialize import materialize, library_instances, phase_one_presets
    entries = phase_one_presets()
    assert len(entries) == 14
    assert not any(e['scene_id'] == '04_cross_transfer/drawer_cycle' for e in entries)
    output = repository / 'library'
    record = materialize(repository, output, geometry_python='unused',
        scene_ids=['01_self_operation/drawer_cycle'])
    assert record['status'] == 'ready_to_run' and record['accepted'] is False
    assert len(library_instances(output / 'library.json')) == 1
    instance = output / record['entries'][0]['instance']
    instance.write_bytes(instance.read_bytes() + b' ')
    with pytest.raises(ValueError, match='hash mismatch'):
        library_instances(output / 'library.json')


def test_verified_export_refuses_incomplete_scope_without_creating_output(tmp_path):
    from simulation.scene_library.materialize import export_verified
    with pytest.raises(ValueError, match='not complete'):
        export_verified(tmp_path, tmp_path / 'export')
    assert not (tmp_path / 'export').exists()


def test_asset_mutation_rejects_replay(repository):
    instance = compose("01_self_operation/drawer_cycle", repository)
    verify_files(instance)
    (repository / "model/SARM/platform/shell.stl").write_bytes(b"changed")
    with pytest.raises(ValueError, match="dependencies changed"):
        verify_files(instance)


def test_lfs_pointer_is_not_a_model_asset(repository):
    (repository / "model/SARM/platform/shell.stl").write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
    with pytest.raises(ValueError, match="Unresolved Git LFS"):
        compose("01_self_operation/drawer_cycle", repository)


def test_incomplete_docking_cannot_pass_from_local_operation(repository):
    instance = compose("02_docked_operation/drawer_cycle", repository)
    result = run(instance, repository / "result")
    assert result["status"] == "unsupported"
    assert not result["accepted"]
    assert "dual_arm_preheld_target_and_hold_validation" in result["missing_capabilities"]


def test_result_paths_never_overwrite_evidence(tmp_path):
    path = tmp_path / "receipt.json"
    write_json(path, {"accepted": False})
    with pytest.raises(FileExistsError):
        write_json(path, {"accepted": True})
    assert not json.loads(path.read_text())["accepted"]


def test_raw_operation_success_cannot_complete_docked_family(tmp_path):
    from simulation.scene_library.status import summarize
    write_json(tmp_path / 'acceptance.json', dict(schema='task-trajectory-contacts/3-local-physical',
        accepted=True, independent_evaluation={'success': True}, trajectory_clear=True))
    write_json(tmp_path / 'receipt.json', dict(schema='space-arm-scene-run/1',
        scene_id='02_docked_operation/drawer_cycle', accepted=True, status='passed'))
    report = summarize(tmp_path)
    assert report['native_operations'][0]['operation_accepted'] is True
    assert report['families'][1]['accepted_tasks'] == 0
    assert report['first_four_complete'] is False
    assert report['phase_one']['families'][1]['accepted_tasks'] == 0
    assert report['phase_one']['families'][3]['required_tasks'] == 3
    assert report['phase_one']['scope']['required_tasks'] == 14


def test_preheld_phase_rejects_deck_mount_and_requires_hold_evidence(tmp_path):
    from simulation.scene_library.status import phase_one_evidence
    from simulation.scene_library.catalog import recipe
    native = tmp_path / 'native'
    (native / 'run').mkdir(parents=True)
    item = recipe('04_cross_transfer/guide_plug_insert')
    record = dict(native_report=str(native / 'acceptance.json'), operation_accepted=True,
                  returncode=0, evidence_scope='rigidly_predocked_operation_only')
    write_json(tmp_path / 'instance.json', dict(recipe=item, model='/model.xml',
        dependencies=[dict(path='/model.xml', sha256='model-sha')],
        operation_fixture=dict(scope=record['evidence_scope'])))
    write_json(native / 'run/summary.json', dict(model_sha256='model-sha',
        preheld_capture=dict(hold_gains=dict(kp=1500., kd=80., torque_limit_nm=80.))))
    write_json(native / 'acceptance.json', dict(accepted=True, coverage_complete=True,
        trajectory_clear=True, independent_evaluation=dict(success=True)))
    inspections = {'model-sha': [dict(schema='space-arm-mount-inspection/2', passed=True)]}
    assert not phase_one_evidence(tmp_path / 'receipt.json', record, inspections, item)
    record['evidence_scope'] = 'dual_arm_preheld_operation'
    instance = json.loads((tmp_path / 'instance.json').read_text(encoding='utf-8'))
    instance['operation_fixture']['scope'] = record['evidence_scope']
    (tmp_path / 'instance.json').write_text(json.dumps(instance))
    assert not phase_one_evidence(tmp_path / 'receipt.json', record, inspections, item)
    from simulation.scene_library.manifest import digest
    trace, contacts = native / 'states-200hz.jsonl.gz', native / 'contacts-400hz.jsonl.gz'
    trace.write_bytes(b'full pose trace')
    contacts.write_bytes(b'full contact trace')
    hold = tmp_path / 'hold.json'
    write_json(hold, dict(model_sha256='model-sha', passed=True,
                         trace_sha256=digest(trace), contact_trace_sha256=digest(contacts)))
    record['preheld_capture_validation'] = str(hold)
    record['preheld_capture_validation_sha256'] = digest(hold)
    assert not phase_one_evidence(tmp_path / 'receipt.json', record, inspections, item)
    scene = tmp_path / 'initial-scene.json'
    write_json(scene, {'test': 'bound initial condition'})
    inspection = tmp_path / 'preheld-inspection.json'
    write_json(inspection, dict(schema='space-arm-preheld-inspection/1', model_sha256='model-sha',
        scene_sha256=digest(scene), coverage_complete=True, unsupported_pairs=[],
        geometry_counts={k: 1 for k in ('target_hull', 'hardware', 'target_all', 'service')},
        checks=[dict(groups=list(pair), tested_pairs=1, penetrations=[])
                for pair in [('target_hull', 'hardware'), ('target_all', 'service')]]))
    instance['operation_fixture'].update(mount_inspection=str(inspection),
        mount_inspection_sha256=digest(inspection), initial_scene_source=str(scene), initial_scene_sha256=digest(scene))
    instance['initial_scene'] = {}
    (tmp_path / 'instance.json').write_text(json.dumps(instance))
    checks = ('pose_coverage_200hz', 'contact_coverage_400hz', 'grasp_position_error_under_5mm',
        'grasp_angle_error_under_0p02rad', 'capture_joint_error_under_0p05rad',
        'target_relative_drift_under_5mm', 'no_unintended_capture_external_contact',
        'no_target_service_hardware_collision', 'retention_reactions_bound_to_actual_owners',
        'capture_controller_installed')
    held = json.loads(hold.read_text())
    held.update(scene_sha256=digest(scene), checks={k: True for k in checks})
    hold.write_text(json.dumps(held))
    record['preheld_capture_validation_sha256'] = digest(hold)
    assert phase_one_evidence(tmp_path / 'receipt.json', record, inspections, item)
    trace.write_bytes(b'another run')
    assert not phase_one_evidence(tmp_path / 'receipt.json', record, inspections, item)
    trace.write_bytes(b'full pose trace')
    held['checks']['retention_reactions_bound_to_actual_owners'] = False
    hold.write_text(json.dumps(held))
    record['preheld_capture_validation_sha256'] = digest(hold)
    assert not phase_one_evidence(tmp_path / 'receipt.json', record, {}, item)


def test_batch_continues_after_failure_and_preserves_scope(repository, monkeypatch):
    from simulation.scene_library.runner import run_batch
    instance = compose('01_self_operation/drawer_cycle', repository)
    calls = []
    def execute(instance, folder, **kwargs):
        calls.append(folder)
        if len(calls) == 1:
            raise RuntimeError('diagnostic runtime unavailable')
        return dict(accepted=False, operation_accepted=True, status='operation_passed')
    monkeypatch.setattr('simulation.scene_library.runner.run', execute)
    result = run_batch([instance, instance], repository / 'batch')
    assert len(calls) == 2
    assert result['runs'][0]['status'] == 'infrastructure_error'
    assert result['runs'][1]['operation_accepted'] is True
    assert result['requested_scopes_passed'] is False
    assert result['complete_scenes_passed'] is False
    with pytest.raises(FileExistsError):
        run_batch([instance], repository / 'batch')


def test_failed_rod_preflight_never_starts_dynamics(repository, monkeypatch):
    import subprocess
    instance = compose('01_self_operation/locking_rod_cycle', repository)
    calls = []
    def reject_preflight(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 1)
    monkeypatch.setattr('simulation.scene_library.runner.subprocess.run', reject_preflight)
    receipt = run(instance, repository / 'rod-result')
    assert len(calls) == 1 and 'prepare_locking_rod_path.py' in calls[0][1]
    assert receipt['status'] == 'preparation_failed'
    assert receipt['accepted'] is False
    assert not (repository / 'rod-result/native').exists()


def test_later_mount_rejection_is_kept_with_successful_operation(tmp_path):
    from simulation.scene_library.status import summarize
    (tmp_path / 'run').mkdir()
    write_json(tmp_path / 'run/summary.json', dict(task_name='drawer_cycle', model_sha256='bad-mount'))
    write_json(tmp_path / 'acceptance.json', dict(schema='task-trajectory-contacts/3-local-physical',
        accepted=True, independent_evaluation={'success': True}, trajectory_clear=True))
    inspection = dict(schema='space-arm-mount-inspection/2', model_sha256='bad-mount',
        coverage_complete=True, geometry_counts=dict(target=1, hardware=1, service=1, receiver=0),
        checks=[dict(groups=list(pair), tested_pairs=(0 if 'receiver' in pair else 1),
                     penetrations=[]) for pair in [('target', 'service'), ('target', 'hardware'),
                                                  ('target', 'receiver'), ('receiver', 'service')]],
        unsupported_pairs=[])
    write_json(tmp_path / 'first-inspection.json', inspection)
    inspection['checks'][0]['penetrations'] = [{'depth_m': .02}]
    write_json(tmp_path / 'later-inspection.json', inspection)
    operation = summarize(tmp_path)['native_operations'][0]
    assert operation['operation_accepted'] is True  # Preserve original evidence.
    assert operation['mount_geometry_status'] == 'rejected'
    assert operation['operation_with_mount_check_passed'] is False
    assert len(operation['mount_inspections']) == 2


def test_scaling_clones_shared_mesh_and_preserves_source():
    xml = '<mujoco><asset><mesh name="shell" file="shell.stl"/></asset><worldbody><body name="target" pos="5 0 0"><inertial pos="0 0 1" mass="2" diaginertia="1 2 3"/><geom mesh="shell"/><body name="panel" pos="0 1 0"/></body><body name="other"><geom mesh="shell"/></body></worldbody></mujoco>'
    source = ET.fromstring(xml)
    scaled, record = scale_satellite(source, "target", 2)
    target = scaled.find('.//body[@name="target"]')
    assert target.get("pos") == "5 0 0"
    assert target.find("inertial").get("mass") == "16"
    assert target.find("inertial").get("diaginertia") == "32 64 96"
    assert target.find("body").get("pos") == "0 2 0"
    assert scaled.find('.//body[@name="other"]/geom').get("mesh") == "shell"
    assert source.find('.//body[@name="target"]/geom').get("mesh") == "shell"
    assert record["task_hardware_scaled"] is False


def test_scaling_refuses_installed_task_hardware():
    source = ET.fromstring('<mujoco><worldbody><body name="target"><body name="task_box_base"/></body></worldbody></mujoco>')
    with pytest.raises(ValueError, match="before mounting"):
        scale_satellite(source, "target", 2)


def test_nested_mount_pose_composes_and_rejects_movable_parent():
    root = ET.fromstring('<mujoco><worldbody><body name="cubesat_bus"><body name="mount" pos="1 2 3"><body name="drawer" pos="0 0 .4"><joint type="slide"/></body></body></body></worldbody></mujoco>')
    position, rotation = mounted_body_pose(root, "drawer")
    assert np.allclose(position, [1, 2, 3.4])
    assert np.allclose(rotation, np.eye(3))
    ET.SubElement(root.find('.//body[@name="mount"]'), "freejoint")
    with pytest.raises(ValueError, match="measured-state"):
        mounted_body_pose(root, "drawer")


def test_cross_star_builder_preserves_pairs_and_source(repository, monkeypatch):
    model = repository / "model/SARM/platform/sarm_20261012.xml"
    model.write_text('''<mujoco><asset><mesh name="shell" file="shell.stl"/></asset><worldbody>
    <body name="cubesat_bus"><geom name="base_link_geom" mesh="shell" pos="0 0 -.2"/>
    <body name="link1" pos="0 0 0"/><body name="task_box_base" pos=".1 .2 0">
      <body name="asm_board1"/><body name="board1_plug_bore_colliders"/><body name="asm_board2"/>
    </body><body name="drawer_base" pos=".4 .1 .03"/></body>
    <body name="capture_target" pos="1 2 3"><joint name="target_free" type="free"/>
      <inertial mass="8" pos="0 0 0" diaginertia="1 1 1"/><geom mesh="shell"/></body>
    <body name="aviation_plug"/><body name="guide_plug"/><body name="cylindrical_plug"/><body name="bolt"/>
    </worldbody></mujoco>''')
    before = model.read_bytes()
    result = mounted_target_fixture(model, repository / "generated", cross_transfer=True,
                                    receiver_shift_xy_m=(-.1, .02))
    root = ET.parse(result["model"]).getroot()
    assert root.find('.//geom[@name="base_link_geom"]').get('pos') == '0 0 -.2'
    assert np.allclose(np.fromstring(root.find('.//body[@name="link1"]').get('pos'), sep=' '), [-.1, 0., .08])
    assert root.find('.//body[@name="docked_target"]//body[@name="asm_board2"]') is not None
    receiver = root.find('.//body[@name="service_receiver_mount"]')
    assert np.allclose(np.fromstring(receiver.get('pos'), sep=' '), [-.655, .22, .15])
    assert np.allclose(np.fromstring(root.find('.//body[@name="bolt"]').get('pos'), sep=' '), [-.755, .02, .15])
    assert receiver.find('body[@name="asm_board1"]') is not None
    assert receiver.find('body[@name="board1_plug_bore_colliders"]') is not None
    assert root.find('.//body[@name="mounted_capture_target"]/joint') is None
    assert model.read_bytes() == before
    assert result["physical_docking_accepted"] is False
    initial = json.loads((repository / "generated/initial-scene.json").read_text())
    assert initial["randomization"]["target_position_m"] == [5, 0, 0]
    inspection = repository / 'mount-inspection.json'
    inspection_record = dict(schema='space-arm-mount-inspection/2', model_sha256=result['sha256'],
        coverage_complete=True, geometry_counts=dict(target=1, hardware=1, service=1, receiver=1),
        checks=[dict(groups=list(pair), tested_pairs=1, penetrations=[]) for pair in
                [('target', 'service'), ('target', 'hardware'), ('target', 'receiver'), ('receiver', 'service')]],
        unsupported_pairs=[])
    write_json(inspection, inspection_record)
    instance = compose_mounted_operation('04_cross_transfer/aviation_plug_insert', repository,
        repository / 'generated/assembly.json', repository / 'generated/initial-scene.json',
        transport_clearance_m=.16, mount_inspection=inspection)
    verify_files(instance)
    # Even a successful independent operation cannot certify physical docking.
    import subprocess
    def successful_operation(command, **kwargs):
        native = Path(command[command.index('--output') + 1])
        native.mkdir()
        write_json(native / 'acceptance.json', {'accepted': True,
                    'independent_evaluation': {'success': True}})
        return subprocess.CompletedProcess(command, 0)
    monkeypatch.setattr('simulation.scene_library.runner.subprocess.run', successful_operation)
    receipt = run(instance, repository / 'fixture-result')
    assert receipt['operation_accepted'] is True
    assert receipt['accepted'] is False
    assert receipt['physical_docking_accepted'] is False
    instance['initial_scene']['randomization']['target_position_m'] = [0, 0, 0]
    with pytest.raises(ValueError, match='configuration changed'):
        verify_files(instance)
    with pytest.raises(ValueError, match='Detachable drawer'):
        compose_mounted_operation('04_cross_transfer/drawer_cycle', repository,
            repository / 'generated/assembly.json', repository / 'generated/initial-scene.json')
    bad_inspection = repository / 'missing-hull-inspection.json'
    inspection_record['geometry_counts']['service'] = 0
    write_json(bad_inspection, inspection_record)
    with pytest.raises(ValueError, match='incomplete'):
        compose_mounted_operation('04_cross_transfer/aviation_plug_insert', repository,
            repository / 'generated/assembly.json', repository / 'generated/initial-scene.json',
            mount_inspection=bad_inspection)


def test_raised_plug_clearance_is_opt_in_and_keeps_protection(tmp_path):
    from simulation.scripted_tasks.scene_plug import PlugInsertTask, PlugInsertConfig
    model = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"
    baseline = PlugInsertTask(model)
    assert baseline.config.transport_height_m == .245
    tree = ET.parse(model)
    mount = tree.getroot().find('.//body[@name="task_box_base"]')
    position = [float(v) for v in mount.get("pos").split()]
    position[2] += .45
    mount.set("pos", " ".join(map(str, position)))
    raised = tmp_path / "raised.xml"
    tree.write(raised)
    task = PlugInsertTask(raised, PlugInsertConfig(transport_clearance_m=.24))
    assert task.config.transport_height_m == pytest.approx(.695)
    assert task.config.force_limit_n == baseline.config.force_limit_n == 40.
    assert task.config.torque_limit_nm == baseline.config.torque_limit_nm == 4.
    assert task.config.seat_tolerance_m == baseline.config.seat_tolerance_m
