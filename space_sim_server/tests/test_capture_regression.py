from pathlib import Path
import json
import pytest

from space_arm_platform.capture_regression import prepare_trial, temporary_vision_runtime, physical_fingerprint
from space_arm_platform.capture_regression import free_reference_scene
from space_arm_platform.capture_regression_score import score_hold, phase_evidence
from space_arm_platform.capture_regression_runtime import ensure_ports_free, ManagedProcesses

ROOT=Path(__file__).resolve().parents[1]


def test_recipe_is_portable_and_keeps_real_capture_configuration(tmp_path):
    prepared=prepare_trial(tmp_path/'trial with spaces',Path('/chosen/vision/python'),root=ROOT)
    scene=json.loads(prepared.scene.read_text())
    runtime=json.loads(prepared.vision.read_text())
    assert Path(runtime['python_executable'])==Path('/chosen/vision/python')
    assert Path(runtime['wrist_evaluation_directory']).is_relative_to(prepared.output)
    assert Path(scene['randomization']['capture_visual_test']['output_directory']).is_relative_to(prepared.output)
    assert scene['randomization']['capture_visual_test']['held_target_pose'] is True
    assert scene['randomization']['coorbital_initialization']['grip_frame_standoff_m']==.5
    assert scene['randomization']['formation_hold']['enabled'] is False
    assert scene['runtime']['simulation_rate']==.5
    assert scene['render_profile']['vision_capture']['stereo_enabled'] is False
    with pytest.raises(FileExistsError):
        prepare_trial(prepared.output,Path('/chosen/vision/python'),root=ROOT)


def test_retained_scene_uses_current_main_attachment_fingerprints():
    scene=json.loads((ROOT/'config/experiments/visual_gyro_capture/scene.json').read_text())
    profile=json.loads((ROOT/'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
    assert scene['render_profile']['visual_attachments']==profile['visual_attachments']


def test_corrupt_visual_attachment_is_rejected_before_any_process_starts(tmp_path):
    import shutil
    root=tmp_path/'checkout'
    shutil.copytree(ROOT/'config/experiments/visual_gyro_capture',root/'config/experiments/visual_gyro_capture')
    scene=json.loads((root/'config/experiments/visual_gyro_capture/scene.json').read_text())
    geometry=root/scene['render_profile']['visual_attachments'][0]['geometry_file']
    geometry.parent.mkdir(parents=True);geometry.write_text('[]')
    with pytest.raises(ValueError,match='attachment'):
        prepare_trial(tmp_path/'trial',Path('/vision/python'),root=root)


@pytest.mark.parametrize('failure',[False,True])
def test_temporary_config_and_previous_observation_are_restored(tmp_path,failure):
    run=tmp_path/'run';run.mkdir()
    config=run/'vision_runtime.json';config.write_bytes(b'original config bytes')
    observation=run/'wrist_alignment_observation.json';observation.write_bytes(b'old observation')
    supplied=tmp_path/'vision.json';supplied.write_text('{}')
    try:
        with temporary_vision_runtime(tmp_path,supplied):
            assert config.read_text()=='{}'
            assert not observation.exists()
            observation.write_text('new observation')
            if failure:raise RuntimeError('test failure')
    except RuntimeError:
        assert failure
    assert config.read_bytes()==b'original config bytes'
    assert observation.read_bytes()==b'old observation'
    assert not (run/'capture_regression.lock').exists()


def test_lease_refuses_another_run_and_cleans_only_its_own_files(tmp_path):
    supplied=tmp_path/'vision.json';supplied.write_text('{}')
    with temporary_vision_runtime(tmp_path,supplied):
        with pytest.raises(RuntimeError,match='already'):
            with temporary_vision_runtime(tmp_path,supplied):pass
    assert not (tmp_path/'run/vision_runtime.json').exists()


def test_physics_is_free_and_contains_expected_mass_data():
    frozen=physical_fingerprint(ROOT)
    assert frozen['model_sha256']
    assert frozen['target_has_freejoint']
    assert frozen['inter_satellite_fixed_constraints']==[]
    assert frozen['inertials']['capture_target']['mass']=='8.0'
    assert frozen['target_assembly_mass_kg']==10


def test_reference_changes_only_experiment_toggles_and_run_speed():
    scene=json.loads((ROOT/'config/experiments/visual_gyro_capture/scene.json').read_text())
    reference=free_reference_scene(scene)
    assert scene['randomization']['capture_window_trial'] is True
    assert reference['randomization']['capture_window_trial'] is False
    assert reference['render_profile']['vision_capture']['wrist_alignment_enabled'] is False
    for key in ('capture_arm_joint_position_rad','coorbital_initialization','target_position_m','formation_hold'):
        assert reference['randomization'][key]==scene['randomization'][key]
    assert reference['environment']==scene['environment']


def test_hold_requires_continuous_four_support_force_and_does_not_bridge_bad_sample():
    def row(t,n=4,force=5):
        return dict(type='step',t=t,phase='hold_force',clamp={'jaw_support_count':n,'pad_force_n':force},contacts=[])
    dense=[row(k/400) for k in range(8001)]
    assert score_hold(dense)['continuous_hold_s']==20
    dense[4000]=row(10,3)
    assert 9.99<score_hold(dense)['continuous_hold_s']<10.01
    assert score_hold([row(0),row(21,force=7)])['continuous_hold_s']==0
    assert score_hold([])['continuous_hold_s']==0


def test_hold_does_not_call_a_gap_in_contact_records_continuous_support():
    rows=[dict(type='step',t=t,phase='hold_force',clamp={'jaw_support_count':4,'pad_force_n':5},contacts=[])
          for t in (0,20)]
    assert score_hold(rows)['continuous_hold_s']==0


def test_writer_shutdown_precedes_configuration_restoration(tmp_path):
    run=tmp_path/'run';run.mkdir()
    old=run/'wrist_alignment_observation.json';old.write_text('previous')
    supplied=tmp_path/'vision.json';supplied.write_text('{}')
    def shutdown():old.write_text('worker shutdown output')
    with temporary_vision_runtime(tmp_path,supplied,cleanup=shutdown):pass
    assert old.read_text()=='previous'


def test_phase_evidence_rejects_abort_and_absent_prediction():
    names=['open','extend','approach_visual','approach_encoder','close','tighten','hold_force']
    controls=[dict(t=i,status='predictive_'+name) for i,name in enumerate(names)]
    assert phase_evidence(controls)['complete']
    assert not phase_evidence(controls[:3]+controls[4:])['complete']
    assert not phase_evidence(controls+[dict(t=8,status='predictive_abort_fault')])['complete']


def test_occupied_port_is_rejected_without_touching_listener():
    import socket
    with socket.create_server(('127.0.0.1',0)) as listener:
        port=listener.getsockname()[1]
        with pytest.raises(RuntimeError,match='occupied'):ensure_ports_free([port])
        assert listener.fileno()>=0


def test_process_cleanup_stops_owned_child_and_preserves_unrelated_process(tmp_path):
    import subprocess,sys
    outsider=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)'])
    try:
        processes=ManagedProcesses(tmp_path)
        child=processes.start('owned',[sys.executable,'-c','import time;time.sleep(20)'])
        processes.close()
        assert child.poll() is not None
        assert outsider.poll() is None
    finally:
        outsider.terminate();outsider.wait(timeout=10)
