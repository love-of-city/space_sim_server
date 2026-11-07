"""Capture task compatibility with the existing batch, recorder and v3 reader."""
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest

from collection.capture_initial_pool import CaptureInitialPool, source_scene
from collection.sequence import SequenceConfig, make_plan, prepare_layout, prepare_task_scene
from simulation.scripted_tasks.capture_initialization import CaptureInitialState
from simulation.scripted_tasks.capture_contact_audit import unplanned_capture_contact
from space_arm_platform.models import SimulationObservation, EpisodeStart, EpisodeStop


def capture_observation(index=1):
    from test_lerobot_recording import observation
    data = observation(index).model_dump()
    q = [.1, -.2, .3, -.4, .5, -.6, -.006, .02, -.003, -.004, -.005, -.006]
    target = [x + .001 for x in q]
    data.update(joint_position_rad=q, joint_position_si=q,
                joint_velocity_rad_s=[0.] * 12, joint_velocity_si_s=[0.] * 12,
                target_joint_position_rad=target, target_joint_position_si=target,
                arm_joint_position_rad=q[:6], arm_joint_velocity_rad_s=[0.] * 6,
                target_arm_joint_position_rad=target[:6], gripper_type='capture26_four_jaw')
    return SimulationObservation.model_validate(data)


def test_capture12_requires_explicit_matching_si_vectors():
    data = capture_observation().model_dump()
    for change in ({'joint_position_si': None}, {'joint_position_si': [0.] * 12},
                   {'gripper_type': 'four_bar_tendon'}, {'gripper_position_m': [0., 0.]}):
        with pytest.raises(ValueError):
            SimulationObservation.model_validate({**data, **change})


def test_bearing_and_tilt_are_independent_about_interface_center():
    from simulation.scripted_tasks.capture_grasp import INTERFACE_CENTER, TARGET_FROM_INTERFACE
    p = np.array([.3, -.4, .2])
    plain = CaptureInitialState(gap_m=.15, bearing_x_deg=5., bearing_y_deg=-3.)
    tilted = CaptureInitialState(**{**asdict(plain), 'tilt_x_deg': 10., 'roll_deg': -7.})
    a, ra = plain.geometry(p, np.eye(3))
    b, rb = tilted.geometry(p, np.eye(3))
    center = a + ra @ INTERFACE_CENTER
    np.testing.assert_allclose(center, b + rb @ INTERFACE_CENTER)
    np.testing.assert_allclose(center-p, [.15*np.tan(np.deg2rad(5)), -.15*np.tan(np.deg2rad(3)), -.15])
    assert not np.allclose(ra @ TARGET_FROM_INTERFACE, rb @ TARGET_FROM_INTERFACE)
    for invalid in ({'gap_m': .001}, {'tilt_x_deg': 31.}, {'gap_m': float('nan')}):
        with pytest.raises(ValueError):
            CaptureInitialState(**invalid)


def test_continuous_random_candidates_respect_cones_and_repeat_seed():
    from tools.prepare_capture_initial_pool import sample_candidates
    from simulation.scripted_tasks.capture_grasp import TARGET_FROM_INTERFACE
    config = dict(schema='capture-initial-sampling/1', seed=7, sample_count=80,
                  gap_range_m=[.10, .20], bearing_cone_deg=10., tilt_cone_deg=10., roll_range_deg=[-5., 5.])
    states = sample_candidates(config)
    assert states == sample_candidates(config)
    assert len({s['gap_m'] for s in states}) == 80
    for data in states:
        p, r = CaptureInitialState(**data).geometry(np.zeros(3), np.eye(3))
        from simulation.scripted_tasks.capture_grasp import INTERFACE_CENTER
        center = p + r @ INTERFACE_CENTER
        assert .10 <= data['gap_m'] <= .20 and abs(data['roll_deg']) <= 5.
        assert np.rad2deg(np.arccos(-center[2]/np.linalg.norm(center))) <= 10. + 1e-9
        axis = (r @ TARGET_FROM_INTERFACE)[:, 2]
        assert np.rad2deg(np.arccos(axis[2])) <= 10. + 1e-9


def test_collision_contract_distinguishes_intended_grasp_from_unsafe_hits():
    roots = ['cubesat_bus', 'capture_target']
    assert not unplanned_capture_contact(['collision_ring_convex_1', 'capture_x_piece_1'],
                                        ['gripper', 'capture_target'], roots, {}, -.001)
    assert unplanned_capture_contact(['arm_shell', 'capture_x_piece_1'],
                                     ['bottom_link3', 'capture_target'], roots, {}, -.001)
    assert unplanned_capture_contact(['jaw', 'target_shell'],
                                     ['jaw_0', 'capture_target'], roots, {}, -.001)
    assert unplanned_capture_contact(['a', 'b'], ['bottom_link1', 'bottom_link4'],
                                     ['cubesat_bus']*2, {}, -.001)
    assert not unplanned_capture_contact(['a', 'b'], ['bottom_link1', 'bottom_link2'],
                                         ['cubesat_bus']*2, {'bottom_link2': 'bottom_link1'}, -.001)


def test_workspace_render_profile_keeps_default_earth_preflight(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from space_arm_platform import collection_runtime as runtime
    calls = []
    monkeypatch.setattr(runtime, 'ensure_environment_level', lambda *args: calls.append('earth'))
    monkeypatch.setattr(runtime, 'unreal_editor', lambda _: tmp_path/'UnrealEditor')
    args = SimpleNamespace(adapter=tmp_path, unreal=tmp_path, render_port=25002,
                           capture_port=25001, offline=True)
    assert runtime.ENVIRONMENT_MAP in runtime.renderer_command(args, tmp_path)
    assert calls == ['earth']
    args.render_environment = 'capture_workspace'
    assert '/Engine/Maps/Entry' in runtime.renderer_command(args, tmp_path)
    assert calls == ['earth']


def test_tested_pool_flows_through_original_batch_without_resampling(tmp_path):
    from simulation.scripted_tasks.capture_platform import platform_contract
    state = asdict(CaptureInitialState(gap_m=.12, bearing_x_deg=3., tilt_y_deg=-4.))
    report = tmp_path/'summary.json'
    report.write_text(json.dumps(dict(result='success', preflight={'accepted': True}, initial_state=state,
                                     capture_platform_contract=platform_contract())))
    entry = dict(id='capture-000', initial_state=state, physical_report=str(report),
                 report_sha256=hashlib.sha256(report.read_bytes()).hexdigest(), process_returncode=0)
    poolfile = tmp_path/'pool.json'
    model = tmp_path/'model.xml'
    model.write_text('<mujoco><worldbody><body name="capture_target">'
                     '<body name="task_box_base"/></body></worldbody></mujoco>')
    document = dict(schema='capture-initial-pool/1', layouts=[entry], model=str(model), catalog='catalog.json')
    poolfile.write_text(json.dumps(document))
    pool = CaptureInitialPool(poolfile)
    cfg = SequenceConfig(capture_initial_pool=str(poolfile), tasks=[{'name': 'capture_target_grasp'}], seed=7)
    group = make_plan(cfg, pool)['groups'][0]
    source = source_scene(7)
    layout = prepare_layout(source, group, pool)
    scene = prepare_task_scene(source, cfg, group, group['jobs'][0], layout)
    assert scene['capture_initial_state'] == state
    assert scene['collection_config']['capture_initial_selection']['report_sha256'] == entry['report_sha256']
    entry['process_returncode'] = -11
    poolfile.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='normally exited'):
        CaptureInitialPool(poolfile)
    with pytest.raises(ValueError):
        SequenceConfig(layout_pool='workbench.json', capture_initial_pool=str(poolfile),
                       tasks=[{'name': 'capture_target_grasp'}])


def test_capture_collection_rejects_workbench_on_wrong_spacecraft(tmp_path):
    from collection.capture_initial_pool import validate_capture_payload_model
    model = tmp_path/'model.xml'
    model.write_text('<mujoco><worldbody><body name="cubesat_bus">'
                     '<body name="task_box_base"/></body>'
                     '<body name="capture_target"/></worldbody></mujoco>')
    with pytest.raises(ValueError, match='target-owned'):
        validate_capture_payload_model(model)
    model.write_text('<mujoco><worldbody><body name="cubesat_bus"/>'
                     '<body name="capture_target"><body name="task_box_base"/>'
                     '</body></worldbody></mujoco>')
    validate_capture_payload_model(model)


def test_original_live_writer_capture12_official_v3_roundtrip(tmp_path, monkeypatch):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from space_arm_platform.recorder import EpisodeRecorder
    from space_arm_platform.recording_config import VideoEncodingConfig
    from space_arm_platform.compact_storage import read_observations
    from test_lerobot_recording import packet
    # Emulate the Windows ANSI locale for our metadata on every test host.
    # The Chinese instruction must survive an explicit UTF-8 round trip.
    original_read, original_write = Path.read_text, Path.write_text
    def read_text(path, *args, **kwargs):
        if path.name == 'platform.json' and not args and kwargs.get('encoding') is None:
            kwargs['encoding'] = 'cp1252'
        return original_read(path, *args, **kwargs)
    def write_text(path, data, *args, **kwargs):
        if path.name == 'platform.json' and not args and kwargs.get('encoding') is None:
            kwargs['encoding'] = 'cp1252'
        return original_write(path, data, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read_text)
    monkeypatch.setattr(Path, 'write_text', write_text)
    cameras = ['teleop/camera/spacecraft_overview', 'teleop/camera/capture_stereo_left']
    recorder = EpisodeRecorder(tmp_path, storage_profile='compact',
        video_encoding=VideoEncodingConfig(streaming_encoding=True, encoder_threads=1))
    meta = recorder.start(EpisodeStart(fps=10, camera_ids=cameras))
    root = tmp_path/meta['episode_id']
    try:
        for i in range(1, 4):
            recorder.record_authoritative_capture(*packet(i, cameras[1]))
            data = capture_observation(i).model_dump()
            data['scripted_task'] = dict(
                task_version='capture-target-grasp/5-preferred-handoff-branch',
                stage='hold' if i==1 else 'pull',
                carry_plan=None if i==1 else dict(accepted=True,
                    goal_seed_kind='validated_positive_wrist', goal_seed_index=0))
            recorder.record_observation(SimulationObservation.model_validate(data), None)
            recorder.record_authoritative_capture(*packet(i, cameras[0]))
        result = recorder.stop(EpisodeStop(outcome='success'))
        assert result['dataset_status'] == 'complete', json.dumps(result, indent=2)
        dataset = LeRobotDataset(f'local/{root.name}', root=root/'lerobot', video_backend='pyav')
        assert len(dataset) == 3
        np.testing.assert_allclose(dataset[0]['observation.state'], capture_observation().joint_position_si)
        np.testing.assert_allclose(dataset[0]['action'], capture_observation().target_joint_position_si)
        assert dataset[0]['observation.images.teleop_camera_capture_stereo_left'].shape == (3, 32, 32)
        platform = json.loads((root/'lerobot/meta/platform.json').read_text(encoding='utf-8'))
        assert platform['request']['instruction'] == EpisodeStart().instruction
        assert platform['joint_units'] == ['rad']*6 + ['m', 'rad'] + ['m']*4
        assert not platform['privileged_target_truth_in_policy_features']
        rows = list(read_observations(root))
        assert rows[0]['gripper_type'] == 'capture26_four_jaw'
        np.testing.assert_allclose(rows[0]['joint_position_si'], capture_observation().joint_position_si)
        assert rows[0]['scripted_task']['carry_plan'] is None
        assert rows[1]['scripted_task']['carry_plan']['goal_seed_kind']=='validated_positive_wrist'
        assert rows[-1]['scripted_task']['task_version']=='capture-target-grasp/5-preferred-handoff-branch'
    finally:
        recorder.close()
