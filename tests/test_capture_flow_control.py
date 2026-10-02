"""Completion credits, missing-image failure isolation and responsive STOP."""
import asyncio
import threading
import time

from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub
from simulation.control_client import SimulationControlClient


def observation(i, episode='', request=''):
    return SimulationObservation(protocol='space-arm-control/1', type='observation', simulation_id='test',
        render_session_id='flow-session', step_id=str(i + 1), render_frame_id=str(i),
        sim_time_ns=str((i * 1_000_000_000 + 15) // 30), wall_time_ns=str(time.time_ns()),
        capture_episode_id=episode, capture_request_id=request, applied_action_sequence='0',
        joint_position_rad=[0.] * 8, joint_velocity_rad_s=[0.] * 8, target_joint_position_rad=[0.] * 8)


def capture(i, episode, camera):
    return dict(protocol='bsk-capture/1', type='camera_frame', session_id='flow-session', camera_id=camera,
        capture_sequence=str(i), source_frame_id=str(i), sim_time_ns=observation(i).sim_time_ns,
        capture_episode_id=episode, stream_kind='authoritative', state_kind='authoritative',
        products=[dict(name='rgb', file_name='rgb.jpg')])


class Writer:
    def __init__(self, root, request):
        self.fps, self.cameras, self.frames = request.fps, tuple(request.camera_ids), 0
        self.release = threading.Event()
        self.release.set()
    def append(self, obs, caps):
        assert self.release.wait(3)
        self.frames += 1
    def finish(self, *args): pass
    def close_failed(self): pass


def test_ack_waits_for_both_images_and_writer_not_just_pairing(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=2)
    ep = r.start(EpisodeStart(camera_ids=['a', 'b']), capture_on_demand=True)['episode_id']
    done = threading.Event()
    obs = observation(0, ep)
    try:
        r._dataset.release.clear()
        r.record_observation(obs, None)
        t = threading.Thread(target=lambda: (r.wait_for_capture_pair(obs), done.set()))
        t.start()
        r.record_authoritative_capture(capture(0, ep, 'a'), {'rgb': b'a'})
        assert not done.wait(.03)
        r.record_authoritative_capture(capture(0, ep, 'b'), {'rgb': b'b'})
        assert not done.wait(.03), 'pairing alone must not credit a stalled writer'
        r._dataset.release.set()
        assert done.wait(2)
        t.join(2)
        assert r.sync_status()['last_written_frame_id'] == '0'
        assert r.stop(EpisodeStop())['dataset_frame_count'] == 1
    finally:
        if r._dataset: r._dataset.release.set()
        r.close()


def test_stop_unblocks_credit_waiter_but_preserves_accepted_sample(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=1)
    ep = r.start(EpisodeStart(camera_ids=['a']), capture_on_demand=True)['episode_id']
    done = threading.Event()
    try:
        obs = observation(0, ep)
        r.record_observation(obs, None)
        t = threading.Thread(target=lambda: (r.wait_for_capture_pair(obs), done.set()))
        t.start()
        assert not done.wait(.03)
        r.freeze_observations()
        assert done.wait(.5)
        r.record_authoritative_capture(capture(0, ep, 'a'), {'rgb': b'a'})
        t.join(1)
        closed = r.stop(EpisodeStop())
        assert closed['dataset_status'] == 'complete' and closed['dataset_frame_count'] == 1
    finally: r.close()


def test_missing_image_fails_recording_without_raising_to_simulation(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=.03)
    ep = r.start(EpisodeStart(camera_ids=['a']), capture_on_demand=True)['episode_id']
    try:
        obs = observation(0, ep)
        r.record_observation(obs, None)
        r.wait_for_capture_pair(obs)
        assert 'missing_cameras' in r.sync_status()['dataset_error']
        assert r.stop(EpisodeStop())['dataset_status'] == 'failed'
    finally: r.close()


def test_explicit_ue_error_fails_only_matching_episode(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer)
    ep = r.start(EpisodeStart(camera_ids=['a']), capture_on_demand=True)['episode_id']
    try:
        error = {**capture(0, 'previous-episode', 'a'), 'type': 'capture_error', 'error': 'readback failed'}
        r.record_authoritative_capture(error, {})
        assert r.sync_status()['dataset_error'] is None
        error['capture_episode_id'] = ep
        r.record_authoritative_capture(error, {})
        assert 'readback failed' in r.sync_status()['dataset_error']
        assert r.stop(EpisodeStop())['dataset_status'] == 'failed'
    finally: r.close()


def test_stalled_writer_reports_writer_not_missing_images(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=.05)
    ep = r.start(EpisodeStart(camera_ids=['a']), capture_on_demand=True)['episode_id']
    try:
        r._dataset.release.clear()
        obs = observation(0, ep)
        r.record_observation(obs, None)
        r.record_authoritative_capture(capture(0, ep, 'a'), {'rgb': b'a'})
        r.wait_for_capture_pair(obs)
        assert 'stage=dataset_writer, missing_cameras=[]' in r.sync_status()['dataset_error']
        r._dataset.release.set()
        assert r.stop(EpisodeStop())['dataset_status'] == 'failed'
        assert r.sync_status()['oldest_unwritten_sample_age_s'] == 0
    finally:
        if r._dataset: r._dataset.release.set()
        r.close()


def test_capture_error_watchdog_sends_off_without_stopping_simulator(tmp_path, monkeypatch):
    import json
    from fastapi.testclient import TestClient
    from space_arm_platform.app import PlatformConfig, create_app

    project = tmp_path / 'project'
    (project / 'frontend').mkdir(parents=True)
    app = create_app(PlatformConfig(project_root=project, data_root=tmp_path / 'data',
                                    simulation_port=0, capture_port=0))
    r, hub = app.state.recorder, app.state.hub
    r._writer_factory = Writer
    off_sent = threading.Event()
    async def set_capture(episode, **kwargs):
        assert episode == ''
        off_sent.set()
    monkeypatch.setattr(hub, 'set_capture_episode', set_capture)
    with TestClient(app):
        ep = r.start(EpisodeStart(camera_ids=['a']), capture_on_demand=True)['episode_id']
        app.state.capture_lifecycle._gated_episode = ep
        r.record_authoritative_capture({**capture(0, ep, 'a'),
            'type': 'capture_error', 'error': 'injected GPU readback failure'}, {})
        assert off_sent.wait(2)
        deadline = time.monotonic() + 2
        while r.episode_id and time.monotonic() < deadline: time.sleep(.01)
        assert r.episode_id is None
        metadata = json.loads((tmp_path / 'data' / ep / 'metadata.json').read_text(encoding='utf-8'))
        assert metadata['dataset_status'] == 'failed'
        assert 'injected GPU readback failure' in metadata['dataset_error']
        assert hub._server.is_serving(), 'recording failure must not tear down control'


def test_delayed_failure_shutdown_cannot_stop_next_episode(tmp_path):
    from space_arm_platform.capture_lifecycle import CaptureLifecycle
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=Writer)
        lifecycle = CaptureLifecycle(SimulationHub(), r)
        try:
            old = r.start(EpisodeStart(camera_ids=[]))['episode_id']
            r.fail('old episode failed')
            r.stop(EpisodeStop())
            new = r.start(EpisodeStart(camera_ids=[]))['episode_id']
            assert await lifecycle.stop(EpisodeStop(), failed_episode_id=old) is None
            assert r.episode_id == new
            # Even an accidentally stale error snapshot with a new ID cannot
            # terminate a healthy recorder.
            assert await lifecycle.stop(EpisodeStop(), failed_episode_id=new) is None
            assert r.episode_id == new
        finally: r.close()
    asyncio.run(run())


def test_real_control_connection_is_responsive_while_capture_ack_waits(tmp_path):
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=2)
        ep = r.start(EpisodeStart(camera_ids=['a', 'b']), capture_on_demand=True)['episode_id']
        hub = SimulationHub()
        accepted = asyncio.Event()
        async def record(obs):
            await asyncio.to_thread(r.record_observation, obs, None)
            accepted.set()
            if hub.capture_pair_ack_supported:
                await asyncio.to_thread(r.wait_for_capture_pair, obs)
        hub.on_observation = record
        await hub.start('127.0.0.1', 0)
        client = SimulationControlClient('127.0.0.1', hub.bound_port, 'test')
        client.start()
        try:
            await asyncio.to_thread(client.wait_until_ready)
            start = asyncio.create_task(hub.set_capture_episode(ep))
            for _ in range(100):
                if client.capture_state()['capture_episode_id'] == ep: break
                await asyncio.sleep(.01)
            state = client.capture_state()
            for i in range(15):
                assert client.has_observation_capacity()
                client.send_observation(observation(i, ep, state['capture_request_id']).model_dump())
            assert not client.has_observation_capacity()
            await start
            # START is confirmed at the render boundary, before the recorder
            # callback necessarily runs. Freeze only after this test's one
            # accepted sample exists; an earlier STOP legitimately yields empty.
            await asyncio.wait_for(accepted.wait(), 2)
            assert hub.capture_pair_ack_supported
            stop = asyncio.create_task(hub.set_capture_episode(''))
            for _ in range(100):
                if not client.capture_state()['capture_episode_id']: break
                await asyncio.sleep(.01)
            assert not client.capture_state()['capture_episode_id'], 'STOP was blocked by the sample ACK'
            r.freeze_observations()
            for camera in ('a', 'b'):
                r.record_authoritative_capture(capture(0, ep, camera), {'rgb': b'image'})
            client.send_observation(observation(15, '', client.capture_state()['capture_request_id']).model_dump())
            await asyncio.wait_for(stop, 2)
            closed = await asyncio.to_thread(r.stop, EpisodeStop())
            assert closed['dataset_status'] == 'complete', closed
        finally:
            client.close()
            await hub.close()
            r.close()
    asyncio.run(run())
