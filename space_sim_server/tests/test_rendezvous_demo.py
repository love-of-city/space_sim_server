import gzip
import json
import struct
import time

from space_arm_platform.rendezvous_demo import RendezvousDemo


class Socket:
    def __init__(self):
        self.messages = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def close(self):
        pass

    def sendall(self, data):
        assert struct.unpack('!I', data[:4])[0] == len(data) - 4
        self.messages.append(json.loads(data[4:]))


def package(tmp_path):
    samples = []
    for i in range(3):
        bodies = [{'object_id': f'rendezvous/{name}', 'position_m': [i, n, 0],
                   'velocity_mps': [1, 0, 0], 'orientation_wxyz': [1, 0, 0, 0],
                   'angular_velocity_B_radps': [0, 0, 0]}
                  for n, name in enumerate(('target', 'chaser'))]
        samples.append({'elapsed_s': i*.02, 'phase': 'Coupled' if i == 2 else 'Final approach',
                        'frame': {'sim_time_ns': str(i*1_000_000_000), 'objects': bodies}})
    data = {'schema': 'rendezvous-demo/1', 'source_dir': str(tmp_path),
            'source_report': {'basic_nominal_passed': True, 'capture': {'latch_time_s': 2},
                              'outcome': 'Docked', 'final_time_s': 2, 'final_body_origin_distance_m': 0.1},
            'hello': {'type':'hello'}, 'manifest': {'type':'scene_manifest'}, 'samples': samples}
    path = tmp_path/'demo.json.gz'
    with gzip.open(path, 'wt') as stream:
        json.dump(data, stream)
    return path


def test_playback_channels_share_time_and_latch(monkeypatch, tmp_path):
    render, state = Socket(), Socket()
    monkeypatch.setattr('socket.create_connection', lambda address, **_: render if address[1] == 5558 else state)
    demo = RendezvousDemo(package(tmp_path), 8768)
    demo.start()
    demo._thread.join(timeout=2)
    assert demo.completed and not demo.error
    assert len(render.messages) == 5 and len(state.messages) == 3
    for video, dashboard in zip(render.messages[2:], state.messages, strict=True):
        assert int(video['sim_time_ns']) / 1e9 == dashboard['sim_time_s']
        assert video['session_id'] == dashboard['metadata']['run_id']
        assert video['objects'][0]['position_m'] == dashboard['bodies'][0]['position_m']
    assert state.messages[0]['metadata']['capture_time_s'] is None
    assert state.messages[-1]['metadata']['capture_time_s'] == 2
    assert demo.finished_at - demo.started_at >= .04


def test_stop_then_repeat_gets_new_run(monkeypatch, tmp_path):
    monkeypatch.setattr('socket.create_connection', lambda *_, **__: Socket())
    demo = RendezvousDemo(package(tmp_path), 8768)
    demo.start()
    first_run = demo.run_id
    demo.stop()
    assert not demo.running and not demo.completed
    demo.start()
    demo._thread.join(timeout=2)
    assert demo.run_id != first_run and demo.completed


def test_platform_demo_reuses_renderer_after_a_flight(monkeypatch, tmp_path):
    from space_arm_platform.rendezvous_runtime import RendezvousRuntime
    folder = tmp_path / 'output/spacesim_demo15'
    folder.mkdir(parents=True)
    package(folder)
    monkeypatch.setattr('socket.create_connection', lambda *_, **__: Socket())
    runtime = RendezvousRuntime(tmp_path, None, 8768, tmp_path / 'logs', launch_config=object())
    renderer_calls = []
    monkeypatch.setattr(runtime, '_ensure_renderer', lambda: renderer_calls.append('ready'))
    monkeypatch.setattr(runtime, 'release_renderer', lambda: renderer_calls.append('released'))
    assert runtime.start(mode='demo')['mode'] == 'demo'
    runtime.stop()
    assert renderer_calls == ['ready']
