"""Startup ownership checks; fake rendering is never counted as GPU acceptance."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.skipif(importlib.util.find_spec('Basilisk') is None,
    reason='render adapter import requires the cloud Basilisk environment; this test starts no physics or GPU')


@pytest.mark.parametrize('fail_metadata', [False, True])
def test_moving_camera_metadata_failure_is_inside_owned_resource_cleanup(tmp_path, monkeypatch, fail_metadata):
    from simulation import benchmark_rendered_camera as base
    from simulation import benchmark_rendered_articulated_camera as runtime
    from space_arm_platform.benchmark_articulated_camera import capture26_mounted_cameras

    repository = Path(__file__).resolve().parents[1]
    (tmp_path/'model').mkdir()
    model = tmp_path/'model/team_flight.xml'
    model.write_bytes(b'<mujoco/>')
    environment = SimpleNamespace(episode_directory=tmp_path, episode_id='owned-camera-episode',
        repository=repository, _assembly=SimpleNamespace(ports=object()),
        _stepper=SimpleNamespace(model_xml_sha256=hashlib.sha256(model.read_bytes()).hexdigest()))
    owned, sensors = [], []

    class FakeOwnedResources:
        def __init__(self, output):
            self.closed = False
            owned.append(self)

        def close(self):
            self.closed = True

    def fake_render_start(sensor, warmup):
        sensors.append(sensor)

    monkeypatch.setattr(base, 'ManagedProcesses', FakeOwnedResources)
    monkeypatch.setattr(base.RenderedCameraSensor, '_start', fake_render_start)
    monkeypatch.setattr(runtime, 'NativeCameraMountReader', lambda *args, **kwargs: object())
    if fail_metadata:
        def fail(*args, **kwargs):
            raise OSError('simulated metadata disk write failure')
        monkeypatch.setattr(runtime, 'write_json', fail)
        with pytest.raises(OSError, match='metadata disk write failure'):
            runtime.ArticulatedRenderedCameraSensor(environment, capture26_mounted_cameras(repository))
    else:
        sensor = runtime.ArticulatedRenderedCameraSensor(environment, capture26_mounted_cameras(repository))
        data = json.loads((tmp_path/'rendered_sensor/moving_mount.json').read_text('utf-8'))
        assert data['model_xml_sha256'] == environment._stepper.model_xml_sha256
        assert data['episode_id'] == environment.episode_id and data['sampling_period_ns'] == 100_000_000
        assert not owned[0].closed
        sensor.close()
    assert len(owned) == len(sensors) == 1 and owned[0].closed
    assert sensors[0]._metadata.closed
