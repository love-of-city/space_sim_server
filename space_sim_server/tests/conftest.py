"""Keep unit tests fast; process-isolation tests explicitly bypass this fixture."""
import pytest

@pytest.fixture(autouse=True)
def recorder_test_lifecycle(monkeypatch, request):
    if request.node.get_closest_marker("process_writer"):
        yield
        return
    from space_arm_platform.recorder import EpisodeRecorder
    from space_arm_platform.lerobot_capture import LiveLeRobotWriter
    original = EpisodeRecorder.__init__
    recorders = []
    def initialize(self, *args, **kwargs):
        kwargs.setdefault("writer_factory", LiveLeRobotWriter)
        original(self, *args, **kwargs)
        recorders.append(self)
    monkeypatch.setattr(EpisodeRecorder, "__init__", initialize)
    yield
    for recorder in recorders:
        recorder._drain_timeout_s = .02
        recorder.close()

def pytest_configure(config):
    config.addinivalue_line("markers", "process_writer: exercise real spawned LeRobot worker")
