"""Reliable dataset transport must not reuse preview's latest-wins queue."""
import json
import queue
import pytest
from bsk_render_adapter.protocol import RenderPublisher


def unpack(packet):
    return json.loads(packet[4:].decode())


def test_dataset_frames_preserved_in_order(monkeypatch):
    publisher = RenderPublisher(reliable_frames=True)
    monkeypatch.setattr(publisher, 'start', lambda: None)
    for i in range(20):
        publisher.publish_frame({'frame_id': str(i)})
    assert [unpack(publisher._latest_frame.get_nowait())['frame_id'] for _ in range(20)] == [str(i) for i in range(20)]
    assert publisher.stats.frames_dropped == 0


def test_preview_keeps_latest(monkeypatch):
    publisher = RenderPublisher()
    monkeypatch.setattr(publisher, 'start', lambda: None)
    for i in range(4):
        publisher.publish_frame({'frame_id': str(i)})
    assert unpack(publisher._latest_frame.get_nowait())['frame_id'] == '3'
    assert publisher.stats.frames_dropped == 3


def test_dataset_full_queue_fails_explicitly(monkeypatch):
    publisher = RenderPublisher(reliable_frames=True)
    monkeypatch.setattr(publisher, 'start', lambda: None)
    def full(*args, **kwargs):
        raise queue.Full
    monkeypatch.setattr(publisher._latest_frame, 'put', full)
    with pytest.raises(RuntimeError, match='refusing to drop'):
        publisher.publish_frame({'frame_id': '1'})
    assert publisher.stats.frames_dropped == 0
