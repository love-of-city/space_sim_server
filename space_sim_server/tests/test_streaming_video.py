import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from space_arm_platform.streaming_video import StrictStreamingVideoEncoder


def queued_encoder():
    encoder = StrictStreamingVideoEncoder(fps=50, vcodec="h264", queue_maxsize=1)
    encoder._episode_active = True
    encoder._threads = {"camera": SimpleNamespace(is_alive=lambda: True)}
    encoder._frame_queues = {"camera": queue.Queue(maxsize=1)}
    encoder._result_queues = {"camera": queue.Queue()}
    encoder._accepted = {"camera": 0}
    return encoder


def test_full_video_queue_backpressures_instead_of_dropping():
    encoder = queued_encoder()
    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    encoder.feed_frame("camera", frame)
    done = threading.Event()

    def feed():
        encoder.feed_frame("camera", frame + 1)
        done.set()

    worker = threading.Thread(target=feed)
    worker.start()
    try:
        assert not done.wait(0.15)  # upstream would already have dropped it
        np.testing.assert_array_equal(encoder._frame_queues["camera"].get(), frame)
        assert done.wait(2)
        np.testing.assert_array_equal(encoder._frame_queues["camera"].get(), frame + 1)
        assert encoder._accepted["camera"] == 2
    finally:
        worker.join(timeout=2)


def test_encoder_failure_is_propagated_with_original_reason():
    encoder = queued_encoder()
    encoder._threads["camera"] = SimpleNamespace(is_alive=lambda: False)
    encoder._result_queues["camera"].put(("error", "device unavailable"))
    with pytest.raises(RuntimeError, match="device unavailable"):
        encoder.feed_frame("camera", np.zeros((32, 32, 3), dtype=np.uint8))
    assert encoder._accepted["camera"] == 0


def test_stalled_encoder_has_bounded_wait():
    encoder = queued_encoder()
    encoder.stall_timeout_s = 0.01
    encoder._frame_queues["camera"].put(None)
    started = time.monotonic()
    with pytest.raises(TimeoutError, match="no frames were dropped"):
        encoder.feed_frame("camera", np.zeros((32, 32, 3), dtype=np.uint8))
    assert time.monotonic() - started < 2
    assert encoder._accepted["camera"] == 0


def test_incomplete_video_cannot_be_marked_complete(tmp_path):
    encoder = StrictStreamingVideoEncoder(fps=50, vcodec="h264", encoder_threads=1)
    encoder.start_episode(["camera"], tmp_path)
    for _ in range(3):
        encoder.feed_frame("camera", np.zeros((32, 32, 3), dtype=np.uint8))
    encoder._accepted["camera"] += 1
    with pytest.raises(RuntimeError, match="3 frames for 4 states"):
        encoder.finish_episode()
    assert not encoder._episode_active


def test_streaming_statistics_and_video_timestamps(tmp_path):
    import av

    encoder = StrictStreamingVideoEncoder(fps=50, vcodec="h264", encoder_threads=1)
    encoder.start_episode(["camera"], tmp_path)
    images = [np.full((32, 32, 3), value, dtype=np.uint8) for value in (0, 128, 255)]
    for frame in images:
        encoder.feed_frame("camera", frame)
    path, stats = encoder.finish_episode()["camera"]
    values = np.stack(images).astype(np.float64)
    np.testing.assert_allclose(stats["mean"], values.mean(axis=(0, 1, 2)))
    np.testing.assert_allclose(stats["std"], values.std(axis=(0, 1, 2)))
    assert stats["count"].tolist() == [3]
    with av.open(str(path)) as video:
        frames = list(video.decode(video=0))
    np.testing.assert_allclose([float(f.time) for f in frames], [0, .02, .04])
