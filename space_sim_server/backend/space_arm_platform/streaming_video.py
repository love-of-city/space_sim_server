"""Lossless-in-time adapter for the pinned LeRobot streaming encoder.

LeRobot's default queue drops images after 100 ms and can accept an unfinished
video at shutdown. Authoritative datasets must instead backpressure or fail.
Uses LeRobot's streaming lifecycle and statistics schema with explicit PyAV
timestamps, GPU keyframe control and float statistics (uint8 squares overflow).
"""
from __future__ import annotations

import queue
import tempfile
import threading
import time
from fractions import Fraction
from pathlib import Path

import numpy as np
from lerobot.datasets.compute_stats import RunningQuantileStats, auto_downsample_height_width
from lerobot.datasets.video_utils import StreamingVideoEncoder


class StrictStreamingVideoEncoder(StreamingVideoEncoder):
    stall_timeout_s = 60.0

    def start_episode(self, video_keys, temp_dir):
        if self._episode_active:
            self.cancel_episode()
        self._dropped_frames.clear()
        self._accepted = dict.fromkeys(video_keys, 0)
        for key in video_keys:
            self._frame_queues[key] = queue.Queue(maxsize=self.queue_maxsize)
            self._result_queues[key] = queue.Queue(maxsize=1)
            self._stop_events[key] = threading.Event()
            self._video_paths[key] = Path(tempfile.mkdtemp(dir=temp_dir)) / "capture.mp4"
            self._threads[key] = threading.Thread(target=self._encode_camera, args=(key,), daemon=True)
            self._threads[key].start()
        self._episode_active = True

    def _encode_camera(self, key):
        import av

        container = stream = None
        stats = RunningQuantileStats()
        count = 0
        try:
            while not self._stop_events[key].is_set():
                try:
                    frame = self._frame_queues[key].get(timeout=0.1)
                except queue.Empty:
                    continue
                if frame is None:
                    break
                if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[-1] != 3:
                    raise ValueError("Authoritative RGB must be HWC uint8")
                if container is None:
                    options = {"g": str(self.g), "threads": str(self.encoder_threads or 1)}
                    if self.vcodec == "h264_nvenc":
                        # NVENC's default B-frame count is incompatible with
                        # the short GOP used for random training frame reads.
                        options.update(rc="constqp", qp=str(self.crf), bf="0")
                    else:
                        options["crf"] = str(self.crf)
                    container = av.open(str(self._video_paths[key]), "w")
                    stream = container.add_stream(self.vcodec, self.fps, options=options)
                    stream.width, stream.height = frame.shape[1], frame.shape[0]
                    stream.pix_fmt = self.pix_fmt
                    stream.time_base = Fraction(1, self.fps)
                video_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
                video_frame.pts = count
                video_frame.time_base = Fraction(1, self.fps)
                for packet in stream.encode(video_frame):
                    container.mux(packet)
                pixels = auto_downsample_height_width(frame.transpose(2, 0, 1))
                stats.update(pixels.transpose(1, 2, 0).reshape(-1, 3).astype(np.float64))
                count += 1
            if stream is not None:
                for packet in stream.encode():
                    container.mux(packet)
            if container is not None:
                container.close()
                container = None
            result = stats.get_statistics() if count else None
            if result is not None:
                # Official image statistics count images, not sampled pixels.
                result["count"] = np.array([count])
            self._result_queues[key].put(("ok", result))
        except Exception as error:
            self._result_queues[key].put(("error", f"{type(error).__name__}: {error}"))
        finally:
            if container is not None:
                container.close()

    def _put(self, video_key, item):
        deadline = time.monotonic() + self.stall_timeout_s
        while True:
            if not self._threads[video_key].is_alive():
                try:
                    _, detail = self._result_queues[video_key].get_nowait()
                except queue.Empty:
                    detail = "worker exited without a result"
                raise RuntimeError(f"Video encoder {video_key} failed: {detail}")
            try:
                self._frame_queues[video_key].put(item, timeout=0.1)
                return
            except queue.Full:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Video encoder {video_key} stalled; no frames were dropped")

    def feed_frame(self, video_key, image):
        if not self._episode_active:
            raise RuntimeError("No active video episode")
        self._put(video_key, image.copy())
        self._accepted[video_key] += 1

    def finish_episode(self):
        import av

        if not self._episode_active:
            raise RuntimeError("No active video episode")
        results = {}
        try:
            for key in self._frame_queues:
                self._put(key, None)
            deadline = time.monotonic() + self.stall_timeout_s
            for key, thread in self._threads.items():
                thread.join(timeout=max(0, deadline - time.monotonic()))
                if thread.is_alive():
                    raise TimeoutError(f"Video encoder {key} did not finish")
                try:
                    status, stats = self._result_queues[key].get_nowait()
                except queue.Empty as error:
                    raise RuntimeError(f"Video encoder {key} returned no result") from error
                if status != "ok":
                    raise RuntimeError(f"Video encoder {key} failed: {stats}")
                path = self._video_paths[key]
                # Count timestamped packets without a costly full decode. The
                # collection acceptance check separately decodes all frames.
                with av.open(str(path)) as container:
                    count = sum(packet.pts is not None for packet in container.demux(video=0))
                if count != self._accepted[key]:
                    raise RuntimeError(f"Video {key}: {count} frames for {self._accepted[key]} states")
                results[key] = (path, stats)
        except BaseException:
            self.cancel_episode()
            raise
        self._cleanup()
        self._episode_active = False
        return results
