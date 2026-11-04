"""Bounded best-effort vision delivery, isolated from authoritative recording.

Vision ACKs acknowledge transport receipt, including a deliberately dropped
sample under congestion. Dataset ACK/admission semantics are unchanged.
"""
from __future__ import annotations

import queue
import threading


class VisionCaptureDispatch:
    def __init__(self, consumer, capacity=8):
        self.consumer = consumer
        self.queue = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.thread = None
        self.received = self.dropped = self.processed = 0
        self.last_error = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop.clear()
        self.thread = threading.Thread(target=self._run, name="vision-capture-dispatch", daemon=True)
        self.thread.start()

    def offer(self, metadata, products):
        self.received += 1
        if self.consumer is None or self.stop.is_set():
            self.dropped += 1
            return
        # One transport producer. Drop stale queued work; never wait on an
        # estimator/config load/disk write from the shared socket reader.
        try:
            self.queue.put_nowait((metadata, products))
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
                self.dropped += 1
            except queue.Empty:
                pass
            self.queue.put_nowait((metadata, products))

    def _run(self):
        while not self.stop.is_set():
            try:
                metadata, products = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.consumer(metadata, products)
                self.processed += 1
                self.last_error = None
            except Exception as error:
                self.last_error = str(error)
            finally:
                self.queue.task_done()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
        while True:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
                self.dropped += 1
            except queue.Empty:
                break

    def status(self):
        return dict(received=self.received, processed=self.processed, dropped=self.dropped,
                    pending=self.queue.qsize(), last_error=self.last_error)
