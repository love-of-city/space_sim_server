"""Real Windows sharing violations must not discard a collector's last report."""

import json
import errno
import os
from pathlib import Path
import threading
import time

import pytest

from space_arm_platform.collection_jobs import write_json


@pytest.mark.skipif(os.name != "nt", reason="Windows reader handles deny atomic replacement")
def test_progress_write_survives_short_reader_lock(tmp_path):
    path = tmp_path / "summary.json"
    write_json(path, {"status": "running"})
    reader = path.open("r", encoding="utf-8")

    def release():
        time.sleep(.15)
        reader.close()

    thread = threading.Thread(target=release)
    thread.start()
    try:
        write_json(path, {"status": "complete", "accepted_episodes": ["episode-one"]})
    finally:
        thread.join(timeout=5)
        reader.close()
    assert json.loads(path.read_text("utf-8"))["status"] == "complete"


@pytest.mark.skipif(os.name != "nt", reason="Windows reader handles deny atomic replacement")
def test_persistent_reader_lock_is_bounded_and_preserves_previous_report(tmp_path):
    path = tmp_path / "summary.json"
    write_json(path, {"status": "running"})
    with path.open("r", encoding="utf-8") as reader:
        started = time.monotonic()
        with pytest.raises(PermissionError):
            write_json(path, {"status": "complete"})
        assert time.monotonic() - started < 5
        assert json.load(reader)["status"] == "running"
    assert json.loads(path.read_text("utf-8"))["status"] == "running"


def test_disk_full_does_not_retry_or_replace_old_progress(tmp_path, monkeypatch):
    path = tmp_path / "summary.json"
    write_json(path, {"status": "running"})
    calls = []

    def disk_full(source, target):
        calls.append(target)
        raise OSError(errno.ENOSPC, "disk full")

    monkeypatch.setattr(Path, "replace", disk_full)
    with pytest.raises(OSError, match="disk full"):
        write_json(path, {"status": "complete"})
    assert calls == [path]
    assert json.loads(path.read_text("utf-8"))["status"] == "running"
