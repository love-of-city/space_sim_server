"""Structured wall-clock stage timing for scene startup and reset diagnostics."""

import json
import time
from contextlib import contextmanager


@contextmanager
def runtime_stage(stage: str):
    started = time.monotonic()
    print(json.dumps({"type": "runtime_stage", "stage": stage, "status": "started",
                      "wall_time_ns": str(time.time_ns())}), flush=True)
    try:
        yield
    except BaseException:
        print(json.dumps({"type": "runtime_stage", "stage": stage, "status": "failed",
                          "elapsed_s": time.monotonic() - started}), flush=True)
        raise
    else:
        print(json.dumps({"type": "runtime_stage", "stage": stage, "status": "completed",
                          "elapsed_s": time.monotonic() - started}), flush=True)
