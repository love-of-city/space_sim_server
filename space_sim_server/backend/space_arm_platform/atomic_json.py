"""Publish a single writer's JSON snapshot while readers keep seeing whole files."""

import json
import os
from pathlib import Path
import time


def write_json(path: Path, value, *, allow_nan=True, trailing_newline=False) -> None:
    temporary = path.with_suffix(".tmp")
    text = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=allow_nan)
    temporary.write_text(text + ("\n" if trailing_newline else ""), encoding="utf-8")
    deadline = time.monotonic() + 1.0
    while True:
        try:
            temporary.replace(path)
            return
        except PermissionError as error:
            # Windows readers/scanners can briefly deny replacement even though
            # the directory is writable. Keep the old file until retry succeeds.
            if (os.name != "nt" or getattr(error, "winerror", None) not in {5, 32, 33}
                    or time.monotonic() >= deadline):
                raise
            time.sleep(.02)
